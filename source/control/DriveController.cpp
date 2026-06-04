// DriveController.cpp — S/T/D/G drive state machines, S-mode watchdog,
// streaming encoder counter, and odometry delta tracking.
//
// Transplanted from CommandProcessor.cpp (Sprint 007, Ticket 003).
// All speeds in mm/s; distances in mm.
//
// Sprint 010, Ticket 007: All wheel setpoints routed through
// BodyKinematics::saturate() before reaching MotorController, preserving
// arc curvature when commanded speeds exceed vWheelMax - steerHeadroom.

#include "DriveController.h"
#include "MotorController.h"
#include "Odometry.h"
#include "OtosSensor.h"
#include "BodyKinematics.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

DriveController::DriveController(MotorController& mc, Odometry& odo, const RobotConfig& cfg,
                                 OtosSensor* otos)
    : _mc(mc)
    , _odo(odo)
    , _cfg(cfg)
    , _otos(otos)
    , _hwState(nullptr)
    , _mode(DriveMode::IDLE)
    , _driveFn(nullptr)
    , _driveCtx(nullptr)
    , _corrId{}
    , _lastSMs(0)
    , _tgtL(0.0f)
    , _tgtR(0.0f)
    , _tEndMs(0)
    , _dEncStartL(0)
    , _dEncStartR(0)
    , _dTargetMm(0)
    , _dTimeoutMs(0)
    , _gPhase(GPhase::IDLE)
    , _gTargetXWorld(0.0f)
    , _gTargetYWorld(0.0f)
    , _gSpeed(0.0f)
    , _vRamped(0.0f)
    , _lastTickMs(0)
    , _currentTimeMs(0)
    , _lastOtosMs(0)
    , _evtHead(0)
    , _evtTail(0)
{
    // Zero-initialise the ring buffer entries.
    for (int i = 0; i < kEvtQueueCap; ++i) {
        _evtQueue[i].msg[0] = '\0';
        _evtQueue[i].fn  = nullptr;
        _evtQueue[i].ctx = nullptr;
    }
}

// ---------------------------------------------------------------------------
// Internal: apply curvature-preserving saturation to a wheel-speed pair.
// Routes through BodyKinematics::saturate() using config ceiling.
// ---------------------------------------------------------------------------

static void applySaturation(float vL, float vR,
                             const RobotConfig& cfg,
                             float& vL_out, float& vR_out)
{
    BodyKinematics::saturate(vL, vR, cfg.vWheelMax, cfg.steerHeadroom, vL_out, vR_out);
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

void DriveController::beginStream(float leftMms, float rightMms, uint32_t now_ms,
                                   ReplyFn fn, void* ctx)
{
    float sL, sR;
    applySaturation(leftMms, rightMms, _cfg, sL, sR);
    _mc.startDrive(sL, sR);
    _mc.setTarget(sL, sR);
    _tgtL    = sL;
    _tgtR    = sR;
    _mode    = DriveMode::STREAMING;
    _lastSMs = now_ms;
    _driveFn  = fn;
    _driveCtx = ctx;
}

void DriveController::beginVelocity(float v_mms, float omega_rads, uint32_t now_ms,
                                     ReplyFn fn, void* ctx, const char* corr_id)
{
    // Convert body-twist (v, ω) → individual wheel speeds, then saturate.
    float vL, vR;
    BodyKinematics::inverse(v_mms, omega_rads, _cfg.trackwidthMm, vL, vR);
    float sL, sR;
    BodyKinematics::saturate(vL, vR, _cfg.vWheelMax, _cfg.steerHeadroom, sL, sR);
    // Delegate to the existing STREAMING path (keeps watchdog logic in one place).
    beginStream(sL, sR, now_ms, fn, ctx);
    // Store corr_id so the watchdog EVT safety_stop echoes it (beginStream clears nothing).
    if (corr_id && corr_id[0] != '\0') {
        strncpy(_corrId, corr_id, sizeof(_corrId) - 1);
        _corrId[sizeof(_corrId) - 1] = '\0';
    } else {
        _corrId[0] = '\0';
    }
}

void DriveController::beginTimed(float leftMms, float rightMms,
                                  uint32_t durationMs, uint32_t now_ms,
                                  ReplyFn fn, void* ctx, const char* corr_id)
{
    float sL, sR;
    applySaturation(leftMms, rightMms, _cfg, sL, sR);
    _mc.startDriveClean(sL, sR);
    _mc.setTarget(sL, sR);
    _tgtL     = sL;
    _tgtR     = sR;
    _tEndMs   = _lastTickMs + durationMs;
    _mode     = DriveMode::TIMED;
    _driveFn  = fn;
    _driveCtx = ctx;
    if (corr_id && corr_id[0] != '\0') {
        strncpy(_corrId, corr_id, sizeof(_corrId) - 1);
        _corrId[sizeof(_corrId) - 1] = '\0';
    } else {
        _corrId[0] = '\0';
    }
    (void)now_ms;
}

void DriveController::beginDistance(float leftMms, float rightMms,
                                     int32_t targetMm, uint32_t now_ms,
                                     ReplyFn fn, void* ctx, const char* corr_id)
{
    float sL, sR;
    applySaturation(leftMms, rightMms, _cfg, sL, sR);
    _mc.startDriveClean(sL, sR);
    _mc.setTarget(sL, sR);
    _tgtL = sL;
    _tgtR = sR;
    _mc.resetEncoderAccumulators();
    _mc.getEncoderPositions(_dEncStartL, _dEncStartR);
    _dTargetMm  = targetMm;
    _dTimeoutMs = _lastTickMs + 5000;
    _mode       = DriveMode::DISTANCE;
    _driveFn    = fn;
    _driveCtx   = ctx;
    if (corr_id && corr_id[0] != '\0') {
        strncpy(_corrId, corr_id, sizeof(_corrId) - 1);
        _corrId[sizeof(_corrId) - 1] = '\0';
    } else {
        _corrId[0] = '\0';
    }
    (void)now_ms;
}

void DriveController::beginGoTo(float tx, float ty, float speedMms, uint32_t now_ms,
                                 ReplyFn fn, void* ctx, const char* corr_id)
{
    // Store goal in world frame by transforming robot-relative (tx, ty)
    // using the current odometry pose.
    float x, y, h_rad;
    getPoseFloat(x, y, h_rad);
    _gTargetXWorld = x + tx * cosf(h_rad) - ty * sinf(h_rad);
    _gTargetYWorld = y + tx * sinf(h_rad) + ty * cosf(h_rad);
    _gSpeed   = speedMms;
    _vRamped  = 0.0f;   // accel ramp starts fresh on each new go-to command
    _mode     = DriveMode::GO_TO;
    _driveFn  = fn;
    _driveCtx = ctx;
    if (corr_id && corr_id[0] != '\0') {
        strncpy(_corrId, corr_id, sizeof(_corrId) - 1);
        _corrId[sizeof(_corrId) - 1] = '\0';
    } else {
        _corrId[0] = '\0';
    }

    // Turn-in-place gate: bearing is computed from the robot-relative input
    // (tx, ty) at command time — the robot frame IS the input frame here.
    float bearing = fabsf(atan2f(ty, tx));
    float gateRad = _cfg.turnInPlaceGate * (3.14159265f / 180.0f);  // degrees → rad

    if (bearing > gateRad) {
        // Target is beside or behind the robot — pre-rotate in place first.
        //
        // Per-direction feedforward gain (012-006):
        //   turnSign > 0 → CCW (positive heading), use rotationGainPos / rotationOffsetDeg.
        //   turnSign < 0 → CW  (negative heading), use rotationGainNeg / rotationOffsetDegNeg.
        //
        // We apply the gain as a wheel-speed scalar on the initial feedforward command.
        // The per-direction gain corrects for mechanical asymmetry (e.g. the CW direction
        // under-rotates relative to CCW at equal wheel speeds). Dividing the commanded speed
        // by the gain means a mechanically "weak" direction spins proportionally faster,
        // delivering the same effective heading change per second as the stronger direction.
        //
        // Oscillation safety: this is a FEEDFORWARD correction applied once at command time.
        // PRE_ROTATE termination is determined by the OTOS-corrected bearing in tick(), so
        // the closed-loop heading accuracy is unaffected. No feedback path runs through the
        // gain, so there is no risk of oscillation or double-correction.
        //
        // The rotationOffsetDeg / rotationOffsetDegNeg fields represent a fixed startup-loss
        // angle (dead-band). In an open-loop model the correction is (target - offset) / gain.
        // Here, since the bearing gate (not a fixed target angle) terminates the turn, there
        // is no fixed target to subtract the offset from. The offset is therefore stored for
        // future open-loop callers and is NOT applied here; the closed-loop bearing gate
        // provides equivalent compensation.
        float turnSign = (ty >= 0.0f) ? 1.0f : -1.0f;
        float dirGain  = (turnSign > 0.0f) ? _cfg.rotationGainPos : _cfg.rotationGainNeg;
        // Guard against divide-by-zero or degenerate gain values.
        if (dirGain < 0.05f) dirGain = 0.05f;
        float rawL = -turnSign * (_gSpeed / dirGain);
        float rawR =  turnSign * (_gSpeed / dirGain);
        float sL, sR;
        BodyKinematics::saturate(rawL, rawR, _cfg.vWheelMax, _cfg.steerHeadroom, sL, sR);
        _mc.startDriveClean(sL, sR);
        _mc.setTarget(sL, sR);
        _gPhase = GPhase::PRE_ROTATE;
    } else {
        // Target is roughly ahead — enter pursuit directly.
        _mc.startDriveClean(_gSpeed, _gSpeed);  // initial setpoint; PURSUE corrects next tick
        _gPhase = GPhase::PURSUE;
    }

    (void)now_ms;
}

void DriveController::stop(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    fullStop(fn, ctx);
    (void)now_ms;
}

// ---------------------------------------------------------------------------
// enqueueEvt — called from the control fiber to record a completion event.
//
// Builds the full EVT string (with " #<corr_id>" suffix if set) and stores
// it in the ring buffer together with the per-drive reply sink.  The comms
// fiber drains the queue via drainEvents().
//
// CONCURRENCY: CODAL is cooperative — a fiber switch happens only at
// explicit yield points.  controlTick() never yields, so this is
// effectively atomic from the control fiber's perspective.
// ---------------------------------------------------------------------------

void DriveController::enqueueEvt(const char* base)
{
    int next = (_evtHead + 1) % kEvtQueueCap;
    if (next == _evtTail) {
        // Ring full — drop the oldest entry to make room (overwrite tail).
        _evtTail = (_evtTail + 1) % kEvtQueueCap;
    }
    EvtEntry& e = _evtQueue[_evtHead];
    if (_corrId[0] != '\0') {
        snprintf(e.msg, sizeof(e.msg), "%s #%s", base, _corrId);
    } else {
        // strncpy-equivalent: safe copy with guaranteed null terminator.
        int i = 0;
        while (base[i] && i < (int)sizeof(e.msg) - 1) {
            e.msg[i] = base[i];
            ++i;
        }
        e.msg[i] = '\0';
    }
    e.fn  = _driveFn  ? _driveFn  : nullptr;
    e.ctx = _driveFn  ? _driveCtx : nullptr;
    _evtHead = next;

    _corrId[0] = '\0';  // clear after enqueuing
}

// ---------------------------------------------------------------------------
// drainEvents — called from the comms+telemetry fiber.
//
// Pops all pending EVT entries and emits them via the supplied fallback
// fn/ctx.  If the entry has a captured per-drive sink, that is used
// instead (preserves the "reply to originating channel" invariant).
// ---------------------------------------------------------------------------

int DriveController::drainEvents(ReplyFn fn, void* ctx)
{
    int count = 0;
    while (_evtTail != _evtHead) {
        EvtEntry& e = _evtQueue[_evtTail];
        ReplyFn  efn = e.fn  ? e.fn  : fn;
        void*    ect = e.fn  ? e.ctx : ctx;
        if (efn && e.msg[0] != '\0') {
            efn(e.msg, ect);
        }
        e.msg[0] = '\0';
        e.fn  = nullptr;
        e.ctx = nullptr;
        _evtTail = (_evtTail + 1) % kEvtQueueCap;
        ++count;
    }
    return count;
}

// ---------------------------------------------------------------------------
// controlTick — control-fiber entry point (013-010).
//
// Runs at a fixed period set by RobotConfig::controlPeriodMs (default 10 ms).
// Executes the deterministic path only:
//   1. MotorController::tick() — encoder I2C reads + PID + Motor::setSpeed()
//   2. Odometry::predict() — dead-reckoning update from encoder delta
//   3. OTOS correction (slow cadence, 10 Hz)
//   4. Drive-mode state machines (STREAMING watchdog, T/D/G termination)
//   5. Enqueue any EVT completions into the ring buffer (no I/O here)
//
// Does NOT call any reply fn and does NOT yield (no fiber_sleep inside).
// The Motor I2C transactions are now busy-wait, so no scheduler switch
// occurs during encoder reads.
// ---------------------------------------------------------------------------

void DriveController::controlTick(uint32_t now_ms)
{
    // Throttle to controlPeriodMs cadence.
    // We still honour the cadence check so that if controlTick is called
    // more frequently than the configured period, extra calls are no-ops.
    if ((now_ms - _lastTickMs) < (uint32_t)_cfg.controlPeriodMs) return;

    float dt_s      = (float)(now_ms - _lastTickMs) / 1000.0f;
    _lastTickMs     = now_ms;
    _currentTimeMs  = now_ms;

    // Motor controller tick is now called by Robot::controlCollect() before
    // DriveController::controlTick() is reached (014-003).
    //
    // Odometry predict is now called by Robot::odometryPredict() which reads
    // encLMm/R directly from HardwareState (014-004). DriveController no longer
    // calls predict() here.

    // OTOS complementary correction (slow cadence: every kOtosSlowMs).
    // NOTE: ticket 005 will move this block into Robot::otosCorrect().
    // _hwState must be set via setHardwareState() before OTOS correction runs.
    if (_otos != nullptr && _hwState != nullptr &&
        (now_ms - _lastOtosMs) >= kOtosSlowMs) {
        _lastOtosMs = now_ms;
        int16_t rx = 0, ry = 0, rh = 0;
        _otos->getPositionRaw(rx, ry, rh);
        constexpr float kPosMmPerLsb  = 0.305f;
        constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);

        float xF = static_cast<float>(rx) * kPosMmPerLsb;
        float yF = static_cast<float>(ry) * kPosMmPerLsb;
        float hF = static_cast<float>(rh) * kHdgRadPerLsb;

        if (_cfg.odomUpsideDown) {
            xF = -xF;
            yF = -yF;
            hF = -hF;
        }

        float angRad = -_cfg.odomYawDeg * (3.14159265f / 180.0f);
        float c = cosf(angRad);
        float s = sinf(angRad);
        float x_mm = c * xF - s * yF - _cfg.odomOffX;
        float y_mm = s * xF + c * yF - _cfg.odomOffY;
        float h_rad = hF + _cfg.odomYawDeg * (3.14159265f / 180.0f);

        // Write OTOS readings into HardwareState for telemetry / downstream tasks.
        _hwState->otosX = x_mm;
        _hwState->otosY = y_mm;
        _hwState->otosH = h_rad;

        _odo.correct(*_hwState, x_mm, y_mm, h_rad,
                     _cfg.alphaPos, _cfg.alphaYaw, _cfg.otosGate);
    }

    // S-mode watchdog — enqueue EVT safety_stop when keepalive times out.
    if (_mode == DriveMode::STREAMING) {
        if ((now_ms - _lastSMs) > (uint32_t)_cfg.sTimeoutMs) {
            fullStop(nullptr, nullptr);
            enqueueEvt("EVT safety_stop");
        }
    }

    // T-mode: stop when deadline reached.
    if (_mode == DriveMode::TIMED && now_ms >= _tEndMs) {
        fullStop(nullptr, nullptr);
        enqueueEvt("EVT done T");
    }

    // D-mode: stop when average encoder travel >= target, or on timeout.
    if (_mode == DriveMode::DISTANCE) {
        int32_t l, r;
        _mc.getEncoderPositions(l, r);
        int32_t traveled = (abs(l - _dEncStartL) + abs(r - _dEncStartR)) / 2;
        if (traveled >= _dTargetMm || now_ms >= _dTimeoutMs) {
            fullStop(nullptr, nullptr);
            enqueueEvt("EVT done D");
        }
    }

    // G-mode: advance go-to state machine.
    if (_mode == DriveMode::GO_TO) {
        if (_gPhase == GPhase::PRE_ROTATE) {
            float x, y, h_rad;
            getPoseFloat(x, y, h_rad);
            float dxW  = _gTargetXWorld - x;
            float dyW  = _gTargetYWorld - y;
            float dx_rf =  dxW * cosf(h_rad) + dyW * sinf(h_rad);
            float dy_rf = -dxW * sinf(h_rad) + dyW * cosf(h_rad);
            float bearing = fabsf(atan2f(dy_rf, dx_rf));
            float gateRad = _cfg.turnInPlaceGate * (3.14159265f / 180.0f);

            if (bearing <= gateRad) {
                _vRamped = 0.0f;
                _gPhase  = GPhase::PURSUE;
            }
        }

        if (_gPhase == GPhase::PURSUE) {
            float x, y, h_rad;
            getPoseFloat(x, y, h_rad);

            float dxW = _gTargetXWorld - x;
            float dyW = _gTargetYWorld - y;
            float dx  =  dxW * cosf(h_rad) + dyW * sinf(h_rad);
            float dy  = -dxW * sinf(h_rad) + dyW * cosf(h_rad);

            float d2          = dx * dx + dy * dy;
            float d_remaining = sqrtf(d2);

            if (d_remaining < _cfg.arriveTolMm) {
                fullStop(nullptr, nullptr);
                _gPhase = GPhase::IDLE;
                enqueueEvt("EVT done G");
                return;
            }

            _vRamped += _cfg.aMax * dt_s;
            if (_vRamped > _gSpeed) _vRamped = _gSpeed;

            float v_cap = sqrtf(2.0f * _cfg.aDecel * d_remaining);
            if (v_cap < _vRamped) _vRamped = v_cap;

            float v     = _vRamped;
            float kappa = (d2 > 0.1f) ? (2.0f * dy / d2) : 0.0f;
            float omega = v * kappa;

            float vL, vR;
            BodyKinematics::inverse(v, omega, _cfg.trackwidthMm, vL, vR);
            float sL, sR;
            BodyKinematics::saturate(vL, vR, _cfg.vWheelMax, _cfg.steerHeadroom, sL, sR);
            _mc.setTarget(sL, sR);
        }
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void DriveController::fullStop(ReplyFn fn, void* ctx)
{
    _mc.stop();
    _mode  = DriveMode::IDLE;
    _tgtL  = 0.0f;
    _tgtR  = 0.0f;
    (void)fn;
    (void)ctx;
}

/**
 * Read the current odometry pose and convert to floating-point values.
 *
 * @param x      Output: x position in mm (float)
 * @param y      Output: y position in mm (float)
 * @param h_rad  Output: heading in radians
 */
void DriveController::getPoseFloat(float& x, float& y, float& h_rad) const {
    if (_hwState == nullptr) {
        x = 0.0f; y = 0.0f; h_rad = 0.0f;
        return;
    }
    int32_t xi, yi, hi;
    Odometry::getPose(*_hwState, xi, yi, hi);
    x     = static_cast<float>(xi);
    y     = static_cast<float>(yi);
    h_rad = static_cast<float>(hi) * (3.14159265f / 18000.0f);  // cdeg → rad
}
