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
{
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
        float turnSign = (ty >= 0.0f) ? 1.0f : -1.0f;
        float rawL = -turnSign * _gSpeed;
        float rawR =  turnSign * _gSpeed;
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
// tick
// ---------------------------------------------------------------------------

void DriveController::tick(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    // Throttle to tickMs cadence
    int32_t tickMs = _cfg.tickMs;
    if ((now_ms - _lastTickMs) < (uint32_t)tickMs) return;

    float dt_s      = (float)(now_ms - _lastTickMs) / 1000.0f;
    _lastTickMs     = now_ms;
    _currentTimeMs  = now_ms;

    // Run motor controller and update odometry (fast cadence: every tick)
    if (_mode != DriveMode::IDLE) {
        _mc.tick(dt_s);

        int32_t encL, encR;
        _mc.getEncoderPositions(encL, encR);
        _odo.predict(static_cast<float>(encL), static_cast<float>(encR),
                     _cfg.trackwidthMm);
    }

    // OTOS complementary correction (slow cadence: every kOtosSlowMs).
    // OtosSensor conversion constants (from OtosSensor.h register map comments):
    //   Position: 1 LSB = 0.305 mm  → x_mm = raw_x * 0.305
    //   Heading:  1 LSB = 0.00549°  → θ_rad = raw_h * 0.00549 * (π/180)
    // Runs even when IDLE so that the pose is corrected during pauses.
    if (_otos != nullptr && (now_ms - _lastOtosMs) >= kOtosSlowMs) {
        _lastOtosMs = now_ms;
        int16_t rx = 0, ry = 0, rh = 0;
        _otos->getPositionRaw(rx, ry, rh);
        constexpr float kPosMmPerLsb  = 0.305f;
        constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);
        float x_mm   = static_cast<float>(rx) * kPosMmPerLsb;
        float y_mm   = static_cast<float>(ry) * kPosMmPerLsb;
        float h_rad  = static_cast<float>(rh) * kHdgRadPerLsb;
        _odo.correct(x_mm, y_mm, h_rad,
                     _cfg.alphaPos, _cfg.alphaYaw, _cfg.otosGate);
    }

    // Convenience: drive sink (for async completions) vs active sink (for streaming).
    // _driveFn/_driveCtx: captured when the drive began — routes completions to
    // the channel that originated the command.
    // fn/ctx: the active channel sink — used for streaming telemetry only.
    ReplyFn  dfn = _driveFn  ? _driveFn  : fn;
    void*    dct = _driveFn  ? _driveCtx : ctx;

    // Helper: build an EVT line, appending " #<id>" when _corrId is set.
    // Uses a local buffer on the stack; safe because dfn() is called inline.
    auto emitEvt = [&](const char* base) {
        if (_corrId[0] != '\0') {
            char evtBuf[64];
            snprintf(evtBuf, sizeof(evtBuf), "%s #%s", base, _corrId);
            dfn(evtBuf, dct);
        } else {
            dfn(base, dct);
        }
        _corrId[0] = '\0';  // clear after emitting
    };

    // S-mode watchdog
    if (_mode == DriveMode::STREAMING) {
        if ((now_ms - _lastSMs) > (uint32_t)_cfg.sTimeoutMs) {
            fullStop(dfn, dct);
            emitEvt("EVT safety_stop");
        }
    }

    // T-mode: stop when deadline reached
    if (_mode == DriveMode::TIMED && now_ms >= _tEndMs) {
        fullStop(dfn, dct);
        emitEvt("EVT done T");
    }

    // D-mode: stop when average encoder travel >= target, or on timeout
    if (_mode == DriveMode::DISTANCE) {
        int32_t l, r;
        _mc.getEncoderPositions(l, r);
        int32_t traveled = (abs(l - _dEncStartL) + abs(r - _dEncStartR)) / 2;
        if (traveled >= _dTargetMm || now_ms >= _dTimeoutMs) {
            fullStop(dfn, dct);
            emitEvt("EVT done D");
        }
    }

    // G-mode: advance go-to state machine
    if (_mode == DriveMode::GO_TO) {
        if (_gPhase == GPhase::PRE_ROTATE) {
            // Continuously re-check the robot-frame bearing to the world-frame goal.
            // Exit to PURSUE when the bearing falls within the gate threshold.
            float x, y, h_rad;
            getPoseFloat(x, y, h_rad);
            float dxW  = _gTargetXWorld - x;
            float dyW  = _gTargetYWorld - y;
            // World → robot frame
            float dx_rf =  dxW * cosf(h_rad) + dyW * sinf(h_rad);
            float dy_rf = -dxW * sinf(h_rad) + dyW * cosf(h_rad);
            float bearing = fabsf(atan2f(dy_rf, dx_rf));
            float gateRad = _cfg.turnInPlaceGate * (3.14159265f / 180.0f);

            if (bearing <= gateRad) {
                // Bearing is now within threshold — transition to PURSUE.
                // Reset _vRamped so the accel ramp starts fresh from zero.
                _vRamped = 0.0f;
                _gPhase  = GPhase::PURSUE;
                // PURSUE tick will set correct wheel speeds on next iteration.
            }
            // else: keep spinning (wheel setpoints set at beginGoTo() remain active).
        }

        if (_gPhase == GPhase::PURSUE) {
            float x, y, h_rad;
            getPoseFloat(x, y, h_rad);

            // World-frame offset → robot frame
            float dxW = _gTargetXWorld - x;
            float dyW = _gTargetYWorld - y;
            float dx  =  dxW * cosf(h_rad) + dyW * sinf(h_rad);  // forward in robot frame
            float dy  = -dxW * sinf(h_rad) + dyW * cosf(h_rad);  // left in robot frame

            float d2          = dx * dx + dy * dy;
            float d_remaining = sqrtf(d2);  // one sqrt per tick; used for decel cap and arrival

            // Arrival detection: stop and emit completion when within tolerance.
            if (d_remaining < _cfg.arriveTolMm) {
                fullStop(dfn, dct);
                _gPhase = GPhase::IDLE;
                emitEvt("EVT done G");
                return;   // skip further PURSUE logic this tick
            }

            // Trapezoidal speed shaper (kinematics-model.md §1.6):
            //   1. Ramp up _vRamped toward _gSpeed at aMax per second.
            //   2. Cap by decel curve: v_cap = sqrt(2 * aDecel * d_remaining).
            //   3. v = min(_vRamped, v_cap, _gSpeed) — three-way min.
            _vRamped += _cfg.aMax * dt_s;
            if (_vRamped > _gSpeed) _vRamped = _gSpeed;   // clamp to user-commanded max

            float v_cap = sqrtf(2.0f * _cfg.aDecel * d_remaining);
            if (v_cap < _vRamped) _vRamped = v_cap;       // clamp ramped speed to decel cap

            float v     = _vRamped;   // v ≤ _gSpeed and v ≤ v_cap

            float kappa = (d2 > 0.1f) ? (2.0f * dy / d2) : 0.0f;  // κ = 2dy/(dx²+dy²)
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
    int32_t xi, yi, hi;
    _odo.getPose(xi, yi, hi);
    x     = static_cast<float>(xi);
    y     = static_cast<float>(yi);
    h_rad = static_cast<float>(hi) * (3.14159265f / 18000.0f);  // cdeg → rad
}
