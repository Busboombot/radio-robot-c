// DriveController.cpp — S/T/D/G drive state machines, S-mode watchdog,
// streaming encoder counter, and odometry delta tracking.
//
// Transplanted from CommandProcessor.cpp (Sprint 007, Ticket 003).
// All speeds in mm/s; distances in mm.
//
// Sprint 010, Ticket 007: All wheel setpoints routed through
// BodyKinematics::saturate() before reaching MotorController, preserving
// arc curvature when commanded speeds exceed vWheelMax - steerHeadroom.
//
// Sprint 014, Ticket 005: EVT ring buffer removed.  Completions emitted
// inline via target.replyFn / target.replyCtx / target.corrId.
// OTOS correction removed — handled by AppContext::otosCorrect() exclusively.
//
// Sprint 017, Ticket 004: VW migrated from STREAMING path onto MotionCommand.
// _bvc and _activeCmd added as value members.  beginVelocity now configures
// a MotionCommand with a TIME stop condition (keepalive watchdog at sTimeoutMs).
// driveAdvance ticks _activeCmd when active; STREAMING watchdog fires only for S.

#include "DriveController.h"
#include "MotorController.h"
#include "Odometry.h"
#include "BodyKinematics.h"
#include "StopCondition.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

DriveController::DriveController(MotorController& mc, Odometry& odo, const RobotConfig& cfg)
    : _mc(mc)
    , _odo(odo)
    , _cfg(cfg)
    , _hwState(nullptr)
    , _bvc(mc, cfg)     // _bvc must be initialised before _activeCmd (declaration order)
    , _activeCmd()
    , _mode(DriveMode::IDLE)
    , _lastSMs(0)
    , _tgtL(0.0f)
    , _tgtR(0.0f)
    , _dEncStartL(0)
    , _dEncStartR(0)
    , _dTargetMm(0)
    , _dTimeoutMs(0)
    , _gPhase(GPhase::IDLE)
    , _gTargetXWorld(0.0f)
    , _gTargetYWorld(0.0f)
    , _gSpeed(0.0f)
    , _lastTickMs(0)
    , _currentTimeMs(0)
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
// emitEvt — inline EVT emission via the captured reply sink in target.
//
// Builds the full EVT string ("<base> #<corrId>" when corrId is non-empty,
// else just "<base>") and calls target.replyFn immediately.
// Clears target.corrId after emitting so a subsequent completion on the
// same target does not re-use a stale id.
//
// Safe to call I/O inline — the single cooperative main loop has no
// fiber boundary that would block I/O in driveAdvance().
// ---------------------------------------------------------------------------

/*static*/ void DriveController::emitEvt(const char* base, TargetState& target)
{
    if (!target.replyFn) return;

    char msg[48];
    if (target.corrId[0] != '\0') {
        snprintf(msg, sizeof(msg), "%s #%s", base, target.corrId);
    } else {
        int i = 0;
        while (base[i] && i < (int)sizeof(msg) - 1) {
            msg[i] = base[i];
            ++i;
        }
        msg[i] = '\0';
    }

    target.replyFn(msg, target.replyCtx);
    target.corrId[0] = '\0';  // consumed
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

void DriveController::beginStream(float leftMms, float rightMms, uint32_t now_ms,
                                   TargetState& target, ReplyFn fn, void* ctx)
{
    float sL, sR;
    applySaturation(leftMms, rightMms, _cfg, sL, sR);
    _mc.startDrive(sL, sR);
    _mc.setTarget(sL, sR);
    _tgtL    = sL;
    _tgtR    = sR;
    _mode    = DriveMode::STREAMING;
    _lastSMs = now_ms;

    target.mode     = DriveMode::STREAMING;
    target.replyFn  = fn;
    target.replyCtx = ctx;
    // corrId cleared — S mode uses watchdog, no completion id
    target.corrId[0] = '\0';
}

void DriveController::beginVelocity(float v_mms, float omega_rads, uint32_t now_ms,
                                     TargetState& target, ReplyFn fn, void* ctx,
                                     const char* corr_id)
{
    // Configure a fresh MotionCommand for body-twist (v, ω) with:
    //   - TIME stop condition at sTimeoutMs (keepalive watchdog).
    //   - SOFT stop style (ramp to zero before completing).
    //   - EVT "EVT safety_stop" on completion (preserves wire contract).
    //   - Reply sink for async EVT delivery.
    _activeCmd.configure(v_mms, omega_rads, &_bvc);
    _activeCmd.addStop(makeTimeStop((float)_cfg.sTimeoutMs));
    _activeCmd.setReplySink(fn, ctx, corr_id);
    _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
    // Override done EVT to preserve the VW keepalive-loss wire contract.
    _activeCmd.setDoneEvt("EVT safety_stop");

    // Snapshot hardware state for MotionBaseline; use _hwState if available.
    HardwareState emptyState{};
    const HardwareState& inputs = _hwState ? *_hwState : emptyState;
    _activeCmd.start(inputs, now_ms);

    // Set mode to VELOCITY — distinct from STREAMING so the S-mode watchdog
    // branch in driveAdvance does NOT fire for VW.
    _mode = DriveMode::VELOCITY;

    // Do NOT write to target.replyFn for VW — the reply sink is captured by
    // _activeCmd.  target is updated only for the TLM mode field.
    target.mode = DriveMode::VELOCITY;
}

void DriveController::beginArc(float speedMms, float radiusMm, uint32_t now_ms,
                                TargetState& target, ReplyFn fn, void* ctx,
                                const char* corr_id)
{
    // Compute arc curvature κ = 1/radius; radius==0 ⇒ κ=0 (straight).
    // Sign convention: positive radius ⇒ positive ω ⇒ CCW/left arc.
    // This matches BodyKinematics::inverse where CCW-positive ω gives vL < vR.
    float kappa = (radiusMm != 0.0f) ? (1.0f / radiusMm) : 0.0f;
    float omega  = speedMms * kappa;

    // Configure a fresh MotionCommand for body-twist (v, ω) with:
    //   - No stop conditions (open-ended; host cancels via X or R 0 r).
    //   - SOFT stop style (ramp to zero before completing).
    //   - EVT "EVT done R" on normal (SOFT ramp-down) completion.
    //   - Reply sink for async EVT delivery.
    _activeCmd.configure(speedMms, omega, &_bvc);
    // No addStop: open-ended arc; keepalive via X or R 0 r.
    _activeCmd.setReplySink(fn, ctx, corr_id);
    _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
    _activeCmd.setDoneEvt("EVT done R");

    // Snapshot hardware state for MotionBaseline.
    HardwareState emptyState{};
    const HardwareState& inputs = _hwState ? *_hwState : emptyState;
    _activeCmd.start(inputs, now_ms);

    // VELOCITY mode — distinct from STREAMING so the S-mode watchdog does not fire.
    _mode = DriveMode::VELOCITY;

    // Update target mode; reply sink captured by _activeCmd (not target.replyFn).
    target.mode = DriveMode::VELOCITY;
}

void DriveController::beginTimed(float leftMms, float rightMms,
                                  uint32_t durationMs, uint32_t now_ms,
                                  TargetState& target, ReplyFn fn, void* ctx,
                                  const char* corr_id)
{
    // Convert (L, R) wheel speeds to body twist (v, ω) via the forward kinematics map.
    // For equal L=R (straight drive), forward() gives v=(L+R)/2 and omega=0 — no steer bias.
    float v_mms, omega_rads;
    BodyKinematics::forward(leftMms, rightMms, _cfg.trackwidthMm, v_mms, omega_rads);

    // Configure a fresh MotionCommand with:
    //   - TIME stop condition at durationMs.
    //   - SOFT stop style (ramp to zero before completing).
    //   - EVT "EVT done T" on completion (preserves wire contract).
    //   - Reply sink for async EVT delivery.
    _activeCmd.configure(v_mms, omega_rads, &_bvc);
    _activeCmd.addStop(makeTimeStop((float)durationMs));
    _activeCmd.setReplySink(fn, ctx, corr_id);
    _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
    _activeCmd.setDoneEvt("EVT done T");

    // Snapshot hardware state for MotionBaseline.
    HardwareState emptyState{};
    const HardwareState& inputs = _hwState ? *_hwState : emptyState;
    _activeCmd.start(inputs, now_ms);

    // VELOCITY mode — distinct from STREAMING so the S-mode watchdog does not fire.
    _mode = DriveMode::VELOCITY;

    // Update target mode; reply sink captured by _activeCmd (not target.replyFn).
    target.mode = DriveMode::VELOCITY;
}

void DriveController::beginDistance(float leftMms, float rightMms,
                                     int32_t targetMm, uint32_t now_ms,
                                     TargetState& target, ReplyFn fn, void* ctx,
                                     const char* corr_id)
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
    // Timeout scales with the expected drive time (distance / speed), not a
    // fixed 5 s — otherwise a slow or long drive (e.g. 900 mm @ 80 mm/s = 11 s)
    // is cut off early and never reaches its target. 2x nominal + 2 s margin.
    {
        float spdMax = fmaxf(fabsf(sL), fabsf(sR));
        if (spdMax < 1.0f) spdMax = 1.0f;
        uint32_t nominalMs = (uint32_t)(((float)targetMm / spdMax) * 1000.0f);
        _dTimeoutMs = _lastTickMs + nominalMs * 2u + 2000u;
    }
    _mode       = DriveMode::DISTANCE;

    target.mode              = DriveMode::DISTANCE;
    target.distanceTargetMm  = static_cast<float>(targetMm);
    target.replyFn           = fn;
    target.replyCtx          = ctx;
    if (corr_id && corr_id[0] != '\0') {
        strncpy(target.corrId, corr_id, sizeof(target.corrId) - 1);
        target.corrId[sizeof(target.corrId) - 1] = '\0';
    } else {
        target.corrId[0] = '\0';
    }
    (void)now_ms;
}

void DriveController::beginGoTo(float tx, float ty, float speedMms, uint32_t now_ms,
                                 TargetState& target, ReplyFn fn, void* ctx,
                                 const char* corr_id)
{
    // Store goal in world frame by transforming robot-relative (tx, ty)
    // using the current odometry pose.
    float x, y, h_rad;
    getPoseFloat(x, y, h_rad);
    _gTargetXWorld = x + tx * cosf(h_rad) - ty * sinf(h_rad);
    _gTargetYWorld = y + tx * sinf(h_rad) + ty * cosf(h_rad);
    _gSpeed   = speedMms;
    _mode     = DriveMode::GO_TO;

    target.mode           = DriveMode::GO_TO;
    target.targetXWorld   = _gTargetXWorld;
    target.targetYWorld   = _gTargetYWorld;
    target.targetSpeedMms = speedMms;
    target.replyFn        = fn;
    target.replyCtx       = ctx;
    if (corr_id && corr_id[0] != '\0') {
        strncpy(target.corrId, corr_id, sizeof(target.corrId) - 1);
        target.corrId[sizeof(target.corrId) - 1] = '\0';
    } else {
        target.corrId[0] = '\0';
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
        // Configure MotionCommand with a POSITION stop at the world target.
        // The per-tick pursuit hook in driveAdvance will update the (v, ω) target
        // each tick before _activeCmd.tick() is called.
        _activeCmd.configure(_gSpeed, 0.0f, &_bvc);
        _activeCmd.addStop(makePositionStop(_gTargetXWorld, _gTargetYWorld, _cfg.arriveTolMm));
        _activeCmd.setReplySink(fn, ctx, corr_id);
        _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
        _activeCmd.setDoneEvt("EVT done G");
        HardwareState emptyState{};
        const HardwareState& inputs = _hwState ? *_hwState : emptyState;
        _activeCmd.start(inputs, now_ms);
        _gPhase = GPhase::PURSUE;
    }
}

void DriveController::stop(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    // Cancel any active MotionCommand before calling fullStop().
    if (_activeCmd.active()) {
        _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    }
    _gPhase = GPhase::IDLE;  // reset G phase on hard stop
    fullStop(fn, ctx);
    (void)now_ms;
}

void DriveController::cancel(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    _mc.stop();
    _mode   = DriveMode::IDLE;
    _gPhase = GPhase::IDLE;  // reset G phase on any cancel
    (void)now_ms;
    (void)fn;
    (void)ctx;
}

// ---------------------------------------------------------------------------
// driveAdvance — cooperative-loop task entry point (014-005).
//
// Runs at a fixed period set by RobotConfig::controlPeriodMs (default 10 ms).
// Executes the drive-mode state machines:
//   1. STREAMING watchdog — emits EVT safety_stop inline on keepalive timeout.
//   2. D-mode — emits EVT done D inline when distance reached or timeout.
//   3. G-mode — advances PRE_ROTATE and PURSUE; emits EVT done G inline.
//   T-mode is now handled by the MotionCommand path (TIME stop condition).
//
// All EVT completions are emitted inline via target.replyFn() — safe because
// there is no fiber boundary in the single cooperative main loop (014-005).
//
// NOTE: OTOS correction is NOT done here.  It is the sole responsibility of
// AppContext::otosCorrect() called at the slow cadence in LoopScheduler
// (ticket 005 wired this; ticket 006 moved it to the scheduler task).
// ---------------------------------------------------------------------------

void DriveController::driveAdvance(HardwareState& inputs, MotorCommands& cmds,
                                    TargetState& target, uint32_t now_ms)
{
    // Throttle to controlPeriodMs cadence.
    if ((now_ms - _lastTickMs) < (uint32_t)_cfg.controlPeriodMs) return;

    float dt_s      = (float)(now_ms - _lastTickMs) / 1000.0f;
    _lastTickMs     = now_ms;
    _currentTimeMs  = now_ms;

    // Motor controller tick and odometry predict are called by AppContext::controlCollectSplitPhase()
    // and odometry.predict() before driveAdvance() is reached (014-003/004).
    (void)cmds;

    // ── MotionCommand tick (VW / R / G PURSUE / future MotionCommand-based verbs) ─
    // When a MotionCommand is active, tick it exactly once and return early.
    // The old S/T/D/G if-chain runs ONLY when no MotionCommand is active.
    // This also prevents the STREAMING watchdog branch below from firing for VW.
    if (_activeCmd.active()) {
        // G PURSUE hook: recompute (v, ω) from current pose each tick and call
        // setTarget BEFORE _activeCmd.tick() so the BVC advances with the
        // updated target this tick.
        if (_mode == DriveMode::GO_TO && _gPhase == GPhase::PURSUE) {
            float x, y, h_rad;
            getPoseFloat(x, y, h_rad);

            float dxW = _gTargetXWorld - x;
            float dyW = _gTargetYWorld - y;
            float dx  =  dxW * cosf(h_rad) + dyW * sinf(h_rad);
            float dy  = -dxW * sinf(h_rad) + dyW * cosf(h_rad);

            float d2          = dx * dx + dy * dy;
            float d_remaining = sqrtf(d2);

            // Terminal decel cap: v_cap = sqrt(2 * aDecel * d_remaining).
            // Clamps the commanded speed to ensure the BVC has time to
            // decelerate to zero before the POSITION stop fires.
            float v     = _gSpeed;
            float v_cap = sqrtf(2.0f * _cfg.aDecel * d_remaining);
            if (v_cap < v) v = v_cap;

            float kappa = (d2 > 0.1f) ? (2.0f * dy / d2) : 0.0f;
            float omega = v * kappa;

            _activeCmd.setTarget(v, omega);
        }

        bool still_running = _activeCmd.tick(inputs, now_ms, dt_s);
        if (!still_running) {
            // MotionCommand terminated; go IDLE.
            _mode = DriveMode::IDLE;
            target.mode = DriveMode::IDLE;
            // Reset G phase so a subsequent go-to command starts clean.
            if (_gPhase != GPhase::IDLE) _gPhase = GPhase::IDLE;
        }
        return;
    }

    // S-mode watchdog — emit EVT safety_stop when keepalive times out.
    // (Re-enabled after the encoder-wedge fix: Motor::setSpeed is now
    // write-on-change, so fullStop()'s 0x5F stop is sent once instead of being
    // spammed every tick — which was what wedged the encoder.)
    // Guarded by _mode == STREAMING so VW (VELOCITY) does NOT trigger this.
    (void)inputs;
    if (_mode == DriveMode::STREAMING) {
        // Wraparound/ordering-SAFE elapsed time. _lastSMs can be a hair AHEAD of
        // now_ms: the scheduler samples now_ms at the top of the loop, but the
        // keepalive S is processed slightly later in the same iteration and sets
        // _lastSMs from a fresh systemTime(). A plain uint32 (now_ms - _lastSMs)
        // then underflows to ~4.29e9 and trips a spurious safety_stop EVERY tick
        // an S lands in that window (the velocity "notches" / momentary stops).
        // A signed delta treats a small negative as "0 ms elapsed".
        int32_t dt = (int32_t)(now_ms - _lastSMs);
        if (dt > (int32_t)_cfg.sTimeoutMs) {
            fullStop(nullptr, nullptr);
            emitEvt("EVT safety_stop", target);
        }
    }

    // D-mode: stop when average encoder travel >= target, or on timeout. Uses a
    // fresh atomic read (NOT the filtered control-loop encLMm): the filtered
    // value can STALL when the outlier filter holds a run of glitchy reads, which
    // makes D fail to reach target and not stop (runaway). The fresh read always
    // tracks the real wheel, so D completes reliably.
    if (_mode == DriveMode::DISTANCE) {
        int32_t l, r;
        _mc.getEncoderPositions(l, r);
        int32_t traveled = (abs(l - _dEncStartL) + abs(r - _dEncStartR)) / 2;
        if (traveled >= _dTargetMm || now_ms >= _dTimeoutMs) {
            fullStop(nullptr, nullptr);
            emitEvt("EVT done D", target);
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
                // PRE_ROTATE → PURSUE transition: configure and start _activeCmd
                // with a POSITION stop at the world target.  The per-tick hook
                // above will update (v, ω) each tick before _activeCmd.tick().
                _activeCmd.configure(_gSpeed, 0.0f, &_bvc);
                _activeCmd.addStop(makePositionStop(_gTargetXWorld, _gTargetYWorld, _cfg.arriveTolMm));
                _activeCmd.setReplySink(target.replyFn, target.replyCtx, target.corrId);
                _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
                _activeCmd.setDoneEvt("EVT done G");
                const HardwareState& hw = _hwState ? *_hwState : inputs;
                _activeCmd.start(hw, now_ms);
                _gPhase = GPhase::PURSUE;
            }
        }

        // PURSUE is now handled by the MotionCommand path at the top of
        // driveAdvance — control never reaches here while PURSUE is active.
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
