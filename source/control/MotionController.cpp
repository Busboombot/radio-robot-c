// MotionController.cpp — S/T/D/G drive state machines, S-mode watchdog,
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
// OTOS correction removed — handled by Robot::otosCorrect() exclusively.
//
// Sprint 017, Ticket 004: VW migrated from STREAMING path onto MotionCommand.
// _bvc and _activeCmd added as value members.  beginVelocity now configures
// a MotionCommand with a TIME stop condition (keepalive watchdog at sTimeoutMs).
// driveAdvance ticks _activeCmd when active; STREAMING watchdog fires only for S.
//
// Sprint 020, Ticket 011: S/T/D/G/R/TURN converted to VW converter handlers.
//
// Sprint 026, Ticket 002: Handler/parser/reply code extracted to
// source/app/MotionCommandHandlers.cpp.  The protocol headers
// (CommandProcessor, CommandQueue) are no longer included here.
// emitEvt now calls through MotionEventSink stored in TargetState.

#include "MotionController.h"
#include "MotorController.h"
#include "Odometry.h"
#include "BodyKinematics.h"
#include "StopCondition.h"
#include "Robot.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

MotionController::MotionController(MotorController& mc, Odometry& odo, const RobotConfig& cfg)
    : _mc(mc)
    , _odo(odo)
    , _cfg(cfg)
    , _hwState(nullptr)
    , _robot(nullptr)   // set later by setRobotCtx()
    , _bvc(mc, cfg)     // _bvc must be initialised before _activeCmd (declaration order)
    , _activeCmd()
    , _mode(DriveMode::IDLE)
    , _tgtL(0.0f)
    , _tgtR(0.0f)
    , _dDistTarget(0.0f)
    , _dOmega(0.0f)
    , _dEnc0(0.0f)
    , _gPhase(GPhase::IDLE)
    , _gTargetXWorld(0.0f)
    , _gTargetYWorld(0.0f)
    , _gSpeed(0.0f)
    , _lastTickMs(0)
    , _currentTimeMs(0)
{
}

// ---------------------------------------------------------------------------
// localEvtEmitter — default MotionEventSink::emitFn for TargetState-based
// EVT delivery.
//
// ctx is a TargetState* (set to &target by beginGoTo when wiring the sink).
// Formats "<base> #<corrId>" and calls target->replyFn(msg, target->replyCtx).
//
// Lives in MotionController.cpp (control layer) so no app-layer includes are
// needed — only TargetState (RobotState.h, also control layer).
//
// Sprint 026-002: replaces the old inline formatting inside emitEvt.
// ---------------------------------------------------------------------------
static void localEvtEmitter(const char* base, const char* corrId, void* ctx)
{
    TargetState* t = static_cast<TargetState*>(ctx);
    if (!t->replyFn) return;
    char msg[48];
    if (corrId && corrId[0] != '\0') {
        snprintf(msg, sizeof(msg), "%s #%s", base, corrId);
    } else {
        int i = 0;
        while (base[i] && i < (int)sizeof(msg) - 1) { msg[i] = base[i]; ++i; }
        msg[i] = '\0';
    }
    t->replyFn(msg, t->replyCtx);
}

// ---------------------------------------------------------------------------
// emitEvt — inline EVT emission via the MotionEventSink stored in target.
//
// Calls target.sink.emitFn(base, corrId, ctx) if set.
// Clears target.corrId after emitting so a subsequent completion on the
// same target does not re-use a stale id.
//
// Sprint 026-002: calls through MotionEventSink rather than formatting inline.
// MotionController has no protocol-header dependency.
// ---------------------------------------------------------------------------

/*static*/ void MotionController::emitEvt(const char* base, TargetState& target)
{
    if (target.sink.emitFn) {
        target.sink.emitFn(base, target.corrId, target.sink.ctx);
    }
    target.corrId[0] = '\0';  // consumed
}

// ---------------------------------------------------------------------------
// _checkSafeOneShot — re-arm safety if the one-shot disable flag is set.
//
// Called at the start of every begin*() entry point after the cancel-if-active
// guard (ticket 002) and before configure().  If _safeOneShotDisable is true:
//   - restore _cfg.safetyEnabled = true
//   - emit "EVT safety re-armed" via the command reply sink
//   - clear the flag
// This ensures SAFE off is a one-shot bypass: the operator can issue a single
// motion command without keepalives, and safety is automatically restored for
// that command and all subsequent ones.
//
// Note: _cfg is a const-ref; safetyEnabled lives in the mutable RobotConfig
// owned by Robot.  Access via _robot->config when available, which IS the
// same object as _cfg (Robot passes &cfg to MotionController).  Cast away
// const here — we are legitimately mutating config.safetyEnabled as the
// designated "re-arm" code path (ticket 024-003 decision: MotionController
// owns the flag; architecture review confirmed).
// ---------------------------------------------------------------------------
void MotionController::_checkSafeOneShot(ReplyFn fn, void* ctx)
{
    if (!_safeOneShotDisable) return;
    _safeOneShotDisable = false;
    // Re-arm via the mutable Robot config (same object as _cfg).
    if (_robot != nullptr) {
        _robot->config.safetyEnabled = true;
    }
    // Emit the re-arm event.
    if (fn) {
        fn("EVT safety re-armed", ctx);
    }
}

// ---------------------------------------------------------------------------
// Entry points
// ---------------------------------------------------------------------------

void MotionController::beginStream(float leftMms, float rightMms, uint32_t now_ms,
                                   TargetState& target, ReplyFn fn, void* ctx)
{
    // Re-arm safety if SAFE off was issued (one-shot disable, sprint 024-003).
    _checkSafeOneShot(fn, ctx);

    // Convert wheel speeds to body twist via forward kinematics, then route
    // through BVC so all wheel commands go through the profiler path.
    float v, omega;
    BodyKinematics::forward(leftMms, rightMms, _cfg.trackwidthMm, v, omega);
    _bvc.seedCurrent(v, omega);
    _bvc.setTarget(v, omega);

    _tgtL = leftMms;
    _tgtR = rightMms;
    _mode = DriveMode::STREAMING;
    (void)now_ms;  // watchdog now lives in LoopScheduler

    target.mode     = DriveMode::STREAMING;
    target.replyFn  = fn;
    target.replyCtx = ctx;
    // corrId cleared — S mode uses system watchdog, no completion id
    target.corrId[0] = '\0';
    target.sink      = {};  // no async EVT for streaming mode
}

void MotionController::beginVelocity(float v_mms, float omega_rads, uint32_t now_ms,
                                     TargetState& target, ReplyFn fn, void* ctx,
                                     const char* corr_id)
{
    // Cancel any stale MotionCommand before configuring the new one.
    // cancel(HARD) emits "EVT cancelled" via the stored reply sink (making the
    // transition observable on the wire), then goes IDLE.  configure() below
    // clears the reply sink pointer afterward — no use-after-free.
    if (_activeCmd.active()) {
        _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    }

    // Re-arm safety if SAFE off was issued (one-shot disable, sprint 024-003).
    _checkSafeOneShot(fn, ctx);

    // Configure a fresh MotionCommand for body-twist (v, ω) with:
    //   - No TIME stop (keepalive watchdog is now the system watchdog in
    //     LoopScheduler — fires EVT safety_stop + X after sTimeoutMs silence).
    //   - SOFT stop style (ramp to zero before completing).
    //   - No reply sink needed — VW has no correlated EVT done; system watchdog
    //     emits EVT safety_stop directly.
    _activeCmd.configure(v_mms, omega_rads, &_bvc);
    // No addStop: system watchdog in LoopScheduler owns keepalive enforcement.
    _activeCmd.setReplySink(fn, ctx, corr_id);
    _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);

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

void MotionController::beginArc(float speedMms, float radiusMm, uint32_t now_ms,
                                TargetState& target, ReplyFn fn, void* ctx,
                                const char* corr_id)
{
    // Compute arc curvature κ = 1/radius; radius==0 ⇒ κ=0 (straight).
    // Sign convention: positive radius ⇒ positive ω ⇒ CCW/left arc.
    // This matches BodyKinematics::inverse where CCW-positive ω gives vL < vR.
    float kappa = (radiusMm != 0.0f) ? (1.0f / radiusMm) : 0.0f;
    float omega  = speedMms * kappa;

    // Cancel any stale MotionCommand before configuring the new one.
    if (_activeCmd.active()) {
        _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    }

    // Re-arm safety if SAFE off was issued (one-shot disable, sprint 024-003).
    _checkSafeOneShot(fn, ctx);

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

void MotionController::beginTimed(float leftMms, float rightMms,
                                  uint32_t durationMs, uint32_t now_ms,
                                  TargetState& target, ReplyFn fn, void* ctx,
                                  const char* corr_id)
{
    // Convert (L, R) wheel speeds to body twist (v, ω) via the forward kinematics map.
    // For equal L=R (straight drive), forward() gives v=(L+R)/2 and omega=0 — no steer bias.
    float v_mms, omega_rads;
    BodyKinematics::forward(leftMms, rightMms, _cfg.trackwidthMm, v_mms, omega_rads);

    // Re-arm safety if SAFE off was issued (one-shot disable, sprint 024-003).
    _checkSafeOneShot(fn, ctx);

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

void MotionController::beginDistance(float leftMms, float rightMms,
                                     int32_t targetMm, uint32_t now_ms,
                                     TargetState& target, ReplyFn fn, void* ctx,
                                     const char* corr_id)
{
    // Convert (L, R) wheel speeds to body twist (v, ω) via forward kinematics.
    float v_mms, omega_rads;
    BodyKinematics::forward(leftMms, rightMms, _cfg.trackwidthMm, v_mms, omega_rads);

    // Encoder-reset workaround: reset the accumulator so DISTANCE delta starts
    // from 0.  The state.inputs.encLMm/R baseline reset is done by Robot::
    // distanceDrive() after this call — do not move that reset here.
    _mc.resetEncoderAccumulators();

    // Capture encoder baseline for per-tick decel hook.  After resetEncoderAccumulators()
    // the hardware positions are 0; reading immediately gives a clean 0 baseline.
    int32_t encL0_raw, encR0_raw;
    _mc.getEncoderPositions(encL0_raw, encR0_raw);
    _dEnc0       = ((float)encL0_raw + (float)encR0_raw) * 0.5f;

    // Store decel-hook state.
    _dDistTarget = (float)targetMm;
    _dOmega      = omega_rads;

    // Re-arm safety if SAFE off was issued (one-shot disable, sprint 024-003).
    _checkSafeOneShot(fn, ctx);

    // Timeout: 2× nominal travel time + 2 s safety margin.
    // The nominal time is |targetMm| / max(|vL|, |vR|) in seconds.
    // With profiled ramp-up the robot covers slightly less ground in the first
    // ~200 ms than at full speed, so actual travel time is slightly longer than
    // nominal — the 2× factor absorbs this comfortably.
    float spdMax = fmaxf(fabsf(leftMms), fabsf(rightMms));
    if (spdMax < 1.0f) spdMax = 1.0f;
    float nominalMs = ((float)targetMm / spdMax) * 1000.0f;
    float timeoutMs = nominalMs * 2.0f + 2000.0f;

    // Configure a fresh MotionCommand with:
    //   - DISTANCE stop condition as the primary trigger.
    //   - TIME stop as safety net (generous; tolerates profiled ramp-up).
    //   - SOFT stop style (ramp to zero before completing).
    //   - EVT "EVT done D" on completion (preserves wire contract).
    //   - Reply sink for async EVT delivery.
    _activeCmd.configure(v_mms, omega_rads, &_bvc);
    _activeCmd.addStop(makeDistanceStop((float)targetMm));
    _activeCmd.addStop(makeTimeStop(timeoutMs));
    _activeCmd.setReplySink(fn, ctx, corr_id);
    _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
    _activeCmd.setDoneEvt("EVT done D");

    // Snapshot hardware state for MotionBaseline.
    // After resetEncoderAccumulators() the accumulators are 0; Robot will
    // also zero state.inputs.encLMm/R immediately after this call returns, so
    // the baseline enc0 captured by MotionCommand::start() will be 0 — matching
    // the DISTANCE stop evaluation which reads (encLMm + encRMm)/2 from HardwareState.
    HardwareState emptyState{};
    const HardwareState& inputs = _hwState ? *_hwState : emptyState;
    _activeCmd.start(inputs, now_ms);

    // DISTANCE mode — distinct from STREAMING so the S-mode watchdog does not fire.
    _mode = DriveMode::DISTANCE;

    // Update target mode; reply sink captured by _activeCmd (not target.replyFn).
    target.mode             = DriveMode::DISTANCE;
    target.distanceTargetMm = static_cast<float>(targetMm);
}

void MotionController::beginGoTo(float tx, float ty, float speedMms, uint32_t now_ms,
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
    // Wire the MotionEventSink so emitEvt() can reach the reply channel.
    // localEvtEmitter formats "<base> #<corrId>" and calls target.replyFn.
    // Sprint 026-002: replaces direct CommandProcessor::replyEvt calls in
    // driveAdvance with a clean sink through target.
    target.sink.emitFn = localEvtEmitter;
    target.sink.ctx    = &target;

    // Re-arm safety if SAFE off was issued (one-shot disable, sprint 024-003).
    // Done before the PRE_ROTATE / PURSUE branch so both paths benefit.
    _checkSafeOneShot(fn, ctx);

    // Turn-in-place gate: bearing is computed from the robot-relative input
    // (tx, ty) at command time — the robot frame IS the input frame here.
    float bearing = fabsf(atan2f(ty, tx));
    float gateRad = _cfg.turnInPlaceGate * (3.14159265f / 180.0f);  // degrees → rad

    if (bearing > gateRad) {
        // Target is beside or behind the robot — pre-rotate in place first.
        // Per-direction feedforward gain (012-006): CCW uses rotationGainPos,
        // CW uses rotationGainNeg.  FEEDFORWARD correction only; bearing gate
        // provides closed-loop compensation (no oscillation risk).
        float turnSign = (ty >= 0.0f) ? 1.0f : -1.0f;
        float dirGain  = (turnSign > 0.0f) ? _cfg.rotationGainPos : _cfg.rotationGainNeg;
        if (dirGain < 0.05f) dirGain = 0.05f;
        // omega = 2*(gSpeed/dirGain) / trackwidth  (spin-in-place from inverse kinematics)
        float wheelSpd = _gSpeed / dirGain;
        float omega    = turnSign * 2.0f * wheelSpd / _cfg.trackwidthMm;
        float omegaMax = 2.0f * _cfg.vWheelMax / _cfg.trackwidthMm;
        if (omega >  omegaMax) omega =  omegaMax;
        if (omega < -omegaMax) omega = -omegaMax;

        // Supervised MotionCommand with HEADING + TIME stops (sprint 024-001).
        // HEADING fires when bearing is within gateRad → transition to PURSUE.
        // TIME net bounds the spin against frozen OTOS/encoder runaway.
        float bearingSigned = atan2f(ty, tx);   // signed angle, robot frame, (-π, π]
        float nominalMs = (fabsf(omega) > 1e-3f)
                          ? (fabsf(bearingSigned) / fabsf(omega)) * 1000.0f
                          : 0.0f;
        float timeoutMs = 2.0f * nominalMs + 2000.0f;

        // Cancel any stale MotionCommand before configuring PRE_ROTATE.
        // If a TURN (or prior G) is still active, cancel(HARD) emits its
        // cancellation EVT via the stored reply sink, then goes IDLE.
        // configure() below clears the stale reply sink — no use-after-free.
        if (_activeCmd.active()) {
            _activeCmd.cancel(MotionCommand::StopStyle::HARD);
        }

        // Configure PRE_ROTATE as a supervised spin-in-place command.
        // No reply sink: PRE_ROTATE does NOT emit "EVT done G" on HEADING success
        // (the transition to PURSUE happens in driveAdvance without a wire event).
        // On TIME net expiry (runaway), driveAdvance emits "EVT done G" directly
        // via target.replyFn so the caller gets a clean terminal event.
        _activeCmd.configure(0.0f, omega, &_bvc);
        _activeCmd.addStop(makeHeadingStop(bearingSigned, gateRad));
        _activeCmd.addStop(makeTimeStop(timeoutMs));
        // No setReplySink: EVT emission is handled by driveAdvance on termination.
        _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
        // BVC ramps under yawAccMax — no instant 180°/s seed (seedCurrent removed).
        HardwareState emptyState{};
        const HardwareState& hwInputs = _hwState ? *_hwState : emptyState;
        _activeCmd.start(hwInputs, now_ms);
        _gPhase = GPhase::PRE_ROTATE;
    } else {
        // Target is roughly ahead — enter pursuit directly.
        // Configure MotionCommand with a POSITION stop at the world target.
        // The per-tick pursuit hook in driveAdvance will update the (v, ω) target
        // each tick before _activeCmd.tick() is called.
        //
        // PURSUE TIME net (sprint 024-001): bound the end-to-end drive so G is
        // not the only motion verb without a TIME backstop.
        float distanceMm = sqrtf(tx * tx + ty * ty);
        float pursueSpd  = (_gSpeed > 1.0f) ? _gSpeed : 1.0f;
        float pursueTimeoutMs = 2.0f * (distanceMm / pursueSpd) * 1000.0f + 4000.0f;

        // Cancel any stale MotionCommand before configuring PURSUE.
        if (_activeCmd.active()) {
            _activeCmd.cancel(MotionCommand::StopStyle::HARD);
        }

        _activeCmd.configure(_gSpeed, 0.0f, &_bvc);
        _activeCmd.addStop(makePositionStop(_gTargetXWorld, _gTargetYWorld, _cfg.arriveTolMm));
        _activeCmd.addStop(makeTimeStop(pursueTimeoutMs));
        _activeCmd.setReplySink(fn, ctx, corr_id);
        _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
        _activeCmd.setDoneEvt("EVT done G");
        HardwareState emptyState{};
        const HardwareState& inputs = _hwState ? *_hwState : emptyState;
        _activeCmd.start(inputs, now_ms);
        _gPhase = GPhase::PURSUE;
    }
}

void MotionController::beginTurn(float headingCdeg, float epsCdeg, uint32_t now_ms,
                                TargetState& target, ReplyFn fn, void* ctx,
                                const char* corr_id)
{
    // Convert centidegrees → radians for the absolute target heading.
    // 1 cdeg = π/18000 rad (same conversion as getPoseFloat uses for poseHrad).
    const float kCdegToRad = 3.14159265f / 18000.0f;
    float theta_rad = headingCdeg * kCdegToRad;
    float eps_rad   = epsCdeg   * kCdegToRad;

    // Read current heading from HardwareState (in radians, via poseHrad).
    // poseHrad is stored as a float in radians in HardwareState (set by Odometry).
    float currentHeadingRad = 0.0f;
    if (_hwState != nullptr) {
        currentHeadingRad = _hwState->poseHrad;
    }

    // Compute shortest-path delta: wrap_angle gives the signed angle in (-π, π].
    // delta > 0 ⇒ CCW (positive ω); delta < 0 ⇒ CW (negative ω).
    // Use inline atan2f(sinf, cosf) pattern matching StopCondition.cpp::wrap_angle.
    float diff       = theta_rad - currentHeadingRad;
    float delta_rad  = atan2f(sinf(diff), cosf(diff));   // wrap to (-π, π]
    float omega_sign = (delta_rad >= 0.0f) ? 1.0f : -1.0f;

    // ω magnitude from yawRateMax (deg/s → rad/s).
    const float kDegToRad = 3.14159265f / 180.0f;
    float omega = omega_sign * _cfg.yawRateMax * kDegToRad;

    // HEADING stop uses a delta from the baseline heading captured at start().
    // The baseline is heading0Rad = currentHeadingRad at start() time.
    // makeHeadingStop(delta_rad, eps_rad) stores delta_rad as 'a' and eps_rad as 'b'.
    // evaluate() checks: |wrap(current - heading0 - a)| < b
    //   = |wrap((currentHeadingRad + delta_rad) - currentHeadingRad - delta_rad)| < eps
    //   = |wrap(0)| < eps → fires when robot has rotated by delta_rad from baseline.
    // This matches the absolute target theta_rad exactly (since delta_rad = theta_rad - baseline).

    // Cancel any stale MotionCommand before configuring the new one.
    if (_activeCmd.active()) {
        _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    }

    // Re-arm safety if SAFE off was issued (one-shot disable, sprint 024-003).
    _checkSafeOneShot(fn, ctx);

    // Configure a fresh MotionCommand with:
    //   - target twist (0, ω): spin-in-place.
    //   - HEADING stop condition.
    //   - SOFT stop style (BVC ramps ω down before completion).
    //   - EVT "EVT done TURN" on arrival.
    //   - Reply sink for async EVT delivery.
    _activeCmd.configure(0.0f, omega, &_bvc);
    _activeCmd.addStop(makeHeadingStop(delta_rad, eps_rad));
    // Safety time-out net (mirrors beginDistance): a TURN must NEVER run away if
    // the HEADING stop never fires — e.g. odometry heading not advancing because
    // encoders are frozen, or the robot physically cannot reach the target. Bound
    // the turn to ~2x its nominal duration plus 2 s of ramp/settle headroom so a
    // stuck heading produces a clean EVT done instead of an unbounded spin.
    float nominalMs = (fabsf(omega) > 1e-3f)
                      ? (fabsf(delta_rad) / fabsf(omega)) * 1000.0f
                      : 0.0f;
    float timeoutMs = 2.0f * nominalMs + 2000.0f;
    _activeCmd.addStop(makeTimeStop(timeoutMs));
    _activeCmd.setReplySink(fn, ctx, corr_id);
    _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
    _activeCmd.setDoneEvt("EVT done TURN");

    // Snapshot hardware state for MotionBaseline (captures heading0Rad at start time).
    HardwareState emptyState{};
    const HardwareState& inputs = _hwState ? *_hwState : emptyState;
    _activeCmd.start(inputs, now_ms);

    // VELOCITY mode — distinct from STREAMING so S-mode watchdog does not fire.
    _mode = DriveMode::VELOCITY;

    // Update target mode; reply sink captured by _activeCmd (not target.replyFn).
    target.mode = DriveMode::VELOCITY;
}

// ---------------------------------------------------------------------------
// beginRotation — RELATIVE spin-in-place, stopped on ENCODER ARC.
//
// Unlike beginTurn (which stops on heading odometry), this stops purely on the
// encoder differential — the geometry-verified arc for the requested angle —
// so it does not depend on poseHrad/OTOS at all. A tight TIME stop bounds the
// spin so a frozen encoder read can never run away.
// ---------------------------------------------------------------------------
void MotionController::beginRotation(float relCdeg, uint32_t now_ms,
                                     TargetState& target, ReplyFn fn, void* ctx,
                                     const char* corr_id)
{
    const float kDegToRad = 3.14159265f / 180.0f;
    // Spin rate for RT — deliberately moderate (not yawRateMax) so the SOFT
    // ramp-down coasts little. Coast-anticipation (kRtCoastArcMm) fires the
    // ROTATION stop early so the ramp lands on target. Both tuned in sim.
    const float kRtRateDps    = 100.0f;
    const float kRtCoastArcMm = 8.0f;   // ~7.3° SOFT-ramp coast at 100°/s (sim-tuned)

    float tw   = _cfg.trackwidthMm;
    // Per-wheel arc = |deg|·(π/180)·(trackwidth/2) / slip.
    // Dividing by slip (< 1.0) enlarges the encoder-arc target so wheels travel
    // far enough for the body to reach the commanded angle despite scrub.
    // effectiveSlip() applies the same migration-safe clamp as Odometry::predict().
    float slip = effectiveSlip(_cfg.rotationalSlip);
    float arc  = fabsf(relCdeg) / 100.0f * kDegToRad * (tw * 0.5f) / slip;
    float stopArc = arc - kRtCoastArcMm;
    if (stopArc < 0.0f) stopArc = 0.0f;
    float rateDps = (_cfg.yawRateMax < kRtRateDps) ? _cfg.yawRateMax : kRtRateDps;
    float omega_sign = (relCdeg >= 0.0f) ? 1.0f : -1.0f;   // + ⇒ CCW
    float omega = omega_sign * rateDps * kDegToRad;        // rad/s

    // Cancel any stale MotionCommand before configuring the new one.
    if (_activeCmd.active()) {
        _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    }

    // Re-arm safety if SAFE off was issued (one-shot disable, sprint 024-003).
    _checkSafeOneShot(fn, ctx);

    _activeCmd.configure(0.0f, omega, &_bvc);
    _activeCmd.addStop(makeRotationStop(stopArc));         // primary: encoder arc

    // Tight time bound (runaway guard): nominal spin time = arc / wheel-linear-
    // speed (|omega|·tw/2), plus headroom for ramp + coast.
    float wheelSpeed = fabsf(omega) * (tw * 0.5f);          // mm/s
    float nominalMs  = (wheelSpeed > 1e-3f) ? (arc / wheelSpeed) * 1000.0f : 0.0f;
    float timeoutMs  = 2.0f * nominalMs + 1000.0f;
    _activeCmd.addStop(makeTimeStop(timeoutMs));

    _activeCmd.setReplySink(fn, ctx, corr_id);
    // SOFT stop: ramp ω to 0 (this is what actually halts the motors; HARD
    // leaves _active=false so driveAdvance stops feeding the BVC and the wheels
    // coast on). Coast-anticipation above compensates for the ramp arc.
    _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
    _activeCmd.setDoneEvt("EVT done RT");

    HardwareState emptyState{};
    const HardwareState& inputs = _hwState ? *_hwState : emptyState;
    _activeCmd.start(inputs, now_ms);

    _mode = DriveMode::VELOCITY;
    target.mode = DriveMode::VELOCITY;
}

void MotionController::stop(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    // Cancel any active MotionCommand before calling fullStop().
    if (_activeCmd.active()) {
        _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    }
    _gPhase = GPhase::IDLE;  // reset G phase on hard stop
    fullStop(fn, ctx);
    (void)now_ms;
}

void MotionController::cancel(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    _mc.stop();
    _mode   = DriveMode::IDLE;
    _gPhase = GPhase::IDLE;  // reset G phase on any cancel
    (void)now_ms;
    (void)fn;
    (void)ctx;
}

void MotionController::disableSafetyOneShot()
{
    _safeOneShotDisable = true;
}

void MotionController::softStop(uint32_t now_ms)
{
    if (_activeCmd.active()) {
        // Active MotionCommand: arm its SOFT ramp-down.
        // tick() will advance BVC toward (0,0) and emit EVT done when converged.
        _activeCmd.softStop(now_ms);
    } else {
        // No active MotionCommand (STREAMING or IDLE mode): just set BVC target
        // to (0,0) and let the profiler ramp down.  No EVT done in this case.
        _bvc.setTarget(0.0f, 0.0f);
    }
}

void MotionController::beginRawVelocity(float v_mms, float omega_rads)
{
    // Seed the profiler's current state to the target — no ramp from zero.
    // Then set the target so advance() holds at this speed immediately.
    _bvc.seedCurrent(v_mms, omega_rads);
    _bvc.setTarget(v_mms, omega_rads);

    // _VW is fire-and-forget: no MotionCommand, no stop conditions.
    // The system watchdog in LoopScheduler owns keepalive enforcement.
    // STREAMING mode: the BVC-tick path in driveAdvance will advance the profiler.
    _mode = DriveMode::STREAMING;
}

// ---------------------------------------------------------------------------
// driveAdvance — cooperative-loop task entry point (014-005).
//
// Runs at a fixed period set by RobotConfig::controlPeriodMs (default 10 ms).
// Executes the drive-mode state machines:
//   1. STREAMING watchdog — emits EVT safety_stop inline on keepalive timeout.
//   2. G-mode — advances PRE_ROTATE and PURSUE; emits EVT done G inline.
//   T/D-mode are now handled by the MotionCommand path (TIME/DISTANCE stop conditions).
//
// All EVT completions are emitted inline via target.sink.emitFn() (sprint 026-002)
// — safe because there is no fiber boundary in the single cooperative main loop.
//
// NOTE: OTOS correction is NOT done here.  It is the sole responsibility of
// Robot::otosCorrect() called at the slow cadence in LoopScheduler
// (ticket 005 wired this; ticket 006 moved it to the scheduler task).
// ---------------------------------------------------------------------------

void MotionController::driveAdvance(HardwareState& inputs, MotorCommands& cmds,
                                    TargetState& target, uint32_t now_ms)
{
    // Throttle to controlPeriodMs cadence.
    if ((now_ms - _lastTickMs) < (uint32_t)_cfg.controlPeriodMs) return;

    float dt_s      = (float)(now_ms - _lastTickMs) / 1000.0f;
    _lastTickMs     = now_ms;
    _currentTimeMs  = now_ms;

    // Motor controller tick and odometry predict are called by Robot::controlCollectSplitPhase()
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

        // D decel hook: clamp commanded speed downward as the robot nears
        // the distance target.  Computes d_remaining from the raw encoder
        // average (same field used by the DISTANCE stop condition in
        // StopCondition::evaluate) so the decel profile and the stop fire
        // at the same point.  Only clamps downward; does not increase speed.
        if (_mode == DriveMode::DISTANCE) {
            float enc_avg     = (inputs.encLMm + inputs.encRMm) * 0.5f;
            float d_traveled  = fabsf(enc_avg - _dEnc0);
            float d_remaining = _dDistTarget - d_traveled;
            if (d_remaining > 0.0f) {
                float v_cap = sqrtf(2.0f * _cfg.aDecel * d_remaining);
                if (v_cap < _bvc.targetV()) {
                    _activeCmd.setTarget(v_cap, _dOmega);
                }
            }
        }

        bool still_running = _activeCmd.tick(inputs, now_ms, dt_s);
        if (!still_running) {
            // MotionCommand terminated.
            //
            // PRE_ROTATE special case (sprint 024-001): when the PRE_ROTATE
            // MotionCommand finishes, check the current bearing.
            //   - bearing <= gateRad (HEADING stop fired) → transition to PURSUE.
            //   - bearing >  gateRad (TIME net fired)     → runaway; emit "EVT done G"
            //     and go IDLE so the caller gets a clean terminal event.
            if (_mode == DriveMode::GO_TO && _gPhase == GPhase::PRE_ROTATE) {
                float x, y, h_rad;
                getPoseFloat(x, y, h_rad);
                float dxW   = _gTargetXWorld - x;
                float dyW   = _gTargetYWorld - y;
                float dx_rf =  dxW * cosf(h_rad) + dyW * sinf(h_rad);
                float dy_rf = -dxW * sinf(h_rad) + dyW * cosf(h_rad);
                float bearingNow = fabsf(atan2f(dy_rf, dx_rf));
                float gateRad    = _cfg.turnInPlaceGate * (3.14159265f / 180.0f);

                if (bearingNow <= gateRad) {
                    // HEADING stop fired → start the PURSUE phase.
                    // Compute distance for the PURSUE TIME net.
                    float distanceMm     = sqrtf(dxW * dxW + dyW * dyW);
                    float pursueSpd      = (_gSpeed > 1.0f) ? _gSpeed : 1.0f;
                    float pursueTimeoutMs = 2.0f * (distanceMm / pursueSpd) * 1000.0f + 4000.0f;

                    _bvc.reset();
                    _activeCmd.configure(_gSpeed, 0.0f, &_bvc);
                    _activeCmd.addStop(makePositionStop(_gTargetXWorld, _gTargetYWorld,
                                                       _cfg.arriveTolMm));
                    _activeCmd.addStop(makeTimeStop(pursueTimeoutMs));
                    _activeCmd.setReplySink(target.replyFn, target.replyCtx, target.corrId);
                    _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
                    _activeCmd.setDoneEvt("EVT done G");
                    const HardwareState& hw = _hwState ? *_hwState : inputs;
                    _activeCmd.start(hw, now_ms);
                    _gPhase = GPhase::PURSUE;
                    // Do NOT go IDLE; PURSUE is now active.
                    return;
                } else {
                    // TIME net fired (runaway spin): emit terminal EVT and go IDLE.
                    // _activeCmd had no reply sink, so we emit directly here.
                    emitEvt("EVT done G", target);
                    _mc.stop();
                    _bvc.reset();
                    _mode = DriveMode::IDLE;
                    target.mode = DriveMode::IDLE;
                    _gPhase = GPhase::IDLE;
                    return;
                }
            }

            // Normal completion (non-PRE_ROTATE): stop motors, reset, go IDLE.
            // Without this the last BVC wheel target persists and the motor PID
            // keeps driving it forever (runaway), since IDLE mode no longer
            // advances the BVC to write fresh (zero) setpoints. _mc.stop() zeros
            // tgtLMms/tgtRMms and resets the PID, so driving=false next tick.
            _mc.stop();
            _bvc.reset();
            _mode = DriveMode::IDLE;
            target.mode = DriveMode::IDLE;
            // Reset G phase so a subsequent go-to command starts clean.
            if (_gPhase != GPhase::IDLE) _gPhase = GPhase::IDLE;
        }
        return;
    }

    // S-mode keepalive watchdog has been removed from driveAdvance.
    // The system watchdog in LoopScheduler now handles keepalive enforcement for
    // all modes (STREAMING, VELOCITY, etc.) — it fires EVT safety_stop + X after
    // sTimeoutMs of inbound command silence (Sprint 020, Ticket 005).
    (void)inputs;

    // ── BVC tick for STREAMING mode ─────────────────────────────────────────
    // STREAMING mode sets BVC targets but does not have an active MotionCommand
    // to call _bvc.advance(). Tick the BVC directly here so the profiler
    // advances and wheel setpoints are written every control period.
    //
    // PRE_ROTATE is no longer ticked here (sprint 024-001): it now runs via
    // _activeCmd (which calls _bvc.advance() internally in tick()), so the
    // PRE_ROTATE branch was removed to prevent double-ticking the BVC.
    if (_mode == DriveMode::STREAMING) {
        _bvc.advance(dt_s);
    }

    // G-mode: no additional state-machine branches needed here.
    // PRE_ROTATE now runs via _activeCmd (handled in the block above).
    // PURSUE runs via _activeCmd (handled in the block above).
    // Both phases are driven entirely by the MotionCommand path at the top
    // of driveAdvance — control never reaches here while GO_TO is active
    // with a running _activeCmd.
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void MotionController::fullStop(ReplyFn fn, void* ctx)
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
void MotionController::getPoseFloat(float& x, float& y, float& h_rad) const {
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
