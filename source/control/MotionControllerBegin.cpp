// MotionControllerBegin.cpp — MotionController begin*() command entry points.
//
// Split from MotionController.cpp (finding A3): pure translation-unit move of
// the begin* family of member-function DEFINITIONS.  These remain
// MotionController:: members declared in MotionController.h — no header,
// signature, or logic change.
//
// Contains:
//   _checkSafeOneShot, beginStream, beginVelocity, beginArc, beginTimed,
//   beginDistance, beginGoTo, beginTurn, beginRotation, beginRawVelocity,
//   _startPreRotate
//
// driveAdvance (per-tick pursuit/advance machinery), stop, cancel,
// disableSafetyOneShot, softStop, fullStop, getPoseFloat, emitEvt, and the
// constructor stay in MotionController.cpp.  Member functions link across
// translation units, so e.g. beginGoTo calling getPoseFloat (kept) and
// _startPreRotate (here), and driveAdvance (kept) calling _startPreRotate, are
// all fine.
//
// localEvtEmitter (file-local static, the default MotionEventSink::emitFn) is
// referenced only by beginGoTo, so it travels here with that body.  The static
// member emitEvt — used by the kept driveAdvance — stays in MotionController.cpp.

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
// localEvtEmitter — default MotionEventSink::emitFn for TargetState-based
// EVT delivery.
//
// ctx is a TargetState* (set to &target by beginGoTo when wiring the sink).
// Formats "<base> #<corrId>" and calls target->replyFn(msg, target->replyCtx).
//
// Lives in the control layer so no app-layer includes are needed — only
// TargetState (RobotState.h, also control layer).
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
    // Cancel-if-active: emit EVT cancelled for the preempted command's corrId
    // before seeding the BVC.  Contract: an explicit S command (stream=1 path)
    // cancels any running self-terminating command (TURN/G/T/D/R/RT) and takes
    // over streaming.  This matches the cancel-then-proceed pattern used by all
    // other begin*() entry points (beginVelocity, beginArc, beginTurn, etc.).
    //
    // N4 fix (sprint 030-004): without this guard an S during TURN/G/T/D leaves
    // _activeCmd running — the old command's TIME/HEADING stop later fires,
    // soft-stops the robot, and emits a stale EVT done, silently killing the
    // stream.  The old command never gets EVT cancelled (P1.1 failure mode).
    //
    // D6 keepalive note: the D6 origin guard in handleVW already intercepts
    // plain VW keepalives and sends them to setTarget() (or busy-replies for
    // non-VW origins) BEFORE they can reach beginStream().  Only an explicit S
    // command (stream=1 marker) bypasses the origin guard and arrives here.
    // So this cancel fires only for genuine preemption, not for keepalives.
    if (_activeCmd.active()) {
        _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    }

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
    _activeCmd.setOrigin(MotionCommand::Origin::VW);
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
    _activeCmd.setOrigin(MotionCommand::Origin::R);
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

    // Cancel-if-active: emit EVT cancelled for the preempted command's corrId
    // before configuring the new T command.  Without this a host awaiting
    // "EVT done G" that issues a T will never receive any terminal event for the
    // G — the previous command's reply sink is silently reset by configure().
    // N5 fix (sprint 030-004): uniform cancel-if-active across all begin*().
    if (_activeCmd.active()) {
        _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    }

    // Re-arm safety if SAFE off was issued (one-shot disable, sprint 024-003).
    _checkSafeOneShot(fn, ctx);

    // Configure a fresh MotionCommand with:
    //   - TIME stop condition at durationMs.
    //   - SOFT stop style (ramp to zero before completing).
    //   - EVT "EVT done T" on completion (preserves wire contract).
    //   - Reply sink for async EVT delivery.
    _activeCmd.configure(v_mms, omega_rads, &_bvc);
    _activeCmd.setOrigin(MotionCommand::Origin::T);
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

    // Cancel-if-active: emit EVT cancelled for the preempted command's corrId
    // before resetting encoders or configuring the new D command.  Cancel comes
    // first so the old command's terminal event is emitted with its own corrId
    // before the new command takes over the reply sink.  N5 fix (sprint 030-004).
    if (_activeCmd.active()) {
        _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    }

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
    _activeCmd.setOrigin(MotionCommand::Origin::D);
    _activeCmd.addStop(makeDistanceStop((float)targetMm));
    _activeCmd.addStop(makeTimeStop(timeoutMs));
    _activeCmd.setReplySink(fn, ctx, corr_id);
    _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
    _activeCmd.setDoneEvt("EVT done D");

    // Zero the software encoder mirror BEFORE snapshotting the baseline so that
    // enc0 starts from 0, matching the freshly-reset hardware accumulators.
    //
    // 033-004 baseline-race fix: previously this zeroing happened only in
    // Robot::distanceDrive() AFTER beginDistance() returned, so MotionCommand::
    // start() below captured the PREVIOUS command's stale encoder average as
    // enc0Mm.  A D following a TURN (with no ZERO enc between) could then
    // instant-complete on the first evaluate: once distanceDrive() zeroed the
    // mirror, traveled = |0 − staleEnc0| already exceeded targetMm.  Zeroing the
    // mirror here makes enc0Mm and encDiff0Mm both 0 regardless of call order.
    // Robot::distanceDrive() still calls resetEncoders() after this (re-zeroing
    // the mirror plus resetting velocity baselines and the odometry snapshot).
    if (_hwState) {
        _hwState->encLMm = 0;
        _hwState->encRMm = 0;
    }

    // Snapshot hardware state for MotionBaseline.  The encoder mirror was just
    // zeroed above and the hardware accumulators were reset at the top of this
    // function, so the baseline enc0/encDiff0 captured by MotionCommand::start()
    // are both 0 — matching the DISTANCE stop evaluation which reads
    // (encLMm + encRMm)/2 from HardwareState.
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
        // Cancel any stale MotionCommand before configuring PRE_ROTATE.
        if (_activeCmd.active()) {
            _activeCmd.cancel(MotionCommand::StopStyle::HARD);
        }
        float bearingSigned = atan2f(ty, tx);   // signed angle, robot frame, (-π, π]
        _startPreRotate(bearingSigned, _gSpeed, now_ms, target);
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
        _activeCmd.setOrigin(MotionCommand::Origin::G);
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
    _activeCmd.setOrigin(MotionCommand::Origin::TURN);
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
    _activeCmd.setOrigin(MotionCommand::Origin::RT);
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

void MotionController::beginRawVelocity(float v_mms, float omega_rads)
{
    // Cancel-if-active: emit EVT cancelled for the preempted command's corrId
    // before seeding the BVC.  _VW (raw velocity, fire-and-forget) must not
    // leave a zombie MotionCommand that will later soft-stop the robot and emit
    // a stale EVT done.  N4 fix (sprint 030-004): uniform cancel-if-active
    // across all begin*() entry points.
    if (_activeCmd.active()) {
        _activeCmd.cancel(MotionCommand::StopStyle::HARD);
    }

    // Seed the profiler's current state to the target — no ramp from zero.
    // Then set the target so advance() holds at this speed immediately.
    _bvc.seedCurrent(v_mms, omega_rads);
    _bvc.setTarget(v_mms, omega_rads);

    // _VW is fire-and-forget: no MotionCommand, no stop conditions.
    // The system watchdog in LoopScheduler owns keepalive enforcement.
    // STREAMING mode: the BVC-tick path in driveAdvance will advance the profiler.
    _mode = DriveMode::STREAMING;
}

// _startPreRotate — configure a supervised PRE_ROTATE MotionCommand.
//
// Extracted as a shared helper (D8 027-004) so both beginGoTo()'s PRE_ROTATE
// branch and the PURSUE re-gate use identical setup logic without duplication.
//
// bearingRad: signed robot-frame bearing to the target, atan2f(dy, dx).
//             Determines turn direction (CCW if > 0, CW if < 0).
// speed:      commanded travel speed (mm/s) — used to derive omega via
//             inverse kinematics (spin-in-place).
// now_ms:     current timestamp; passed to MotionCommand.start().
// target:     TargetState with replyFn/replyCtx/corrId already wired by
//             beginGoTo().  The PRE_ROTATE command does NOT set a reply sink;
//             driveAdvance handles the PRE_ROTATE → PURSUE transition and emits
//             "EVT done G" on TIME-net expiry directly via target.replyFn.
void MotionController::_startPreRotate(float bearingRad, float speed,
                                        uint32_t now_ms, TargetState& target)
{
    float gateRad = _cfg.turnInPlaceGate * (3.14159265f / 180.0f);

    // Per-direction feedforward gain (012-006): CCW uses rotationGainPos,
    // CW uses rotationGainNeg.  FEEDFORWARD correction only; bearing gate
    // provides closed-loop compensation (no oscillation risk).
    float turnSign = (bearingRad >= 0.0f) ? 1.0f : -1.0f;
    float dirGain  = (turnSign > 0.0f) ? _cfg.rotationGainPos : _cfg.rotationGainNeg;
    if (dirGain < 0.05f) dirGain = 0.05f;

    // omega = 2*(speed/dirGain) / trackwidth  (spin-in-place from inverse kinematics)
    float wheelSpd = speed / dirGain;
    float omega    = turnSign * 2.0f * wheelSpd / _cfg.trackwidthMm;
    float omegaMax = 2.0f * _cfg.vWheelMax / _cfg.trackwidthMm;
    if (omega >  omegaMax) omega =  omegaMax;
    if (omega < -omegaMax) omega = -omegaMax;

    // TIME net: 2× nominal spin time + 2000 ms guard (sprint 024-001).
    float nominalMs = (fabsf(omega) > 1e-3f)
                      ? (fabsf(bearingRad) / fabsf(omega)) * 1000.0f
                      : 0.0f;
    float timeoutMs = 2.0f * nominalMs + 2000.0f;

    // Configure PRE_ROTATE as a supervised spin-in-place command.
    // No reply sink: PRE_ROTATE does NOT emit "EVT done G" on HEADING success
    // (the transition to PURSUE happens in driveAdvance without a wire event).
    // On TIME net expiry (runaway), driveAdvance emits "EVT done G" directly
    // via target.replyFn so the caller gets a clean terminal event.
    _bvc.reset();
    _activeCmd.configure(0.0f, omega, &_bvc);
    _activeCmd.setOrigin(MotionCommand::Origin::G);
    _activeCmd.addStop(makeHeadingStop(bearingRad, gateRad));
    _activeCmd.addStop(makeTimeStop(timeoutMs));
    // No setReplySink: EVT emission is handled by driveAdvance on termination.
    _activeCmd.setStopStyle(MotionCommand::StopStyle::SOFT);
    HardwareState emptyState{};
    const HardwareState& hwInputs = _hwState ? *_hwState : emptyState;
    _activeCmd.start(hwInputs, now_ms);
    _gPhase = GPhase::PRE_ROTATE;

    (void)target;  // target.replyFn used by driveAdvance, not by this helper directly
}
