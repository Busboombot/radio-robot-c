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
// Each converter builds a ParsedCommand targeting the VW descriptor and calls
// queue->push_front(). VW's handler extended to read stop params (t, dist, x,
// y, h) from ArgList args[2..] encoded as ArgType::STR "key=value" strings.
// handleOP in Odometry.cpp refactored to read from hwState->otosX/Y/H.

#include "MotionController.h"
#include "MotorController.h"
#include "Odometry.h"
#include "BodyKinematics.h"
#include "StopCondition.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "CommandQueue.h"
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
    , _ctx{this, nullptr, nullptr, {}}   // robot/queue/vwDesc set later by setCtx()
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
// Forward declarations for static handler functions used by setCtx().
// Full definitions appear later in this file (getCommands section).
// ---------------------------------------------------------------------------
static ParseResult parseVW(const char* const* tokens, int ntokens,
                           const KVPair* kvs, int nkv);
static void handleVW(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx);

// ---------------------------------------------------------------------------
// setCtx — bind the Robot* for Commandable handlers and initialise vwDesc.
//
// vwDesc is a stable CommandDescriptor stored inside _ctx so that converter
// handlers (S, T, D, G, R, TURN) can build a ParsedCommand with pc.desc =
// &ctx->vwDesc and call queue->push_front(pc). dequeueOne() then invokes
// handleVW directly (bypassing the parse step since args are pre-built).
// ---------------------------------------------------------------------------
void MotionController::setCtx(struct Robot* r)
{
    _ctx.mc    = this;
    _ctx.robot = r;
    _ctx.queue = nullptr;  // set later by setQueue() from LoopScheduler

    // Initialise the stable VW descriptor that converter handlers reference.
    _ctx.vwDesc.prefix     = "VW";
    _ctx.vwDesc.parseFn    = parseVW;     // not called when args are pre-built
    _ctx.vwDesc.handlerFn  = handleVW;
    _ctx.vwDesc.handlerCtx = &_ctx;       // points into _ctx itself (stable for lifetime of Robot)
    _ctx.vwDesc.errFmt     = "badarg";
    _ctx.vwDesc.forceReply = ForceReply::NONE;
    _ctx.vwDesc.flags      = CMD_ACCESS_HARDWARE;
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

/*static*/ void MotionController::emitEvt(const char* base, TargetState& target)
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

void MotionController::beginStream(float leftMms, float rightMms, uint32_t now_ms,
                                   TargetState& target, ReplyFn fn, void* ctx)
{
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
}

void MotionController::beginVelocity(float v_mms, float omega_rads, uint32_t now_ms,
                                     TargetState& target, ReplyFn fn, void* ctx,
                                     const char* corr_id)
{
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
        // Compute ω from per-direction gain: spin-in-place at (_gSpeed / dirGain)
        // wheel speed.  ω = vWheel_tangential / (trackwidth/2) = (speed/dirGain) / (b/2).
        // More precisely: vR-vL = 2*omega*(b/2) = omega*b, so omega = (vR-vL)/b.
        // With vR=+speed/dirGain and vL=-speed/dirGain: omega = 2*(speed/dirGain)/b.
        float wheelSpd = _gSpeed / dirGain;
        float omega    = turnSign * 2.0f * wheelSpd / _cfg.trackwidthMm;
        // Clamp omega magnitude to avoid overflow (mirrors vWheelMax saturation).
        // Maximum omega corresponds to vWheelMax at each wheel:
        // omega_max = 2*vWheelMax / trackwidthMm.
        float omegaMax = 2.0f * _cfg.vWheelMax / _cfg.trackwidthMm;
        if (omega >  omegaMax) omega =  omegaMax;
        if (omega < -omegaMax) omega = -omegaMax;
        _bvc.seedCurrent(0.0f, omega);
        _bvc.setTarget(0.0f, omega);
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
// All EVT completions are emitted inline via target.replyFn() — safe because
// there is no fiber boundary in the single cooperative main loop (014-005).
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
            // MotionCommand terminated; go IDLE.
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

    // ── BVC tick for STREAMING and PRE_ROTATE modes ─────────────────────────
    // These modes set BVC targets but do not have an active MotionCommand to
    // call _bvc.advance(). Tick the BVC directly here so the profiler advances
    // and wheel setpoints are written every control period.
    if (_mode == DriveMode::STREAMING ||
        (_mode == DriveMode::GO_TO && _gPhase == GPhase::PRE_ROTATE)) {
        _bvc.advance(dt_s);
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

// ---------------------------------------------------------------------------
// parseSensorToken — parse "sensor=<ch>:<op>:<thr>" into channel, cmp, threshold.
//
// Duplicated from CommandProcessor.cpp (static function; not exported).
// Will be deduplicated once a shared SensorToken helper is extracted.
//
// Returns true on success; false on any parse/lookup failure.
// ---------------------------------------------------------------------------

static bool mc_parseSensorToken(const char* value,
                                uint8_t& ch_out, float& thr_out,
                                StopCondition::Cmp& cmp_out)
{
    char buf[32];
    int vlen = 0;
    for (const char* p = value; *p && vlen < (int)sizeof(buf) - 1; ++p, ++vlen)
        buf[vlen] = *p;
    buf[vlen] = '\0';

    char* colon1 = strchr(buf, ':');
    if (!colon1) return false;
    *colon1 = '\0';
    const char* ch_name = buf;
    const char* rest    = colon1 + 1;

    char* colon2 = strchr(const_cast<char*>(rest), ':');
    if (!colon2) return false;
    *colon2 = '\0';
    const char* op_str  = rest;
    const char* thr_str = colon2 + 1;

    uint8_t ch = 0;
    bool found = false;
    struct { const char* name; uint8_t idx; } chMap[] = {
        { "line0",  0 }, { "line1",  1 }, { "line2",  2 }, { "line3",  3 },
        { "colorR", 4 }, { "colorG", 5 }, { "colorB", 6 }, { "colorC", 7 },
    };
    for (int i = 0; i < 8; ++i) {
        if (strcmp(ch_name, chMap[i].name) == 0) {
            ch    = chMap[i].idx;
            found = true;
            break;
        }
    }
    if (!found) return false;

    StopCondition::Cmp cmp;
    if (strcmp(op_str, "ge") == 0) {
        cmp = StopCondition::Cmp::GE;
    } else if (strcmp(op_str, "le") == 0) {
        cmp = StopCondition::Cmp::LE;
    } else {
        return false;
    }

    int thr = atoi(thr_str);
    ch_out  = ch;
    thr_out = (float)thr;
    cmp_out = cmp;
    return true;
}

// ---------------------------------------------------------------------------
// getCommands — Commandable interface.  Returns descriptors for all 9 motion
// commands: S, T, D, G, R, TURN, VW, X, STOP.
//
// Handler context (_ctx) is a MotionCtx{this, robot} populated by setCtx().
// All handlers cast handlerCtx to MotionCtx*.
//
// Argument packing conventions (mirror the old switch cases in
// CommandProcessor.cpp exactly so EVT async completions are equivalent):
//   S    — args[0].ival=l, args[1].ival=r
//   T    — args[0].ival=l, args[1].ival=r, args[2].ival=ms
//          args[3] present (type STR, sval="sensor=<tok>") if sensor= given
//   D    — args[0].ival=l, args[1].ival=r, args[2].ival=mm
//          args[3] present (type STR) if sensor= given
//   G    — args[0].ival=x, args[1].ival=y, args[2].ival=speed
//   R    — args[0].ival=speed, args[1].ival=radius
//   TURN — args[0].ival=heading_cdeg, args[1].ival=eps_cdeg
//   VW   — args[0].ival=v, args[1].ival=omega (mrad/s)
//   X    — no args
//   STOP — no args
// ---------------------------------------------------------------------------

// ── Helper macro: set one INT arg ───────────────────────────────────────────
// Avoids repeating the three-field assignment pattern for every arg slot.
// Sets .type = ArgType::INT, .ival = v, .sval[0] = '\0'.
static inline void setIntArg(Argument& a, int v)
{
    a.type    = ArgType::INT;
    a.ival    = v;
    a.sval[0] = '\0';
}

// ── Helper: copy a sensor= KV value string into args[idx] as STR ───────────
// Returns the new args.count (idx + 1) on success, or original count if not
// found.  Does NOT validate the sensor string — validation happens at handler
// time via mc_parseSensorToken().
static int packSensorArg(ArgList& out, int nextIdx,
                         const KVPair* kvs, int nkv)
{
    for (int i = 0; i < nkv; ++i) {
        if (kvs[i].key && strcmp(kvs[i].key, "sensor") == 0) {
            out.args[nextIdx].type = ArgType::STR;
            out.args[nextIdx].ival = 0;
            out.args[nextIdx].fval = 0.0f;
            int slen = 0;
            const char* src = kvs[i].value;
            while (*src && slen < (int)(sizeof(out.args[nextIdx].sval) - 1))
                out.args[nextIdx].sval[slen++] = *src++;
            out.args[nextIdx].sval[slen] = '\0';
            return nextIdx + 1;
        }
    }
    return nextIdx;  // no sensor= found
}

// ---------------------------------------------------------------------------
// pushVW — build a VW ParsedCommand and push_front it onto the queue.
//
// args must already contain args[0].ival=v (mm/s), args[1].ival=omega (mrad/s),
// and any stop params packed in args[2..] as ArgType::STR "key=value" strings.
//
// Returns false if the queue is null (caller falls back to direct begin*() call).
// ---------------------------------------------------------------------------
static bool pushVW(MotionCtx* ctx, const ArgList& args,
                   const char* corrId, ReplyFn replyFn, void* replyCtx)
{
    if (ctx->queue == nullptr) return false;
    ParsedCommand pc;
    pc.desc    = &ctx->vwDesc;
    pc.args    = args;
    pc.replyFn = replyFn;
    pc.replyCtx = replyCtx;
    int cidLen = 0;
    while (corrId && corrId[cidLen] != '\0' && cidLen < (int)(sizeof(pc.corrId) - 1)) {
        pc.corrId[cidLen] = corrId[cidLen];
        ++cidLen;
    }
    pc.corrId[cidLen] = '\0';
    return ctx->queue->push_front(pc);
}

// ── Helper: pack a "key=value" STR arg ─────────────────────────────────────
// Writes a key=value string into args[idx].sval as ArgType::STR.
// Returns idx + 1 (next slot).
static int packKVArg(ArgList& out, int idx, const char* key, int ival)
{
    out.args[idx].type = ArgType::STR;
    out.args[idx].ival = 0;
    out.args[idx].fval = 0.0f;
    snprintf(out.args[idx].sval, sizeof(out.args[idx].sval), "%s=%d", key, ival);
    return idx + 1;
}

// ── S ────────────────────────────────────────────────────────────────────────

static ParseResult parseS(const char* const* tokens, int ntokens,
                           const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int l = atoi(tokens[0]);
    int r = atoi(tokens[1]);
    if (l < -1000 || l > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "l"; return res;
    }
    if (r < -1000 || r > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "r"; return res;
    }
    res.ok = true;
    res.args.count = 2;
    setIntArg(res.args.args[0], l);
    setIntArg(res.args.args[1], r);
    return res;
}

// handleS — VW converter (no stop params → S/streaming mode via beginStream fallback).
//
// Computes (v, ω) from (l, r) via BodyKinematics::forward(), encodes as
// VW args[0]=v_mms, args[1]=omega_mrads (no stop params), and pushes to the
// queue. Falls back to beginStream() when queue is null (sim / unit test).
static void handleS(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int l = args.args[0].ival;
    int r = args.args[1].ival;

    if (ctx->queue != nullptr) {
        // Compute body twist via forward kinematics; encode as VW mrad/s integers.
        float v_mms, omega_rads;
        BodyKinematics::forward((float)l, (float)r, ctx->robot->config.trackwidthMm,
                                v_mms, omega_rads);
        int v_int     = (int)v_mms;
        int omega_int = (int)(omega_rads * 1000.0f);  // rad/s → mrad/s

        ArgList vwArgs;
        vwArgs.count = 2;  // no stop params → S/stream mode
        setIntArg(vwArgs.args[0], v_int);
        setIntArg(vwArgs.args[1], omega_int);

        pushVW(ctx, vwArgs, corrId, replyFn, replyCtx);
    } else {
        // Queue not available (sim): fall back to direct beginStream().
        ctx->mc->beginStream((float)l, (float)r,
                             ctx->robot->systemTime(),
                             ctx->robot->state.target,
                             replyFn, replyCtx);
    }

    char body[32];
    snprintf(body, sizeof(body), "l=%d r=%d", l, r);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ── T ────────────────────────────────────────────────────────────────────────

static ParseResult parseT(const char* const* tokens, int ntokens,
                           const KVPair* kvs, int nkv)
{
    ParseResult res;
    if (ntokens < 3) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int l  = atoi(tokens[0]);
    int r  = atoi(tokens[1]);
    int ms = atoi(tokens[2]);
    if (l < -1000 || l > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "l"; return res;
    }
    if (r < -1000 || r > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "r"; return res;
    }
    if (ms < 1 || ms > 30000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "ms"; return res;
    }
    res.ok = true;
    res.args.count = 3;
    setIntArg(res.args.args[0], l);
    setIntArg(res.args.args[1], r);
    setIntArg(res.args.args[2], ms);
    // Pack optional sensor= into args[3].
    res.args.count = packSensorArg(res.args, 3, kvs, nkv);
    return res;
}

// handleT — VW converter with t=<ms> stop param.
//
// Computes (v, ω) from (l, r) via forward kinematics, builds VW args with
// "t=<ms>" stop param, and pushes to the queue. Falls back to direct
// beginTimed() when queue is null (sim / unit test).
// Note: the optional sensor= stop condition is NOT forwarded via the queue
// path in this sprint; it is applied only on the direct path.
static void handleT(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int l  = args.args[0].ival;
    int r  = args.args[1].ival;
    int ms = args.args[2].ival;

    if (ctx->queue != nullptr) {
        float v_mms, omega_rads;
        BodyKinematics::forward((float)l, (float)r, ctx->robot->config.trackwidthMm,
                                v_mms, omega_rads);
        int v_int     = (int)v_mms;
        int omega_int = (int)(omega_rads * 1000.0f);

        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], v_int);
        setIntArg(vwArgs.args[1], omega_int);
        vwArgs.count = packKVArg(vwArgs, 2, "t", ms);

        // sensor= forwarding: pack into the VW args if present.
        if (args.count >= 4) {
            vwArgs.args[vwArgs.count] = args.args[3];  // copy STR sensor arg
            ++vwArgs.count;
        }

        pushVW(ctx, vwArgs, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct beginTimed().
        ctx->mc->beginTimed((float)l, (float)r, (uint32_t)ms,
                            ctx->robot->systemTime(),
                            ctx->robot->state.target,
                            replyFn, replyCtx, corrId);
        // Optional sensor= stop condition (packed into args[3] by parseT).
        if (args.count >= 4) {
            uint8_t ch; float thr; StopCondition::Cmp cmp;
            if (!mc_parseSensorToken(args.args[3].sval, ch, thr, cmp)) {
                char rbuf[64];
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                           corrId, replyFn, replyCtx);
                ctx->mc->cancel(ctx->robot->systemTime(), replyFn, replyCtx);
                return;
            }
            ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
        }
    }

    char body[48];
    snprintf(body, sizeof(body), "l=%d r=%d ms=%d", l, r, ms);
    char rbuf[80];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ── D ────────────────────────────────────────────────────────────────────────

static ParseResult parseD(const char* const* tokens, int ntokens,
                           const KVPair* kvs, int nkv)
{
    ParseResult res;
    if (ntokens < 3) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int l  = atoi(tokens[0]);
    int r  = atoi(tokens[1]);
    int mm = atoi(tokens[2]);
    if (l < -1000 || l > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "l"; return res;
    }
    if (r < -1000 || r > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "r"; return res;
    }
    if (mm < 1 || mm > 10000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "mm"; return res;
    }
    res.ok = true;
    res.args.count = 3;
    setIntArg(res.args.args[0], l);
    setIntArg(res.args.args[1], r);
    setIntArg(res.args.args[2], mm);
    // Pack optional sensor= into args[3].
    res.args.count = packSensorArg(res.args, 3, kvs, nkv);
    return res;
}

// handleD — VW converter with dist=<mm> stop param.
//
// Computes (v, ω) from (l, r), builds VW args with "dist=<mm>" stop param,
// and pushes to the queue. Falls back to direct distanceDrive() when queue
// is null (sim / unit test). The direct path also resets encoder baseline.
static void handleD(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int l  = args.args[0].ival;
    int r  = args.args[1].ival;
    int mm = args.args[2].ival;

    if (ctx->queue != nullptr) {
        float v_mms, omega_rads;
        BodyKinematics::forward((float)l, (float)r, ctx->robot->config.trackwidthMm,
                                v_mms, omega_rads);
        int v_int     = (int)v_mms;
        int omega_int = (int)(omega_rads * 1000.0f);

        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], v_int);
        setIntArg(vwArgs.args[1], omega_int);
        vwArgs.count = packKVArg(vwArgs, 2, "dist", mm);

        // sensor= forwarding: pack into the VW args if present.
        if (args.count >= 4) {
            vwArgs.args[vwArgs.count] = args.args[3];  // copy STR sensor arg
            ++vwArgs.count;
        }

        pushVW(ctx, vwArgs, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct distanceDrive() (resets enc baseline).
        ctx->robot->distanceDrive((int32_t)l, (int32_t)r, (int32_t)mm,
                                   replyFn, replyCtx, corrId);
        // Optional sensor= stop condition (packed into args[3] by parseD).
        if (args.count >= 4) {
            uint8_t ch; float thr; StopCondition::Cmp cmp;
            if (!mc_parseSensorToken(args.args[3].sval, ch, thr, cmp)) {
                char rbuf[64];
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                           corrId, replyFn, replyCtx);
                ctx->mc->cancel(ctx->robot->systemTime(), replyFn, replyCtx);
                return;
            }
            ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
        }
    }

    char body[48];
    snprintf(body, sizeof(body), "l=%d r=%d mm=%d", l, r, mm);
    char rbuf[80];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
}

// ── G ────────────────────────────────────────────────────────────────────────

static ParseResult parseG(const char* const* tokens, int ntokens,
                           const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 3) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int x     = atoi(tokens[0]);
    int y     = atoi(tokens[1]);
    int speed = atoi(tokens[2]);
    if (x < -10000 || x > 10000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "x"; return res;
    }
    if (y < -10000 || y > 10000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "y"; return res;
    }
    if (speed < 1 || speed > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "speed"; return res;
    }
    res.ok = true;
    res.args.count = 3;
    setIntArg(res.args.args[0], x);
    setIntArg(res.args.args[1], y);
    setIntArg(res.args.args[2], speed);
    return res;
}

// handleG — VW converter with x=<mm>, y=<mm>, speed=<mm/s> stop params.
//
// G is a go-to command: VW args use speed as v, 0 as omega (G's own logic
// computes steering), with "x=<mm>", "y=<mm>", "speed=<mm/s>" stop params.
// Falls back to direct beginGoTo() when queue is null (sim / unit test).
static void handleG(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int x     = args.args[0].ival;
    int y     = args.args[1].ival;
    int speed = args.args[2].ival;

    if (ctx->queue != nullptr) {
        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], speed);   // v = speed
        setIntArg(vwArgs.args[1], 0);       // omega = 0 (G's steering computed by VW handler)
        vwArgs.count = packKVArg(vwArgs, 2, "x", x);
        vwArgs.count = packKVArg(vwArgs, vwArgs.count, "y", y);
        vwArgs.count = packKVArg(vwArgs, vwArgs.count, "speed", speed);

        pushVW(ctx, vwArgs, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct beginGoTo().
        ctx->mc->beginGoTo((float)x, (float)y, (float)speed,
                           ctx->robot->systemTime(),
                           ctx->robot->state.target,
                           replyFn, replyCtx, corrId);
    }

    char body[64];
    snprintf(body, sizeof(body), "x=%d y=%d speed=%d", x, y, speed);
    char rbuf[96];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "goto", body, corrId, replyFn, replyCtx);
}

// ── R ────────────────────────────────────────────────────────────────────────

static ParseResult parseR(const char* const* tokens, int ntokens,
                           const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int speed  = atoi(tokens[0]);
    int radius = atoi(tokens[1]);
    if (speed < -1000 || speed > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "speed"; return res;
    }
    if (radius < -10000 || radius > 10000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "radius"; return res;
    }
    res.ok = true;
    res.args.count = 2;
    setIntArg(res.args.args[0], speed);
    setIntArg(res.args.args[1], radius);
    return res;
}

// handleR — VW converter with "speed=<mm/s>", "radius=<mm>" stop params.
//
// R (arc) is open-ended: v = speed, omega = speed/radius (κ = 1/radius).
// No stop condition; stop params encode raw speed + radius for VW handler.
// Falls back to direct beginArc() when queue is null (sim / unit test).
static void handleR(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int speed  = args.args[0].ival;
    int radius = args.args[1].ival;

    if (ctx->queue != nullptr) {
        // Compute omega = speed * kappa = speed / radius (kappa = 1/radius).
        float omega_rads = (radius != 0) ? ((float)speed / (float)radius) : 0.0f;
        int v_int     = speed;
        int omega_int = (int)(omega_rads * 1000.0f);  // rad/s → mrad/s

        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], v_int);
        setIntArg(vwArgs.args[1], omega_int);
        vwArgs.count = packKVArg(vwArgs, 2, "speed", speed);
        vwArgs.count = packKVArg(vwArgs, vwArgs.count, "radius", radius);

        pushVW(ctx, vwArgs, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct beginArc().
        uint32_t now = ctx->robot->systemTime();
        ctx->mc->beginArc((float)speed, (float)radius, now,
                          ctx->robot->state.target,
                          replyFn, replyCtx, corrId);
    }

    char body[48];
    snprintf(body, sizeof(body), "speed=%d radius=%d", speed, radius);
    char rbuf[80];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "arc", body, corrId, replyFn, replyCtx);
}

// ── TURN ─────────────────────────────────────────────────────────────────────

static ParseResult parseTURN(const char* const* tokens, int ntokens,
                              const KVPair* kvs, int nkv)
{
    ParseResult res;
    if (ntokens < 1) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int heading_cdeg = atoi(tokens[0]);
    if (heading_cdeg < -18000 || heading_cdeg > 18000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "heading"; return res;
    }
    // Parse optional eps=<cdeg> kv; default 300.
    int eps_cdeg = 300;
    for (int i = 0; i < nkv; ++i) {
        if (kvs[i].key && strcmp(kvs[i].key, "eps") == 0) {
            eps_cdeg = atoi(kvs[i].value);
            if (eps_cdeg < 10 || eps_cdeg > 1800) {
                res.ok = false; res.err.code = "range"; res.err.detail = "eps"; return res;
            }
            break;
        }
    }
    res.ok = true;
    res.args.count = 2;
    setIntArg(res.args.args[0], heading_cdeg);
    setIntArg(res.args.args[1], eps_cdeg);
    // Pack optional sensor= into args[2].
    res.args.count = packSensorArg(res.args, 2, kvs, nkv);
    return res;
}

// handleTURN — VW converter with "h=<cdeg>", "eps=<cdeg>" stop params.
//
// TURN is an absolute-heading rotation: v = 0 (spin-in-place), omega is
// computed by VW handler from heading_cdeg. Stop params "h=<cdeg>" and
// "eps=<cdeg>" tell VW handler to call beginTurn().
// Falls back to direct beginTurn() when queue is null (sim / unit test).
static void handleTURN(const ArgList& args, const char* corrId,
                       ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int heading_cdeg = args.args[0].ival;
    int eps_cdeg     = args.args[1].ival;

    if (ctx->queue != nullptr) {
        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], 0);   // v = 0 (spin-in-place; omega computed by VW handler)
        setIntArg(vwArgs.args[1], 0);   // omega placeholder; VW handler uses "h" param
        vwArgs.count = packKVArg(vwArgs, 2, "h", heading_cdeg);
        vwArgs.count = packKVArg(vwArgs, vwArgs.count, "eps", eps_cdeg);

        // sensor= forwarding: pack into the VW args if present.
        if (args.count >= 3) {
            vwArgs.args[vwArgs.count] = args.args[2];  // copy STR sensor arg
            ++vwArgs.count;
        }

        pushVW(ctx, vwArgs, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct beginTurn().
        uint32_t now = ctx->robot->systemTime();
        ctx->mc->beginTurn((float)heading_cdeg, (float)eps_cdeg, now,
                           ctx->robot->state.target,
                           replyFn, replyCtx, corrId);
        // Optional sensor= stop condition (packed into args[2] by parseTURN).
        if (args.count >= 3) {
            uint8_t ch; float thr; StopCondition::Cmp cmp;
            if (!mc_parseSensorToken(args.args[2].sval, ch, thr, cmp)) {
                char rbuf[64];
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                           corrId, replyFn, replyCtx);
                ctx->mc->cancel(ctx->robot->systemTime(), replyFn, replyCtx);
                return;
            }
            ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
        }
    }

    char body[48];
    snprintf(body, sizeof(body), "heading=%d eps=%d", heading_cdeg, eps_cdeg);
    char rbuf[80];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "turn", body, corrId, replyFn, replyCtx);
}

// ── VW ───────────────────────────────────────────────────────────────────────
//
// VW — body-twist velocity command (open-ended, keepalive watchdog).
//
// Wire format: VW <v_mms> <omega_mrads> [key=value ...]
//   args[0].ival = v in mm/s
//   args[1].ival = omega in mrad/s (wire units; converted to rad/s here)
//   args[2..] = optional stop params (ArgType::STR "key=value"):
//     "t=<ms>"        → call beginTimed(v, omega, ms, ...)
//     "dist=<mm>"     → call beginDistance(vL, vR equivalent, mm, ...)
//     "x=<mm>"+"y=<mm>"+"speed=<mm/s>" → call beginGoTo(x, y, speed, ...)
//     "h=<cdeg>"+"eps=<cdeg>"           → call beginTurn(h_cdeg, eps_cdeg, ...)
//     "speed=<mm/s>"+"radius=<mm>"      → call beginArc(speed, radius, ...)
//     (no stop params) → open-ended velocity (existing VW / S behaviour)
//
// The converter handlers (S, T, D, G, R, TURN) build a ParsedCommand with
// stop params packed as "key=value" STR args and push_front it onto the queue.
// This handler is then dispatched by dequeueOne() with the pre-built args.

// ── Helper: scan args[2..] for a "key=value" string and return int value. ──
// Returns defVal if the key is not found.
static int vwScanKV(const ArgList& args, const char* key, int defVal)
{
    int keyLen = 0;
    while (key[keyLen]) ++keyLen;
    for (int i = 2; i < args.count; ++i) {
        if (args.args[i].type != ArgType::STR) continue;
        const char* s = args.args[i].sval;
        // Match "key="
        int j = 0;
        while (j < keyLen && s[j] == key[j]) ++j;
        if (j == keyLen && s[j] == '=') {
            return atoi(s + j + 1);
        }
    }
    return defVal;
}

// ── Helper: check if a key is present in args[2..] ──────────────────────────
static bool vwHasKey(const ArgList& args, const char* key)
{
    int keyLen = 0;
    while (key[keyLen]) ++keyLen;
    for (int i = 2; i < args.count; ++i) {
        if (args.args[i].type != ArgType::STR) continue;
        const char* s = args.args[i].sval;
        int j = 0;
        while (j < keyLen && s[j] == key[j]) ++j;
        if (j == keyLen && s[j] == '=') return true;
    }
    return false;
}

static ParseResult parseVW(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int v     = atoi(tokens[0]);
    int omega = atoi(tokens[1]);
    if (v < -1000 || v > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "v"; return res;
    }
    if (omega < -3142 || omega > 3142) {
        res.ok = false; res.err.code = "range"; res.err.detail = "omega"; return res;
    }
    res.ok = true;
    res.args.count = 2;
    setIntArg(res.args.args[0], v);
    setIntArg(res.args.args[1], omega);
    return res;
}

static void handleVW(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int v     = args.args[0].ival;
    int omega = args.args[1].ival;
    float omega_rads = (float)omega / 1000.0f;  // mrad/s → rad/s
    uint32_t now = ctx->robot->systemTime();

    // ── Stop-param dispatch ─────────────────────────────────────────────────
    // Converter handlers (S, T, D, G, R, TURN) pack stop params as STR args
    // at args[2..].  Scan them here to select the appropriate begin*() call.

    // Check for TURN: "h=<cdeg>" present (and no "x" key).
    if (vwHasKey(args, "h") && !vwHasKey(args, "x")) {
        int h_cdeg  = vwScanKV(args, "h",   0);
        int eps     = vwScanKV(args, "eps", 300);

        ctx->mc->beginTurn((float)h_cdeg, (float)eps, now,
                           ctx->robot->state.target,
                           replyFn, replyCtx, corrId);

        // Optional sensor= forwarding.
        for (int i = 2; i < args.count; ++i) {
            if (args.args[i].type == ArgType::STR &&
                strncmp(args.args[i].sval, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (mc_parseSensorToken(args.args[i].sval + 7, ch, thr, cmp)) {
                    ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
                }
                break;
            }
        }

        char body[48];
        snprintf(body, sizeof(body), "h=%d eps=%d", h_cdeg, eps);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "vw", body, corrId, replyFn, replyCtx);
        return;
    }

    // Check for G (go-to): "x=<mm>" and "y=<mm>" present.
    if (vwHasKey(args, "x") && vwHasKey(args, "y")) {
        int x_mm    = vwScanKV(args, "x",     0);
        int y_mm    = vwScanKV(args, "y",     0);
        int speed   = vwScanKV(args, "speed", v);  // fallback to v

        ctx->mc->beginGoTo((float)x_mm, (float)y_mm, (float)speed, now,
                           ctx->robot->state.target,
                           replyFn, replyCtx, corrId);

        char body[64];
        snprintf(body, sizeof(body), "x=%d y=%d speed=%d", x_mm, y_mm, speed);
        char rbuf[96];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "vw", body, corrId, replyFn, replyCtx);
        return;
    }

    // Check for R (arc): "radius=<mm>" present (speed=<mm/s> also present).
    if (vwHasKey(args, "radius")) {
        int speed   = vwScanKV(args, "speed",  v);
        int radius  = vwScanKV(args, "radius", 0);

        ctx->mc->beginArc((float)speed, (float)radius, now,
                          ctx->robot->state.target,
                          replyFn, replyCtx, corrId);

        char body[48];
        snprintf(body, sizeof(body), "speed=%d radius=%d", speed, radius);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "vw", body, corrId, replyFn, replyCtx);
        return;
    }

    // Check for T (timed): "t=<ms>" present.
    if (vwHasKey(args, "t")) {
        int ms = vwScanKV(args, "t", 0);

        // Convert (v, omega) back to (vL, vR) for beginTimed (which takes wheel speeds).
        // beginTimed itself calls forward() again — just pass v as both wheels (straight).
        // For the queue path, vL = vR = v (forward kinematics: omega=0 straight).
        // More precisely: vL = v - omega*b/2, vR = v + omega*b/2 (inverse kinematics).
        float b = ctx->robot->config.trackwidthMm;
        float vL = (float)v - omega_rads * (b * 0.5f);
        float vR = (float)v + omega_rads * (b * 0.5f);

        ctx->mc->beginTimed(vL, vR, (uint32_t)ms, now,
                            ctx->robot->state.target,
                            replyFn, replyCtx, corrId);

        // Optional sensor= forwarding.
        for (int i = 2; i < args.count; ++i) {
            if (args.args[i].type == ArgType::STR &&
                strncmp(args.args[i].sval, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (mc_parseSensorToken(args.args[i].sval + 7, ch, thr, cmp)) {
                    ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
                }
                break;
            }
        }

        char body[48];
        snprintf(body, sizeof(body), "v=%d omega=%d ms=%d", v, omega, ms);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "vw", body, corrId, replyFn, replyCtx);
        return;
    }

    // Check for D (distance): "dist=<mm>" present.
    if (vwHasKey(args, "dist")) {
        int mm = vwScanKV(args, "dist", 0);

        // Convert (v, omega) back to (vL, vR) for distanceDrive (which takes wheel speeds).
        float b = ctx->robot->config.trackwidthMm;
        float vL = (float)v - omega_rads * (b * 0.5f);
        float vR = (float)v + omega_rads * (b * 0.5f);

        ctx->robot->distanceDrive((int32_t)vL, (int32_t)vR, (int32_t)mm,
                                   replyFn, replyCtx, corrId);

        // Optional sensor= forwarding.
        for (int i = 2; i < args.count; ++i) {
            if (args.args[i].type == ArgType::STR &&
                strncmp(args.args[i].sval, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (mc_parseSensorToken(args.args[i].sval + 7, ch, thr, cmp)) {
                    ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
                }
                break;
            }
        }

        char body[48];
        snprintf(body, sizeof(body), "v=%d omega=%d dist=%d", v, omega, mm);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "vw", body, corrId, replyFn, replyCtx);
        return;
    }

    // ── No stop params: open-ended velocity (VW / S mode) ──────────────────
    if (ctx->mc->hasActiveCommand()) {
        // Keepalive re-send: update target and re-arm TIME baseline.
        ctx->mc->activeCmd().setTarget((float)v, omega_rads);
    } else {
        // New VW command: configure MotionCommand from scratch.
        ctx->mc->beginVelocity((float)v, omega_rads, now,
                               ctx->robot->state.target,
                               replyFn, replyCtx, corrId);
    }

    char body[32];
    snprintf(body, sizeof(body), "v=%d omega=%d", v, omega);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "vw", body, corrId, replyFn, replyCtx);
}

// ── _VW ──────────────────────────────────────────────────────────────────────
// Raw velocity command: seeds BVC current state immediately (no ramp).
// Fire-and-forget — no MotionCommand, system watchdog handles keepalive.

static ParseResult parse_VW(const char* const* tokens, int ntokens,
                             const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int v     = atoi(tokens[0]);
    int omega = atoi(tokens[1]);
    if (v < -1000 || v > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "v"; return res;
    }
    if (omega < -3142 || omega > 3142) {
        res.ok = false; res.err.code = "range"; res.err.detail = "omega"; return res;
    }
    res.ok = true;
    res.args.count = 2;
    setIntArg(res.args.args[0], v);
    setIntArg(res.args.args[1], omega);
    return res;
}

static void handle_VW(const ArgList& args, const char* corrId,
                      ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int v     = args.args[0].ival;
    int omega = args.args[1].ival;
    float omega_rads = (float)omega / 1000.0f;  // mrad/s → rad/s
    ctx->mc->beginRawVelocity((float)v, omega_rads);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "_VW", nullptr, corrId, replyFn, replyCtx);
}

// ── X and STOP ───────────────────────────────────────────────────────────────

static ParseResult parseNoArgs(const char* const* /*tokens*/, int /*ntokens*/,
                               const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    res.ok = true;
    res.args.count = 0;
    return res;
}

// parseX — optional "soft" positional token; stored as STR arg if present.
static ParseResult parseX(const char* const* tokens, int ntokens,
                           const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    res.ok = true;
    if (ntokens >= 1 && strcmp(tokens[0], "soft") == 0) {
        // Pack "soft" as STR arg[0].
        res.args.count = 1;
        res.args.args[0].type = ArgType::STR;
        res.args.args[0].ival = 0;
        res.args.args[0].fval = 0.0f;
        res.args.args[0].sval[0] = 's';
        res.args.args[0].sval[1] = 'o';
        res.args.args[0].sval[2] = 'f';
        res.args.args[0].sval[3] = 't';
        res.args.args[0].sval[4] = '\0';
    } else {
        res.args.count = 0;
    }
    return res;
}

static void handleX(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    uint32_t now = ctx->robot->systemTime();

    // Check for "soft" positional arg — soft stop ramps BVC to zero.
    bool isSoft = (args.count >= 1 &&
                   args.args[0].type == ArgType::STR &&
                   strcmp(args.args[0].sval, "soft") == 0);

    if (isSoft) {
        ctx->mc->softStop(now);
    } else {
        ctx->mc->cancel(now, replyFn, replyCtx);
    }
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "x", nullptr, corrId, replyFn, replyCtx);
}

static void handleSTOP(const ArgList& /*args*/, const char* corrId,
                       ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    uint32_t now = ctx->robot->systemTime();
    ctx->mc->cancel(now, replyFn, replyCtx);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stop", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> MotionController::getCommands() const {
    void* ctx = const_cast<MotionCtx*>(&_ctx);
    return {
        makeCmd("S",    parseS,      handleS,    ctx, "badarg"), // set wheel speeds (mm/s)
        makeCmd("T",    parseT,      handleT,    ctx, "badarg"), // timed drive (ms)
        makeCmd("D",    parseD,      handleD,    ctx, "badarg"), // distance drive (mm)
        makeCmd("G",    parseG,      handleG,    ctx, "badarg"), // goto encoder position
        makeCmd("R",    parseR,      handleR,    ctx, "badarg"), // rotate in place (deg)
        makeCmd("TURN", parseTURN,   handleTURN, ctx, "badarg"), // arc turn (radius, deg)
        makeCmd("VW",   parseVW,     handleVW,   ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // velocity + angular vel (unicycle)
        makeCmd("_VW",  parse_VW,    handle_VW,  ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // raw velocity (no ramp): seed+set BVC immediately
        makeCmd("X",    parseX,      handleX,    ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // stop immediately (or "X soft" for ramp)
        makeCmd("STOP", parseNoArgs, handleSTOP, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE), // stop with deceleration
    };
}
