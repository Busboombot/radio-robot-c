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

// localEvtEmitter (file-local static MotionEventSink::emitFn) moved to
// source/control/MotionControllerBegin.cpp alongside beginGoTo, its only user
// (finding A3 split).  The emitEvt static member below stays here — it is used
// by the kept driveAdvance and reaches the sink through target.sink.emitFn.

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

            // Re-gate counter (D8 027-004): if the target is behind the robot
            // for 3 consecutive ticks, cancel PURSUE and restart PRE_ROTATE.
            // fabsf(bearing) > π/2 means dx < 0 (target behind robot-frame x axis).
            float bearing_rf = atan2f(dy, dx);
            if (fabsf(bearing_rf) > 1.5707963f) {  // π/2
                if (++_pursueBacktrackTicks >= 3) {
                    _pursueBacktrackTicks = 0;
                    // N11 fix (030-009): use cancelQuiet() instead of cancel() to
                    // suppress the spurious "EVT cancelled #<corrId>" that would
                    // otherwise be emitted for the G command's correlation id.
                    // The G command is still in progress (transitioning back to
                    // PRE_ROTATE), so emitting "EVT cancelled" for the G's corrId
                    // falsely signals to the host that the G has failed.
                    _activeCmd.cancelQuiet();
                    _startPreRotate(bearing_rf, _gSpeed, now_ms, target);
                    return;
                }
            } else {
                _pursueBacktrackTicks = 0;
            }

            // Terminal decel cap: v_cap = sqrt(2 * aDecel * d_remaining).
            // Clamps the commanded speed to ensure the BVC has time to
            // decelerate to zero before the POSITION stop fires.
            float v     = _gSpeed;
            float v_cap = sqrtf(2.0f * _cfg.aDecel * d_remaining);
            if (v_cap < v) v = v_cap;

            // Curvature clamp (D8 027-004): bound κ so passing abeam the target
            // (small d, dy ≠ 0) cannot drive ω into a tight orbit.
            // kappaMax = 2 / max(d_remaining, 2·arriveTolMm) limits the turning
            // radius to at most 0.5·arriveTolMm at the tightest point.
            float kappaMax = 2.0f / fmaxf(d_remaining,
                                          2.0f * _cfg.arriveTolMm);
            float kappa = (d2 > 0.1f)
                ? fmaxf(-kappaMax, fminf(kappaMax, 2.0f * dy / d2))
                : 0.0f;
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

