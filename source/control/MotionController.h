#pragma once
#include <stdint.h>
#include <math.h>
#include <stddef.h>
#include "Config.h"
#include "Protocol.h"
#include "RobotState.h"
#include "BodyVelocityController.h"
#include "MotionCommand.h"
#include "CommandTypes.h"

class MotorController;
class Odometry;
class MotionController;
struct Robot;

/**
 * MotionController — owns and advances the S/T/D/G drive state machines,
 * S-mode watchdog, and odometry delta tracking.
 *
 * Calls MotorController for wheel control and reads Odometry for pose.
 * Does not parse commands. Does not emit telemetry — telemetry is
 * assembled by Robot::buildTlmFrame() into a unified TLM frame.
 *
 * Single cooperative main loop (014-005):
 *   - driveAdvance() is the single task entry point.  It advances all
 *     S/T/D/G state machines and emits completions (EVT done T/D/G,
 *     EVT safety_stop) inline via the captured per-drive reply sink
 *     (target.replyFn / target.replyCtx / target.corrId).  No ring
 *     buffer; no fiber boundary — I/O is safe inline.
 *
 * Per-drive sink capture: each begin*() writes the originating reply
 * sink into TargetState so that async completions (EVT done, EVT
 * safety_stop) are returned over the channel that initiated the drive,
 * even if a later command arrives on a different channel.
 *
 * OTOS complementary correction is handled entirely by Robot::otosCorrect()
 * (ticket 004 / 005).  MotionController no longer holds the OtosSensor
 * pointer or the slow-cadence timer.
 */

// Context bundle used by Commandable-registered handlers.
struct MotionCtx { MotionController* mc; struct Robot* robot; };

class MotionController : public Commandable {
public:
    MotionController(MotorController& mc, Odometry& odo, const RobotConfig& cfg);

    virtual std::vector<CommandDescriptor> getCommands() const override;

    // Bind the authoritative HardwareState (called by Robot after state init,
    // before the first tick).  Required so getPoseFloat() can read pose fields.
    void setHardwareState(HardwareState* s) { _hwState = s; }

    // Entry points — called from Robot drive methods.
    // Each writes mode, deadline/goal, speed, and the reply sink into target,
    // and also captures into the private members for legacy compatibility.
    // corr_id: originating command correlation id (digits only, no '#');
    //          nullptr or empty string when no id was supplied.
    void beginStream(float leftMms, float rightMms, uint32_t now_ms,
                     TargetState& target, ReplyFn fn, void* ctx);
    // VW command entry point: converts (v, ω) body-twist to (vL, vR) via
    // BodyKinematics::inverse() + saturate(), then delegates to beginStream().
    // VW command entry point: configures a MotionCommand with a TIME stop condition
    // (keepalive watchdog) and the BodyVelocityController, then starts it.
    // Does NOT delegate to beginStream(); VW is now VELOCITY mode.
    void beginVelocity(float v_mms, float omega_rads, uint32_t now_ms,
                       TargetState& target, ReplyFn fn, void* ctx,
                       const char* corr_id = nullptr);

    // R arc command entry point: computes κ = 1/radius (0 when radius==0),
    // configures a MotionCommand with target (speedMms, speedMms * κ),
    // SOFT stop style, no stop conditions (open-ended; host cancels via X
    // or soft-stops via R 0 r).  EVT "EVT done R" on SOFT ramp-down.
    // Sign convention: positive radius ⇒ positive ω ⇒ CCW (left arc).
    // speedMms == 0 ⇒ target (0, 0), SOFT ramp-down triggers immediately.
    void beginArc(float speedMms, float radiusMm, uint32_t now_ms,
                  TargetState& target, ReplyFn fn, void* ctx,
                  const char* corr_id = nullptr);
    void beginTimed(float leftMms, float rightMms, uint32_t durationMs, uint32_t now_ms,
                    TargetState& target, ReplyFn fn, void* ctx,
                    const char* corr_id = nullptr);
    void beginDistance(float leftMms, float rightMms, int32_t targetMm, uint32_t now_ms,
                       TargetState& target, ReplyFn fn, void* ctx,
                       const char* corr_id = nullptr);
    void beginGoTo(float tx, float ty, float speedMms, uint32_t now_ms,
                   TargetState& target, ReplyFn fn, void* ctx,
                   const char* corr_id = nullptr);

    // TURN command entry point: rotate to an absolute heading using HEADING stop condition.
    // headingCdeg: target heading in centidegrees (same unit as TLM pose field); range ±18000.
    // epsCdeg: heading tolerance in centidegrees; default 300 cdeg (3°).
    // Sign convention: positive headingCdeg ⇒ CCW (positive ω), matching OTOS CCW convention.
    // Omega magnitude from yawRateMax (deg/s → rad/s). Shortest-path sign computed at start.
    // EVT "EVT done TURN" on arrival within eps. SOFT stop style.
    void beginTurn(float headingCdeg, float epsCdeg, uint32_t now_ms,
                   TargetState& target, ReplyFn fn, void* ctx,
                   const char* corr_id = nullptr);
    void stop(uint32_t now_ms, ReplyFn fn, void* ctx);

    // Cancel the active MotionCommand (HARD stop) and go IDLE.
    // Used by the X verb and STOP handler when a VW command is active.
    void cancel(uint32_t now_ms, ReplyFn fn, void* ctx);

    // setCtx — bind the Robot* for Commandable handlers.
    // Called by Robot's constructor after motionController is fully constructed.
    void setCtx(struct Robot* r) { _ctx.mc = this; _ctx.robot = r; }

    // Query whether a MotionCommand is currently active (running or soft-stopping).
    // Used by CommandProcessor to distinguish new VW vs keepalive VW.
    bool hasActiveCommand() const { return _activeCmd.active(); }

    // Access the active MotionCommand for keepalive re-arm (setTarget).
    // Only call when hasActiveCommand() returns true.
    MotionCommand& activeCmd() { return _activeCmd; }

    // Cooperative-loop task entry point (014-005).
    //
    // Advances all S/T/D/G state machines.  Emits EVT completions (done T/D/G,
    // safety_stop) inline via target.replyFn(msg, target.replyCtx).
    // Safe to call I/O inline — there is no fiber boundary in the single
    // cooperative main loop.
    void driveAdvance(HardwareState& inputs, MotorCommands& cmds,
                      TargetState& target, uint32_t now_ms);

    DriveMode mode() const { return _mode; }

private:
    MotorController&   _mc;
    Odometry&          _odo;
    const RobotConfig& _cfg;
    HardwareState*     _hwState;  // authoritative state; set by setHardwareState()

    // Context bundle for Commandable-registered handlers.
    // Populated by setCtx() (called from Robot constructor).
    mutable MotionCtx  _ctx;

    // MotionCommand subsystem (Sprint 017).
    // _bvc MUST be declared before _activeCmd: MotionController's constructor
    // passes &_bvc to _activeCmd.configure(), so _bvc must be fully constructed
    // first.  Do not reorder.
    BodyVelocityController _bvc;        // body-level (v,ω) profiler
    MotionCommand          _activeCmd;  // the single active MotionCommand (VW, …)

    // Drive mode
    DriveMode _mode;

    // Current speed targets (kept for internal use only)
    float _tgtL;
    float _tgtR;

    // D-command state for per-tick decel hook
    float _dDistTarget;  // target distance in mm
    float _dOmega;       // commanded yaw rate at begin (from forward kinematics)
    float _dEnc0;        // encoder average at begin (baseline for decel cap)

    // G go-to state machine
    enum class GPhase { IDLE, PRE_ROTATE, PURSUE };
    GPhase _gPhase;
    float  _gTargetXWorld;  // goal x in world frame (mm), set at beginGoTo()
    float  _gTargetYWorld;  // goal y in world frame (mm), set at beginGoTo()
    float  _gSpeed;

    // Tick timing
    uint32_t _lastTickMs;

    // Updated at top of driveAdvance()
    uint32_t _currentTimeMs;

    // Internal helpers
    void fullStop(ReplyFn fn, void* ctx);
    void getPoseFloat(float& x, float& y, float& h_rad) const;

    // Emit an EVT message inline via the captured reply sink.
    // Builds "<base> #<corrId>" if corrId is non-empty, else just <base>.
    // Clears target.corrId after emitting (marks this completion consumed).
    static void emitEvt(const char* base, TargetState& target);
};
