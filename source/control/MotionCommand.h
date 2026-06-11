#pragma once
#include <stdint.h>
#include "RobotState.h"
#include "Protocol.h"
#include "StopCondition.h"

class BodyVelocityController;

// ---------------------------------------------------------------------------
// MotionCommand — active-command object for body-velocity motion.
//
// Lifecycle:
//   1. configure(v, omega, bvc*)  — store target and BVC pointer; reset state.
//   2. addStop(c)                 — append stop conditions (up to kMaxStopConds).
//   3. setReplySink(fn, ctx, id)  — capture EVT reply channel.
//   4. setStopStyle(s)            — SOFT (default) or HARD teardown.
//   5. start(inputs, now_ms)      — snapshot MotionBaseline; hand off to BVC.
//   6. tick(inputs, now_ms, dt_s) — advance BVC; evaluate conditions; handle teardown.
//   7. cancel(s)                  — abort with EVT cancelled.
//
// A single MotionCommand instance is owned by MotionController. Calling
// configure() + start() a second time recycles the object with no residue.
//
// EVT emission mirrors MotionController::emitEvt: builds "<base> #<corrId>"
// when corrId is non-empty, then calls the captured reply function.
//
// No heap. Stop conditions are held in a fixed-size inline array.
//
// Architecture reference: .clasi/sprints/017-.../architecture-update.md §MotionCommand
// Sprint 017, Ticket 003.
// ---------------------------------------------------------------------------
class MotionCommand {
public:
    /** Maximum number of stop conditions per command. */
    static constexpr uint8_t kMaxStopConds = 4;

    /** SOFT = ramp to (0,0) then emit EVT done; HARD = stop immediately. */
    enum class StopStyle : uint8_t { SOFT, HARD };

    // ------------------------------------------------------------------
    // Configuration phase (call before start)
    // ------------------------------------------------------------------

    /**
     * configure — store target twist, BVC pointer; reset all state.
     *
     * Clears the stop-condition array, reply sink, stop style, and active flag.
     * Must be called before addStop / start.
     *
     * @param v_mms      Commanded body forward speed, mm/s.
     * @param omega_rads Commanded yaw rate, rad/s.
     * @param bvc        Pointer to the owning MotionController's BVC (non-null).
     */
    void configure(float v_mms, float omega_rads, BodyVelocityController* bvc);

    /**
     * addStop — append a stop condition to the fixed array.
     *
     * @return false (and assert in debug) if the array is already full.
     *         The caller should check the return value and not add conditions
     *         beyond kMaxStopConds.
     */
    bool addStop(const StopCondition& c);

    /**
     * setReplySink — capture the EVT reply channel.
     *
     * @param fn     Reply function pointer (may be nullptr — EVT suppressed).
     * @param ctx    Opaque context passed through to fn.
     * @param corrId Correlation ID string (copied; up to 15 chars + NUL).
     */
    void setReplySink(ReplyFn fn, void* ctx, const char* corrId);

    /**
     * setStopStyle — choose SOFT (graceful ramp-down) or HARD (immediate).
     *
     * Default is SOFT.
     */
    void setStopStyle(StopStyle s);

    /**
     * setDoneEvt — override the EVT name emitted on normal completion.
     *
     * Default is "EVT done". VW sets this to "EVT safety_stop" so that
     * keepalive-loss termination preserves the existing wire contract.
     * The label is copied; up to 23 chars + NUL.
     *
     * @param label  Full EVT string prefix, e.g. "EVT safety_stop".
     */
    void setDoneEvt(const char* label);

    // ------------------------------------------------------------------
    // Execution phase
    // ------------------------------------------------------------------

    /**
     * start — snapshot MotionBaseline and hand target to BVC.
     *
     * Captures enc/pose/heading baseline from inputs; calls
     * bvc->setTarget(_vTgt, _omegaTgt); sets active = true.
     *
     * @param inputs  Current hardware state (for baseline snapshot).
     * @param now_ms  Current system time, ms (for t0Ms baseline).
     */
    void start(const HardwareState& inputs, uint32_t now_ms);

    /**
     * setTarget — live-update the target twist while the command is running.
     *
     * Updates _vTgt/_omegaTgt; calls bvc->setTarget.
     *
     * Safe to call while active or while idle (configure phase).
     *
     * @param v_mms      New body forward speed, mm/s.
     * @param omega_rads New yaw rate, rad/s.
     */
    void setTarget(float v_mms, float omega_rads);

    /**
     * tick — advance BVC one tick; evaluate stop conditions; handle teardown.
     *
     * Returns true while the command is still running (including SOFT ramp-down).
     * Returns false once fully terminated (IDLE).
     *
     * Ordering: setTarget (if needed) → bvc->advance(dt_s) → evaluate stops.
     *   If a stop fires with SOFT style: _stopping = true, target (0,0), deadline set.
     *   If a stop fires with HARD style: bvc->reset(), emit EVT done, go IDLE.
     *   During SOFT ramp: if bvc->atTarget() or deadline passed, emit EVT done, go IDLE.
     *
     * @param inputs  Current hardware state (for stop evaluation).
     * @param now_ms  Current system time, ms.
     * @param dt_s    Elapsed time since last tick, seconds.
     * @return        active()
     */
    bool tick(const HardwareState& inputs, uint32_t now_ms, float dt_s);

    /**
     * cancel — abort the command immediately.
     *
     * HARD (default): calls bvc->reset(); emits "EVT cancelled"; goes IDLE.
     *   The caller (MotionController::cancel) is responsible for calling
     *   mc.stop() after this returns.
     * SOFT: same as HARD cancel for now (cancel is always an emergency abort).
     *
     * No-op if not active.
     *
     * @param s  Stop style (default HARD).
     */
    void cancel(StopStyle s = StopStyle::HARD);

    /**
     * softStop — arm SOFT ramp-down from outside (no stop condition needed).
     *
     * Sets BVC target to (0, 0) and enters the _stopping sub-phase so that
     * tick() will emit EVT done when the BVC converges to zero.
     * No-op if already stopping or not active.
     *
     * @param now_ms  Current system time (for soft deadline).
     */
    void softStop(uint32_t now_ms);

    /**
     * active — true while the command is running or during SOFT ramp-down.
     *
     * False when in IDLE state (not yet started, or fully terminated).
     */
    bool active() const { return _active; }

    /**
     * isOpenEnded — true when the command has no stop conditions.
     *
     * Open-ended commands (VW, R) run indefinitely until cancelled or timed
     * out by the system watchdog.  Self-terminating commands (T, D, G, TURN)
     * have at least one stop condition and manage their own lifetime.
     *
     * Used by the system watchdog to determine whether to fire safety_stop.
     */
    bool isOpenEnded() const { return _nStops == 0; }

    /**
     * hasTimeStop — true when at least one stop condition has Kind::TIME.
     *
     * Self-terminating commands (T, D, G, TURN, RT) all carry a TIME stop
     * as a runaway backstop; open-ended streaming (S / VW / R) do not.
     *
     * The system watchdog uses this to exempt time-bounded commands from the
     * keepalive requirement: a command that already has a TIME net cannot spin
     * forever if the host goes silent, so the watchdog skips its check for it.
     * Open-ended commands (no TIME stop) remain keepalive-bound.
     */
    bool hasTimeStop() const;

private:
    BodyVelocityController* _bvc            = nullptr;
    float       _vTgt                        = 0.0f;
    float       _omegaTgt                   = 0.0f;
    StopCondition _stops[kMaxStopConds]     = {};
    uint8_t     _nStops                     = 0;
    MotionBaseline _baseline                = {};
    ReplyFn     _replyFn                    = nullptr;
    void*       _replyCtx                   = nullptr;
    char        _corrId[16]                 = {};
    StopStyle   _stopStyle                  = StopStyle::SOFT;
    bool        _active                     = false;
    bool        _stopping                   = false;   // true during SOFT ramp-down
    uint32_t    _softDeadlineMs             = 0;
    /** Absolute SOFT-stop deadline: 3000 ms after a stop fires. */
    static constexpr uint32_t kSoftDeadlineMs = 3000;

    /** EVT label emitted on normal (non-cancel) completion. Default "EVT done". */
    char        _doneEvtLabel[24]    = "EVT done";

    /**
     * emitEvt — build and emit an EVT message via the captured reply sink.
     *
     * Builds "<base> #<corrId>" if corrId is non-empty, else just "<base>".
     * Mirrors MotionController::emitEvt.
     */
    void emitEvt(const char* base);
};
