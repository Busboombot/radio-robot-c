// MotionCommand.cpp — body-velocity active command lifecycle.
//
// See MotionCommand.h for full API documentation.
// Architecture reference: .clasi/sprints/017-.../architecture-update.md §MotionCommand
// Sprint 017, Ticket 003.

#include "MotionCommand.h"
#include "BodyVelocityController.h"
#include <cstdio>
#include <cstring>
#include <cassert>

// ---------------------------------------------------------------------------
// Configuration phase
// ---------------------------------------------------------------------------

void MotionCommand::configure(float v_mms, float omega_rads, BodyVelocityController* bvc)
{
    _bvc       = bvc;
    _vTgt      = v_mms;
    _omegaTgt  = omega_rads;

    // Clear all per-command state so a recycled instance has no residue.
    _nStops          = 0;
    for (uint8_t i = 0; i < kMaxStopConds; ++i) {
        _stops[i] = StopCondition{};
    }
    _replyFn         = nullptr;
    _replyCtx        = nullptr;
    _corrId[0]       = '\0';
    _stopStyle       = StopStyle::SOFT;
    _active          = false;
    _stopping        = false;
    _softDeadlineMs  = 0;
    _baseline        = MotionBaseline{};
    // Reset done EVT label to default; caller must call setDoneEvt() after
    // configure() to override it for the new command.
    strncpy(_doneEvtLabel, "EVT done", sizeof(_doneEvtLabel) - 1);
    _doneEvtLabel[sizeof(_doneEvtLabel) - 1] = '\0';
}

void MotionCommand::setDoneEvt(const char* label)
{
    if (!label) return;
    strncpy(_doneEvtLabel, label, sizeof(_doneEvtLabel) - 1);
    _doneEvtLabel[sizeof(_doneEvtLabel) - 1] = '\0';
}

bool MotionCommand::addStop(const StopCondition& c)
{
    if (_nStops >= kMaxStopConds) {
        assert(false && "MotionCommand: addStop overflow — kMaxStopConds reached");
        return false;
    }
    _stops[_nStops++] = c;
    return true;
}

void MotionCommand::setReplySink(ReplyFn fn, void* ctx, const char* corrId)
{
    _replyFn  = fn;
    _replyCtx = ctx;
    if (corrId && corrId[0] != '\0') {
        strncpy(_corrId, corrId, sizeof(_corrId) - 1);
        _corrId[sizeof(_corrId) - 1] = '\0';
    } else {
        _corrId[0] = '\0';
    }
}

void MotionCommand::setStopStyle(StopStyle s)
{
    _stopStyle = s;
}

// ---------------------------------------------------------------------------
// Execution phase
// ---------------------------------------------------------------------------

void MotionCommand::start(const HardwareState& inputs, uint32_t now_ms)
{
    // Capture motion baseline.
    _baseline.t0Ms       = now_ms;
    _baseline.enc0Mm     = (inputs.encLMm + inputs.encRMm) * 0.5f;
    _baseline.heading0Rad = inputs.poseHrad;
    _baseline.pose0X     = inputs.poseX;
    _baseline.pose0Y     = inputs.poseY;

    _active   = true;
    _stopping = false;

    // Hand target to BVC; BVC will start ramping on the next tick.
    if (_bvc) {
        _bvc->setTarget(_vTgt, _omegaTgt);
    }
}

void MotionCommand::setTarget(float v_mms, float omega_rads)
{
    _vTgt     = v_mms;
    _omegaTgt = omega_rads;

    if (_bvc) {
        _bvc->setTarget(_vTgt, _omegaTgt);
    }
}

bool MotionCommand::tick(const HardwareState& inputs, uint32_t now_ms, float dt_s)
{
    if (!_active) return false;

    // ------------------------------------------------------------------
    // SOFT ramp-down sub-phase.
    // ------------------------------------------------------------------
    if (_stopping) {
        // Still ramping to (0, 0) — advance BVC.
        if (_bvc) _bvc->advance(dt_s);

        // Check termination conditions: converged OR deadline passed.
        bool converged = _bvc ? _bvc->atTarget() : true;
        int32_t dt_deadline = (int32_t)(now_ms - _softDeadlineMs);
        bool deadline_hit   = (dt_deadline >= 0);

        if (converged || deadline_hit) {
            _active   = false;
            _stopping = false;
            emitEvt(_doneEvtLabel);
        }
        return _active;
    }

    // ------------------------------------------------------------------
    // Normal running sub-phase.
    // ------------------------------------------------------------------

    // Advance BVC one tick (profile → inverse → saturate → setTarget).
    if (_bvc) _bvc->advance(dt_s);

    // Evaluate stop conditions (OR-across-array: first hit terminates).
    bool stopped = false;
    for (uint8_t i = 0; i < _nStops; ++i) {
        if (_stops[i].evaluate(inputs, now_ms, _baseline)) {
            stopped = true;
            break;
        }
    }

    if (stopped) {
        if (_stopStyle == StopStyle::HARD) {
            // Immediate teardown.
            if (_bvc) _bvc->reset();
            _active   = false;
            _stopping = false;
            emitEvt(_doneEvtLabel);
        } else {
            // SOFT: ramp to (0, 0) over up to kSoftDeadlineMs.
            _stopping       = true;
            _softDeadlineMs = now_ms + kSoftDeadlineMs;
            if (_bvc) _bvc->setTarget(0.0f, 0.0f);
        }
    }

    return _active;
}

void MotionCommand::cancel(StopStyle s)
{
    if (!_active) return;

    // HARD cancel (cancel is always an emergency abort regardless of style arg).
    if (_bvc) _bvc->reset();
    _active   = false;
    _stopping = false;
    emitEvt("EVT cancelled");

    (void)s;  // style argument reserved for future use
}

void MotionCommand::softStop(uint32_t now_ms)
{
    // No-op if not active or already ramping down.
    if (!_active || _stopping) return;

    // Arm SOFT ramp-down: BVC target → (0,0); tick() will emit EVT done
    // once the BVC converges or the 3 s deadline passes.
    _stopping       = true;
    _softDeadlineMs = now_ms + kSoftDeadlineMs;
    if (_bvc) _bvc->setTarget(0.0f, 0.0f);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void MotionCommand::emitEvt(const char* base)
{
    if (!_replyFn) return;

    char msg[48];
    if (_corrId[0] != '\0') {
        snprintf(msg, sizeof(msg), "%s #%s", base, _corrId);
    } else {
        // Manual copy to avoid another snprintf call with no format args.
        int i = 0;
        while (base[i] && i < (int)sizeof(msg) - 1) {
            msg[i] = base[i];
            ++i;
        }
        msg[i] = '\0';
    }

    _replyFn(msg, _replyCtx);
}
