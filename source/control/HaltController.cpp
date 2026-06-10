// HaltController.cpp — user-facing named stop-condition registry.
//
// See HaltController.h for the API.
// Sprint 020, Ticket 007.

#include "HaltController.h"
#include "CommandProcessor.h"
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// add — register a new StopCondition.
//
// Returns the assigned uint8_t ID, or -1 if the table is full.
// ---------------------------------------------------------------------------

int HaltController::add(const StopCondition& cond, StopStyle style, const char* str)
{
    if (_count >= kMaxEntries) return -1;

    StopEntry& e = _entries[_count];
    e.cond   = cond;
    e.id     = _nextId++;
    e.style  = style;
    e.active = true;
    int slen = str ? (int)strlen(str) : 0;
    if (slen >= (int)sizeof(e.str)) slen = (int)sizeof(e.str) - 1;
    if (slen > 0) memcpy(e.str, str, (size_t)slen);
    e.str[slen] = '\0';

    ++_count;
    return (int)e.id;
}

// ---------------------------------------------------------------------------
// remove — deactivate the entry with the given id.
// ---------------------------------------------------------------------------

bool HaltController::remove(uint8_t id)
{
    for (int i = 0; i < _count; ++i) {
        if (_entries[i].id == id && _entries[i].active) {
            _entries[i].active = false;
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// clear — deactivate all entries; return the count that were active.
// ---------------------------------------------------------------------------

int HaltController::clear()
{
    int n = 0;
    for (int i = 0; i < _count; ++i) {
        if (_entries[i].active) ++n;
    }
    clearAll();
    return n;
}

// ---------------------------------------------------------------------------
// info — write a human-readable description for a given id into buf.
// ---------------------------------------------------------------------------

bool HaltController::info(uint8_t id, char* buf, int bufLen) const
{
    for (int i = 0; i < _count; ++i) {
        if (_entries[i].id == id) {
            snprintf(buf, (size_t)bufLen, "id=%u active=%s str=\"%s\"",
                     (unsigned)_entries[i].id,
                     _entries[i].active ? "yes" : "no",
                     _entries[i].str);
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// list — emit one line per active entry via fn/ctx.
// ---------------------------------------------------------------------------

void HaltController::list(ReplyFn fn, void* ctx) const
{
    for (int i = 0; i < _count; ++i) {
        if (!_entries[i].active) continue;
        char line[80];
        snprintf(line, sizeof(line), "OK HALT id=%u str=\"%s\"",
                 (unsigned)_entries[i].id, _entries[i].str);
        fn(line, ctx);
    }
}

// ---------------------------------------------------------------------------
// count — return the number of currently active entries.
// ---------------------------------------------------------------------------

int HaltController::count() const
{
    int n = 0;
    for (int i = 0; i < _count; ++i) {
        if (_entries[i].active) ++n;
    }
    return n;
}

// ---------------------------------------------------------------------------
// clearAll — internal: reset the entry table and counter.
// ---------------------------------------------------------------------------

void HaltController::clearAll()
{
    for (int i = 0; i < _count; ++i) {
        _entries[i].active = false;
    }
    _count = 0;
}

// ---------------------------------------------------------------------------
// evaluate — check all active conditions each tick.
//
// Constructs a MotionBaseline using the stored timer and distance baselines
// (pose fields zeroed since halt conditions use absolute HardwareState values
// for position, not baseline-relative deltas). Iterates entries in order;
// the first condition that fires wins.
// ---------------------------------------------------------------------------

HaltAction HaltController::evaluate(const HardwareState& s, uint32_t now_ms,
                                     ReplyFn evtFn, void* evtCtx)
{
    if (_count == 0) return HaltAction::NONE;

    // Build a synthetic MotionBaseline. Timer and distance baselines are taken
    // from ZERO T / ZERO D; pose fields are zeroed (unused by TIME/DIST/LINE_ANY/COLOR).
    MotionBaseline base;
    base.t0Ms        = _timerBaselineMs;
    base.enc0Mm      = _distBaselineMm;
    base.heading0Rad = 0.0f;
    base.pose0X      = 0.0f;
    base.pose0Y      = 0.0f;

    for (int i = 0; i < _count; ++i) {
        if (!_entries[i].active) continue;

        if (_entries[i].cond.evaluate(s, now_ms, base)) {
            // Capture fired entry details before clearAll() wipes them.
            uint8_t  firedId    = _entries[i].id;
            StopStyle firedStyle = _entries[i].style;

            // Emit EVT halt before clearing so the id is still valid.
            char evtBuf[64];
            char evtBody[32];
            snprintf(evtBody, sizeof(evtBody), "id=%u", (unsigned)firedId);
            CommandProcessor::replyEvt(evtBuf, sizeof(evtBuf),
                                       "halt", evtBody,
                                       evtFn, evtCtx);

            clearAll();

            return (firedStyle == StopStyle::SOFT)
                   ? HaltAction::SOFT : HaltAction::HARD;
        }
    }

    return HaltAction::NONE;
}
