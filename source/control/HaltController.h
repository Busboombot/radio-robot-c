#pragma once
#include <stdint.h>
#include "StopCondition.h"
#include "Protocol.h"

// ---------------------------------------------------------------------------
// HaltController — user-facing named stop-condition registry (ticket 020-007).
//
// Owns up to kMaxEntries user-registered StopEntry records. Each tick the
// caller invokes evaluate(); if any condition fires HaltController emits
// "EVT halt id=<n>", clears all conditions, and returns HaltAction::HARD or
// HaltAction::SOFT to tell LoopScheduler which X variant to dispatch.
//
// Baselines (timer and distance) are set by the ZERO T / ZERO D command
// extensions and are folded into a synthetic MotionBaseline for evaluate().
//
// Wire commands (HALT TIME/DIST/CLEAR) add/remove conditions by calling
// add() / clear() from their handlers.
//
// Design note: LoopScheduler owns HaltController as a public member so that
// command handlers (registered in Robot::buildCommandTable) can reach it via
// RobotSysCtx::sched->haltController without adding a separate context type.
// ---------------------------------------------------------------------------

// HaltAction — returned by evaluate() each tick.
enum class HaltAction : uint8_t {
    NONE,  // no condition fired; keep running
    HARD,  // fire X (immediate stop)
    SOFT   // fire X soft (ramp to zero)
};

// StopStyle — stored per entry; determines HaltAction when the entry fires.
enum class StopStyle : uint8_t { HARD, SOFT };

// StopEntry — one registered halt condition.
struct StopEntry {
    StopCondition cond;
    uint8_t       id;
    StopStyle     style;
    bool          active;
    char          str[40];   // original command string for HALT INFO/LIST
};

class HaltController {
public:
    static constexpr int kMaxEntries = 8;

    HaltController() = default;

    // add — register a new condition.
    // Returns the assigned ID (monotonically increasing uint8_t); -1 on full.
    int add(const StopCondition& cond, StopStyle style, const char* str);

    // remove — deactivate the entry with the given id.
    // Returns true if found and removed.
    bool remove(uint8_t id);

    // clear — remove all entries; return the count that were active.
    int clear();

    // info — write a description of entry id into buf (at most bufLen-1 chars).
    // Returns true if id was found.
    bool info(uint8_t id, char* buf, int bufLen) const;

    // list — call fn(msg, ctx) once per active entry with a summary line.
    void list(ReplyFn fn, void* ctx) const;

    // setTimerBaseline — set the t0Ms reference for TIME conditions.
    void setTimerBaseline(uint32_t now_ms) { _timerBaselineMs = now_ms; }

    // setDistBaseline — set the enc0Mm reference for DISTANCE conditions.
    void setDistBaseline(float enc_avg_mm) { _distBaselineMm = enc_avg_mm; }

    // count — return the number of currently active entries.
    int count() const;

    // evaluate — check all active conditions each tick.
    // If any fires: emits "EVT halt id=<n>" via evtFn/evtCtx, clears all
    // conditions, and returns HARD or SOFT. Otherwise returns NONE.
    HaltAction evaluate(const HardwareState& s, uint32_t now_ms,
                        ReplyFn evtFn, void* evtCtx);

private:
    StopEntry _entries[kMaxEntries] = {};
    int       _count   = 0;
    uint8_t   _nextId  = 0;
    uint32_t  _timerBaselineMs = 0;
    float     _distBaselineMm  = 0.0f;

    // clearAll — internal: mark all entries inactive and reset _count.
    void clearAll();
};
