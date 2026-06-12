---
status: in-progress
sprint: '030'
tickets:
- 030-007
---

# FR2-N10 (Med) — HALT TIME/DIST baselines default to boot epoch: instant trip

## Context

Source: `docs/code_review/2026-06-12-Fable-correctness-review/findings.md` §N10.

`HaltController::_timerBaselineMs`/`_distBaselineMm` default to 0
(`HaltController.h:87-88`); `evaluate()` builds the baseline from them
(`HaltController.cpp:139-140`). `HALT TIME 5000` registered minutes after boot
without a prior `ZERO T` fires on the next tick — an unexpected HARD X that also
wipes all other halt entries when it fires.

Secondary: `remove()` deactivates but never frees a slot — `add` always appends at
`_entries[_count]`, so 8 cumulative adds fill the table for the session even if all
were removed (`HaltController.cpp:17-48`). And one fired condition clears *all*
registered conditions (`clearAll()` at `:161`) — at least worth documenting on the
wire.

## Fix

Baseline TIME/DIST entries at `add()` time (capture `now`/current distance), or reject
TIME/DIST registration when the baseline was never set. Make `remove()` free its slot
so the table can be reused within a session. Document the "one trip clears all"
behavior on the wire.

## Acceptance

- `HALT TIME 5000` long after boot (no prior `ZERO T`) does not trip on the next tick
  (sim test).
- Repeated add/remove cycles do not exhaust the 8-slot table.
