---
id: '007'
title: 'N10: HALT TIME/DIST baseline at registration time, not boot epoch'
status: open
use-cases:
  - SUC-008
depends-on: []
github-issue: ''
issue: fr2-n10-halt-baselines.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# N10: HALT TIME/DIST baseline at registration time, not boot epoch

## Description

`HaltController::_timerBaselineMs` and `_distBaselineMm` both default to 0
(`HaltController.h:87-88`). `evaluate()` builds the halt condition threshold from
them (`HaltController.cpp:139-140`). A `HALT TIME 5000` registered two minutes after
boot without a prior `ZERO T` fires on the very next tick — the elapsed time since
boot-epoch far exceeds 5000 ms. When it fires it also calls `clearAll()` (:161),
wiping every other registered halt condition.

Secondary: `remove()` deactivates a slot but never frees it. `add()` always appends
at `_entries[_count]`, so 8 cumulative add/remove cycles exhaust the table for the
session even if all entries were removed. The "one trip clears all" behavior is also
undocumented on the wire.

## Acceptance Criteria

- [ ] `add()` for TYPE_TIME entries baselines `_timerBaselineMs` at `now` if it has
      not been explicitly set by a `ZERO T` command (or always baselines at call time
      if that's the cleaner design — see implementation note).
- [ ] `add()` for TYPE_DIST entries baselines `_distBaselineMm` at the current
      odometry distance if not set.
- [ ] `remove()` frees its slot so it can be reused by `add()`.
- [ ] New sim test: `HALT TIME 5000` long after boot (no prior `ZERO T`) does not
      trip on the next tick; fires ~5000 ms after registration.
- [ ] New sim test: 8 add/remove cycles — the 9th add succeeds (table is not
      exhausted).
- [ ] `python3 build.py` clean build passes.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.

## Implementation Plan

### Approach

Two independent fixes in `HaltController.cpp`:
1. Baseline at `add()` time: when adding a TIME or DIST entry, capture `now` (passed
   in or read from a clock) as the baseline for that entry. The cleanest design is
   to baseline per-entry at registration, not relying on a global baseline that must
   be set by a prior `ZERO T`/`ZERO D` command.
2. Slot reuse in `remove()`: instead of only setting `active = false`, also decrement
   `_count` (if the removed entry is the last) or compact the array / use a free-list.
   The simplest correct fix is to allow the removed slot index to be reused by the
   next `add()` — check what's feasible in the current data structure.

Document the "one fired condition clears all" behavior in the HALT wire-protocol
comment (no behavior change, just documentation).

### Files to modify

- `source/halt/HaltController.h` — adjust `_timerBaselineMs`/`_distBaselineMm`
  storage if they become per-entry fields.
- `source/halt/HaltController.cpp`
  - `add()`: capture registration-time baseline for TIME/DIST entries.
  - `remove()`: free the slot for reuse.
  - `clearAll()` / fire path: add a wire-protocol comment documenting "fires one
    condition → clears all".
- `host_tests/` or `host/tests/` — add the two tests above.

### Testing plan

Run: `uv run --with pytest python -m pytest host_tests/ host/tests/ -v`

Build: `python3 build.py` (clean).

### Notes

- Independent of tickets 001-006 (only `HaltController.cpp` changes).
- The `ZERO T` / `ZERO D` commands should still work as explicit baseline overrides
  if the host wants precise control; the fix only ensures `add()` without a prior
  `ZERO` doesn't trip instantly.
