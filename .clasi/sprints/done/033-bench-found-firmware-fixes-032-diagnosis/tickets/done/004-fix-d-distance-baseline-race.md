---
id: '004'
title: Fix D distance baseline race
status: done
use-cases:
- SUC-004
depends-on: []
issue: fr-bench-d-distance-baseline-race.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix D distance baseline race

## Description

A `D` command following a `TURN` (without a `ZERO enc` between them) can instant-complete
with zero motion. The root cause is an ordering bug in `MotionController::beginDistance()`
(`source/control/MotionController.cpp:340-389`):

1. `beginDistance()` resets the hardware accumulators.
2. `_activeCmd.start(inputs, now_ms)` snapshots `base.enc0Mm = (encLMm + encRMm)/2` from
   `state.inputs` — which still holds the **previous command's** encoder values.
3. Only after `beginDistance()` returns does `Robot::distanceDrive()` zero
   `state.inputs.encLMm/R` (`Robot.cpp:432-441`).

On the first evaluate, the DISTANCE stop computes `traveled = |0 − enc0|` = the stale
average from the prior command. If that average ≥ targetMm, the stop fires immediately.

Fix: zero `state.inputs.encLMm/R` (call `resetEncoders()` or equivalent) **before**
`_activeCmd.start()` inside `beginDistance()`. Update the now-incorrect comment at
`MotionController.cpp:382-386` ("the baseline enc0 captured by MotionCommand::start() will
be 0") to reflect the corrected ordering.

Log evidence: sqD2/sqD4 in the 032 bench run instant-completed (`enc=0,0`, `mode=I` from
the start) because each followed a TURN that left avg ≈ 250 mm — exactly matching target 250.

## Acceptance Criteria

- [ ] Sim test: `D` → `TURN` → `D` (no `ZERO enc` between) → the second `D` travels the
      full commanded distance (does not instant-complete, does not travel less than target)
- [ ] The stale comment at `MotionController.cpp:382-386` is corrected
- [ ] `python3 build.py` clean build passes
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes

## Testing

- **Existing tests to run**: `uv run --with pytest python -m pytest host_tests/ host/tests/`
- **New tests to write**: Sim test: issue D → TURN → D with no ZERO in between; assert the
  second D's traveled distance equals (or closely approximates) the commanded distance and
  that `enc0Mm` is 0 at the start of the second D
- **Verification command**: `uv run --with pytest python -m pytest host_tests/ host/tests/`

## Implementation Plan

### Approach

In `MotionController::beginDistance()` (`source/control/MotionController.cpp`):

Move the `state.inputs.encLMm = state.inputs.encRMm = 0` assignments (or the
`resetEncoders()` call that does this) to occur **before** `_activeCmd.start(inputs,
now_ms)`. The hardware accumulator reset already happens at the top of `beginDistance()`;
the only change is that the software `state.inputs` mirror must be zeroed before the
baseline snapshot.

Check the call in `Robot::distanceDrive()` (`Robot.cpp:432-441`) — after this change, the
zeroing there becomes a no-op (both are now 0), but leave it in place to be safe.

Update the comment at `MotionController.cpp:382-386`.

### Files to Modify

- `source/control/MotionController.cpp` — move encoder-input zeroing before
  `_activeCmd.start()`; fix the misleading comment
- `host_tests/` — add D→TURN→D baseline race sim test

### Documentation Updates

Correct the comment at `MotionController.cpp:382-386`.
