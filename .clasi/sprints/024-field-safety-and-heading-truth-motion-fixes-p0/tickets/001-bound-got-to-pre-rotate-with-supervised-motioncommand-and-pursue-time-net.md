---
id: '001'
title: Bound GOT_TO PRE_ROTATE with supervised MotionCommand and PURSUE TIME net
status: open
use-cases:
  - SUC-001
  - SUC-002
depends-on: []
github-issue: ''
issue: d05-bound-goto-pre-rotate-phase.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 024-001 — Bound GO_TO PRE_ROTATE with supervised MotionCommand and PURSUE TIME net

**Completes issue:** `d05-bound-goto-pre-rotate-phase.md`
**Chain:** D5 (no dependencies — must land first in the motion-bounding chain)

## Description

`MotionController::beginGoTo()` seeds the BVC directly when the bearing
target exceeds `turnInPlaceGate` (35°), then relies on `driveAdvance()` to
exit PRE_ROTATE once the bearing falls under the gate. No MotionCommand is
created for this phase, so there are no HEADING or TIME stop conditions. If
the fused bearing is frozen or wrong (slip, wedge, invalid OTOS), the spin is
unbounded. This is the primary confirmed cause of the "robot goes wild and
spins until power-off" failure.

Additionally, once PRE_ROTATE exits, the PURSUE phase has no overall time net
— G is the only motion verb without a TIME backstop.

The fix replaces the raw BVC seeding in PRE_ROTATE with a proper supervised
`MotionCommand` (HEADING + TIME stops) and adds a TIME net to the PURSUE
phase. The instant 180 deg/s start is removed; the BVC ramps under
`yawAccMax`.

## Files to Touch

- `source/control/MotionController.cpp` — PRE_ROTATE branch of `beginGoTo()`
  (lines ~383–395): replace `_bvc.seedCurrent(0, omega)` / `_bvc.setTarget(0, omega)`
  with `_activeCmd.configure(0.0f, omega, &_bvc)`, `addStop(makeHeadingStop(...))`,
  `addStop(makeTimeStop(...))`, set reply-sink + done EVT, call `_activeCmd.start()`.
  PURSUE branch (~line 400): add `_activeCmd.addStop(makeTimeStop(2 * (distance /
  speed) * 1000 + 4000))`.
- `source/control/MotionController.h` — no new public methods needed; verify
  `GPhase` enum has PRE_ROTATE and PURSUE values accessible.
- `host_tests/` (MockMotor / field-profile fixture) — add or update the
  field-profile fixture: `slipTurnExtra ≈ 0.26`, `fuseOtos = true`. New test
  exercises the PRE_ROTATE TIME-net trigger with frozen heading.

## Acceptance Criteria

- [ ] PRE_ROTATE branch of `beginGoTo()` creates a `MotionCommand` with both
  a HEADING stop (`bearing_delta`, `gateRad`) and a TIME stop (`2 × nominal +
  2000 ms`). The raw `_bvc.seedCurrent` / `_bvc.setTarget` calls are removed.
- [ ] BVC is ramped under `yawAccMax` — the instant 180 deg/s start is gone.
- [ ] PURSUE phase has `makeTimeStop(2 × (distance / speed) × 1000 + 4000 ms)`.
- [ ] **BVC double-tick guard:** the `_bvc.advance(dt_s)` call inside the
  PRE_ROTATE special-case block of `driveAdvance()` (lines ~683–689) is removed
  or guarded so that once PRE_ROTATE runs through `_activeCmd`, the BVC is not
  ticked twice per loop. Verify there is exactly one `_bvc.advance()` call on the
  PRE_ROTATE path through `driveAdvance()`.
- [ ] **Sim (field profile, slip on, fusion on):** issue `G` to a 135° bearing
  target with heading frozen (mock); command ends via the PRE_ROTATE TIME net and
  emits `EVT done G`. Must NOT spin forever. Must NOT emit `EVT safety_stop`
  (keepalives flowing throughout so the watchdog cannot mask the result).
- [ ] **Hardware (keepalives flowing / daemon ON):** `G` to a behind-the-robot
  target with a frozen/wrong heading → robot stops via the PRE_ROTATE TIME stop
  and emits the timeout `EVT done G`, not `EVT safety_stop`. A tour run on the
  field produces no unbounded spin.
- [ ] Existing exact-profile host_tests pass unmodified.

## Implementation Plan

### Approach

Work inside `beginGoTo()` only. The PRE_ROTATE branch is currently ~5 lines;
replace the BVC seeding block with `_activeCmd.configure(...)` following the
same pattern as `beginTurn()`. Compute the time budget as `ceil(bearingDeltaRad
/ yawRateMaxRad) * 2000 + 2000` (ms), ensuring the omega guard runs before the
division. After the configure block, confirm `driveAdvance()`'s PRE_ROTATE exit
condition transitions to PURSUE via the existing "command done → reconfigure"
pattern rather than via a separate branch, and that the extra `_bvc.advance()`
call is removed from the PRE_ROTATE block.

For the PURSUE TIME net, locate the PURSUE configure block and add
`_activeCmd.addStop(makeTimeStop(...))` immediately after the existing
`addStop(makePositionStop(...))` call.

### Testing Plan

1. Add a host_tests field-profile fixture (`slip=0.26, fuseOtos=true`).
2. Write a test `test_pre_rotate_time_net`: build a `MotionController` in the
   field-profile sim, issue `G` with heading locked (mock OTOS returns stale),
   run until done, assert terminal EVT is `done G` not `safety_stop`, and that
   elapsed ticks are less than the runaway threshold.
3. Write a test `test_pursue_time_net`: issue `G` to a reachable target in exact
   profile, confirm a TIME stop is present on the PURSUE command after `beginGoTo`.
4. Run `uv run pytest host_tests/` to confirm all prior tests still pass.

### Documentation Updates

None required beyond acceptance criteria. Open question 2 (omega guard before
TIME-net calculation) must be verified during implementation.
