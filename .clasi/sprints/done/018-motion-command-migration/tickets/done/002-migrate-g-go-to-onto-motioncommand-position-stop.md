---
id: '002'
title: Migrate G go-to onto MotionCommand POSITION stop
status: done
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate G go-to onto MotionCommand POSITION stop

## Description

Replace the inline `_vRamped` trapezoid in `DriveController`'s PURSUE loop with a
`POSITION`-stop MotionCommand whose per-tick pursuit hook updates `(v, ω)` via
`MotionCommand::setTarget`. Remove `_vRamped` member. PRE_ROTATE stays raw.
`EVT done G` wire contract preserved.

**Key design (from architecture-update.md §G Pursuit Hook):**
- `beginGoTo` entering PURSUE directly: configure `_activeCmd` with initial target
  `(_gSpeed, 0)`, add `makePositionStop(gTargetXWorld, gTargetYWorld, arriveTolMm)`,
  `setDoneEvt("EVT done G")`, capture reply sink, call `_activeCmd.start(...)`.
- PRE_ROTATE → PURSUE transition in `driveAdvance`: configure and start `_activeCmd`
  at this point (same as direct-pursue path).
- Per-tick PURSUE hook (in `driveAdvance`): compute `d_remaining`, `kappa`,
  `v_cap = sqrt(2 * aDecel * d_remaining)`; call
  `_activeCmd.setTarget(min(_gSpeed, v_cap), min(_gSpeed, v_cap) * kappa)`.
  This call must happen **before** `_activeCmd.tick()` in the control flow.
- The MotionCommand `tick()` is called via the early-return path at the top of
  `driveAdvance` — no change to the tick dispatch structure.
- The old `if (d_remaining < arriveTolMm) { fullStop; emitEvt; }` block is removed;
  the POSITION stop condition handles arrival.
- Remove `_vRamped = 0.0f` from all call sites; remove member declaration.

## Acceptance Criteria

- [x] `_vRamped` member removed from `DriveController.h` and `.cpp` (grep confirms 0 occurrences).
- [x] `tests/dev/test_pursuit_arc_steering.py` passes unchanged.
- [x] `EVT done G` wire format preserved (grep all test files for `done G` before editing emission).
- [x] POSITION stop condition terminates on arrival; old arrival branch removed from `driveAdvance`.
- [x] Terminal decel cap applied each tick in pursuit hook.
- [x] PRE_ROTATE branch unchanged (still raw `startDriveClean` + bearing check).
- [x] `uv run --with pytest python -m pytest -q` passes at 1226/8 baseline (ticket had 1179 — pre-existing pass count was already 1226).
- [x] Clean build: `python3 build.py --clean` succeeds.
- [ ] On-robot bench (G arcs to target + decelerates cleanly; no jerk from rest) — stakeholder-deferred.

## Implementation Plan

### Files to modify
- `source/control/DriveController.h` — remove `float _vRamped`
- `source/control/DriveController.cpp`:
  - Constructor: remove `_vRamped(0.0f)` initialiser
  - `beginGoTo`: remove `_vRamped = 0.0f`; when bearing ≤ gate (direct PURSUE), configure
    and start `_activeCmd` with POSITION stop + done EVT + reply sink
  - `driveAdvance` PRE_ROTATE block: remove `_vRamped = 0.0f` on transition; instead
    configure and start `_activeCmd` at PRE_ROTATE → PURSUE boundary
  - `driveAdvance` PURSUE block: replace the `_vRamped` ramp + `inverse/saturate/setTarget`
    with the pursuit hook (compute d_remaining, kappa, v_cap; call `_activeCmd.setTarget`).
    Remove the `d_remaining < arriveTolMm` fullStop/emitEvt block. Ensure setTarget
    precedes the `_activeCmd.tick()` dispatch at the top of `driveAdvance`.

### Control flow note
The natural structure is:
```
if (_activeCmd.active()) {
    if (_mode == GO_TO && _gPhase == GPhase::PURSUE) {
        // pursuit hook: update setTarget before tick
        ... compute and call _activeCmd.setTarget(v_cap, omega) ...
    }
    bool running = _activeCmd.tick(inputs, now_ms, dt_s);
    if (!running) { _mode = IDLE; target.mode = IDLE; }
    return;
}
```
PRE_ROTATE check still lives in the else-branch (no active command).

### Testing plan
- Run `tests/dev/test_pursuit_arc_steering.py` — must pass with no changes.
- Grep `done G` in all test and calibration files; verify EVT string unchanged.
- Full pytest suite: `uv run --with pytest python -m pytest -q`.
- Bench (stakeholder-deferred): G arcs in and decelerates cleanly; no jerk at PURSUE entry.
