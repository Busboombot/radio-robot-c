---
id: '004'
title: Wire VW onto MotionCommand in DriveController
status: open
use-cases:
  - SUC-001
  - SUC-002
  - SUC-005
depends-on:
  - '003'
github-issue: ''
issue: motion-command-body-velocity-control.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 017-004: Wire VW onto MotionCommand in DriveController

## Description

Integrate `BodyVelocityController` and `MotionCommand` into `DriveController` and migrate
the VW command from the raw STREAMING path onto a MotionCommand. This is the first
firmware behaviour change of the sprint.

After this ticket:
- `DriveController` owns `_bvc` and `_activeCmd` as value members.
- `beginVelocity` configures a MotionCommand with a TIME stop condition (safety watchdog).
- `driveAdvance` ticks the active command when one is running.
- VW now ramps smoothly (trapezoid); keepalive loss still fires a safety stop.
- The `S` command is completely unchanged (still uses `beginStream` → STREAMING watchdog).

## Files to Modify

- `source/control/DriveController.h` — add `_bvc`, `_activeCmd` members; add `cancel()`
  method.
- `source/control/DriveController.cpp` — implement `beginVelocity` via MotionCommand;
  update `driveAdvance`; add `cancel()` implementation.
- `source/app/CommandProcessor.cpp` — update VW handler for keepalive re-arm path.
- `tests/dev/test_vw_command.py` — update or extend existing VW tests (currently tests the
  old raw path; add assertions for ramp behaviour and safety-stop on keepalive loss).

## Acceptance Criteria

### DriveController

- [ ] `_bvc (BodyVelocityController)` and `_activeCmd (MotionCommand)` are value members
  declared in DriveController private section (`_bvc` before `_activeCmd`).
- [ ] Constructor initialises `_bvc(mc, cfg)`.
- [ ] `beginVelocity(v, omega, now_ms, target, fn, ctx, corr_id)`:
  - Calls `_activeCmd.configure(v, omega, &_bvc)`.
  - Adds a TIME stop condition: `a = (float)_cfg.sTimeoutMs`.
  - Calls `_activeCmd.setReplySink(fn, ctx, corr_id)`.
  - Calls `_activeCmd.setStopStyle(SOFT)`.
  - Calls `_activeCmd.start(inputs, now_ms)`.
  - Does NOT call `beginStream` or set `_mode = STREAMING`.
  - Sets `_mode` to a tag that does NOT match `STREAMING` (e.g. use existing enum or add
    a new `VELOCITY` DriveMode tag — see open question in architecture-update.md; choose
    the simplest approach that prevents the STREAMING watchdog from firing).
- [ ] VW keepalive re-send path in CommandProcessor: if `_activeCmd.active()`, calls
  `_activeCmd.setTarget(v, omega)` (which re-arms TIME and updates target) instead of
  calling `beginVelocity` from scratch. CommandProcessor detects this via
  `driveController._activeCmd.active()` — expose an `hasActiveCommand()` method or
  expose mode.
- [ ] `driveAdvance`: at the top of the tick, if `_activeCmd.active()`:
  - Compute `dt_s` from `now_ms - _lastTickMs`.
  - Call `_activeCmd.tick(inputs, now_ms, dt_s)`.
  - If `tick` returns false (terminated), set `_mode = IDLE`.
  - Return early (bypass the old S/T/D/G if-chain).
- [ ] `cancel(uint32_t now_ms, ReplyFn fn, void* ctx)`:
  - Calls `_activeCmd.cancel(HARD)`.
  - Calls `_mc.stop()`.
  - Sets `_mode = IDLE`.
- [ ] STREAMING watchdog branch in `driveAdvance` is guarded: fires only when
  `_mode == STREAMING` (i.e. only the `S` command triggers it).
- [ ] `S` command and `beginStream` are completely unchanged.

### Safety watchdog parity
- [ ] VW with no keepalive within `sTimeoutMs`: motors ramp to zero; safety EVT emitted.
- [ ] VW keepalive re-sends within `sTimeoutMs`: motor keeps running at new target.
- [ ] `S` command keepalive / safety_stop behaviour unchanged (existing test_vw_command.py
  / test_tlm_stream.py tests still pass).

### Host tests
- [ ] `test_vw_command.py` updated: assert VW response is `OK vw`; no regression on
  existing parsing/response tests.
- [ ] New host test or extended test: simulate VW then keepalive loss; assert `EVT
  safety_stop` (or `EVT done` with TIME condition, depending on open-question resolution)
  is received within `sTimeoutMs + ramp_time`.
- [ ] All existing tests: `uv run --with pytest python -m pytest -q` at 1035/8.

### Build
- [ ] Clean build: `python3 build.py --clean` completes without errors.

## Implementation Plan

1. Add `_bvc` and `_activeCmd` to `DriveController.h` (private, in declaration order:
   `_bvc` before `_activeCmd`).
2. Add `cancel()` public method declaration to `DriveController.h`.
3. Add `hasActiveCommand() const { return _activeCmd.active(); }` (or similar) to expose
   active state to CommandProcessor without breaking encapsulation.
4. In `DriveController.cpp`:
   - Initialize `_bvc` in the constructor member-init list: `, _bvc(_mc, _cfg)`.
   - Rewrite `beginVelocity` to configure `_activeCmd` (delete the old `beginStream`
     delegation).
   - In `driveAdvance`: add the `if (_activeCmd.active())` early-return block at the top
     of the cadence-gated section.
   - Implement `cancel()`.
   - Guard the STREAMING watchdog with `if (_mode == DriveMode::STREAMING)` (it already is
     guarded this way; verify `beginVelocity` no longer sets `_mode = STREAMING`).
5. In `CommandProcessor.cpp`, VW handler: check `_robot.driveController.hasActiveCommand()`
   before deciding whether to call `beginVelocity` (new command) or `setTarget` (re-arm).
6. Update `tests/dev/test_vw_command.py`.
7. Run `python3 build.py --clean` and `uv run --with pytest python -m pytest -q`.

## Notes

- Open question from architecture-update.md §Open Questions 1: DriveMode tag for VW in
  TLM. Resolve pragmatically: add `DriveMode::VELOCITY = 5` to the enum in `Config.h` so
  TLM shows `mode=VELOCITY` for VW. Confirm this does not break any host script that
  checks `mode==STREAMING` for VW (check `test_vw_command.py`).
- Open question 2: EVT name for safety stop. Check `test_vw_command.py` and any other
  host script that asserts `EVT safety_stop`. If asserted, emit `EVT safety_stop` from
  `MotionCommand` as the done EVT name for the TIME condition on VW. This can be done by
  passing a custom EVT name through `setReplySink` or a dedicated `setSafetyStopEvt`
  method. Simplest: hardcode the EVT name as `EVT safety_stop` when the stop was a TIME
  condition, vs `EVT done` for other conditions. Confirm with existing tests what name is
  expected before choosing an approach.

## Bench Verification (stakeholder-deferred)

- Flash robot (verify flash target is robot, not relay).
- `VW 200 0` → robot ramps smoothly from rest; no instantaneous step.
- `VW 200 314` (arc) → smooth yaw ramp; curvature maintained.
- Stop sending keepalives → robot slows to a stop; `EVT safety_stop` received.
- `S 200 200` → still works, still uses STREAMING path, no regression.
