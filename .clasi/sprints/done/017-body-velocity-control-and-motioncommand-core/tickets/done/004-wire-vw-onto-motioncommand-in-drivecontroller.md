---
id: '004'
title: Wire VW onto MotionCommand in DriveController
status: done
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

## Open Question Resolutions (Sprint 017-004)

### Q1: DriveMode tag for VW

**Decision: Add `DriveMode::VELOCITY = 5`; VW reports `mode=V` in TLM.**

Grep evidence: `test_vw_command.py::TestVWWatchdog::test_vw_mode_is_S_not_new_mode` was
checking a hardcoded string, not live firmware — it did not assert a wire dependency on
`mode=S`. No other host test in `host/tests/` or `tests/dev/` grep-matches `mode==STREAMING`
or `mode=S` for VW specifically (the S command tests check mode=S in their own context).

Action taken:
- `DriveMode::VELOCITY = 5` added to `Config.h`.
- `AppContext::buildTlmFrame` adds `case DriveMode::VELOCITY: modeChar = 'V';`.
- `test_vw_mode_is_S_not_new_mode` renamed/updated to `test_vw_mode_is_V_not_S`.
- New test `test_s_command_still_uses_streaming_mode` confirms S still reports mode=S.

### Q2: EVT name on VW keepalive loss

**Decision: Emit `EVT safety_stop` (preserve wire contract).**

Grep evidence: Multiple host test files assert `EVT safety_stop` as the expected VW
keepalive-loss EVT:
- `host/tests/test_protocol_v2.py`: 8 assertions on `EVT safety_stop` / `"safety_stop"`.
- `host/tests/test_nezha_drive.py`: 12+ assertions; `stream_drive` terminates on it.
- `host/robot_radio/robot/protocol.py`: `wait_for_evt_done` accepts `safety_stop`.
- `host/robot_radio/robot/nezha.py`, `nezha_kinematic.py`: reference `safety_stop`.
- `tests/dev/test_vw_command.py`: 4 assertions on `EVT safety_stop` format.

Changing to `EVT done` would break all of the above. Wire contract is preserved.

Mechanism: `MotionCommand::setDoneEvt(const char* label)` added. `beginVelocity` calls
`_activeCmd.setDoneEvt("EVT safety_stop")` after configure. `configure()` resets
`_doneEvtLabel` to `"EVT done"` to ensure per-command cleanliness. The SOFT-stop
completion path (both converged and deadline paths) and HARD-stop path both use
`_doneEvtLabel` instead of the hardcoded `"EVT done"`.

## Acceptance Criteria

### DriveController

- [x] `_bvc (BodyVelocityController)` and `_activeCmd (MotionCommand)` are value members
  declared in DriveController private section (`_bvc` before `_activeCmd`).
- [x] Constructor initialises `_bvc(mc, cfg)`.
- [x] `beginVelocity(v, omega, now_ms, target, fn, ctx, corr_id)`:
  - Calls `_activeCmd.configure(v, omega, &_bvc)`.
  - Adds a TIME stop condition: `a = (float)_cfg.sTimeoutMs`.
  - Calls `_activeCmd.setReplySink(fn, ctx, corr_id)`.
  - Calls `_activeCmd.setStopStyle(SOFT)`.
  - Calls `_activeCmd.start(inputs, now_ms)`.
  - Does NOT call `beginStream` or set `_mode = STREAMING`.
  - Sets `_mode` to `VELOCITY` (new `DriveMode::VELOCITY = 5`), which does NOT match
    `STREAMING` — STREAMING watchdog does not fire for VW.
- [x] VW keepalive re-send path in CommandProcessor: if `_activeCmd.active()`, calls
  `_activeCmd.setTarget(v, omega)` (which re-arms TIME and updates target) instead of
  calling `beginVelocity` from scratch. CommandProcessor detects this via
  `driveController.hasActiveCommand()` / `activeCmd()`.
- [x] `driveAdvance`: at the top of the tick, if `_activeCmd.active()`:
  - Compute `dt_s` from `now_ms - _lastTickMs`.
  - Call `_activeCmd.tick(inputs, now_ms, dt_s)`.
  - If `tick` returns false (terminated), set `_mode = IDLE`.
  - Return early (bypass the old S/T/D/G if-chain).
- [x] `cancel(uint32_t now_ms, ReplyFn fn, void* ctx)`:
  - Calls `_activeCmd.cancel(HARD)`.
  - Calls `_mc.stop()`.
  - Sets `_mode = IDLE`.
- [x] STREAMING watchdog branch in `driveAdvance` is guarded: fires only when
  `_mode == STREAMING` (i.e. only the `S` command triggers it).
- [x] `S` command and `beginStream` are completely unchanged.

### Safety watchdog parity
- [x] VW with no keepalive within `sTimeoutMs`: motors ramp to zero; safety EVT emitted
  (`EVT safety_stop` via `setDoneEvt`). (Bench-deferred; mechanism unit-verified.)
- [x] VW keepalive re-sends within `sTimeoutMs`: motor keeps running at new target
  (`setTarget` re-arms TIME baseline). (Bench-deferred; mechanism unit-verified.)
- [x] `S` command keepalive / safety_stop behaviour unchanged (existing test_vw_command.py
  / test_tlm_stream.py tests still pass).

### Host tests
- [x] `test_vw_command.py` updated: assert VW response is `OK vw`; no regression on
  existing parsing/response tests.
- [x] New host test or extended test: added `test_safety_stop_not_evt_done`,
  `test_safety_stop_keepalive_loss_format`, `test_vw_mode_is_V_not_S`, and
  `test_s_command_still_uses_streaming_mode` in `TestVWWatchdog`. All 38 VW tests pass.
- [x] All existing tests: `uv run --with pytest python -m pytest -q` → 1165 pass / 8 fail
  (same 8 pre-existing failures; +3 new passes vs 1162 baseline).

### Build
- [x] Clean build: `python3 build.py --clean` completes without errors.

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

DEFERRED per ticket spec. Mechanism unit-verified via host tests and build.
On-robot acceptance gates to be verified in a follow-up bench session:

- Flash robot (verify flash target is robot, not relay — see memory note).
- `VW 200 0` → robot ramps smoothly from rest; no instantaneous step.
- `VW 200 314` (arc) → smooth yaw ramp; curvature maintained.
- Stop sending keepalives → robot slows to a stop; `EVT safety_stop` received.
- `S 200 200` → still works, still uses STREAMING path, no regression.
- TLM `mode=V` shown during VW; `mode=S` shown during S.
