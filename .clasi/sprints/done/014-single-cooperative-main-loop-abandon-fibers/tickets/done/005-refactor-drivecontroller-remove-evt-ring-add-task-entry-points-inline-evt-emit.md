---
id: '005'
title: 'Refactor DriveController: remove EVT ring, add task entry points, inline EVT
  emit'
status: done
use-cases:
- SUC-003
- SUC-004
- SUC-005
depends-on:
- '001'
- '003'
- '004'
github-issue: ''
issue: plan-single-cooperative-main-loop-abandon-fibers.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Refactor DriveController: remove EVT ring, add task entry points, inline EVT emit

## Description

Remove the EVT ring buffer from `DriveController` and replace event emission
with inline calls to the reply function captured in `TargetState.replyFn/replyCtx`.

Split `controlTick()` into two task entry points:
- `driveAdvance(HardwareState& inputs, MotorCommands& cmds, TargetState& target, uint32_t now_ms)` —
  advances S/T/D/G state machines; emits completions inline via
  `target.replyFn(msg, target.replyCtx)`.
- The OTOS-correct call (`Odometry::correct`) has already been lifted into
  `Robot::otosCorrect()` in ticket 004; `DriveController` no longer holds
  the `_otos` pointer or the `_lastOtosMs` timer.

The `begin*()` entry points are updated to write into `TargetState` instead of
only capturing the reply sink as private members. The captured reply sink
(`_driveFn`/`_driveCtx`/`_corrId`) may remain as private members for
compatibility, or be replaced by reads from `TargetState` — since `TargetState`
now holds `replyFn/replyCtx/corrId`, the private copies are redundant and should
be removed.

## Files to Modify

- `source/control/DriveController.h` — remove `_evtQueue`, `_evtHead`,
  `_evtTail`, `enqueueEvt`, `drainEvents`; remove `_otos` pointer and
  `_lastOtosMs`; update `controlTick` → `driveAdvance()` signature; update
  `begin*()` signatures to accept/write `TargetState&`.
- `source/control/DriveController.cpp` — implement the new entry points;
  replace `enqueueEvt()` calls with direct `target.replyFn()` calls; remove
  `drainEvents()`.
- `source/robot/Robot.{h,cpp}` — add `driveAdvance(now_ms)` task entry point
  that calls `_dc.driveAdvance(_state.inputs, _state.commands, _state.target, now_ms)`.
  Update `begin*()` drive action methods to pass `_state.target` references.
  Keep `Robot::controlTick()` stub working until ticket 007.

## Acceptance Criteria

- [x] `DriveController` has no `_evtQueue`, `_evtHead`, `_evtTail`,
  `enqueueEvt`, `drainEvents`, `_otos`, `_lastOtosMs` members.
- [x] `DriveController::driveAdvance(HardwareState&, MotorCommands&, TargetState&, uint32_t)`
  advances all S/T/D/G state machines and emits completions inline via
  `target.replyFn(msg, target.replyCtx)`.
- [x] `DriveController::controlTick()` (old signature) is removed.
- [x] `DriveController::setOtos()` and the OTOS-correct code path are removed
  (handled by `Robot::otosCorrect()` from ticket 004).
- [x] `begin*()` methods are updated to write `mode`, `deadline`, goal, and
  captured sink into `TargetState`.
- [x] Firmware builds cleanly; `main.cpp` / `Robot::controlTick()` continue
  to compile.
- [x] `uv run --with pytest python -m pytest` passes — specifically
  `test_motion_verbs_v2.py`, `test_pursuit_arc_steering.py`,
  `test_tlm_stream.py`.

## Implementation Plan

1. In `DriveController.h`, remove the ring buffer block (lines ~130–149 in
   current source), `_otos`, `_lastOtosMs`.
2. Add `driveAdvance(HardwareState& inputs, MotorCommands& cmds, TargetState& target, uint32_t now_ms)`.
3. In `DriveController.cpp`:
   - Implement `driveAdvance()`: copy the body of `controlTick()`, replacing:
     - `_driveFn/_driveCtx` with `target.replyFn/target.replyCtx`.
     - `_corrId` with `target.corrId`.
     - Every `enqueueEvt(str)` call with `target.replyFn(str, target.replyCtx)`.
     - Remove the OTOS block (lines guarded by `_otos != nullptr`).
   - Remove `enqueueEvt()`, `drainEvents()`, and the ring buffer helpers.
4. Update `begin*()` methods to also write the relevant `TargetState` fields
   (`mode`, `deadlineMs`, `targetX/Y/SpeedMms`, `distanceTargetMm`,
   `replyFn`, `replyCtx`, `corrId`) from their parameters.
5. In `Robot.cpp`, add `driveAdvance(now_ms)` that calls
   `_dc.driveAdvance(_state.inputs, _state.commands, _state.target, now_ms)`.
6. Update `Robot::controlTick()` stub to call `driveAdvance()` after
   `controlCollect()`.

## Testing Plan

- **Build verification**: `python build.py` — no new errors.
- **Automated tests**: `uv run --with pytest python -m pytest` — focus on
  `test_motion_verbs_v2.py` (S/T/D/G completions), `test_pursuit_arc_steering.py`
  (G command), `test_tlm_stream.py` (streaming watchdog / safety_stop).
- **Hardware bench**: Deferred to ticket 009.
