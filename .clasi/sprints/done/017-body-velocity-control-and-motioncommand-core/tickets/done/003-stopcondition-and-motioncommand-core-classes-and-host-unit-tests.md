---
id: '003'
title: "StopCondition and MotionCommand \u2014 core classes and host unit tests"
status: done
use-cases:
- SUC-003
- SUC-004
- SUC-006
depends-on:
- '002'
github-issue: ''
issue: motion-command-body-velocity-control.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 017-003: StopCondition and MotionCommand — core classes and host unit tests

## Description

Create `source/control/StopCondition.{h,cpp}` and `source/control/MotionCommand.{h,cpp}`.
These two classes are created and tested together because `MotionCommand` owns the
`StopCondition` array and exercises all stop kinds through its lifecycle. Neither class is
wired to any command verb yet — that happens in ticket 004.

Refer to `architecture-update.md` for field layouts, the `MotionBaseline` struct, and the
SOFT/HARD teardown sub-phases.

## Files to Create / Modify

- **Create** `source/control/StopCondition.h`
- **Create** `source/control/StopCondition.cpp`
- **Create** `source/control/MotionCommand.h`
- **Create** `source/control/MotionCommand.cpp`
- **Create** `tests/dev/test_stop_condition.py`
- **Create** `tests/dev/test_motion_command.py`

## Acceptance Criteria

### StopCondition

- [x] `MotionBaseline` struct: `t0Ms`, `enc0Mm`, `heading0Rad`, `pose0X`, `pose0Y`.
- [x] `StopCondition` with `Kind` enum (NONE, TIME, DISTANCE, HEADING, POSITION, SENSOR),
  `Cmp` enum (GE, LE), fields `a`, `b`, `ax`, `sensor`, `cmp`.
- [x] `evaluate()` implements each Kind per architecture-update.md:
  - NONE: always false.
  - TIME: `now_ms - base.t0Ms >= a`.
  - DISTANCE: `|(s.encLMm + s.encRMm) * 0.5 - base.enc0Mm| >= a` (raw encoder sum).
  - HEADING: `|wrap(s.poseHrad - base.heading0Rad - a)| < b`.
  - POSITION: Euclidean dist from `(ax, a)` to `(s.poseX, s.poseY)` < `b`.
  - SENSOR: compare named channel vs `a` using `cmp` (GE/LE).

### MotionCommand

- [x] `configure(v, omega, bvc*)` — store target and BVC pointer; clear stop array and
  active flag.
- [x] `addStop(const StopCondition&)` — append; returns false if full; assert in debug.
- [x] `setReplySink(fn, ctx, corrId)`.
- [x] `setStopStyle(StopStyle)` — default SOFT.
- [x] `armTime(now_ms)` — bumps `t0Ms` in the first TIME condition baseline.
- [x] `start(inputs, now_ms)` — snapshot `MotionBaseline`; call `bvc->setTarget(v, omega)`.
- [x] `setTarget(v, omega)` — update target + call `bvc->setTarget`; re-arm TIME condition.
- [x] `tick(inputs, now_ms, dt_s)` — advance BVC; evaluate stops; handle teardown;
  emit EVT; return `active()`.
- [x] `cancel(StopStyle)` — HARD: emit `EVT cancelled`, go IDLE immediately.
- [x] `active()` — true while running or during SOFT-stop ramp.
- [x] SOFT-stop absolute deadline: 3000 ms after stop-fire, force IDLE + emit EVT.
- [x] `kMaxStopConds = 4` constant.

### Host unit tests — `tests/dev/test_stop_condition.py`

- [x] TIME fires at threshold, not one tick before.
- [x] DISTANCE fires when `|enc_avg - enc0| >= threshold`; not one mm short.
- [x] HEADING fires within eps of target heading delta.
- [x] POSITION fires within radius; not just outside.
- [x] SENSOR GE fires when value >= threshold; LE fires when value <= threshold.
- [x] NONE always returns false.
- [x] OR-across-array: two conditions; first fires; second not yet satisfied.
- [x] Zero-condition command: no self-termination.

### Host unit tests — `tests/dev/test_motion_command.py`

- [x] SOFT teardown: on stop-fire, `(0,0)` targeted; active stays true during ramp;
  `EVT done` emitted when BVC mock `atTarget()` returns true.
- [x] SOFT absolute deadline: if BVC never reaches zero, EVT emitted after 3 s.
- [x] HARD cancel: `EVT cancelled` on same tick; `active()` false immediately.
- [x] `active()` false after full termination.
- [x] Recycled command (`configure` + `start` called twice): baseline resets; no residue
  from prior run.
- [x] `armTime`: TIME condition not re-fired within new `sTimeoutMs` window.

### Build and regression

- [x] Clean build: `python3 build.py --clean` completes without errors.
- [x] Host baseline: `uv run --with pytest python -m pytest -q` at 1035/8.

## Implementation Plan

1. Write `StopCondition.h` (define structs and enums) and `StopCondition.cpp` (implement
   `evaluate()` switch; add `wrap_angle` helper using `atan2f(sinf(x), cosf(x))`).
2. Write `MotionCommand.h` and `MotionCommand.cpp`:
   - Implement SOFT-stop sub-phase state machine (`_stopping`, `_softDeadlineMs`).
   - Private `emitDone(const char* tag)` helper (mirrors `DriveController::emitEvt`).
   - `DriveController` (not `MotionCommand`) calls `_mc.stop()` after `cancel()` — keep
     `MotionCommand` decoupled from `MotorController` directly.
3. Write `tests/dev/test_stop_condition.py` as pure-Python mirrors of each Kind's logic
   with synthetic state dicts; assert fire/no-fire at boundary values.
4. Write `tests/dev/test_motion_command.py`: Python `MotionCommand` state-machine mirror
   with mock `BodyVelocityController` (small Python class with `setTarget`, `advance`,
   `atTarget`, `reset`).
5. Run clean build and verify baseline.

## Notes

- HARD cancel in `MotionCommand::cancel()` calls `bvc->reset()` to zero the profiler. The
  actual `MotorController::stop()` call is made by `DriveController::cancel()` after
  returning from `_activeCmd.cancel(HARD)`. This keeps the class boundary clean.
- `MotionCommand` does not need a direct reference to `MotorController`.

## Bench Verification (stakeholder-deferred)

Not applicable — classes not yet wired to any command verb.
