---
id: '005'
title: Migrate CommandProcessor to AppContext
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-007
depends-on:
- '004'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate CommandProcessor to AppContext

## Description

Migrate `CommandProcessor` from `Robot&` to `AppContext&`. This is the largest
caller (~55–60 sites) and the last caller to migrate before `Robot` can be
deleted. After this ticket, no file outside `Robot.h`/`Robot.cpp` references
`Robot` (except the old Robot construction in `main.cpp`, which is cleaned up
in T006).

Work verb-by-verb through `CommandProcessor.cpp`, applying the inline
deletions described in the architecture update. The changes are mechanical
substitutions — no logic changes.

### CommandProcessor.h changes

1. `class Robot;` forward declaration → `struct AppContext;`
2. `explicit CommandProcessor(Robot& robot);` → `explicit CommandProcessor(AppContext& robot);`
3. `Robot& _robot;` → `AppContext& _robot;`

### CommandProcessor.cpp changes

1. `#include "Robot.h"` → `#include "AppContext.h"`
2. Constructor: `CommandProcessor::CommandProcessor(Robot& robot)` →
   `CommandProcessor::CommandProcessor(AppContext& robot)`

Verb-by-verb substitution table (all sites from reading CommandProcessor.cpp):

| Verb / site | Old | New |
|------------|-----|-----|
| PING | `_robot.systemTime()` | `_robot.systemTime()` (unchanged — kept method) |
| ID caps block | `_robot.otos()` (returns `OtosSensor*`) | `_robot.otos.is_initialized()` |
| ID caps block | `_robot.lineSensor()` | `_robot.line.is_initialized()` |
| ID caps block | `_robot.colorSensor()` | `_robot.color.is_initialized()` |
| ID caps block | `_robot.servo()` | **Omit `gripper` from `caps=`** — comment out the gripper entry (stakeholder: no gripper hardware now or for the foreseeable future). The `Servo` is still constructed/bound on its port in `main.cpp`; only its advertisement in `caps=` is removed. Leave a comment so it can be re-enabled later. |
| GET VEL | `_robot.state().inputs.velLMms` | `_robot.state.inputs.velLMms` |
| GET VEL | `_robot.state().inputs.velRMms` | `_robot.state.inputs.velRMms` |
| GET | `_robot.config()` | `_robot.config` |
| SET | `_robot.config()` | `_robot.config` |
| SET | `_robot.motor()` | `_robot.motorController` |
| STREAM | `_robot.config().tlmPeriodMs` | `_robot.config.tlmPeriodMs` |
| STREAM | `_robot.config().tlmFields` | `_robot.config.tlmFields` |
| SNAP | `_robot.buildTlmFrame(...)` | `_robot.buildTlmFrame(...)` (unchanged — kept method) |
| DBG I2C | `_robot.motor().stuckCountL()` | `_robot.motorController.stuckCountL()` |
| DBG I2C | `_robot.motor().stuckCountR()` | `_robot.motorController.stuckCountR()` |
| DBG I2C | `_robot.motor().resetStuckCounters()` | `_robot.motorController.resetStuckCounters()` |
| S | `_robot.streamDrive(l, r, fn, ctx)` | `_robot.driveController.beginStream((float)l, (float)r, _robot.systemTime(), _robot.state.target, fn, ctx)` |
| T | `_robot.timedDrive(l, r, ms, fn, ctx, corr_id)` | `_robot.driveController.beginTimed((float)l, (float)r, (uint32_t)ms, _robot.systemTime(), _robot.state.target, fn, ctx, corr_id)` |
| D | `_robot.distanceDrive(l, r, mm, fn, ctx, corr_id)` | `_robot.distanceDrive(l, r, mm, fn, ctx, corr_id)` (unchanged — kept method with workaround) |
| G | `_robot.goTo(x, y, speed, fn, ctx, corr_id)` | `_robot.driveController.beginGoTo((float)x, (float)y, (float)speed, _robot.systemTime(), _robot.state.target, fn, ctx, corr_id)` |
| VW | `_robot.velocityDrive(v, omega_rads, fn, ctx, corr_id)` | `_robot.driveController.beginVelocity(v, omega_rads, _robot.systemTime(), _robot.state.target, fn, ctx, corr_id)` |
| STOP | `_robot.stop()` | `{ uint32_t now = _robot.systemTime(); _robot.driveController.stop(now, [](const char*, void*){}, nullptr); }` |
| GRIP set | `_robot.setGripperAngle(deg)` | `{ uint8_t clamped = (deg < 0) ? 0 : (deg > 180) ? 180 : (uint8_t)deg; _robot.gripper.setAngle(clamped); }` |
| GRIP query | `_robot.gripperAngle()` | `_robot.gripper.currentAngle()` |
| ZERO enc | `_robot.zeroEncoders()` | `_robot.motorController.resetEncoderAccumulators()` |
| ZERO pose | `_robot.zeroOdometry()` | `_robot.odometry.zero(_robot.state.inputs)` |
| OI | `OtosSensor* otos = _robot.otos(); if (!otos) {...}` | `if (!_robot.otos.is_initialized()) { replyErr(...); return; }` then call `_robot.otos.init()` |
| OZ | same pattern | `_robot.otos.setPositionRaw(0, 0, 0)` |
| OR | same pattern | `_robot.otos.resetTracking()` |
| OP | same pattern | `_robot.otos.getPositionRaw(ox, oy, oh)` |
| OV | same pattern | `_robot.otos.setPositionRaw(ox, oy, oh)` |
| OL get | same pattern | `_robot.otos.getLinearScalar()` |
| OL set | same pattern | `_robot.otos.setLinearScalar(val)` |
| OA get | same pattern | `_robot.otos.getAngularScalar()` |
| OA set | same pattern | `_robot.otos.setAngularScalar(val)` |
| P | `_robot.portIO().setDigital(...)` | `_robot.portio.setDigital(...)` |
| P | `_robot.portIO().readDigital(...)` | `_robot.portio.readDigital(...)` |
| PA | `_robot.portIO().setAnalog(...)` | `_robot.portio.setAnalog(...)` |
| PA | `_robot.portIO().readAnalog(...)` | `_robot.portio.readAnalog(...)` |

**Gripper presence in ID caps= (stakeholder decision):** **Omit `gripper` from
`caps=`** — comment out the gripper advertisement entirely. There is no gripper
hardware now and none planned for the foreseeable future, so do NOT add
`Servo::is_initialized()`. The `Servo` object is still constructed and bound to
its port in `main.cpp` (so the wiring is ready when a gripper is added later);
only its listing in `caps=` is removed. Leave a `// gripper: omitted from caps —
no gripper hardware (re-enable when added)` comment at the commented-out site.

### main.cpp change

Update `CommandProcessor` construction to pass `appCtx`:
```cpp
// Old:
static CommandProcessor cmd(robot);  // robot = Robot type
// New:
static CommandProcessor cmd(appCtx); // appCtx = AppContext type
```

The old `static Robot robot(...)` line can now be deleted (or kept temporarily
until T006 verifies nothing else references it). The safest approach for this
ticket: delete the `Robot robot(...)` line and `Communicator` argument since
`AppContext` does not take `Communicator`. The `comm` static remains (used by
`LoopScheduler`).

## Acceptance Criteria

- [x] `CommandProcessor.h` uses `struct AppContext;` forward declaration and
      `AppContext& _robot` member.
- [x] `CommandProcessor.cpp` includes `AppContext.h` (not `Robot.h`).
- [x] All OTOS verb handlers use `_robot.otos.is_initialized()` instead of
      nullable pointer accessor.
- [x] Drive verb handlers (S, T, G, VW) call `driveController.beginX(...)`
      directly (no `Robot::streamDrive`, `timedDrive`, `goTo`, `velocityDrive`).
- [x] `D` verb still calls `_robot.distanceDrive(...)` (the kept method).
- [x] `STOP` verb calls `_robot.driveController.stop(now, noop, nullptr)`.
- [x] `GRIP` set calls `_robot.gripper.setAngle(clamped)`.
- [x] `GRIP` query calls `_robot.gripper.currentAngle()`.
- [x] `gripper` is omitted (commented out) from the `ID caps=` block, with a
      re-enable comment; the `Servo` is still constructed/bound on its port in
      `main.cpp`. No `Servo::is_initialized()` added.
- [x] `ZERO enc` calls `_robot.motorController.resetEncoderAccumulators()`.
- [x] `ZERO pose` calls `_robot.odometry.zero(_robot.state.inputs)`.
- [x] `P` / `PA` call `_robot.portio.*` directly.
- [x] `main.cpp` passes `appCtx` to `CommandProcessor`; old `Robot robot(...)`
      line is deleted.
- [x] Clean build: `python3 build.py` passes.
- [x] Host unit tests pass: `uv run --with pytest python -m pytest`.
- [ ] DEFERRED (bench): PING/ID caps/S/T/D/G/VW/STOP/GRIP/ZERO/OTOS/PORT/STREAM/SNAP
      on-robot smoke test. No gripper hardware — confirm `gripper` absent from
      `ID caps=`. Defer to stakeholder.

## Implementation Plan

**Approach**: Verb-by-verb substitution. Work top-to-bottom through
`CommandProcessor.cpp`. After each verb group, do a local build check
(`python3 build.py`) to catch errors early before the full diff grows large.

**Files to modify**:
- `source/app/CommandProcessor.h` — forward decl + member type swap
- `source/app/CommandProcessor.cpp` — include swap + all verb substitutions
- `source/main.cpp` — update `CommandProcessor cmd(appCtx)`; delete `Robot robot(...)` line

**Files NOT to touch**: `Robot.h`, `Robot.cpp` (these are deleted in T006),
`AppContext.h/.cpp`, `LoopScheduler`, `WedgeTest`.

**Testing plan**:
- `python3 build.py` — clean build; no `Robot` references remain outside
  `Robot.h`/`Robot.cpp`.
- `uv run --with pytest python -m pytest` — no regressions.
- Bench smoke test (all drive + sensor commands): PING, ID (check caps=),
  S/T/D/G/VW/STOP, GRIP set+query, ZERO enc+pose, OI/OZ/OR/OP/OV/OL/OA,
  P/PA, STREAM/SNAP.

**Documentation updates**: None required.
