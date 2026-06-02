---
id: "003"
title: "Implement VelocityController and replace ratio PID inner loop in MotorController"
status: open
use-cases:
- SUC-003
depends-on:
- "010-001"
- "010-002"
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement VelocityController and replace ratio PID inner loop in MotorController

## Description

`MotorController.tick()` currently drives wheels using a cumulative-distance
ratio PID (`RatioPidController` + `kAdj*` cross-coupling). This is a distance-
equalization heuristic — it cannot hold a commanded wheel velocity, only
equalize accumulated travel. This ticket replaces the inner loop with a proper
per-wheel `VelocityController` (PI + feed-forward, anti-windup, deadband) and
wires `MotorController` to use it.

Depends on Ticket 001 (corrected chip-velocity signal as feedback) and
Ticket 002 (`BodyKinematics` providing the `(vL, vR)` setpoints).

## Acceptance Criteria

- [ ] `source/control/VelocityController.h` and `.cpp` created.
- [ ] `VelocityController` holds: `kP`, `kI`, `kFF` gains; integrator state;
  `iMax` clamp; `minWheelMms` deadband; `reset()` method.
- [ ] `VelocityController::update(setpoint, measured, dt_s) → float pwmPct`
  computes `pwm = kFF*|sp| + kP*err + I`; anti-windup freezes integrator
  when output is rail-limited (|pwm| >= 100); deadband suppresses integrator
  accumulation below `minWheelMms`; output clamped to ±100.
- [ ] `MotorController` holds two `VelocityController` instances (`_vcL`,
  `_vcR`).
- [ ] `MotorController.tick()` uses `_vcL.update(tgtL, measuredL, dt)` and
  `_vcR.update(tgtR, measuredR, dt)` for the PWM computation. The old ratio
  PID path (`_pid.update()`, `_cmdRatio`, `kAdj*`) is not called in normal
  drive.
- [ ] `RatioPidController` class is retained (compile passes) but bypassed —
  no active calls from `MotorController.tick()`.
- [ ] New `RobotConfig` fields: `velKp` (key `vel.kP`, default 0.3),
  `velKi` (key `vel.kI`, default 0.05), `velKff` (key `vel.kFF`, default
  0.15), `minWheelMms` (default 20.0 mm/s). Added to `Config.h` and
  `defaultRobotConfig()`.
- [ ] [BENCH] Command `S 200 200` (straight); measured per-wheel mm/s tracks
  the 200 mm/s setpoint within ±30 mm/s at steady state.
- [ ] [BENCH] Command a turning arc (e.g. `S 100 200`); robot holds arc
  without drifting off it under load.

## Implementation Plan

**Approach**: New `VelocityController` class; minimal changes to
`MotorController` structure — only `tick()` body rewritten and two
`VelocityController` members added.

**Files to create**:
- `source/control/VelocityController.h` — declare class, gains struct, `update()`, `reset()`.
- `source/control/VelocityController.cpp` — implement update logic.

**Files to modify**:
- `source/control/MotorController.h` — add `_vcL`, `_vcR` members; remove or
  comment out ratio-PID-specific private fields (`_cmdRatio`, `_fasterIsRight`,
  `_cmdEncStartL/R`, `gains.kRatio`) as appropriate.
- `source/control/MotorController.cpp` — rewrite `tick()` body; update
  constructor to initialize `VelocityController` instances with config gains;
  `stop()` calls `_vcL.reset()` and `_vcR.reset()`.
- `source/types/Config.h` — add `velKp`, `velKi`, `velKff`, `minWheelMms`
  to `RobotConfig` and `defaultRobotConfig()`.

**Testing plan**:
- Unit test `VelocityController::update` with mock setpoint/measured inputs:
  verify zero error → only FF term; positive error → integrator grows;
  saturation → integrator frozen.
- Bench: steady-speed and turning-arc tests per ACs above.

**Documentation updates**:
- Header comments cite §2.1 of `docs/kinematics-model.md`.
- `MotorController.h` comment updated: "inner loop is VelocityController
  (PI+FF); ratio PID retained but bypassed."
