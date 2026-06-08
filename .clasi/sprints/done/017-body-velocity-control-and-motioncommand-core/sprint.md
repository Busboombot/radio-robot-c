---
id: '017'
title: Body velocity control and MotionCommand core
status: done
branch: sprint/017-body-velocity-control-and-motioncommand-core
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
issues:
- motion-command-body-velocity-control.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 017: Body velocity control and MotionCommand core

> **Roadmap sprint** — lightweight plan only. Full artifacts produced in detail
> planning before execution. Second of three: 016 → **017 (this)** → 018.
> Depends on 016 (built against the post-AppContext structure).

## Goals

Stand up the body-level motion engine and the `MotionCommand` object, and move the
first command (VW) onto it. Establishes the three core classes and the `(v, ω)`
twist + stop-condition model that 018 then migrates the remaining commands onto.

## Problem

Motion is commanded as raw per-wheel speeds applied instantly (jerks the chassis
from a dead stop); the only velocity profiler is an ad-hoc `_vRamped` trapezoid
buried in the go-to PURSUE loop; and per-command termination is a hand-written
`driveAdvance` if-chain with no way to compose or add stop conditions. See issue
`motion-command-body-velocity-control.md` (core scope).

## Solution

Three new classes in `source/control/`:
- **`BodyVelocityController`** — profiled `(v, ω)` ramp (trapezoid, S-curve-ready
  via jerk config defaulting to 0) → `BodyKinematics::inverse`/`saturate` →
  `MotorController::setTarget`. Owned by `DriveController`.
- **`StopCondition`** — POD tagged struct (TIME / DISTANCE / HEADING / POSITION /
  SENSOR), evaluated each tick against `HardwareState` + a captured baseline. A
  small fixed array per command; OR-across-array termination.
- **`MotionCommand`** — target `(v, ω)` + fixed stop-condition array + reply sink +
  reference to the velocity controller; lifecycle configure → start → tick →
  terminate (soft/hard).

Plus: new `Config.h` limit params (`vBodyMax`, `yawRateMax`, `yawAccMax`, `jMax`,
`yawJerkMax`) + SET/GET registry keys; wire **VW** onto a MotionCommand (replacing
the STREAMING watchdog with a re-armed safety TIME condition); add the **X**
cancel verb (hard-stop teardown) with STOP as an alias. `S` stays raw/unramped.

## Success Criteria

- Host unit tests: `BodyVelocityController` ramp slopes = `aMax`/`aDecel`, yaw
  obeys rate/accel limits; each `StopCondition` kind fires at threshold; OR-array;
  zero-condition command never self-terminates; SOFT vs HARD teardown.
- Clean build; bench: VW now *ramps* (not steps), respects yaw limits, keepalive
  loss safety-stops; `X`/`STOP` cancel immediately; `S` unchanged (no steer bias).

## Scope

### In Scope

- `BodyVelocityController`, `StopCondition`, `MotionCommand` classes + host tests.
- `Config.h` params + SET/GET registry keys.
- Wire VW onto MotionCommand; `X` cancel verb (+ STOP alias).
- Host protocol `cancel()` wrapper.

### Out of Scope

- R arc command; migrating G/T/D; TURN/HEADING and SENSOR stop verbs; S-curve
  enablement (provisioned but off) — all sprint 018.
- `S` raw path stays unchanged.

## Test Strategy

Pure-Python host unit tests mirroring `test_velocity_controller.py` /
`test_body_kinematics.py` for the controller, stop conditions, and command
lifecycle. Bench verification of VW ramp + cancel via `uv run rogo`; clean build
before flash; verify flash target is the robot not the relay.

## Architecture Notes

Profiler lives in `BodyVelocityController` (owned by DriveController), *referenced*
by the active `MotionCommand` — not buried in MotorController. `advance()` ticks on
the PID `dt` only (dual-clock hazard). `tgtLMms/R` written every tick (wedge/coast
gates). Twist model is `(v, ω)`; ratio is at most a thin parse-time adapter. Single
owned `MotionCommand` + controller instance (no heap). Detailed math, field layout,
and risks finalized in the architecture-update during detail planning.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Config params and kRegistry entries for body motion limits | — |
| 002 | BodyVelocityController — trapezoid profiler and host unit tests | 001 |
| 003 | StopCondition and MotionCommand — core classes and host unit tests | 002 |
| 004 | Wire VW onto MotionCommand in DriveController | 003 |
| 005 | X cancel verb and host cancel() wrapper | 004 |

Tickets execute serially in the order listed.
