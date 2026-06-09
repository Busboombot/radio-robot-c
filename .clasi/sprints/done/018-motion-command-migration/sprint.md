---
id: 018
title: Motion command migration
status: done
branch: sprint/018-motion-command-migration
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
issues:
- motion-command-body-velocity-control.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 018: Motion command migration

> **Roadmap sprint** — lightweight plan only. Full artifacts produced in detail
> planning before execution. Third of three: 016 → 017 → **018 (this)**.
> Depends on 017 (the MotionCommand / BodyVelocityController core).

## Goals

Migrate the remaining motion commands onto the `MotionCommand` + stop-condition
model built in 017, add the arc (`R`) command, and introduce the new
heading/sensor stop capabilities. After this sprint every motion command except
raw `S` runs on one uniform `(v, ω)` + stop-condition engine.

## Problem

017 builds the engine and moves only VW onto it. G still uses the inline
`_vRamped` ramp; T and D still use bespoke `driveAdvance` termination branches;
there is no arc primitive, no turn-to-heading, and no sensor-triggered stop. See
issue `motion-command-body-velocity-control.md` (migration scope).

## Solution

- **R arc command**: `R <speed> <radius>` as `(speed, speed·κ)`, `κ=1/radius`
  (`radius=0` ⇒ straight), with a soft-stop; arc is a thin `ω = v·κ` adapter on the
  017 core. Host `arc()` wrapper.
- **Migrate G**: replace inline `_vRamped` with a `POSITION`-stop MotionCommand
  whose pursuit hook updates `(v, ω = v·κ_bearing)` each tick; keep the
  `√(2·aDecel·d)` terminal decel cap. PRE_ROTATE stays a raw turn-in-place.
- **Migrate T and D** (separate tickets): `(L,R) → forward() → (v, ω)` at begin;
  `TIME` / `DISTANCE` stop conditions; re-verify the D-timeout heuristic tolerates
  ramp-up; DISTANCE uses raw (not filtered) encoder sum.
- **New stop verbs**: `TURN`/`HEADING` (turn-to-heading, `v=0, ω=±yawRate`,
  `HEADING` condition) and a `SENSOR`-stop ("drive until line/colour/OTOS reads X").
- **S-curve provisioning**: enable the jerk-limited path when `jMax`/`yawJerkMax`
  > 0 (defaults stay 0 ⇒ trapezoid).

## Success Criteria

- Host tests: `(speed,radius)→κ→inverse→saturate→(vL,vR)` incl. `radius=0` and
  signed radius (CCW sign); G pursuit tests still green; TURN/SENSOR conditions fire
  correctly.
- Clean build; bench: `R` straight/left/right/soft-stop arcs smooth from rest; G
  arcs in and decelerates cleanly (`_vRamped` removal regression); D reaches
  distance accurately without spasm or timeout; TURN stops at commanded heading; a
  SENSOR-stop halts on threshold; `S`/calibration scripts unchanged.

## Scope

### In Scope

- `R` arc command + host wrapper.
- Migrate G, T, D onto MotionCommand.
- `TURN`/`HEADING` and `SENSOR` stop-condition verbs.
- S-curve enablement behind the jerk config.

### Out of Scope

- The core classes themselves (built in 017).
- `S` raw path (stays unchanged).

## Test Strategy

Host unit tests for arc kinematics mapping, heading/sensor conditions, and the G
pursuit regression. Bench verification of R, G, D, TURN, and a SENSOR-stop via
`uv run rogo`; clean build before flash; confirm flash target is the robot.

## Architecture Notes

Builds directly on 017's `MotionCommand` / `BodyVelocityController` / `StopCondition`.
G's pursuit becomes a per-tick `setTarget` hook + `POSITION` stop rather than a
separate state machine; the terminal decel cap clamps `vTgt` handed to the
controller. `_vRamped` and the per-mode `driveAdvance` termination branches are
removed as commands migrate. Detailed per-command mapping and risks finalized in
the architecture-update during detail planning.

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
| 001 | R arc command — firmware verb + host wrapper | — |
| 002 | Migrate G go-to onto MotionCommand POSITION stop | 001 |
| 003 | Migrate T timed-drive onto MotionCommand TIME stop | 002 |
| 004 | Migrate D distance-drive onto MotionCommand DISTANCE stop with terminal decel | 003 |
| 005 | TURN verb — turn-to-heading with HEADING stop condition | 004 |
| 006 | sensor= modifier — attach SENSOR stop to T, D, TURN commands | 005 |
| 007 | S-curve activation in BodyVelocityController | 006 |

Tickets execute serially in the order listed.
