---
id: '011'
title: 'Kinematics: Pose Control (Go-To)'
status: planning-docs
branch: sprint/011-kinematics-pose-control-go-to
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
issues:
- kinematics-pose-control-goto.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 011: Kinematics — Pose Control (Go-To)

## Goals

The capstone: closed-loop go-to, living in `DriveController` and expressed
via v2 commands (Layers 3–4 of the kinematics model):

- **Pursuit-arc steering**: goal in robot frame `(dx,dy)` → curvature
  `κ = 2·dy/(dx²+dy²)`; set `ω = v·κ`; **recompute every pose update**
  (receding horizon) rather than committing to a fixed arc.
- **Turn-in-place gate**: if the bearing to target exceeds
  `turnInPlaceGate`, rotate in place to roughly face it first — handles
  targets beside/behind the robot.
- **Accel/decel shaping**: online trapezoidal profile — slew `v` by
  `aMax·dt`, cap by `v_cap = sqrt(2·aDecel·d_remaining)`,
  `v = min(v_ramped, v_cap, v_user_max)` (one `sqrt`, no stored plan).
- **Arrival** within `arriveTolMm`; emit a completion event routed to the
  originating channel.
- Provide the **velocity command `(v, ω)`** primitive (watchdogged)
  alongside heading-free **go-to `(x, y)`**. Full pose `(x,y,θ)`
  regulator is explicitly out of scope.

## Issues Addressed

- `kinematics-pose-control-goto.md` — pursuit-arc go-to + turn-in-place
  gate + online accel/decel + arrival tolerance + the `(v,ω)` primitive.

## Rationale for Grouping

This is the capstone go-to behavior and the natural terminal sprint of
the kinematics arc. It is a single issue, sequenced last because it
depends on both the velocity-control and pose-estimation layers
delivered in 010 and rides on the v2 command surface from 009.

## Dependency Notes

- **Depends on:** 010 — consumes the velocity-control `(v,ω)` input +
  saturation scaler and steers off the fused authoritative pose; 009 —
  the go-to / `(v,ω)` commands are authored in the v2 wire format.
- Transitively depends on 007 (this logic lives in `DriveController`).
- **Blocks:** none (terminal sprint of this roadmap).

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | RobotConfig: add aMax, aDecel, turnInPlaceGate, arriveTolMm fields and registry entries | — |
| 002 | DriveController: pursuit-arc steering law (receding-horizon curvature, replace computeArc) | 011-001 |
| 003 | DriveController: turn-in-place gate (PRE_ROTATE reactivation on bearing threshold) | 011-002 |
| 004 | DriveController: online trapezoidal accel/decel shaping and arrival detection | 011-003 |
| 005 | VW command: watchdogged (v,ω) velocity primitive in v2 protocol | 011-001 |
| 006 | Bench verification: go-to end-to-end from 3 start positions and VW drive | 011-004, 011-005 |

Tickets execute serially in the listed order. Ticket 005 depends only on
011-001 and may be developed in parallel with 002–004 if two programmers
are available; the serial ordering above is safe for a single programmer.
