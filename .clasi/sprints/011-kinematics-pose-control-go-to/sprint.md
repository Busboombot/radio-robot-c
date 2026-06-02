---
id: "011"
title: "Kinematics: Pose Control (Go-To)"
status: roadmap
branch: sprint/011-kinematics-pose-control-go-to
use-cases: []
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

Tickets execute serially in the order listed. (Populated in detail mode.)
