---
id: '002'
title: Add BodyKinematics module with inverse/forward maps and saturation scaling
status: done
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add BodyKinematics module with inverse/forward maps and saturation scaling

## Description

There is no single source of truth for the `(v, ω) ↔ (vL, vR)` conversion.
Each drive mode computes wheel speeds ad hoc. This ticket creates a stateless
`BodyKinematics` module that centralizes the differential-drive inverse and
forward kinematic maps, plus the curvature-preserving saturation scaler from
§1.7 of `docs/kinematics-model.md`.

This module is the foundation for Ticket 003 (`VelocityController`) and Sprint
011 (pose controller) — both need a canonical `(v, ω) → (vL, vR)` path.

## Acceptance Criteria

- [x] `source/control/BodyKinematics.h` and `.cpp` created.
- [x] `BodyKinematics::inverse(v, omega, b, vL_out, vR_out)` implements
  `vL = v - omega*(b/2)`, `vR = v + omega*(b/2)`.
- [x] `BodyKinematics::forward(vL, vR, b, v_out, omega_out)` implements
  `v = (vR+vL)/2`, `omega = (vR-vL)/b`.
- [x] `BodyKinematics::saturate(vL, vR, vWheelMax, steerHeadroom, vL_out, vR_out)`
  scales both wheel speeds by `s = (vWheelMax - steerHeadroom) / max(|vL|, |vR|)`
  when `max(|vL|, |vR|) > (vWheelMax - steerHeadroom)`; passes through
  unchanged otherwise.
- [x] New `RobotConfig` fields: `vWheelMax` (default 400.0 mm/s),
  `steerHeadroom` (default 20.0 mm/s).
- [x] Unit tests: inverse then forward round-trip returns original `(v, ω)`;
  saturation with `vL=300, vR=500, vWheelMax=400, headroom=20` scales both by
  `380/500 = 0.76`; curvature `κ = (vR-vL)/(b*(vR+vL)/2)` is preserved after
  scaling.
- [x] No heap allocation; all functions are pure (no internal state).

## Implementation Plan

**Approach**: New `.h/.cpp` pair in `source/control/`. Stateless free functions
or a class with only static methods. No `Robot` or `MotorController` changes in
this ticket.

**Files to create**:
- `source/control/BodyKinematics.h` — declare inverse, forward, saturate.
- `source/control/BodyKinematics.cpp` — implement; include `<math.h>` only.

**Files to modify**:
- `source/types/Config.h` — add `vWheelMax`, `steerHeadroom` fields to
  `RobotConfig` struct and `defaultRobotConfig()`.

**Testing plan**:
- Unit tests in `tests/` (or equivalent host-side test): verify all three
  functions with known inputs. Use the curvature-preservation check as the
  regression anchor.

**Documentation updates**:
- Header doc comment cites §1.3 and §1.7 of `docs/kinematics-model.md`.
