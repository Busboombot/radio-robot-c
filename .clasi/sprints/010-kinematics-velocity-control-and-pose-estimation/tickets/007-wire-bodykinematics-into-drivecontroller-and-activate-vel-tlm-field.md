---
id: "007"
title: "Wire BodyKinematics into DriveController and activate vel= TLM field"
status: open
use-cases:
- SUC-002
- SUC-003
depends-on:
- "010-003"
- "010-004"
- "010-006"
github-issue: ""
issue: ""
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wire BodyKinematics into DriveController and activate vel= TLM field

## Description

With Tickets 001–006 complete, all the modules exist but the full control path
is not yet end-to-end. This integration ticket:

1. Routes S-command `(v, ω)` body-twist inputs through `BodyKinematics::inverse`
   and `::saturate` before they reach `MotorController::setTarget()`.
2. Activates the `vel=` TLM sub-field (bit 2, `TLM_FIELD_VEL`) so the
   streaming telemetry frame emits per-wheel mm/s using corrected chip velocity.
3. Closes the two sprint issues (`kinematics-velocity-control-layer.md` and
   `kinematics-pose-estimation-fusion.md`) as the final integration ticket.

This is the acceptance-verification ticket for the sprint — the bench tests
here are the stakeholder's go/no-go.

## Acceptance Criteria

- [ ] `DriveController::beginStream(v, omega, ...)` (or the S-command handler)
  calls `BodyKinematics::inverse(v, omega, cfg.trackwidthMm, vL, vR)` then
  `BodyKinematics::saturate(vL, vR, cfg.vWheelMax, cfg.steerHeadroom, vL, vR)`
  before passing `(vL, vR)` to `MotorController`.
- [ ] The `vel=` TLM field (bit 2) emits `vel=<vL_mmps>,<vR_mmps>` using
  `MotorController::getActualVelocity()` (chip velocity when available, encoder-
  delta fallback otherwise).
- [ ] `STREAM fields=vel` enables the `vel=` field; it appears in the TLM
  frame alongside `enc=` and `pose=` when selected.
- [ ] [BENCH] `STREAM 50 fields=vel` streams per-wheel mm/s at 50 ms period;
  values are non-zero and sensible during forward drive.
- [ ] [BENCH] Command a body twist `S v=200 omega=0` (straight); both wheels
  track ~200 mm/s; robot drives straight.
- [ ] [BENCH] Command a body twist `S v=150 omega=0.5` (left-curving arc);
  `vL < vR`; robot curves left; under a load event the robot slows but holds
  the arc (does not drift straight).
- [ ] [BENCH] Drive a square loop using sequential `S` commands; final position
  error is measurably smaller with OTOS fusion active than encoder-only.
  (Confirms Ticket 006 is integrated end-to-end.)

## Implementation Plan

**Approach**: Integration wiring in `DriveController.cpp` and `Robot.cpp` (or
wherever the S-command `(v, ω)` tuple is parsed and dispatched). Activate the
`vel=` TLM branch in the telemetry assembly code (likely in `Robot::tick()`).

**Files to modify**:
- `source/control/DriveController.cpp` — update `beginStream()` and
  `beginGoTo()` to route through `BodyKinematics`.
- `source/app/Robot.cpp` (or wherever TLM is assembled) — activate
  `TLM_FIELD_VEL` branch: call `_mc.getActualVelocity()` and format
  `vel=%d,%d` (integer mm/s).
- `source/app/CommandProcessor.cpp` — update the S-command parser if it
  currently parses `v=` and `omega=` but passes them as raw left/right
  mm/s; if S already accepts left/right mm/s only, add a `(v, omega)` variant
  or route through the body kinematics at the dispatch site.

**Testing plan**:
- Full bench run per ACs above. This is the sprint's primary acceptance gate.
- Confirm no regressions on existing T/D commands (they set wheel speeds
  directly; no body-kinematics path for those).

**Documentation updates**:
- `docs/architecture.md` — update the Control Layer section to reflect
  `VelocityController`, `BodyKinematics`, and the predict/correct `Odometry`
  interface (or note that `architecture-update.md` for Sprint 010 is the
  authoritative delta).
