---
id: "008"
title: "Motor/HAL Layer: Vendor Coverage, Chip Velocity and Cleanup"
status: roadmap
branch: sprint/008-motor-hal-layer-vendor-coverage-chip-velocity-and-cleanup
use-cases: []
issues:
  - nezha-chip-velocity-readspeed-0x47.md
  - nezha-full-vendor-i2c-coverage.md
  - source-fixme-cleanup.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 008: Motor/HAL Layer — Vendor Coverage, Chip Velocity and Cleanup

## Goals

Bring the motor/HAL layer to full vendor capability and complete the
naming/structure cleanup that rides on the new `RobotConfig`:

- **Vendor the advisory `pxt-nezha2` reference** into
  `vendor/pxt-nezha2/` (advisory, not compiled) so the vendor I2C command
  surface is auditable in-repo.
- **Wrap all remaining vendor I2C commands** in the HAL: `0x70` timed
  move, `0x5D` move-to-absolute-angle (preserving the BUG-critical
  post-write delay with no task interleave), `0x1D` reset/home, `0x77`
  global servo speed, `0x88` read firmware version. Maintain a green
  vendor-reg → HAL-method coverage checklist.
- **Chip-native wheel velocity (`0x47` `readSpeed`)**: `readSpeedRaw` +
  `readSpeed(leftWheel, mmPerSec)` with the `floor(raw/3.6)*0.01` laps/s
  conversion, 4 ms pre/post delays, per-wheel sign; **chip velocity
  primary, encoder-delta/dt fallback** on I2C error or implausible
  reading; expose which source is live. Pin the laps→mm/s scale
  empirically on the bench.
- **`NezhaV2` → per-motor `Motor` abstraction**: rename class/file; one
  motor per object owning its own state; **per-motor forward direction as
  a `RobotConfig` value** instead of hardcoded `LEFT_FWD`/`RIGHT_FWD`.
- **`GripperServo` → `Servo`**: rename; make range configurable for
  180° vs 360° (continuous-rotation) servos.
- **Line-sensor calibration**: min/max sweep capture + per-channel
  normalization, optional smoothing.

## Issues Addressed

- `nezha-chip-velocity-readspeed-0x47.md` — chip velocity (0x47) +
  encoder-delta fallback (the priority item).
- `nezha-full-vendor-i2c-coverage.md` — vendor the reference and wrap the
  remaining I2C commands.
- `source-fixme-cleanup.md` — the underlying Motor/Servo/LineSensor
  refactors behind the removed FIXMEs (comments already removed; this
  sprint does the work).

## Rationale for Grouping

All three issues are HAL/motor-layer work and cohere tightly: the chip
velocity read feeds the kinematics inner loop later, and the
`source-fixme-cleanup` refactors (per-motor `Motor`, per-motor direction
in `RobotConfig`, `Servo` rename, line-sensor calibration) land naturally
alongside the Motor/vendor work rather than as a separate pass. The FIXME
omnibus explicitly cross-links the Motor abstraction and the vendor
methods, so co-scheduling avoids touching the same files twice.

## Dependency Notes

- **Depends on:** 007 — needs `RobotConfig` (per-motor direction,
  velocity-source toggle, geometry constants) and the
  Motor/VelocityController scaffolding to land the per-motor abstraction.
- **Blocks:** 010 — supplies the clean per-wheel chip-velocity signal the
  velocity PID consumes.

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed. (Populated in detail mode.)
