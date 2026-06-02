---
status: in-progress
sprint: '010'
tickets:
- 010-001
---

# Kinematics: Velocity-Control Layer (per-wheel PID + body kinematics + saturation)

Implements **Layers 1–2** of [docs/kinematics-model.md](../../docs/kinematics-model.md)
(§1.2, §1.3, §1.7, §2.1). First of three kinematics issues; siblings
[[kinematics-pose-estimation-fusion]] and [[kinematics-pose-control-goto]].

## Depends on

- [[firmware-architecture-refactor]] — provides the `VelocityController` class,
  `RobotConfig`, the `DriveController`, and the multi-rate scheduler this fills in.
- [[nezha-chip-velocity-readspeed-0x47]] — the clean per-wheel velocity signal the
  PID consumes (chip `readSpeed`; encoder-delta/dt fallback).

## Current state (the gap)

Today `MotorController` (`source/control/MotorController.cpp`) has **no per-wheel
velocity PID**. Its inner loop is a **cumulative-distance ratio cross-coupling**
controller (`RatioPidController` + `kFF`, `kScaleLF/LB/RF/RB`, `kAdjThreshold`,
`kAdjGain`) that equalizes *distance* between wheels, with velocity only computed
as `(encMm − prevEncMm)/dt`. There is no explicit body-twist `(v, ω)`
representation; wheel mm/s targets are set ad hoc by the command handlers.

## Scope (target)

1. **Per-wheel velocity PID** (`VelocityController`): setpoint = wheel mm/s,
   feedback = measured wheel velocity (chip velocity preferred). PI + feed-forward
   (`pwm = kFF·|sp| + kP·err + I`), anti-windup, low-speed deadband; output PWM%
   clamped ±100. Retire the ratio/`kAdj*` cross-coupling as the inner loop (the
   per-wheel loops + body kinematics replace its job).
2. **Body kinematics** (§1.3): `vL = v − ω·b/2`, `vR = v + ω·b/2` (inverse) and
   the forward map; single source for the `(v,ω)↔(vL,vR)` conversion. `b` =
   track width from `RobotConfig`.
3. **Saturation scaling** (§1.7): when `max(|vL|,|vR|) > vWheelMax`, scale **both**
   setpoints by `s = vWheelMax / max(|vL|,|vR|)` to preserve curvature (same arc,
   slower); keep a `steerHeadroom` reserve below the ceiling.

## Config additions (RobotConfig)

`vel.kP`, `vel.kI`, `vel.kFF` (per wheel ok), `minWheelMms` (deadband),
`vWheelMax`, `steerHeadroom`. **No velocity-scale field** — see chip-velocity note.

## Chip-velocity conversion fix (folded in from sprint 008, ticket 003)

Sprint 008 shipped `Motor::readSpeed` (0x47) but its mm/s conversion is wrong and
carries a bogus `RobotConfig::lapsToMmScale` (default `1980`, ~11× off). It only
feeds telemetry today (PWM never uses it; encoder-delta fallback), so it's
harmless until this layer consumes it — fix it here:

- The 0x47 raw value is the chip's **angular velocity in the same angular unit as
  the 0x46 angle register** (tenths of a degree). So velocity reuses the **already-
  calibrated `mmPerDegL/R`** — there is **no separate velocity calibration**:
  `mm/s = (raw / 10) · mmPerDeg · sign`, exactly mirroring `readEncoder`'s
  `(rawTenths/10) · mmPerDeg`.
- **Delete `lapsToMmScale`** from `RobotConfig`; drop the `floor(raw/3.6)*0.01`
  laps/s path (it also quantizes to 3.6°/s steps).
- **Bench-confirm the ×1 vs ÷10 unit factor** (vendor `readSpeed` formula reads as
  *whole* °/s, which would contradict the tenths 0x46 register — likely a vendor
  bug): command a steady speed, compare `raw·mmPerDeg` vs encoder-delta mm/s; if
  ~10× high it's tenths → divide by 10. This is a confirmation, not a calibration.
- Add a velocity-readout serial command (e.g. `V` → per-wheel mm/s + source flag)
  to make the bench check and PID tuning observable from the host.

## Verification

- Unit-test the inverse/forward kinematics and the saturation scaler (ratio /
  curvature preserved after scaling).
- Bench: command a body twist `(v, ω)`; measured wheel velocities track setpoints
  within tolerance; under a stalled/loaded wheel the robot **slows but holds its
  arc radius** instead of drifting off it.
