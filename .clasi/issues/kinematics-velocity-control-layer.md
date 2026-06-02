---
status: pending
sprint: '010'
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
`vWheelMax`, `steerHeadroom`, `lapsToMm` (if velocity sourced from chip).

## Verification

- Unit-test the inverse/forward kinematics and the saturation scaler (ratio /
  curvature preserved after scaling).
- Bench: command a body twist `(v, ω)`; measured wheel velocities track setpoints
  within tolerance; under a stalled/loaded wheel the robot **slows but holds its
  arc radius** instead of drifting off it.
