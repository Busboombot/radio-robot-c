---
status: in-progress
sprint: '027'
tickets:
- 027-005
---

# D9 — Gate OTOS fusion on sensor validity (directly addresses spin-on-placement)

## Context

`OtosSensor::readTransformed/readVelocityTransformed` never read the chip's STATUS
register (the SparkFun OTOS exposes tilt-warning and optical-tracking-invalid bits).
A lifted or just-placed robot, dust, or a too-tall gap feeds zeros/garbage straight
into the EKF. The position update may be Mahalanobis-gated (D3), but the **velocity**
updates at v=0/ω=0 sit well inside the χ² gate and actively drag fused velocity to
zero, fighting the controller. I2C read failure leaves the int16 buffers at 0 →
reported pose `(−odomOffX, −odomOffY, 0)` with no error signal. This is the
"on placement" timing of the wild spin: place the robot → OTOS re-acquires with
garbage → heading/velocity poisoned → pre-rotate gate never settles.

## Fix (improvement-plan P2.1)

1. Read the STATUS register in `otosCorrect()`'s cadence *before* using pose/velocity;
   on warn/fatal or I2C read failure, set `state.inputs.otos.valid = false` and
   **skip fusion entirely** that tick.
2. Distinguish "I2C returned zeros" from a genuine (0,0,0) pose — propagate a bool
   from `readXYH` instead of silently keeping zeroed int16s.
3. While invalid > ~500 ms during active motion, emit a one-shot `EVT otos lost` so
   the host knows pose quality degraded.
4. Fix the mounting-offset transform: apply `odomOffX/Y` in the sensor frame rotated
   by current heading, not subtracted as world constants (dormant — offsets are 0 in
   `tovez.json` — but wrong; keep no-op for zero offsets).

## Acceptance

- **Hardware:** lift the robot mid-G → motion stops via its stop conditions,
  `EVT otos lost` appears, **no spin on placement**; pose recovers after `SI`/camera
  fix.

## Source
Defect **D9** in the 2026-06-11 sim2real review (+ scenario 4.2); fix P2.1.
Complements D5: D5 bounds the spin, D9 removes the bad input that triggers it.
