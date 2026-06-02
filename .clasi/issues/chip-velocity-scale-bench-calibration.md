---
status: pending
---

# Chip-Velocity Scale (lapsToMmScale) Bench Calibration + Velocity Readout

**Follow-up from sprint 008, ticket 003 (chip-native wheel velocity via readSpeed
0x47).** The chip-velocity path works on hardware (the robot drives with readSpeed
enabled in the control loop, with a safe encoder-delta fallback), but its scale
constant is **provisional**.

## Context

`RobotConfig::lapsToMmScale` defaults to `1980.0f` — a *theoretical* estimate
(wheel circumference ≈ 200 mm × typical gear ratio), not empirically pinned. The
sprint-008 architecture review explicitly flagged: "laps-to-mm/s scale requires
empirical bench pinning before the chip-velocity ticket is trusted." It could not
be calibrated during the sprint-008 bench session because **there is no serial
command that reports chip velocity** — `readSpeed`/`getActualVelocity` are used
internally by `MotorController` but not surfaced to the host.

## Scope

1. **Add a velocity-readout serial command** (e.g. `V` → reports per-wheel mm/s and
   which source is live, mirroring `getVelocitySourceFlags`). Needed to observe
   chip velocity on the bench.
2. **Bench-calibrate `lapsToMmScale`**: drive at known PWMs (e.g. 20/50/80), record
   `(raw 0x47 reading, encoder-derived mm/s)` per wheel, compute
   `lapsToMmScale = encoder_mmps / (floor(raw/3.6) * 0.01)` at mid-range, update
   `defaultRobotConfig()`.
3. Re-verify on the stand that chip velocity tracks encoder-derived velocity.

## Notes

- Not urgent: the control loop has a safe encoder-delta fallback and an
  implausibility gate, so a wrong scale does not endanger driving.
- See [[clean-build-before-bench]] — bench-test only clean builds.
