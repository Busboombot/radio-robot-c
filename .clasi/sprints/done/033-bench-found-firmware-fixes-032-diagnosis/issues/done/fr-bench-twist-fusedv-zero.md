---
status: done
sprint: '033'
tickets:
- 033-003
---

# Bench finding — `twist=` (fused body velocity) reads 0,0 on hardware during driving

## Context

Found during sprint 032 hardware bench validation (firmware v0.20260612.17, robot `tovez`).
Across the entire run, the `twist=` TLM field (firmware `buildTlmFrame`: `twist=fusedV,fusedOmega*1000`)
read `0,0` on EVERY frame — including while the robot was actively driving with non-zero wheel
velocities (e.g. `vel=-209,194`, `vel=234,120`). The per-wheel `vel=` field is populated correctly; only
the fused body-velocity `twist=` stays at zero.

So `state.inputs.fusedV` / `fusedOmega` never become non-zero on hardware. The EKF velocity fusion
(sprint 023) appears not to be populating the fused-velocity state, or the twist field reads a variable
that is never updated on the real device. EKF `ekf_rej` stayed 0 throughout, so it is not a gate storm.

## To investigate

- Trace `fusedV`/`fusedOmega` from the EKF velocity update (sprint 023) to `state.inputs` to
  `buildTlmFrame` (`source/robot/Robot.cpp`). Confirm whether the velocity correction runs on hardware
  (it may be gated on OTOS velocity validity — on the stand the real OTOS velocity is ~0/frozen, which
  could be suppressing the fused-velocity update; cf. N9 same-tick OTOS gating in 030-008).
- Note interaction with the EKF Q dt-rescale (030-009/N15) — confirm that didn't zero the velocity-state
  growth.

## Acceptance

- On hardware, `twist=` shows a non-zero fused body velocity that tracks the commanded/encoder motion
  during a drive (and returns to 0 at rest). Add a sim/hardware check so it can't silently read 0.
