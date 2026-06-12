---
status: in-progress
sprint: '030'
tickets:
- 030-001
---

# FR2-N1 (High) — Atomic encoder reset: D command and `ZERO enc` corrupt world pose / EKF

## Context

Source: `docs/code_review/2026-06-12-Fable-correctness-review/findings.md` §N1.
(Also subsumes `d12` item #4, reset-ordering fragility.)

`MotionController::beginDistance()` calls `_mc.resetEncoderAccumulators()`
(`MotionController.cpp:306`) and `Robot::distanceDrive()` zeroes
`state.inputs.encLMm/R` (`Robot.cpp:318-319`), but **neither re-baselines
`Odometry::_prevEncL/_prevEncR`**. Queue dispatch runs before odometry in
`loopTickOnce`, so the same tick `Odometry::predict()` computes
`dL = 0 − _prevEncL` — a large negative delta equal to all travel since the last
reset — and feeds it straight into pose integration and `EKF::predict()` (no gate
on predict). Every `D` teleports the fused pose backward by the prior segment's
length; with OTOS fusion live the Mahalanobis gate rejects ~10 cycles and `ekf_rej`
climbs while a queued `G` drives toward a garbage frame; with OTOS off the
corruption is permanent until `SI`. Heading degrades too (dTheta picks up the stale
baseline differential).

`ZERO enc` (`Robot.cpp:752`) is worse: resets hardware accumulators + MotorController
baselines but leaves `state.inputs.encLMm/R` stale, so the outlier filter in
`controlCollectSplitPhase()` rejects every read until the fresh accumulator climbs
back — frozen encoders, velocity PID windup, then the same odometry jump.

## Fix

One robot-level `resetEncoders()` that **atomically** resets: hardware accumulators,
MotorController velocity baselines, `state.inputs.encLMm/R`, and re-baselines
`Odometry::_prevEncL/R` (without touching pose). Call it from both `distanceDrive()`
and `handleZero` (the `ZERO enc` path). Also fix `Odometry::setPose()` to snapshot
the *current* encoder inputs rather than assuming zero (d12 #4).

## Acceptance

- A `D`-then-`G` sequence with OTOS fusion OFF shows pose continuity (no backward
  jump) — sim regression test.
- `ZERO enc` with nonzero encoders produces no frozen-encoder window and no pose
  jump on the next tick.
- `ekf_rej` does not spike after a `D` command with fusion ON.
