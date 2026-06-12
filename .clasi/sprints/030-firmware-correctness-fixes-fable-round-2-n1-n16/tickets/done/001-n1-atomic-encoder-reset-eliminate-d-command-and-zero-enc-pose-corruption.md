---
id: '001'
title: "N1: Atomic encoder reset \u2014 eliminate D-command and ZERO enc pose corruption"
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue:
- fr2-n1-atomic-encoder-reset.md
- d12-numerical-and-timing-hygiene.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# N1: Atomic encoder reset — eliminate D-command and ZERO enc pose corruption

## Description

Every `D` command teleports the fused pose backward by the prior segment's travel.
`MotionController::beginDistance()` resets hardware accumulators
(`MotionController.cpp:306`) and `Robot::distanceDrive()` zeroes
`state.inputs.encLMm/R` (`Robot.cpp:318-319`), but neither re-baselines
`Odometry::_prevEncL/_prevEncR`. On the same tick `Odometry::predict()` computes a
large negative delta and feeds it into pose integration and `EKF::predict()`.

`ZERO enc` (`Robot.cpp:752`) is worse: hardware accumulators and MotorController
baselines reset, but `state.inputs.encLMm/R` stay stale, causing the outlier filter
to freeze encoder reads until the fresh accumulator climbs back, then a pose jump.

Also covers d12 #4: `Odometry::setPose()` should snapshot current encoder inputs
into `_prevEncL/R` (currently assumes zero), so a `ZERO pose` after encoder travel
does not produce a jump on the next predict call.

## Acceptance Criteria

- [x] `Robot::resetEncoders()` added: atomically resets hardware accumulators,
      MotorController velocity baselines, `state.inputs.encLMm/R`, and
      `Odometry::_prevEncL/R` without touching pose.
- [x] `Robot::distanceDrive()` calls `resetEncoders()` (replaces the split reset).
- [x] `Robot::handleZero()` ZERO enc path calls `resetEncoders()`.
- [x] `Odometry::setPose()` snapshots current encoder inputs into `_prevEncL/R`.
      (Already implemented in sprint 023; verified it reads s.encLMm/s.encRMm.)
- [x] New sim regression test: D-then-G with OTOS fusion OFF — pose delta after D
      completes is less than 5 mm (no backward jump).
- [x] `ekf_rej` count after a D command with fusion ON is 0 in the sim test.
- [x] `ZERO enc` with nonzero accumulators: no frozen-encoder window on the
      following tick (sim test verifies no negative outlier rejection).
- [x] `python3 build.py` clean build passes with no new warnings.
- [x] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.

## Implementation Plan

### Approach

Add `Robot::resetEncoders()` as the single canonical encoder reset. Both existing
partial reset sites become a single call. No other callers or interfaces change.

### Files to modify

- `source/robot/Robot.h` — declare `void resetEncoders();`
- `source/robot/Robot.cpp`
  - Implement `resetEncoders()`: call `_mc.resetEncoderAccumulators()`, reset MC
    velocity baselines, set `state.inputs.encLMm = state.inputs.encRMm = 0`, call
    `_odometry.rebaselinePrev(0, 0)`.
  - `distanceDrive()`: replace split reset with `resetEncoders()`.
  - `handleZero()` ZERO enc branch: call `resetEncoders()`.
- `source/odometry/Odometry.h` — add `void rebaselinePrev(float encL, float encR);`
- `source/odometry/Odometry.cpp`
  - Implement `rebaselinePrev()`: sets `_prevEncL = encL; _prevEncR = encR;`
  - `setPose()`: after updating pose, snapshot
    `_prevEncL = _state.inputs.encLMm; _prevEncR = _state.inputs.encRMm;`
- `host_tests/` or `host/tests/` — add:
  - `test_encoder_reset_pose_continuity`: D-then-G fusion-OFF pose-jump regression.
  - `test_zero_enc_no_frozen_window`: ZERO enc + tick, assert clean (non-negative)
    delta and no outlier rejection.

### Testing plan

Run: `uv run --with pytest python -m pytest host_tests/ host/tests/ -v`

Confirm existing odometry/encoder sim tests still pass. Build: `python3 build.py`.

### Notes

- DefaultConfig.cpp is auto-generated from `tovez.json` — no changes needed here.
- Do NOT touch pose (x, y, theta) in `resetEncoders()` — only encoder state.
- Independent of ticket 002 (no shared files).
