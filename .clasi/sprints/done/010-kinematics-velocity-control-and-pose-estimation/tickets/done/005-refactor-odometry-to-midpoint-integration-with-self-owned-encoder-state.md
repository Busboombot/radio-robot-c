---
id: '005'
title: Refactor Odometry to midpoint integration with self-owned encoder state
status: done
use-cases:
- SUC-005
depends-on: []
github-issue: ''
issue: kinematics-pose-estimation-fusion.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Refactor Odometry to midpoint integration with self-owned encoder state

## Description

`Odometry::update(dL, dR, tw)` uses forward-Euler integration with the
start-of-tick heading. This biases the heading estimate during turns — the
longer the arc, the larger the accumulated error. The fix is midpoint
integration: use `θ_mid = θ + dθ/2` when computing the position increment.

Additionally, the previous-encoder snapshot (`_prevOdoEncL/R`) is currently
held in `DriveController`, which computes the deltas before passing them to
`Odometry`. This ticket moves that state into `Odometry` itself via a new
`predict(float encLMm, float encRMm, float trackwidthMm)` method, so
`Odometry` owns its full integration cadence.

This is a pure refactor of the predict step; OTOS fusion is added in
Ticket 006.

## Acceptance Criteria

- [x] `Odometry` has a new `predict(float encLMm, float encRMm, float
  trackwidthMm)` method that:
  - Computes `dL = encLMm - _prevEncL`, `dR = encRMm - _prevEncR`.
  - Updates `_prevEncL = encLMm`, `_prevEncR = encRMm`.
  - Applies midpoint integration: `dC = (dL+dR)/2`, `dθ = (dR-dL)/tw`,
    `θ_mid = _headingRad + dθ/2`, `_x += dC*cosf(θ_mid)`,
    `_y += dC*sinf(θ_mid)`, `_headingRad = wrapPi(_headingRad + dθ)`.
  - `wrapPi` keeps heading in `(-π, π]`: `atan2f(sinf(θ), cosf(θ))`.
- [x] `Odometry` internal state adds `float _prevEncL` and `float _prevEncR`,
  initialized to 0 in the constructor; reset to current encoder positions on
  `zero()` / `setPose()` (caller must pass current encoder values, or
  `Odometry::zero()` resets prev to 0 and DriveController snapshots encoders).
- [x] `DriveController` removes `_prevOdoEncL` and `_prevOdoEncR` private
  fields; its `tick()` calls `_odo.predict(encLMm, encRMm, cfg.trackwidthMm)`
  passing current encoder positions (not pre-computed deltas).
- [x] The old `Odometry::update(dL, dR, tw)` method is either removed or made
  private/deprecated; all call sites updated.
- [x] Unit test: drive a constant-radius arc for N ticks (constant `dL`, `dR`
  per tick); the midpoint result has a smaller position error than the old
  forward-Euler result for the same inputs (14 tests in
  tests/test_odometry_midpoint.py — straight line, pure rotation, arc
  comparison, 90° arc accuracy, getPose cdeg output).
- [x] Existing `getPose()`, `setPose()`, `zero()` semantics are unchanged.

## Implementation Plan

**Approach**: Atomic commit — `Odometry` gains `predict()` and new state,
`DriveController` drops `_prevOdoEncL/R` in the same commit. No intermediate
broken state.

**Files to modify**:
- `source/control/Odometry.h` — add `predict()` declaration; add `_prevEncL`,
  `_prevEncR` private fields; update doc comment.
- `source/control/Odometry.cpp` — implement `predict()`; update `zero()` to
  reset prev state; remove or deprecate `update()`.
- `source/control/DriveController.h` — remove `_prevOdoEncL`, `_prevOdoEncR`
  fields.
- `source/control/DriveController.cpp` — update `tick()` to call
  `_odo.predict(encLMm, encRMm, cfg.trackwidthMm)` with current encoder
  positions from `MotorController::getEncoderPositions()`.

**Testing plan**:
- Unit test: construct `Odometry`, call `predict()` with identical deltas for
  10 ticks on a 2:1 wheel ratio arc; compare final heading to geometric truth.
  Assert midpoint result has smaller angle error than forward-Euler equivalent.
- Confirm `getPose()` returns consistent x, y, h after the refactor.

**Documentation updates**:
- `Odometry.h` doc comment: cite §2.4 of `docs/kinematics-model.md` for
  midpoint integration formula.
