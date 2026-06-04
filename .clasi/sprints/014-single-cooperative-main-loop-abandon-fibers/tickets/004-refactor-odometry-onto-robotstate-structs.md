---
id: '004'
title: Refactor Odometry onto RobotState structs
status: done
use-cases:
- SUC-003
- SUC-004
depends-on:
- '001'
github-issue: ''
issue: plan-single-cooperative-main-loop-abandon-fibers.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Refactor Odometry onto RobotState structs

## Description

Update `Odometry::predict()` and `Odometry::correct()` to read encoder
positions from `HardwareState` and write pose (`poseX`, `poseY`, `poseHrad`)
back into `HardwareState`, rather than maintaining a separate internal pose
state that callers must explicitly read via `getPose()`.

`Odometry` retains its own `_prevEncL/_prevEncR` snapshot — it runs at a
different cadence than the control task (via the odometry-predict task), so
it needs its own prev-encoder tracking.

The `getPose()` / `setPose()` / `zero()` methods are updated to operate on the
`HardwareState.pose*` fields, or are removed in favor of direct struct access
through `Robot::state()`. The internal `_x`, `_y`, `_headingRad` floats move
to `HardwareState.poseX/Y/Hrad`.

`correct()` reads OTOS values from `HardwareState.otosX/Y/H` (written by the
otos-correct task entry point in `Robot`) rather than taking them as
parameters — or it retains parameter-based signature for testability and the
caller passes values read from the struct. The parameter-based approach is
preferred for testability.

## Files to Modify

- `source/control/Odometry.h` — add `predict(HardwareState&, float trackwidth)`
  and `correct(HardwareState&, float alphaPos, float alphaYaw, float otosGate)`
  overloads; mark or remove the `_x/_y/_headingRad` internal fields.
- `source/control/Odometry.cpp` — implement the new overloads; remove internal
  pose storage (reads/writes `HardwareState.pose*` directly).
- `source/robot/Robot.{h,cpp}` — add `odometryPredict()` and `otosCorrect(now_ms)`
  task entry points that call the new `Odometry` overloads with the state container.

## Acceptance Criteria

- [x] `Odometry::predict(HardwareState& inputs, float trackwidth)` reads
  `inputs.encLMm/R`, integrates midpoint dead-reckoning, and writes
  `inputs.poseX`, `inputs.poseY`, `inputs.poseHrad`. The midpoint integration
  math is unchanged.
- [x] `Odometry::correct(HardwareState& inputs, ...)` reads OTOS values
  (passed as parameters by the caller, who reads them from `inputs.otosX/Y/H`)
  and applies the complementary correction to `inputs.poseX/Y/Hrad`.
- [x] `Odometry` no longer has `_x`, `_y`, `_headingRad` private fields (they
  live in `HardwareState`).
- [x] `Robot::odometryPredict()` calls `_odo.predict(_state.inputs, cfg.trackwidthMm)`.
- [x] `Robot::otosCorrect(now_ms)` reads OTOS via `OtosSensor::getPositionRaw()`,
  writes `_state.inputs.otosX/Y/H`, calls `_odo.correct(_state.inputs, ...)`.
- [x] `getPose(int32_t& x, int32_t& y, int32_t& h)` is updated to read from
  `HardwareState.pose*` (or removed — callers may use `Robot::getPose()` which
  reads the struct).
- [x] `uv run --with pytest python -m pytest` passes — specifically
  `test_odometry_midpoint.py`, `test_otos_fusion.py`.
- [x] Firmware builds cleanly.

## Implementation Plan

1. In `Odometry.h/cpp`, add new overloads:
   - `predict(HardwareState& s, float trackwidthMm)`:
     - `dL = s.encLMm - _prevEncL`; `dR = s.encRMm - _prevEncR`; update prevs.
     - Run existing midpoint integration; write result to `s.poseX/Y/Hrad`.
   - `correct(HardwareState& s, float xOtos, float yOtos, float thetaOtos,
              float alphaPos, float alphaYaw, float otosGate)`:
     - Apply existing complementary correction to `s.poseX/Y/Hrad`.
   - Keep the old `predict(float encLMm, float encRMm, float trackwidth)` and
     `getPose()/setPose()/zero()` signatures intact for now; `setPose()` will
     need to write into `HardwareState` — pass a `HardwareState&` parameter
     or have `Robot` bridge the call.
2. Remove `_x`, `_y`, `_headingRad` from `Odometry` — the authoritative pose
   now lives in `HardwareState`. `_prevEncL/_prevEncR` stay.
3. In `Robot.cpp`, implement:
   - `odometryPredict()`: call `_odo.predict(_state.inputs, _config.trackwidthMm)`.
   - `otosCorrect(now_ms)`: if OTOS present, call
     `_otos.getPositionRaw(...)`, convert LSB → mm/rad, write
     `_state.inputs.otosX/Y/H`, update `_state.inputs.otos.lastUpdMs`, call
     `_odo.correct(_state.inputs, ...)`.
4. Update `Robot::getPose()` to read from `_state.inputs.pose*`.
5. Update `Robot::setPose()` / `zeroOdometry()` to write `_state.inputs.poseX/Y/Hrad`.

## Testing Plan

- **Build verification**: `python build.py` — no new errors.
- **Automated tests**: `uv run --with pytest python -m pytest` — focus on
  `test_odometry_midpoint.py`, `test_otos_fusion.py`.
- **Hardware bench**: Deferred to ticket 009.
