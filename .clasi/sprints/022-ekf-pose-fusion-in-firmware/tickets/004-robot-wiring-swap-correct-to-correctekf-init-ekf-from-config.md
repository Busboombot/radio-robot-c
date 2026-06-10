---
id: '004'
title: Robot wiring (swap correct to correctEKF, init EKF from config)
status: done
use-cases:
- SUC-002
- SUC-003
depends-on:
- '003'
completes_issue: false
---

# T004: Robot wiring (swap correct to correctEKF, init EKF from config)

## Description

Wire the EKF into the robot runtime. Two changes to `Robot.cpp`:

1. `Robot::otosCorrect()` switches from calling `odometry.correct()` (the fixed-
   alpha complementary filter) to `odometry.correctEKF()` (the Kalman update).
2. The `Robot` constructor calls `odometry.initEKF()` with the three new config
   fields so the EKF is ready before the first `predict()` or `correctEKF()` tick.

No changes to `Robot.h`.

## Acceptance Criteria

- [x] `source/robot/Robot.cpp` — `Robot::otosCorrect()` replaces:
  ```cpp
  odometry.correct(state.inputs, p.x, p.y, p.h,
                   config.alphaPos, config.alphaYaw, config.otosGate);
  ```
  with:
  ```cpp
  odometry.correctEKF(state.inputs, p.x, p.y);
  ```
  The lines storing `state.inputs.otosX/Y/H` and `state.inputs.otos.lastUpdMs/valid`
  are unchanged (OTOS heading is still stored for telemetry).
- [x] `source/robot/Robot.cpp` — `Robot::Robot()` (constructor body) includes a
  call to `odometry.initEKF(config.ekfQxy, config.ekfQtheta, config.ekfROtosXy)`
  after the `odometry` member is constructed.
- [x] `source/robot/Robot.h` is NOT modified (no new declarations needed; the
  `Robot` constructor already takes `const RobotConfig& cfg`).
- [x] Firmware builds cleanly: `python3 build.py`.
- [x] Full test suite passes: `uv run --with pytest python -m pytest`.

## Implementation Plan

### Approach

**Locating the Robot constructor:**

Read `Robot.cpp` to find `Robot::Robot(Hardware& hal, const RobotConfig& cfg)`.
Add `odometry.initEKF(config.ekfQxy, config.ekfQtheta, config.ekfROtosXy);`
in the constructor body. Since `Robot` members are initialised in declaration
order and `odometry` has a default ctor, the value member is live at the point
the constructor body runs — this is the correct call site.

**Locating otosCorrect():**

`Robot::otosCorrect()` is at approximately line 168 of `Robot.cpp` (confirmed
from source reading). Replace the single `odometry.correct(...)` call. Do not
touch the four lines above it that write `state.inputs.otosX/Y/H` and the
validity stamp.

### Files to modify

- `source/robot/Robot.cpp`

### Testing plan

```
python3 build.py
uv run --with pytest python -m pytest
```

The build confirms wiring compiles. The test suite confirms no regressions.
No new tests are added in this ticket; end-to-end correctness is verified by
bench telemetry during sprint close.

### Documentation updates

None required. The `otosCorrect()` comment block in `Robot.cpp` should be
updated to note "EKF correction" replaces "complementary correction":
```cpp
// Uses odometry.correctEKF() — EKF Kalman update (sprint 022).
// Replaces the fixed-alpha complementary blend (odometry.correct()).
```
