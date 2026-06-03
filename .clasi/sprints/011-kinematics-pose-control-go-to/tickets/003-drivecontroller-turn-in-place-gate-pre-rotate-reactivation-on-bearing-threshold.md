---
id: '003'
title: 'DriveController: turn-in-place gate (PRE_ROTATE reactivation on bearing threshold)'
status: open
use-cases:
- SUC-002
depends-on:
- "011-002"
github-issue: ''
issue: kinematics-pose-control-goto.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 011-003: DriveController — turn-in-place gate (PRE_ROTATE on bearing threshold)

## Description

Restore and rewrite the PRE_ROTATE phase in `beginGoTo()` using the new
architecture: the gate decision uses the robot-frame bearing to the goal, and
the transition to `PURSUE` happens when the bearing drops below the threshold
rather than when a pre-committed encoder distance is reached.

After ticket 002, `beginGoTo()` always enters `PURSUE` directly. This ticket
adds the gate check that routes beside/behind targets through a spin-in-place
phase first.

### Gate decision in `beginGoTo()`

After computing the world-frame target storage (from ticket 002), compute the
robot-frame goal bearing and compare to the threshold:

```cpp
float x, y, h_rad;
getPoseFloat(x, y, h_rad);
_gTargetXWorld = x + tx * cosf(h_rad) - ty * sinf(h_rad);
_gTargetYWorld = y + tx * sinf(h_rad) + ty * cosf(h_rad);
_gSpeed = speedMms;
_mode   = DriveMode::GO_TO;

// Robot-frame bearing to goal
float dx = tx * cosf(0.0f) - ty * sinf(0.0f);  // at t=0, robot frame IS the input
float dy = tx * sinf(0.0f) + ty * cosf(0.0f);  // simplifies: dx=tx, dy=ty at begin
float bearing = fabsf(atan2f(ty, tx));          // bearing in robot frame at command time
float gateRad = _cfg.turnInPlaceGate * (3.14159265f / 180.0f);  // degrees → rad

if (bearing > gateRad) {
    float turnSign  = (ty >= 0.0f) ? 1.0f : -1.0f;
    float rawL = -turnSign * _gSpeed;
    float rawR =  turnSign * _gSpeed;
    float sL, sR;
    BodyKinematics::saturate(rawL, rawR, _cfg.vWheelMax, _cfg.steerHeadroom, sL, sR);
    _mc.startDriveClean(sL, sR);
    _mc.setTarget(sL, sR);
    _gPhase = GPhase::PRE_ROTATE;
} else {
    _mc.startDriveClean(_gSpeed, _gSpeed);   // straight initial setpoint; PURSUE corrects next tick
    _gPhase = GPhase::PURSUE;
}
```

Note: the bearing is computed from the **robot-relative input** `(tx, ty)` at
command time. This is correct: the command is always issued in robot frame.
The world-frame storage is only used for the continuous re-steering in PURSUE.

### PRE_ROTATE tick logic

In the PURSUE-based design, PRE_ROTATE no longer needs pre-committed encoder
distances. Instead, each PRE_ROTATE tick re-reads the robot-frame bearing to
the world-frame goal and exits when the bearing is below the gate:

```cpp
if (_gPhase == GPhase::PRE_ROTATE) {
    float x, y, h_rad;
    getPoseFloat(x, y, h_rad);
    float dxW = _gTargetXWorld - x;
    float dyW = _gTargetYWorld - y;
    // Robot-frame bearing
    float dx_rf =  dxW * cosf(h_rad) + dyW * sinf(h_rad);
    float dy_rf = -dxW * sinf(h_rad) + dyW * cosf(h_rad);
    float bearing = fabsf(atan2f(dy_rf, dx_rf));
    float gateRad = _cfg.turnInPlaceGate * (3.14159265f / 180.0f);

    if (bearing <= gateRad) {
        // Bearing is now within threshold — transition to pursue
        _gPhase = GPhase::PURSUE;
        // PURSUE tick will set correct wheel speeds on next iteration
    }
    // else: keep spinning (wheel setpoints set at beginGoTo() remain)
}
```

### State cleanup

The PRE_ROTATE encoder fields (`_gArcLeftMm`, `_gArcRightMm`, `_gArcStartL`,
`_gArcStartR`) were already removed in ticket 002 — no additional field changes
needed here.

## Acceptance Criteria

- [ ] Unit test: bearing computation — target at `(tx=-300, ty=0)` (directly
  behind) gives bearing = π (> default 45° gate); gate fires. [unit]
- [ ] Unit test: target at `(tx=300, ty=10)` gives bearing ≈ 1.9° (< 45° gate);
  gate does not fire, enters PURSUE directly. [unit]
- [ ] Unit test: target at `(tx=0, ty=300)` gives bearing = 90° (> 45° gate);
  gate fires. [unit]
- [ ] Unit test: bearing threshold configurable — `SET turnGate=30` makes target
  at `(tx=200, ty=150)` (≈37°) trigger the gate. [unit]
- [ ] **Bench**: `G -200 0 150` (directly behind) causes a visible in-place
  rotation before forward pursuit; robot arrives within `arriveTolMm` of
  target. [bench — HARDWARE REQUIRED]
- [ ] **Bench**: `G 0 300 150` (90° left) causes in-place rotation then forward
  pursuit. [bench — HARDWARE REQUIRED]
- [ ] **Bench**: `G 300 50 200` (slight left, within gate) does not pre-rotate;
  goes directly to pursuit arc. [bench — HARDWARE REQUIRED]
- [ ] All existing tests pass.

## Implementation Plan

### Approach

Modify `beginGoTo()` to add the gate check and set `GPhase::PRE_ROTATE` when
bearing exceeds threshold. Replace the old PRE_ROTATE tick logic (encoder-distance
check) with the continuous-bearing check against the world-frame goal.

### Files to modify

- `source/control/DriveController.cpp` — `beginGoTo()` gate decision, PRE_ROTATE
  tick logic rewrite

### Files to create

- Unit test cases added to the test file created in ticket 002 (bearing
  computation tests; gate threshold tests)

### Testing plan

Unit tests cover the bearing arithmetic and the threshold comparison (both sides
of the boundary). Bench tests validate the full PRE_ROTATE → PURSUE sequence
on hardware, including the three start configurations from the issue Verification
section (front, behind, beside).

### Documentation updates

None at this stage.
