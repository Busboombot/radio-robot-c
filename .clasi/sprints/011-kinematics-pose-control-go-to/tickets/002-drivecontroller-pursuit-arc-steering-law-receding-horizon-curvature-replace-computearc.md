---
id: '002'
title: 'DriveController: pursuit-arc steering law (receding-horizon curvature, replace
  computeArc)'
status: open
use-cases:
- SUC-001
- SUC-002
depends-on:
- "011-001"
github-issue: ''
issue: kinematics-pose-control-goto.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 011-002: DriveController — pursuit-arc steering law (receding-horizon curvature, replace computeArc)

## Description

Replace the one-shot `computeArc()` / `GPhase::ARC` mechanism with a
**receding-horizon pursuit-arc controller** that recomputes curvature every tick
from the current fused pose. This is the core steering law from
kinematics-model.md §1.5 and §2.5 step 3.

This ticket implements the steering geometry only: it does not yet include the
turn-in-place gate (ticket 003), accel/decel shaping (ticket 004), or arrival
detection (ticket 004). After this ticket, `beginGoTo()` always transitions
directly to `PURSUE` (no PRE_ROTATE) and drives at fixed speed `_gSpeed` until
STOP is called — a useful intermediate state for bench-testing the steering law.

### Changes to `DriveController.h`

**Rename `GPhase::ARC` → `GPhase::PURSUE`** (PRE_ROTATE and IDLE stay).

**Remove**: `_gArcLeftMm`, `_gArcRightMm`, `_gArcStartL`, `_gArcStartR`.

**Add**:
```cpp
float _gTargetXWorld;  // goal x in world frame (mm), set at beginGoTo()
float _gTargetYWorld;  // goal y in world frame (mm), set at beginGoTo()
```

**Remove** `computeArc()` private static declaration.

**Add** private helper declaration: `void getPoseFloat(float& x, float& y, float& h_rad) const;`

### Changes to `DriveController.cpp`

**`beginGoTo()`** — store goal in world frame by transforming the robot-relative
`(tx, ty)` input using the current odometry pose. PRE_ROTATE gate omitted in
this ticket; added in ticket 003:
```cpp
float x, y, h_rad;
getPoseFloat(x, y, h_rad);
_gTargetXWorld = x + tx * cosf(h_rad) - ty * sinf(h_rad);
_gTargetYWorld = y + tx * sinf(h_rad) + ty * cosf(h_rad);
_gSpeed  = speedMms;
_gPhase  = GPhase::PURSUE;
_mode    = DriveMode::GO_TO;
// capture sink + corr_id (existing pattern unchanged)
```

**New private helper `getPoseFloat()`** — reads Odometry integers, converts to
floats:
```cpp
void DriveController::getPoseFloat(float& x, float& y, float& h_rad) const {
    int32_t xi, yi, hi;
    _odo.getPose(xi, yi, hi);
    x     = static_cast<float>(xi);
    y     = static_cast<float>(yi);
    h_rad = static_cast<float>(hi) * (3.14159265f / 18000.0f);  // cdeg → rad
}
```

**`tick()` — PURSUE branch** (replaces the old ARC branch):
```cpp
} else if (_gPhase == GPhase::PURSUE) {
    float x, y, h_rad;
    getPoseFloat(x, y, h_rad);

    // World-frame offset → robot frame
    float dxW = _gTargetXWorld - x;
    float dyW = _gTargetYWorld - y;
    float dx  =  dxW * cosf(h_rad) + dyW * sinf(h_rad);  // forward in robot frame
    float dy  = -dxW * sinf(h_rad) + dyW * cosf(h_rad);  // left in robot frame

    float d2    = dx * dx + dy * dy;
    float kappa = (d2 > 0.1f) ? (2.0f * dy / d2) : 0.0f;  // κ = 2dy/(dx²+dy²)

    float v     = _gSpeed;    // ticket 004 replaces with trapezoidal v
    float omega = v * kappa;

    float vL, vR;
    BodyKinematics::inverse(v, omega, _cfg.trackwidthMm, vL, vR);
    float sL, sR;
    BodyKinematics::saturate(vL, vR, _cfg.vWheelMax, _cfg.steerHeadroom, sL, sR);
    _mc.setTarget(sL, sR);
}
```

**Remove `computeArc()` definition** from DriveController.cpp.

## Acceptance Criteria

- [ ] `computeArc()` deleted from header and implementation; no other file
  references it. [compilation]
- [ ] `GPhase::ARC` renamed to `GPhase::PURSUE` throughout. [compilation]
- [ ] Unit test: goal `(dx=300, dy=0)` straight ahead → `κ = 0`, `ω = 0`,
  `vL = vR = _gSpeed`. [unit]
- [ ] Unit test: goal `(dx=100, dy=100)` (45° left) → `κ = 2·100/(100²+100²) = 0.01`,
  `ω = _gSpeed · 0.01`. [unit]
- [ ] Unit test: goal `(dx=0, dy=100)` (90° left) → `κ = 2·100/10000 = 0.02`. [unit]
- [ ] Unit test: goal `(dx=0, dy=0)` → `d2 ≤ 0.1` guard fires, `κ = 0`, no
  divide-by-zero. [unit]
- [ ] Bench (informational — no arrival yet): `G 300 0 200` drives robot
  approximately straight forward; `G 300 100 200` causes leftward curve. [bench]
- [ ] All existing tests pass.

## Implementation Plan

### Approach

Structural replacement in `DriveController` only. No new files except the unit
test file.

### Files to modify

- `source/control/DriveController.h` — field changes, remove `computeArc()` decl,
  add `getPoseFloat()` decl, rename `GPhase::ARC` → `GPhase::PURSUE`
- `source/control/DriveController.cpp` — replace `beginGoTo()` and tick
  ARC→PURSUE branch, delete `computeArc()`, add `getPoseFloat()`

### Files to create

- Unit test file covering the curvature formula and the d2 zero-guard
  (follow existing test file conventions in the project)

### Testing plan

Unit tests for `κ = 2·dy/(dx²+dy²)` are pure arithmetic — cover straight-ahead,
45° offset, 90° offset, and zero-distance guard. Bench tests are informational at
this stage (arrival detection not yet present).

### Documentation updates

None at this stage (protocol-v2.md updated in ticket 005).
