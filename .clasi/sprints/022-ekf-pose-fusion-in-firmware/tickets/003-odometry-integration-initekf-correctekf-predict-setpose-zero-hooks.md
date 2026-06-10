---
id: '003'
title: Odometry integration (initEKF, correctEKF, predict/setPose/zero hooks)
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- '001'
- '002'
completes_issue: false
---

# T003: Odometry integration (initEKF, correctEKF, predict/setPose/zero hooks)

## Description

Integrate the EKF (T001) into the `Odometry` class so that:
- Every `predict()` tick advances the EKF state alongside the midpoint integration
  and writes EKF state back as the authoritative pose.
- A new `correctEKF()` method applies an OTOS position observation through the
  EKF and writes the corrected state back to `HardwareState`.
- `setPose()` and `zero()` keep the EKF in sync with any external pose resets.
- `initEKF()` wires EKF noise parameters at startup.

The existing `correct()` method is NOT modified (it is exercised by existing
tests and kept for backward compatibility).

## Acceptance Criteria

- [x] `source/control/Odometry.h` includes `"EKF.h"` and declares:
  - `EKF _ekf` as a private value member
  - `void initEKF(float q_xy, float q_theta, float r_otos_xy)`
  - `void correctEKF(HardwareState& s, float x_otos, float y_otos)`
- [x] `source/control/Odometry.cpp` implements `initEKF()` as a direct call to
  `_ekf.init(q_xy, q_theta, r_otos_xy)`.
- [x] `source/control/Odometry.cpp` implements `correctEKF()`:
  ```cpp
  void Odometry::correctEKF(HardwareState& s, float x_otos, float y_otos) {
      _ekf.update(x_otos, y_otos);
      s.poseX    = _ekf.x();
      s.poseY    = _ekf.y();
      s.poseHrad = _ekf.theta();
  }
  ```
- [x] `Odometry::predict()` is extended — AFTER the existing midpoint integration
  (after `s.poseHrad = wrapPi(s.poseHrad + dTheta)`) — with:
  ```cpp
  float theta_before = s.poseHrad - dTheta;   // must be captured BEFORE wrapPi write
  _ekf.predict(dCenter, dTheta, theta_before);
  s.poseX    = _ekf.x();
  s.poseY    = _ekf.y();
  s.poseHrad = _ekf.theta();
  ```
  IMPORTANT: `theta_before` must be the pre-update heading. Read the existing
  code carefully — `s.poseHrad` is the value AFTER `wrapPi(s.poseHrad + dTheta)`.
  The correct approach is to capture `theta_before` BEFORE the integration runs,
  not subtract dTheta after. Capture it as the first line of `predict()`:
  ```cpp
  float theta_before = s.poseHrad;   // heading before this step
  ```
  Then pass it to `_ekf.predict(dCenter, dTheta, theta_before)`.
- [x] `Odometry::setPose()` calls `_ekf.setPose(static_cast<float>(x_mm),
  static_cast<float>(y_mm), static_cast<float>(h_cdeg) * CDEG_TO_RAD)` after
  setting the HardwareState fields.
- [x] `Odometry::zero()` inherits the EKF reset via its call to `setPose(s,0,0,0)`.
- [x] The existing `correct()` method is unchanged.
- [x] Firmware builds cleanly: `python3 build.py`.
- [x] Test suite passes: `uv run --with pytest python -m pytest` (existing
  `test_otos_fusion.py` must still pass).

## Implementation Plan

### Approach

Three files change: `Odometry.h`, `Odometry.cpp`, and (implicit) the build
catches any include errors.

**Odometry.h changes:**
1. Add `#include "EKF.h"` near the top (after existing includes).
2. Add `EKF _ekf;` to the private section (after `_otosRejected`).
3. Add `void initEKF(float q_xy, float q_theta, float r_otos_xy);` and
   `void correctEKF(HardwareState& s, float x_otos, float y_otos);` to the
   Primary API section.

**Odometry.cpp changes:**

1. In `predict()`: capture `theta_before` as the very first local variable
   (before `dL`/`dR` computation is fine, but must be before `s.poseHrad` is
   modified). Then after the existing `wrapPi` write, add the three EKF lines.

2. Add `initEKF()` implementation (one line: `_ekf.init(q_xy, q_theta,
   r_otos_xy);`).

3. Add `correctEKF()` implementation (three lines as shown above).

4. In `setPose()`: after the existing four field assignments, add
   `_ekf.setPose(s.poseX, s.poseY, s.poseHrad);`.

**Note on `zero()`:** `zero()` calls `setPose(s, 0, 0, 0)`, so the EKF reset
propagates automatically without touching `zero()` directly.

### Files to modify

- `source/control/Odometry.h`
- `source/control/Odometry.cpp`

### Testing plan

```
python3 build.py
uv run --with pytest python -m pytest
```

The existing `test_otos_fusion.py` tests `correct()` (unchanged). The new
`test_ekf.py` (T005) tests the EKF math. This ticket's changes are covered
indirectly by the compile check and the full test suite regression run.

### Documentation updates

Update the `Odometry` class comment in `Odometry.h` to mention the EKF upgrade:
add a line under the existing sprint attribution notes referencing sprint 022
and EKF integration.
