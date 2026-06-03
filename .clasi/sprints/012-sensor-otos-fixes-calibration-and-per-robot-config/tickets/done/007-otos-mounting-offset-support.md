---
id: '007'
title: OTOS mounting offset support
status: done
use-cases:
- SUC-004
- SUC-005
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---

# OTOS mounting offset support

## Description

The prior system (`src/otos.ts`) applies a mounting offset+yaw+upside-down
transform (`poseRobotFrame`) that converts OTOS chip-frame readings to the
robot-center frame. This firmware lacks that transform.

For nezha bots, the OTOS chip mounting offsets are expected to be ~0 (confirm
physically). At default values (offsets=0, yaw=0, upsideDown=false), the
transform is a mathematical no-op. This ticket adds the transform so the
architecture is complete and ready for non-zero offsets when needed.

The config fields (`odomOffX`, `odomOffY`, `odomYawDeg`, `odomUpsideDown`)
are added in T01 and registered in kRegistry (SET/GET). This ticket applies
them in the correction path.

## Files to Modify

- **`source/control/DriveController.cpp`** — in the OTOS `correct()` call block:
  After reading `rx, ry, rh` from `_otos->getPositionRaw()` and converting to
  mm, apply the transform before calling `_odo.correct()`:

  ```cpp
  // OTOS chip frame -> robot center frame transform
  float x_chip = static_cast<float>(rx) * kPosMmPerLsb;
  float y_chip = static_cast<float>(ry) * kPosMmPerLsb;

  // Flip X if chip is mounted upside-down
  if (_cfg.odomUpsideDown) x_chip = -x_chip;

  // Apply yaw rotation and translation offset
  float yawRad = _cfg.odomYawDeg * (3.14159265f / 180.0f);
  float c = cosf(yawRad), s = sinf(yawRad);
  float x_mm = c * x_chip - s * y_chip + _cfg.odomOffX;
  float y_mm = s * x_chip + c * y_chip + _cfg.odomOffY;

  float h_rad = static_cast<float>(rh) * kHdgRadPerLsb;
  _odo.correct(x_mm, y_mm, h_rad, _cfg.alphaPos, _cfg.alphaYaw, _cfg.otosGate);
  ```

  At default values (offsets=0, yaw=0, upsideDown=false), this is a no-op
  relative to the current code.

- **`source/control/DriveController.h`** — no interface change needed.

## Approach

1. Add the transform in DriveController.cpp as described.
2. Verify that at default values (all zeros/false), `_odo.correct()` receives
   the same values as before (mathematically check: cos(0)=1, sin(0)=0,
   offsets=0 -> x_mm = x_chip, y_mm = y_chip).
3. Confirm physically that the nezha robot OTOS chip offsets are ~0.
   If nonzero, measure and note for T08 (robot JSON values).
4. Write a unit test for the transform function (pass non-zero offset/yaw,
   verify the output math).
5. Clean build. Reflash robot enum 2.
6. Verify no change to pose behavior at default offsets (regression test).

## Acceptance Criteria

- [x] At default config (all offsets=0, yaw=0, upsideDown=false), pose behavior is unchanged (regression-free).
- [x] With a nonzero offset set via `SET odomOffX=10`, fused pose reflects the robot-center frame offset.
- [x] Unit test for the transform math passes.
- [x] Clean build (`mbdeploy build --clean`) succeeds.
- [ ] OTOS chip offsets for the nezha robot are measured and documented (may be ~0; record either way). [DEFERRED — bench AC, requires hardware]

## Testing

- **New tests to write**: unit test for the offset+yaw transform with known inputs.
- **Regression check**: run `uv run pytest tests/test_otos_fusion.py` — must pass at default offsets.
- **Verification command**: `mbdeploy build --clean && uv run pytest`
