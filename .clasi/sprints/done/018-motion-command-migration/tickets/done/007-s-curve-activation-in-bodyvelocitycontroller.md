---
id: '007'
title: S-curve activation in BodyVelocityController
status: done
use-cases:
- SUC-007
depends-on:
- '006'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# S-curve activation in BodyVelocityController

## Description

Activate the jerk-limited (S-curve) path in `BodyVelocityController::advance()` when
`jMax > 0` / `yawJerkMax > 0`. At default values (both 0), the code path degenerates to
the existing trapezoid — no behaviour change for existing users.

This is the last ticket in sprint 018 and completes the motion-command issue.

**Design (from architecture-update.md §S-Curve Activation):**
- Add `float _aLive` (current live acceleration, linear) and `float _omegaALive` (yaw)
  to `BodyVelocityController` as private members, zero-initialised.
- In `reset()`, zero `_aLive` and `_omegaALive`.
- In `advance(dt_s)`, replace the direct trapezoid step with:

  **Linear channel with jerk limit:**
  ```
  if (cfg.jMax > 0) {
      // Target acceleration = sign(vTgt_clamped - v) * aMax (or aDecel for decel)
      float aTarget = (vTgt_clamped >= v) ? cfg.aMax : -cfg.aDecel;
      float jerkStep = cfg.jMax * dt_s;
      _aLive = approach(_aLive, aTarget, jerkStep);  // slew acceleration
      v = clamp(v + _aLive * dt_s, -cfg.vBodyMax, cfg.vBodyMax);  // integrate
  } else {
      v = approach(v, vTgt_clamped, dv_max);  // existing trapezoid
  }
  ```

  **Yaw channel with jerk limit:**
  ```
  if (cfg.yawJerkMax > 0) {
      float yawJerkMaxRad = cfg.yawJerkMax * (PI / 180.0f);  // deg/s³ → rad/s³
      float omegaATarget = (omegaTgt_clamped >= omega) ? yawAccelMaxRad : -yawAccelMaxRad;
      _omegaALive = approach(_omegaALive, omegaATarget, yawJerkMaxRad * dt_s);
      omega = clamp(omega + _omegaALive * dt_s, -yawRateMaxRad, yawRateMaxRad);
  } else {
      omega = approach(omega, omegaTgt_clamped, yawAccelMaxRad * dt_s);  // existing trapezoid
  }
  ```

- After the profile step, `inverse → saturate → setTarget` remain unchanged.
- `atTarget()` is unchanged (converges on clamped target regardless of profile).

**Degeneration check:** At `jMax = 0`: the else-branch runs the original trapezoid.
`_aLive` is unused (can be left at zero). This must produce identical output to the
pre-018 BVC for any test that does not set `jMax`.

**No config/registry change needed** — `jMax` and `yawJerkMax` are already in
`Config.h` and `kRegistry[]` (added in Sprint 017).

## Acceptance Criteria

- [x] `_aLive`, `_omegaALive` added to `BodyVelocityController.h` private section.
- [x] `reset()` zeroes both new members.
- [x] At `jMax = 0`: `advance()` output is byte-identical to pre-018 trapezoid.
- [x] At `jMax > 0`: ramp reaches target speed later than trapezoid at the same `aMax`
  (host unit test: simulate N ticks with same dt, compare time-to-target).
- [x] All existing `test_body_velocity_controller.py` tests pass unchanged (trapezoid
  behaviour preserved at default config).
- [x] `SET jMax=500` / `GET jMax` round-trips (pre-existing from Sprint 017 registry).
- [x] `uv run --with pytest python -m pytest -q` passes at 1334/8 (sprint baseline; original
  ticket said 1179/8 — the sprint added tests on earlier tickets; all 8 pre-existing failures unchanged).
- [x] Clean build: `python3 build.py --clean` succeeds.
- [ ] Bench: `SET jMax=500` then `VW 300 0` — observe smoother onset on velocity chart
  (stakeholder-deferred; on-robot S-curve tuning is not required for sprint close).

## Implementation Plan

### Files to modify
- `source/control/BodyVelocityController.h` — add `float _aLive = 0.0f`, `float _omegaALive = 0.0f` to private section
- `source/control/BodyVelocityController.cpp`:
  - `reset()`: add `_aLive = 0.0f; _omegaALive = 0.0f;`
  - `advance(dt_s)`: replace trapezoid step with the jerk-branched version above;
    keep the clamp-toward-clamped-target logic identical in the else-branch

### No changes to
- `MotionCommand`, `StopCondition`, `DriveController`, `CommandProcessor`
- `Config.h` (fields already present)
- `kRegistry[]` (entries already present)

### Testing plan
- Run existing `tests/dev/test_body_velocity_controller.py` — all must pass (jMax = 0 default).
- Add new test: configure BVC with `jMax = 1000 mm/s³` and simulate 50 ticks at 10 ms;
  verify that speed at tick 20 is less than trapezoid speed at tick 20 with same `aMax`.
- Full pytest suite: `uv run --with pytest python -m pytest -q`.
- Bench (stakeholder-deferred): `SET jMax=500`, then VW 300 0 — observe smoother onset
  on velocity chart vs trapezoid.

### Issue completion
This ticket sets `completes_issue: true` for the motion-command-body-velocity-control
issue. The close-sprint step will mark the issue done and archive it.
