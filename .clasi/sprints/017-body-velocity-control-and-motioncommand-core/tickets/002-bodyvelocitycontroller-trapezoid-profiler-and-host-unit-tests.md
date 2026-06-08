---
id: '002'
title: "BodyVelocityController — trapezoid profiler and host unit tests"
status: open
use-cases:
  - SUC-001
  - SUC-002
  - SUC-006
depends-on:
  - '001'
github-issue: ''
issue: motion-command-body-velocity-control.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 017-002: BodyVelocityController — trapezoid profiler and host unit tests

## Description

Create `source/control/BodyVelocityController.{h,cpp}` — the body-level `(v, ω)` motion
profiler that ramps toward a commanded twist under configurable acceleration and rate limits,
then calls `BodyKinematics::inverse` → `saturate` → `MotorController::setTarget` each tick.

This class is owned by `DriveController` (wired in ticket 004). This ticket builds and
tests it in isolation — the class is not yet called from anywhere in the firmware. CMake
auto-discovers the new `.cpp` via the existing `RECURSIVE_FIND_FILE` glob; no build-list
edit is needed.

Refer to `architecture-update.md` field layout and per-tick math for the exact interface
and algorithm.

## Files to Create / Modify

- **Create** `source/control/BodyVelocityController.h`
- **Create** `source/control/BodyVelocityController.cpp`
- **Create** `tests/dev/test_body_velocity_controller.py` — pure-Python host unit tests

## Acceptance Criteria

### Class interface
- [ ] `BodyVelocityController(MotorController& mc, const RobotConfig& cfg)` constructor.
- [ ] `void setTarget(float v_mms, float omega_rads)` — update commanded twist.
- [ ] `bool advance(float dt_s)` — ramp one step; returns true while still ramping;
  writes wheel targets via `mc.setTarget(sL, sR)`.
- [ ] `void reset()` — zero `_v`, `_omega`, `_vTgt`, `_omegaTgt`; does not call
  `MotorController::stop()` (no brake).
- [ ] `void seedCurrent(float v_mms, float omega_rads)` — set `_v`/`_omega` to the
  given values (handoff without a lurch).
- [ ] `float currentV()`, `currentOmega()`, `targetV()`, `targetOmega()`, `bool atTarget()`.

### Per-tick math (ordering invariant)
- [ ] Linear ramp: asymmetric acceleration (`aMax` accelerating, `aDecel` decelerating).
- [ ] Target clamped to `[-vBodyMax, +vBodyMax]` before ramping.
- [ ] Yaw ramp: `yawAccMax` (deg/s²) converted to rad/s² at use site; rate clamped to
  `[-yawRateMax_rad, +yawRateMax_rad]`.
- [ ] Ordering invariant: `profile → inverse → saturate → setTarget` on every advance call.
- [ ] `jMax == 0` (default): pure trapezoid (S-curve path not taken; TODO comment left).

### Host unit tests (`tests/dev/test_body_velocity_controller.py`)
- [ ] **Linear ramp slope**: step `v` 0→300 mm/s, dt=0.01 s; each tick advances by
  `aMax * dt` until target reached.
- [ ] **Decel slope**: step `v` 300→0 mm/s; slope = `aDecel`.
- [ ] **Yaw ramp slope**: step `omega` 0→`yawRateMax`; slope = `yawAccMax_rad * dt`.
- [ ] **Spin-in-place** (`v=0, omega>0`): resulting wheel targets `sL != 0` and `sR != 0`.
- [ ] **Straight** (`omega=0`): resulting `sL == sR` within float tolerance.
- [ ] **vBodyMax clamp**: target `v=600`, `vBodyMax=400`; live `_v` never exceeds 400.
- [ ] **yawRateMax clamp**: target `omega` above limit; live `_omega` never exceeds limit.
- [ ] **atTarget()**: false while ramping, true once within epsilon of target.
- [ ] **reset()**: after driving, `reset()` → `currentV()==0` and `currentOmega()==0`.
- [ ] **seedCurrent()**: seeds `_v`/`_omega`; next `advance` ramps from seeded values,
  not from zero.
- [ ] **Wheel math**: verify `(sL, sR)` from `advance` matches manual Python computation
  of `inverse(v, omega, tw)` then `saturate(vL, vR, vWheelMax, headroom)`.

### Build and regression
- [ ] Clean build: `python3 build.py --clean` completes without errors.
- [ ] Host test baseline: `uv run --with pytest python -m pytest -q` at 1035/8.

## Implementation Plan

1. Write `BodyVelocityController.h` per `architecture-update.md` field layout.
2. Write `BodyVelocityController.cpp`:
   - Constructor: store refs, zero all floats.
   - `setTarget(v, omega)`: update `_vTgt`, `_omegaTgt`.
   - `advance(dt_s)`: implement `approach(cur, tgt, step)` = `cur + clamp(tgt-cur, -step, +step)`;
     apply linear ramp (asymmetric accel/decel); apply yaw ramp; call
     `BodyKinematics::inverse` then `saturate` then `mc.setTarget`; return `!atTarget()`.
   - `reset()`: zero all four floats.
   - `seedCurrent(v, omega)`: set `_v`, `_omega`.
   - `atTarget()`: `fabs(_v - _vTgt) < 0.5f && fabs(_omega - _omegaTgt) < 0.001f`.
3. Write `tests/dev/test_body_velocity_controller.py` as pure-Python math (same pattern as
   `test_velocity_controller.py` and `test_body_kinematics.py`). Implement the approach
   and ramp math in Python; verify step-by-step computed values.
4. Run clean build and verify host test baseline.

## Notes

- `BodyVelocityController` is not yet owned by `DriveController` in this ticket; wiring
  is in ticket 004.
- `jMax > 0` S-curve path: not implemented; leave a `// TODO(017): S-curve when jMax > 0`
  comment in `advance()`.

## Bench Verification (stakeholder-deferred)

Not applicable — class not yet wired to any command verb.
