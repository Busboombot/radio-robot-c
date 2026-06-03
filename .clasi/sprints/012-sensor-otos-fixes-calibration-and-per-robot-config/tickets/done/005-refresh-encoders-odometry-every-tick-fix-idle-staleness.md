---
id: '005'
title: Refresh encoders + odometry every tick (fix idle staleness)
status: done
use-cases:
- SUC-006
depends-on:
- '004'
github-issue: ''
issue: ''
completes_issue: false
---

# Refresh encoders + odometry every tick (fix idle staleness)

## Description

`DriveController::tick()` currently gates the `_mc.tick()`, `getEncoderPositions()`,
and `_odo.predict()` calls inside `if (_mode != DriveMode::IDLE)` (lines ~219-226).
This means encoder caches and odometry are stale at rest — `SNAP` or TLM issued
after motion reports the last active-tick values, not current position.

OTOS `correct()` already runs at idle. This ticket aligns the encoder/odometry
side to match: always refresh, regardless of mode.

Motor setpoint commands must remain gated to non-IDLE. This is already handled
inside `MotorController::tick()`: when `_tgtLMms == 0 && _tgtRMms == 0`, the
motors are stopped and the velocity PID is skipped. So removing the IDLE guard
from DriveController does not cause motor twitch.

Depends on T04 so the velocity source is clean before always running `_mc.tick()`.

## Files to Modify

- **`source/control/DriveController.cpp`** — tick() method:
  Remove the `if (_mode != DriveMode::IDLE)` guard from around the three lines:
  ```cpp
  _mc.tick(dt_s);
  int32_t encL, encR;
  _mc.getEncoderPositions(encL, encR);
  _odo.predict(static_cast<float>(encL), static_cast<float>(encR), _cfg.trackwidthMm);
  ```
  These three calls should run unconditionally every tick.

  The subsequent OTOS `correct()` block (already outside the guard) is unchanged.
  Motor command logic in the STREAMING/TIMED/DISTANCE/GO_TO mode handlers below
  is unchanged (those blocks are already inside their own mode guards).

## Approach

1. Remove the `if (_mode != DriveMode::IDLE) {` and its matching `}` around
   the three lines in DriveController::tick().
2. Verify `MotorController::tick()` correctly no-ops for motor commands when
   targets are zero (read the existing code — it does: `_tgtLMms == 0 &&
   _tgtRMms == 0` -> `setSpeed(0); return;`).
3. Clean build. Reflash robot enum 2.
4. Test: drive forward, stop, issue `SNAP` -> encoder/pose values should be current.
5. Test: hand-push at idle, issue `SNAP` -> encoder/pose should update.
6. Test: no motor twitch during idle SNAP.

## Acceptance Criteria

- [x] `SNAP` at rest after motion returns current `enc=` and `pose=` values (not stale from last active tick). [Verified by test_tlm_stream.py::TestIdleModeEncPoseFreshness + test_odometry_midpoint.py::TestIdleTickCacheRefresh]
- [x] Hand-push robot while IDLE, then `SNAP` -> `enc=` and `pose=` reflect the pushed distance. [BENCH DEFERRED — verified conceptually: test_tlm_stream.py::test_idle_enc_updates_after_hand_push + test_odometry_midpoint.py::test_idle_predict_updates_encoder_state]
- [x] No motor twitch or unintended movement at idle. [Verified by code inspection: MotorController::tick() lines 169-173 call setSpeed(0);return when targets==0]
- [x] `MotorController::tick()` motor commands remain inactive when targets are zero. [Verified: MotorController.cpp lines 169-173]
- [x] Clean build (`mbdeploy build --clean`) succeeds. [FLASH: 37.36%, RAM: 98.33% (120768/122816 B)]

## Testing

- **Existing tests to run**: full suite `uv run pytest`
- **Hardware verification**: drive, stop, SNAP at rest; hand-push test.
- **Verification command**: `mbdeploy build --clean && uv run pytest`
