---
id: '004'
title: Robot wiring (otosCorrect velocity read + twist TLM field)
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-006
depends-on:
- '003'
issue: ekf-velocity-fusion-and-robot-state.md
---

# T004: Robot wiring (otosCorrect velocity read + twist TLM field)

## Description

Wire the new OTOS velocity/accel read path into `Robot::otosCorrect()`, pass
velocity measurements to `Odometry::correctEKF()`, update `buildTlmFrame()` to
emit `twist=v,omega`, update the `STREAM fields=` parser to recognise `twist`,
and update the `Robot` constructor's `initEKF()` call to pass the four new noise
params.

This is pure wiring in `Robot.cpp` — no new algorithms.

## Acceptance Criteria

**`Robot::Robot()` constructor (SUC-001):**
- [x] `odometry.initEKF(...)` call updated to pass all seven params:
  ```cpp
  odometry.initEKF(config.ekfQxy, config.ekfQtheta,
                   config.ekfQv, config.ekfQomega,
                   config.ekfROtosXy, config.ekfROtosV, config.ekfREncV);
  ```

**`Robot::otosCorrect()` (SUC-002, SUC-003):**
- [x] After reading position via `otos.readTransformed(config)`, also call
  `OtosVelocity vel = otos.readVelocityTransformed(config)` and
  `OtosAccel acc = otos.readAccelTransformed(config)`.
- [x] `state.inputs.otosAccelX = acc.ax_mmps2` and
  `state.inputs.otosAccelY = acc.ay_mmps2` stored.
- [x] `odometry.correctEKF(state.inputs, p.x, p.y, vel.v_mmps, vel.omega_rads, enc_v, enc_omega)` called.
  `enc_v` and `enc_omega` are the encoder-rate values computed by the most recent
  `predict()` call. The cleanest approach: store the last predict's `enc_v`/`enc_omega`
  as private members on `Robot` (or `Odometry`) so `otosCorrect()` can retrieve
  them. Alternative: pass them as parameters from the cooperative loop that calls
  both `predict()` and `otosCorrect()`. Choose the approach that requires the
  fewest new cross-method coupling points. Document the choice.
- [x] `otos.readVelocityTransformed()` and `readAccelTransformed()` calls are
  guarded by `otos.is_initialized()` (same guard as existing position read).

**`Robot::buildTlmFrame()` (SUC-006):**
- [x] New `TLM_FIELD_TWIST` check:
  ```cpp
  bool haveTwist = (config.tlmFields & TLM_FIELD_TWIST) != 0;
  ```
- [x] When `haveTwist`, emit:
  ```cpp
  n = snprintf(buf + pos, (size_t)rem, " twist=%d,%d",
               (int)state.inputs.fusedV,
               (int)(state.inputs.fusedOmega * 1000.0f));  // mrad/s
  ```
  `fusedV` is integer mm/s; `fusedOmega` is converted to integer mrad/s (matching
  the `omega_mrads` convention used by the existing `VW` command and `NezhaProtocol.vw()`).
- [x] The `twist=` field is emitted after `vel=` (if present) and before `line=`,
  following the existing field ordering.

**`STREAM fields=` parser (SUC-006):**
- [x] The `STREAM fields=...` token parser in `Robot.cpp` recognises `"twist"` and
  sets `TLM_FIELD_TWIST` in the mask.
- [x] The `STREAM fields=...` dump loop (that prints active fields) includes
  `{ TLM_FIELD_TWIST, "twist" }` in its table.

**Build and test:**
- [x] `python3 build.py` passes cleanly.
- [x] `uv run --with pytest python -m pytest -v` passes.
- [x] The cooperative loop caller of `predict()` (the control fiber in Robot.cpp)
  is updated to pass `now_ms` as the new argument.

## Implementation Plan

### Approach

The single file is `Robot.cpp`. Four change sites:

1. **Constructor:** Extend the `initEKF()` call — straightforward.

2. **`otosCorrect()`:** Add two new HAL read calls after the existing
   `otos.readTransformed(config)` call. Store accel. Pass velocity to
   `correctEKF()`. For `enc_v`/`enc_omega`: the simplest approach is to store
   the encoder-rate values computed in `predict()` as private members
   `_lastEncV` and `_lastEncOmega` on `Odometry` (set at the end of each
   `predict()` call; zero-initialised). `correctEKF()` then reads them via
   getter methods. This avoids changing the caller's call site.

3. **`buildTlmFrame()`:** Insert the `twist=` snprintf block following the
   existing `vel=` pattern.

4. **`STREAM fields=` parser:** Add `"twist"` to the two existing tables
   (the parser `if strcmp` chain and the dump table).

**`predict()` call site update:** Find where `odometry.predict(s, config.trackwidthMm)`
is called in the control fiber (search for `odometry.predict(` in Robot.cpp).
Add `now_ms` (= `systemTime()` at that point) as the third argument.

### Files to modify

- `source/robot/Robot.cpp` — four change sites as described

### Testing plan

```
python3 build.py
uv run --with pytest python -m pytest -v
```

No new unit tests are added in this ticket — the behavior is verified
end-to-end by T006's replay harness and, optionally, bench telemetry.

Verify `twist=` appears in a SNAP response by examining the output:
```
uv run rogo snap
```
(This requires the robot to be connected and running the new firmware.)

### Documentation updates

Update the comment at the top of `otosCorrect()` in Robot.cpp to reflect the
extended behavior (reads velocity and acceleration; passes to EKF).
