---
id: '004'
title: sim_api C ABI extensions and sim_conn Python bindings
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- '003'
github-issue: ''
issue: ''
completes_issue: false
---

# sim_api C ABI extensions and sim_conn Python bindings

## Description

The C++ mock hardware noise model (tickets 001-003) is invisible to Python until
the C ABI and Python bindings are extended. This ticket adds eleven new `extern "C"`
functions to `sim_api.cpp` and six new methods to `sim_conn.py`, making the noise
controls and oracle pose accessible from Python.

`SimHandle`'s constructor is also extended to call `hal.setTrackwidth(cfg.trackwidthMm)`
so `ExactPoseTracker::update()` uses the correct robot geometry from the start.

## Acceptance Criteria

### sim_api.cpp

- [x] `SimHandle` constructor body calls `hal.setTrackwidth(cfg.trackwidthMm)` after
  member initialisation is complete.
- [x] New `extern "C"` functions added (in the `extern "C"` block):
  - `float sim_get_exact_pose_x(void* h)` — returns `hal.exactPoseMock().x`
  - `float sim_get_exact_pose_y(void* h)` — returns `hal.exactPoseMock().y`
  - `float sim_get_exact_pose_h(void* h)` — returns `hal.exactPoseMock().h`
  - `void sim_set_motor_slip(void* h, int side, float straight, float turn_extra)`
    — side 0=left, 1=right, 2=both; calls `motor.setSlip(straight, turn_extra)`
  - `void sim_set_encoder_noise(void* h, int side, float sigma_mm)`
    — side 0=left, 1=right, 2=both; calls `motor.setEncoderNoise(sigma_mm)`
  - `void sim_enable_otos_model(void* h)` — calls `hal.otosMock().enableSimModel(true)`
  - `void sim_set_otos_linear_noise(void* h, float sigma_fraction)` — calls
    `hal.otosMock().setLinearNoise(sigma_fraction)`
  - `void sim_set_otos_yaw_noise(void* h, float sigma_fraction)` — calls
    `hal.otosMock().setYawNoise(sigma_fraction)`
  - `float sim_get_otos_x(void* h)` — returns OTOS accumulated x (mm)
  - `float sim_get_otos_y(void* h)` — returns OTOS accumulated y (mm)
  - `float sim_get_otos_h(void* h)` — returns OTOS accumulated h (rad)
- [x] The OTOS pose getters read the accumulated `_odomX/Y/H` from MockOtosSensor.
  If `MockOtosSensor` does not expose these directly via getters, add `odomX()`,
  `odomY()`, `odomH()` accessors in ticket 003's header and use them here.

### sim_conn.py

- [x] `_setup_types()` registers all 11 new C functions with correct argtypes/restype.
- [x] New methods on `SimConnection`:
  - `get_exact_pose() -> dict[str, float]` returns `{"x": ..., "y": ..., "h": ...}`
  - `set_slip(straight: float = 0.005, turn_extra: float = 0.03) -> None`
    (both wheels; calls `sim_set_motor_slip(h, 2, straight, turn_extra)`)
  - `set_encoder_noise(sigma_mm: float = 0.05) -> None`
    (both wheels; calls `sim_set_encoder_noise(h, 2, sigma_mm)`)
  - `enable_otos_model() -> None`
  - `set_otos_noise(linear: float = 0.01, yaw: float = 0.025) -> None`
  - `get_otos_pose() -> dict[str, float]` returns `{"x": ..., "y": ..., "h": ...}`
- [x] `_snapshot()` extended with keys `exact_pose_x`, `exact_pose_y`, `exact_pose_h`,
  `otos_x`, `otos_y`, `otos_h` (all floats).
- [x] `libfirmware_host` builds cleanly after `sim_api.cpp` changes.
- [x] `uv run --with pytest python -m pytest` passes with no regressions.
- [x] Smoke test from Python REPL:
  ```python
  conn = SimConnection(); conn.connect()
  conn.set_slip(0.005, 0.03); conn.set_encoder_noise(0.05)
  conn.enable_otos_model(); conn.set_otos_noise(0.01, 0.025)
  conn.send("VW 400 0"); conn.tick(1000)
  assert conn.get_exact_pose()["x"] > conn.get_state()["pose_x"]  # exact > noisy
  assert conn.get_otos_pose()["x"] > 0
  ```

## Implementation Plan

### Approach

Append the new functions to `sim_api.cpp` within the existing `extern "C"` block.
The `side` parameter pattern for motor functions mirrors the existing
`sim_set_motor_offset` function. Keep the same `if (side == 0 || side > 1)` /
`if (side == 1 || side > 1)` guard pattern.

For `sim_get_otos_x/y/h`: if `MockOtosSensor` exposes `odomX()/odomY()/odomH()`
accessors (added in ticket 003), use those. If not, add them in a follow-on edit to
`MockOtosSensor.h` within this ticket.

### `SimHandle` constructor body

```cpp
SimHandle()
    : hal()
    , cfg(defaultRobotConfig())
    , robot(hal, cfg)
    , cmd(robot.buildCommandTable(nullptr, nullptr))
{
    hal.setTrackwidth(cfg.trackwidthMm);  // <- add this line
}
```

### Files to modify

- `host_tests/sim_api.cpp` — add 11 `extern "C"` functions; extend `SimHandle` ctor
- `host/robot_radio/io/sim_conn.py` — add 6 methods; extend `_setup_types()` and
  `_snapshot()`
- `source/hal/mock/MockOtosSensor.h` — add `odomX()`, `odomY()`, `odomH()` accessors
  if not added by ticket 003

### Testing plan

- Existing pytest suite must pass unchanged.
- Run the smoke test above from a Python REPL after building.
- Verify `state_df()` has the six new columns.

### Documentation updates

No additional docs; architecture update already describes the new C functions and
Python methods.
