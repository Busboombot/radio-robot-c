---
id: '003'
title: Host CMake build + sim_api C wrapper
status: done
use-cases:
- SUC-003
depends-on:
- 020-002
github-issue: ''
issue: plan-sprint-020-firmware-host-testing.md
completes_issue: false
---

# Host CMake build + sim_api C wrapper

## Description

Create a CMake-based host build target that compiles the firmware control and app layers
(CommandProcessor, MotorController, MotionController, BodyVelocityController, Odometry,
Robot, and all MockHAL classes) as a shared library (`libfirmware_host`) for native
execution. CODAL-dependent code is excluded via `HOST_BUILD=1`.

Also create `host_tests/sim_api.cpp` — an `extern "C"` wrapper providing a stable C
ABI over an opaque simulation handle. Python test code (ticket 020-004) loads this via
ctypes.

The programmer must audit each included source file for CODAL includes and add
`HOST_BUILD` guards where needed before the CMake build can succeed.

## Acceptance Criteria

- [x] `host_tests/CMakeLists.txt` created with `project(radio_robot_host CXX)`, C++11 standard, `HOST_BUILD=1` definition, and source list including all control/app firmware files and all MockHAL files. Excludes `NezhaHAL`, `DebugCommandable`, `LoopScheduler` (CODAL-dependent).
- [x] `host_tests/sim_api.cpp` created with `extern "C"` lifecycle functions: `sim_create()`, `sim_destroy(void*)`, `sim_tick(void*, uint32_t)`, `sim_command(void*, const char*, char*, int)`.
- [x] `sim_api.cpp` includes state read functions: `sim_get_pose_x/y/h`, `sim_get_enc_l/r`, `sim_get_vel_l/r`, `sim_get_pwm_l/r`.
- [x] `sim_api.cpp` includes state injection: `sim_set_enc_l/r(void*, float)`, `sim_set_otos_pose(void*, float, float, float)`, `sim_set_motor_offset(void*, int, float)`.
- [x] `cmake -S host_tests -B host_tests/build && cmake --build host_tests/build` exits 0. (Note: `-S host_tests` not `-S .` — repo root CMakeLists.txt is the firmware build.)
- [x] `libfirmware_host.dylib` (macOS) or `.so` (Linux) is produced in `host_tests/build/`.
- [x] Python one-liner loads without error: `python3 -c "import ctypes; ctypes.CDLL('./host_tests/build/libfirmware_host.dylib')"`.
- [x] All CODAL-only code paths in included firmware files are guarded with `#ifndef HOST_BUILD`.
- [x] `python3 build.py --clean` still passes (host build is separate; does not affect firmware build).
- [x] `uv run --with pytest python -m pytest` still passes.

## Implementation Plan

### Approach

1. Create `host_tests/` directory.
2. Write `host_tests/CMakeLists.txt` with the source list from the architecture doc.
3. Attempt `cmake --build`; for each CODAL-include error, add `#ifndef HOST_BUILD` guards
   to the offending source file.
4. Write `host_tests/sim_api.cpp` with the opaque handle pattern (allocates MockHAL +
   Robot + CommandProcessor on the heap via `new`).
5. Rebuild and verify the Python one-liner.

### Files to Create

- `host_tests/CMakeLists.txt`
- `host_tests/sim_api.cpp`
- `host_tests/sim_api.h` (optional: for any C++ consumers of the API)

### Files to Modify

- Any firmware source file that fails the host build due to CODAL includes — add
  `#ifndef HOST_BUILD ... #endif` guards around CODAL-only code.
- `source/control/LoopScheduler.h` — forward-declare, do not include in host build.

### sim_api.cpp opaque handle pattern

```cpp
struct SimHandle {
    MockHAL      hal;
    Robot        robot;
    CommandProcessor cmd;
    // ...
};
extern "C" void* sim_create() { return new SimHandle(); }
extern "C" void  sim_destroy(void* h) { delete static_cast<SimHandle*>(h); }
extern "C" void  sim_tick(void* h, uint32_t now_ms) {
    auto* s = static_cast<SimHandle*>(h);
    s->hal.tick(now_ms);
    s->robot.controlCollectSplitPhase(now_ms, ...);
    // one control tick
}
```

Heap use is acceptable here — simulation handles are created once per test, not in hot paths.

### HOST_BUILD guard pattern

```cpp
#ifndef HOST_BUILD
    #include "MicroBit.h"
    // CODAL-specific member or code
#endif
```

### Testing Plan

1. `cmake -S . -B host_tests/build && cmake --build host_tests/build` — zero errors.
2. `python3 -c "import ctypes; ctypes.CDLL('./host_tests/build/libfirmware_host.dylib')"` — no error.
3. `python3 build.py --clean` — firmware build unaffected.
4. `uv run --with pytest python -m pytest` — existing tests pass.

### Notes

- `DebugCommandable.cpp` depends on `I2CBus` and timing diagnostics — exclude from
  host build. The Robot `buildCommandTable()` method must compile without DebugCommandable;
  pass `nullptr` for the dbg parameter in `sim_create()`.
- `LoopScheduler` uses `uBit.sleep()` — do not include. The sim loop in `sim_tick()`
  replaces LoopScheduler's role.
- If `Robot.cpp` includes `LoopScheduler.h` (for `RobotSysCtx`), move that forward
  declaration to `Robot.h` and guard it.
- `std::vector` is allowed (C++11). No `std::variant`, no `std::function`.
