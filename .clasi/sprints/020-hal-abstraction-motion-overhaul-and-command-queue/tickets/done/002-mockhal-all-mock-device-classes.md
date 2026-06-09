---
id: '002'
title: MockHAL + all mock device classes
status: done
use-cases:
- SUC-002
depends-on:
- 020-001
github-issue: ''
issue: hal-mockhal-implementation-plan.md
completes_issue: true
---

# MockHAL + all mock device classes

## Description

Create the `source/hal/mock/` directory with six mock device classes and a `MockHAL`
aggregate. These classes implement the HAL interfaces introduced in ticket 020-001 but
with no CODAL dependency. `MockMotor` integrates commanded speed into an encoder
accumulator on each `tick(dt_ms)`. The other mocks cycle through preset value schedules
or return stored state. `MockHAL` owns all mock devices as value members and advances
them on each `tick(now_ms)`.

This ticket does not create the host CMake build or Python test harness (tickets 020-003
and 020-004 do that). It only creates the mock classes. Verify compile via a standalone
native g++/clang++ invocation at the end.

## Acceptance Criteria

- [x] `source/hal/mock/MockMotor.h/.cpp` created: integrates `cmdSpeed` into `encoderMm` on `tick(dt_ms)` using `encoderMm += (cmdSpeed / 100.0f) * kNominalMaxMms * offsetFactor * (dt_ms / 1000.0f)`; supports `requestEncoder()` / `collectEncoder()` split-phase returning `int32_t(encoderMm)`.
- [x] `source/hal/mock/MockLineSensor.h/.cpp` created: cycles a `uint16_t[N][4]` schedule on `tick()`; `readNormalized()` returns current row; `is_initialized()` always true.
- [x] `source/hal/mock/MockColorSensor.h/.cpp` created: similar schedule for RGBC; `is_initialized()` always true.
- [x] `source/hal/mock/MockOtosSensor.h/.cpp` created: returns zero pose by default; `setInjectedPose(x, y, h)` for test injection; `is_initialized()` always true.
- [x] `source/hal/mock/MockPortIO.h/.cpp` created: stores digital/analog state; reads return last-written value.
- [x] `source/hal/mock/MockServo.h/.cpp` created: records last `setAngle()`, no output.
- [x] `source/hal/mock/MockHAL.h/.cpp` created: owns all six mock devices as value members; `tick(now_ms)` computes signed `dt_ms` and calls each device's `advance(dt_ms)`.
- [x] All mock classes implement their respective `I<Name>` interface.
- [x] Mock files compile on host (native clang++ or g++) with a trivial driver: `MockHAL hal; hal.tick(0); hal.tick(24);` — no CODAL, no MicroBit.
- [x] `kNominalMaxMms` exposed as a public constant on `MockMotor` so test harness can reference it.
- [x] `python3 build.py --clean` still passes (mocks not included in firmware build).
- [x] `uv run --with pytest python -m pytest` still passes.

## Implementation Plan

### Approach

Create the `source/hal/mock/` directory. Write each mock class in order: MockMotor
first (physics), then the schedule-based sensors, then MockPortIO and MockServo, then
MockHAL. Do a standalone compile check after MockHAL.

### Files to Create

- `source/hal/mock/MockMotor.h` / `MockMotor.cpp`
- `source/hal/mock/MockLineSensor.h` / `MockLineSensor.cpp`
- `source/hal/mock/MockColorSensor.h` / `MockColorSensor.cpp`
- `source/hal/mock/MockOtosSensor.h` / `MockOtosSensor.cpp`
- `source/hal/mock/MockPortIO.h` / `MockPortIO.cpp`
- `source/hal/mock/MockServo.h` / `MockServo.cpp`
- `source/hal/mock/MockHAL.h` / `MockHAL.cpp`

### MockMotor Physics

```cpp
static constexpr float kNominalMaxMms = 400.0f;
float _offsetFactor = 1.0f;
float _encoderMm    = 0.0f;
int8_t _cmdSpeed    = 0;

void tick(uint32_t dt_ms) {
    float vel = (_cmdSpeed / 100.0f) * kNominalMaxMms * _offsetFactor;
    _encoderMm += vel * (dt_ms / 1000.0f);
}
void setSpeed(int8_t s) { _cmdSpeed = s; }
void requestEncoder() {}
int32_t collectEncoder() { return static_cast<int32_t>(_encoderMm); }
```

Noise omitted for determinism. A `setNoiseMms(float)` no-op stub acceptable.

### MockHAL tick timing

Use signed delta to avoid uint32 underflow:
```cpp
int32_t dt = static_cast<int32_t>(now_ms - _lastTickMs);
if (dt > 0) { _mockMotorL.tick(static_cast<uint32_t>(dt)); ... }
_lastTickMs = now_ms;
```

### Files to Modify

None (all new files).

### Testing Plan

1. Standalone compile check: `clang++ -std=c++11 source/hal/mock/*.cpp -I source -c`.
2. `python3 build.py --clean` — firmware build must not regress.
3. `uv run --with pytest python -m pytest` — existing tests must pass.

### Notes

- MockHAL must not include any CODAL header. If `Config.h` transitively pulls in CODAL,
  use `#ifdef HOST_BUILD` guards or a minimal stub `HostConfig.h` — defer to ticket 020-003
  which sorts out the HOST_BUILD boundary.
- `setInjectedPose` on `MockOtosSensor` enables deterministic position tests in ticket 020-004.
- `readEncoderMmF(cfg)` on MockMotor should return `_encoderMm` (float) — check the exact
  signature in `IMotor.h` produced by ticket 020-001 and match it exactly.
