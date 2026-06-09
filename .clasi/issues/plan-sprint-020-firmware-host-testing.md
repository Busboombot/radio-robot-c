---
status: pending
---

# Plan: Sprint 020 — Firmware Host Testing

## Context

The robot firmware has no offline test capability. Every verification requires flashing hardware. The control layer (MotorController, MotionController, CommandProcessor) is already hardware-free in logic — the only CODAL coupling sits in the concrete HAL devices (`Motor.cpp`, `OtosSensor.cpp`, etc.) and `main.cpp`. The HAL abstraction issue (`hal-mockhal-implementation-plan.md`) proposes the interface layer that makes the split clean. This sprint implements that interface, then adds a host CMake build target and a Python test harness layered bottom-up: MockHAL → controllers → commands.

---

## Ticket Breakdown (4 tickets)

### Ticket 1 — HAL Interfaces + NezhaHAL + Inherit
*(from existing issue, zero behavior change)*

Create pure-virtual interfaces in `source/hal/`:
- `IMotor.h`, `ILineSensor.h`, `IColorSensor.h`, `IOtosSensor.h`, `IPortIO.h`, `IServo.h`, `Hardware.h`

Create `source/hal/NezhaHAL.h` / `.cpp`:
- Constructor: `NezhaHAL(MicroBitI2C&, MicroBitIO&, const RobotConfig&)`
- Owns concrete devices as value members; `begin()` inits sensors; `tick()` no-op
- Exposes `I2CBus& bus()` for `setI2CBus` binding

Inherit existing concrete classes:
- `Motor : IMotor`, `OtosSensor : IOtosSensor`, `LineSensor : ILineSensor`, `ColorSensor : IColorSensor`, `Servo : IServo`, `PortIO : IPortIO`

Change `source/control/MotorController.h/.cpp`:
- `Motor& _motorL/R` → `IMotor& _motorL/R`; constructor signature changes accordingly

Change `source/control/Odometry.h/.cpp`:
- `setCtx(OtosSensor*)` → `setCtx(IOtosSensor*)`

Change `source/robot/Robot.h/.cpp`:
- New constructor: `Robot(Hardware& hal, const RobotConfig& cfg)`
- Members `motorL`/`motorR`/`otos`/`line`/`colorSensor`/`gripper`/`portio` become `IMotor&`, `IOtosSensor&`, etc., initialized from `hal.motorL()` etc. in initializer list order
- Declaration order: `Hardware& hal` first, then owned values, then interface refs, then controllers

Change `source/main.cpp`:
- Replace 7 static device statics with `static NezhaHAL hardware(uBit.i2c, uBit.io, cfg)`
- Replace 7-arg Robot constructor with `Robot robot(hardware, cfg)`

**Verification**: `python3 build.py` succeeds; bench drive command works; existing `uv run --with pytest python -m pytest` passes.

---

### Ticket 2 — MockHAL + Mock Devices
*(from existing issue)*

Create `source/hal/mock/`:
- `MockMotor.h/.cpp` — integrates `cmdSpeed` into `encoderMm` on `tick(dt_ms)`: `encoderMm += (cmdSpeed/100.0f) * kNominalMaxMms * offsetFactor * dt_ms/1000.0f`; supports `requestEncoder()`/`collectEncoder()` split-phase
- `MockLineSensor.h/.cpp` — cycles a `uint16_t[N][4]` schedule on `tick()`
- `MockColorSensor.h/.cpp` — similar schedule for RGBC
- `MockOtosSensor.h/.cpp` — returns zero pose (or externally injected pose); `is_initialized()` always true
- `MockPortIO.h/.cpp` — stores digital/analog state; reads return last-written value
- `MockServo.h/.cpp` — records last `setAngle()`, no output
- `MockHAL.h/.cpp` — owns all mock devices; `tick(now_ms)` calls each device's `advance(dt_ms)` computed from last-tick timestamp

**Verification**: compile `MockHAL` on host (native g++/clang++ invocation, no CODAL) with a trivial `main` that creates `MockHAL`, ticks 100 times, asserts encoder grows.

---

### Ticket 3 — Host CMake Build + C Simulation API

**`host_tests/CMakeLists.txt`**:
```cmake
cmake_minimum_required(VERSION 3.16)
project(radio_robot_host CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(FW_SRC ${CMAKE_SOURCE_DIR}/source)
include_directories(${FW_SRC})

set(HOST_SOURCES
    ${FW_SRC}/app/CommandProcessor.cpp
    ${FW_SRC}/control/MotorController.cpp
    ${FW_SRC}/control/MotionController.cpp
    ${FW_SRC}/control/BodyVelocityController.cpp
    ${FW_SRC}/control/Odometry.cpp
    ${FW_SRC}/robot/Robot.cpp
    ${FW_SRC}/hal/mock/MockMotor.cpp
    ${FW_SRC}/hal/mock/MockOtosSensor.cpp
    ${FW_SRC}/hal/mock/MockLineSensor.cpp
    ${FW_SRC}/hal/mock/MockColorSensor.cpp
    ${FW_SRC}/hal/mock/MockPortIO.cpp
    ${FW_SRC}/hal/mock/MockServo.cpp
    ${FW_SRC}/hal/mock/MockHAL.cpp
    host_tests/sim_api.cpp          # extern "C" wrapper (see below)
)

add_library(firmware_host SHARED ${HOST_SOURCES})
target_compile_definitions(firmware_host PRIVATE HOST_BUILD=1)
```

*Note*: `DebugCommandable.cpp` depends on timing and I2C diagnostics — defer to later sprint; exclude for now.

**`host_tests/sim_api.cpp`** — `extern "C"` wrapper providing an opaque simulation handle:
```cpp
// Lifecycle
void* sim_create();           // allocates MockHAL + Robot + CommandProcessor with defaultRobotConfig()
void  sim_destroy(void*);

// Advance time — drives MockHAL physics + one Robot control tick
void  sim_tick(void*, uint32_t now_ms);

// Command interface — feeds one line, appends NUL-terminated replies to reply_buf; returns reply byte count
int   sim_command(void*, const char* line, char* reply_buf, int buf_len);

// State reads — HardwareState / MotorCommands
float sim_get_pose_x(void*);
float sim_get_pose_y(void*);
float sim_get_pose_h(void*);   // radians
float sim_get_enc_l(void*);    // mm
float sim_get_enc_r(void*);
float sim_get_vel_l(void*);    // mm/s
float sim_get_vel_r(void*);
float sim_get_pwm_l(void*);    // [-100, 100]
float sim_get_pwm_r(void*);

// State injection — override MockMotor physics for isolated controller tests
void  sim_set_enc_l(void*, float mm);
void  sim_set_enc_r(void*, float mm);
void  sim_set_otos_pose(void*, float x, float y, float h);
void  sim_set_motor_offset(void*, int motor_id, float factor);
```

Build: `cmake -S . -B host_tests/build && cmake --build host_tests/build`
Output: `host_tests/build/libfirmware_host.dylib` (macOS) or `.so` (Linux)

**Verification**: build succeeds; a trivial Python one-liner `import ctypes; ctypes.CDLL('./host_tests/build/libfirmware_host.dylib')` loads without error.

---

### Ticket 4 — Python Test Harness + Test Files

**`host_tests/firmware.py`** — ctypes loader and `Sim` class:
```python
# Loads the shared library, declares argtypes/restypes for all sim_* functions.
# Sim class wraps opaque handle with __enter__/__exit__ (context manager).
# tick_for(total_ms, step_ms=24) helper: loop tick in steps, returns elapsed ticks.
# send_command(line) helper: calls sim_command, splits reply lines.
```

**`host_tests/conftest.py`** — pytest fixtures:
```python
@pytest.fixture(scope="session", autouse=True)
def build_lib():
    # cmake configure + build if .dylib is missing or sources are newer
    ...

@pytest.fixture
def sim():
    s = Sim()
    yield s
    s.destroy()
```

**`host_tests/test_mock_hal.py`** — MockHAL physics:
- At 100% speed, encoder grows at ≥ 80% of kNominalMaxMms after 1 second of ticks
- At 0% speed, encoder is stable
- At -100% speed, encoder decreases
- `sim_set_enc_l()` injection: encoder read-back matches injected value after one tick

**`host_tests/test_motor_controller.py`** — velocity PID:
- Set encoder directly (bypass physics), target 200 mm/s, tick 50 cycles: PWM converges and stays within ±10 of steady-state
- Integral windup: force encoder to zero for 2s at high target — integrator output should clamp at iMax, not overflow
- Stop: after `sim_command("S")`, PWM goes to 0 within one tick

**`host_tests/test_motion_controller.py`** — drive state machines:
- D command 500 mm: tick until EVT reply, assert `sim_get_enc_l() + sim_get_enc_r() ≈ 1000 mm` (sum of both wheels)
- D command with dist=0: gets immediate EVT OK (edge case)
- S (stream) keepalive: no keepalive for > sTimeoutMs → motors stop

**`host_tests/test_command_processor.py`** — command parsing + routing:
- `PING` → reply contains `OK`
- `HELLO` → reply contains robot ID field
- Unknown verb → reply contains `ERR`
- `VW v=500 w=0` → sim_get_vel_l/r approach 500 after settling ticks
- `SET velKp=2.0` → config field updated (verify via `GET velKp`)

Run: `uv run --with pytest python -m pytest host_tests/ -v`

---

## Critical Files

| File | Action |
|------|--------|
| `source/hal/IMotor.h` (new) | Pure-virtual IMotor interface |
| `source/hal/Hardware.h` (new) | Abstract HAL factory/registry |
| `source/hal/NezhaHAL.h/.cpp` (new) | Real hardware wrapper, CODAL only |
| `source/hal/mock/MockHAL.h/.cpp` (new) | Simulation HAL, no CODAL |
| `source/hal/mock/MockMotor.h/.cpp` (new) | Encoder integrator physics |
| `source/control/MotorController.h/.cpp` | `Motor&` → `IMotor&` |
| `source/control/Odometry.h/.cpp` | `OtosSensor*` → `IOtosSensor*` |
| `source/robot/Robot.h/.cpp` | Constructor: `Hardware&` + single cfg |
| `source/main.cpp` | Replace 7 statics with `NezhaHAL` |
| `host_tests/CMakeLists.txt` (new) | Host-target build, no CODAL |
| `host_tests/sim_api.cpp` (new) | `extern "C"` simulation wrapper |
| `host_tests/firmware.py` (new) | Python ctypes loader + Sim class |
| `host_tests/test_*.py` (new) | pytest test files |

## Verification (end-to-end)

1. Firmware build still passes: `python3 build.py`
2. Bench smoke test: flash, drive D dist=500, verify telemetry reports correct encoder
3. Host build: `cmake -S . -B host_tests/build && cmake --build host_tests/build`
4. Full test suite: `uv run --with pytest python -m pytest host_tests/ -v` — all tests pass
5. Existing tests unchanged: `uv run --with pytest python -m pytest tests/` — still pass
