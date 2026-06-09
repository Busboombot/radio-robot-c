---
status: pending
---

# HAL + MockHAL Implementation Plan

## Context

The robot currently constructs hardware objects (Motor, OtosSensor, LineSensor, etc.) as concrete statics in main.cpp and passes them by reference into Robot, which passes them into controllers (MotorController, Odometry, etc.). There is no abstraction layer — controllers hold direct references to concrete device classes.

The goal is to introduce a `Hardware` interface (HAL factory/registry) so:
1. Controllers can be tested or simulated without real I2C hardware
2. Robot is constructed from a `Hardware&` instead of 7 separate device references
3. A `MockHAL` can be dropped in to run the full control loop off-board

---

## Architecture Overview

### Layer Stack (after change)

```
main.cpp
  └── NezhaHAL (Hardware*)       OR   MockHAL (Hardware*)
        └── IMotor, IOtosSensor, ...
              └── Robot(hal)
                    └── MotorController(IMotor&, IMotor&)
                    └── Odometry(IOtosSensor*)
                    └── MotionController(...)
```

### Interface Hierarchy

Each device type gets a pure-virtual interface. Concrete classes add `: public I<Name>`.

| Interface      | Concrete (real)  | Concrete (mock)     |
|----------------|------------------|---------------------|
| `IMotor`       | `Motor`          | `MockMotor`         |
| `ILineSensor`  | `LineSensor`     | `MockLineSensor`    |
| `IColorSensor` | `ColorSensor`    | `MockColorSensor`   |
| `IOtosSensor`  | `OtosSensor`     | `MockOtosSensor`    |
| `IPortIO`      | `PortIO`         | `MockPortIO`        |
| `IServo`       | `Servo`          | `MockServo`         |

`Hardware` (abstract HAL class): lazy factory + registry. Returns `I<Name>&` for each device. Has `begin()` and `tick(uint32_t now_ms)`.

---

## Files to Create

**Interfaces** (`source/hal/`):
- `IMotor.h` — `setSpeed(int8_t)`, `requestEncoder()`, `collectEncoder()`, `readEncoderMmF(cfg)`, `resetEncoder()`
- `ILineSensor.h` — `begin()`, `is_initialized()`, `readValues(uint16_t[4])`, `readNormalized(uint16_t[4])`
- `IColorSensor.h` — `begin()`, `is_initialized()`, `readRGBC(...)`, `pollRGBC(...)`
- `IOtosSensor.h` — `begin()`, `is_initialized()`, `readTransformed()`, `calibrateImu()`, `resetTracking()`
- `IPortIO.h` — `setDigital(port,v)`, `readDigital(port)`, `setAnalog(port,v)`, `readAnalog(port)`
- `IServo.h` — `setAngle(degrees)`
- `Hardware.h` — abstract HAL: `motorL()`, `motorR()`, `lineSensor()`, `colorSensor()`, `otos()`, `portIO()`, `gripper()`, `begin()`, `tick(now_ms)`

**Real HAL** (`source/hal/`):
- `NezhaHAL.h` / `NezhaHAL.cpp`
  - Constructor: `NezhaHAL(MicroBitI2C& i2c, MicroBitIO& io, const RobotConfig& cfg)`
  - Owns I2CBus, Motor×2, OtosSensor, LineSensor, ColorSensor, Servo, PortIO as direct value members (no heap)
  - `begin()` calls `otos.begin()`, `line.begin()`, `color.begin()`
  - `tick()` is a no-op for real hardware
  - Exposes `I2CBus& bus()` for optional binding in main.cpp (setI2CBus)

**Mock HAL** (`source/hal/mock/`):
- `MockMotor.h` — integrates commanded speed into encoder mm with configurable offset factor + noise
- `MockLineSensor.h` — cycles through a preset `uint16_t[N][4]` table on a ~3-second schedule
- `MockColorSensor.h` — returns values from a configurable schedule, advances on `tick()`
- `MockOtosSensor.h` — returns zero pose (or computes from encoder deltas passed in)
- `MockPortIO.h` — stores digital/analog state, returns last written value on read
- `MockServo.h` — records last angle, no-op output
- `MockHAL.h` / `MockHAL.cpp` — owns all mock devices, `tick(now_ms)` calls each mock's `advance(dt_ms)`

---

## Files to Modify

### `source/hal/Motor.h`
Add `: public IMotor` to class declaration. No body changes needed.

### `source/hal/LineSensor.h`, `ColorSensor.h`, `OtosSensor.h`, `PortIO.h`, `Servo.h`
Same: add `: public I<Name>` to each class declaration.

### `source/control/MotorController.h` / `.cpp`
- Change `Motor& _motorL`, `Motor& _motorR` → `IMotor& _motorL`, `IMotor& _motorR`
- Constructor: `MotorController(IMotor& left, IMotor& right, const RobotConfig& cal)`
- All call sites (e.g. `_motorL.setSpeed(...)`, `_motorL.requestEncoder()`) unchanged — same method names

### `source/control/Odometry.h` / `.cpp`
- `setCtx(OtosSensor* otos)` → `setCtx(IOtosSensor* otos)`, member `_otos` type changes

### `source/robot/Robot.h` / `.cpp`
**Key change**: Constructor takes `Hardware& hal` instead of 7 device refs.
```cpp
struct Robot {
    Hardware& hal;           // declared first (initialization order)
    RobotConfig config;
    RobotStateContainer state;
    
    IMotor& motorL;          // = hal.motorL()
    IMotor& motorR;          // = hal.motorR()
    IOtosSensor& otos;       // = hal.otos()
    ILineSensor& line;       // = hal.lineSensor()
    IColorSensor& colorSensor; // = hal.colorSensor()
    IServo& gripper;         // = hal.gripper()
    IPortIO& portio;         // = hal.portIO()
    
    MotorController motorController;
    Odometry odometry;
    MotionController motionController;
    ...
};

// Constructor initializer list order must match declaration order:
Robot::Robot(Hardware& hal, const RobotConfig& cfg)
    : hal(hal), config(cfg), state(...),
      motorL(hal.motorL()), motorR(hal.motorR()),
      otos(hal.otos()), line(hal.lineSensor()),
      colorSensor(hal.colorSensor()), gripper(hal.gripper()), portio(hal.portIO()),
      motorController(motorL, motorR, config),
      ...
```

### `source/main.cpp`
Replace 7 static device declarations + individual `begin()` calls with:
```cpp
static NezhaHAL hardware(uBit.i2c, uBit.io, cfg);
comm.begin(rfChannel);
hardware.begin();                          // calls otos/line/color .begin()
static Robot robot(hardware, cfg);
robot.motorController.setI2CBus(&hardware.bus());
```

---

## MockMotor Physics

```
tick(dt_ms):
    velocity_mms = (_cmdSpeed / 100.0f) * kNominalMaxMms * _offsetFactor
    velocity_mms += gaussian_noise(_noiseMms)
    _encoderMm += velocity_mms * (dt_ms / 1000.0f)
    _lastEncoderSnapshot = _encoderMm   // for collectEncoder()

requestEncoder() → sets _pending flag
collectEncoder() → returns int32_t(_lastEncoderSnapshot / kDegreesPerMm)
```

Configurable: `offsetFactor` (default 1.0), `noiseMms` (default ~2mm/s stddev).

## MockLineSensor Schedule

Holds a table of `uint16_t[N][4]` readings and a period (default 3000ms). `advance(dt_ms)` increments phase; `readNormalized()` returns the current row. Default table simulates a line crossing (first channel dark → all dark → last channel dark).

## MockColorSensor Schedule

Similar: holds `ColorReading[N]` + durations. Default returns black until ~1s, then red-ish, then black again.

---

## MockHAL Null-Object Pattern for Missing Sensors

`Hardware::otos()`, `lineSensor()`, etc. return default implementations when not physically present. The real classes already guard on `is_initialized()` (returning 0 when absent). Mock classes always return `is_initialized() = true` and their scheduled values. No "null object" wrapper needed — the existing guard pattern handles it.

---

## Verification

1. **Build (real)**: `python3 build.py` should produce same binary as before. Robot constructor change is API-only; all internal calls to motorL/motorR etc. are identical.
2. **Bench test**: `mbdeploy deploy robot --clean` → run drive command, confirm encoders reported correctly in telemetry.
3. **MockHAL standalone test**: Create `tests/dev/test_mock_hal.py` — instantiate MockHAL + Robot entirely in Python simulation using ctypes or a stub; advance tick 100× at 24ms intervals; assert encoder values grow at expected rate.
4. **Existing tests**: `uv run --with pytest python -m pytest` must still pass.

---

## Sprint Suggestion

This is a clean two-ticket sprint:
- **Ticket 1**: Interfaces + NezhaHAL + inherit changes to existing classes + Robot/main changes (no behavior change, green tests)
- **Ticket 2**: MockHAL + all Mock device classes + standalone mock test

The interface changeover is zero-behavior-change (same code paths, different types). MockHAL can be added as pure-new code without touching the real hardware path.
