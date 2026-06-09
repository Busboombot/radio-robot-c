---
id: '001'
title: HAL interfaces + NezhaHAL + concrete class inheritance
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: hal-mockhal-implementation-plan.md
completes_issue: false
---

# HAL interfaces + NezhaHAL + concrete class inheritance

## Description

Introduce a pure-virtual HAL interface layer and a concrete `NezhaHAL` that wraps the
existing device objects. Change Robot's constructor from 7 device refs to `Hardware& hal`.
Update all downstream consumers (`MotorController`, `Odometry`) to use interface types.
This is a zero-behavior-change refactor: the same code paths run; only types change.

The key deliverables are:
- Six pure-virtual interfaces in `source/hal/`
- `Hardware` abstract base class (factory/registry)
- `NezhaHAL` concrete implementation owning all seven devices as value members
- Existing concrete classes inherit their interface
- `MotorController` and `Odometry` use interface refs/pointers
- `Robot` constructor takes `Hardware& hal`
- `main.cpp` constructs `NezhaHAL` and passes it to Robot

## Acceptance Criteria

- [x] `source/hal/IMotor.h` created: pure-virtual `setSpeed`, `requestEncoder`, `collectEncoder`, `readEncoderMmF`, `readEncoderMmFAtomic`, `readEncoderMmFSettle`, `resetEncoder`.
- [x] `source/hal/ILineSensor.h`, `IColorSensor.h`, `IOtosSensor.h`, `IPortIO.h`, `IServo.h` created with matching pure-virtual methods.
- [x] `source/hal/Hardware.h` created: abstract base with `motorL()`, `motorR()`, `lineSensor()`, `colorSensor()`, `otos()`, `portIO()`, `gripper()`, `begin()`, `tick(uint32_t)`.
- [x] `source/hal/NezhaHAL.h/.cpp` created: owns all devices as value members; `begin()` calls `otos.begin()`, `line.begin()`, `color.begin()`; exposes `I2CBus& bus()`.
- [x] `Motor.h` adds `: public IMotor`; `OtosSensor.h` adds `: public IOtosSensor`; same pattern for LineSensor, ColorSensor, PortIO, Servo.
- [x] `MotorController` members `_motorL`, `_motorR` changed to `IMotor&`; constructor signature updated.
- [x] `Odometry::setCtx(OtosSensor*)` changed to `setCtx(IOtosSensor*)`.
- [x] `Robot.h/.cpp` constructor changed to `Robot(Hardware& hal, const RobotConfig& cfg)`; members `motorL`, `motorR`, `otos`, `line`, `colorSensor`, `gripper`, `portio` become interface refs initialized from `hal.*()`.
- [x] `main.cpp` replaces 7 static device declarations with `static NezhaHAL hardware(uBit.i2c, uBit.io, cfg)`; Robot constructor call updated.
- [x] `python3 build.py --clean` succeeds with no warnings.
- [x] BSS section size checked before and after build; delta documented in commit message.
- [ ] Flash robot; send `D dist=500`; `EVT done D` received; encoder telemetry correct.
- [x] `uv run --with pytest python -m pytest` passes (all existing tests).

## Implementation Plan

### Approach

Create all interface headers first (pure declarations, no cpp needed). Then create
NezhaHAL (new file). Then update concrete classes one at a time (add `: public IInterface`
— one line each). Then update MotorController and Odometry. Then update Robot.h/.cpp.
Then update main.cpp. Build after each group.

### Files to Create

- `source/hal/IMotor.h`
- `source/hal/ILineSensor.h`
- `source/hal/IColorSensor.h`
- `source/hal/IOtosSensor.h`
- `source/hal/IPortIO.h`
- `source/hal/IServo.h`
- `source/hal/Hardware.h`
- `source/hal/NezhaHAL.h`
- `source/hal/NezhaHAL.cpp`

### Files to Modify

- `source/hal/Motor.h` — add `: public IMotor`; include `IMotor.h`
- `source/hal/OtosSensor.h` — add `: public IOtosSensor`
- `source/hal/LineSensor.h` — add `: public ILineSensor`
- `source/hal/ColorSensor.h` — add `: public IColorSensor`
- `source/hal/PortIO.h` — add `: public IPortIO`
- `source/hal/Servo.h` — add `: public IServo`
- `source/control/MotorController.h/.cpp` — `Motor&` → `IMotor&`
- `source/control/Odometry.h/.cpp` — `OtosSensor*` → `IOtosSensor*`
- `source/robot/Robot.h/.cpp` — constructor + member types
- `source/main.cpp` — NezhaHAL, updated Robot construction

### Testing Plan

1. `python3 build.py --clean` — zero warnings is the pass gate.
2. Check BSS delta via `arm-none-eabi-size` on the ELF output.
3. Flash via `mbdeploy deploy robot --clean`.
4. Send `D dist=500` via serial or rogo; confirm `EVT done D` and encoder telemetry.
5. `uv run --with pytest python -m pytest` — all tests must pass.

### Notes

- Declaration order in `Robot.h` is load-bearing. `Hardware& hal` must be declared first
  (before the interface refs); interface refs must be declared before `motorController`
  (which binds them in its constructor).
- NezhaHAL must not be included in `host_tests/CMakeLists.txt` (CODAL dependency).
  This ticket does not create the host build — that is ticket 020-003. Just ensure
  NezhaHAL includes are guarded with a forward declaration boundary.
- `OdomCtx` currently holds `OtosSensor*`; after this ticket it must hold `IOtosSensor*`.
  Verify that all OdomCtx usage sites compile with the interface pointer.
