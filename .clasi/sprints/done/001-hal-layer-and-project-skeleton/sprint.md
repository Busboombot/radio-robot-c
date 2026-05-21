---
id: '001'
title: HAL Layer and Project Skeleton
status: done
branch: sprint/001-hal-layer-and-project-skeleton
use-cases: []
issues:
- plan-c-port-of-radio-robot-firmware
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 001: HAL Layer and Project Skeleton

## Goals

Get the firmware compiling, booting on micro:bit V2, announcing `DEVICE:` over serial, and
responding to `HELLO`. This sprint lays the foundational project structure: all eight HAL
drivers, shared type headers, and the Robot skeleton with its init/run loop. Nothing from
the control or nav layers is included.

## Problem

The existing `source/main.cpp` is a placeholder. There is no C++ project structure — no HAL
drivers, no type system, no application skeleton. The firmware cannot compile into anything
useful. All subsequent sprints depend on this foundation existing.

## Solution

Create the full `source/` directory structure defined in the issue. Implement the eight HAL
classes (NezhaV2, OtosSensor, LineSensor, ColorSensor, GripperServo, PortIO, SerialPort,
Radio), the two type headers (Config.h, Protocol.h), the Announcer, and a Robot skeleton
that constructs and initializes subsystems in the correct order and runs the tick loop.
Replace `source/main.cpp` with a 15-line version that constructs Robot and calls `run()`.

Because `CMakeLists.txt` uses `RECURSIVE_FIND_FILE` on `source/`, no CMake changes are needed —
adding files is sufficient.

## Success Criteria

- `python build.py` succeeds (Docker-based CODAL build produces `.hex`)
- Firmware boots on micro:bit V2 and emits `DEVICE:<type>:<name>:<hwName>:<serial>` over serial
- Sending `HELLO` over serial returns the same announcement string
- Firmware remains stable (no panic/fault) for at least 60 seconds without further input

## Scope

### In Scope

**Types (`source/types/`)**
- `Config.h` — `CalibParams` struct, `MotorGains` struct, `DriveMode` enum (IDLE, STREAMING, TIMED, DISTANCE)
- `Protocol.h` — command prefix constants, reply format string constants

**HAL (`source/hal/`)**
- `NezhaV2` — motor driver + encoders: `setPwm(leftPct, rightPct)`, `readEncoder(isLeft)` → int32_t mm, `resetEncoders()`. Constants: LEFT_MOTOR=M2, RIGHT_MOTOR=M1, LEFT_FWD_SIGN=+1, RIGHT_FWD_SIGN=-1
- `OtosSensor` — SparkFun OTOS at I2C 0x17: `begin()`, `init()`, `calibrateImu(samples)`, `resetTracking()`, `getPositionRaw(x,y,h)` / `setPositionRaw(x,y,h)`, `getVelocityRaw(x,y,h)`, linear/angular scalar getters/setters
- `LineSensor` — I2C 0x1A thin wrapper; `readValues(uint16_t[4])` returning 4 grayscale readings
- `ColorSensor` — I2C 0x39/0x43 thin wrapper; `readRGBC(r,g,b,c)`
- `GripperServo` — servo on P1; `setAngle(degrees)`
- `PortIO` — J1-J4 GPIO; `setDigital(port, val)`, `readDigital(port)`, `readAnalog(port)`
- `SerialPort` — line-buffered 115200 serial; `readLine(buf, len)` → bool, `send(const char*)`, `sendf(fmt, ...)`
- `Radio` — group 10, channel 0, power 7; 4-slot ring buffer; `poll(buf, len, isRelayed&)` → bool, `send(msg, relay)`

**App (`source/app/`)**
- `Announcer` — emits `DEVICE:<type>:<name>:<hwName>:<serial>` on startup; intercepts `HELLO` before CommandProcessor and re-emits announcement
- `Robot` — owns static instances of all subsystems; controls init order (I2C → sensors → motor → cmd → serial → radio → announcer); `run()` loop: drain serial, drain radio, call `cmd.tick()`, `uBit.sleep(tickMs)`; ReplyFn as `void(*)(const char*, void*)` with `void* ctx` for serial-vs-radio/relay-vs-direct

**Entry point**
- `source/main.cpp` — ~15 lines; constructs static Robot, calls `robot.run()`

### Out of Scope

- `MotorController` and `Odometry` (sprint 2)
- `CommandProcessor` with command dispatch (sprint 2)
- All motion, sensor, OTOS commands (sprints 2-3)
- Navigation layer (sprint 4)
- Ratio PID and G command (sprint 5)

## Test Strategy

Hardware-in-the-loop only — the CODAL framework does not support unit testing off-device.

1. Build: `python build.py` — confirms all C++ compiles cleanly with no warnings treated as errors
2. Deploy: `python scripts/deploy.py` — flash `.hex` to micro:bit V2
3. Open serial monitor at 115200 baud and observe `DEVICE:` announcement on boot
4. Send `HELLO\n` and verify announcement is re-emitted
5. Leave running 60 s; confirm no panic LED pattern

## Architecture Notes

**Static instances in Robot** — all subsystems are member variables of the single static
`Robot` object declared in `main.cpp`. No heap allocation; explicit member ordering in the
constructor body controls initialization sequence and eliminates the static-init-order fiasco.

**ReplyFn as `void(*)(const char*, void*)`** — avoids `std::function` and heap. The `void* ctx`
carries a small struct identifying serial vs. radio and relay vs. direct. Defined in Protocol.h.

**Nullable optional peripherals** — OtosSensor, LineSensor, ColorSensor, GripperServo, and
PortIO are stored as pointers in Robot and passed as nullable pointers wherever used. The robot
boots and announces without any optional sensor present.

**Reference files from the TypeScript original:**
- `radio-robot/src/nezha.ts` — motor sign conventions and encoder constants
- `radio-robot/src/otos.ts` — OTOS I2C register map and LSB conversion factors (1 pos LSB ≈ 0.305 mm; 1 heading LSB ≈ 0.00549°)

## GitHub Issues

None linked yet.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Create shared type headers Config.h and Protocol.h | — |
| 002 | Implement NezhaV2 HAL driver (I2C motor and encoder) | 001 |
| 003 | Implement SerialPort and Radio HAL drivers | 001 |
| 004 | Implement sensor HAL drivers (OtosSensor, LineSensor, ColorSensor, GripperServo, PortIO) | 001, 002 |
| 005 | Implement Announcer, Robot skeleton, and replacement main.cpp | 002, 003, 004 |
| 006 | Build verification — run python build.py and fix all compile errors | 005 |

Tickets execute serially in the order listed.
