---
id: "003"
title: "Full Sensor and OTOS Command Set"
status: open
branch: sprint/003-full-sensor-and-otos-command-set
use-cases: []
issues:
  - plan-c-port-of-radio-robot-firmware
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 003: Full Sensor and OTOS Command Set

## Goals

Achieve feature parity with the TypeScript firmware for all 30+ commands. After this sprint
every command the Python host stack can send — OTOS pose, line sensor, color sensor, gripper,
and GPIO — works correctly. The firmware is fully capable for classroom use.

## Problem

After sprint 2 the robot drives and reports odometry, but ~15 commands are unimplemented:
all OTOS commands, line sensor (LS), color sensor (CS), gripper (G), and port I/O (P, PA).
These are needed for most robot tasks: reading sensor arrays, using optical tracking, and
actuating the gripper. Without them the firmware is not usable for the league curriculum.

## Solution

Add all OTOS and sensor/peripheral command handlers to CommandProcessor. The HAL drivers
(OtosSensor, LineSensor, ColorSensor, GripperServo, PortIO) were written in sprint 1 and
injected as nullable pointers. This sprint plugs in the command handlers that call those
drivers and emit the correct protocol responses. No new classes or files are needed beyond
CommandProcessor changes.

## Success Criteria

- `OI` initialises OTOS and responds `ACK:OI`
- `OP` returns pose as `OP+XXXX-YYYY+HHH` with centidegree heading
- `OL` sets linear scalar; `OA` sets angular scalar; `OK` calibrates IMU; `OZ` resets tracking; `OR` returns velocity; `OV` sets pose offset
- `LS` returns four grayscale values (format `LS+NNN+NNN+NNN+NNN`)
- `CS` returns RGBC values
- `G+90` moves gripper to 90°; `G` with no arg returns current angle
- `P 1 1` sets digital port J1 high; `P 1 0` clears it; `PA 2` returns analog reading on J2
- All commands return `ERR:command` when optional peripheral is absent (null pointer)
- `K` dump still lists all params from sprints 1-2 (no regressions)
- Streaming CS and LS fields appear when sensors are present (every `encReportEvery` ticks)

## Scope

### In Scope

**CommandProcessor additions — OTOS commands:**
- `O` — init + calibrate shortcut (calls `begin()` + `init()` + `calibrateImu(255)`)
- `OI` — init only (calls `begin()` + `init()`)
- `OK` — calibrate IMU with optional sample count arg (default 255); responds `ACK:OK`
- `OZ` — reset tracking (`resetTracking()`); responds `ACK:OZ`
- `OR` — return velocity as `OR+VX+VY+VH`
- `OP` — return current pose from OTOS as `OP+X_mm+Y_mm+H_cdeg` (apply LSB conversions: 1 pos LSB ≈ 0.305 mm; 1 heading LSB ≈ 0.00549°)
- `OV` — set pose offset (`setPositionRaw`)
- `OL` — set/get linear scalar
- `OA` — set/get angular scalar

**CommandProcessor additions — sensor and peripheral commands:**
- `LS` — read 4 grayscale values from LineSensor; format `LS+NNN+NNN+NNN+NNN`
- `CS` — read RGBC from ColorSensor; format `CS+RRR+GGG+BBB+CCC`
- `G` — gripper: `G+90` sets to 90°; `G` alone queries angle; calls `GripperServo::setAngle()`
- `P` — digital port: `P portNum val` sets J1-J4 digital; calls `PortIO::setDigital()`; `P portNum` reads digital
- `PA` — analog port: `PA portNum` reads analog from J1-J4; calls `PortIO::readAnalog()`

**Streaming additions:**
- CS and LS output in `tick()` streaming block (guarded by null check on sensor pointers)

**Null-guard policy:**
- Every command handler checks its peripheral pointer; on null responds `ERR:command`

### Out of Scope

- Navigation layer PoseProvider/PathFollower (sprint 4)
- Ratio PID motor control (sprint 5)
- G go-to command (sprint 5) — note: `G` gripper command IS in this sprint; `G` go-to is sprint 5 with a different signature
- ExternalCameraPoseProvider (future, not in roadmap)

## Test Strategy

Hardware-in-the-loop:

1. Build and deploy
2. OTOS init: `OI` — ACK; `OP` — pose near zero at start; `OK` — runs calibration
3. OTOS pose: manually move robot ~300 mm forward; `OP` shows X ≈ 300
4. OTOS zero: `OZ` then `OP` — pose back to ~0
5. Line sensor: hold robot over a line; `LS` — one or more channels reads low
6. Color sensor: `CS` over different surfaces — RGBC values differ
7. Gripper: `G+0` fully open; `G+90` halfway; `G+180` fully closed (verify servo movement)
8. Port digital: `P 1 1` — J1 high, can verify with multimeter or attached LED
9. Port analog: `PA 1` — returns plausible 0-1023 reading
10. Null guard: disconnect OTOS; `OP` should return `ERR:OP` not crash
11. Regression: `S+200+200`, `ENC`, `SO`, `K` all still work

## Architecture Notes

**No new files or classes in this sprint.** All work is adding handler functions to
CommandProcessor and connecting them to the HAL drivers that already exist from sprint 1.
The HAL driver pointers are already injected into CommandProcessor via `init()`; this sprint
just implements the missing command handlers.

**LSB conversion for OTOS is in CommandProcessor, not OtosSensor.** OtosSensor returns raw
register values. CommandProcessor applies the conversion factors (0.305 mm/LSB for position,
0.00549°/LSB for heading, then convert degrees to centidegrees for protocol) to produce
protocol-compatible integers. This keeps OtosSensor a pure I2C driver with no protocol
knowledge.

**`G` command disambiguation.** The gripper command `G+NNN` takes a single signed angle
argument. The go-to command added in sprint 5 is `G+X+Y+Speed` with three arguments. The
command table entry added in this sprint handles single-arg form; sprint 5 replaces it with
multi-arg form. Plan the dispatch table entry to be easily replaced.

**Reference files from the TypeScript original:**
- `radio-robot/src/command.ts` — O*, LS, CS, G (gripper), P, PA handler implementations
- `radio-robot/src/otos.ts` — OTOS register map and LSB constants

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

Tickets execute serially in the order listed.
