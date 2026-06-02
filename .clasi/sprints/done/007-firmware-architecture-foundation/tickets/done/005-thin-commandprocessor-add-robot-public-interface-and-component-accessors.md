---
id: '005'
title: "Thin CommandProcessor \u2014 add Robot public interface and component accessors"
status: done
use-cases:
- SUC-005
depends-on:
- '003'
- '004'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 005 — Thin CommandProcessor — add Robot public interface and component accessors

## Description

After Tickets 003 and 004, `CommandProcessor` no longer owns drive state or `tick()`,
but it still injects hardware pointers (`_motor`, `_mc`, `_odo`, `_otos`, `_line`,
`_color`, `_gripper`, `_portio`, `_config`) and calls them directly. This ticket
completes the thinning:

- `Robot` gets its full public action interface, query interface returning structs,
  and component accessors (`config()`, `motor()`, `driveController()`, `odometry()`,
  `otos()`, `lineSensor()`, `colorSensor()`, `gripper()`, `portIO()`).
- `CommandProcessor` is reduced to `Robot& _robot` + `process(line, sink)`. All
  command handlers call Robot methods or Robot accessors.
- `CommandProcessor::init()` and `setConfig()` are deleted. Constructor takes
  `Robot&` only.

All existing wire commands (X/S/T/D/G, ENC/EZ/SO/SZ/SI, K*/O*/OTOS, LS/CS, gripper,
P/PA) continue to work identically. Wire strings are unchanged.

## Acceptance Criteria

- [x] `CommandProcessor` members: `Robot& _robot` and parse helpers only (`parseSignedArgs`,
  `clampInt`, `clampMinSpeed` static methods). No hardware pointers. No config pointers.
  No `_cal`, `_motor`, `_mc`, `_odo`, `_otos`, `_line`, `_color`, `_gripper`, `_portio`.
- [x] `CommandProcessor::init()` does not exist.
- [x] `CommandProcessor::setCalib()` / `setConfig()` does not exist.
- [x] `CommandProcessor::tick()` does not exist.
- [x] `Robot` public action interface implemented: `stop()`, `streamDrive()`,
  `timedDrive()`, `distanceDrive()`, `goTo()`, `setGripperAngle()`, `zeroEncoders()`,
  `setPose()`, `zeroOdometry()`, plus OTOS and portIO delegates.
- [x] `Robot` query interface: `getEncoders()` → `EncoderReading`; `getPose()` → `Pose`.
- [x] `Robot` component accessors: `config()`, `motor()`, `driveController()`,
  `odometry()`, `otos()`, `lineSensor()`, `colorSensor()`, `gripper()`, `portIO()`.
- [x] All 35+ commands in `CommandProcessor` verified: K* setters write via
  `_robot.config()` or `_robot.motor()`; O* OTOS commands delegate via `_robot.otos()`;
  port I/O via `_robot.portIO()`; gripper via `_robot.gripper()`.
- [x] Firmware builds via `mbdeploy deploy --build`.
- [ ] **Bench gate**: Full command-parity verification on the stand:
  - All motion commands: X, S (with SAFETY_STOP), T (with T+DONE), D (with D+DONE).
  - All query commands: ENC, EZ, SO, SZ, SI.
  - K dump and at least 3 K setters (e.g. `KmmPerDegL`, `KtrackwidthMm`, `KtickMs`).
  - OTOS init/position/velocity if sensor present: `O`, `OP`, `OR`.
  - Line sensor: `LS`. Color sensor: `CS`. Gripper: `GA+90`.
  - Port I/O: `P1+1` (digital write), `PA1` (analog read) if ports wired.
  - All commands work identically over both serial and radio.

## Implementation Plan

### Approach

1. Add all `Robot` public methods to `Robot.h`; implement them in `Robot.cpp` as
   thin delegates to subsystems (most are one-liners).
2. Rewrite each `CommandProcessor` command handler to call `_robot.*` instead of
   directly accessing injected pointers.
3. Delete `CommandProcessor::init()`, `setCalib/setConfig()`, `tick()`, and all
   hardware pointer members.
4. Update `Robot`'s constructor in `Robot.cpp` to no longer call `_cmd.init(...)` or
   `_cmd.setConfig(...)`.
5. Update `main.cpp` construction: `CommandProcessor cmd(robot)` takes only Robot ref.

### Files to Modify

| File | Change |
|---|---|
| `source/robot/Robot.h` | Add full public interface: action methods, query structs + methods, all accessors |
| `source/robot/Robot.cpp` | Implement action/query/accessor methods; remove `_cmd.init()`/`setConfig()` calls |
| `source/app/CommandProcessor.h` | Strip to `Robot& _robot` + `process()` + static helpers; delete `init()`, `setCalib`, `tick()`, all pointer fields |
| `source/app/CommandProcessor.cpp` | Rewrite all ~35 handlers to use `_robot.*`; delete `init()`, `setCalib`, `tick()` bodies |
| `source/main.cpp` | Update construction: `CommandProcessor cmd(robot)` with no `init()` call |

### Query Struct Definitions (in Robot.h)

```cpp
struct EncoderReading { int32_t leftMm; int32_t rightMm; };
struct Pose            { int32_t x_mm;  int32_t y_mm;  int32_t h_cdeg; };
```

These replace the inline formatting that was previously done inside CommandProcessor
handlers. Handlers now call `getEncoders()` / `getPose()` and format the result into
the same wire strings as before (e.g. `ENC+%d+%d`).

### Testing Plan

- Build and flash: `mbdeploy deploy --build`.
- Bench gate: full command-parity check as in Acceptance Criteria — exercise every
  command category over both serial and radio.
- Pay special attention to K* setters: confirm the value written to `robot.config()`
  is reflected in subsystem behavior (e.g. changed `mmPerDegL` affects encoder output).

### Documentation Updates

None needed this ticket.
