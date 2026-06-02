---
id: '003'
title: "Extract DriveController \u2014 move S/T/D/G state machines out of CommandProcessor"
status: done
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 003 — Extract DriveController — move S/T/D/G state machines out of CommandProcessor

## Description

The S/T/D/G drive state machines, S-mode watchdog, streaming encoder counter, and
odometry-delta state are all owned by `CommandProcessor`. This is wrong: the parser
should not own drive state. This ticket creates `DriveController` in the control layer,
moves all drive state into it, adds `Robot` drive methods that delegate to it, and
keeps `CommandProcessor::tick()` calling `_robot.tick()` (or the DriveController
directly as a temporary bridge until Ticket 005 removes the tick entirely from
CommandProcessor).

After this ticket:
- `DriveController` owns `_mode`, `_lastSMs`, `_tEndMs`, `_dEnc*`, `_gPhase/Arc*`,
  `_encTickCount`, `_prevOdoEncL/R`.
- `CommandProcessor` still calls into drive state for S/T/D/G/X dispatch (by calling
  Robot's new drive methods), but no longer stores any drive fields itself.
- `CommandProcessor::tick()` still exists but now delegates to `DriveController::tick()`
  through `Robot`. Full removal of `CommandProcessor::tick()` happens in Ticket 004/005.

## Acceptance Criteria

- [x] `source/control/DriveController.h` and `source/control/DriveController.cpp` exist.
- [x] `DriveController` holds all drive state: `DriveMode _mode`, `_lastSMs`,
  `_tEndMs`, `_dEncStartL/R`, `_dTargetMm`, `GPhase _gPhase`, `_gTarget*`,
  `_gArc*`, `_encTickCount`, `_prevOdoEncL/R`.
- [x] `DriveController` public interface: `beginStream`, `beginTimed`,
  `beginDistance`, `beginGoTo`, `stop`, `tick(now_ms, dt_ms, fn, ctx)`, `mode()`.
- [x] `computeArc()` pure geometry function lives in `DriveController` as a private static.
- [x] `Robot` owns a `DriveController` member and exposes drive methods:
  `stop()`, `streamDrive()`, `timedDrive()`, `distanceDrive()`, `goTo()`.
- [x] `CommandProcessor` drive command handlers call `_robot.stop()`, `_robot.streamDrive()`,
  etc. — no drive fields remain in `CommandProcessor`.
- [x] `CommandProcessor` has no `_mode`, `_lastSMs`, `_tEndMs`, `_dEnc*`, `_gPhase`,
  `_gArc*`, `_encTickCount`, `_prevOdoEncL/R` members.
- [x] Firmware builds and all drive commands work identically:
  - `S+200+200` → streams; watchdog fires `SAFETY_STOP` after 200 ms without re-send.
  - `T+200+200+2000` → drives 2 s, emits `T+DONE`.
  - `D+200+200+300` → drives 300 mm, emits `D+DONE`.
  - `X` → stops immediately, emits `ACK:X`.
- [ ] **Bench gate**: Deploy to robot. Exercise all four drive modes: S (with watchdog),
  T (with `T+DONE`), D (with `D+DONE`), G (if feasible on stand). Confirm
  completions appear and wheels behave as expected.

## Implementation Plan

### Approach

1. Create `source/control/DriveController.h` with the interface described above.
2. Create `source/control/DriveController.cpp` by transplanting drive state and
   tick body from `CommandProcessor.cpp`. Adjust internal calls to use the injected
   `MotorController&` and `Odometry&` references.
3. Add `DriveController _dc` member to `Robot.h`; construct it with `_mc`, `_odo`,
   `_config` refs.
4. Add `Robot` drive methods (`stop`, `streamDrive`, `timedDrive`, `distanceDrive`,
   `goTo`) that delegate to `_dc`.
5. In `CommandProcessor`, replace direct field manipulation with calls to `_robot.*`
   drive methods. Remove the drive state fields.
6. `CommandProcessor::tick()` calls `_robot._dc.tick()` or (cleaner) a new
   `Robot::tickDrive(now_ms, dt_ms, fn, ctx)` helper — either works for this ticket.

### Files to Create

- `source/control/DriveController.h`
- `source/control/DriveController.cpp`

### Files to Modify

| File | Change |
|---|---|
| `source/robot/Robot.h` | Add `DriveController _dc`; add drive method declarations |
| `source/robot/Robot.cpp` | Construct `_dc(_mc, _odo, _config)`; implement drive methods as delegates |
| `source/app/CommandProcessor.h` | Remove all drive state fields |
| `source/app/CommandProcessor.cpp` | Replace direct field manipulation with `_robot.*` drive method calls; `tick()` delegates to DriveController |
| `codal.json` (or CMakeLists) | Add `DriveController.cpp` to the build |

### State Inventory to Move from CommandProcessor to DriveController

```
DriveMode _mode
uint32_t  _lastSMs
float     _tgtL, _tgtR
uint32_t  _tEndMs
int32_t   _dEncStartL, _dEncStartR, _dTargetMm
uint32_t  _dTimeoutMs
GPhase    _gPhase
float     _gTargetX/Y, _gSpeed
float     _gArcLeftMm, _gArcRightMm, _gArcStartL, _gArcStartR
int32_t   _encTickCount
uint32_t  _lastTickMs
uint32_t  _currentTimeMs
int32_t   _prevOdoEncL, _prevOdoEncR
```

`computeArc()` static helper moves to `DriveController` as private static.

### Testing Plan

- Build and flash: `mbdeploy deploy --build`.
- Bench gate: exercise all four drive modes as in Acceptance Criteria.
- Confirm S-mode watchdog fires `SAFETY_STOP` after 200 ms without a keepalive.
- Confirm `T+DONE` / `D+DONE` complete correctly; `X` stops mid-drive.

### Documentation Updates

None needed this ticket.
