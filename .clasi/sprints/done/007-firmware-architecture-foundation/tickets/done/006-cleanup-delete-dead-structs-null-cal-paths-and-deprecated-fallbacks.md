---
id: '006'
title: "Cleanup \u2014 delete dead structs, null-cal paths, and deprecated fallbacks"
status: done
use-cases:
- SUC-006
depends-on:
- '005'
github-issue: ''
issue: firmware-architecture-refactor.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 006 — Cleanup — delete dead structs, null-cal paths, and deprecated fallbacks

## Description

After the five prior tickets the architecture is in place, but scaffolding may remain:
- Any temporary bridge methods or compatibility shims added during migration.
- Any remaining `?? default` null-calibration guard paths in `MotorController`,
  `Odometry`, or `NezhaV2` (e.g. `if (_cal == nullptr) return defaultCalibParams()`).
- Any `CommandProcessor::Params` or `CalibParams` remnants if not fully cleaned in
  Ticket 001 (belt-and-suspenders pass).
- Unused includes, dead fields, or unreachable code paths in any changed file.
- Build warnings introduced during migration.

This ticket audits all changed files and removes every dead code path. The firmware
must build with zero new warnings after this ticket.

## Acceptance Criteria

- [x] `CalibParams` struct and `defaultCalibParams()` do not exist anywhere in
  the codebase (search: `grep -r CalibParams source/`).
- [x] `CommandProcessor::Params` struct does not exist (search: `grep -r "struct Params" source/app/`).
- [x] No null-calibration guard paths remain: no `_cal == nullptr` or `_config == nullptr`
  checks in `MotorController`, `Odometry`, or `NezhaV2` (search: `grep -r "_cal ==" source/`).
- [x] No `Robot::run()` declaration or definition exists.
- [x] No `CommandProcessor::init()`, `tick()`, or `setCalib/setConfig()` declarations
  or definitions exist.
- [x] No unused `#include` directives introduced during migration (spot-check changed files).
- [x] Firmware builds via `mbdeploy deploy --build` with zero warnings on files touched
  in this sprint.
- [ ] **Bench gate**: Deploy to robot. Final full smoke sequence:
  - HELLO → DEVICE:…
  - EZ / ENC round-trip
  - S+200+200 → streams; X stops; SAFETY_STOP fires after 200 ms silence
  - T+200+200+1500 → T+DONE on the originating channel (serial and radio both)
  - D+150+150+200 → D+DONE
  - SO, SZ odometry
  - KmmPerDegL+0.500 → confirmed in subsequent ENC output
  - LS, CS, gripper GA+90, GA+0
  - Confirm all commands work over both serial and radio (behavioral parity).

## Implementation Plan

### Approach

Systematic audit pass over all files modified in Tickets 001–005:

1. Run `grep -r "CalibParams\|defaultCalibParams\|struct Params\|_cal ==" source/` —
   resolve every hit.
2. Check `source/robot/Robot.h` and `.cpp` for any `run()` remnant.
3. Check `source/app/CommandProcessor.h` and `.cpp` for any `init()`, `tick()`,
   `setCalib`, `setConfig`, hardware pointer members.
4. Check `source/control/MotorController.cpp` for any null-cal guard.
5. Scan build output for warnings; fix each one.
6. Final build and bench run.

### Files to Audit

| File | What to check |
|---|---|
| `source/types/Config.h` | Only `RobotConfig`, `defaultRobotConfig()`, `MotorGains`, `DriveMode` remain |
| `source/robot/Robot.h/.cpp` | No `run()`, no `CalibParams`, no `MicroBit` member |
| `source/app/CommandProcessor.h/.cpp` | Only `Robot& _robot` + `process()` + static helpers |
| `source/control/MotorController.h/.cpp` | `const RobotConfig& _cal`; no null guard |
| `source/control/DriveController.h/.cpp` | No dead fields from CommandProcessor migration |
| `source/main.cpp` | Clean, no leftover shims |

### Testing Plan

- `mbdeploy deploy --build` — zero warnings on changed files.
- Bench smoke sequence as in Acceptance Criteria.
- grep checks as listed above.

### Documentation Updates

- Update `docs/architecture.md` to reflect the final Sprint 007 structure:
  - Update the layer diagram to show `DriveController` in the control layer.
  - Update `Config.h` description to reference `RobotConfig` and `defaultRobotConfig()`.
  - Update `CommandProcessor` description to "pure parse-and-dispatch, `Robot& _robot` only".
  - Update `Robot` description: no `MicroBit` member; public interface + accessors + `tick()`.
  - Update `main.cpp` entry in the dependency diagram.
  - Update dependency/ownership diagram to reflect the new structure.
