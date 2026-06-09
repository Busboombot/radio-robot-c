---
id: '004'
title: "Rename DriveController to MotionController \u2014 class name, file names,\
  \ all include sites"
status: done
use-cases:
- SUC-001
depends-on:
- '003'
github-issue: ''
issue: ''
completes_issue: false
---

# Rename DriveController to MotionController — class name, file names, all include sites

## Description

Rename the `DriveController` class to `MotionController` across the entire codebase:
header filename, source filename, class name inside both files, and all `#include`
and reference sites. The `Robot.h` member `driveController` is renamed `motionController`.
No behavior changes. The class inherits `Commandable` at the declaration level (the
`getCommands()` implementation comes in T008).

## Acceptance Criteria

- [x] `source/control/DriveController.h` renamed to `source/control/MotionController.h`; class name inside changed to `MotionController`
- [x] `source/control/DriveController.cpp` renamed to `source/control/MotionController.cpp`; all `DriveController::` definitions updated to `MotionController::`
- [x] `source/control/MotionController.h` declares `class MotionController : public Commandable` (includes `CommandTypes.h`); adds `virtual int getCommands(CommandDescriptor* buf, int max) const override` declaration
- [x] `source/robot/Robot.h`: `#include "DriveController.h"` updated to `#include "MotionController.h"`; `DriveController driveController` member renamed to `MotionController motionController`
- [x] `source/app/CommandProcessor.cpp`: `#include "DriveController.h"` updated to `#include "MotionController.h"`; all `_robot.driveController` references updated to `_robot.motionController`
- [x] `source/main.cpp`: any `DriveController` or `driveController` references updated
- [x] No other files contain `DriveController` after this ticket (grep to verify)
- [x] `python3 build.py` passes with no errors
- [ ] Bench smoke: S command and D command (drive + EVT done D) still work correctly

## Implementation Plan

### Approach

1. Rename the files using `git mv source/control/DriveController.h source/control/MotionController.h`
   and `git mv source/control/DriveController.cpp source/control/MotionController.cpp`.
2. Inside the renamed header: change `class DriveController` to `class MotionController : public Commandable`;
   add `#include "CommandTypes.h"`; add `virtual int getCommands(CommandDescriptor* buf, int max) const override;`
   declaration. Add `struct MotionCtx { MotionController* mc; Robot* robot; };`.
3. Inside the renamed source: find-replace `DriveController::` with `MotionController::`.
4. Update all include sites (`Robot.h`, `CommandProcessor.cpp`, `main.cpp`, any test files).
5. Update all usage sites (`_robot.driveController` → `_robot.motionController`).
6. Run `grep -r "DriveController" source/` to confirm no stragglers.

### Files to Rename

- `source/control/DriveController.h` → `source/control/MotionController.h`
- `source/control/DriveController.cpp` → `source/control/MotionController.cpp`

### Files to Modify

- `source/control/MotionController.h` — class rename, Commandable inheritance, getCommands() declaration
- `source/control/MotionController.cpp` — class rename in definitions
- `source/robot/Robot.h` — include and member rename
- `source/app/CommandProcessor.cpp` — include and reference rename
- `source/main.cpp` — any DriveController references

### Testing Plan

- Build: `python3 build.py` must pass.
- Grep: `grep -r "DriveController" source/` must return nothing.
- Bench: S command (streaming drive) and D command (distance drive + `EVT done D`) work correctly via `uv run rogo`.
