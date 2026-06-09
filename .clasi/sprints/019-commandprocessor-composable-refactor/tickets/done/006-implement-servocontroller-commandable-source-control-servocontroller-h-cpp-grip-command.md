---
id: '006'
title: "Implement ServoController Commandable \u2014 source/control/ServoController.h/.cpp,\
  \ GRIP command"
status: done
use-cases:
- SUC-001
depends-on:
- '005'
github-issue: ''
issue: ''
completes_issue: false
---

# Implement ServoController Commandable — source/control/ServoController.h/.cpp, GRIP command

## Description

Create `ServoController` — a thin `Commandable` wrapper around `Servo&` that owns the
GRIP command. This decouples the gripper command handler from `CommandProcessor.cpp`.
`Robot` gains a `servoController` value member. The old switch still handles GRIP
during this ticket (command migration completes in T010).

## Acceptance Criteria

- [x] `source/control/ServoController.h` declares:
  - `class ServoController : public Commandable` with constructor `ServoController(Servo& srv)`
  - `virtual int getCommands(CommandDescriptor* buf, int max) const override`
- [x] `source/control/ServoController.cpp` implements `getCommands()` returning one descriptor:
  - `"GRIP"` — parse: one int (position 0–180); handler calls `_srv.setAngle(pos)`, replies `OK GRIP <pos>`
  - Parse function validates arg count and range; on failure `errFmt = "badarg"`
- [x] `source/robot/Robot.h` declares `ServoController servoController` as a value member after `gripper` ref
- [x] `source/robot/Robot.cpp` (or constructor list) wires `servoController(gripper)`
- [x] `python3 build.py` passes with no errors
- [x] GRIP command continues to work via old switch path (no behavior change yet)

## Implementation Plan

### Approach

Read the existing GRIP handler in `CommandProcessor.cpp` to understand the argument layout
and reply format. The new handler must produce identical wire output. Context is
`ServoController*` cast in the handler body.

### Files to Create

- `source/control/ServoController.h`
- `source/control/ServoController.cpp`

### Files to Modify

- `source/robot/Robot.h` — add `ServoController servoController` member (after `gripper` ref)
- `source/robot/Robot.cpp` — add `servoController(gripper)` to constructor initializer list

### Testing Plan

- Build: `python3 build.py` must pass.
- Bench smoke: GRIP command works via `uv run rogo` (old switch path still active).
