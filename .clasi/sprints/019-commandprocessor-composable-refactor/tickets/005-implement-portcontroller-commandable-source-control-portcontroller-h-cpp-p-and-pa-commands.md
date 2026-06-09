---
id: '005'
title: "Implement PortController Commandable \u2014 source/control/PortController.h/.cpp,\
  \ P and PA commands"
status: done
use-cases:
- SUC-001
depends-on:
- '004'
github-issue: ''
issue: ''
completes_issue: false
---

# Implement PortController Commandable — source/control/PortController.h/.cpp, P and PA commands

## Description

Create `PortController` — a thin `Commandable` wrapper around `PortIO&` that owns the
P (single-port write) and PA (all-ports write) commands. This decouples the port I/O
command handlers from `CommandProcessor.cpp`. The P and PA handlers are moved out of
the old switch into `PortController::getCommands()`. `Robot` gains a `portController`
value member. The old switch still handles P/PA during this ticket (command migration
completes in T010).

## Acceptance Criteria

- [x] `source/control/PortController.h` declares:
  - `class PortController : public Commandable` with constructor `PortController(PortIO& pio)`
  - `virtual int getCommands(CommandDescriptor* buf, int max) const override`
  - `PortCtx` is not needed separately — handler context is `PortController*` itself
- [x] `source/control/PortController.cpp` implements `getCommands()` returning two descriptors:
  - `"P"` — parse: port index (int) + value (int); handler calls `_pio.setDigital`/`readDigital`, replies `OK port p=<port> v=<val>`
  - `"PA"` — parse: port + optional val (0-1023); handler calls `_pio.setAnalog`/`readAnalog`, replies `OK aport p=<port> v=<val>`
  - Parse functions validate arg counts; on failure `errFmt = "badarg"`
- [x] `source/robot/Robot.h` declares `PortController portController` as a value member after `portio` ref
- [x] `source/robot/Robot.cpp` (or constructor list) wires `portController(portio)`
- [x] `python3 build.py` passes with no errors
- [x] P and PA commands continue to work via old switch path (no behavior change yet)

## Implementation Plan

### Approach

Read the existing P and PA handlers in `CommandProcessor.cpp` to understand the exact
argument layout and reply format before writing the new handlers. The new handlers must
produce identical wire output. Context is `PortController*` cast in handler body.

### Files to Create

- `source/control/PortController.h`
- `source/control/PortController.cpp`

### Files to Modify

- `source/robot/Robot.h` — add `PortController portController` member (after `portio` ref)
- `source/robot/Robot.cpp` — add `portController(portio)` to constructor initializer list

### Testing Plan

- Build: `python3 build.py` must pass.
- Bench smoke: P and PA commands work via `uv run rogo` (old switch path still active).
