---
id: 019
title: CommandProcessor Composable Refactor
status: done
branch: sprint/019-commandprocessor-composable-refactor
use-cases: []
issues:
- commandprocessor-composable-refactor
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 019: CommandProcessor Composable Refactor

## Goals

Replace the 1742-line monolithic `CommandProcessor::process()` switch statement with a registration-based dispatch table. Each subsystem declares its own commands via a `Commandable` interface; `CommandProcessor` becomes a thin table-driven dispatcher.

## Problem

All command logic lives in a single 1742-line switch statement in `CommandProcessor.cpp`. Adding commands for new subsystems requires editing the central dispatcher. The system cannot be modularly extended — every new command is a surgery on the same file.

## Solution

- Introduce `CommandTypes.h` with C++11-compatible tagged-union `Argument`, `ArgList`, `ParseResult`, `CommandDescriptor`, and a `Commandable` abstract interface.
- Rename `DriveController` → `MotionController`.
- Create `PortController` (wraps `PortIO`) and `ServoController` (wraps `Servo`), both `Commandable`.
- Make `MotionController`, `Odometry`, `PortController`, `ServoController`, and a new `DebugCommandable` all inherit `Commandable` and implement `getCommands()`.
- Add `Robot::buildCommandTable()` that aggregates descriptors from all Commandables and adds system-level commands.
- Migrate `CommandProcessor` to accept a `CommandDescriptor[]` table; replace the switch with a longest-prefix linear-scan dispatcher.
- Extract the GET/SET config registry to `ConfigRegistry.h/.cpp`.
- Use staged migration — keep the old switch path live until all commands are migrated.

## Success Criteria

- Firmware builds cleanly with `python3 build.py`.
- All existing commands (35+) work identically over the wire.
- Each subsystem's commands are defined in that subsystem's `.cpp` file, not in `CommandProcessor.cpp`.
- `CommandProcessor.cpp` switch statement is deleted; file is < 200 lines.

## Scope

### In Scope

- `source/types/CommandTypes.h` — new type definitions
- `DriveController` → `MotionController` rename (header, source, all includes)
- `source/control/PortController.h/.cpp` — P, PA commands
- `source/control/ServoController.h/.cpp` — GRIP command
- `MotionController::getCommands()` — S, T, D, G, R, TURN, VW, X, STOP
- `Odometry::getCommands()` — OI, OZ, OR, OP, OV, OL, OA
- `PortController::getCommands()` — P, PA
- `ServoController::getCommands()` — GRIP
- `source/app/DebugCommandable.h/.cpp` — DBG LOOP/I2C/IRQGUARD/WEDGE, I2CW, I2CR
- `source/robot/ConfigRegistry.h/.cpp` — extracted kRegistry[] + GET/SET handlers
- `Robot::buildCommandTable()` — PING, ID, HELLO, VER, ECHO, HELP, SNAP, ZERO, GET, SET, STREAM, RF
- `CommandProcessor` new constructor + table dispatch; staged cutover

### Out of Scope

- Protocol changes (wire format is unchanged)
- New commands
- Python host changes

## Test Strategy

Build verification after each migration step (`python3 build.py`). After cutover, connect robot and verify representative commands from each group via `uv run rogo`: motion (S/D/T), config (GET/SET round-trip), OTOS (OZ/OL), debug (DBG LOOP to serial), I/O (P/PA/GRIP), error paths (unknown verb, bad arg count).

## Architecture Notes

- C++11 only (`-std=c++11 -fno-exceptions -fno-rtti` from `target-locked.json`)
- `HandlerFn` = plain function pointer + `void* handlerCtx` (same pattern as existing `ReplyFn`)
- No heap allocation; `CommandDescriptor` table is a static array (~42 × 24 bytes = 1008 bytes BSS)
- Prefix matching: longest-prefix linear scan; multi-word prefixes ("DBG LOOP RESET") beat shorter ones

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Add CommandTypes.h — Argument, ArgList, ParseResult, CommandDescriptor, Commandable, makeCmd | — |
| 002 | Extract ConfigRegistry — kRegistry[], handleGet, handleSet to source/robot/ConfigRegistry.h/.cpp | 001 |
| 003 | Add CommandProcessor new constructor and dispatchTable() path (dual-mode) | 001, 002 |
| 004 | Rename DriveController to MotionController — class name, file names, all include sites | 003 |
| 005 | Implement PortController Commandable — source/control/PortController.h/.cpp, P and PA commands | 004 |
| 006 | Implement ServoController Commandable — source/control/ServoController.h/.cpp, GRIP command | 005 |
| 007 | Implement DebugCommandable — source/app/DebugCommandable.h/.cpp, all DBG subcommands + I2CW/I2CR with ForceReply::SERIAL | 006 |
| 008 | MotionController::getCommands() — S, T, D, G, R, TURN, VW, X, STOP with EVT async completion verification | 007 |
| 009 | Odometry::getCommands() — OI, OZ, OR, OP, OV, OL, OA | 008 |
| 010 | Robot::buildCommandTable() — aggregate all Commandables plus system commands | 009 |
| 011 | Cutover — main.cpp uses CommandProcessor(cmds, count), delete old Robot& constructor and switch statement | 010 |

Tickets execute serially in the order listed.
