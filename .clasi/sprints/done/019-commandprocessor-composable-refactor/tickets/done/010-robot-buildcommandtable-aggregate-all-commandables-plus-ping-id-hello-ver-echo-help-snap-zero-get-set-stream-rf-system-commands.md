---
id: '010'
title: "Robot::buildCommandTable() \u2014 aggregate all Commandables plus PING, ID,\
  \ HELLO, VER, ECHO, HELP, SNAP, ZERO, GET, SET, STREAM, RF system commands"
status: done
use-cases:
- SUC-001
- SUC-003
- SUC-005
depends-on:
- 009
github-issue: ''
issue: ''
completes_issue: false
---

# Robot::buildCommandTable() — aggregate all Commandables plus PING, ID, HELLO, VER, ECHO, HELP, SNAP, ZERO, GET, SET, STREAM, RF system commands

## Description

Implement `Robot::buildCommandTable(CommandDescriptor* buf, int max) -> int` which:

1. Calls `getCommands()` on each `Commandable` member (`motionController`, `odometry`,
   `portController`, `servoController`) and appends their descriptors into `buf`.
2. Calls `getCommands()` on the caller-supplied `DebugCommandable*` parameter.
3. Adds system-level command descriptors directly: PING, ID, HELLO, VER, ECHO, HELP,
   SNAP, ZERO, STREAM, RF.
4. Adds GET and SET descriptors with `handlerFn = handleGet / handleSet` from
   `ConfigRegistry.h` and `handlerCtx = &_cfgCtx` (a static `CfgCtx` member).

After this ticket, `main.cpp` can call `buildCommandTable()`, verify the count, and the
full table is ready to pass to the new `CommandProcessor` constructor. The cutover
(switching `main.cpp` to the new constructor) is T011.

## Acceptance Criteria

- [x] `Robot.h` declares `int buildCommandTable(CommandDescriptor* buf, int max, DebugCommandable* dbg)` (or `dbg` passed another way consistent with main.cpp wiring)
- [x] `Robot.cpp` implements `buildCommandTable()` filling `buf` with descriptors from all Commandables plus system commands
- [x] System command handlers (PING, ID, HELLO, VER, ECHO, HELP, SNAP, ZERO, STREAM, RF) produce wire output identical to the corresponding old switch cases; `handlerCtx = this` (i.e., `Robot*`)
- [x] GET and SET descriptors use `handleGet`/`handleSet` from `ConfigRegistry.h`; `CfgCtx` is a static value member on `Robot` or in `main.cpp`
- [x] Total descriptor count does not exceed 60 (stays well within static buffer); assert or static_assert in `buildCommandTable()`
- [x] `python3 build.py` passes with no errors
- [x] `buildCommandTable()` is callable from `main.cpp` (wiring is in place) even though the new constructor is not yet activated

## Implementation Plan

### Approach

System commands (PING, ID, HELLO, VER, ECHO, HELP, SNAP, ZERO, STREAM, RF) need
access to `Robot*`. Define a `SysCtx` struct or simply use `Robot*` as `handlerCtx`
for these. Each handler body is a static function that casts `handlerCtx` to `Robot*`
and calls the appropriate method (`buildTlmFrame`, `systemTime`, etc.).

Static buffer size in `main.cpp`: `static CommandDescriptor cmdTable[60];` with the
count returned from `buildCommandTable()` passed to the new constructor in T011.

Read every system command switch case (PING, ID, HELLO, VER, ECHO, HELP, SNAP, ZERO,
STREAM, RF) in `CommandProcessor.cpp` before writing any handler. Exact wire format
fidelity is required.

### Files to Modify

- `source/robot/Robot.h` — declare `buildCommandTable()`; add `CfgCtx _cfgCtx` member or equivalent
- `source/robot/Robot.cpp` — implement `buildCommandTable()`; add system command handler static functions

### Testing Plan

- Build: `python3 build.py` must pass.
- Code review: verify handler count <= 60; verify PING/ID/HELLO handlers match old switch byte-for-byte.
- Bench (dry run): add a temporary `buildCommandTable()` call in `main.cpp` that counts
  entries and prints the count via `DBG`; verify expected count (~42) appears on serial.
