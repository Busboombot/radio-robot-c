---
id: 008
title: 'Shrink main.cpp: switch to LoopScheduler; add lag CFG registry entries'
status: done
use-cases:
- SUC-001
- SUC-005
- SUC-006
depends-on:
- '006'
- '007'
github-issue: ''
issue: plan-single-cooperative-main-loop-abandon-fibers.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Shrink main.cpp: switch to LoopScheduler; add lag CFG registry entries

## Description

This is the cutover ticket. Replace the two-fiber `main()` body with a call to
`LoopScheduler::run()`, remove all two-fiber infrastructure, and wire up the
lag configuration registry entries.

After this ticket the firmware is running the single cooperative loop for the
first time end-to-end. All previous tickets kept the old `main.cpp` loop
running to preserve build continuity; this ticket completes the transition.

## Files to Modify

- `source/main.cpp` — remove `controlFiberFn`, `gRobot`, `create_fiber`, the
  `kCommsPeriodMs` constant, the `while(true)` loop body; add `LoopScheduler`
  construction + `sched.run()`. Emit HELLO before `sched.run()`. The
  `serialReply`/`radioReply` adapters moved to `LoopScheduler.cpp` in ticket
  006.
- `source/app/CommandProcessor.cpp` — add four `CFG_I` entries to `kRegistry[]`:
  `lag.otos` → `lagOtosMs`, `lag.line` → `lagLineMs`,
  `lag.color` → `lagColorMs`, `lag.ports` → `lagPortsMs`.

## Acceptance Criteria

- [x] `main.cpp` no longer contains `controlFiberFn`, `gRobot`, `create_fiber`,
  or the comms `while(true)` loop.
- [x] `main()` body is: `uBit.init()`, display boot icon, construct `Robot`,
  construct `CommandProcessor`, construct `LoopScheduler`, emit HELLO,
  `sched.run()`.
- [x] `CommandProcessor`'s `kRegistry[]` contains entries for `lag.otos`,
  `lag.line`, `lag.color`, `lag.ports` mapping to the corresponding
  `RobotConfig` fields.
- [x] `SET lag.otos 200` writes `200` to `cfg.lagOtosMs`; `GET lag.otos`
  returns `200`.
- [x] Default lag values confirmed via `GET lag.*` immediately after boot:
  otos=100, line=50, color=100, ports=50.
- [x] Firmware builds cleanly with no references to `controlTick` or
  `telemetryTick` remaining in `main.cpp`.
- [x] `uv run --with pytest python -m pytest` passes — full suite.

## Implementation Plan

1. In `main.cpp`:
   - Remove `gRobot`, `controlFiberFn`, `serialReply`, `radioReply` (moved to
     `LoopScheduler.cpp` in ticket 006).
   - Remove `create_fiber(controlFiberFn, ...)`.
   - Remove `kCommsPeriodMs`, the serial/radio drain loop, the
     `robot.telemetryTick(...)` call, and `uBit.sleep(kCommsPeriodMs)`.
   - Add `#include "LoopScheduler.h"`.
   - Add `static LoopScheduler sched(robot, cmd, uBit);` after `cmd`.
   - Replace the old loop with `sched.run()`.
2. In `CommandProcessor.cpp`, add to `kRegistry[]` (after the existing entries):
   ```cpp
   CFG_I("lag.otos",  lagOtosMs),
   CFG_I("lag.line",  lagLineMs),
   CFG_I("lag.color", lagColorMs),
   CFG_I("lag.ports", lagPortsMs),
   ```
3. Verify no remaining references to `controlTick` or `telemetryTick` in any
   source file.

## Testing Plan

- **Build verification**: `python build.py` — no new errors or warnings about
  removed symbols.
- **Automated tests**: `uv run --with pytest python -m pytest` — full suite.
  This is the first run of the complete single-loop firmware against all tests.
- **Smoke test** (before bench gate): Flash to robot, send HELLO, confirm DEVICE
  response, send `GET lag.otos`, confirm `100`. Then ticket 009 performs the
  full bench gate.
