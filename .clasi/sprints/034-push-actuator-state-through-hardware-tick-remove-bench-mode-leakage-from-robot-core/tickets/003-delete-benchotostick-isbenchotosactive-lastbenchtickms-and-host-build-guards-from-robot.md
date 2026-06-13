---
id: '003'
title: Delete benchOtosTick, isBenchOtosActive, _lastBenchTickMs and HOST_BUILD guards from Robot
status: open
use-cases:
- SUC-034-001
depends-on:
- '002'
github-issue: ''
issue: hardware-tick-actuator-state.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete benchOtosTick, isBenchOtosActive, _lastBenchTickMs and HOST_BUILD guards from Robot

## Description

Remove the bench-mode implementation that leaked into the Robot core. After ticket 002, `loopTickOnce` no longer calls `robot.benchOtosTick(now)`, so the method is unreachable. This ticket deletes it and all its supporting infrastructure from `Robot.h` / `Robot.cpp`, leaving the robot core free of any bench/sim/NezhaHAL knowledge.

Items to delete:
1. `Robot::benchOtosTick(uint32_t now_ms)` — declaration in `Robot.h`, definition in `Robot.cpp:416-462`.
2. `Robot::_lastBenchTickMs` — private member in `Robot.h:218-220`.
3. `Robot::_simBenchOtosActive` — private member in `Robot.h:175`; used only by `isBenchOtosActive` in HOST_BUILD.
4. The `#ifndef HOST_BUILD` / `#include "NezhaHAL.h"` block in `Robot.cpp:432-434`.
5. `Robot::setBenchOtosEnabled(bool on)` — `Robot.h:165-166`, `Robot.cpp:469-478`. This method is called only from `handleDbgOtosBench` in `DebugCommandable.cpp`, which is firmware-only. The DebugCommandable handler will be updated in ticket 004 to call `NezhaHAL` directly instead of going through Robot. Delete this method.
6. `Robot::isBenchOtosActive() const` — `Robot.h:167-169`, `Robot.cpp:480-493`. Same: only called from `handleDbgOtosBench`. Delete; the handler will read from NezhaHAL directly.

After deletion, `Robot.cpp` must have zero `#ifndef HOST_BUILD` preprocessor guards for this feature, no `#include "NezhaHAL.h"`, and no mention of `NezhaHAL`, `BenchOtosSensor`, `benchOtosTick`, or `isBenchOtosActive`.

Note: if `setBenchOtosEnabled` or `isBenchOtosActive` are called from any other site, do not delete them yet — flag the exception. A quick grep will confirm they are only called from DebugCommandable.

## Files to Modify

- `source/robot/Robot.h` — remove declarations of `benchOtosTick`, `setBenchOtosEnabled`, `isBenchOtosActive`, `_lastBenchTickMs`, `_simBenchOtosActive`.
- `source/robot/Robot.cpp` — remove definitions of `benchOtosTick`, `setBenchOtosEnabled`, `isBenchOtosActive`; remove `#ifndef HOST_BUILD` / `#include "NezhaHAL.h"` blocks for this feature; remove comment block at lines ~415-430.

## Acceptance Criteria

- [ ] `grep -r "benchOtosTick\|isBenchOtosActive" source/` returns no hits.
- [ ] `grep -n "NezhaHAL\|BenchOtosSensor" source/robot/Robot.cpp` returns no hits.
- [ ] `grep -n "ifndef HOST_BUILD" source/robot/Robot.cpp` returns no hits (or only hits for unrelated features — confirm by inspection).
- [ ] `grep -rn "NezhaHAL\|BenchOtosSensor" source/control/` returns no hits.
- [ ] `python3 build.py` exits clean.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` exits green.

## Implementation Plan

1. Grep `source/` for all call sites of `setBenchOtosEnabled` and `isBenchOtosActive`. Confirm they are only in `DebugCommandable.cpp` (and `Robot.cpp` itself). If any other file calls them, flag the issue — do not delete those methods until ticket 004 removes the DebugCommandable calls.
2. Remove `benchOtosTick` declaration and definition.
3. Remove `setBenchOtosEnabled` and `isBenchOtosActive` declarations and definitions.
4. Remove `_lastBenchTickMs` and `_simBenchOtosActive` private members.
5. Remove the `#ifndef HOST_BUILD` / `#include "NezhaHAL.h"` block at the top of the affected region in `Robot.cpp`.
6. Build; fix any compile errors (typically: DebugCommandable.cpp will break when it tries to call the now-deleted methods — that is expected and will be fixed in ticket 004).
7. Run full sim suite to confirm green.

## Testing

- **Build gate**: `python3 build.py` clean.
- **Sim gate**: `uv run --with pytest python -m pytest host_tests/ host/tests/` green.
- The acceptance grep checks are the primary correctness gate.
