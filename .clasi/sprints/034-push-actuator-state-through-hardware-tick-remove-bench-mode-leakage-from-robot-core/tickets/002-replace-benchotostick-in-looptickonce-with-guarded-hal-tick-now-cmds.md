---
id: '002'
title: Replace benchOtosTick in loopTickOnce with guarded hal.tick(now, cmds)
status: open
use-cases:
- SUC-034-001
- SUC-034-002
depends-on:
- '001'
github-issue: ''
issue: hardware-tick-actuator-state.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Replace benchOtosTick in loopTickOnce with guarded hal.tick(now, cmds)

## Description

Replace the `robot.benchOtosTick(now)` call in `loopTickOnce` (currently `LoopTickOnce.cpp:100`) with the `Hardware::tick(now, cmds)` overload added in ticket 001. This is variation point 2 — the single guarded switch in the loop body that distinguishes firmware bench-debug from production.

The guard is `#if defined(BENCH_BUILD) && !defined(HOST_BUILD)`. The `!defined(HOST_BUILD)` part is critical: in the sim, `hal.tick` is called in `sim_api.cpp` before `controlCollectSplitPhase` to maintain the correct ordering (plant advances before encoder reads). If the loopTickOnce call also ran in HOST_BUILD, MockHAL would double-tick per sim iteration, breaking the encoder model.

In HOST_BUILD, the guard suppresses the call entirely — `benchOtosTick` is already gone after ticket 003, and sim_api handles the hal.tick call at its existing position.

The call is placed at exactly the same source location as the removed `benchOtosTick` call (between odometry.predict and the OTOS block) to preserve the integrate-before-fuse ordering invariant.

## Files to Modify

- `source/control/LoopTickOnce.cpp` — replace `robot.benchOtosTick(now)` and its comment block with:
  ```cpp
  // ===== BENCH HAL TICK: advance bench sensor with commanded velocity =========
  // Firmware bench-debug only. In sim, hal.tick(now, cmds) is called in
  // sim_api.cpp before controlCollectSplitPhase to preserve plant-to-encoder ordering.
  // In production, this block is absent entirely.
  #if defined(BENCH_BUILD) && !defined(HOST_BUILD)
      robot.hal.tick(now, robot.state.commands);
  #endif
  ```
- `source/control/LoopTickOnce.cpp` — remove any `#include` for `NezhaHAL.h` or `BenchOtosSensor.h` if they were added for the `benchOtosTick` call (check the current includes).

## Acceptance Criteria

- [ ] `LoopTickOnce.cpp` no longer calls `robot.benchOtosTick(now)`.
- [ ] The replacement guard is `#if defined(BENCH_BUILD) && !defined(HOST_BUILD)` (use the project's actual define names — verify against `CMakeLists.txt`).
- [ ] The call site is in the same position (between odometry.predict and the OTOS block).
- [ ] No `NezhaHAL.h` or `BenchOtosSensor.h` include in `LoopTickOnce.cpp`.
- [ ] `python3 build.py` exits clean.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` exits green (719+ passing; no double-tick regression).

## Implementation Plan

1. Locate `robot.benchOtosTick(now)` at `LoopTickOnce.cpp:100` (approximately). Remove it and its comment block.
2. Insert the guarded `hal.tick` call at the same position.
3. Scan `LoopTickOnce.cpp` includes; remove any that were only needed for `benchOtosTick`.
4. Build and run tests.

## Testing

- **Build gate**: `python3 build.py` clean.
- **Sim gate**: `uv run --with pytest python -m pytest host_tests/ host/tests/` green.
- The sim tests exercise `loopTickOnce` on every tick; if the double-tick guard is wrong, the physics will diverge and EKF/odometry tests will fail. Green sim suite is the primary regression check.
