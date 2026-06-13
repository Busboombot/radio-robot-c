---
id: '001'
title: Add Hardware::tick(now, cmds) overload; implement in NezhaHAL
status: open
use-cases:
- SUC-034-002
depends-on: []
github-issue: ''
issue: hardware-tick-actuator-state.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add Hardware::tick(now, cmds) overload; implement in NezhaHAL

## Description

Add the debug-build `Hardware::tick(uint32_t now_ms, const MotorCommands& cmds)` pure-virtual overload to the `Hardware` interface, and implement it in `NezhaHAL`. This is the new input channel through which the synthetic bench plant receives commanded actuator state without requiring a downcast in Robot.

`NezhaHAL::tick(now, cmds)` integrates the bench sensor when bench mode is active: it calls `_benchOtos.tick(cmds.tgtLMms, cmds.tgtRMms, config.trackwidthMm, dt_ms)`. When bench mode is off it is a no-op. The dt logic (signed delta, first-call zero) that currently lives in `Robot::benchOtosTick` moves here. `NezhaHAL` already holds `_benchOtos` and `_otosActive` — no new members needed.

`Hardware.h` guards the new overload with `#ifdef BENCH_BUILD` (or the project's existing debug/bench define — determine the actual define name from the project's `CMakeLists.txt` and match it). The overload is NOT present in the production binary.

This ticket establishes the interface contract only. The call site in `loopTickOnce` (ticket 002) and the MockHAL implementation (ticket 005) come later and depend on this.

## Files to Modify

- `source/hal/Hardware.h` — add `#ifdef BENCH_BUILD` pure-virtual `tick(now_ms, cmds)` overload. Include `RobotState.h` (or forward-declare `MotorCommands`) as needed.
- `source/hal/NezhaHAL.h` — declare `tick(uint32_t now_ms, const MotorCommands& cmds)` override.
- `source/hal/NezhaHAL.cpp` — implement `NezhaHAL::tick(now, cmds)`. Port the dt-signed-delta logic from `Robot::benchOtosTick` (currently `Robot.cpp:438-461`). The `config.trackwidthMm` needed for the bench tick: `NezhaHAL` already receives `RobotConfig&` at construction — confirm it stores or can access trackwidth, and if not, add a `float _trackwidthMm` member initialized from the cfg.

## Acceptance Criteria

- [ ] `Hardware::tick(uint32_t, const MotorCommands&)` is declared as pure-virtual, guarded by `#ifdef BENCH_BUILD` (use the project's actual debug/bench define name).
- [ ] `NezhaHAL` overrides it; when bench mode is active, calls `_benchOtos.tick(cmds.tgtLMms, cmds.tgtRMms, trackwidthMm, dt_ms)` using signed-delta dt.
- [ ] When bench mode is off, `NezhaHAL::tick(now, cmds)` returns immediately (no-op).
- [ ] First call with `_lastBenchTickMs == 0` passes `dt_ms = 0` to `_benchOtos.tick` (consistent with prior behavior in `benchOtosTick`).
- [ ] `python3 build.py` exits clean (firmware). Normal ~98.33% RAM is expected.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` exits green (sim suite 719+ passing). Note: MockHAL does not yet implement the overload; the compiler must not require it in HOST_BUILD (confirm the guard excludes it from the mock's pure-virtual set).

## Implementation Plan

1. Check `CMakeLists.txt` for the actual bench/debug build define name (search for `BENCH`, `DEBUG_BUILD`, `BENCH_BUILD`). Use the project's existing define — do not invent a new one.
2. Add `RobotState.h` include (or forward declaration) to `Hardware.h` — needed for `MotorCommands`. Check whether `Hardware.h` already transitively includes it.
3. Add the guarded pure-virtual to `Hardware.h`.
4. Declare the override in `NezhaHAL.h`.
5. In `NezhaHAL.cpp`: implement using the signed-delta dt pattern from `Robot::benchOtosTick`. Add a `uint32_t _lastBenchTickMs = 0` private member to `NezhaHAL.h` (moving it from `Robot`'s private section).
6. Confirm `NezhaHAL` can access `trackwidthMm` — check its constructor signature. Add a stored copy if needed.
7. Build and run tests.

## Testing

- **Build gate**: `python3 build.py` clean.
- **Sim gate**: `uv run --with pytest python -m pytest host_tests/ host/tests/` green.
- No new host test is required for this ticket (the interface addition is guarded out of HOST_BUILD; the NezhaHAL behavior is verified end-to-end in later tickets via the bench-hardware test in ticket 004).
