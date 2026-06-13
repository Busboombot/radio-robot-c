---
id: '034'
title: Push actuator state through Hardware::tick; remove bench-mode leakage from
  robot core
status: planning-docs
branch: sprint/034-push-actuator-state-through-hardware-tick-remove-bench-mode-leakage-from-robot-core
use-cases: []
issues:
- hardware-tick-actuator-state.md
- bench-otos-synthetic-otos-sensor-for-full-stack-bench-testing.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 034: Push actuator state through Hardware::tick; remove bench-mode leakage from robot core

## Goals

1. Remove Bench-OTOS implementation leakage from the firmware core: `Robot`, `Odometry`, and `MotionController` must compile identically in production vs bench/sim with zero mention of bench/sim/NezhaHAL/BenchOtosSensor. All variation lives in exactly two build-define seams.
2. Route the commanded actuator state to `Hardware::tick` so the synthetic plant (bench OTOS, MockHAL) receives it through the same interface as the real tick, not through side-channel downcasts or direct harness calls.
3. Fix the `DBG OTOS` hardware readout (F1: float printf emits nothing on newlib-nano; reformat as scaled integers) to complete the Bench-OTOS tooling end-to-end.

## Problem

The Bench-OTOS integration (sprint 031) left several implementation concerns in the firmware core:

- `Robot::benchOtosTick()` / `Robot::isBenchOtosActive()` downcast `hal` to `NezhaHAL*` with `#ifndef HOST_BUILD` guards inside `Robot.cpp`.
- `LoopTickOnce.cpp` calls `robot.benchOtosTick(now)` — a bench-only method on an otherwise clean orchestration struct.
- `DebugCommandable.cpp` downcasts `hal` to `NezhaHAL*` in `handleDbgOtosBench` and `handleDbgOtos` to reach `BenchOtosSensor`.
- The sim harness (`sim_api.cpp`) calls `hal.tick(now)` directly before `loopTickOnce` AND pokes `SimHandle::benchOtos` directly — workarounds for the missing command-state input.
- `DBG OTOS` reply uses `%f` which CODAL/newlib-nano does not support; all floats print empty on hardware (confirmed bench-033-findings F1).

## Solution

Add a debug-build `Hardware::tick(uint32_t now_ms, const MotorCommands& cmds)` overload. `NezhaHAL` implements it to drive `BenchOtosSensor::tick`; `MockHAL` implements it as its plant-integration step. Replace `robot.benchOtosTick(now)` in `loopTickOnce` with this `hal.tick` call. Delete `benchOtosTick`, `isBenchOtosActive`, `_lastBenchTickMs`, all `NezhaHAL` downcasts, and `#ifndef HOST_BUILD` guards from `Robot.cpp`. Rework DBG handlers to reach the bench sensor without downcasting Robot's `hal` (DebugCommandable is firmware-side and may know the concrete type). Remove the sim harness's direct `hal.tick` / bench-poke calls — they are superseded by the loop call. Fix `DBG OTOS` integer formatting (F1). Exclude `BenchOtosSensor` and DBG OTOS commands from the production build.

## Success Criteria

- `grep -r "benchOtosTick\|isBenchOtosActive" source/` produces no hits.
- No `static_cast<NezhaHAL*>` outside `source/hal/` and `source/app/DebugCommandable.cpp`.
- `#ifndef HOST_BUILD` count in `Robot.cpp` reduced to zero for this feature.
- Build-define variation in exactly two places: hardware creation and the `loopTickOnce` `hal.tick` call.
- `python3 build.py` (firmware) and `uv run --with pytest python -m pytest host_tests/ host/tests/` both green (sim suite was 719 passing on master; no regression).
- `DBG OTOS` on hardware emits non-empty integers for ideal, otos, fused, and err fields.
- Bench OTOS behavior on hardware unchanged: `DBG OTOS BENCH 1`, drive, synthetic pose advances in SNAP.
- Production build links with no BenchOtosSensor or DBG OTOS commands compiled in.

## Scope

### In Scope

- Add `Hardware::tick(now, cmds)` debug-build overload; `NezhaHAL` and `MockHAL` implementations.
- Replace `robot.benchOtosTick(now)` in `LoopTickOnce.cpp` with `hal.tick(now, state.commands)`.
- Delete `Robot::benchOtosTick`, `Robot::isBenchOtosActive`, `Robot::_lastBenchTickMs`, both `static_cast<NezhaHAL*>` sites in `Robot.cpp`, and `#ifndef HOST_BUILD` blocks for this feature.
- Rework `handleDbgOtosBench` / `handleDbgOtos` in `DebugCommandable.cpp` to remove the `hal` downcast.
- Remove direct `hal.tick(now)` calls (`sim_api.cpp:183, 492`) and the explicit bench-sensor poke (`sim_api.cpp:685`) from the sim harness; let the loop call drive both.
- Fix `DBG OTOS` integer formatting (F1): replace `%f` with scaled-integer format consistent with SNAP (`mm`, `cdeg`/`mrad`).
- Exclude `BenchOtosSensor` and DBG OTOS commands from the production build via a compile-define guard.

### Out of Scope

- Runtime hardware-set selection (`HW SET` + reboot).
- `MotorCommands` to `ActuatorCommands` rename.
- Gripper servo joining the command state.
- Any changes to `IOtosSensor` interface, `OtosSensor` production path, safety/watchdog behavior, or persisted config.

## Test Strategy

Every ticket requires:
1. `python3 build.py` clean firmware build (normal ~98.33% RAM is expected).
2. `uv run --with pytest python -m pytest host_tests/ host/tests/` green.

The F1 integer-format fix is NOT verifiable by the host sim (host libc has full `%f`). The relevant ticket calls for an integer-format unit assertion (host-testable: format the same values with the new integer path and check they are non-empty and in-range) PLUS an on-hardware verification step where `DBG OTOS` output is read by the stakeholder.

The sim harness changes (ticket 5) carry the most regression risk: removing the direct `hal.tick` calls changes when `MockHAL::tick` executes relative to `controlCollectSplitPhase`. The ticket specifies verification by running `sim_tick_collect_tlm` behavior tests.

## Architecture Notes

See `architecture-update.md` for the full design rationale. Key constraints:
- The two variation points are `#ifdef`-guarded: hardware creation (main.cpp vs sim_api.cpp) and the `hal.tick` call form in `loopTickOnce`.
- `Robot`, `Odometry`, `MotionController` must have zero bench/sim mentions after this sprint.
- `DebugCommandable` is firmware-side (not compiled into host builds via existing guards) and may hold the `NezhaHAL*` directly — the remaining concrete-type knowledge lives there, not in `Robot`.

## GitHub Issues

(None linked yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Add Hardware::tick(now, cmds) overload; implement in NezhaHAL | — |
| 002 | Replace benchOtosTick in loopTickOnce with guarded hal.tick(now, cmds) | 001 |
| 003 | Delete benchOtosTick, isBenchOtosActive, _lastBenchTickMs and HOST_BUILD guards from Robot | 002 |
| 004 | Rework DebugCommandable DBG OTOS handlers: remove hal downcast, fix integer formatting (F1) | 003 |
| 005 | MockHAL: add tick(now, cmds) overload; update sim_api to pass cmds and remove bench poke | 001 |
| 006 | Exclude BenchOtosSensor and DBG OTOS commands from production build | 004, 005 |

Tickets execute serially in the order listed. Note: ticket 005 depends only on 001 (not on 002/003/004), so it can be worked in parallel with 002–004 if desired, but the serial sequence 001→002→003→004→005→006 is also correct and simpler to manage.
