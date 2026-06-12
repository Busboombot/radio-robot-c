---
id: '031'
title: Bench OTOS debug sensor
status: ticketing
branch: sprint/031-bench-otos-debug-sensor
use-cases: [SUC-001, SUC-002, SUC-003, SUC-004]
issues:
- bench-otos-synthetic-otos-sensor-for-full-stack-bench-testing.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 031: Bench OTOS debug sensor

## Goals

Add `BenchOtosSensor`, a debug `IOtosSensor` implementation that synthesizes
OTOS pose from commanded wheel velocity so the full firmware stack can be
validated on the bench (robot on a stand, wheels free-spinning). The real
optical OTOS sees no floor motion in this configuration; the synthesized sensor
gives the EKF a valid input, allowing distance stop conditions, TLM pose
streams, and estimator behavior to be exercised without a floor surface.

## Problem

When the robot sits on a stand, the real OTOS reports a frozen pose. The EKF
cannot be exercised, distance-based stop conditions (D command) never fire
because pose never advances, and there is no observable to validate estimator
tuning. Firmware validation is currently blocked by the bench configuration.

## Solution

Implement `BenchOtosSensor` as a concrete `IOtosSensor` that:
- Integrates `state.commands.tgtLMms` / `tgtRMms` each control tick into a
  noiseless ideal accumulator and an errored accumulator (Gaussian noise +
  slow yaw drift).
- Lives at `source/hal/BenchOtosSensor.{h,cpp}` (NOT under `hal/mock/`,
  which is build-excluded on device).
- Always returns `true` from `readTransformed`/`readVelocityTransformed` so
  `Robot::otosCorrect` fuses it unconditionally.
- Is swappable live via `DBG OTOS BENCH 0|1` (volatile, default off) through
  an active-pointer swap in `NezhaHAL`.
- Exposes a `DBG OTOS` query reporting ideal vs errored-OTOS vs EKF-fused pose.

## Success Criteria

- `python3 build.py` clean build passes with all new files included.
- `uv run --with pytest python -m pytest host_tests/ host/tests/` passes,
  including the new BenchOtosSensor integrator correctness tests.
- Post-sprint (team-lead): flash firmware, issue `DBG OTOS BENCH 1`, drive
  with `D 500`, observe `EVT done` fires and TLM shows pose advancing.

## Scope

### In Scope

- `BenchOtosSensor` class (`source/hal/`)
- `NezhaHAL` active-pointer swap (`_otosActive`, `setOtosBench()`)
- `Robot::benchOtosTick()` + `LoopTickOnce.cpp` one-line insert
- `DebugCommandable` additions: `DBG OTOS BENCH` toggle + `DBG OTOS` query
- `host_tests/test_bench_otos.cpp` integrator correctness test

### Out of Scope

- Changes to `IOtosSensor` interface (unchanged from sprint 030)
- Changes to `MockHAL` / `MockOtosSensor`
- Persistent configuration of bench mode (volatile only)
- Hardware bench execution (post-sprint team-lead validation)

## Test Strategy

All firmware changes are verified SIM-side: `python3 build.py` clean build
plus `uv run --with pytest python -m pytest host_tests/ host/tests/`. A
dedicated unit test (`test_bench_otos.cpp`) validates the integrator math
against analytic ground truth before hardware is ever involved.

## Architecture Notes

- `BenchOtosSensor` at `source/hal/` (not `hal/mock/`) — compiled on device.
- Active-pointer swap in `NezhaHAL` because `Robot.otos` is an
  `IOtosSensor&` reference that cannot be reseated after construction.
- Truth source = commanded velocity (`state.commands.tgtLMms/tgtRMms`), not
  encoder feedback — free-spinning encoders are unreliable at zero load.
- PRNG = `microbit_random` on device; deterministic seed under `HOST_BUILD`
  for reproducible unit tests.
- See `architecture-update.md` for full design rationale.

## GitHub Issues

(None — driven by internal issue file.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | BenchOtosSensor class | — |
| 002 | NezhaHAL wiring and Robot::benchOtosTick | 031-001 |
| 003 | DBG OTOS BENCH command and DBG OTOS query | 031-002 |
| 004 | Host-sim unit test: BenchOtosSensor integrator correctness | 031-001 |

Tickets execute serially in the order listed. Ticket 004 depends only on 001
and can in principle run alongside 002; for simplicity it is sequenced last.
