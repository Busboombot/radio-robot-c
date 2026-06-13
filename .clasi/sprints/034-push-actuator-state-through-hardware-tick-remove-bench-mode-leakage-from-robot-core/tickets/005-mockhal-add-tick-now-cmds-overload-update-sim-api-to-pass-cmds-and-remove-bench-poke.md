---
id: '005'
title: 'MockHAL: add tick(now, cmds) overload; update sim_api to pass cmds and remove bench poke'
status: open
use-cases:
- SUC-034-002
depends-on:
- '001'
github-issue: ''
issue: hardware-tick-actuator-state.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# MockHAL: add tick(now, cmds) overload; update sim_api to pass cmds and remove bench poke

## Description

Complete the sim-side of the `tick(now, cmds)` plumbing:

1. `MockHAL` must implement the `Hardware::tick(now, cmds)` pure-virtual overload declared in ticket 001. `MockHAL`'s existing `tick(uint32_t now_ms)` already drives the full plant (motors, encoders, OTOS) by reading `_motorL.cmdSpeed()` / `_motorR.cmdSpeed()` from the motor objects. The `cmds` parameter is available to the new overload but MockHAL MAY continue reading from the motor objects — the implementer's choice. The signature must match the pure-virtual to satisfy the compiler.

2. `sim_api.cpp:183` and `sim_api.cpp:492` call `s->hal.tick(t)`. These calls must be upgraded to `s->hal.tick(t, s->robot.state.commands)` so they compile against the new `BENCH_BUILD`-guarded interface. **Ordering invariant: these calls must remain BEFORE `controlCollectSplitPhase` in both `sim_tick` and `sim_tick_collect_tlm`.** Do not move them.

3. `sim_api.cpp:685-715` (the `sim_bench_otos_tick`, `sim_get_bench_otos_x/y/h`, `sim_bench_otos_reset`, `sim_bench_otos_set_noise` standalone functions) and `SimHandle::benchOtos` member are removed. These were a workaround for the missing command-state input channel; they are superseded by the loop-driven `hal.tick(t, cmds)`.

4. Any host tests that called `sim_bench_otos_tick(...)` directly must be updated to drive the bench OTOS behavior through the normal `sim_tick` path instead. Search for `sim_bench_otos_tick` in `host_tests/` and `host/tests/` and update.

### Risk note

This ticket carries the highest regression risk. Removing `SimHandle::benchOtos` and the direct `sim_bench_otos_*` functions may cause compile errors in test files that called them; fix all call sites. The plant ordering (hal.tick before controlCollectSplitPhase) must be preserved exactly — any deviation will cause position/velocity divergence in EKF tests.

## Files to Modify

- `source/hal/mock/MockHAL.h` — add `tick(uint32_t now_ms, const MotorCommands& cmds)` override declaration (guarded with the same `#ifdef BENCH_BUILD` as `Hardware.h`).
- `source/hal/mock/MockHAL.cpp` — implement the overload. Simplest correct implementation: call the existing `tick(now_ms)` logic (either refactor to a private helper or just delegate).
- `host_tests/sim_api.cpp` — upgrade two `hal.tick(t)` calls to `hal.tick(t, s->robot.state.commands)`; remove `sim_bench_otos_tick`, `sim_get_bench_otos_*`, `sim_bench_otos_reset`, `sim_bench_otos_set_noise`; remove `SimHandle::benchOtos` member.
- `host_tests/sim_api.h` (if it exists) — remove `sim_bench_otos_*` function declarations.
- `host_tests/*.py` or `host/tests/*.py` — update any test that called `sim_bench_otos_tick(...)` to use `sim_tick(...)` / `send_command(...)` instead.

## Acceptance Criteria

- [ ] `MockHAL` compiles in HOST_BUILD (satisfies the pure-virtual `tick(now, cmds)` overload without compiler error).
- [ ] `sim_api.cpp` has no direct `s->hal.tick(t)` calls (all two are upgraded to `s->hal.tick(t, s->robot.state.commands)`).
- [ ] `sim_api.cpp` has no `sim_bench_otos_tick`, `sim_get_bench_otos_*`, `sim_bench_otos_reset`, or `sim_bench_otos_set_noise` functions.
- [ ] `SimHandle` has no `benchOtos` member.
- [ ] No test file in `host_tests/` or `host/tests/` calls `sim_bench_otos_tick`.
- [ ] The two `hal.tick(t, cmds)` calls in sim_api remain BEFORE `controlCollectSplitPhase` in their respective tick loops.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` exits green (719+ passing; EKF/odometry/velocity tests must not regress).

## Implementation Plan

1. Confirm the `BENCH_BUILD` define name from ticket 001's findings and apply the same guard.
2. Add the `MockHAL::tick(uint32_t, const MotorCommands&)` declaration and implementation. Refactor `MockHAL::tick(uint32_t)` to a private `_tickPlant(uint32_t dt_ms)` helper, then have both overloads call it — avoids code duplication.
3. Search `sim_api.cpp` for all `hal.tick(` occurrences. Upgrade the two pre-controlCollect calls; verify the order is preserved.
4. Remove the `sim_bench_otos_*` block and `SimHandle::benchOtos`.
5. Search `host_tests/`, `host/tests/`, and any Python files for `sim_bench_otos_tick` calls. For each: identify what the test was exercising and rewrite it using `sim_tick` + command dispatch.
6. Run full sim suite. Investigate any failures carefully — most likely caused by plant-ordering change or removed bench-poke tests.

## Testing

- **Build gate**: `python3 build.py` clean (firmware must still build since `MockHAL` is host-only; confirm no accidental firmware include).
- **Sim gate**: `uv run --with pytest python -m pytest host_tests/ host/tests/` green (all 719+ tests).
- Pay particular attention to: EKF pose accuracy tests, odometry predict tests, velocity fusion tests, any test that previously called `sim_bench_otos_tick`. These are the most likely casualties of the ordering or interface change.
