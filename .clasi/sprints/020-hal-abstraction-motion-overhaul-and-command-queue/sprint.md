---
id: '020'
title: HAL Abstraction, Motion Overhaul, and Command Queue
status: planning-docs
branch: sprint/020-hal-abstraction-motion-overhaul-and-command-queue
use-cases: []
issues:
- hal-mockhal-implementation-plan.md
- plan-sprint-020-firmware-host-testing.md
- issue-motion-system-overhaul.md
- plan-command-flags-vw-unification-command-queue-and-test-loop.md
---

# Sprint 020: HAL Abstraction, Motion Overhaul, and Command Queue

## Goals

1. Introduce a HAL interface layer (`IMotor`, `IOtosSensor`, etc.) with a `NezhaHAL` concrete implementation and a `MockHAL` for offline testing.
2. Add a host-side CMake build target so firmware control logic can be compiled and tested without flashing hardware.
3. Overhaul the motion system: unify all motion commands through `BodyVelocityController`, replace per-command keepalives with a single system watchdog, add `_VW`/`X soft`, and implement first-class HALT stop conditions.
4. Add `ACCESS_HARDWARE` flags to `CommandDescriptor`, convert S/T/D/G/R/TURN to VW converters pushing to a parsed command queue, and add `LoopScheduler::run_test()` for hardware-free command dispatch verification.

## Problem

The firmware has no abstraction between controllers and concrete HAL devices (motors, sensors), making offline testing impossible. The motion system has two parallel code paths to the motor (legacy bypass vs BVC), duplicated keepalive watchdogs, and no way to register user-named stop conditions. Commands have no metadata about hardware access, so a test loop cannot safely skip hardware-touching commands.

## Solution

**Phase A — HAL + MockHAL + Host Build:** Pure-virtual interfaces for each device type; `NezhaHAL` wraps existing concretions; `MockHAL` provides simulated physics; host CMake target compiles control code for native test execution.

**Phase B — Motion System Overhaul:** All motion routes through BVC only. Single `_watchdogMs` on LoopScheduler. `_VW` raw command, `X soft` variant. `HaltController` for named stop conditions (HALT TIME/DIST/POS/COLOR/LINE family).

**Phase C — Command Flags + Queue + Test Loop:** `ACCESS_HARDWARE` flag in `CommandDescriptor`; parsed command queue in `CommandProcessor`; S/T/D/G/R/TURN become VW converters; `run_test()` dispatches non-hardware commands and reports skips.

## Success Criteria

- `python3 build.py --clean` succeeds with no warnings on the firmware target.
- Host CMake build compiles control layer for native execution.
- MockHAL test exercises motor commands and reports encoder deltas.
- All existing firmware tests pass (`uv run --with pytest python -m pytest`).
- `S`, `T`, `D` commands over serial in `run_test()` produce `DBG skip VW ...` output (no motor writes).
- `HALT TIME 1500` fires `EVT halt` after ~1.5 s of driving.

## Scope

### In Scope

- HAL interfaces (`IMotor`, `ILineSensor`, `IColorSensor`, `IOtosSensor`, `IPortIO`, `IServo`, `Hardware`)
- `NezhaHAL` concrete implementation wrapping existing devices
- `MockHAL` + all mock device classes
- Host CMake target + Python test harness
- Motion system BVC unification (all commands through BVC)
- System watchdog (`_watchdogMs` on LoopScheduler)
- `_VW` raw command, `X soft` variant, `+` keepalive command
- `HaltController` + HALT command family (TIME/DIST/POS/COLOR/LINE)
- `ZERO T` and `ZERO D` baseline commands
- `ACCESS_HARDWARE` flag in `CommandDescriptor` / `makeCmd()`
- Parsed command queue (`CommandQueue` ring buffer)
- S/T/D/G/R/TURN → VW converter refactor
- `LoopScheduler::run_test()`

### Out of Scope

- Runtime switching into test mode via a `DBG TEST` command (future sprint)
- Multi-robot HAL registry
- Profiler/trajectory planner

## Test Strategy

- Firmware: existing `uv run --with pytest python -m pytest` suite
- HAL smoke test: build MockHAL, drive 100ms, assert encoder delta > 0
- Host CMake build: `cmake --build` must succeed
- Motion bench: `HALT TIME 1500` fires within 50ms of target

## Architecture Notes

- Implementation order: HAL → MockHAL/Host Build → Motion Overhaul → Command Flags/Queue
- Phase B (motion) requires Phase A (HAL) to be complete first; Phase C requires Phase B.
- `_VW` and HALT family are new wire commands; no backward-compatibility concerns since they are additive.
- Sprint-planner should produce tickets in strict dependency order.

## Issues

- `hal-mockhal-implementation-plan.md` — HAL interfaces, NezhaHAL, MockHAL
- `plan-sprint-020-firmware-host-testing.md` — host CMake target + Python harness
- `issue-motion-system-overhaul.md` — BVC unification, watchdog, HALT conditions
- `plan-command-flags-vw-unification-command-queue-and-test-loop.md` — flags, queue, run_test

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 020-001 | HAL interfaces + NezhaHAL + concrete class inheritance | — |
| 020-002 | MockHAL + all mock device classes | 020-001 |
| 020-003 | Host CMake build + sim_api C wrapper | 020-002 |
| 020-004 | Python test harness + host test suite | 020-003 |
| 020-005 | BVC unification: S and G PRE_ROTATE through BVC + system watchdog | 020-001 |
| 020-006 | _VW raw command, + keepalive, X soft variant | 020-005 |
| 020-007 | HaltController + StopCondition extensions (COLOR, LINE_ANY) | 020-006 |
| 020-008 | HALT POS and HALT COLOR wire commands + complete HALT family | 020-007 |
| 020-009 | ACCESS_HARDWARE flag in CommandDescriptor + annotate all makeCmd calls | 020-008 |
| 020-010 | CommandQueue ring buffer + CommandProcessor queue integration | 020-009 |
| 020-011 | S/T/D/G/R/TURN VW converters + OP cached-state refactor | 020-010 |
| 020-012 | LoopScheduler run_test() serial-only hardware-free dispatch loop | 020-011 |

Tickets execute serially in the order listed.
