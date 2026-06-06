---
id: '015'
title: Encoder/I2C Wedge Diagnosis and Fix
status: done
branch: sprint/015-encoder-i2c-wedge-diagnosis-and-fix
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
issues:
- residual-motor-encoder-wedge-after-stop.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 015: Encoder/I2C Wedge Diagnosis and Fix

## Goals

Instrument the firmware and create a deterministic host harness so the encoder
wedge can be reproduced headlessly, observed from inside the firmware, and
correlated with actual I2C error codes and re-entrancy data -- before any fix
is written.

This sprint covers **Phase 0** (headless repro harness) and **Phase 1**
(I2C instrumentation + firmware wedge detector) from the issue plan at
`.clasi/issues/residual-motor-encoder-wedge-after-stop.md`.

**Phase 2** (theory discrimination experiments) and **Phase 3** (fix
implementation) are explicitly out of scope. Those phases require the
stakeholder and team-lead to run hardware experiments using the instrumentation
produced here. New tickets for those phases will be created once the root cause
is identified.

## Problem

The encoder intermittently freezes at a constant value while a drive is
commanded. The robot recovers only on a micro:bit reset. The failure is
micro:bit-side (TWIM/CODAL state, not the Nezha chip), intermittent, and
invisible -- every I2C return code is currently discarded. Without a
deterministic reproduction and firmware-side visibility, the root cause cannot
be confirmed and any fix would be a guess.

## Solution

Two parallel workstreams (T1 and T2 are independent; T3 depends on T2):

1. **Wedge repro harness** (`tests/bench/wedge_repro.py`): pure-pyserial,
   drives N cycles with two stop-trigger modes (clean STOP vs watchdog-fired
   stop), auto-detects the wedge, and reports a numeric wedge rate.

2. **I2CBus wrapper** (`source/hal/I2CBus.h/.cpp`): thin class wrapping
   `MicroBitI2C&` that captures CODAL return codes per-device and holds a
   diagnostic re-entrancy guard (not a lock). All four device classes threaded
   through it.

3. **DBG I2C + EVT enc_wedged**: `DBG I2C` command dumps all I2CBus counters
   over serial; `EVT enc_wedged` fires automatically when N consecutive
   identical encoder reads occur while commanded to drive.

## Success Criteria

- `wedge_repro.py` produces a numeric wedge rate for both stop-trigger modes.
- `DBG I2C` returns a line without crashing and shows accumulating txn counts.
- `EVT enc_wedged` fires when a wedge is induced on hardware, with bus error
  and re-entrancy stats included.
- Build and host tests pass clean after all firmware changes.

## Scope

### In Scope

- `tests/bench/wedge_repro.py` -- new host bench script
- `source/hal/I2CBus.h/.cpp` -- new firmware HAL wrapper
- Constructor sig change for Motor, OtosSensor, LineSensor, ColorSensor
  (`MicroBitI2C&` -> `I2CBus&`)
- `source/main.cpp` -- construct `I2CBus bus(uBit.i2c)`; update 4 device ctors
- `DBG I2C` command in `source/app/CommandProcessor.cpp`
- `EVT enc_wedged` detector in `source/control/MotorController.cpp`
- Wiring `I2CBus` access to CommandProcessor and MotorController

### Out of Scope

- Phase 2 experiments (sensor gating, fullStop path comparison, recovery test)
- Phase 3 fix implementation (TWIM recovery, idle-gap enforcement, etc.)
- Any change to the cooperative loop rate, PID, or drive behavior
- `velocity_chart.py` hardening (deferred to the fix sprint)

## Test Strategy

- Each firmware ticket: `python3 build.py --clean` must pass with zero new
  errors or warnings.
- After firmware tickets: `uv run --with pytest python -m pytest` for host
  regression testing.
- Bench acceptance: `wedge_repro.py` run on hardware robot per SUC-001
  acceptance criteria.

## Architecture Notes

See `architecture-update.md`. Key decision: `I2CBus` is a diagnostic guard,
not a lock. The re-entrancy guard measures T3 (concurrency) rather than
asserting it. If the counter never trips, T3 is ruled out by data.

## GitHub Issues

(None linked yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Headless wedge reproduction harness (tests/bench/wedge_repro.py) | -- |
| 002 | I2CBus wrapper with re-entrancy guard and return-code capture | -- |
| 003 | DBG I2C dump command and EVT enc_wedged firmware detector | 002 |

Tickets execute serially in the order listed. T001 and T002 are independent and
can be worked in any order; T003 must follow T002.
