---
id: '014'
title: Single Cooperative Main Loop (abandon fibers)
status: done
branch: sprint/014-single-cooperative-main-loop-abandon-fibers
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
issues:
- plan-single-cooperative-main-loop-abandon-fibers.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 014: Single Cooperative Main Loop (abandon fibers)

## Goals

Replace the two-fiber firmware model with a single cooperative main loop backed
by a priority task table (`LoopScheduler`). Introduce three authoritative state
structs that replace private state caches scattered across subsystems. Eliminate
the busy-wait encoder I2C and the EVT ring buffer. Confirm the new loop on
hardware bench. Close the sprint 013 deferred bench gate.

## Problem

The current firmware runs two CODAL fibers:
- A **control fiber** (`controlFiberFn`) that executes encoder reads, PID, and
  motor PWM every `controlPeriodMs`.
- A **comms+telemetry fiber** (the `main()` while-loop) that drains serial/radio
  and emits TLM.

The two fibers require two mechanisms that exist solely to make multi-fiber
operation safe: **busy-wait I2C** in `Motor::readEncoderRaw()` (8 ms per tick
per wheel = ~16 ms total, capping control rate at ~40 Hz despite a 10 ms
configured period) and a **lock-free EVT ring buffer** in `DriveController` to
cross the fiber boundary for drive completions.

## Solution

A single cooperative main loop (`LoopScheduler`):
1. **Control task always first**: collect encoder (non-blocking) → PID ×2 →
   PWM write. Cost < 1 ms. Sets the metronome.
2. **Round-robin low-priority sweep**: comms-in, drive-advance, odometry-predict,
   otos-correct, line-read, color-read, ports-read, telemetry-emit — each gated
   by a due-check (per-sensor configurable lag) and a worst-case-cost budget
   gate against the next control deadline.
3. **Split-phase encoder I2C**: `requestEncoder()` fires the `0x46` write at
   the end of each iteration; `collectEncoder()` reads back at the top of the
   next. The idle sleep supplies the vendor's required settling time. Alternating
   wheels (L/R per iteration) gives ~50 Hz per-wheel sample rate.
4. **Three authoritative state structs** (`MotorCommands`, `HardwareState`,
   `TargetState`) replace private caches in all subsystems.
5. **EVT ring removed**: drive completions emitted inline via reply sink
   captured in `TargetState`.

## Success Criteria

- Control rate ≥ 40 Hz confirmed on hardware bench.
- All pytest tests pass.
- Full hardware bench smoke sequence passes (S/T/D/G, streaming watchdog,
  lag tuning, I2C stress, radio stress, no motor throb).
- Sprint 013 deferred bench gate is closed.

## Scope

### In Scope

- New `source/control/RobotState.h` (three authoritative state structs).
- New `source/control/LoopScheduler.{h,cpp}` (task table + cooperative loop).
- `Motor` HAL: split `readEncoderRaw` → `requestEncoder` + `collectEncoder`;
  delete both busy-wait loops.
- `MotorController`: slim to struct-based orchestration; remove private caches.
- `Odometry`: `predict`/`correct` operate on `HardwareState`.
- `DriveController`: EVT ring removed; inline emit; OTOS correct lifted out.
- `Robot`: owns `RobotStateContainer`; granular task entry points.
- `main.cpp`: shrinks to construction + `sched.run()`.
- `CommandProcessor`/`Config.h`: add lag registry entries for per-sensor rates.
- Hardware bench verification (sprint 013 deferred gate + sprint 014 gate).

### Out of Scope

- Navigation layer (`PurePursuitFollower`, `StanleyFollower`) — not in the task
  table for this sprint.
- Motor `readSpeedRaw()` (0x47) — already disabled; stays disabled.
- `moveToAngle()` busy-wait — not in the drive loop; no action this sprint.
- Protocol wire format — unchanged; all verbs and responses are identical.

## Test Strategy

- **Automated (CI)**: `uv run --with pytest python -m pytest` — run after each
  ticket. All tests must pass before advancing to the next ticket.
- **Build verification**: `python build.py` (Docker CODAL) after each ticket.
- **Hardware bench** (ticket 009): full gate per `docs/hardware-bench-testing.md`
  plus sprint 014-specific checks (control rate, lag tuning, I2C stress,
  radio stress).

## Architecture Notes

See `architecture-update.md` for full detail. Key decisions:
- Single loop over retained two-fiber model (I2C atomicity is structural in the
  ordering rule, not mutex-based).
- Alternating wheel encoder requests (one per iteration; ZOH on the non-sampled
  wheel).
- Lag values live in `RobotConfig` flat fields (compatible with `CFG_I`
  registry); `ValueSet` in `HardwareState` retains `lastUpdMs`/`valid` for
  staleness but not `lagMs`.

## GitHub Issues

None (design originated in a CLASI issue: `plan-single-cooperative-main-loop-abandon-fibers.md`).

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Define RobotState.h authoritative state structs | — |
| 002 | Split Motor encoder I/O into requestEncoder / collectEncoder (remove busy-waits) | 001 |
| 003 | Refactor MotorController onto RobotState structs | 001, 002 |
| 004 | Refactor Odometry onto RobotState structs | 001 |
| 005 | Refactor DriveController: remove EVT ring, add task entry points, inline EVT emit | 001, 003, 004 |
| 006 | Implement LoopScheduler with task table and cooperative loop algorithm | 003, 004, 005 |
| 007 | Refactor Robot: expose granular task entry points; add RobotStateContainer ownership | 003, 004, 005, 006 |
| 008 | Shrink main.cpp: switch to LoopScheduler; add lag CFG registry entries | 006, 007 |
| 009 | Hardware bench verification: single-loop firmware end-to-end (closes sprint 013 deferred gate) | 008 |

Tickets execute serially in the order listed.
