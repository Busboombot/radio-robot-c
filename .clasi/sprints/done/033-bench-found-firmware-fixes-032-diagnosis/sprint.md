---
id: '033'
title: Bench-found firmware fixes (032 diagnosis)
status: done
branch: sprint/033-bench-found-firmware-fixes-032-diagnosis
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- fr-bench-twist-fusedv-zero.md
- fr-bench-dbg-otos-no-reply.md
- fr-bench-d-distance-baseline-race.md
- fr-bench-right-encoder-wedge.md
---

# Sprint 033: Bench-found firmware fixes (032 diagnosis)

## Goals

Fix the five root-cause firmware and bench-harness bugs identified by the sprint 032
hardware bench validation and subsequent code-analysis diagnosis. These are surgical
targeted fixes — root causes are fully identified; no exploratory work required.

## Problem

Sprint 032 bench validation produced confusing results because of five compounding bugs:

1. The bench harness talked to the relay (wrong transport) — DBG replies were routed to
   the robot's USB serial by design, so the relay harness never saw them.
2. Even over USB serial, `DBG OTOS BENCH 1` never engages bench mode — the
   parse→setOtosBench→isBenchMode round-trip is broken and always replies `bench=0`.
3. EKF encoder-velocity fusion is buried inside the OTOS validity gates — when OTOS is
   lifted/invalid (bench stand), `twist=` stays 0 forever regardless of wheel motion.
4. `D` after `TURN` (without `ZERO enc`) instant-completes with zero motion — the distance
   baseline snapshot races the encoder input zeroing.
5. The encoder wedge detector cannot distinguish hardware fault from filter-hold from stall;
   odometry integrates phantom heading from a single wedged wheel.

## Solution

Five coherent, dependency-ordered firmware fixes plus bench harness rewrite:

1. Rewrite bench harness to use the robot's USB serial directly (no relay).
2. Fix the DBG OTOS BENCH enable bug so bench mode actually engages.
3. Ungate encoder-velocity fusion so `twist=` tracks encoder motion when OTOS is invalid.
4. Fix the D distance baseline race by zeroing encoder inputs before the baseline snapshot.
5. Harden the encoder wedge detector and odometry defense (items a-e from the diagnosis).

## Success Criteria

- All four linked issues addressed (fr-bench-dbg-otos-no-reply Part B, fr-bench-twist-fusedv-zero,
  fr-bench-d-distance-baseline-race, fr-bench-right-encoder-wedge hardening items).
- `python3 build.py` clean build passes.
- `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.
- New sim tests cover finding 3 (enc-velocity when OTOS invalid) and finding 4 (D-after-TURN
  baseline race), and wedge defense items where mockable.

## Scope

### In Scope

- `tests/bench/bench_validation_032.py` — rewrite to direct USB serial
- `tests/bench/enc_balance_test.py` — rewrite to direct USB serial
- `source/app/DebugCommandable.cpp` — fix `handleDbgOtosBench` token/pointer round-trip
- `source/control/Odometry.cpp` — ungate encoder-velocity fusion
- `source/control/MotionController.cpp` — fix D distance baseline ordering
- `source/robot/Robot.cpp` — move encoder-input zeroing before `_activeCmd.start()`
- `source/hal/Motor.cpp` — median-of-3 + readback verification for `resetEncoder()`
- `source/control/MotorController.cpp` / `source/control/MotorController.h` — raw read in EVT,
  arming grace, expose wedge state
- Sim tests for findings 3, 4, and 5 hardening items
- Physical wedge hardware re-run is a post-sprint team-lead task (not an acceptance gate)

### Out of Scope

- Hardware root-cause investigation of the physical encoder wedge (battery vs. chip fault)
- Changes to `ForceReply` routing (DBG replies stay on serial by design)
- Any protocol or TLM schema changes beyond the `enc_wedged` EVT raw-field addition
- Bench re-run execution (team-lead task post-sprint)

## Test Strategy

All firmware fixes verified in the sim (`host_tests/` and `host/tests/`). Build command:
`python3 build.py` (clean). Test command: `uv run --with pytest python -m pytest host_tests/ host/tests/`.

New sim tests required per ticket:
- T003: OTOS invalid + wheels moving → `fusedV`/`fusedOmega` nonzero; `enc_omega` gated on both wheels healthy
- T004: D → TURN → D (no ZERO) → second D travels full commanded distance
- T005: Mock garbage ZERO-enc read → readback retry; wedged wheel → no phantom dTheta

## Architecture Notes

- Encoder-velocity fusion (T003) and enc_omega wedge-gating (T005 item e) are coupled:
  T003 adds the unconditional `enc_omega` observation; T005 adds the suppression gate.
  T005 depends on T003.
- T002 (bench mode fix) and T003 (fusion ungate) are independent but both required to make the
  post-sprint bench re-run meaningful.
- DefaultConfig.cpp is auto-generated — run `scripts/gen_default_config.py` if a firmware
  default changes.

## GitHub Issues

(none yet linked)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Rewrite bench harness to use robot USB serial | — |
| 002 | Fix DBG OTOS BENCH enable bug | — |
| 003 | Ungate EKF encoder-velocity fusion | 002 |
| 004 | Fix D distance baseline race | — |
| 005 | Encoder wedge detector and odometry hardening | 003 |

Tickets execute serially in the order listed.
