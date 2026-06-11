---
id: '024'
title: Field-safety and heading-truth motion fixes (P0)
status: planning-docs
branch: sprint/024-field-safety-and-heading-truth-motion-fixes-p0
use-cases:
  - SUC-001
  - SUC-002
  - SUC-003
  - SUC-004
  - SUC-005
issues:
  - d05-bound-goto-pre-rotate-phase.md
  - d07-motioncommand-ownership-in-pre-rotate.md
  - d04-watchdog-role-and-safe-rearm.md
  - d01-fuse-otos-heading-into-ekf.md
  - d03-ekf-gate-recovery-path.md
  - d02-apply-rotational-slip.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 024: Field-safety and heading-truth motion fixes (P0)

## Goals

Eliminate the confirmed P0 field failures documented in the 2026-06-11 sim-to-real
review: (1) unbounded spin when GO_TO is issued to a behind-the-robot target with a
frozen or wrong heading, and (2) systematic navigation failure ("gets turned around
and drives into the boards") caused by heading being pure encoder integration with
calibrated-but-dead rotational slip correction.

## Problem

Two independent causal chains produce field failures:

**Motion-bounding chain:** `beginGoTo()`'s PRE_ROTATE phase has no MotionCommand, no
HEADING stop, no TIME stop. If the bearing never crosses the exit gate (due to slip,
frozen OTOS, or an encoder wedge), the spin is unbounded. Compounding this: no
cancel-if-active guard means a stale prior command can fire the wrong EVT label. The
watchdog, which should catch this, was demoted to a dead-process detector by the
background keepalive daemon.

**EKF heading-fusion chain:** The EKF fuses x/y and v/ω but never heading. OTOS
heading is read every 100 ms and dropped. Heading is pure encoder integration. The
calibrated `rotationalSlip = 0.74` is defined but applied nowhere in firmware. Once
heading drifts past the Mahalanobis gate threshold, every subsequent OTOS fix is
rejected permanently.

## Solution

**Motion-bounding chain (D5 → D7 → D4):**
1. Replace raw BVC seeding in PRE_ROTATE with a supervised MotionCommand (HEADING +
   TIME stops). Add overall TIME net to the PURSUE phase. Remove instant 180 deg/s
   start (ticket 001).
2. Add cancel-if-active guard to all `begin*()` entry points (ticket 002).
3. Add TIME-stop exemption to watchdog, SAFE one-shot re-arm on new command, quiet
   keepalive reply, and remove sTimeout=60000 overrides (ticket 003).

**EKF heading-fusion chain (D1 → D3 → D2):**
4. Add `EKF::updateHeading()`, wire OTOS heading through Odometry and Robot, fix
   `setPose()` P-prior, add `ekfROtosTheta` config (ticket 004).
5. Add consecutive-rejection streak counters and R-inflation gate recovery; add
   `ekf_rej` telemetry (ticket 005).
6. Apply `rotationalSlip` in `Odometry::predict()` and RT wheel-arc target; fix
   MockMotor turn-slip sign; resolve dead `turnScale`/`distScale` (ticket 006).

The two chains are independent and can execute in parallel, but within each chain
the sequencing constraint is strict.

## Success Criteria

- G to a behind-the-robot target with frozen heading: stops via TIME net, emits
  `EVT done G`, never spins forever.
- Four `TURN 9000` in a row return the robot to starting heading within a few degrees.
- `RT 9000` lands 90° ± 3° physical (up from ~67° today).
- Full square run completes with keepalive daemon OFF, no spurious safety_stops.
- Field-profile sim (slip on, fusion on) passes all motion-bounding and heading-fusion
  regression tests.

## Scope

### In Scope

- Firmware: `MotionController`, `MotionCommand`, `LoopScheduler`, `EKF`, `Odometry`,
  `Robot`, `RobotConfig` changes per the two defect chains.
- Config: new `ekfROtosTheta` field; `DefaultConfig.cpp` regenerated.
- Host firmware lib: `protocol.py`, `NezhaState` — `ekf_rej` TLM field.
- Tests: `tests/dev/test_ekf.py` Python EKF mirror; `host_tests/` field-profile fixture;
  `tests/bench/square_run.py` cleanup; `tests/dev/safe_cmd_bench.py` update.

### Out of Scope

- D6 (keepalive must not mutate active command), D8 (pursuit law hardening), D9 (OTOS
  validity gating), D10/D11/D12 — deferred to future sprints.
- New host-side navigation features; no changes to `CommandProcessor`, `PathFollower`,
  `MotorController`, `RatioPidController`, or HAL.

## Test Strategy

Every ticket requires a host_tests field-profile regression test (slip + fusion ON)
and a hardware check. The Python EKF mirror (`tests/dev/test_ekf.py`) must be updated
in lockstep with firmware EKF changes (tickets 004 and 005). The bench script
(`tests/bench/square_run.py`) is updated as part of ticket 003. Verification command
for firmware changes: `uv run pytest host_tests/ tests/dev/test_ekf.py`.

## Architecture Notes

See `architecture-update.md` for full module definitions, diagrams, design rationale,
and open questions. Critical sequencing: D5 → D7 → D4 (D4's keepalive exemption is
only safe once per-command TIME nets exist). D1 → D3 → D2 (gate recovery before slip
correction avoids transient gate strangulation). The two chains are otherwise independent.

Open questions requiring team-lead / stakeholder resolution before or during execution:

1. `ekfROtosTheta` initial value (0.01 vs. 0.04 rad²) — verify against field-profile sim.
2. `turnScale` / `distScale` — remove or wire? Confirm before ticket 006 execution.
3. `+` quiet keepalive — firmware-side (preferred) or host-filter fallback? Confirm
   before ticket 003 execution.
4. `wrapPi` utility — `wrap_angle()` from StopCondition.cpp or inline atan2f? Confirm
   before ticket 004 execution.

## GitHub Issues

(None linked yet — all source issues are CLASI issues.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed (APPROVE WITH CHANGES — 2026-06-11)
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 024-001 | Bound GO_TO PRE_ROTATE with supervised MotionCommand and PURSUE TIME net | — |
| 024-002 | Cancel stale MotionCommand on beginGoTo PRE_ROTATE and all begin* entry points | 024-001 |
| 024-003 | Watchdog TIME-stop exemption, SAFE one-shot re-arm, and quiet keepalive | 024-001, 024-002 |
| 024-004 | Fuse OTOS heading into EKF and set sane P-prior on setPose | — |
| 024-005 | EKF gate recovery: consecutive-rejection R-inflation and ekf_rej telemetry | 024-004 |
| 024-006 | Apply rotational-slip correction in Odometry predict and RT wheel-arc target | 024-004, 024-005 |

Tickets execute serially in the order listed. The two chains (001-002-003 and
004-005-006) are independent and may be executed in either order, but within each
chain the ordering is strict.
