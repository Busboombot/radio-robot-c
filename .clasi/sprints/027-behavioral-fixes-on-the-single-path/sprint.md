---
id: '027'
title: Behavioral fixes on the single path
status: planning-docs
branch: sprint/027-behavioral-fixes-on-the-single-path
use-cases: []
issues:
- d06-keepalive-must-not-mutate-active-command
- d08-pursuit-law-hardening
- d09-otos-validity-gating
- field-profile-test-harness-and-ci
- hardware-smoke-ritual-and-field-log
- bench-programs-runaway-auto-abort
- field-024-full-speed-spin-unresolved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 027: Behavioral fixes on the single path

## Goals

The remaining field-behavior defects — keepalive stomping active commands,
unbounded pursuit curvature, OTOS validity poisoning the EKF — are fixed and
now reproducible in sim. The four incident scenarios from the sim2real review
become named regression tests. The hardware smoke ritual and bench safety
wrapper are in place as standing process gates.

## Problem

With sprint 026's single dispatch path operational, these defects can finally
be reproduced and verified in sim:

- **D6:** `handleVW`'s no-stop-params branch calls `setTarget(v, ω)` on any
  active MotionCommand. A `VW` or `S` keepalive arriving during a TURN
  overwrites the TURN's ω, causing it to complete at the wrong heading with
  `EVT done TURN` as if it succeeded. Silent navigation corruption.
- **D8:** Pursuit curvature `κ = 2·dy/d²` is unbounded near the target;
  arrival tolerance is 5 mm (unreachable on carpet); no mid-flight re-gating
  if the target ends up behind the robot. Causes the "stops and pivots /
  hunting" field behavior.
- **D9:** OTOS status register (tilt/tracking-invalid flags) is never read.
  A lifted or just-placed robot feeds zeros/garbage into the EKF velocity
  update — which sits inside the χ² gate — dragging fused velocity to zero
  while the controller fights it. Direct cause of the "spin on placement"
  failure mode.

Additionally, the **field-profile test harness** (CI runs tests twice: exact
profile and field profile with slip, fusion, deadband, latency) needs to land
here so that every subsequent sprint's firmware changes are verified under
conditions that reproduce real failures, not the friendly sim defaults.

The `field-024-full-speed-spin-unresolved` open leads — SNAP enc=0/mode=IDLE
while spinning, host abandoning an autonomous G without sending X — are
addressed here: the SNAP TLM discrepancy maps to d10 telemetry (partial) and
the D9 OTOS validity path; the host-abandons-G pattern is directly covered by
the bench-programs-runaway-auto-abort wrapper.

## Solution

**D6:** Add `MotionCommand::Origin` enum set at begin time. In `handleVW`'s
no-stop-params branch, only `setTarget` for plain VW sessions; for any other
origin, reset the watchdog, reply `OK vw busy=<origin>`, do NOT setTarget.
Update `protocol.py` docstrings. The `+` keepalive is the right keepalive for
everything else.

**D8:** In the PURSUE per-tick hook: clamp curvature; re-gate to PRE_ROTATE
if bearing > 90° for ~3 consecutive ticks; widen `arriveTolMm` to 20–25 mm
with the position stop radius ≥ worst-case decel distance. Values land in
`tovez.json` → regenerate DefaultConfig.

**D9:** Read the OTOS STATUS register in `otosCorrect()`'s cadence before
using pose/velocity. On warn/fatal or I2C read failure, set
`state.inputs.otos.valid = false` and skip fusion that tick. Distinguish I2C
zeros from genuine (0,0,0). Emit `EVT otos lost` after 500 ms of invalidity
during active motion. Fix the mounting-offset lever-arm transform.

**Field-profile harness:** Define the field-profile sim fixture (OTOS+EKF
fusion ON, MockMotor slip set to measured values, motor deadband ~35 PWM,
~15 ms latency). Wire all motion-control tests to run twice in CI. Encode the
four §4 incident scenarios as named regression tests (G-into-boards,
fast-spin-on-placement, TURN-under-rotate, keepalive-kills-TURN) — these
should fail against today's code and pass after the Dx fixes land.

**Hardware smoke ritual:** Scripted 5-minute bench check in `tests/bench/`
— SAFE query, TURN×4 closure, G square, lift test (EVT otos lost, no spin),
stream drop-rate print from TLM seq gaps. Appends to `docs/knowledge/field-log.md`
with date + git SHA. Run before and after every firmware flash.

**Bench runaway wrapper:** Small shared safety wrapper for `tests/bench/` and
`tests/dev/` drive programs — bounded duration, runaway detection (full-tilt
with no encoder motion, no-progress, frozen encoders) → immediate X + abort,
always-X on exit/exception/Ctrl-C, preflight liveness check.

## Success Criteria

- §4 scenario tests (G-into-boards, fast-spin-on-placement, TURN-under-rotate,
  keepalive-kills-TURN) pass in the field profile.
- D6 sim test: start TURN, inject `S 0 0` mid-turn → TURN completes at the
  commanded heading.
- D8 sim test (field profile): targets at 0°, ±90°, 180°, and 30 mm lateral
  offset all converge; no orbit > 1.5 revolutions.
- D9 hardware test: lift mid-G → `EVT otos lost`, no spin on placement.
- Hardware smoke ritual: script exists, runs, logs to field-log.md.
- Bench runaway wrapper: induced runaway sends X within detection window.

## Scope

### In Scope

- `source/control/MotionController.cpp` — D6 Origin enum + handleVW guard,
  D8 curvature clamp + re-gate + tolerance widening.
- `source/sensors/OtosSensor.cpp/.h` — D9 STATUS register read, valid gating.
- `source/robot/Robot.cpp` — D9 EVT otos lost emission.
- `data/robots/tovez.json` + `scripts/gen_default_config.py` — D8 tolerance
  values.
- `host/robot_radio/robot/protocol.py` — D6 docstring update.
- `host_tests/` — field-profile fixture, incident scenario regression tests,
  CI dual-profile gate.
- `tests/bench/smoke_ritual.py` + `docs/knowledge/field-log.md`.
- `tests/bench/` / `tests/dev/` runaway safety wrapper.

### Out of Scope

- D10 firmware telemetry (seq numbers, idle rate) — sprint 028.
- Calibration consolidation — sprint 028.
- Navigation ownership decision — sprint 029.
- A2/A5 structural refactors (completed in 025–026).

## Test Strategy

- All motion-control tests run in both exact and field profiles in CI.
- Four named incident scenario regression tests.
- Hardware smoke ritual runs and logs before and after firmware flash.
- Bench wrapper test: induced runaway aborts within the detection window.

## Architecture Notes

D8's PRE_ROTATE re-gate path depends on PRE_ROTATE being supervised (D5,
fixed in sprint 024). D9 bounds the spin-on-placement trigger; D8's PRE_ROTATE
supervision contains the spin if D9 is slow to detect. Both fixes are needed.

The `field-024-full-speed-spin-unresolved` anomaly — SNAP `enc=0`/`mode=IDLE`
while spinning — overlaps the D10 telemetry stream work (seq numbers, SNAP vs
STREAM frame divergence). The SNAP TLM frame discrepancy introduced in 024-005
(`buildTlmFrame` changes) must be diagnosed here as part of D9/D10 triage; if
it requires D10 firmware changes it will be flagged and addressed in sprint 028.
The host-abandons-G pattern is directly closed by the bench runaway wrapper.

The `set-config-validation` issue (SET writes invalid live config via raw
atof/atoi with no range checks) is a filler item that fits naturally here
alongside the D8 tolerance values landing in DefaultConfig. Include it at
detail-planning time if sprint capacity allows; defer to sprint 028 otherwise.

## Why Third

These fixes require the single dispatch path (sprint 026) for reliable sim
reproduction. Fixing behavioral defects before the dispatch path was unified
would mean writing tests against a code path that is about to be deleted.

## Sizing

Medium — approximately 2 focused sessions.

## GitHub Issues

(None yet — link when created.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Field-profile CI gate and incident scenario regression tests | — |
| 002 | Bench runaway safety wrapper and bench program hardening | — |
| 003 | D6: handleVW Origin guard — keepalives must not stomp active commands | 027-001 |
| 004 | D8: Pursuit-law hardening — curvature clamp, re-gate, arriveTolMm widening | 027-001 |
| 005 | D9: OTOS validity gating and hardware smoke ritual | 027-002, 027-003, 027-004 |
| 006 | field-024 diagnosis closure: SNAP TLM discrepancy + issue resolution | 027-002, 027-005 |

Tickets execute serially in the order listed.
