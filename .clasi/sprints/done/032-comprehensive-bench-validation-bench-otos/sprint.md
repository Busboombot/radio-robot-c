---
id: '032'
title: Comprehensive bench validation (Bench OTOS)
status: done
branch: sprint/032-comprehensive-bench-validation-bench-otos
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
issues: []
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 032: Comprehensive bench validation (Bench OTOS)

## Goals

Validate the full firmware stack (sprint 030 correctness fixes N1–N16 + sprint 031
Bench OTOS, both merged to master at v0.20260612.17) on the real robot, using Bench
OTOS (`DBG OTOS BENCH 1`) so the optical-odometry path runs even though the robot is
on a stand. Also fix the sim drive-validation harness so it correctly parses firmware
TLM units and becomes a reliable non-hardware regression gate.

## Problem

Sprint 030 and 031 delivered significant firmware changes (EKF, velocity loop, motion
corrections, Bench OTOS). None of these were hardware-validated after the merge.
Failure signatures of concern: bad starts/stops, large tick-to-tick velocity jumps,
runaway spin (heading/omega uncontrolled), and climbing EKF rejection counts.
Additionally the existing sim harness parses TLM integers with wrong unit assumptions,
producing absurd million-degree heading values and meaningless assertions.

## Solution

Two parallel workstreams:

1. **Hardware validation (T001)** — team-lead executes directly on the robot: enable
   Bench OTOS, STREAM telemetry, drive TURN closure × 4, a D+TURN 300 mm square, and
   D/T velocity profiles at slow/medium/fast speeds. Capture raw TLM logs to `docs/`.
   Write a validation verdict; any pathology found becomes a new issue.

2. **Sim harness fix (T002)** — fix `host_tests/test_zz_comprehensive_bench_validation.py`
   unit-parsing bugs: `pose` heading is centidegrees, `twist` omega is mrad/s (not
   radians/floats). Convert correctly before applying assertions. Make `uv run --with
   pytest python -m pytest host_tests/test_zz_comprehensive_bench_validation.py -s`
   pass with a clean printed report.

## Success Criteria

- Hardware: raw TLM logs captured for all three drive sequences; written verdict
  ("PASS" or specific pathologies filed as issues).
- Sim harness: pytest passes; no absurd million-degree headings; assertions meaningful.

## Scope

### In Scope

- Bench hardware validation via Bench OTOS (T001)
- Fix sim harness TLM unit parsing and make pytest pass (T002)
- Capturing raw TLM to `docs/` for future reference

### Out of Scope

- Firmware code changes
- Architecture changes
- Any motion tuning or parameter changes (those are separate sprints)

## Test Strategy

T001 is manual hardware validation; T002 is a pytest that runs in CI.
No `python3 build.py` needed for T002.

## Architecture Notes

No architecture changes. Validation sprint only. References sprint 030 (N1–N16
correctness fixes) and sprint 031 (Bench OTOS).

## GitHub Issues

(None linked at planning time.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Hardware bench validation via Bench OTOS — turns, square, velocity profiles | — |
| 002 | Comprehensive sim drive-validation harness — fix TLM parsing and assert drive health | — |

Tickets execute serially in the order listed.
