---
sprint: '032'
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Architecture Update -- Sprint 032: Comprehensive bench validation (Bench OTOS)

## What Changed

No architecture changes in this sprint. Sprint 032 is a pure validation sprint.

## Why

Sprint 030 (correctness fixes N1–N16) and sprint 031 (Bench OTOS) delivered substantial
firmware and host-side changes. Sprint 032 validates these on real hardware and fixes a
unit-parsing bug in the existing sim harness — neither constitutes an architectural
change.

## Impact on Existing Components

The sim harness fix (`host_tests/test_zz_comprehensive_bench_validation.py`) corrects
unit conversions for `pose` heading (centidegrees → degrees: divide by 100) and `twist`
omega (mrad/s → rad/s: divide by 1000) before comparisons. No firmware or protocol
changes. The host parser (`host/robot_radio/robot/protocol.py`) `TLMFrame` already
documents the correct units; the harness was simply not converting them.

## Migration Concerns

None. No schema changes, no firmware changes, no interface changes.

## Test Plan

| Sequence | Method | Pass Criteria |
|---|---|---|
| TURN × 4 (90 deg each) | Hardware via rogo + STREAM | Total heading ≤ 720 deg; clean stops; omega bounded |
| 300 mm square (D + TURN) | Hardware via rogo + STREAM | |dv| ≤ 120 mm/s/tick; ekf_rej climb ≤ 20 |
| Velocity profiles (150/300/500 mm/s D, T) | Hardware via rogo + STREAM | No instant start; |dv| ≤ 120; heading drift ≤ 25 deg |
| Sim harness regression | `uv run --with pytest python -m pytest host_tests/test_zz_comprehensive_bench_validation.py -s` | pytest passes; report values in correct units |
