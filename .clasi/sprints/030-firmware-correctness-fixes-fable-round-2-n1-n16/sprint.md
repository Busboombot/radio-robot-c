---
id: '030'
title: Firmware correctness fixes (Fable round-2 N1-N16)
status: planning
branch: sprint/030-firmware-correctness-fixes-fable-round-2-n1-n16
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
- SUC-009
issues:
- fr2-n1-atomic-encoder-reset.md
- fr2-n2-queue-rewire.md
- fr2-n3-tlm-null-ctx.md
- fr2-n4-n5-cancel-if-active.md
- fr2-n6-config-validation.md
- fr2-n7-queue-full-err.md
- fr2-n8-n9-sensor-validity.md
- fr2-n10-halt-baselines.md
- fr2-n11-16-cleanup.md
- d12-numerical-and-timing-hygiene.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 030: Firmware correctness fixes (Fable round-2 N1-N16)

## Goals

Fix all 16 correctness findings from the round-2 Fable code review
(`docs/code_review/2026-06-12-Fable-correctness-review/findings.md`). Every fix is
surgical and targeted — no redesigns. After this sprint the High and Med-High field
risks (encoder pose corruption, firmware/sim dispatch split, TLM HardFault, zombie
stop conditions) are eliminated, and the remaining Medium/Low findings are closed.

## Problem

The round-2 review identified critical correctness gaps:
- N1 (High): every `D` command teleports the EKF pose backward by the prior segment's
  travel; `ZERO enc` freezes encoders.
- N2 (High): Phase 3 reassignment silently wipes the queue wiring; firmware runs the
  direct path while the sim tests the queue path.
- N3 (High, crash-grade): `SET tlmPeriod` without STREAM causes a null fn-pointer
  HardFault; mixed serial+radio TLM uses the wrong context.
- N4+N5 (Med-High): `beginStream`, `beginRawVelocity`, `beginTimed`, `beginDistance`
  skip the cancel-if-active contract, leaving zombie stop conditions.
- N6-N10 (Med): config validation gaps, silent queue overflow, sticky sensor TLM,
  one-tick-stale OTOS gate, boot-epoch HALT baselines.
- N11-N16 (Low): spurious PURSUE events, GET serial truncation, dead code, corrId
  truncation, EKF Q loop-rate coupling, silent sensor-stop validation skip.

## Solution

Ten targeted tickets in dependency order, following the review's suggested fix order
(N1+N2 first — small and high-impact, then N3, N4+N5, Medium cluster, cleanup). All
changes land in `source/`; verification is primarily by sim regression tests in
`host_tests/` and `host/tests/`.

## Success Criteria

- All 16 findings (N1-N16) addressed with code changes and sim regression tests.
- `python3 build.py` clean build passes on all tickets.
- `uv run --with pytest python -m pytest host_tests/ host/tests/` passes end-to-end.
- The four residual-risk regression tests from findings.md added: D-then-G pose
  continuity (N1), boot queue-wiring test (N2), SET tlmPeriod without STREAM (N3),
  S mid-TURN on queue path (N4).
- N12 GET truncation bench-confirmed or chunked defensively.

## Scope

### In Scope

- All findings N1-N16 from the round-2 Fable review
- All items in `d12-numerical-and-timing-hygiene.md` (#1 absorbed into N15, #4 into N1)
- New sim regression tests for every finding
- Bench confirmation of GET truncation (N12)

### Out of Scope

- Host-side (`host/robot_radio/`) changes
- Navigation / pose-authority work (sprint 029)
- Any firmware feature additions

## Test Strategy

Every ticket produces at least one new sim test. Verification command for all tickets:
```
uv run --with pytest python -m pytest host_tests/ host/tests/
```
Build verification: `python3 build.py` (clean, from repo root).
N12 bench step: `uv run rogo get` over serial to confirm full config delivery.

## Architecture Notes

See `architecture-update.md` in this sprint directory. Key decisions: `Robot::resetEncoders()`
consolidates the split encoder reset (N1); one-line re-wire in `run_blocks()` mirrors
`run_test()` (N2); TLM null guard uses silent suppression, not ERR (N3); N12 GET
chunking is bench-gated but implemented defensively.

## GitHub Issues

(None linked yet.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | N1: Atomic encoder reset — eliminate D-command and ZERO enc pose corruption | — |
| 002 | N2: Re-wire queue in run_blocks() — restore firmware queue-path dispatch from boot | — |
| 003 | N3: Guard TLM null function pointer and fix fn/ctx mismatch (crash-grade) | — |
| 004 | N4+N5: Uniform cancel-if-active across all begin*() entry points | 002 |
| 005 | N7: Report queue overflow with ERR full/busy — silent enqueue failures | 002 |
| 006 | N6: Extend validateConfig() to cover rate/accel/timeout family | — |
| 007 | N10: HALT TIME/DIST baseline at registration time, not boot epoch | — |
| 008 | N8+N9: Sensor freshness gate in TLM and same-tick OTOS fusion skip | — |
| 009 | N11+N14+N15+N16: Correctness cleanup A — spurious events, corrId width, EKF Q scaling, sensor-stop validation | 002 |
| 010 | N12+N13: Correctness cleanup B — GET serial chunking and dead code removal | 009 |

Tickets execute serially in the order listed.
