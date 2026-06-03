---
id: '006'
title: 'Bench verification: go-to end-to-end from 3 start positions and VW drive'
status: open
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
depends-on:
- "011-004"
- "011-005"
github-issue: ''
issue: kinematics-pose-control-goto.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 011-006: Bench verification — go-to end-to-end from 3 start positions and VW drive

## Description

This is the **sprint acceptance gate** ticket. It consolidates all hardware
bench tests from tickets 002–005 into a single structured verification run and
confirms the complete behavior specified in the issue Verification section.

No code is written in this ticket. The programmer flash-deploys the firmware
from `master` (with all sprint tickets merged) and runs the bench test sequence
below. Results are noted as pass/fail; any failure opens a bug ticket against
the relevant earlier ticket.

The robot is on the bench stand per the `hardware-bench.md` memory note: all
tests are safe to run. Zero the pose before each go-to test (`ZERO enc pose`).

## Acceptance Criteria

### Go-To: target in front (SUC-001)

- [ ] **Bench**: `ZERO enc pose` then `G 300 0 200 #1` — robot drives straight
  ≈300 mm, stops within `arriveTolMm` (measure with ruler), emits
  `EVT done G #1`. [HARDWARE]
- [ ] **Bench**: `ZERO enc pose` then `G 300 80 200` — robot curves left to
  target, arrives within `arriveTolMm`, emits `EVT done G`. [HARDWARE]
- [ ] **Bench**: Accel phase visible (not a lurch from rest); decel brings robot
  to stop on target, not past it. Verify by eye. [HARDWARE]
- [ ] **Bench**: `SNAP` → `TLM mode=G` while go-to is in progress; `TLM mode=I`
  after completion. [HARDWARE]

### Go-To: target behind/beside (SUC-002 — issue Verification item)

- [ ] **Bench**: `ZERO enc pose` then `G -300 0 150 #2` (target directly behind)
  — robot visibly rotates in place, then pursues forward, arrives within
  `arriveTolMm`, emits `EVT done G #2`. [HARDWARE]
- [ ] **Bench**: `ZERO enc pose` then `G 0 300 150` (target hard left, 90°) —
  robot rotates in-place, then pursues arc to target, arrives within
  `arriveTolMm`. [HARDWARE]
- [ ] **Bench**: Re-steering confirmed: place hand gently on robot during arc
  phase (slight resistance), then release — robot re-steers back to target
  without restarting (receding-horizon behavior). [HARDWARE — qualitative]

### VW drive and watchdog (SUC-003)

- [ ] **Bench**: `VW 200 0` — robot drives straight forward. `STOP` halts it;
  no EVT emitted. [HARDWARE]
- [ ] **Bench**: `VW 0 500` — robot spins in place CCW. [HARDWARE]
- [ ] **Bench**: `VW 200 400` — robot drives a curved arc leftward. [HARDWARE]
- [ ] **Bench**: Start `VW 200 0`, wait `sTimeoutMs` + 50 ms without resending
  — robot stops, `EVT safety_stop` emitted. [HARDWARE]
- [ ] **Bench**: `VW 200 0 #9` watchdog fires → `EVT safety_stop #9`. [HARDWARE]

### Config SET/GET (SUC-004)

- [ ] **Bench/unit**: `SET aMax=150 aDecel=100` then `G 400 0 200` — accel
  ramp noticeably slower than with default 300/250 values. [HARDWARE — qualitative]
- [ ] **Bench/unit**: `SET arriveTol=20` — robot stops 20 mm from target
  (larger tolerance); `SET arriveTol=3` — stops 3 mm from target. [HARDWARE]
- [ ] **Bench/unit**: `GET aMax aDecel turnGate arriveTol` returns current
  values in CFG line. [HARDWARE]

### Regression

- [ ] All firmware unit tests pass on the sprint branch before flashing. Run
  `uv run pytest` (or project equivalent) and confirm green. [CI]
- [ ] `S 200 150` still works (streaming drive not broken by VW addition). [HARDWARE]
- [ ] `T 200 200 1000` still works (timed drive unchanged). [HARDWARE]
- [ ] `D 200 200 300` still works (distance drive unchanged). [HARDWARE]

## Implementation Plan

### Approach

No code. Flash latest firmware from sprint branch (after tickets 001–005
merged). Run each test case in order. Document pass/fail results in a bench
session note (free-form, not a formal artifact).

### Files to modify

None.

### Testing plan

Hardware bench run per the acceptance criteria above. Any failure: note the
failing criterion, open a bug ticket against the relevant earlier ticket
(002/003/004/005), and uncheck that criterion until the bug is fixed and
re-verified.

### Documentation updates

None — documentation was completed in ticket 005 (protocol-v2.md updates).
