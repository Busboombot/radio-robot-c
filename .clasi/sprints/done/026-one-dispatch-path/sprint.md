---
id: '026'
title: One dispatch path
status: done
branch: sprint/026-one-dispatch-path
use-cases: []
issues:
- sim-runs-real-dispatch-path
- a2-protocol-out-of-control-layer
- d11-single-ok-per-command
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 026: One dispatch path

## Goals

Sim and hardware run the same code; one reply per command — by construction,
not by patch. After this sprint the simulator is a trustworthy proxy for
hardware behavior, and duplicate-OK / keepalive-stomp defects can be
reproduced and verified in sim.

## Problem

The firmware has two completely different dispatch paths:

- **Hardware:** command → CommandQueue → converter → handleVW → begin*().
  Two OK replies per command (D11). Keepalives mutate active commands (D6).
- **Sim:** `sim_api.cpp` never wires a CommandQueue — commands go directly to
  begin*(). One OK reply. D6/D11 simply don't exist.

Separately, the control layer (`source/control/`) depends upward on the
protocol/app layer: six .cpp files include `CommandProcessor.h`; converters,
reply formatting, and queue manipulation live inside `MotionController`. This
is the root cause of the double-dispatch and the reason sim can never be wired
identically to hardware while the architecture stays inverted (a2).

`MotionController.cpp` is 1953 lines and `Robot.cpp` is 1490 lines because of
this — every change spans unrelated concerns (a3, treated as review criteria
inside a2, not a separate task).

## Solution

Three coordinated items:

**sim-runs-real-dispatch-path (P1.3):** Wire a CommandQueue in `sim_api.cpp`
and drain it via `cmd.dequeueOne(q)` — the same call `run_blocks()` makes.
Extract the body of `run_blocks()` into `LoopScheduler::tickOnce(now)` that
both the firmware loop and `sim_tick()` call; delete the hand-mirrored copy.
Add a CI grep-lint for "MUST mirror".

**a2 — protocol out of control layer:** Move all command parsing/conversion
(the S/T/D/G/TURN/RT→VW converters) and reply/EVT formatting to `app/`.
Control exposes typed begin*/cancel/advance APIs and reports completion through
a narrow callback or event struct; `app/` turns those into `OK`/`EVT` lines.
After this, `control/` includes no `CommandProcessor.h`, `CommandQueue.h`, or
`Protocol.h` reply types.

**a3 as review criteria within a2:** Write A3's file-size and separation
targets (no single .cpp > ~600 lines on the motion/robot paths; telemetry
format changes touch one file; motion mode additions touch control/ only) into
a2's acceptance criteria and review checklist. A3 is not a separate scheduled
task; the a2 refactor should split the god objects as a natural consequence.

**d11 as acceptance gate:** After a2, one reply per command falls out by
construction. Keep the d11 test (`test_protocol_v2.py` — exactly one OK per
command on the queue path) as the acceptance gate confirming the fix. Do not
implement the `quiet=true` patch separately; it is superseded by a2's
structural fix.

## Success Criteria

- `grep -rl 'CommandProcessor.h\|CommandQueue.h' source/control/` returns
  nothing (a2 acceptance).
- D11 double-OK test passes **in sim** (sim-runs-real-dispatch-path + a2).
- No hand-mirrored loop body in `sim_api.cpp`; "MUST mirror" comment gone and
  lint-guarded.
- Full hardware smoke ritual passes after firmware flash.

## Scope

### In Scope

- `host_tests/sim_api.cpp` — wire CommandQueue, extract tickOnce().
- `source/control/MotionController.cpp/.h`, `LoopScheduler.cpp/.h`,
  `HaltController.cpp`, `PortController.cpp`, `ServoController.cpp`,
  `Odometry.cpp` — remove all protocol-layer dependencies.
- New or refactored `source/app/` dispatch/converter/reply layer.
- `LoopScheduler::tickOnce()` extraction.
- CI lint: grep "MUST mirror" fails build.
- d11 `quiet=true`-equivalent resolved as a structural consequence of a2.
- a3 file-size targets enforced as review criteria.

### Out of Scope

- D6 keepalive-stomp fix (sprint 027 — needs the single path first for
  reliable sim reproduction).
- D8 pursuit law hardening (sprint 027).
- D9 OTOS validity gating (sprint 027).
- Host calibration or navigation changes.

## Test Strategy

- `test_vw_converters.py` passes against the queue path in sim.
- D11 double-OK test: exactly one OK per converter command in sim.
- D6 cannot stomp a TURN in sim (add the test now; fix lands in sprint 027).
- Hardware smoke ritual after flash: TURN×4 closure, G square, SAFE on.
- Both exact-profile and field-profile CI gates (field-profile harness lands
  in sprint 027; set up the scaffolding here to enable it).

## Architecture Notes

This is the highest-risk sprint in the roadmap (large firmware diff, protocol
layering inversion). Sprint 025's trustworthy stream is a hard dependency: the
smoke ritual after this flash relies on correct EVT delivery.

The `sim-runs-real-dispatch-path` issue notes that `source/main.cpp` and
`tests/bench/square_run.py` have uncommitted local changes that must be
reconciled before refactoring the loop — confirm and land those first.

a3 is not treated as an independent deliverable. The sprint architect must
verify during architecture-review phase that the a2 refactor naturally produces
MotionController and Robot files within the ~600-line target. If the sizes
remain above that after a2, flag as a follow-on in the sprint retrospective.

The `field-024-full-speed-spin-unresolved` issue (anomaly: SNAP showed
`mode=IDLE` while the robot spun at full speed; bench program abandoned an
autonomous G without sending X) may be related to the dispatch split and the
double-OK stream pollution. Both are eliminated by this sprint's changes; the
spin-unresolved post-mortem should be revisited after this flash.

## Why Second

This is the biggest-risk sprint; sprint 025 gives it a trustworthy test
harness. Everything in sprint 027 needs the single path to be testable — fixing
behavioral defects before the dispatch path is unified means writing tests
against code that is about to be deleted.

## Sizing

Large — approximately 2–3 focused sessions, highest risk in the roadmap.

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
| 001 | Extract tickOnce and wire CommandQueue in sim | — |
| 002 | Move motion command handlers and reply formatting to app layer | 026-001 |
| 003 | Add D11 double-OK gate test in sim | 026-001, 026-002 |
| 004 | CI grep-lint and hardware smoke ritual | 026-001, 026-002, 026-003 |

Tickets execute serially in the order listed.
