---
status: pending
---

# Sim must run the real dispatch path (close the sim/real split)

## Context

This is the single largest reason "it works in sim and fails on the field," and the
direct cause of the repeated "go run the actual simulator on our code" frustration.
The simulator exercises a *different system* than hardware:

- `host_tests/sim_api.cpp` **never wires a `CommandQueue`**, so S/T/D/G/TURN
  dispatch through the *direct* `begin*()` path; on hardware they go
  converter → queue → `handleVW` (different code, different replies, different
  timing). This is why the double-OK (D11) and keepalive-stomp (D6) can't reproduce
  in sim.
- `sim_api.cpp` **hand-mirrors** the `LoopScheduler` loop with a "MUST mirror
  LoopScheduler.cpp exactly" comment — a divergence generator by construction.
- (Sensor-fidelity defaults — fusion off, `MockMotor` slip = 0 — are covered by the
  field-profile harness issue; this issue is about the **dispatch path** and **loop
  body**.)

## Fix (improvement-plan P1.3)

1. `sim_api.cpp`: instantiate a `CommandQueue`, call `cmd.setQueue(&q)` and
   `robot.motionController.setQueue(&q)`, and drain it in `sim_tick()` via
   `cmd.dequeueOne(q)` — the same calls `run_blocks()` makes. Keep the direct
   `begin*()` fallbacks only for unit tests that target them explicitly.
2. Extract the body of `run_blocks()` into `LoopScheduler::tickOnce(now)` that both
   the firmware loop and `sim_tick()` call; delete the hand-mirrored copy. Add a
   CI grep-lint for the words "MUST mirror" so the divergence can't reappear.

## Acceptance

- `test_vw_converters.py` passes against the queue path; the D11 double-OK test runs
  **in sim** and would have caught it. No hand-mirrored loop body remains; the
  "MUST mirror" comment is gone and lint-guarded.

## Source
Improvement-plan **P1.3** (and §4 sim/real-split analysis) in the 2026-06-11 review.
Unblocks reproducible testing of D6 and D11. Note: `source/main.cpp` and
`tests/bench/square_run.py` currently have uncommitted local changes — reconcile
before refactoring the loop.
