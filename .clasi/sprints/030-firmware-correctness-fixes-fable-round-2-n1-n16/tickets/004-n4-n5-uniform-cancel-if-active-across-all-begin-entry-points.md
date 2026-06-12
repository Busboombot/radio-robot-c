---
id: '004'
title: 'N4+N5: Uniform cancel-if-active across all begin*() entry points'
status: open
use-cases:
  - SUC-004
depends-on:
  - '002'
github-issue: ''
issue: fr2-n4-n5-cancel-if-active.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# N4+N5: Uniform cancel-if-active across all begin*() entry points

## Description

Five of the seven `begin*()` entry points in `MotionController` cancel an active
command before starting the new one. Two do not:

N4: `beginStream()` (`MotionController.cpp:148-172`) and `beginRawVelocity()` skip
the cancel. An `S` issued while TURN/G/T/D is active (queue path routes it through
`handleVW` stream=1 branch to `beginStream`): seeds the BVC mid-motion (instant
velocity jump — the "fast spin signature"), leaves `_activeCmd` running so its stop
conditions keep evaluating against the new stream (when the old command's TIME/HEADING
fires, `driveAdvance` soft-stops the robot and emits a stale `EVT done`), and the
old command never gets `EVT cancelled`. P1.1's own verify scenario ("start TURN,
inject S 0 0 mid-turn — TURN must complete") fails on this code.

N5: `beginTimed()` (`MotionController.cpp:257`) and `beginDistance()` (`:294`) go
straight to `configure()`, silently resetting the previous command's reply sink.
A host awaiting `EVT done G` that issues a `T` never gets any terminal event for the G.

Depends on ticket 002 because after the queue re-wire the queue path is used from
first boot, making these paths consistently exercised.

## Acceptance Criteria

- [ ] `beginStream()` cancels any active command (emits `EVT cancelled` for its
      corrId) before seeding the BVC.
- [ ] `beginRawVelocity()` cancels any active command before proceeding.
- [ ] `beginTimed()` cancels any active command before calling `configure()`.
- [ ] `beginDistance()` cancels any active command before calling `configure()`
      (note: this is in addition to the `resetEncoders()` call from ticket 001).
- [ ] New sim regression test (P1.1 verify scenario): start TURN, inject `S 0 0`
      mid-turn on the queue path — TURN completes at the commanded heading; no BVC
      jump; `EVT cancelled` is emitted for the TURN's corrId.
- [ ] New sim test: G preempted by T — host receives `EVT cancelled` for the G's
      corrId before any T-related event.
- [ ] No regression in existing motion preemption tests.
- [ ] `python3 build.py` clean build passes.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.

## Implementation Plan

### Approach

Each of the four deficient `begin*()` methods gains the same three-line pattern
used by the other five entry points. The pattern is: check `_activeCmd != nullptr`,
emit `EVT cancelled` for its corrId, then clear it. The programmer should confirm
the exact idiom by reading any of the currently-compliant begin*() methods in
`MotionController.cpp`.

Additionally, decide whether `S` (stream) should be *rejected* while a self-
terminating command is running, or should cancel it. The current plan is to cancel
(same as all other begin*()). Document the chosen contract in a short comment.

### Files to modify

- `source/control/MotionController.cpp`
  - `beginStream()` (:148-172): add cancel-if-active prefix.
  - `beginRawVelocity()`: add cancel-if-active prefix.
  - `beginTimed()` (:257): add cancel-if-active prefix.
  - `beginDistance()` (:294): add cancel-if-active prefix (in addition to the
    `resetEncoders()` call already added by ticket 001).
- `host_tests/` or `host/tests/` — add:
  - `test_s_mid_turn_cancels_turn` (P1.1 verify scenario).
  - `test_g_preempted_by_t_emits_cancelled`.

### Testing plan

Run: `uv run --with pytest python -m pytest host_tests/ host/tests/ -v`

Build: `python3 build.py` (clean).

### Notes

- Depends on ticket 002 (queue re-wire) because these paths are exercised via the
  queue path.
- `beginDistance()` already receives the `resetEncoders()` call from ticket 001;
  the cancel-if-active prefix goes *before* the configure/reset sequence (cancel
  the old command first, then reset encoders for the new D).
