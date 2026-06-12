---
id: '009'
title: "N11+N14+N15+N16: Correctness cleanup A — spurious events, corrId width, EKF Q scaling, sensor-stop validation"
status: open
use-cases:
  - SUC-009
depends-on:
  - '002'
github-issue: ''
issue: fr2-n11-16-cleanup.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# N11+N14+N15+N16: Correctness cleanup A — spurious events, corrId width, EKF Q scaling, sensor-stop validation

## Description

Four low-severity correctness issues grouped as a cleanup ticket:

**N11:** PURSUE backtrack re-gate cancels the PURSUE `MotionCommand` with HARD
(`MotionController.cpp:698`). `cancel()` emits `EVT cancelled #<corrId>` for the G
command's correlation id. A host treating `EVT cancelled` as terminal for that id
concludes the G failed, then later receives `EVT done G #<same id>`. Suppress the
EVT for this internal phase transition (`cancelQuiet()` or clear the sink before
cancel, as `_startPreRotate` already does for PRE_ROTATE).

**N14:** `ParsedCommand::corrId` is 8 bytes (`CommandTypes.h:158`) while the
tokenizer, `MotionCommand`, and `TargetState` all carry 16. A host using ms-timestamp
correlation ids (>7 digits) gets silently truncated ids on every queued reply and EVT.
Widen to 16 uniformly.

**N15:** `EKF::predict()` adds full `Q` per call ignoring `dt_s` (`EKF.cpp:149`).
`Odometry::predict()` runs every loop iteration; the real loop period swings ~10-25 ms
with I2C load, so effective process noise varies ~2.5x. Scale Q by `dt_s` so Q is
per-second (P2.3.1 from the improvement plan; also d12 #1).

**N16:** Invalid `sensor=` stop token on the queue path (T/D/TURN in
`MotionCommandHandlers.cpp:784-793`) causes parse failure to silently skip the stop —
the command runs without its sensor trigger after the host already got `OK`. On the
direct path, parse failure replies ERR and cancels. Move validation to the converter
(before replying OK) to match.

Depends on ticket 002 (queue re-wire) because N14 and N16 are only consistently
exercised on the queue path.

## Acceptance Criteria

- [ ] N11: PURSUE re-gate does not emit `EVT cancelled` for the G's corrId (sim test).
- [ ] N14: A 16-char corrId round-trips intact on the queue path (sim test verifying
      no truncation in queued reply and EVT).
- [ ] N15: EKF Q effect is invariant to loop rate — sim test with two different loop
      periods confirms equal Q accumulation per second.
- [ ] N16: Invalid `sensor=` token on the queue path returns `ERR` before `OK` (sim
      test); command does not start.
- [ ] `python3 build.py` clean build passes.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.

## Implementation Plan

### Files to modify

- `source/control/MotionController.cpp`
  - PURSUE re-gate (~:698): use `cancelQuiet()` or clear the sink before calling
    `cancel()` so no `EVT cancelled` is emitted for the G's corrId.
- `source/control/MotionController.h` (if needed)
  - Add `cancelQuiet()` to `MotionCommand` or equivalent, or expose a sink-clear.
- `source/app/CommandTypes.h`
  - `ParsedCommand::corrId`: widen from `char[8]` to `char[16]`.
- `source/ekf/EKF.cpp`
  - `predict()`: multiply `Q` by `dt_s` before adding to `P` (or gate predict to
    run only at `controlPeriodMs` intervals and pass `dt_s` explicitly).
- `source/app/MotionCommandHandlers.cpp`
  - T/D/TURN converter `sensor=` parse sites (:784-793 etc.): validate the sensor
    token before replying OK; return ERR and do not enqueue if invalid.
- `host_tests/` or `host/tests/` — add tests for N11, N14, N15, N16.

### Testing plan

Run: `uv run --with pytest python -m pytest host_tests/ host/tests/ -v`

Build: `python3 build.py` (clean).

### Notes

- `completes_issue: false` — issue `fr2-n11-16-cleanup.md` also covers N12 and N13,
  which are in ticket 010.
- Depends on ticket 002 (queue re-wire) for consistent queue-path test coverage.
- N14 corrId widening: `ParsedCommand` is stack-allocated per command dispatch — +8
  bytes per dispatch. No heap impact. Confirm no struct-size asserts exist.
- N15: if `dt_s` is not currently passed to `EKF::predict()`, add it as a parameter
  or read it from the same `controlPeriodMs` source used by the rest of the control
  tick. The simplest correct approach is to multiply Q by `dt_s` inside `predict()`.
