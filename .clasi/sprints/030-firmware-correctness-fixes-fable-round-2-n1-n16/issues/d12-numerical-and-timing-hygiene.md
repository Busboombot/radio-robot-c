---
status: in-progress
sprint: '030'
tickets:
- 030-001
---

# D12 — Numerical / timing hygiene (assorted, lower severity)

## Context

A cluster of smaller correctness/maintainability issues flagged for later. None is
an active motion runaway, but each erodes trust in the estimator or adds hidden
latency:

1. **EKF process noise is loop-rate-coupled.** `Odometry::predict()` runs every loop
   iteration with no period gate; EKF Q is added per *call*, not per second, so
   process-noise tuning silently depends on loop frequency.
2. **Dispatch latency.** `dequeueOne()` dispatches one command per ~10 ms tick; a
   burst of N commands takes N ticks, and a converter's target is computed from the
   pose at *handler* time, not arrival time.
3. **EVT truncation.** `MotionController::emitEvt` / `MotionCommand::emitEvt`
   truncate at 48 bytes.
4. **Reset ordering fragility (folds in the 2026-06-08 finding).** `Robot::distanceDrive`
   zeroes `encLMm/R` *after* `beginDistance` captured baselines — a fragile ordering
   contract, already bit once (documented in comments). More broadly: encoder/pose
   reset should be one atomic robot-level operation that syncs hardware accumulators,
   `MotorController` velocity baselines, `HardwareState` encoder fields, and
   `Odometry` previous-encoder snapshots; `Odometry::setPose()` should snapshot the
   *current* encoder inputs rather than assuming zero.

## Fix (improvement-plan P2.3 + 2026-06-08 reset-desync finding)

- Gate `predict()` to `controlPeriodMs` (or scale Q by dt) so Q is per-second.
- Decide whether burst dispatch latency needs draining > 1/tick; document the
  contract either way.
- Widen or bound-check the EVT buffer.
- Create one atomic reset op; fix `setPose()` to snapshot current encoders.

## Acceptance

- Unit test: Q effect is invariant to loop rate (predict gated). Reset test: a
  `ZERO enc` / `ZERO pose` with nonzero encoders produces no pose jump on the next
  `predict()`. EVT labels are not truncated for the standard verbs.

## Source
Defect **D12** in the 2026-06-11 sim2real review; fix P2.3. Item 4 also absorbs the
"encoder and pose reset paths can desynchronize odometry" High finding from the
2026-06-08 `source-code-review-findings.md` (re-mapped from the deleted
`DriveController.cpp` to current files).
