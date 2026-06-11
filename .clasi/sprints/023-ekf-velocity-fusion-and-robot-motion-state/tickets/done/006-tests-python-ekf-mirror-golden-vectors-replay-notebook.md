---
id: '006'
title: Tests (Python EKF mirror 5-state, golden vectors, replay harness, notebook)
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-004
- SUC-005
- SUC-007
depends-on:
- '001'
- '005'
issue: ekf-velocity-fusion-and-robot-state.md
---

# T006: Tests (Python EKF mirror 5-state, golden vectors, replay harness, notebook)

## Description

Extend `tests/dev/test_ekf.py` with a full 5-state Python EKF mirror, new test
classes covering velocity predict/update, Mahalanobis gating, and the setPose
re-baseline regression. Add golden-vector tests for Python/C++ parity. Implement
an offline replay harness (`replay_tlm_log()`). Create the demonstration notebook
`host_tests/demo_ekf_velocity_fusion.ipynb`.

This is the validation layer for the entire sprint — it confirms the firmware EKF
math (via Python mirror parity) and demonstrates the improvement visually.

## Acceptance Criteria

**Python EKF mirror (5-state):**
- [x] The `EKF` class in `test_ekf.py` is extended to match the T001 5-state API:
  - `init(q_xy, q_theta, q_v, q_omega, r_otos_xy, r_otos_v, r_enc_v)`
  - `predict(dCenter, dTheta, theta_before, dt_s)` — position block unchanged;
    velocity block random-walk; full 5x5 P update.
  - `update_position(x_otos, y_otos)` — renamed from `update()`; Mahalanobis gate.
  - `update_velocity(v_meas, omega_meas, r_v, r_omega)` — sequential scalar updates
    with Mahalanobis gates.
  - `v`, `omega` properties.
  - `rejected_count` property (counter incremented on gated updates).
- [x] All existing sprint-022 test classes (`TestPredictStraight`, `TestPredictTurn`,
  `TestHeadingWrap`, `TestUpdate`, `TestConvergence`, `TestNoDriftWithoutUpdate`,
  `TestSetPose`) pass without modification to their test logic (update only the
  EKF method names if needed).

**`TestPredictVelocity`:**
- [x] After a straight predict with `dCenter=100, dt_s=0.1`, `v` is near 1000 mm/s
  and `omega` is near 0.
- [x] P[3][3] grows by `q_v` after one predict from zero P (random-walk).
- [x] P[4][4] grows by `q_omega` after one predict from zero P.
- [x] P[0][3], P[0][4], P[1][3], P[1][4] remain 0 (block decoupling invariant).

**`TestUpdateVelocity`:**
- [x] With `v` state at 500 mm/s and measurement at 300 mm/s (within Mahalanobis
  threshold), state moves toward 300 mm/s after one `update_velocity()`.
- [x] P[3][3] decreases after a velocity update (uncertainty reduced).
- [x] Position states `x`, `y`, `theta` are NOT changed by `update_velocity()`.

**`TestMahalanobisGating`:**
- [x] `update_position` with a large innovation (e.g. 1000 mm from a 10 mm
  uncertainty filter) is rejected; `rejected_count` increments.
- [x] `update_position` with a small innovation (within 1-sigma) is accepted.
- [x] `update_velocity` with an outlier velocity is rejected; counter increments.
- [x] After a rejection, state and covariance are identical to pre-update values.

**`TestSetPoseRebaseline` (regression test for the setPose fix):**
- [x] Test scenario:
  1. Create an `Odometry`-equivalent structure in Python that tracks `_prevEncL`
     and `_prevEncR`.
  2. Simulate driving: set `encLMm = 500.0`, `encRMm = 495.0`.
  3. Call `set_pose(0, 0, 0)` with the old behaviour (zeros `_prev`) — confirm
     the next predict produces `dL = 500.0` (a spurious jump).
  4. Call `set_pose(0, 0, 0)` with the fix (`_prev = encLMm`) — confirm the next
     predict produces `dL ~= 0`.
- [x] Test is named `test_setpose_rebaselines_encoder_prev` and has a clear
  comment explaining the bug.

**Golden vectors (Python/C++ parity):**
- [x] A `TestGoldenVectors` class with at least two test cases:
  - **Golden vector 1:** From a known initial state, run `predict(dCenter=50.0,
    dTheta=0.1, theta_before=0.0, dt_s=0.05)`. Record the exact output state
    `[x, y, theta, v, omega]` and P diagonal `[P[0][0], P[1][1], P[2][2], P[3][3], P[4][4]]`
    from the Python implementation. The test asserts these values match to 6
    significant figures. The same values must also pass in the C++ unit test (the
    programmer adds a corresponding C++ test in a separate file or asserts via the
    Python/C++ parity pattern from sprint 022).
  - **Golden vector 2:** Same for `update_position(x=49.94, y=2.50)` after a
    predict step with non-zero P.
- [x] A comment above the golden vector tests documents: "These values are the
  source of truth. If the C++ EKF produces different values, the C++ is wrong."

**Replay harness:**
- [x] A `replay_tlm_log(log_path)` function in `tests/dev/ekf_replay.py` that:
  - Reads a newline-delimited TLM log file (each line is a raw TLM string from
    the firmware).
  - Parses each line using `parse_tlm()` from `host/robot_radio/robot/protocol.py`.
  - For each TLM frame, calls `ekf.predict()` with encoder deltas and dt, and
    `ekf.update_position()` with OTOS position when present.
  - Returns a list of `(t_ms, x, y, theta, v, omega, P_diag)` tuples.
  - Supports three modes via a flags argument: `encoder_only`, `otos_position`,
    `otos_velocity` (boolean flags). When a flag is False, the corresponding
    update is skipped.
- [x] A small synthetic TLM log file is committed to `tests/dev/fixtures/` for
  deterministic replay testing. The log contains: 20 TLM frames with straight-line
  encoder data, 10 frames with a 90-degree turn, 10 frames continuing after turn,
  1 camera fix (large pose jump), and 9 frames of encoder-only continuation.
- [x] A test `TestReplayHarness.test_replay_encoder_only_advances_x` confirms
  that the encoder-only replay with a straight-line log produces a monotonically
  increasing x estimate.

**Demonstration notebook (`host_tests/demo_ekf_velocity_fusion.ipynb`):**
- [x] Notebook header cell documents purpose, required data file, and usage.
- [x] Data cell loads the committed log file from `tests/dev/fixtures/`.
- [x] Replay cell runs `replay_tlm_log()` in the three configurations.
- [x] Plot 1: x-y trajectory overlay (encoder-only, +OTOS position, +OTOS velocity).
- [x] Plot 2: position error vs time for each configuration.
- [x] Plot 3: fused v and omega vs encoder-rate and OTOS raw velocity.
- [x] Plot 4: P[0][0] (position uncertainty) and P[3][3] (velocity uncertainty) vs
  time, showing covariance shrinking on updates and growing between.
- [x] Plot 5: theta error vs time — illustrating heading drift between camera fixes
  and correction at each fix.
- [x] All five plots use `matplotlib`; notebook runs `Run All` without exceptions.
- [x] A note cell below Plot 5 explicitly documents the heading-axis weakness:
  "theta is corrected only indirectly via position cross-covariance and by camera
  resets; its absolute drift is fully removed only by a camera fix (SI command).
  OTOS heading fusion remains future work."

**Full test suite:**
- [x] `uv run --with pytest python -m pytest -v` passes with all new and existing
  tests green.

## Implementation Plan

### Approach

Work in this order:

**Step 1: Extend the Python EKF mirror class** to 5-state. The position block
of `predict()` is unchanged; add the two-element velocity block with random-walk.
The `update()` method is renamed `update_position()` and gains Mahalanobis gating.
The new `update_velocity()` is two sequential scalar updates.

**Step 2: Update existing test classes** to use the new `init()` signature (add
`q_v`, `q_omega`, `r_otos_v`, `r_enc_v` args to each `_make_ekf()` factory). The
`TestUpdate` class method `update()` becomes `update_position()`. All existing
tests should pass without logic changes.

**Step 3: Add new test classes** for velocity, gating, and rebaseline.

**Step 4: Add golden vector tests.** Run the Python EKF by hand (or compute
analytically) to get the expected values, then hard-code them in the test. The
key is that the Python mirror and the C++ EKF MUST produce the same output for
the same inputs — if they diverge, there is a transcription bug.

**Step 5: Implement `replay_tlm_log()`** and the synthetic fixture log.

**Step 6: Write the notebook.** Import `replay_tlm_log` from the test module (or
a shared replay module). Use the committed fixture log. Plot with `matplotlib`.

**Mahalanobis gating implementation in Python mirror:**

For `update_position()`:
```python
d2 = yi0*(si00*yi0 + si01*yi1) + yi1*(si10*yi0 + si11*yi1)
if d2 > 5.99:
    self._rejected += 1
    return
```

For `update_velocity()` (per channel):
```python
s = self._P[j][j] + r
if s < 1e-9:
    return  # degenerate
inn = meas - self._x[j]
d2 = inn*inn / s
if d2 > 3.84:
    self._rejected += 1
    continue  # skip this channel, try next
```

**Synthetic fixture log format:**

Each line is a raw TLM string as emitted by the firmware:
```
TLM t=1000 mode=S enc=50,49 pose=50,0,0
TLM t=1100 mode=S enc=100,99 pose=100,0,0
...
TLM t=1500 mode=S enc=250,248 pose=250,0,0 twist=250,0
```
The log should include: straight-line motion, a 90-degree turn, and two camera
fixes (large pose jumps with enc non-zero to demonstrate the setPose fix).

### Files to create

- `tests/dev/fixtures/tlm_log_sample.txt` — synthetic TLM log
- `tests/dev/ekf_replay.py` — replay harness module (or add directly to test_ekf.py)
- `host_tests/demo_ekf_velocity_fusion.ipynb` — demonstration notebook

### Files to modify

- `tests/dev/test_ekf.py` — extend Python EKF mirror; add test classes

### Testing plan

```
uv run --with pytest python -m pytest tests/dev/test_ekf.py -v
uv run --with pytest python -m pytest -v
```

The notebook is verified by running all cells; no separate test runner needed
for notebooks. CI can optionally run `jupyter nbconvert --to notebook --execute`
to verify.

### Documentation updates

Update the module docstring in `test_ekf.py` to reference sprint 023 and the
5-state model. Add an inline comment in the Python EKF class explaining the
block-decoupled Jacobian invariant.
