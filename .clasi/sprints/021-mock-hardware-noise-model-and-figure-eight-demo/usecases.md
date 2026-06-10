---
status: draft
sprint: "021"
---

# Sprint 021 Use Cases

## SUC-001: Configure Encoder Noise and Slip Parameters at Runtime

- **Actor**: Test author or notebook cell
- **Preconditions**: SimConnection is connected to libfirmware_host. No prior noise
  configuration has been applied.
- **Main Flow**:
  1. Caller invokes `conn.set_slip(straight=0.005, turn_extra=0.03)` to set a
     baseline under-report fraction plus a turn-rate-coupled extra fraction.
  2. Caller invokes `conn.set_encoder_noise(sigma_mm=0.05)` to add per-tick
     Gaussian noise to the encoder accumulator.
  3. Caller drives the robot in a straight line.
  4. `conn.get_state()["pose_x"]` diverges from `conn.get_exact_pose()["x"]` by
     at least 1 mm over 1 m of travel.
  5. Caller repeats with a point-turn; divergence is larger than in the straight case.
- **Postconditions**: Each tick thereafter, MockMotor applies slip and noise before
  adding to its encoder accumulator. ExactPoseTracker accumulates the pre-slip velocity
  and remains the oracle ground truth.
- **Acceptance Criteria**:
  - [ ] `conn.set_slip()` and `conn.set_encoder_noise()` are callable after `connect()`.
  - [ ] Encoder dead-reckoning diverges from ExactPoseTracker after 1 m straight drive.
  - [ ] Divergence is larger after a full point-turn than after straight driving.
  - [ ] Existing pytest suite passes unchanged (noise off by default).

---

## SUC-002: Query Oracle Ground-Truth Pose (ExactPoseTracker)

- **Actor**: Notebook experiment cell or test
- **Preconditions**: SimConnection is connected. Robot is driving with slip or noise
  enabled.
- **Main Flow**:
  1. Caller invokes `conn.get_exact_pose()`.
  2. Returns `{"x": float, "y": float, "h": float}` in mm and radians.
  3. Values reflect the pre-slip true position accumulated from motor commands via
     midpoint integration.
- **Postconditions**: The caller has an oracle reference it can compare against noisy
  dead-reckoning (`pose_x/y/h`) and noisy OTOS pose.
- **Acceptance Criteria**:
  - [ ] `get_exact_pose()` returns a dict with `x`, `y`, `h` keys.
  - [ ] With no slip/noise, `get_exact_pose()` matches `get_state()["pose_x/y/h"]`
    within float precision.
  - [ ] With slip enabled, `get_exact_pose()` and `get_state()["pose_x"]` diverge over
    distance.

---

## SUC-003: Enable OTOS Sim Integration Model with Independent Drift

- **Actor**: Test author or notebook cell
- **Preconditions**: SimConnection is connected.
- **Main Flow**:
  1. Caller invokes `conn.enable_otos_model()` to switch MockOtosSensor from
     injection mode to integration mode.
  2. Caller optionally invokes `conn.set_otos_noise(linear=0.01, yaw=0.025)`.
  3. Robot drives; OTOS pose accumulates independently via `tick()` with its own
     Gaussian linear and yaw noise.
  4. Caller reads OTOS pose via `conn.get_otos_pose()`.
  5. OTOS pose drifts differently from encoder dead-reckoning.
- **Postconditions**: `get_otos_pose()` returns a noisy pose that drifts independently
  from encoder-based odometry. The `setInjectedPose()` injection path still works when
  the model is disabled.
- **Acceptance Criteria**:
  - [ ] `enable_otos_model()` switches MockOtosSensor to integration mode.
  - [ ] `get_otos_pose()` returns `{"x", "y", "h"}` diverging from `get_exact_pose()`
    after a multi-metre drive.
  - [ ] OTOS drift is independent of encoder drift (different noise realisations).
  - [ ] Disabling the model (`setInjectedPose` API) still works for existing tests.

---

## SUC-004: Drive a Figure-Eight Path with Pure Pursuit in Simulation

- **Actor**: Notebook user executing demo_figure_eight.ipynb
- **Preconditions**: libfirmware_host is built. Notebook has been opened in Jupyter.
  Slip and OTOS noise model are enabled via `make_sim()`.
- **Main Flow**:
  1. Notebook generates a catmull-rom figure-eight path through 9 control points
     (~600 mm span).
  2. Pure Pursuit (`pure_pursuit_vw`) computes `(v_mms, omega_mrads)` from the
     robot's current estimated pose and the path look-ahead point.
  3. Experiment 1 (dead reckoning): the firmware's noisy encoder odometry drives the
     pursuit controller. Ground truth from ExactPoseTracker is logged separately.
  4. Notebook collects per-tick truth, estimated, and path reference arrays.
  5. Experiment 1 trajectory visibly drifts from the reference path.
- **Postconditions**: Cell 4 completes without error and produces a trajectory plot
  showing encoder drift.
- **Acceptance Criteria**:
  - [ ] Figure-eight path is generated and plotted (Cell 2).
  - [ ] `pure_pursuit_vw` returns `(v_mms, omega_mrads)` for any pose and path.
  - [ ] Experiment 1 trajectory diverges visibly from reference over two loops.
  - [ ] Cell executes to completion without hardware.

---

## SUC-005: Compare Dead Reckoning, OTOS+Camera, and EKF Fusion Accuracy

- **Actor**: Notebook user executing demo_figure_eight.ipynb Experiments 2 and 3
- **Preconditions**: Experiment 1 complete. Slip and OTOS noise model enabled.
- **Main Flow**:
  1. Experiment 2: OTOS pose is used as the position estimate each cycle. Every 30
     cycles, a camera fix (exact pose with 5-cycle delay) hard-resets the OTOS
     accumulator.
  2. Experiment 3: A 3-state EKF predicts from VW control input each cycle and updates
     from OTOS each cycle. The same delayed camera fix triggers an update step.
  3. Cell 7 plots all three estimated trajectories against the reference path and
     computes RMS cross-track error for each.
  4. EKF (Experiment 3) has lower RMS cross-track error than dead reckoning
     (Experiment 1).
- **Postconditions**: All experiment cells complete. Comparison plot is rendered.
  EKF curve is closer to reference than dead-reckoning curve.
- **Acceptance Criteria**:
  - [ ] Experiment 2 cell completes and shows periodic camera-fix correction kinks.
  - [ ] Experiment 3 EKF cell completes; `EKF.predict()` and `EKF.update()` are
    callable.
  - [ ] Cell 7 comparison plot shows all three trajectories + reference path.
  - [ ] RMS cross-track error: Exp3 (EKF) < Exp1 (dead reckoning).
  - [ ] Notebook executes end-to-end via
    `jupyter nbconvert --to notebook --execute host_tests/demo_figure_eight.ipynb`.
