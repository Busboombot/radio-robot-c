---
status: draft
---

# Sprint 023 Use Cases

## SUC-001: EKF 5-State Predict Propagates Position and Velocity Together

- **Actor**: Firmware odometry task (called every control tick via `Odometry::predict()`)
- **Preconditions**: EKF is initialised with the 5-state `initEKF()`; encoder
  deltas `dCenter`, `dTheta`, and elapsed `dt` are available.
- **Main Flow**:
  1. `Odometry::predict()` computes `dCenter`, `dTheta`, `dt`, and `theta_before`
     from the encoder snapshot.
  2. It calls `_ekf.predict(dCenter, dTheta, theta_before, dt)`.
  3. The EKF propagates the position block `[x,y,theta]` with the arc-segment
     motion model (unchanged from sprint 022).
  4. The EKF propagates `v` and `omega` as random-walk states with process noise
     `ekfQv` and `ekfQomega`. v and omega do NOT feed into x,y in the Jacobian.
  5. Covariance P (5x5) grows by Q.
  6. `predict()` writes the full 5-state EKF output back into `HardwareState`:
     `poseX/Y/Hrad`, `fusedV`, `fusedOmega`.
- **Postconditions**: `HardwareState` pose and velocity fields reflect the EKF
  prediction; P has grown by Q.
- **Acceptance Criteria**:
  - [ ] After a straight 100 mm predict step, `x` advances by ~100 mm; `v` is
        consistent with dCenter/dt; `omega` is near 0.
  - [ ] After a 90-degree turn predict, `theta` is near ±pi/2 and `omega` reflects
        the encoded dTheta/dt.
  - [ ] P[0][0], P[3][3], P[4][4] all grow by their respective Q values after one
        predict from zero P.
  - [ ] v/omega changes do NOT appear in P[0][2]/P[1][2] cross-terms (block
        decoupling — velocity block does not couple into position block).

---

## SUC-002: EKF Fuses Two Velocity Measurements (Encoder-Rate and OTOS)

- **Actor**: Firmware OTOS correction task (`Robot::otosCorrect()` at ~100 ms cadence)
- **Preconditions**: OTOS sensor has been read for the current tick; encoder-derived
  instantaneous speed `dCenter/dt` and `dTheta/dt` are available.
- **Main Flow**:
  1. `Robot::otosCorrect()` reads OTOS position via `otos.readTransformed(config)`
     and OTOS velocity via `otos.readVelocityTransformed(config)`.
  2. It calls `odometry.correctEKF()` with position, OTOS velocity, and
     encoder-derived velocity measurements.
  3. `Odometry::correctEKF()` calls the EKF update for the OTOS position
     channel (`x_otos`, `y_otos`) with Mahalanobis gating.
  4. It calls the EKF update for the OTOS velocity channel (`v_otos`, `omega_otos`)
     with Mahalanobis gating using `ekfROtosV`.
  5. It calls the EKF update for the encoder velocity channel (`v_enc`, `omega_enc`)
     with Mahalanobis gating using `ekfREncV`.
  6. The fused `v` and `omega` are written back into `HardwareState.fusedV` and
     `HardwareState.fusedOmega`.
- **Postconditions**: `fusedV` and `fusedOmega` reflect the Kalman-fused estimate;
  P[3][3] and P[4][4] shrink on accepted updates; outlier updates are rejected.
- **Acceptance Criteria**:
  - [ ] With both encoder and OTOS velocity consistent, both update channels are
        accepted and P[3][3]/P[4][4] decrease.
  - [ ] A velocity measurement grossly inconsistent with the predicted state
        (innovation beyond Mahalanobis threshold) is rejected without crashing.
  - [ ] `fusedV` is closer to the true value than either raw measurement alone
        after 10 predict+update cycles.

---

## SUC-003: OTOS Velocity and Acceleration Read in Firmware

- **Actor**: Firmware HAL layer (`OtosSensor::readVelocityTransformed()`,
  `OtosSensor::readAccelTransformed()`)
- **Preconditions**: OTOS sensor is initialised (`is_initialized() == true`);
  signal processing is running.
- **Main Flow**:
  1. Caller invokes `otos.readVelocityTransformed(config)`.
  2. `OtosSensor` reads 6 bytes from `REG_VELOCITY_XL (0x26)` as three int16
     values using the existing `readXYH()` helper.
  3. The raw LSB values are converted to mm/s and rad/s using the same scale
     factors as `readTransformed()`.
  4. The mounting-offset rotation and upside-down flip from `config` are applied.
  5. Body-frame linear speed `v` (mm/s) and yaw rate `omega` (rad/s) are derived
     and returned.
  6. Similarly, `readAccelTransformed(config)` reads `REG_ACCELERATION_XL (0x2C)`
     and returns body-frame `ax`, `ay` in mm/s^2.
- **Postconditions**: Caller has transformed velocity and acceleration in the same
  frame as `readTransformed()` position.
- **Acceptance Criteria**:
  - [ ] `readVelocityTransformed()` returns zero-values when not initialized.
  - [ ] `readAccelTransformed()` returns zero-values when not initialized.
  - [ ] `IOtosSensor` interface declares both methods as pure virtual.
  - [ ] `OtosSensor` concrete class implements both methods.

---

## SUC-004: Mahalanobis Gating Replaces Fixed-Distance Outlier Rejection

- **Actor**: Firmware EKF update path (all three update channels)
- **Preconditions**: Innovation covariance S is computed as part of the Kalman
  update; chi-square threshold constants are defined in `EKF.h`.
- **Main Flow**:
  1. Before applying a Kalman correction, compute the Mahalanobis distance
     `d2 = y^T * S_inv * y` where `y` is the innovation vector.
  2. Compare `d2` to the chi-square threshold for the measurement's DOF
     (2-DOF for position: 5.99; 1-DOF for each velocity: 3.84 at alpha=0.05).
  3. If `d2 > threshold`, increment the rejection counter and skip the update.
  4. If `d2 <= threshold`, apply the normal Kalman update.
- **Postconditions**: Outlier updates are rejected without corrupting state.
- **Acceptance Criteria**:
  - [ ] A small innovation (within 1-sigma) passes gating and updates state.
  - [ ] An innovation 10 times larger than the 1-sigma bound is rejected.
  - [ ] The rejection counter increments on each rejected update.

---

## SUC-005: setPose Re-Baselines Encoder Delta Reference

- **Actor**: Firmware `Odometry::setPose()` (called by the SI camera-fix handler)
- **Preconditions**: `SI` command has provided a new pose from the camera;
  `s.encLMm` / `s.encRMm` hold non-zero cumulative encoder values.
- **Main Flow**:
  1. Robot SI handler calls `odometry.setPose(s, x, y, h)`.
  2. `setPose()` writes `s.poseX`, `s.poseY`, `s.poseHrad`.
  3. `setPose()` re-baselines: `_prevEncL = s.encLMm; _prevEncR = s.encRMm`.
  4. On the next `predict()` tick, `dL = s.encLMm - _prevEncL ~= 0`.
- **Postconditions**: The first predict after a camera fix does NOT produce a
  spurious jump equal to the cumulative encoder count.
- **Acceptance Criteria**:
  - [ ] Regression test confirms: after non-zero encoder accumulation, setPose(),
        then one predict(), position changes by less than 1 mm.
  - [ ] The pre-fix behaviour (zeroing _prevEncL) is confirmed to produce the bug
        in the test before the fix.

---

## SUC-006: Host RobotState Carries Unified Motion State from Telemetry

- **Actor**: Host Python client (`NezhaState._process_line()`)
- **Preconditions**: Firmware is streaming TLM frames with `pose=` and `twist=`
  fields; `parse_tlm()` is updated to parse `twist=`.
- **Main Flow**:
  1. TLM line: `TLM t=12345 mode=S pose=350,-12,1780 twist=210,150`.
  2. `parse_tlm()` populates `TLMFrame.pose = (350, -12, 1780)` and
     `TLMFrame.twist = (210, 150)` (v_mmps, omega_mradps).
  3. `NezhaState._process_line()` builds a `Pose` from `tlm.pose` and a
     `RobotState(pose=pose, v=v_mps, omega=omega_rads, accel=None, stamp=now)`.
  4. `NezhaState.robot_state` is updated under the lock.
- **Postconditions**: `state.robot_state.v` and `.omega` reflect latest estimates.
- **Acceptance Criteria**:
  - [ ] `parse_tlm("TLM t=1000 pose=100,200,1800 twist=300,500")` returns a
        `TLMFrame` with `twist = (300, 500)`.
  - [ ] `TLMFrame.twist` is None for frames without `twist=`.
  - [ ] `RobotState` has `pose: Pose`, `v: float`, `omega: float`,
        `accel: tuple | None`, `stamp: float`.
  - [ ] `NezhaState.robot_state` is None initially; set on first frame with
        `pose=` and `twist=`.

---

## SUC-007: Fused Velocity and Trajectory Visualised in Demonstration Notebook

- **Actor**: Developer running `host_tests/demo_ekf_velocity_fusion.ipynb`
- **Preconditions**: A captured TLM log exists; Python EKF 5-state mirror is
  importable from the project.
- **Main Flow**:
  1. Notebook loads a TLM log (encoder + OTOS + camera fixes).
  2. Replays the log through the Python EKF in three configurations (encoder-only,
     +OTOS, +OTOS+camera) and records trajectory/covariance/velocity.
  3. Renders plots: trajectory overlay, position error vs time, fused v/omega vs
     raw, covariance diagonals vs time, heading drift between camera fixes.
- **Postconditions**: All cells execute without errors; plots rendered inline.
- **Acceptance Criteria**:
  - [ ] Notebook runs without exceptions on the committed log file.
  - [ ] Plot 1 shows visibly better tracking with each additional sensor.
  - [ ] Plot 3 shows fused v/omega following raw measurements but with less noise.
  - [ ] Plot 4 shows covariance shrinking after OTOS updates and growing between.
  - [ ] Plot 5 shows heading drift accumulating between camera fixes.
