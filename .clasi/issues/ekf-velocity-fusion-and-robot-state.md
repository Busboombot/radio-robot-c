---
status: pending
---

# Extend the firmware EKF to fuse velocity and expose a unified robot motion state

## Context

The robot needs a single, coherent motion state — position, velocity, and
acceleration — rather than the position-only pose it maintains today. This is
the natural follow-on to sprint 022 (EKF pose fusion in firmware).

### Current state
- The firmware EKF (`source/control/EKF.{h,cpp}`) is a **3-state** filter:
  `x = [x_mm, y_mm, theta_rad]`. `predict()` is encoder arc-segment driven;
  `update()` folds OTOS x,y only (heading not fused). After each step the result
  is written back into `HardwareState.poseX/poseY/poseHrad`
  (`source/control/RobotState.h:46-50`), the single source of truth for
  telemetry and control.
- Velocity exists only as raw per-wheel `velLMms`/`velRMms`
  (`RobotState.h:42-44`) — there is **no body-frame velocity** and **no
  acceleration** as state.
- Host: the `Pose` dataclass (`host/robot_radio/nav/pose.py`) is frozen,
  position+heading only. `NezhaState` (`host/robot_radio/robot/nezha_state.py`)
  mirrors raw telemetry (`encoders`, `otos_pose`, `heading_rad`, `dt_s`) with no
  velocity/accel. `NezhaKinematic` computes velocity via WPILib but it's
  scattered across properties.
- A Python EKF mirror exists for tests (`tests/dev/test_ekf.py`) and must stay
  in lockstep (sprint 022 pattern).

## Goal / decided design

1. **Extend the firmware EKF from 3-state to a 5-state CTRV model**
   `[x, y, theta, v, omega]` — `v` = linear speed (mm/s), `omega` = yaw rate
   (rad/s). Covariance grows to 5×5. Stays float-only / no heap / no STL / fully
   unrolled.
   - Keep the **encoder arc-segment as the position predict** (most trustworthy
     distance signal); `v`,`omega` carried as random-walk with process noise.
   - **Fuse velocity from two measurement sources:** encoder-derived
     instantaneous speed (`dCenter/dt`, `dTheta/dt`) and OTOS native velocity
     (new read path — see Notes). The OTOS position update on x,y stays.
   - `predict()` now needs `dt`.

2. **Acceleration: passthrough, not fused.** Carried as a raw OTOS-measured
   field, not part of the EKF state vector. The OTOS measures acceleration
   directly. Can be promoted into the filter later if a consumer needs it.

   **Velocity-block coupling:** position is propagated from the encoder delta
   (not from `v·dt`), so `v`/`omega` do **not** couple into `x`/`y` in the
   predict Jacobian. The filter is effectively two blocks — `[x,y,theta]`
   (encoder predict + OTOS position) and `[v,omega]` (random-walk smoothed by
   the encoder-rate and OTOS-velocity measurements). This is a deliberate v1
   simplification that preserves the strong encoder-distance signal; coupling
   `v`/`omega` back into position is a possible future enhancement.

3. **New host `RobotState` composite dataclass:** `{ pose: Pose, v, omega,
   accel | None, stamp }`. It **contains** a `Pose`; `Pose` stays pure
   position+heading (matches the WPILib `Pose2d` / `ChassisSpeeds` split). Built
   in `NezhaState` when the new telemetry arrives; replaces the loose
   `heading_rad`/`dt_s` scatter.

4. **Telemetry:** add a TLM field (e.g. `twist=v_mmps,omega_mradps`) built next
   to the existing `pose=` in `buildTlmFrame`; host `TLMFrame`/`parse_tlm`
   consume it.

5. **New EKF tuning params in `Config.h`** (`qV`, `qOmega`, `rOtosV`, `rEncV`)
   alongside the existing `q_xy`/`q_theta`/`r_otos_xy`.

6. **Keep the Python EKF mirror** (`tests/dev/test_ekf.py`) in lockstep with
   unit tests.

7. **Encoders feed the filter as relative motion only — re-baseline on pose
   set.** The encoder is the motion model (`predict` consumes per-tick deltas
   `dCenter`/`dTheta`), never an absolute position; its accumulated drift never
   enters the fused estimate, and OTOS/camera corrections continuously remove
   the uncorrected portion. **No periodic/time-based encoder reset is needed.**
   The one reset that matters: on every pose set (camera fix via `SI` →
   `setPose`), the odometry delta reference must be **re-baselined to the current
   encoder reading** so the next delta is ~0 (see the `setPose` bug in Notes).

## Testing

Weighted toward offline + a graphical demonstration.

1. **Offline unit tests** — extend the Python EKF mirror (`tests/dev/test_ekf.py`)
   to the 5-state model; deterministic `predict`/`update` assertions against
   hand-computed values; Python↔C++ parity via golden vectors (sprint-022
   pattern). The encoder re-baseline / `setPose` fix gets a regression test
   (no spurious jump after a pose set with non-zero encoder accumulators).
2. **Offline replay harness** — capture real runs (encoder deltas + OTOS +
   camera fixes) from the live TLM stream to a log, then replay them through the
   Python EKF deterministically — no robot required. Makes the demo
   reproducible and lets us A/B fusion configurations.
3. **Demonstration notebook** (`host_tests/`) — the graphical view:
   - Overlay **encoder-only dead-reckoning** vs **+OTOS fused** vs **+OTOS
     +camera-reset** trajectories against the camera ground-truth path.
   - Drift / position-error vs time for each configuration.
   - Fused `v`/`omega` estimates vs raw encoder- and OTOS-derived velocity.
   - EKF covariance (uncertainty) shrinking on each update / growing between.
   - Toggling each sensor on/off shows exactly what each one contributes.
4. **(Optional, later) on-robot bench verification** through `rogo`.

## Filter behavior to address

- **Heading has no direct absolute measurement.** OTOS heading is not fused
  (deferred in sprint 022), so `theta` is corrected only *indirectly* via the
  position-update cross-covariance (`P[2][0]`/`P[2][1]`, built up by the predict
  Jacobian) plus dead-reckoned by `dTheta`. Its absolute drift is fully removed
  only by a camera reset. This is acceptable under the stopped-camera strategy
  but is the filter's weakest axis — document it explicitly and make sure the
  notebook visualizes heading drift between camera fixes. (Fusing OTOS heading
  as a third measurement channel remains future work.)
- **Mahalanobis outlier gating on updates.** The innovation covariance `S`
  already tells the filter how surprised it *should* be, so each measurement
  update should gate on the Mahalanobis distance `yᵀ·S⁻¹·y` against a chi-square
  threshold — a principled replacement for the current fixed-distance `otosGate`
  in `Odometry::correct`. The present EKF `update` only guards a singular `S`
  (`EKF.cpp:157`). Apply gating to the OTOS position and both velocity updates.

## Explicitly out of scope (decided)

- **No measurement-latency / out-of-sequence / rewind-replay machinery.** The
  common filter runs **on the robot**; encoder + OTOS are sampled in the same
  cooperative loop (co-timed, negligible skew), so age doesn't matter for
  onboard fusion. `lagMs` and the TLM `t` stamp stay as harmless metadata, not
  wired into fusion.
- **Camera latency is handled operationally** by only applying a camera fix when
  the robot is **stopped** (`speed × latency = 0`). The existing `SI` →
  `Odometry::setPose` path (overwrite pose + zero covariance) is already correct
  for stopped updates and **does not change**. The new `v`/`omega` states
  conveniently provide an "am I stopped?" gate — accept a camera fix only when
  `|v|` and `|omega|` are below a threshold.

## Notes / gotchas

- The firmware `OV` command is **mislabeled**: the comment at
  `Odometry.cpp:398` says "report vx,vy,omega" but `handleOV`/`parseOV` actually
  **set** the OTOS position (`setPositionRaw`). There is currently **no firmware
  path that reads OTOS velocity or acceleration** — that read path (new OTOS
  wrapper methods + a read command) is new work required for items 1 and 2.
- Any millisecond timestamp subtraction must use **signed deltas** (uint32
  underflow gotcha — see the `watchdog-uint32-underflow` finding) if `dt` is
  computed from robot-clock stamps.
- **Latent `setPose` bug:** `Odometry::setPose` (`Odometry.cpp:95-103`) sets
  `_prevEncL = _prevEncR = 0.0f` but does **not** zero `encLMm/encRMm`, and the
  `SI` handler (`Robot.cpp:1026-1039`) doesn't either. So the next `predict`
  computes `dL = encLMm - 0` — a spurious jump equal to the entire cumulative
  encoder count — right after a camera fix. Fix: re-baseline to the current
  reading (`_prevEncL = s.encLMm; _prevEncR = s.encRMm`) instead of zeroing.
  This is the same "reset on camera" requirement as Goal item 7.
