---
id: '006'
title: 'Notebook cells 5-6: OTOS+camera Experiment 2 and EKF Experiment 3'
status: done
use-cases:
- SUC-005
depends-on:
- '005'
github-issue: ''
issue: ''
completes_issue: false
---

# Notebook cells 5-6: OTOS+camera Experiment 2 and EKF Experiment 3

## Description

Add cells 5 and 6 to `demo_figure_eight.ipynb`.

**Cell 5 — Experiment 2:** OTOS + periodic camera fix. Position estimate uses
`conn.get_otos_pose()` (accumulated noisy OTOS pose). Every 30 drive steps, a camera
fix is queued; after a 5-step delay it is applied by calling `conn.set_otos_pose(x, y, h)`
with the exact pose values, resetting the OTOS accumulator to the true position.

**Cell 6 — Experiment 3:** EKF fusion. A 3-state (x, y, theta) EKF is defined in this
cell as a Python class. Each step: `predict(v_mms, omega_mrads, dt_s)` propagates the
state; `update(z, R)` applies the OTOS reading. The same delayed camera fix triggers
an additional `update` with lower covariance. The EKF estimate is the position fed to
the pursuit controller.

Both cells also depend on the `EKF` class, which is added to Cell 3 (or Cell 6 itself)
as a new section after `pure_pursuit_vw`.

## Acceptance Criteria

### EKF class (added to Cell 3 or Cell 6)

- [x] `EKF` class with:
  - `__init__(self, x0, y0, h0, Q_diag, R_otos_diag)` — initialises state vector
    `[x, y, h]` and covariance matrix `P`, process noise covariance `Q`, default
    OTOS observation noise `R_otos`.
  - `predict(self, v_mms, omega_mrads, dt_s)` — nonlinear unicycle prediction:
    `dx = v*cos(h)*dt`, `dy = v*sin(h)*dt`, `dh = omega*dt`. Updates `x`, `P`
    using Jacobian F. (Linearised EKF is sufficient.)
  - `update(self, z_xy, R=None)` — linear position update (`H = [[1,0,0],[0,1,0]]`
    or full 3-state depending on whether yaw is observed). `R` defaults to
    `R_otos`. Applies standard Kalman update: `K = P H^T (H P H^T + R)^{-1}`,
    `x += K (z - H x)`, `P = (I - K H) P`.
  - State is stored as a numpy array; `P` as a 3x3 numpy matrix.
- [x] EKF tested inline: predict a straight step, then update with the true position —
  state should move closer to truth.

### Cell 5 — Experiment 2

- [x] Fresh sim via `make_sim()`.
- [x] Same drive loop structure as Cell 4 (same number of steps, same tick size).
- [x] Each step: estimate from `conn.get_otos_pose()` (`otos_x`, `otos_y`, `otos_h`).
- [x] Camera fix mechanism:
  - Maintain `cam_queue = []` and `step_counter`.
  - Every 30 steps: append `(step_counter + 5, conn.get_exact_pose())` to `cam_queue`.
  - Each step: check if the front of `cam_queue` has `due_step <= step_counter`;
    if so, call `conn.set_otos_pose(truth["x"], truth["y"], truth["h"])` and pop.
- [x] Logs truth (ExactPoseTracker), estimate (OTOS pose), and path reference.
- [x] After the loop, disconnects and plots estimate vs truth vs reference.
- [x] Estimate trajectory shows visible kinks at camera fix points.

### Cell 6 — Experiment 3

- [x] Fresh sim via `make_sim()`.
- [x] Creates `ekf = EKF(...)` at the start with reasonable process noise Q and OTOS
  observation noise R.
- [x] Same drive loop: each step calls `ekf.predict(v_mms, omega_mrads/1000, 0.05)`
  (convert mrad/s to rad/s; dt = 50 ms = 0.05 s).
- [x] Each step: calls `otos = conn.get_otos_pose()` and
  `ekf.update(np.array([otos["x"], otos["y"]]))`.
- [x] Same camera fix queue mechanism as Cell 5; on camera fix: calls
  `ekf.update(np.array([truth["x"], truth["y"]]), R=R_camera)` where `R_camera`
  has much smaller diagonal than `R_otos` (e.g., 1% of `R_otos`).
- [x] The estimate fed to `pure_pursuit_vw` uses `(ekf.x[0], ekf.x[1])` and
  `ekf.x[2]` for yaw.
- [x] Logs truth, EKF estimate, and path reference.
- [x] After the loop, disconnects and plots estimate vs truth vs reference.
- [x] EKF trajectory is visually smoother and closer to reference than Cell 4 (dead
  reckoning) trajectory.

## Implementation Plan

### EKF placement

Define the `EKF` class in Cell 3 (after `pure_pursuit_vw`) so it is available to both
Cell 5 and Cell 6. Cell 3 already runs before all experiment cells.

### Noise parameter tuning

Choose Q and R so the EKF converges visually but doesn't look perfect. Starting point:
- `Q_diag = [1.0, 1.0, 0.01]` (process noise: 1 mm x, 1 mm y, 0.01 rad heading)
- `R_otos_diag = [100.0, 100.0]` (OTOS observation noise: 10 mm std on x and y)
- `R_camera_diag = [1.0, 1.0]` (camera: 1 mm std)

The programmer should adjust these to produce a visible improvement in Cell 6 vs Cell 4.

### Import numpy

Cell 1 already imports `numpy as np`. Cell 6 uses `np.eye`, `np.array`, `np.linalg.inv`
— all standard. No additional imports needed.

### Camera fix via set_otos_pose

`conn.set_otos_pose()` calls `MockOtosSensor::setInjectedPose()`, which after ticket 003
also resets `_odomX/Y/H`. So the camera fix simultaneously resets the OTOS accumulator
to the true position — exactly the intended behavior for Experiment 2.

### Files to modify

- `host_tests/demo_figure_eight.ipynb` — add cells 3 (EKF class), 5 (Experiment 2),
  and 6 (Experiment 3); Cell 3 already exists from ticket 005 and needs the `EKF`
  class appended to it.

### Testing plan

- `jupyter nbconvert --to notebook --execute host_tests/demo_figure_eight.ipynb`
  must succeed through Cell 6 after this ticket.
- Verify visually: Experiment 2 shows kink corrections; Experiment 3 shows smoother
  trajectory than Experiment 1.

### Documentation updates

No additional docs; notebook cells are self-documenting with markdown headers.
