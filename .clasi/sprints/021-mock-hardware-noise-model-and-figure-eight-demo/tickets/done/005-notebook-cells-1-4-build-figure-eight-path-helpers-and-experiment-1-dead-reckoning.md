---
id: '005'
title: 'Notebook cells 1-4: build, figure-eight path, helpers, and Experiment 1 dead
  reckoning'
status: done
use-cases:
- SUC-004
depends-on:
- '004'
github-issue: ''
issue: ''
completes_issue: false
---

# Notebook cells 1-4: build, figure-eight path, helpers, and Experiment 1 dead reckoning

## Description

Create `host_tests/demo_figure_eight.ipynb` with its first four cells. These cells:
1. Build `libfirmware_host`, import dependencies, and define `make_sim()` (which enables
   the noise model).
2. Generate and plot a catmull-rom figure-eight path through 9 control points.
3. Define the `pure_pursuit_vw` geometry helper.
4. Run Experiment 1 (dead reckoning only) and plot the trajectory.

The notebook is modelled after `host_tests/demo_square.ipynb` in style and boilerplate.
The pure pursuit helper is reimplemented inline (~10 lines) to output
`(v_mms, omega_mrads)` directly, suitable for the `VW` command.

## Acceptance Criteria

### Cell 1 — Build and imports

- [x] Cell mirrors `demo_square.ipynb` build boilerplate (cmake configure + build,
  stdout streamed, error on nonzero exit).
- [x] Imports: `ctypes`, `math`, `subprocess`, `pathlib`, `sys`, `numpy as np`,
  `matplotlib.pyplot as plt`.
- [x] `REPO_ROOT`, `HOST_DIR`, `LIB_DIR` computed identically to `demo_square.ipynb`.
- [x] `sys.path.insert(0, str(HOST_DIR))` so `robot_radio` is importable.
- [x] Imports `SimConnection` from `robot_radio.io.sim_conn` and `NezhaProtocol`
  from `robot_radio.robot.protocol`.
- [x] Imports `catmull_rom` (or the relevant function) from
  `robot_radio.path.catmull_rom` and `Path` from `robot_radio.path.path_helper`.
  Confirm the exact import names by reading those files.
- [x] `make_sim(slip=(0.005, 0.03), enc_noise=0.05, otos_linear=0.01, otos_yaw=0.025)`
  helper: creates `SimConnection`, connects, creates `NezhaProtocol`,
  calls `proto.set_config(sTimeout=60000)`, calls `conn.set_slip(*slip)`,
  `conn.set_encoder_noise(enc_noise)`, `conn.enable_otos_model()`,
  `conn.set_otos_noise(otos_linear, otos_yaw)`, returns `(conn, proto)`.
- [x] Smoke-test PING at end of cell: `proto_test.ping()` succeeds, prints result,
  disconnects.

### Cell 2 — Figure-eight path

- [x] 9 control points defining a figure-eight (~600 mm span). Suggested points
  (in mm, starting at origin, looping through two lobes):
  ```
  (0,0), (150,150), (300,0), (150,-150),
  (0,0), (-150,150), (-300,0), (-150,-150), (0,0)
  ```
  Adjust to produce a smooth, closed figure-eight shape.
- [x] Catmull-rom spline sampled at sufficient resolution (e.g., 500 points) to give
  smooth pursuit tracking.
- [x] Path plotted with control points marked.
- [x] Path object stored as `path` (a `Path` instance or array — whatever
  `pure_pursuit_vw` in Cell 3 expects).

### Cell 3 — pure_pursuit_vw helper

- [x] Function signature: `pure_pursuit_vw(path_pts, pos_mm, yaw_rad, lookahead_mm,
  base_speed_mms) -> tuple[float, float]` returning `(v_mms, omega_mrads)`.
- [x] Implementation (Pure Pursuit geometry):
  1. Find the closest point on `path_pts` to `pos_mm`.
  2. Walk forward along `path_pts` until cumulative arc length >= `lookahead_mm`;
     use that as the goal point.
  3. Compute `alpha` = angle from robot heading to goal in robot frame.
  4. `kappa = 2 * sin(alpha) / lookahead_mm` (curvature).
  5. `omega = kappa * base_speed_mms` (rad/s); convert to mrad/s.
  6. Return `(base_speed_mms, omega_mrads)`.
- [x] `v_mms` is clamped to `[50, base_speed_mms]` to avoid stopping.
- [x] `omega_mrads` is clamped to `[-2000, 2000]` to stay within VW command range.
- [x] Function is tested inline with a trivial example (robot at origin facing right,
  goal directly ahead — expect omega ≈ 0).

### Cell 4 — Experiment 1: dead reckoning

- [x] Creates a fresh sim via `make_sim()`.
- [x] Drives the simulated robot along the figure-eight for at least 2 loops
  (or a fixed number of steps, e.g., 400 steps at 50 ms each).
- [x] Each step: reads `state = conn.get_state()`, extracts `(pose_x, pose_y, pose_h)`,
  calls `pure_pursuit_vw(...)`, sends `VW {v} {omega}` and `+` keepalive, calls
  `conn.tick(50)`, logs `conn.get_exact_pose()` as truth and `state` as estimate.
- [x] After the loop, calls `conn.disconnect()`.
- [x] Plots the estimated trajectory (dead reckoning), the true trajectory
  (ExactPoseTracker), and the reference path. Three curves on the same axes.
- [x] Estimated trajectory visibly diverges from the reference path after ~1 loop
  (demonstrating encoder drift).
- [x] Cell runs to completion without hardware and without error.

## Implementation Plan

### Files to create

- `host_tests/demo_figure_eight.ipynb` — create as a new Jupyter notebook with 4 cells
  (Cells 5-7 added by tickets 006 and 007).

### Path library imports

Read `host/robot_radio/path/catmull_rom.py` and `host/robot_radio/path/path_helper.py`
to confirm the exact class/function names before writing Cell 1 and Cell 2.

### Step size

Use 50 ms tick steps (not 24 ms) to reduce sim time and log size. 400 steps = 20 s of
sim time — enough for approximately 2 figure-eight loops at ~200 mm/s average.

### VW command integer conversion

`VW` takes integer mm/s and integer mrad/s. Use `int(v_mms)` and `int(omega_mrads)`.

### Notebook structure

Use the same cell-ID naming convention as `demo_square.ipynb` where possible. Markdown
cells above each section provide the section heading and a brief description.

### Testing plan

- `jupyter nbconvert --to notebook --execute host_tests/demo_figure_eight.ipynb`
  must succeed (all 4 cells) after this ticket.
- Verify visually that the path plot is a figure-eight shape.
- Verify that Experiment 1 trajectory drifts from reference.

### Documentation updates

No separate docs needed; the notebook itself is the documentation artifact.
