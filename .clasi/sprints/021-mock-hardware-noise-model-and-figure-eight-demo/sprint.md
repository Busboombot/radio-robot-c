---
id: '021'
title: Mock Hardware Noise Model and Figure-Eight Demo
status: planning-docs
branch: sprint/021-mock-hardware-noise-model-and-figure-eight-demo
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- plan-demo-figure-eight-ipynb-pure-pursuit-with-sensor-fusion.md
---

# Sprint 021: Mock Hardware Noise Model and Figure-Eight Demo

## Goals

1. Add realistic noise and error models to the host-side mock hardware layer
   (`MockMotor`, `MockOtosSensor`, `MockHAL`) so simulated encoder drift and
   OTOS drift match real-hardware characteristics (~1% encoder error, ~2.5% OTOS yaw drift).
2. Add an `ExactPoseTracker` inside `MockHAL` that accumulates the true robot
   pose from motor commands (no noise), providing an oracle ground-truth for tests.
3. Expose the new noise controls and exact-pose query through `sim_api.cpp` (C ABI)
   and `sim_conn.py` (Python bindings).
4. Deliver `host_tests/demo_figure_eight.ipynb` — a notebook that drives the simulated
   robot along a catmull-rom figure-eight path using Pure Pursuit and compares three
   position-estimation regimes: dead-reckoning, OTOS + camera fixes, and Kalman filter
   fusion.

## Problem

The current `MockMotor` integrates commanded speed into encoder mm with no noise or
slip — the encoder is mathematically perfect. This makes the sim useless for validating
sensor-fusion algorithms: dead-reckoning always matches the reference path, so there is
nothing for the Kalman filter to improve. The sim needs believable hardware errors
before it can support sensor-fusion development.

## Solution

**Part A — C++ mock hardware noise model:**
- `MockMotor`: add slip model (baseline + turn-rate-coupled extra slip) and Gaussian
  encoder noise. Turn rate is computed by `MockHAL` from both motor commands and fed
  to each motor before `tick()`. `trueVelocityMms()` exposes the pre-slip velocity.
- `MockHAL`: add `ExactPoseTracker` struct that integrates from `trueVelocityMms()` using
  midpoint kinematics. Coordinate turn-rate computation and tick OTOS mock.
- `MockOtosSensor`: add `tick()` that integrates from true velocities + applies independent
  Gaussian linear and yaw noise fractions. `readTransformed()` returns the accumulated
  noisy pose when sim model is enabled; injection API unchanged otherwise.
- `sim_api.cpp`: new C functions for exact pose read, slip/noise set, OTOS model enable.
- `sim_conn.py`: Python bindings for all new C functions; `_snapshot()` extended.

**Part B — Notebook:**
- Figure-eight path via catmull-rom through 9 control points (~600 mm span).
- Pure pursuit geometry (`pure_pursuit_vw`) in notebook Python, outputs `(v_mms, omega_mrads)` for VW command.
- Experiment 1: dead reckoning only (firmware pose from noisy encoders).
- Experiment 2: OTOS + periodic camera fix (hard-reset from oracle every ~1.5 s, 5-step delay).
- Experiment 3: EKF fusion (encoder prediction + OTOS update + delayed camera update).
- Comparison plot: all three trajectories vs reference + RMS cross-track error.

## Success Criteria

- `libfirmware_host` builds cleanly after C++ changes.
- Driving straight: `conn.get_state()["pose_x"]` diverges from `conn.get_exact_pose()["x"]`
  by ≥1 mm over 1 m (slip/noise working).
- Point turn: divergence is larger than straight (turn-rate coupling working).
- OTOS accumulated pose drifts independently of encoder dead-reckoning.
- Notebook executes end-to-end without hardware; Exp 3 (EKF) has lower RMS cross-track
  error than Exp 1 (dead reckoning).
- Existing pytest suite passes unchanged.

## Scope

### In Scope

- `source/hal/mock/MockMotor.h/.cpp` — slip + noise fields, enhanced `tick()`
- `source/hal/mock/MockHAL.h/.cpp` — `ExactPoseTracker`, turn-rate coordination, OTOS tick
- `source/hal/mock/MockOtosSensor.h/.cpp` — sim integration model + noise
- `host_tests/sim_api.cpp` — new C ABI functions
- `host/robot_radio/io/sim_conn.py` — new Python bindings
- `host_tests/demo_figure_eight.ipynb` — new notebook (all cells)

### Out of Scope

- Real firmware changes (no `.cpp`/`.h` outside `source/hal/mock/`)
- Changes to `NezhaHAL` (real hardware implementation)
- Modifying existing tests or `demo_square.ipynb`
- Acceleration-dependent slip (deferred to future sprint)

## Test Strategy

- The existing pytest suite (`uv run --with pytest python -m pytest`) must pass
  after C++ changes — no regressions.
- The notebook is the primary integration test: run via
  `jupyter nbconvert --to notebook --execute host_tests/demo_figure_eight.ipynb`
  and verify all cells complete without error.
- Manual checks: verify divergence between exact pose and encoder dead-reckoning grows
  during turns more than during straight driving.

## Architecture Notes

- Gaussian noise uses `std::mt19937` + `std::normal_distribution<float>` (host-only code;
  `<random>` is available). One RNG per mock object; seeded from a fixed constant for
  reproducibility.
- `ExactPoseTracker` uses midpoint integration identical to `Odometry::predict` to avoid
  introducing a new kinematic model.
- Turn rate: `turnRate = |cmdR - cmdL| / (|cmdL| + |cmdR| + ε)` ∈ [0,1].
- `MockHAL` needs trackwidth to tick `ExactPoseTracker`; passed via `setTrackwidth()`
  called from `SimHandle` constructor after `cfg` is built.
- OTOS noise model disabled by default; existing `setInjectedPose()` injection still works
  when model is off.

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | MockMotor slip model and Gaussian encoder noise | — |
| 002 | MockHAL ExactPoseTracker and turn-rate coordination | 001 |
| 003 | MockOtosSensor sim integration model and independent noise | 002 |
| 004 | sim_api C ABI extensions and sim_conn Python bindings | 003 |
| 005 | Notebook cells 1-4: build, figure-eight path, helpers, and Experiment 1 dead reckoning | 004 |
| 006 | Notebook cells 5-6: OTOS+camera Experiment 2 and EKF Experiment 3 | 005 |
| 007 | Notebook cell 7: comparison plot and end-to-end execution verification | 006 |

Tickets execute serially in the order listed.
