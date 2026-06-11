---
id: '023'
title: EKF Velocity Fusion and Robot Motion State
status: done
branch: sprint/023-ekf-velocity-fusion-and-robot-motion-state
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
issues:
- ekf-velocity-fusion-and-robot-state.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 023: EKF Velocity Fusion and Robot Motion State

## Goals

Extend the firmware EKF from a 3-state pose-only filter to a 5-state CTRV
model that also tracks body-frame linear speed `v` and yaw rate `omega`. Fuse
two velocity measurement sources (encoder-derived rate and OTOS native velocity)
using principled Mahalanobis gating to replace the fixed-distance outlier gate.
Expose the fused velocity in the TLM stream and build a composite `RobotState`
dataclass on the host side to carry pose, velocity, and acceleration together.

## Problem

After Sprint 022 the firmware EKF fuses encoder dead-reckoning with OTOS
position — but produces only pose (`x`, `y`, `theta`). Body-frame velocity `v`
and yaw rate `omega` remain as raw per-wheel values (`velLMms`/`velRMms`)
scattered across `HardwareState`, with no body-frame aggregation and no
filtering. The host `Pose` dataclass is position+heading only; the `NezhaState`
class holds `otos_pose`, `heading_rad`, and `dt_s` as loose fields with no
unified motion state. Two bugs must also be fixed: (1) `Odometry::setPose()`
zeroes `_prevEncL/_prevEncR` instead of re-baselining to the current encoder
reading, causing a spurious jump in the next `predict()` after every camera fix;
(2) the `OV` command comment says "report velocity" but it actually calls
`setPositionRaw()` — there is no firmware path that reads OTOS velocity at all.

## Solution

1. Grow the firmware `EKF` class from `[x,y,theta]` to `[x,y,theta,v,omega]`
   with a 5x5 covariance. `v`/`omega` use a random-walk predict (no
   coupling back into `x`,`y`). Add two new EKF update channels: encoder-derived
   velocity (`dCenter/dt`, `dTheta/dt`) and OTOS native velocity (new hardware
   read path). Replace the fixed `otosGate` distance check with Mahalanobis
   gating on every update channel.
2. Add `readVelocityTransformed()` to `IOtosSensor` / `OtosSensor` to feed the
   OTOS velocity measurements into the filter. Add `readAccelTransformed()` for
   the passthrough acceleration field.
3. Add new EKF tuning params `ekfQv`, `ekfQomega`, `ekfROtosV`, `ekfREncV` to
   `Config.h`; regenerate `DefaultConfig.cpp`.
4. Extend `HardwareState` with `fusedV`, `fusedOmega`, `otosAccelX`,
   `otosAccelY` fields. Fix the `setPose` encoder re-baseline bug.
5. Add `twist=v_mmps,omega_mradps` to the TLM frame. Add `TLM_FIELD_TWIST`
   bitmask bit to `Config.h`.
6. On the host, add `RobotState` dataclass (`host/robot_radio/robot/robot_state.py`)
   containing `pose: Pose`, `v`, `omega`, `accel | None`, `stamp`. Extend
   `TLMFrame` with a `twist` field. Update `NezhaState._process_line()` to
   build a `RobotState` from each TLM frame.
7. Extend `tests/dev/test_ekf.py` to the 5-state Python EKF mirror, add golden
   vectors, replay harness, and regression tests for the setPose fix and
   Mahalanobis gating. Add a demonstration notebook to `host_tests/`.

## Success Criteria

- `uv run --with pytest python -m pytest tests/dev/test_ekf.py -v` passes all
  new 5-state EKF tests (predict/update/convergence/golden vectors/setPose fix).
- `uv run --with pytest python -m pytest` — full suite passes, no regressions.
- Firmware builds cleanly with `python3 build.py`.
- TLM frames emitted by the firmware contain `twist=<v>,<omega>` when
  `TLM_FIELD_TWIST` is set.
- `NezhaState.robot_state` returns a `RobotState` with non-None `v` and `omega`
  when a `twist=` field is present in the TLM stream.
- Demonstration notebook runs to completion and produces trajectory/velocity/
  covariance plots without errors.

## Scope

### In Scope

- `source/control/EKF.h` / `EKF.cpp` — extend to 5-state CTRV; add
  `updateVelocity()` method; add Mahalanobis gating to all update channels.
- `source/hal/IOtosSensor.h` / `OtosSensor.h` / `OtosSensor.cpp` — add
  `readVelocityTransformed()` and `readAccelTransformed()` to the interface
  and concrete class.
- `source/types/Config.h` — add `ekfQv`, `ekfQomega`, `ekfROtosV`, `ekfREncV`;
  add `TLM_FIELD_TWIST` bitmask constant.
- `source/control/RobotState.h` — add `fusedV`, `fusedOmega`, `otosAccelX`,
  `otosAccelY` fields to `HardwareState`.
- `source/control/Odometry.h` / `Odometry.cpp` — extend `initEKF()` / `predict()`
  / `correctEKF()` for 5-state; fix `setPose` re-baseline bug; extend
  `correctEKF` signature to include velocity measurements and dt.
- `source/robot/Robot.cpp` — extend `otosCorrect()` to read velocity/accel and
  feed into filter; extend `buildTlmFrame()` with `twist=` field; update
  `STREAM fields=` parser for `twist`.
- `scripts/gen_default_config.py` / `source/robot/DefaultConfig.cpp` — add four
  new EKF params.
- `host/robot_radio/robot/robot_state.py` — new `RobotState` dataclass.
- `host/robot_radio/robot/protocol.py` — extend `TLMFrame` with `twist` field;
  extend `parse_tlm()` to parse `twist=`.
- `host/robot_radio/robot/nezha_state.py` — add `robot_state` attribute built
  from `TLMFrame`.
- `tests/dev/test_ekf.py` — extend Python EKF mirror to 5-state; new test
  classes; golden vectors; replay harness.
- `host_tests/demo_ekf_velocity_fusion.ipynb` — new demonstration notebook.

### Out of Scope

- OTOS heading fusion in the EKF (follow-on sprint).
- Measurement latency / out-of-sequence / rewind-replay machinery.
- Camera latency handling changes (the `SI`→`setPose` path is already correct
  for stopped updates; only the encoder re-baseline bug is fixed).
- Promotion of acceleration into the EKF state.
- SET/GET keys for the four new EKF noise params.
- Removing legacy `Odometry::correct()` or `alphaPos`/`alphaYaw`/`otosGate`.

## Test Strategy

Primarily offline. The testing pyramid has three levels:

1. **Unit tests** (`tests/dev/test_ekf.py`) — deterministic predict/update
   assertions on the Python EKF mirror; golden-vector parity with C++; regression
   test for the setPose re-baseline fix; Mahalanobis gating rejection test.
2. **Offline replay harness** — capture a real TLM log (encoder + OTOS +
   camera frames), replay it through the Python EKF deterministically, produce
   trajectory comparison. No robot required; log can be committed.
3. **Demonstration notebook** (`host_tests/demo_ekf_velocity_fusion.ipynb`) —
   graphical overlay of encoder-only vs +OTOS vs +OTOS+camera trajectories,
   drift/error vs time, fused v/omega vs raw, covariance evolution, per-sensor
   toggle panels, heading drift between camera fixes.

On-robot bench verification via `rogo` is optional and deferred.

## Architecture Notes

The EKF 5-state extension uses block-decoupled Jacobians: the position block
`[x,y,theta]` is propagated by the encoder arc-segment (unchanged from sprint
022); the velocity block `[v,omega]` uses a random-walk model with two
measurement updates. v/omega do NOT feed back into x,y in the predict Jacobian.
This preserves the sprint-022 position accuracy while adding velocity estimation.

`OtosSensor::getVelocityRaw()` already exists (reads `REG_VELOCITY_XL = 0x26`).
A new `readVelocityTransformed()` on `IOtosSensor` / `OtosSensor` applies the
same mounting-offset/flip transforms as `readTransformed()`. Acceleration follows
the same pattern via `REG_ACCELERATION_XL = 0x2C` (new constant).

The `setPose` re-baseline fix is `_prevEncL = s.encLMm; _prevEncR = s.encRMm`
instead of zeroing — a one-line correctness fix with its own regression test.

Mahalanobis gating replaces the fixed `otosGate` distance check in
`Odometry::correctEKF`. Threshold is chi-square with 2 DOF (OTOS position: 5.99;
each velocity channel: 3.84 at p=0.05).

## GitHub Issues

(None — tracked via CLASI issue file `ekf-velocity-fusion-and-robot-state.md`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| T001 | EKF 5-state extension (EKF.h / EKF.cpp) | — |
| T002 | Config params and OTOS velocity/accel interface | — |
| T003 | Odometry integration (5-state EKF wiring + setPose fix + Mahalanobis gating) | T001, T002 |
| T004 | Robot wiring (otosCorrect velocity read + twist TLM field) | T003 |
| T005 | Host: RobotState dataclass, TLMFrame twist, NezhaState wiring | T004 |
| T006 | Tests: Python EKF mirror 5-state extension, golden vectors, replay harness, notebook | T001, T005 |

Tickets execute serially in the order listed.
