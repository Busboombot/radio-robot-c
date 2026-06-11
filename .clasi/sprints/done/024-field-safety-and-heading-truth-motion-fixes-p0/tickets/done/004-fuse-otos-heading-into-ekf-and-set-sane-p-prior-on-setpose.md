---
id: '004'
title: Fuse OTOS heading into EKF and set sane P-prior on setPose
status: done
use-cases:
- SUC-002
- SUC-004
depends-on: []
github-issue: ''
issue: d01-fuse-otos-heading-into-ekf.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 024-004 — Fuse OTOS heading into EKF and set sane P-prior on setPose

**Completes issue:** `d01-fuse-otos-heading-into-ekf.md`
**Chain:** D1 (no dependencies in EKF chain — foundation for D3 and D2)

## Description

The EKF observes x/y position and v/ω velocity but never heading. OTOS heading is
read into `state.inputs.otosH` every 100 ms but dropped at `correctEKF()` — only
`p.x, p.y` are passed. Heading is therefore pure encoder integration. Combined with
unapplied rotational slip (D2), heading error accumulates monotonically per turn.
Every G command transforms its world-frame target through that wrong heading — the
confirmed root cause of "gets turned around and drives into the boards".

Additionally, `EKF::setPose()` zeros the full P matrix (false perfect certainty),
which makes the Mahalanobis gate artificially tight after any pose injection, slowing
re-acquisition (compounds D3).

The fix adds `EKF::updateHeading()`, wires it through `Odometry::correctEKF()` and
`Robot::otosCorrect()`, adds the `ekfROtosTheta` config field, and changes `setPose()`
to initialize a sane diagonal P-prior.

## Files to Touch

- `source/control/EKF.h` — declare `updateHeading(float theta_meas, float r_theta)`;
  add `_rejHead_streak` int counter.
- `source/control/EKF.cpp` — implement `updateHeading()`: H = [0,0,1,0,0], scalar
  innovation `y = wrapPi(theta_meas - _x[2])`, χ²(1) gate at 3.84, P row/col update
  following `updateVelocity` template. Change `setPose()` to set diagonal P:
  `[100mm², 100mm², (5deg in rad)², vx_var, vy_var]` instead of zeroing.
  Add `_rejHead_streak` management (used by ticket 005).
  Resolve open question 3 (`wrapPi` utility): use the same `wrap_angle()` already in
  `StopCondition.cpp`, or inline `atan2f(sinf(y), cosf(y))` — pick one and document.
- `source/control/Odometry.h` — update `correctEKF()` signature to accept
  `float theta_otos_rad`.
- `source/control/Odometry.cpp` — `correctEKF()`: call `_ekf.updateHeading(theta_otos_rad,
  cfg.ekfROtosTheta)` between `updatePosition` and `updateVelocity`. `initEKF()`:
  pass `cfg.ekfROtosTheta`.
- `source/robot/Robot.cpp` — `otosCorrect()`: pass `p.h` as `theta_otos_rad` to
  `odometry.correctEKF()`. `initEKF()` call: pass `cfg.ekfROtosTheta`.
- `source/types/Config.h` — add `float ekfROtosTheta;` after existing EKF noise fields;
  add `TLM_FIELD_EKFREJ = (1u << 6)` bitmask.
- `source/robot/ConfigRegistry.cpp` — register `ekfROtosTheta` as SET-accessible.
- `data/robots/tovez.json` — add `ekfROtosTheta: 0.01`; verify `rotationalSlip: 0.74`
  is present. Re-run `scripts/gen_default_config.py` to regenerate `source/robot/DefaultConfig.cpp`.
- `tests/dev/test_ekf.py` — **update in lockstep with firmware EKF changes**:
  add `updateHeading(theta_meas, r_theta)` to the Python EKF class with wrap-safe
  innovation and χ²(1) gate matching the firmware. Add `TestUpdateHeading` test class
  asserting heading correction behavior. Update `TestSetPosePrior`: assert P diagonal
  matches the new sane prior (not zero) after `setPose()`. Update any existing test
  that asserts P == 0 after `setPose()` — those assertions must change.

## Acceptance Criteria

- [x] `EKF::updateHeading(theta_meas, r_theta)` is implemented with H = [0,0,1,0,0],
  wrap-safe innovation, and χ²(1) gate at 3.84.
- [x] `Odometry::correctEKF()` calls `updateHeading()` between `updatePosition` and
  `updateVelocity`.
- [x] `Robot::otosCorrect()` passes `p.h` to `correctEKF()`.
- [x] `EKF::setPose()` sets a sane diagonal P-prior (≈ 100 mm², 100 mm², (5°)²,
  velocity variances) instead of zeroing P. Any existing test asserting P == 0 after
  `setPose()` is updated.
- [x] `ekfROtosTheta` config field exists, is SET-accessible, defaults to ~0.01 rad²,
  and `DefaultConfig.cpp` is regenerated.
- [x] **`tests/dev/test_ekf.py` updated in lockstep:** Python EKF class has
  `updateHeading()` matching firmware behavior. `TestUpdateHeading` tests pass.
  `TestSetPosePrior` asserts the sane diagonal prior. Any `setPose` P-prior assertion
  changes are reflected consistently in both firmware and Python.
- [x] **Sim (field profile, fusion on + mock slip):** square + figure-eight; fused
  heading tracks mock-OTOS truth within ~2° per turn where it currently drifts.
  `TestHeadingConvergence` Python suite test verifies ≤2° error per corrected turn.
- [ ] **Hardware:** four `TURN 9000` in a row return the robot to its starting
  orientation within a few degrees (today ~90° off physically).
  **[deferred → sprint-end bench gate]**
- [x] Existing `host_tests/` and `tests/dev/test_ekf.py` pass after updates.

## Implementation Plan

### Approach

Follow `updateVelocity` as the template for `updateHeading`. The scalar-update P
update reduces to modifying only the third row/column. `wrapPi` resolution: locate
existing usage in the codebase and reuse; do not introduce a new utility. The
`setPose` P-prior change is a straightforward constant replacement — assign the
diagonal and leave off-diagonal at zero.

For config plumbing: the pattern (Config.h field → ConfigRegistry.cpp entry →
tovez.json → gen_default_config.py → DefaultConfig.cpp) is well-established from
prior sprints; follow it exactly.

For the Python mirror: every firmware change to `EKF.cpp` in this ticket must be
mirrored in `tests/dev/test_ekf.py` in the same commit.

### Testing Plan

1. `TestUpdateHeading` in `test_ekf.py`: issue a heading correction, verify P[2][2]
   narrows and `_x[2]` updates toward truth.
2. `TestSetPosePrior`: call `setPose()`, assert P[0][0] ≈ 100, P[1][1] ≈ 100,
   P[2][2] ≈ (5 * pi/180)^2, off-diagonal zero.
3. Field-profile sim test in `host_tests/`: run square trajectory with OTOS heading
   fusion active; assert cumulative heading error < 5° per turn.
4. `uv run pytest tests/dev/test_ekf.py host_tests/`.

### Documentation Updates

Add `ekfROtosTheta` to robot JSON schema comments. Note open question 1 (initial
value may need tuning to 0.04 rad² if OTOS is noisier on carpet).
