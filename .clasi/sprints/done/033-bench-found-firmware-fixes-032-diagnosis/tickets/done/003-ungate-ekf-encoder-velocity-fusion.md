---
id: '003'
title: Ungate EKF encoder-velocity fusion
status: done
use-cases:
- SUC-003
depends-on:
- '002'
issue: fr-bench-twist-fusedv-zero.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Ungate EKF encoder-velocity fusion

## Description

`state.inputs.fusedV` and `fusedOmega` are permanently 0 whenever OTOS is invalid (lifted
bench stand, real-world dropout). The root cause: `Odometry::correctEKF()` calls both the
OTOS and encoder `updateVelocity()` observations, but `correctEKF()` is only reached through
`Robot::otosCorrect()`, which early-returns when the OTOS validity gates fail
(`Odometry.cpp:203-206`). Encoder velocity is derived independently in `predict()` but is
only ever fused inside the OTOS-gated path.

Fix: add an unconditional `_ekf.updateVelocity(enc_v, enc_omega, _rEncV, _rEncV)` call in
`Odometry::predict()` (after `dL`/`dR` are computed, before or after the existing predict
step). This fuses encoder velocity every tick regardless of OTOS health. OTOS
pose/heading/velocity fusion stays behind the validity gates in `correctEKF()`.

**Coupling with T005:** the `enc_omega` observation added here must be gated on both encoders
being healthy (a wedged wheel injects phantom omega into the fused state). Add a boolean
parameter or flag that suppresses the `enc_omega` term when either wheel is wedged. T005
will supply the `wheelWedged()` accessor; stub it as `false` for now if T005 is not yet
merged, and the coupling will be wired in T005.

Note: T002 (bench mode enable fix) is listed as a dependency so the post-sprint bench re-run
can validate both together, but the firmware fix here is independent.

## Acceptance Criteria

- [ ] Sim test: OTOS invalid + wheels moving (non-zero dL/dR in predict) → `fusedV` nonzero,
      `fusedOmega` nonzero after the tick
- [ ] Sim test: `enc_omega` observation is suppressed (effectively zero) when either wheel is
      flagged wedged (stub the flag for now if T005 not merged; wire in T005)
- [ ] Existing sim tests that assert `fusedV == 0` or `fusedOmega == 0` during OTOS-invalid
      scenarios are updated to the new expected non-zero behavior
- [ ] `python3 build.py` clean build passes
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes

## Testing

- **Existing tests to run**: `uv run --with pytest python -m pytest host_tests/ host/tests/`
  (may require updating assertions on `fusedV`/`fusedOmega` for OTOS-invalid scenarios)
- **New tests to write**: Sim test: set up OTOS-invalid state, run predict() with non-zero
  wheel deltas, assert `fusedV`/`fusedOmega` are non-zero. Second sim test: same but with a
  wheel wedged flag set, assert `enc_omega` is suppressed.
- **Verification command**: `uv run --with pytest python -m pytest host_tests/ host/tests/`

## Implementation Plan

### Approach

In `Odometry::predict()` (`source/control/Odometry.cpp`), after computing `enc_v` and
`enc_omega` from `dCenter` and `dTheta`, add:

```cpp
// Fuse encoder velocity unconditionally (OTOS-gated path may be skipped).
// Suppress enc_omega when a wheel is wedged to avoid phantom heading from
// a frozen encoder (coupled with T005 wedge-state accessor).
float omega_obs = bothEncodersHealthy ? enc_omega : 0.0f;
_ekf.updateVelocity(enc_v, omega_obs, _rEncV, _rEncV);
```

The `bothEncodersHealthy` flag comes from `MotorController::wheelWedged()` (added in T005);
for now stub it as `true` so the observation always fires, and T005 will add the actual
suppression.

Check whether `enc_v` and `enc_omega` are already available at the call site in `predict()`,
or need to be re-derived from `dCenter`/`dTheta`/`dt_s`.

### Files to Modify

- `source/control/Odometry.cpp` — add unconditional enc-velocity `updateVelocity` call in
  `predict()` with `enc_omega` health gate
- `source/control/Odometry.h` — update `predict()` signature if `bothEncodersHealthy` param
  needed, or have it pull from a MotorController reference
- `host_tests/` — add new sim tests; update any OTOS-invalid assertions that break

### Documentation Updates

Update the `Odometry.h` / `Odometry.cpp` comment describing the predict/correctEKF split to
note that encoder velocity is now fused in predict unconditionally.
