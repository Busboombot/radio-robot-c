---
id: '003'
title: Odometry integration (5-state EKF wiring + setPose fix + Mahalanobis gating)
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-004
- SUC-005
depends-on:
- '001'
- '002'
issue: ekf-velocity-fusion-and-robot-state.md
---

# T003: Odometry integration (5-state EKF wiring + setPose fix + Mahalanobis gating)

## Description

Wire the 5-state EKF (T001) and the new config params (T002) into `Odometry`.
This ticket has three distinct sub-changes:

1. **`setPose` re-baseline fix** — the most critical correctness fix in this
   sprint. Currently `setPose()` sets `_prevEncL = _prevEncR = 0.0f`, causing a
   spurious encoder jump on the very next `predict()` call after every camera fix.
   Fix: `_prevEncL = s.encLMm; _prevEncR = s.encRMm`.

2. **`predict()` extension** — gains a `dt_ms` argument (signed int32 to avoid
   uint32 underflow), computes encoder-rate velocity (`v_enc = dCenter / dt_s`,
   `omega_enc = dTheta / dt_s`), passes `dt_s` to `_ekf.predict()`, and writes
   `fusedV`/`fusedOmega` back to `HardwareState`.

3. **`correctEKF()` extension** — accepts OTOS position, OTOS velocity, encoder
   velocity, and dt; calls `_ekf.updatePosition()`, then `_ekf.updateVelocity()`
   twice (once for OTOS vel, once for encoder vel). The Mahalanobis gating is
   inside the EKF (T001); Odometry just passes the measurements through.

4. **`initEKF()` extension** — passes four new noise params to `_ekf.init()`.

## Acceptance Criteria

**setPose fix (SUC-005):**
- [x] `Odometry::setPose()` re-baselines: `_prevEncL = s.encLMm;` and
  `_prevEncR = s.encRMm;` (not `= 0.0f`).
- [x] The `zero()` method (which calls `setPose(s, 0, 0, 0)`) uses the same fixed
  path. Note: `zero()` is called at robot start when encoders read 0 — so the
  re-baseline correctly sets `_prevEncL = 0` in that case, which is identical to
  the old behavior on startup. The fix only matters when encoders are non-zero.

**`initEKF()` (SUC-001):**
- [x] Signature extended to:
  ```cpp
  void initEKF(float q_xy, float q_theta, float q_v, float q_omega,
               float r_otos_xy, float r_otos_v, float r_enc_v);
  ```
- [x] Body calls `_ekf.init(q_xy, q_theta, q_v, q_omega, r_otos_xy, r_otos_v, r_enc_v)`.

**`predict()` (SUC-001):**
- [x] Signature extended to include `uint32_t now_ms` (robot clock timestamp):
  `void predict(HardwareState& s, float trackwidthMm, uint32_t now_ms)`.
- [x] `dt_s` is computed as `(int32_t)(now_ms - _lastPredictMs) * 0.001f`.
  Uses a signed cast to avoid uint32 underflow (per `watchdog-uint32-underflow`
  finding). `_lastPredictMs` is a new private member (uint32_t) initialised to 0.
- [x] On the first call (`_lastPredictMs == 0`): set `_lastPredictMs = now_ms`,
  skip the velocity update (pass `dt_s = 0.0f` or a guard — do not divide by 0).
- [x] `dt_s > 0` guard: only compute `v_enc = dCenter / dt_s` and
  `omega_enc = dTheta / dt_s` when `dt_s > 0.001f` (1 ms minimum). Otherwise
  use `v_enc = 0.0f`, `omega_enc = 0.0f`.
- [x] `_ekf.predict(dCenter, dTheta, theta_before, dt_s)` called after encoder
  delta computation (position before theta midpoint, same ordering as sprint 022).
- [x] `s.fusedV = _ekf.v()` and `s.fusedOmega = _ekf.omega()` written back
  alongside the existing `s.poseX = _ekf.x()`, etc.
- [x] `_lastPredictMs = now_ms` updated at the end of each `predict()` call.

**`correctEKF()` (SUC-002, SUC-004):**
- [x] Signature extended to:
  ```cpp
  void correctEKF(HardwareState& s,
                  float x_otos, float y_otos,
                  float v_otos_mmps, float omega_otos_rads,
                  float v_enc_mmps, float omega_enc_rads);
  ```
- [x] Body calls `_ekf.updatePosition(x_otos, y_otos)` first.
- [x] Body calls `_ekf.updateVelocity(v_otos_mmps, omega_otos_rads, _ekf._rOtosV, _ekf._rOtosV)`.
  NOTE: `_rOtosV` and `_rEncV` must be accessible — either as public constants or
  via a getter, or pass them directly from the stored values. Recommended: store
  them as private members on `Odometry` (copies from `initEKF()` call) and pass to
  `updateVelocity()` explicitly. Do not expose private EKF fields.
- [x] Body calls `_ekf.updateVelocity(v_enc_mmps, omega_enc_rads, r_enc_v, r_enc_v)`.
- [x] All EKF outputs written back: `s.poseX`, `s.poseY`, `s.poseHrad`,
  `s.fusedV`, `s.fusedOmega`.

**Odometry header:**
- [x] `source/control/Odometry.h` updated with new `initEKF()`, `predict()`,
  `correctEKF()` signatures.
- [x] New private member `uint32_t _lastPredictMs` declared.
- [x] New private members `float _rOtosV` and `float _rEncV` for storing noise
  params (passed to `updateVelocity()`).

**Build:**
- [x] `python3 build.py` passes cleanly.
- [x] `uv run --with pytest python -m pytest -v` passes (T004 caller update may
  be needed before the full suite passes — see dependency).

## Implementation Plan

### Approach

Work the three sub-changes in this order to minimize merge conflicts:

**Step 1: Fix setPose.**
One-line change in `Odometry.cpp`:
```cpp
// Before:
_prevEncL  = 0.0f;
_prevEncR  = 0.0f;
// After:
_prevEncL  = s.encLMm;
_prevEncR  = s.encRMm;
```
This does NOT change `zero()` behavior at startup (encoders are 0 at boot), only
after driving.

**Step 2: Add `_lastPredictMs`, `_rOtosV`, `_rEncV` private members.**
Initialise `_lastPredictMs = 0` in the Odometry constructor.

**Step 3: Extend `initEKF()` signature and body.**

**Step 4: Extend `predict()`.**
The `now_ms` argument is added. The dt computation and encoder-rate velocity follow
directly. The EKF call changes from `_ekf.predict(dCenter, dTheta, theta_before)`
to `_ekf.predict(dCenter, dTheta, theta_before, dt_s)`. Add the `fusedV`/
`fusedOmega` write-back lines.

**Step 5: Extend `correctEKF()`.**
The new arguments are appended. The three EKF calls replace the current single
`_ekf.update()` call.

**Note on `r_v` and `r_omega` in `updateVelocity()`:** The `_rOtosV` and `_rEncV`
scalars are used for both v and omega of each measurement source. This is a
simplification: a more precise model would use separate v and omega noise values.
The symmetric single-scalar approach is acceptable for v1 and should be documented
in a comment.

### Files to modify

- `source/control/Odometry.h` — new signatures; new private members
- `source/control/Odometry.cpp` — implement all changes

### Testing plan

After changes, T004 must also be updated before the full test suite runs. Run:
```
python3 build.py
```
to confirm compilation. The Python-mirror unit tests (T006) verify the EKF math;
T003 is structural wiring.

The setPose fix is regression-tested by T006's `TestSetPoseRebaseline` class.

### Documentation updates

Update the `Odometry.h` class comment block (sprint reference, new API note).
Add an inline comment above the `_prevEncL = s.encLMm` line explaining the
re-baseline requirement.
