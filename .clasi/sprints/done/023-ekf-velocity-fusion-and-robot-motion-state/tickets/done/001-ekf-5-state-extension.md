---
id: '001'
title: EKF 5-state extension (EKF.h / EKF.cpp)
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-004
- SUC-005
depends-on: []
issue: ekf-velocity-fusion-and-robot-state.md
---

# T001: EKF 5-state extension (EKF.h / EKF.cpp)

## Description

Extend the existing 3-state EKF (`[x, y, theta]`) to a 5-state CTRV model
(`[x, y, theta, v, omega]`) with a 5x5 covariance matrix. This is the
mathematical core of sprint 023. It has no dependencies on HAL or OTOS — once
it compiles and passes the Python-mirrored unit tests in T006, T003 can integrate
it into Odometry.

Key design decisions (from the issue and architecture):
- Position block `[x,y,theta]` uses the arc-segment motion model unchanged from
  sprint 022. v and omega do NOT couple into x,y in the predict Jacobian
  (block-decoupled design).
- v and omega are random-walk states: `v_{k+1} = v_k + w_v`, `omega_{k+1} = omega_k + w_omega`.
- The existing `update(x_otos, y_otos)` is renamed `updatePosition(x, y)`.
- A new `updateVelocity(v_meas, omega_meas, r_v, r_omega)` fuses a
  velocity pair. v and omega are treated as independent 1D measurements.
- All three update methods (`updatePosition`, `updateVelocity`) apply
  Mahalanobis gating before modifying state or covariance.
- `setPose()` also zeroes the velocity states (v=0, omega=0).
- `predict()` gains a `dt` argument (seconds, float) — used to derive the
  encoder-rate velocity in Odometry (T003), but passed to `predict()` here for
  use in future full-coupling extension.

All operations unrolled; no heap, no STL.

## Acceptance Criteria

- [x] `source/control/EKF.h` updated with:
  ```cpp
  // State: [x_mm, y_mm, theta_rad, v_mmps, omega_rads]
  void init(float q_xy, float q_theta, float q_v, float q_omega,
            float r_otos_xy, float r_otos_v, float r_enc_v);
  void setPose(float x, float y, float theta);  // zeroes v,omega
  void predict(float dCenter, float dTheta, float theta_before, float dt_s);
  void updatePosition(float x_otos, float y_otos);   // renamed from update()
  void updateVelocity(float v_meas, float omega_meas, float r_v, float r_omega);
  float x() const;
  float y() const;
  float theta() const;
  float v() const;
  float omega() const;
  uint32_t rejectedCount() const;
  ```
- [x] Private members: `float _x[5]`, `float _P[5][5]`, `float _Q[5][5]`,
  `float _rOtosXy`, `float _rOtosV`, `float _rEncV`, `uint32_t _rejected`.
- [x] `init()` sets the 5x5 Q diagonal: `Q[0][0]=Q[1][1]=q_xy`, `Q[2][2]=q_theta`,
  `Q[3][3]=q_v`, `Q[4][4]=q_omega`; sets `_rOtosXy`, `_rOtosV`, `_rEncV`; resets
  `_x` and `_P` to zero.
- [x] `setPose(x, y, theta)` sets `_x[0..2]`, zeroes `_x[3]` and `_x[4]`, and
  zeroes all of `_P`.
- [x] `predict(dCenter, dTheta, theta_before, dt_s)`:
  - Position block `[0..2]` uses arc-segment exactly as sprint 022 `predict()`.
  - Velocity block: `_x[3]` and `_x[4]` unchanged (random-walk — identity
    sub-Jacobian). Process noise: `P[3][3] += _Q[3][3]`, `P[4][4] += _Q[4][4]`.
  - The off-diagonal entries P[0..2][3..4] and P[3..4][0..2] are zero and remain
    zero (block-decoupling — Jacobian has zero cross-block entries).
  - Full 5x5 P update is unrolled.
- [x] `updatePosition(x_otos, y_otos)`:
  - Observation model H is 2x5 with H[0][0]=1, H[1][1]=1, rest zero.
  - Innovation: `y[0] = x_otos - _x[0]`, `y[1] = y_otos - _x[1]`.
  - Innovation covariance S (2x2): `S[0][0] = P[0][0] + _rOtosXy`, etc.
  - Mahalanobis gate: compute `d2 = y[0]*si00*y[0] + y[0]*si01*y[1] + y[1]*si10*y[0] + y[1]*si11*y[1]`; if `d2 > 5.99f` increment `_rejected` and return without update.
  - Kalman gain K is 5x2; state update and P update fully unrolled.
- [x] `updateVelocity(v_meas, omega_meas, r_v, r_omega)`:
  - Fuse v independently: H_v = [0,0,0,1,0]; innovation `yv = v_meas - _x[3]`;
    `s_v = P[3][3] + r_v`; gate: `yv*yv/s_v > 3.84f` → skip (increment `_rejected`);
    else K_v[i] = P[i][3]/s_v for i in 0..4; update state and P.
  - Fuse omega independently: H_w = [0,0,0,0,1]; same pattern with `r_omega`.
  - Each is a 1D Kalman update; treat them as two sequential scalar updates.
- [x] `rejectedCount()` returns `_rejected`.
- [x] No `new`/`malloc`, no STL, no `<random>`.
- [x] Compiles cleanly in both embedded and host builds (`python3 build.py` no
  errors for EKF.h / EKF.cpp).

## Implementation Plan

### Approach

The existing `EKF.h` and `EKF.cpp` are replaced in place. The structure of
`predict()` follows the sprint 022 implementation exactly for the position block;
the velocity block appends two more rows/columns to the P update.

**5x5 P update in predict():**

Let `a = -dCenter*sinf(theta_mid)`, `b = dCenter*cosf(theta_mid)`.
The Jacobian F is 5x5 identity except F[0][2]=a, F[1][2]=b (position block
entries, same as sprint 022). The velocity block sub-Jacobian is identity
(random-walk). Off-diagonal entries between blocks are zero.

The full 5x5 `P = F*P*F^T + Q` unrolls as:
- Rows 0..2, cols 0..2: same as sprint 022 3x3 unroll.
- Row 0/1/2, cols 3/4: `P[i][3]` and `P[i][4]` are updated by the arc-segment
  Jacobian cross-terms with col 3,4 of P. Since F[0][3]=F[0][4]=0 and
  F[3][0..4]=identity row, these cross-terms simplify significantly.
  With block decoupling initialized to zero, they remain zero and do not need
  explicit unrolling — but the programmer should confirm by writing the general
  formula and verifying the zero-cross-term invariant is maintained.
- Row 3, col 3: `P[3][3] += Q[3][3]` (pure random-walk).
- Row 4, col 4: `P[4][4] += Q[4][4]` (pure random-walk).
- All cross-block entries (P[0..2][3..4], P[3..4][0..2]) remain 0 throughout
  because both F and Q have zero cross-block entries. The programmer may
  explicitly set these to 0 in the unrolled update for clarity, or omit them
  with a clear comment explaining the invariant.

**Mahalanobis gating in updatePosition():**

The gate uses the S_inv that is already computed for the Kalman gain:
```cpp
float d2 = yi0*(si00*yi0 + si01*yi1) + yi1*(si10*yi0 + si11*yi1);
if (d2 > 5.99f) { ++_rejected; return; }
```
Place the gate check immediately after computing S_inv and before computing K.

**Sequential scalar updates in updateVelocity():**

Treating v and omega as independent 1D measurements is equivalent to sequential
scalar Kalman updates. For each:
```
s = P[j][j] + r
if (y*y/s > 3.84f) { skip; continue; }
K[i] = P[i][j] / s  for i=0..4
_x[i] += K[i] * y
P[i][k] -= K[i] * P[j][k]  for k=0..4   (Joseph form simplifies to this for scalar)
```
where j=3 for v and j=4 for omega.

### Files to modify

- `source/control/EKF.h` — update class definition, private members, method signatures
- `source/control/EKF.cpp` — full rewrite of all method bodies for 5-state

### Files unchanged

All other files. T003 will update the callers.

### Testing plan

Build verification:
```
python3 build.py
```
Mathematical correctness is verified by the Python EKF mirror in T006. The
programmer should write the C++ first, then T006's Python mirror test is run to
confirm parity. The two can be developed in parallel since the Python mirror is
independent.

```
uv run --with pytest python -m pytest tests/dev/test_ekf.py -v
```

### Documentation updates

Update the file-header comment in `EKF.h` and `EKF.cpp` to reference sprint 023
and the 5-state CTRV model.
