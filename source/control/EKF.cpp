#include "EKF.h"
#include <math.h>

// ===========================================================================
// EKF — 5-state Extended Kalman Filter for pose and velocity fusion
//
// State: [x_mm, y_mm, theta_rad, v_mmps, omega_rads]
// Sprint 023, Ticket 001.
// Sprint 024, Ticket 004: added updateHeading(); sane P-prior in setPose();
//   _rejHead_streak counter stub for D3 gate recovery.
// Sprint 024, Ticket 005: _rejPos_streak; P-inflation re-baseline recovery at 10
//   consecutive rejections in updatePosition() and updateHeading() independently;
//   getRejectCount() accessor alias for TLM.
//   Architecture deviation: original design called for R×10 inflation, but for
//   the 200mm/<2s acceptance criterion d²=200²/(P+10·R)≫5.99 — still permanently
//   rejected. P-inflation (set position/heading P block to a large value, then run
//   the standard update) makes S large so the gate passes (d²≈0) and K≈1,
//   snapping the state to the OTOS measurement in one update.
// ===========================================================================

// ---------------------------------------------------------------------------
// EKF() — default constructor: zero all state and noise matrices.
// ---------------------------------------------------------------------------

EKF::EKF()
{
    for (int i = 0; i < 5; ++i)
        _x[i] = 0.0f;

    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            _P[i][j] = _Q[i][j] = 0.0f;

    _rOtosXy = 0.0f;
    _rOtosV  = 0.0f;
    _rEncV   = 0.0f;
    _rejected = 0;
    _rejHead_streak = 0;
    _rejPos_streak  = 0;
}

// ---------------------------------------------------------------------------
// init — set noise parameters; reset state and covariance.
// ---------------------------------------------------------------------------

void EKF::init(float q_xy, float q_theta, float q_v, float q_omega,
               float r_otos_xy, float r_otos_v, float r_enc_v)
{
    // Zero _Q then set diagonal.
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            _Q[i][j] = 0.0f;

    _Q[0][0] = q_xy;
    _Q[1][1] = q_xy;
    _Q[2][2] = q_theta;
    _Q[3][3] = q_v;
    _Q[4][4] = q_omega;

    _rOtosXy = r_otos_xy;
    _rOtosV  = r_otos_v;
    _rEncV   = r_enc_v;
    _rejected = 0;
    _rejHead_streak = 0;
    _rejPos_streak  = 0;

    // Reset state and covariance.
    for (int i = 0; i < 5; ++i)
        _x[i] = 0.0f;

    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            _P[i][j] = 0.0f;
}

// ---------------------------------------------------------------------------
// setPose — overwrite state with known pose; zero v and omega; set sane P-prior.
//
// Sane diagonal P-prior (sprint 024-004): instead of zeroing P (which creates
// falsely tight Mahalanobis gates and strangles re-acquisition after a pose
// injection), we set a modest diagonal that reflects realistic uncertainty:
//   P[0][0] = P[1][1] = kPriorXY    (100 mm^2  — ~1-sigma 10mm position)
//   P[2][2]           = kPriorTheta (~(5 deg)^2 — ~1-sigma 5° heading)
//   P[3][3]           = kPriorV     (100 (mm/s)^2)
//   P[4][4]           = kPriorOmega (0.01 (rad/s)^2)
//   Off-diagonal entries remain zero.
// ---------------------------------------------------------------------------

void EKF::setPose(float x, float y, float theta)
{
    _x[0] = x;
    _x[1] = y;
    _x[2] = theta;
    _x[3] = 0.0f;  // v
    _x[4] = 0.0f;  // omega

    // Zero all of P first, then set sane diagonal entries.
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            _P[i][j] = 0.0f;

    _P[0][0] = kPriorXY;
    _P[1][1] = kPriorXY;
    _P[2][2] = kPriorTheta;
    _P[3][3] = kPriorV;
    _P[4][4] = kPriorOmega;
}

// ---------------------------------------------------------------------------
// predict — arc-segment motion model (position block) + random-walk (velocity
// block). Block-decoupled: cross-block Jacobian and P entries remain zero.
//
// Motion equations (position block):
//   theta_mid = theta_before + dTheta/2
//   _x[0] += dCenter * cos(theta_mid)
//   _x[1] += dCenter * sin(theta_mid)
//   _x[2]  = wrapPi(_x[2] + dTheta)
//
// Velocity block (random-walk):
//   _x[3] unchanged,  _x[4] unchanged
//   P[3][3] += Q[3][3],  P[4][4] += Q[4][4]
//
// Jacobian F is 5x5 identity except:
//   F[0][2] = a = -dCenter*sin(theta_mid)
//   F[1][2] = b =  dCenter*cos(theta_mid)
//   (velocity sub-block is identity; all cross-block entries are zero)
//
// Covariance update: P = F*P*F^T + Q, fully unrolled.
//
// Block-decoupling: because F has zero cross-block entries and Q has zero
// cross-block entries, and P is initialized with zero cross-block entries
// (via init() or setPose()), the cross-block P entries remain zero at every
// predict step. This invariant is maintained explicitly: we only update the
// 3x3 position block and the 2x2 velocity diagonal; cross entries stay 0.
//
// P = F*P*F^T derivation for the position block (rows/cols 0..2):
//   T = F*P (F is identity with F[0][2]=a, F[1][2]=b):
//     T[0][j] = P[0][j] + a*P[2][j]
//     T[1][j] = P[1][j] + b*P[2][j]
//     T[2][j] = P[2][j]
//   Result = T*F^T (F^T is identity with FT[2][0]=a, FT[2][1]=b):
//     Result[i][0] = T[i][0] + T[i][2]*a
//     Result[i][1] = T[i][1] + T[i][2]*b
//     Result[i][2] = T[i][2]
// ---------------------------------------------------------------------------

void EKF::predict(float dCenter, float dTheta, float theta_before, float dt_s)
{
    // N15 fix (030-009): scale Q by dt_s so process noise is per-second rather
    // than per-call.  Without scaling, the effective Q grows proportionally to
    // call rate: at 10 ms/tick Q is added 100×/s, at 25 ms/tick only 40×/s —
    // a 2.5× variance swing with I2C load.  Multiplying by dt_s normalises to
    // Q_per_second regardless of loop period.  Clamp dt_s to [0, 0.5] to avoid
    // numerical blow-up on the very first tick (dt=0 → no noise added) or after
    // a long pause.
    if (dt_s < 0.0f) dt_s = 0.0f;
    if (dt_s > 0.5f) dt_s = 0.5f;

    float theta_mid = theta_before + dTheta * 0.5f;
    float ct = cosf(theta_mid);
    float st = sinf(theta_mid);

    // Position state update.
    _x[0] += dCenter * ct;
    _x[1] += dCenter * st;
    _x[2]  = wrapPi(_x[2] + dTheta);
    // _x[3] and _x[4] (v, omega) are unchanged — random-walk.

    // Jacobian non-identity entries for the position block.
    float a = -dCenter * st;   // F[0][2]
    float b =  dCenter * ct;   // F[1][2]

    // Load position block of P (rows/cols 0..2).
    float p00 = _P[0][0]; float p01 = _P[0][1]; float p02 = _P[0][2];
    float p10 = _P[1][0]; float p11 = _P[1][1]; float p12 = _P[1][2];
    float p20 = _P[2][0]; float p21 = _P[2][1]; float p22 = _P[2][2];

    // T = F * P for the position block rows.
    float t00 = p00 + a*p20;  float t01 = p01 + a*p21;  float t02 = p02 + a*p22;
    float t10 = p10 + b*p20;  float t11 = p11 + b*p21;  float t12 = p12 + b*p22;
    float t20 = p20;          float t21 = p21;           float t22 = p22;

    // New position block: T * F^T + Q*dt_s.
    _P[0][0] = t00 + t02*a + _Q[0][0] * dt_s;
    _P[0][1] = t01 + t02*b;
    _P[0][2] = t02;
    _P[1][0] = t10 + t12*a;
    _P[1][1] = t11 + t12*b + _Q[1][1] * dt_s;
    _P[1][2] = t12;
    _P[2][0] = t20 + t22*a;
    _P[2][1] = t21 + t22*b;
    _P[2][2] = t22 + _Q[2][2] * dt_s;

    // Cross-block entries (rows 0..2, cols 3..4 and rows 3..4, cols 0..2)
    // remain zero because F and Q have zero cross-block entries and these
    // entries start at zero (invariant maintained since init/setPose).
    // They are explicitly kept at zero here for clarity.
    _P[0][3] = 0.0f; _P[0][4] = 0.0f;
    _P[1][3] = 0.0f; _P[1][4] = 0.0f;
    _P[2][3] = 0.0f; _P[2][4] = 0.0f;
    _P[3][0] = 0.0f; _P[3][1] = 0.0f; _P[3][2] = 0.0f;
    _P[4][0] = 0.0f; _P[4][1] = 0.0f; _P[4][2] = 0.0f;

    // Velocity block: random-walk — add process noise scaled by dt_s.
    _P[3][3] += _Q[3][3] * dt_s;
    _P[3][4]  = 0.0f;
    _P[4][3]  = 0.0f;
    _P[4][4] += _Q[4][4] * dt_s;
}

// ---------------------------------------------------------------------------
// updatePosition — 2D position-only Kalman update (OTOS x, y observation).
//
// Observation model: H is 2x5 with H[0][0]=1, H[1][1]=1, rest zero.
//   So H*x = [_x[0], _x[1]]
//   P*H^T selects columns 0 and 1 of P:
//     (P*H^T)[i][0] = P[i][0]
//     (P*H^T)[i][1] = P[i][1]
//
// Innovation:        yi[i] = otos_meas[i] - _x[i]
// Innovation cov:    S = H*P*H^T + R  (2x2)
//                    S[0][0] = P[0][0] + _rOtosXy
//                    S[0][1] = P[0][1]
//                    S[1][0] = P[1][0]
//                    S[1][1] = P[1][1] + _rOtosXy
// Mahalanobis gate:  d2 = yi^T * S_inv * yi; if d2 > 5.99 reject.
// Kalman gain:       K = P*H^T * S_inv  (5x2)
// State update:      _x += K * yi
// Covariance update: P = (I - K*H) * P  (simplified: P[i][j] -= K[i][0]*P[0][j] + K[i][1]*P[1][j])
// ---------------------------------------------------------------------------

void EKF::updatePosition(float x_otos, float y_otos)
{
    // Innovation.
    float yi0 = x_otos - _x[0];
    float yi1 = y_otos - _x[1];

    // Innovation covariance S (2x2).
    float s00 = _P[0][0] + _rOtosXy;
    float s01 = _P[0][1];
    float s10 = _P[1][0];
    float s11 = _P[1][1] + _rOtosXy;

    // Analytic 2x2 inverse of S.
    float det = s00 * s11 - s01 * s10;
    if (det > -1e-9f && det < 1e-9f) {
        return;  // singular — skip update
    }
    float inv_det = 1.0f / det;
    float si00 =  s11 * inv_det;
    float si01 = -s01 * inv_det;
    float si10 = -s10 * inv_det;
    float si11 =  s00 * inv_det;

    // Mahalanobis gating: d2 = yi^T * S_inv * yi; chi-square 2-DOF threshold = 5.99.
    float d2 = yi0*(si00*yi0 + si01*yi1) + yi1*(si10*yi0 + si11*yi1);
    bool accepted = (d2 <= 5.99f);

    if (!accepted) {
        ++_rejected;
        ++_rejPos_streak;
        // D3 gate recovery (sprint 024-005): after 10 consecutive position
        // rejections, perform a P-inflation re-baseline and re-run the standard
        // update. P-inflation sets P[0][0] and P[1][1] to a large value
        // (kRebaselineP) and zeros the cross-terms P[0][1]/P[1][0], so that:
        //   S = P + R_normal ≈ kRebaselineP  (>>  innovation²)
        //   K = P/(P+R) ≈ 1  →  state snaps to OTOS in one update.
        // This satisfies the 200mm/<2s acceptance criterion.
        // R×10 inflation cannot pass a 200mm gate at steady-state P (math:
        //   d²=200²/(P+10·R)≫5.99 for P≈3mm², R=10mm²).
        // _rejPos_streak is independent of _rejHead_streak.
        if (_rejPos_streak >= 10) {
            _rejPos_streak = 0;
            // Inflate the position block of P so the next standard update snaps.
            static constexpr float kRebaselineP = 1.0e6f;  // mm² — K≈1
            _P[0][0] = kRebaselineP;
            _P[0][1] = 0.0f;
            _P[1][0] = 0.0f;
            _P[1][1] = kRebaselineP;
            // Zero position cross-covariances with heading and velocity blocks.
            _P[0][2] = 0.0f; _P[0][3] = 0.0f; _P[0][4] = 0.0f;
            _P[1][2] = 0.0f; _P[1][3] = 0.0f; _P[1][4] = 0.0f;
            _P[2][0] = 0.0f; _P[2][1] = 0.0f;
            _P[3][0] = 0.0f; _P[3][1] = 0.0f;
            _P[4][0] = 0.0f; _P[4][1] = 0.0f;
            // Recompute S and S_inv with inflated P — gate will easily pass.
            s00 = _P[0][0] + _rOtosXy;
            s01 = 0.0f;
            s10 = 0.0f;
            s11 = _P[1][1] + _rOtosXy;
            float detR = s00 * s11;   // s01=s10=0 → det = s00*s11
            if (detR < 1e-9f) {
                return;  // degenerate after inflation — skip (should never happen)
            }
            inv_det = 1.0f / detR;
            si00 =  s11 * inv_det;
            si01 = 0.0f;
            si10 = 0.0f;
            si11 =  s00 * inv_det;
            accepted = true;  // K≈1 path: gate is trivially satisfied
        }
        if (!accepted) {
            return;
        }
    } else {
        // Normal accept — reset streak.
        _rejPos_streak = 0;
    }

    // Kalman gain K = P*H^T * S_inv  (5x2).
    // P*H^T selects columns 0 and 1 of P.
    // K[i][0] = P[i][0]*si00 + P[i][1]*si10
    // K[i][1] = P[i][0]*si01 + P[i][1]*si11
    float k00 = _P[0][0]*si00 + _P[0][1]*si10;
    float k01 = _P[0][0]*si01 + _P[0][1]*si11;
    float k10 = _P[1][0]*si00 + _P[1][1]*si10;
    float k11 = _P[1][0]*si01 + _P[1][1]*si11;
    float k20 = _P[2][0]*si00 + _P[2][1]*si10;
    float k21 = _P[2][0]*si01 + _P[2][1]*si11;
    float k30 = _P[3][0]*si00 + _P[3][1]*si10;
    float k31 = _P[3][0]*si01 + _P[3][1]*si11;
    float k40 = _P[4][0]*si00 + _P[4][1]*si10;
    float k41 = _P[4][0]*si01 + _P[4][1]*si11;

    // State update: _x += K * yi.
    _x[0] += k00*yi0 + k01*yi1;
    _x[1] += k10*yi0 + k11*yi1;
    _x[2] += k20*yi0 + k21*yi1;
    _x[2]  = wrapPi(_x[2]);
    _x[3] += k30*yi0 + k31*yi1;
    _x[4] += k40*yi0 + k41*yi1;

    // Covariance update: P = (I - K*H) * P.
    // (I - K*H)[i][j] = delta(i,j) - K[i][0]*H[0][j] - K[i][1]*H[1][j]
    // H[0] has 1 at col 0, H[1] has 1 at col 1, rest zero.
    // => P_new[i][j] = P[i][j] - K[i][0]*P[0][j] - K[i][1]*P[1][j]
    float p00 = _P[0][0]; float p01 = _P[0][1]; float p02 = _P[0][2]; float p03 = _P[0][3]; float p04 = _P[0][4];
    float p10 = _P[1][0]; float p11 = _P[1][1]; float p12 = _P[1][2]; float p13 = _P[1][3]; float p14 = _P[1][4];
    float p20 = _P[2][0]; float p21 = _P[2][1]; float p22 = _P[2][2]; float p23 = _P[2][3]; float p24 = _P[2][4];
    float p30 = _P[3][0]; float p31 = _P[3][1]; float p32 = _P[3][2]; float p33 = _P[3][3]; float p34 = _P[3][4];
    float p40 = _P[4][0]; float p41 = _P[4][1]; float p42 = _P[4][2]; float p43 = _P[4][3]; float p44 = _P[4][4];

    _P[0][0] = p00 - k00*p00 - k01*p10;
    _P[0][1] = p01 - k00*p01 - k01*p11;
    _P[0][2] = p02 - k00*p02 - k01*p12;
    _P[0][3] = p03 - k00*p03 - k01*p13;
    _P[0][4] = p04 - k00*p04 - k01*p14;

    _P[1][0] = p10 - k10*p00 - k11*p10;
    _P[1][1] = p11 - k10*p01 - k11*p11;
    _P[1][2] = p12 - k10*p02 - k11*p12;
    _P[1][3] = p13 - k10*p03 - k11*p13;
    _P[1][4] = p14 - k10*p04 - k11*p14;

    _P[2][0] = p20 - k20*p00 - k21*p10;
    _P[2][1] = p21 - k20*p01 - k21*p11;
    _P[2][2] = p22 - k20*p02 - k21*p12;
    _P[2][3] = p23 - k20*p03 - k21*p13;
    _P[2][4] = p24 - k20*p04 - k21*p14;

    _P[3][0] = p30 - k30*p00 - k31*p10;
    _P[3][1] = p31 - k30*p01 - k31*p11;
    _P[3][2] = p32 - k30*p02 - k31*p12;
    _P[3][3] = p33 - k30*p03 - k31*p13;
    _P[3][4] = p34 - k30*p04 - k31*p14;

    _P[4][0] = p40 - k40*p00 - k41*p10;
    _P[4][1] = p41 - k40*p01 - k41*p11;
    _P[4][2] = p42 - k40*p02 - k41*p12;
    _P[4][3] = p43 - k40*p03 - k41*p13;
    _P[4][4] = p44 - k40*p04 - k41*p14;
}

// ---------------------------------------------------------------------------
// updateVelocity — fuse v and omega as two sequential scalar 1-DOF updates.
//
// For linear velocity (j=3, H_v = [0,0,0,1,0]):
//   innovation:     yv = v_meas - _x[3]
//   innov cov:      s_v = P[3][3] + r_v
//   gate:           yv^2 / s_v > 3.84 → skip (chi-square 1-DOF, p=0.05)
//   gain:           K_v[i] = P[i][3] / s_v   for i=0..4
//   state update:   _x[i] += K_v[i] * yv
//   cov update:     P[i][k] -= K_v[i] * P[3][k]   for k=0..4
//
// For angular velocity (j=4, H_w = [0,0,0,0,1]):
//   Same pattern using P[4][4] + r_omega, P[i][4], P[4][k].
//   Note: after the v update, P has changed; the omega update sees the
//   post-v-update P, which is correct for sequential scalar updates.
// ---------------------------------------------------------------------------

void EKF::updateVelocity(float v_meas, float omega_meas, float r_v, float r_omega)
{
    // --- Fuse linear velocity (state index 3) ---
    {
        float yv  = v_meas - _x[3];
        float s_v = _P[3][3] + r_v;

        if (s_v > 1e-12f && (yv * yv / s_v) <= 3.84f) {
            // Kalman gain for this scalar update.
            float kv0 = _P[0][3] / s_v;
            float kv1 = _P[1][3] / s_v;
            float kv2 = _P[2][3] / s_v;
            float kv3 = _P[3][3] / s_v;
            float kv4 = _P[4][3] / s_v;

            // State update.
            _x[0] += kv0 * yv;
            _x[1] += kv1 * yv;
            _x[2] += kv2 * yv;
            _x[2]  = wrapPi(_x[2]);
            _x[3] += kv3 * yv;
            _x[4] += kv4 * yv;

            // Covariance update: P[i][k] -= K[i] * P[3][k].
            float p3k0 = _P[3][0]; float p3k1 = _P[3][1]; float p3k2 = _P[3][2];
            float p3k3 = _P[3][3]; float p3k4 = _P[3][4];

            _P[0][0] -= kv0 * p3k0; _P[0][1] -= kv0 * p3k1; _P[0][2] -= kv0 * p3k2; _P[0][3] -= kv0 * p3k3; _P[0][4] -= kv0 * p3k4;
            _P[1][0] -= kv1 * p3k0; _P[1][1] -= kv1 * p3k1; _P[1][2] -= kv1 * p3k2; _P[1][3] -= kv1 * p3k3; _P[1][4] -= kv1 * p3k4;
            _P[2][0] -= kv2 * p3k0; _P[2][1] -= kv2 * p3k1; _P[2][2] -= kv2 * p3k2; _P[2][3] -= kv2 * p3k3; _P[2][4] -= kv2 * p3k4;
            _P[3][0] -= kv3 * p3k0; _P[3][1] -= kv3 * p3k1; _P[3][2] -= kv3 * p3k2; _P[3][3] -= kv3 * p3k3; _P[3][4] -= kv3 * p3k4;
            _P[4][0] -= kv4 * p3k0; _P[4][1] -= kv4 * p3k1; _P[4][2] -= kv4 * p3k2; _P[4][3] -= kv4 * p3k3; _P[4][4] -= kv4 * p3k4;
        } else if (s_v <= 1e-12f) {
            // Degenerate: skip.
        } else {
            ++_rejected;
        }
    }

    // --- Fuse angular velocity (state index 4) ---
    {
        float yw    = omega_meas - _x[4];
        float s_w   = _P[4][4] + r_omega;

        if (s_w > 1e-12f && (yw * yw / s_w) <= 3.84f) {
            // Kalman gain for this scalar update.
            float kw0 = _P[0][4] / s_w;
            float kw1 = _P[1][4] / s_w;
            float kw2 = _P[2][4] / s_w;
            float kw3 = _P[3][4] / s_w;
            float kw4 = _P[4][4] / s_w;

            // State update.
            _x[0] += kw0 * yw;
            _x[1] += kw1 * yw;
            _x[2] += kw2 * yw;
            _x[2]  = wrapPi(_x[2]);
            _x[3] += kw3 * yw;
            _x[4] += kw4 * yw;

            // Covariance update: P[i][k] -= K[i] * P[4][k].
            float p4k0 = _P[4][0]; float p4k1 = _P[4][1]; float p4k2 = _P[4][2];
            float p4k3 = _P[4][3]; float p4k4 = _P[4][4];

            _P[0][0] -= kw0 * p4k0; _P[0][1] -= kw0 * p4k1; _P[0][2] -= kw0 * p4k2; _P[0][3] -= kw0 * p4k3; _P[0][4] -= kw0 * p4k4;
            _P[1][0] -= kw1 * p4k0; _P[1][1] -= kw1 * p4k1; _P[1][2] -= kw1 * p4k2; _P[1][3] -= kw1 * p4k3; _P[1][4] -= kw1 * p4k4;
            _P[2][0] -= kw2 * p4k0; _P[2][1] -= kw2 * p4k1; _P[2][2] -= kw2 * p4k2; _P[2][3] -= kw2 * p4k3; _P[2][4] -= kw2 * p4k4;
            _P[3][0] -= kw3 * p4k0; _P[3][1] -= kw3 * p4k1; _P[3][2] -= kw3 * p4k2; _P[3][3] -= kw3 * p4k3; _P[3][4] -= kw3 * p4k4;
            _P[4][0] -= kw4 * p4k0; _P[4][1] -= kw4 * p4k1; _P[4][2] -= kw4 * p4k2; _P[4][3] -= kw4 * p4k3; _P[4][4] -= kw4 * p4k4;
        } else if (s_w <= 1e-12f) {
            // Degenerate: skip.
        } else {
            ++_rejected;
        }
    }
}

// ---------------------------------------------------------------------------
// updateHeading — fuse OTOS heading as a scalar (1-DOF) Kalman update.
//
// Observation model: H = [0,0,1,0,0]  (observes only state index 2, theta).
//   P*H^T selects column 2 of P: (P*H^T)[i] = P[i][2]
//
// Innovation (wrap-safe):  y = wrapPi(theta_meas - _x[2])
// Innovation covariance:   s = P[2][2] + r_theta
// Mahalanobis gate:        y^2 / s > 3.84 → reject (chi-square 1-DOF, p=0.05)
// Kalman gain:             K[i] = P[i][2] / s
// State update:            _x[i] += K[i] * y
// Covariance update:       P[i][k] -= K[i] * P[2][k]   for k=0..4
//
// _rejHead_streak: increments on each rejection; resets to 0 on each accept.
// D3 gate recovery (sprint 024-005): at 10 consecutive rejections, inflate
// R×10 for one update and reset the streak. This converts "heading locked out
// forever" to "recovers within ~1 s at 100 ms OTOS cadence".
// _rejHead_streak is independent of _rejPos_streak (position divergence does
// not trigger heading recovery and vice versa).
// ---------------------------------------------------------------------------

void EKF::updateHeading(float theta_meas, float r_theta)
{
    // Wrap-safe innovation.
    float y = wrapPi(theta_meas - _x[2]);
    float s = _P[2][2] + r_theta;

    if (s <= 1e-12f) {
        return;  // degenerate — skip silently
    }

    bool accepted = (y * y / s) <= 3.84f;

    if (!accepted) {
        ++_rejected;
        ++_rejHead_streak;
        // D3 gate recovery (sprint 024-005): at streak == 10, perform a P-inflation
        // re-baseline on the heading block. Sets P[2][2] to a large value so that
        // S = P[2][2] + r_theta is large, gate passes trivially, and K = P[2][2]/S ≈ 1
        // — heading state snaps to the measurement in one update.
        // R×10 inflation cannot satisfy the 200mm/<2s AC for large divergences.
        if (_rejHead_streak >= 10) {
            _rejHead_streak = 0;
            static constexpr float kRebaselinePTheta = 1.0e5f;  // rad² — K≈1
            // Inflate heading variance; zero cross-covariances with x,y,v,omega.
            _P[2][2] = kRebaselinePTheta;
            _P[2][0] = 0.0f; _P[2][1] = 0.0f; _P[2][3] = 0.0f; _P[2][4] = 0.0f;
            _P[0][2] = 0.0f; _P[1][2] = 0.0f; _P[3][2] = 0.0f; _P[4][2] = 0.0f;
            // Recompute s with inflated P[2][2].
            s = _P[2][2] + r_theta;
            accepted = (s > 1e-12f);  // gate trivially passes
        }
        if (!accepted) {
            return;  // degenerate after inflation — skip (should never happen)
        }
    }

    // Accepted (normal or recovery path).
    _rejHead_streak = 0;

    // Kalman gain: K[i] = P[i][2] / s.
    float k0 = _P[0][2] / s;
    float k1 = _P[1][2] / s;
    float k2 = _P[2][2] / s;
    float k3 = _P[3][2] / s;
    float k4 = _P[4][2] / s;

    // State update.
    _x[0] += k0 * y;
    _x[1] += k1 * y;
    _x[2] += k2 * y;
    _x[2]  = wrapPi(_x[2]);
    _x[3] += k3 * y;
    _x[4] += k4 * y;

    // Covariance update: P[i][k] -= K[i] * P[2][k].
    float p2k0 = _P[2][0]; float p2k1 = _P[2][1]; float p2k2 = _P[2][2];
    float p2k3 = _P[2][3]; float p2k4 = _P[2][4];

    _P[0][0] -= k0 * p2k0; _P[0][1] -= k0 * p2k1; _P[0][2] -= k0 * p2k2; _P[0][3] -= k0 * p2k3; _P[0][4] -= k0 * p2k4;
    _P[1][0] -= k1 * p2k0; _P[1][1] -= k1 * p2k1; _P[1][2] -= k1 * p2k2; _P[1][3] -= k1 * p2k3; _P[1][4] -= k1 * p2k4;
    _P[2][0] -= k2 * p2k0; _P[2][1] -= k2 * p2k1; _P[2][2] -= k2 * p2k2; _P[2][3] -= k2 * p2k3; _P[2][4] -= k2 * p2k4;
    _P[3][0] -= k3 * p2k0; _P[3][1] -= k3 * p2k1; _P[3][2] -= k3 * p2k2; _P[3][3] -= k3 * p2k3; _P[3][4] -= k3 * p2k4;
    _P[4][0] -= k4 * p2k0; _P[4][1] -= k4 * p2k1; _P[4][2] -= k4 * p2k2; _P[4][3] -= k4 * p2k3; _P[4][4] -= k4 * p2k4;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

float    EKF::x()              const { return _x[0]; }
float    EKF::y()              const { return _x[1]; }
float    EKF::theta()          const { return _x[2]; }
float    EKF::v()              const { return _x[3]; }
float    EKF::omega()          const { return _x[4]; }
uint32_t EKF::rejectedCount()  const { return _rejected; }
int      EKF::getRejectCount() const { return (int)_rejected; }
int      EKF::rejHeadStreak()  const { return _rejHead_streak; }
int      EKF::rejPosStreak()   const { return _rejPos_streak; }

// ---------------------------------------------------------------------------
// wrapPi — wrap angle to (-pi, pi] using atan2f identity.
// ---------------------------------------------------------------------------

float EKF::wrapPi(float theta)
{
    return atan2f(sinf(theta), cosf(theta));
}
