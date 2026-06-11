#pragma once
#include <math.h>
#include <stdint.h>

// ===========================================================================
// EKF — 5-state Extended Kalman Filter for pose and velocity fusion
//
// State vector: x = [x_mm, y_mm, theta_rad, v_mmps, omega_rads]
//
// Motion model (predict):
//   Position block [0..2]: arc-segment (midpoint integration), unchanged from
//     sprint 022. F[0][2] = -dCenter*sin(theta_mid), F[1][2] = dCenter*cos(theta_mid).
//   Velocity block [3..4]: random-walk identity sub-Jacobian.
//     v_{k+1} = v_k + w_v,  omega_{k+1} = omega_k + w_omega.
//   Block-decoupled: cross-block Jacobian entries are zero; cross-block P entries
//     are initialized to zero and remain zero throughout.
//
// Update channels:
//   updatePosition(x_otos, y_otos): 2D position observation, H is 2x5.
//     Mahalanobis gating: chi-square 2-DOF threshold 5.99.
//   updateVelocity(v_meas, omega_meas, r_v, r_omega): two sequential 1D
//     scalar Kalman updates. Gate threshold: chi-square 1-DOF = 3.84.
//   updateHeading(theta_meas, r_theta): scalar heading observation.
//     H = [0,0,1,0,0]; wrap-safe innovation y = wrapPi(theta_meas - _x[2]);
//     Mahalanobis gate: chi-square 1-DOF threshold 3.84.
//
// All matrix operations (5x5, 5x2, 5x1) are fully unrolled as plain float
// arithmetic. No heap allocation, no STL, no Eigen.
//
// Sprint 023, Ticket 001.
// Sprint 024, Ticket 004: added updateHeading(); sane P-prior in setPose().
// Sprint 024, Ticket 005: _rejPos_streak + P-inflation re-baseline recovery;
//   getRejectCount(). Architecture deviation: original design called for R×10
//   inflation, but math shows d²=200²/(P+10·R)≫5.99 even at R×10 — still
//   permanently rejected. P-inflation (P→1e6 mm² / r_theta×1e5) instead widens
//   S so the standard gate passes (d²≈0) and K≈1, snapping state to OTOS in
//   one update. This is the only mechanism that satisfies the 200mm/<2s AC.
// ===========================================================================

class EKF {
public:
    EKF();

    // Initialize noise parameters and reset state to origin.
    //   q_xy      — process noise variance for x and y (mm^2)
    //   q_theta   — process noise variance for heading (rad^2)
    //   q_v       — process noise variance for linear velocity (mm/s)^2
    //   q_omega   — process noise variance for angular velocity (rad/s)^2
    //   r_otos_xy — OTOS measurement noise variance for x and y (mm^2)
    //   r_otos_v  — OTOS measurement noise variance for linear velocity (mm/s)^2
    //   r_enc_v   — encoder measurement noise variance for linear velocity (mm/s)^2
    void init(float q_xy, float q_theta, float q_v, float q_omega,
              float r_otos_xy, float r_otos_v, float r_enc_v);

    // Overwrite state with a known pose; zeroes v and omega; sets sane diagonal
    // P-prior (100 mm², 100 mm², (5°)², v-vars) instead of zeroing P.
    // A zeroed P after setPose() would create falsely tight Mahalanobis gates
    // and strangle re-acquisition after pose injection.
    void setPose(float x, float y, float theta);

    // Predict step: arc-segment motion model for position block; random-walk
    // for velocity block. dt_s is the timestep in seconds (passed through for
    // future full-coupling extension; not used in position block).
    //   dCenter      — distance traveled by the center point (mm)
    //   dTheta       — change in heading (rad)
    //   theta_before — heading at the start of this step (rad)
    //   dt_s         — timestep in seconds
    void predict(float dCenter, float dTheta, float theta_before, float dt_s);

    // Update step: 2D position-only observation from OTOS.
    // Applies Mahalanobis gating (chi-square 2-DOF threshold 5.99).
    void updatePosition(float x_otos, float y_otos);

    // Update step: fuse linear and angular velocity measurements.
    // Performs two sequential scalar (1-DOF) Kalman updates.
    // Each is independently gated (chi-square 1-DOF threshold 3.84).
    //   v_meas    — measured linear velocity (mm/s)
    //   omega_meas — measured angular velocity (rad/s)
    //   r_v       — measurement noise variance for v (mm/s)^2
    //   r_omega   — measurement noise variance for omega (rad/s)^2
    void updateVelocity(float v_meas, float omega_meas, float r_v, float r_omega);

    // Update step: fuse OTOS heading measurement (scalar 1-DOF update).
    // H = [0,0,1,0,0]; innovation is wrap-safe: y = wrapPi(theta_meas - _x[2]).
    // Mahalanobis gate: chi-square 1-DOF threshold 3.84.
    //   theta_meas — measured heading (rad), e.g. OTOS p.h
    //   r_theta    — heading measurement noise variance (rad^2)
    void updateHeading(float theta_meas, float r_theta);

    // Accessors
    float    x()            const;
    float    y()            const;
    float    theta()        const;
    float    v()            const;
    float    omega()        const;
    uint32_t rejectedCount() const;
    int      getRejectCount() const;   // alias for TLM: same as rejectedCount()
    int      rejHeadStreak() const;
    int      rejPosStreak()  const;

private:
    float    _x[5];       // state: [x_mm, y_mm, theta_rad, v_mmps, omega_rads]
    float    _P[5][5];    // covariance matrix (5x5)
    float    _Q[5][5];    // process noise (diagonal)
    float    _rOtosXy;    // OTOS position noise variance (same for x and y)
    float    _rOtosV;     // OTOS velocity noise variance
    float    _rEncV;      // encoder velocity noise variance
    uint32_t _rejected;       // cumulative count of gated (rejected) observations
    int      _rejHead_streak; // consecutive heading-update rejection streak (D3 gate recovery)
    int      _rejPos_streak;  // consecutive position-update rejection streak (D3 gate recovery)

    // Wrap angle to (-pi, pi] using atan2f identity.
    // Form: atan2f(sinf(theta), cosf(theta)) — exact match with Python mirror's
    // math.atan2(math.sin, math.cos) and with Odometry::wrapPi().
    static float wrapPi(float theta);

    // Sane P-prior diagonal values used by setPose() (sprint 024-004).
    static constexpr float kPriorXY    = 100.0f;          // mm^2
    static constexpr float kPriorTheta = 0.00762f;        // (5 deg in rad)^2 ≈ (5*pi/180)^2
    static constexpr float kPriorV     = 100.0f;          // (mm/s)^2
    static constexpr float kPriorOmega = 0.01f;           // (rad/s)^2
};
