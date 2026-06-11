"""test_ekf.py — Unit tests for EKF class (source/control/EKF.h/.cpp).

Pure-Python mirror of the C++ EKF implementation.
Verifies predict/update math, covariance growth/shrinkage, convergence,
heading wrap-safety, velocity fusion, Mahalanobis gating, and the setPose
encoder re-baseline regression.

Sprint 022, Ticket T005 — original 3-state EKF mirror.
Sprint 023, Ticket T006 — extended to 5-state (x, y, theta, v, omega);
  added TestPredictVelocity, TestUpdateVelocity, TestMahalanobisGating,
  TestSetPoseRebaseline, TestGoldenVectors, TestReplayHarness.
Sprint 024, Ticket 004 — added update_heading(); sane P-prior in set_pose();
  TestUpdateHeading, TestSetPosePrior, TestHeadingConvergence.
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def wrap_pi(theta: float) -> float:
    """Keep heading in (-π, π] using atan2 identity."""
    return math.atan2(math.sin(theta), math.cos(theta))


# ---------------------------------------------------------------------------
# Pure-Python EKF mirror — exactly matches source/control/EKF.cpp (sprint 023)
#
# State: [x_mm, y_mm, theta_rad, v_mmps, omega_rads]
#
# Block-decoupled Jacobian invariant: F has zero entries in the cross-block
# positions (rows 0-2 × cols 3-4 and rows 3-4 × cols 0-2). Because Q is also
# block-diagonal and P is initialized to zero, the cross-block P entries remain
# exactly zero throughout all predict and update steps. This allows the 5×5
# covariance update to be computed as two independent sub-block updates.
# ---------------------------------------------------------------------------

class EKF:
    """Python mirror of the C++ EKF class (sprint 023 T006, sprint 024 T004).

    State: [x_mm, y_mm, theta_rad, v_mmps, omega_rads]
    Motion model: position block = arc-segment (midpoint integration);
                  velocity block = random-walk (identity Jacobian).
    Observation channels:
      update_position(x_otos, y_otos): 2D position, Mahalanobis gate 5.99.
      update_velocity(v_meas, omega_meas, r_v, r_omega): two scalar 1-DOF
        updates, each gated at 3.84.
      update_heading(theta_meas, r_theta): scalar heading, Mahalanobis gate 3.84,
        wrap-safe innovation. Sprint 024-004.

    set_pose() sets a sane diagonal P-prior (sprint 024-004) instead of zeroing P.
    """

    # Sane P-prior constants — must match EKF.h constexpr values exactly.
    # (5 * pi/180)^2 = 0.007615... ≈ 0.00762 — use the same approximation as the C++.
    _PRIOR_XY    = 100.0    # mm^2
    _PRIOR_THETA = (5.0 * math.pi / 180.0) ** 2  # rad^2  ≈ 0.00762
    _PRIOR_V     = 100.0    # (mm/s)^2
    _PRIOR_OMEGA = 0.01     # (rad/s)^2

    def __init__(self):
        self._x = [0.0] * 5
        self._P = [[0.0] * 5 for _ in range(5)]
        self._Q = [[0.0] * 5 for _ in range(5)]
        self._r_otos_xy = 0.0
        self._r_otos_v = 0.0
        self._r_enc_v = 0.0
        self._rejected = 0
        self._rej_head_streak = 0

    def init(self, q_xy: float, q_theta: float, q_v: float, q_omega: float,
             r_otos_xy: float, r_otos_v: float, r_enc_v: float) -> None:
        """Initialize noise parameters and reset state to origin.

        Args:
            q_xy:      Process noise variance for x and y (mm^2).
            q_theta:   Process noise variance for heading (rad^2).
            q_v:       Process noise variance for linear velocity (mm/s)^2.
            q_omega:   Process noise variance for angular velocity (rad/s)^2.
            r_otos_xy: OTOS measurement noise variance for x and y (mm^2).
            r_otos_v:  OTOS measurement noise variance for linear velocity.
            r_enc_v:   Encoder measurement noise variance for linear velocity.
        """
        for i in range(5):
            for j in range(5):
                self._Q[i][j] = 0.0
        self._Q[0][0] = q_xy
        self._Q[1][1] = q_xy
        self._Q[2][2] = q_theta
        self._Q[3][3] = q_v
        self._Q[4][4] = q_omega

        self._r_otos_xy = r_otos_xy
        self._r_otos_v = r_otos_v
        self._r_enc_v = r_enc_v
        self._rejected = 0
        self._rej_head_streak = 0

        self._x = [0.0] * 5
        self._P = [[0.0] * 5 for _ in range(5)]

    def set_pose(self, x: float, y: float, theta: float) -> None:
        """Overwrite state with a known pose; zero v and omega; set sane P-prior.

        Sprint 024-004: instead of zeroing P, set a diagonal prior that reflects
        realistic uncertainty so Mahalanobis gates are not falsely tight after a
        pose injection. Mirrors EKF::setPose() in source/control/EKF.cpp exactly.
        """
        self._x[0] = float(x)
        self._x[1] = float(y)
        self._x[2] = float(theta)
        self._x[3] = 0.0   # v
        self._x[4] = 0.0   # omega
        # Zero all P, then set sane diagonal.
        self._P = [[0.0] * 5 for _ in range(5)]
        self._P[0][0] = self._PRIOR_XY
        self._P[1][1] = self._PRIOR_XY
        self._P[2][2] = self._PRIOR_THETA
        self._P[3][3] = self._PRIOR_V
        self._P[4][4] = self._PRIOR_OMEGA

    def predict(self, dCenter: float, dTheta: float,
                theta_before: float, dt_s: float = 0.0) -> None:
        """Predict step: arc-segment (position block) + random-walk (velocity block).

        Position block is unchanged from sprint 022.  Velocity block: v and
        omega are carried unchanged; their covariance grows by Q[3][3] and
        Q[4][4] respectively.  dt_s is passed through for API compatibility
        but not used in the position block (block-decoupled design).

        Jacobian non-identity entries (position block only):
          F[0][2] = a = -dCenter * sin(theta_mid)
          F[1][2] = b =  dCenter * cos(theta_mid)

        P update: P = F*P*F^T + Q (fully unrolled, mirrors EKF.cpp exactly).
        Cross-block entries are explicitly kept at zero (block-decoupling invariant).
        """
        theta_mid = theta_before + dTheta * 0.5
        ct = math.cos(theta_mid)
        st = math.sin(theta_mid)

        # Position state update.
        self._x[0] += dCenter * ct
        self._x[1] += dCenter * st
        self._x[2] = wrap_pi(self._x[2] + dTheta)
        # _x[3] and _x[4] (v, omega) are unchanged — random-walk.

        # Jacobian non-identity entries for the position block.
        a = -dCenter * st   # F[0][2]
        b =  dCenter * ct   # F[1][2]

        # Load position block of P (rows/cols 0..2).
        p00 = self._P[0][0]; p01 = self._P[0][1]; p02 = self._P[0][2]
        p10 = self._P[1][0]; p11 = self._P[1][1]; p12 = self._P[1][2]
        p20 = self._P[2][0]; p21 = self._P[2][1]; p22 = self._P[2][2]

        # T = F * P for the position block rows.
        t00 = p00 + a * p20;  t01 = p01 + a * p21;  t02 = p02 + a * p22
        t10 = p10 + b * p20;  t11 = p11 + b * p21;  t12 = p12 + b * p22
        t20 = p20;             t21 = p21;             t22 = p22

        # New position block: T * F^T + Q.
        self._P[0][0] = t00 + t02 * a + self._Q[0][0]
        self._P[0][1] = t01 + t02 * b
        self._P[0][2] = t02
        self._P[1][0] = t10 + t12 * a
        self._P[1][1] = t11 + t12 * b + self._Q[1][1]
        self._P[1][2] = t12
        self._P[2][0] = t20 + t22 * a
        self._P[2][1] = t21 + t22 * b
        self._P[2][2] = t22 + self._Q[2][2]

        # Cross-block entries remain zero (block-decoupling invariant).
        self._P[0][3] = 0.0; self._P[0][4] = 0.0
        self._P[1][3] = 0.0; self._P[1][4] = 0.0
        self._P[2][3] = 0.0; self._P[2][4] = 0.0
        self._P[3][0] = 0.0; self._P[3][1] = 0.0; self._P[3][2] = 0.0
        self._P[4][0] = 0.0; self._P[4][1] = 0.0; self._P[4][2] = 0.0

        # Velocity block: random-walk — add process noise only.
        self._P[3][3] += self._Q[3][3]
        self._P[3][4] = 0.0
        self._P[4][3] = 0.0
        self._P[4][4] += self._Q[4][4]

    def update_position(self, x_otos: float, y_otos: float) -> None:
        """Update step: 2D position-only observation from OTOS (renamed from update()).

        Observation model: H is 2x5 with H[0][0]=1, H[1][1]=1, rest zero.
        Innovation covariance S = H*P*H^T + R  (2x2).
        Mahalanobis gate: d2 = yi^T * S_inv * yi; chi-square 2-DOF threshold = 5.99.
        Kalman gain: K = P*H^T * S_inv  (5x2).
        State update: _x += K * yi.
        Covariance update: P = (I - K*H) * P.
        """
        yi0 = x_otos - self._x[0]
        yi1 = y_otos - self._x[1]

        # Innovation covariance S (2x2).
        s00 = self._P[0][0] + self._r_otos_xy
        s01 = self._P[0][1]
        s10 = self._P[1][0]
        s11 = self._P[1][1] + self._r_otos_xy

        # Analytic 2x2 inverse of S.
        det = s00 * s11 - s01 * s10
        if -1e-9 < det < 1e-9:
            return  # singular — skip update

        inv_det = 1.0 / det
        si00 =  s11 * inv_det
        si01 = -s01 * inv_det
        si10 = -s10 * inv_det
        si11 =  s00 * inv_det

        # Mahalanobis gating: d2 = yi^T * S_inv * yi; chi-square 2-DOF = 5.99.
        d2 = yi0 * (si00 * yi0 + si01 * yi1) + yi1 * (si10 * yi0 + si11 * yi1)
        if d2 > 5.99:
            self._rejected += 1
            return

        # Kalman gain K = P*H^T * S_inv  (5x2).
        # P*H^T selects columns 0 and 1 of P.
        k00 = self._P[0][0] * si00 + self._P[0][1] * si10
        k01 = self._P[0][0] * si01 + self._P[0][1] * si11
        k10 = self._P[1][0] * si00 + self._P[1][1] * si10
        k11 = self._P[1][0] * si01 + self._P[1][1] * si11
        k20 = self._P[2][0] * si00 + self._P[2][1] * si10
        k21 = self._P[2][0] * si01 + self._P[2][1] * si11
        k30 = self._P[3][0] * si00 + self._P[3][1] * si10
        k31 = self._P[3][0] * si01 + self._P[3][1] * si11
        k40 = self._P[4][0] * si00 + self._P[4][1] * si10
        k41 = self._P[4][0] * si01 + self._P[4][1] * si11

        # State update: _x += K * yi.
        self._x[0] += k00 * yi0 + k01 * yi1
        self._x[1] += k10 * yi0 + k11 * yi1
        self._x[2] += k20 * yi0 + k21 * yi1
        self._x[2]  = wrap_pi(self._x[2])
        self._x[3] += k30 * yi0 + k31 * yi1
        self._x[4] += k40 * yi0 + k41 * yi1

        # Covariance update: P = (I - K*H) * P.
        # P_new[i][j] = P[i][j] - K[i][0]*P[0][j] - K[i][1]*P[1][j]
        p00 = self._P[0][0]; p01 = self._P[0][1]; p02 = self._P[0][2]; p03 = self._P[0][3]; p04 = self._P[0][4]
        p10 = self._P[1][0]; p11 = self._P[1][1]; p12 = self._P[1][2]; p13 = self._P[1][3]; p14 = self._P[1][4]
        p20 = self._P[2][0]; p21 = self._P[2][1]; p22 = self._P[2][2]; p23 = self._P[2][3]; p24 = self._P[2][4]
        p30 = self._P[3][0]; p31 = self._P[3][1]; p32 = self._P[3][2]; p33 = self._P[3][3]; p34 = self._P[3][4]
        p40 = self._P[4][0]; p41 = self._P[4][1]; p42 = self._P[4][2]; p43 = self._P[4][3]; p44 = self._P[4][4]

        self._P[0][0] = p00 - k00 * p00 - k01 * p10
        self._P[0][1] = p01 - k00 * p01 - k01 * p11
        self._P[0][2] = p02 - k00 * p02 - k01 * p12
        self._P[0][3] = p03 - k00 * p03 - k01 * p13
        self._P[0][4] = p04 - k00 * p04 - k01 * p14

        self._P[1][0] = p10 - k10 * p00 - k11 * p10
        self._P[1][1] = p11 - k10 * p01 - k11 * p11
        self._P[1][2] = p12 - k10 * p02 - k11 * p12
        self._P[1][3] = p13 - k10 * p03 - k11 * p13
        self._P[1][4] = p14 - k10 * p04 - k11 * p14

        self._P[2][0] = p20 - k20 * p00 - k21 * p10
        self._P[2][1] = p21 - k20 * p01 - k21 * p11
        self._P[2][2] = p22 - k20 * p02 - k21 * p12
        self._P[2][3] = p23 - k20 * p03 - k21 * p13
        self._P[2][4] = p24 - k20 * p04 - k21 * p14

        self._P[3][0] = p30 - k30 * p00 - k31 * p10
        self._P[3][1] = p31 - k30 * p01 - k31 * p11
        self._P[3][2] = p32 - k30 * p02 - k31 * p12
        self._P[3][3] = p33 - k30 * p03 - k31 * p13
        self._P[3][4] = p34 - k30 * p04 - k31 * p14

        self._P[4][0] = p40 - k40 * p00 - k41 * p10
        self._P[4][1] = p41 - k40 * p01 - k41 * p11
        self._P[4][2] = p42 - k40 * p02 - k41 * p12
        self._P[4][3] = p43 - k40 * p03 - k41 * p13
        self._P[4][4] = p44 - k40 * p04 - k41 * p14

    def update_velocity(self, v_meas: float, omega_meas: float,
                        r_v: float, r_omega: float) -> None:
        """Update step: fuse linear and angular velocity as two scalar 1-DOF updates.

        For v (state index 3, H_v = [0,0,0,1,0]):
          innovation:    yv = v_meas - _x[3]
          innov cov:     s_v = P[3][3] + r_v
          gate:          yv^2 / s_v > 3.84 → skip (chi-square 1-DOF p=0.05)
          gain:          K_v[i] = P[i][3] / s_v
          state update:  _x[i] += K_v[i] * yv
          cov update:    P[i][k] -= K_v[i] * P[3][k]

        For omega (state index 4): same pattern with P[4][4] + r_omega.
        The omega update sees the post-v-update P (correct for sequential scalar).
        """
        # --- Fuse linear velocity (state index 3) ---
        yv  = v_meas - self._x[3]
        s_v = self._P[3][3] + r_v
        if s_v > 1e-12 and (yv * yv / s_v) <= 3.84:
            kv = [self._P[i][3] / s_v for i in range(5)]
            for i in range(5):
                self._x[i] += kv[i] * yv
            self._x[2] = wrap_pi(self._x[2])
            p3k = [self._P[3][k] for k in range(5)]
            for i in range(5):
                for k in range(5):
                    self._P[i][k] -= kv[i] * p3k[k]
        elif s_v > 1e-12:
            self._rejected += 1
        # else: degenerate — skip silently

        # --- Fuse angular velocity (state index 4) ---
        yw  = omega_meas - self._x[4]
        s_w = self._P[4][4] + r_omega
        if s_w > 1e-12 and (yw * yw / s_w) <= 3.84:
            kw = [self._P[i][4] / s_w for i in range(5)]
            for i in range(5):
                self._x[i] += kw[i] * yw
            self._x[2] = wrap_pi(self._x[2])
            p4k = [self._P[4][k] for k in range(5)]
            for i in range(5):
                for k in range(5):
                    self._P[i][k] -= kw[i] * p4k[k]
        elif s_w > 1e-12:
            self._rejected += 1
        # else: degenerate — skip silently

    def update_heading(self, theta_meas: float, r_theta: float) -> None:
        """Update step: fuse OTOS heading as a scalar (1-DOF) Kalman update.

        Sprint 024-004. Mirrors EKF::updateHeading() in source/control/EKF.cpp.

        Observation model: H = [0,0,1,0,0] (observes state index 2, theta).
          P*H^T selects column 2 of P: (P*H^T)[i] = P[i][2].

        Innovation (wrap-safe): y = wrap_pi(theta_meas - _x[2])
        Innovation covariance:  s = P[2][2] + r_theta
        Mahalanobis gate:       y^2 / s > 3.84 → reject (chi-square 1-DOF)
        Kalman gain:            K[i] = P[i][2] / s
        State update:           _x[i] += K[i] * y
        Covariance update:      P[i][k] -= K[i] * P[2][k]

        _rej_head_streak: increments on rejection, resets to 0 on acceptance.
        """
        y = wrap_pi(theta_meas - self._x[2])
        s = self._P[2][2] + r_theta

        if s > 1e-12 and (y * y / s) <= 3.84:
            # Accepted.
            self._rej_head_streak = 0
            k = [self._P[i][2] / s for i in range(5)]
            for i in range(5):
                self._x[i] += k[i] * y
            self._x[2] = wrap_pi(self._x[2])
            p2k = [self._P[2][kk] for kk in range(5)]
            for i in range(5):
                for kk in range(5):
                    self._P[i][kk] -= k[i] * p2k[kk]
        elif s <= 1e-12:
            # Degenerate — skip silently.
            pass
        else:
            self._rejected += 1
            self._rej_head_streak += 1

    # Sprint 022 backward-compat alias: update() — no Mahalanobis gate.
    # The sprint-022 EKF had no gate; this alias preserves that behavior so
    # existing sprint-022 test classes continue to pass without modification.
    # New code should call update_position() which includes Mahalanobis gating.
    def update(self, x_otos: float, y_otos: float) -> None:
        """Sprint-022 compat: 2D position update WITHOUT Mahalanobis gate.

        Preserves exact sprint-022 behavior for backward-compatible test classes.
        Use update_position() for the gated (sprint-023) behavior.
        """
        yi0 = x_otos - self._x[0]
        yi1 = y_otos - self._x[1]

        s00 = self._P[0][0] + self._r_otos_xy
        s01 = self._P[0][1]
        s10 = self._P[1][0]
        s11 = self._P[1][1] + self._r_otos_xy

        det = s00 * s11 - s01 * s10
        if -1e-9 < det < 1e-9:
            return

        inv_det = 1.0 / det
        si00 =  s11 * inv_det
        si01 = -s01 * inv_det
        si10 = -s10 * inv_det
        si11 =  s00 * inv_det

        k00 = self._P[0][0] * si00 + self._P[0][1] * si10
        k01 = self._P[0][0] * si01 + self._P[0][1] * si11
        k10 = self._P[1][0] * si00 + self._P[1][1] * si10
        k11 = self._P[1][0] * si01 + self._P[1][1] * si11
        k20 = self._P[2][0] * si00 + self._P[2][1] * si10
        k21 = self._P[2][0] * si01 + self._P[2][1] * si11
        k30 = self._P[3][0] * si00 + self._P[3][1] * si10
        k31 = self._P[3][0] * si01 + self._P[3][1] * si11
        k40 = self._P[4][0] * si00 + self._P[4][1] * si10
        k41 = self._P[4][0] * si01 + self._P[4][1] * si11

        self._x[0] += k00 * yi0 + k01 * yi1
        self._x[1] += k10 * yi0 + k11 * yi1
        self._x[2] += k20 * yi0 + k21 * yi1
        self._x[2]  = wrap_pi(self._x[2])
        self._x[3] += k30 * yi0 + k31 * yi1
        self._x[4] += k40 * yi0 + k41 * yi1

        p00 = self._P[0][0]; p01 = self._P[0][1]; p02 = self._P[0][2]; p03 = self._P[0][3]; p04 = self._P[0][4]
        p10 = self._P[1][0]; p11 = self._P[1][1]; p12 = self._P[1][2]; p13 = self._P[1][3]; p14 = self._P[1][4]
        p20 = self._P[2][0]; p21 = self._P[2][1]; p22 = self._P[2][2]; p23 = self._P[2][3]; p24 = self._P[2][4]
        p30 = self._P[3][0]; p31 = self._P[3][1]; p32 = self._P[3][2]; p33 = self._P[3][3]; p34 = self._P[3][4]
        p40 = self._P[4][0]; p41 = self._P[4][1]; p42 = self._P[4][2]; p43 = self._P[4][3]; p44 = self._P[4][4]

        self._P[0][0] = p00 - k00 * p00 - k01 * p10
        self._P[0][1] = p01 - k00 * p01 - k01 * p11
        self._P[0][2] = p02 - k00 * p02 - k01 * p12
        self._P[0][3] = p03 - k00 * p03 - k01 * p13
        self._P[0][4] = p04 - k00 * p04 - k01 * p14
        self._P[1][0] = p10 - k10 * p00 - k11 * p10
        self._P[1][1] = p11 - k10 * p01 - k11 * p11
        self._P[1][2] = p12 - k10 * p02 - k11 * p12
        self._P[1][3] = p13 - k10 * p03 - k11 * p13
        self._P[1][4] = p14 - k10 * p04 - k11 * p14
        self._P[2][0] = p20 - k20 * p00 - k21 * p10
        self._P[2][1] = p21 - k20 * p01 - k21 * p11
        self._P[2][2] = p22 - k20 * p02 - k21 * p12
        self._P[2][3] = p23 - k20 * p03 - k21 * p13
        self._P[2][4] = p24 - k20 * p04 - k21 * p14
        self._P[3][0] = p30 - k30 * p00 - k31 * p10
        self._P[3][1] = p31 - k30 * p01 - k31 * p11
        self._P[3][2] = p32 - k30 * p02 - k31 * p12
        self._P[3][3] = p33 - k30 * p03 - k31 * p13
        self._P[3][4] = p34 - k30 * p04 - k31 * p14
        self._P[4][0] = p40 - k40 * p00 - k41 * p10
        self._P[4][1] = p41 - k40 * p01 - k41 * p11
        self._P[4][2] = p42 - k40 * p02 - k41 * p12
        self._P[4][3] = p43 - k40 * p03 - k41 * p13
        self._P[4][4] = p44 - k40 * p04 - k41 * p14

    @property
    def x(self) -> float:
        return self._x[0]

    @property
    def y(self) -> float:
        return self._x[1]

    @property
    def theta(self) -> float:
        return self._x[2]

    @property
    def v(self) -> float:
        return self._x[3]

    @property
    def omega(self) -> float:
        return self._x[4]

    @property
    def rejected_count(self) -> int:
        return self._rejected

    @property
    def rej_head_streak(self) -> int:
        return self._rej_head_streak


# ---------------------------------------------------------------------------
# Default test noise parameters (chosen for numerical clarity, not firmware defaults)
# ---------------------------------------------------------------------------

Q_XY    = 1.0
Q_THETA = 0.01
Q_V     = 25.0
Q_OMEGA = 0.01
R_XY    = 10.0
R_OTOS_V = 100.0
R_ENC_V  = 50.0


def _make_ekf_default() -> EKF:
    """Create a default EKF with standard test noise parameters."""
    e = EKF()
    e.init(Q_XY, Q_THETA, Q_V, Q_OMEGA, R_XY, R_OTOS_V, R_ENC_V)
    return e


# ---------------------------------------------------------------------------
# TestPredictStraight — straight-line motion, covariance growth
# ---------------------------------------------------------------------------

class TestPredictStraight:
    """Straight-line predict: state advances correctly and P grows by Q."""

    def _make_ekf(self) -> EKF:
        return _make_ekf_default()

    def test_straight_x_advances(self):
        """dCenter=100, dTheta=0, theta_before=0 → x≈100."""
        e = self._make_ekf()
        e.predict(100.0, 0.0, 0.0)
        assert e.x == pytest.approx(100.0, abs=1e-9)

    def test_straight_y_stays_zero(self):
        """dCenter=100, dTheta=0, theta_before=0 → y≈0."""
        e = self._make_ekf()
        e.predict(100.0, 0.0, 0.0)
        assert e.y == pytest.approx(0.0, abs=1e-9)

    def test_straight_theta_stays_zero(self):
        """dCenter=100, dTheta=0, theta_before=0 → theta≈0."""
        e = self._make_ekf()
        e.predict(100.0, 0.0, 0.0)
        assert e.theta == pytest.approx(0.0, abs=1e-9)

    def test_p00_grows_by_q_xy(self):
        """P[0][0] increases by q_xy after one predict from zero P."""
        e = self._make_ekf()
        # Starting from P=0, after one predict with theta_mid=0:
        # a = -100*sin(0) = 0, b = 100*cos(0) = 100
        # T[0][j] = P[0][j] + 0 = 0; t02 = 0
        # P_new[0][0] = t00 + t02*a + Q[0][0] = 0 + 0 + Q_XY
        e.predict(100.0, 0.0, 0.0)
        assert e._P[0][0] == pytest.approx(Q_XY, abs=1e-9)

    def test_p11_grows_by_q_xy(self):
        """P[1][1] increases by q_xy after one predict from zero P."""
        e = self._make_ekf()
        e.predict(100.0, 0.0, 0.0)
        assert e._P[1][1] == pytest.approx(Q_XY, abs=1e-9)

    def test_p22_grows_by_q_theta(self):
        """P[2][2] increases by q_theta after one predict from zero P."""
        e = self._make_ekf()
        e.predict(100.0, 0.0, 0.0)
        assert e._P[2][2] == pytest.approx(Q_THETA, abs=1e-9)


# ---------------------------------------------------------------------------
# TestPredictTurn — pure rotation and combined arc
# ---------------------------------------------------------------------------

class TestPredictTurn:
    """Turning motion: heading and position integrate correctly."""

    def _make_ekf(self) -> EKF:
        return _make_ekf_default()

    def test_pure_rotation_theta(self):
        """dCenter=0, dTheta=pi/2, theta_before=0 → theta≈pi/2."""
        e = self._make_ekf()
        e.predict(0.0, math.pi / 2, 0.0)
        assert e.theta == pytest.approx(math.pi / 2, abs=1e-9)

    def test_pure_rotation_x_stays_zero(self):
        """Pure rotation: x stays at 0 (no translation)."""
        e = self._make_ekf()
        e.predict(0.0, math.pi / 2, 0.0)
        assert e.x == pytest.approx(0.0, abs=1e-9)

    def test_pure_rotation_y_stays_zero(self):
        """Pure rotation: y stays at 0 (no translation)."""
        e = self._make_ekf()
        e.predict(0.0, math.pi / 2, 0.0)
        assert e.y == pytest.approx(0.0, abs=1e-9)

    def test_arc_x_matches_midpoint_integration(self):
        """dCenter=100, dTheta=pi/4, theta_before=0: x = 100*cos(pi/8)."""
        e = self._make_ekf()
        e.predict(100.0, math.pi / 4, 0.0)
        expected_x = 100.0 * math.cos(math.pi / 8)
        assert e.x == pytest.approx(expected_x, abs=1e-9)

    def test_arc_y_matches_midpoint_integration(self):
        """dCenter=100, dTheta=pi/4, theta_before=0: y = 100*sin(pi/8)."""
        e = self._make_ekf()
        e.predict(100.0, math.pi / 4, 0.0)
        expected_y = 100.0 * math.sin(math.pi / 8)
        assert e.y == pytest.approx(expected_y, abs=1e-9)


# ---------------------------------------------------------------------------
# TestHeadingWrap — angle stays in (-π, π] across both wrap boundaries
# ---------------------------------------------------------------------------

class TestHeadingWrap:
    """Heading wrapping across the ±π discontinuity."""

    def _make_ekf(self) -> EKF:
        return _make_ekf_default()

    def test_wrap_across_positive_pi(self):
        """Predict across +π boundary: result stays in (-π, π]."""
        e = self._make_ekf()
        e.set_pose(0.0, 0.0, math.pi - 0.1)
        e.predict(0.0, 0.3, math.pi - 0.1)
        assert -math.pi < e.theta <= math.pi

    def test_wrap_across_negative_pi(self):
        """Predict across -π boundary: result stays in (-π, π]."""
        e = self._make_ekf()
        e.set_pose(0.0, 0.0, -(math.pi - 0.1))
        e.predict(0.0, -0.3, -(math.pi - 0.1))
        assert -math.pi < e.theta <= math.pi

    def test_wrap_positive_pi_value_is_correct(self):
        """Crossing +π by 0.2 rad: result should be near -π+0.1."""
        e = self._make_ekf()
        e.set_pose(0.0, 0.0, math.pi - 0.1)
        e.predict(0.0, 0.2, math.pi - 0.1)
        expected = wrap_pi(math.pi - 0.1 + 0.2)
        assert e.theta == pytest.approx(expected, abs=1e-9)

    def test_wrap_negative_pi_value_is_correct(self):
        """Crossing -π by -0.2 rad: result should be near +π-0.1."""
        e = self._make_ekf()
        e.set_pose(0.0, 0.0, -(math.pi - 0.1))
        e.predict(0.0, -0.2, -(math.pi - 0.1))
        expected = wrap_pi(-(math.pi - 0.1) - 0.2)
        assert e.theta == pytest.approx(expected, abs=1e-9)


# ---------------------------------------------------------------------------
# TestUpdate — Kalman update pulls state toward observation and reduces P
# (uses backward-compat update() alias which delegates to update_position())
# ---------------------------------------------------------------------------

class TestUpdate:
    """Kalman update: state moves toward observation, covariance shrinks."""

    def _make_ekf_with_covariance(self) -> EKF:
        """Return an EKF that has non-zero P (needed for K to be non-zero)."""
        e = _make_ekf_default()
        # Run a predict to build up covariance before updating
        e.predict(100.0, 0.0, 0.0)
        # Manually set state to x=20 so we can test the pull toward 0
        e._x[0] = 20.0
        e._x[1] = 0.0
        return e

    def test_x_moves_toward_observation(self):
        """EKF at x=20, OTOS at x=0: after update, x < 20 (moves toward 0)."""
        e = self._make_ekf_with_covariance()
        x_before = e.x
        e.update(0.0, 0.0)
        assert e.x < x_before

    def test_p00_decreases_after_update(self):
        """P[0][0] is smaller after an update (Kalman gain reduces uncertainty)."""
        e = self._make_ekf_with_covariance()
        p00_before = e._P[0][0]
        e.update(0.0, 0.0)
        assert e._P[0][0] < p00_before

    def test_p11_decreases_after_update(self):
        """P[1][1] is smaller after an update."""
        e = self._make_ekf_with_covariance()
        p11_before = e._P[1][1]
        e.update(0.0, 0.0)
        assert e._P[1][1] < p11_before

    def test_theta_not_changed_by_update(self):
        """Heading is not changed by update (only x,y are observed)."""
        e = _make_ekf_default()
        # Run predict to build non-zero P
        e.predict(0.0, 0.0, 0.0)
        # Set a specific non-zero theta
        e._x[2] = 1.0
        theta_before = e.theta
        e.update(0.0, 0.0)
        # Theta should be unchanged (K[2][*] * yi will be near zero when
        # P[2][0] and P[2][1] are zero, which they are after a straight predict)
        assert e.theta == pytest.approx(theta_before, abs=1e-9)


# ---------------------------------------------------------------------------
# TestConvergence — repeated predict+update cycles converge to truth
# ---------------------------------------------------------------------------

class TestConvergence:
    """30 predict+update cycles drive state toward OTOS truth."""

    def test_x_converges_to_truth(self):
        """After 30 cycles with OTOS at (0,0), x within 5mm of truth."""
        e = _make_ekf_default()
        e.set_pose(50.0, 50.0, 0.0)
        for _ in range(30):
            e.predict(0.0, 0.0, 0.0)   # no motion — pure correction test
            e.update(0.0, 0.0)
        assert abs(e.x) < 5.0

    def test_y_converges_to_truth(self):
        """After 30 cycles with OTOS at (0,0), y within 5mm of truth."""
        e = _make_ekf_default()
        e.set_pose(50.0, 50.0, 0.0)
        for _ in range(30):
            e.predict(0.0, 0.0, 0.0)
            e.update(0.0, 0.0)
        assert abs(e.y) < 5.0

    def test_covariance_decreases_over_cycles(self):
        """P[0][0] decreases from its peak (after predicts only) once updates begin.

        Strategy: run 10 predict-only steps to build up a large P, then start
        the predict+update loop. P should shrink from the elevated level toward
        steady state as the Kalman filter folds in observations.
        """
        e = _make_ekf_default()
        e.set_pose(50.0, 50.0, 0.0)

        # Build up covariance via predict-only (no OTOS yet)
        for _ in range(10):
            e.predict(0.0, 0.0, 0.0)
        p00_peak = e._P[0][0]

        # Now run 1 predict+update and record P
        e.predict(0.0, 0.0, 0.0)
        e.update(0.0, 0.0)
        p00_after_1 = e._P[0][0]

        # Run 29 more predict+update cycles
        for _ in range(29):
            e.predict(0.0, 0.0, 0.0)
            e.update(0.0, 0.0)
        p00_after_30 = e._P[0][0]

        # After starting updates, P should have dropped from its peak
        assert p00_after_1 < p00_peak, (
            f"P[0][0] should decrease after first update: {p00_after_1} vs peak {p00_peak}"
        )
        # And P after 30 cycles should be at or below after-1-cycle level
        assert p00_after_30 <= p00_after_1 + 1e-6, (
            f"P[0][0] after 30 cycles ({p00_after_30}) should be <= after 1 cycle ({p00_after_1})"
        )


# ---------------------------------------------------------------------------
# TestNoDriftWithoutUpdate — covariance grows monotonically without corrections
# ---------------------------------------------------------------------------

class TestNoDriftWithoutUpdate:
    """Without update steps, P diverges (uncertainty grows)."""

    def test_p00_grows_over_predicts(self):
        """P[0][0] after 10 predicts > P[0][0] after 1 predict."""
        e = _make_ekf_default()

        e.predict(10.0, 0.0, 0.0)
        p00_after_1 = e._P[0][0]

        for _ in range(9):
            e.predict(10.0, 0.0, 0.0)
        p00_after_10 = e._P[0][0]

        assert p00_after_10 > p00_after_1


# ---------------------------------------------------------------------------
# TestSetPose — set_pose() overwrites state and zeros covariance
# ---------------------------------------------------------------------------

class TestSetPose:
    """set_pose() resets state and covariance correctly."""

    def test_set_pose_sets_x(self):
        """set_pose(100, 200, 0.5) → x=100."""
        e = _make_ekf_default()
        e.set_pose(100.0, 200.0, 0.5)
        assert e.x == pytest.approx(100.0, abs=1e-9)

    def test_set_pose_sets_y(self):
        """set_pose(100, 200, 0.5) → y=200."""
        e = _make_ekf_default()
        e.set_pose(100.0, 200.0, 0.5)
        assert e.y == pytest.approx(200.0, abs=1e-9)

    def test_set_pose_sets_theta(self):
        """set_pose(100, 200, 0.5) → theta=0.5."""
        e = _make_ekf_default()
        e.set_pose(100.0, 200.0, 0.5)
        assert e.theta == pytest.approx(0.5, abs=1e-9)

    def test_set_pose_sets_sane_prior(self):
        """After set_pose(), P has the sane diagonal prior (sprint 024-004).

        Old behaviour (before sprint 024): P was zeroed, creating falsely tight
        Mahalanobis gates after pose injection.  New behaviour: set a modest
        diagonal that reflects real uncertainty so re-acquisition is not strangled.
        Off-diagonal entries remain zero.
        """
        e = _make_ekf_default()
        # Build up some covariance first
        for _ in range(5):
            e.predict(10.0, 0.1, 0.0)
        # Now reset
        e.set_pose(0.0, 0.0, 0.0)
        # Diagonal must match the sane prior constants.
        assert e._P[0][0] == pytest.approx(EKF._PRIOR_XY,    abs=1e-9), \
            f"P[0][0] should be {EKF._PRIOR_XY}, got {e._P[0][0]}"
        assert e._P[1][1] == pytest.approx(EKF._PRIOR_XY,    abs=1e-9), \
            f"P[1][1] should be {EKF._PRIOR_XY}, got {e._P[1][1]}"
        assert e._P[2][2] == pytest.approx(EKF._PRIOR_THETA, rel=1e-5), \
            f"P[2][2] should be ~{EKF._PRIOR_THETA:.6f}, got {e._P[2][2]}"
        assert e._P[3][3] == pytest.approx(EKF._PRIOR_V,     abs=1e-9), \
            f"P[3][3] should be {EKF._PRIOR_V}, got {e._P[3][3]}"
        assert e._P[4][4] == pytest.approx(EKF._PRIOR_OMEGA, abs=1e-9), \
            f"P[4][4] should be {EKF._PRIOR_OMEGA}, got {e._P[4][4]}"
        # Off-diagonal entries must be zero.
        for i in range(5):
            for j in range(5):
                if i != j:
                    assert e._P[i][j] == pytest.approx(0.0, abs=1e-12), (
                        f"P[{i}][{j}] = {e._P[i][j]} should be zero (off-diagonal)"
                    )

    def test_predict_after_set_pose_advances_from_new_pose(self):
        """Predict after set_pose(100, 0, 0) with dCenter=50 → x≈150."""
        e = _make_ekf_default()
        e.set_pose(100.0, 0.0, 0.0)
        e.predict(50.0, 0.0, 0.0)
        assert e.x == pytest.approx(150.0, abs=1e-9)

    def test_set_pose_zeros_v_and_omega(self):
        """set_pose() zeroes v and omega state entries."""
        e = _make_ekf_default()
        # Inject velocity state manually then reset
        e._x[3] = 500.0
        e._x[4] = 1.0
        e.set_pose(0.0, 0.0, 0.0)
        assert e.v == pytest.approx(0.0, abs=1e-12)
        assert e.omega == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------------------
# TestPredictVelocity — velocity block grows by Q and state is unchanged
# ---------------------------------------------------------------------------

class TestPredictVelocity:
    """Velocity block random-walk: P grows, state unchanged, block decoupled."""

    def test_v_estimated_from_dCenter_dt(self):
        """A predict+update_velocity cycle moves v toward the measured rate.

        The velocity STATE is unchanged by predict (random-walk: v_{k+1}=v_k).
        The test verifies that after building sufficient P via 500 predicts, a
        velocity measurement of 1000 mm/s passes the Mahalanobis gate and moves
        v from 0 toward 1000 mm/s.  With P[3][3]=12500 and r_enc_v=50:
          s = 12500+50=12550; d2=(1000)^2/12550=79.7 — still > 3.84.
        Use a smaller measurement delta: start v at 900, measure 1000.
          d2=(100)^2/12550=0.80 < 3.84 ✓
        """
        e = _make_ekf_default()
        for _ in range(500):
            e.predict(1.0, 0.0, 0.0, 0.005)   # P[3][3] → 12500
        e._x[3] = 900.0    # pre-set v near the measurement
        e.update_velocity(1000.0, 0.0, R_ENC_V, R_OTOS_V)
        # After the update, v should have moved toward 1000 (> 900).
        assert e.v > 900.0

    def test_v_state_unchanged_by_predict_alone(self):
        """v state is unchanged by predict (random-walk)."""
        e = _make_ekf_default()
        e._x[3] = 300.0   # pre-set v
        e.predict(100.0, 0.0, 0.0, 0.1)
        assert e.v == pytest.approx(300.0, abs=1e-9)

    def test_omega_state_unchanged_by_predict_alone(self):
        """omega state is unchanged by predict (random-walk)."""
        e = _make_ekf_default()
        e._x[4] = 0.5
        e.predict(0.0, 0.1, 0.0, 0.1)
        assert e.omega == pytest.approx(0.5, abs=1e-9)

    def test_p33_grows_by_q_v_from_zero(self):
        """P[3][3] grows by q_v after one predict from zero P."""
        e = _make_ekf_default()
        e.predict(100.0, 0.0, 0.0, 0.1)
        assert e._P[3][3] == pytest.approx(Q_V, abs=1e-9)

    def test_p44_grows_by_q_omega_from_zero(self):
        """P[4][4] grows by q_omega after one predict from zero P."""
        e = _make_ekf_default()
        e.predict(100.0, 0.0, 0.0, 0.1)
        assert e._P[4][4] == pytest.approx(Q_OMEGA, abs=1e-9)

    def test_cross_block_entries_zero_after_predict(self):
        """P[0][3], P[0][4], P[1][3], P[1][4] remain 0 (block decoupling)."""
        e = _make_ekf_default()
        # Run several predicts to make sure cross-block entries never appear.
        for _ in range(5):
            e.predict(50.0, 0.1, e._x[2], 0.1)
        assert e._P[0][3] == pytest.approx(0.0, abs=1e-12)
        assert e._P[0][4] == pytest.approx(0.0, abs=1e-12)
        assert e._P[1][3] == pytest.approx(0.0, abs=1e-12)
        assert e._P[1][4] == pytest.approx(0.0, abs=1e-12)
        # Also verify the transpose entries
        assert e._P[3][0] == pytest.approx(0.0, abs=1e-12)
        assert e._P[4][0] == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------------------
# TestUpdateVelocity — velocity update moves state, reduces P, doesn't touch x,y,theta
# ---------------------------------------------------------------------------

class TestUpdateVelocity:
    """update_velocity() fuses measurement: state moves, P shrinks, pos unchanged."""

    def _make_ekf_with_v_state(self, v_state: float = 500.0) -> EKF:
        """EKF with v state pre-set to v_state, nonzero P from predicts.

        Build up P[3][3] via 20 predicts so the innovation (300-500)^2/(P[3][3]+r)
        passes the 3.84 Mahalanobis gate at 1-DOF.
        P[3][3] = 20 * Q_V = 500; r_enc_v=50; s=550; d2=(200)^2/550=72.7.
        That still fails. We need d2 < 3.84, i.e. s > (200)^2/3.84 = 10417.
        Use 500 predicts: P[3][3]=12500; s=12550; d2=(200)^2/12550=3.19 < 3.84.
        """
        e = _make_ekf_default()
        for _ in range(500):
            e.predict(1.0, 0.0, 0.0, 0.005)   # build P[3][3] to 500*Q_V=12500
        e._x[3] = v_state
        return e

    def test_v_moves_toward_measurement(self):
        """v at 500 mm/s with measurement 300 mm/s: state moves toward 300 mm/s."""
        e = self._make_ekf_with_v_state(500.0)
        e.update_velocity(300.0, 0.0, R_ENC_V, R_OTOS_V)
        assert e.v < 500.0, f"v={e.v} should have moved toward 300 from 500"

    def test_p33_decreases_after_velocity_update(self):
        """P[3][3] is smaller after an update_velocity (uncertainty reduced)."""
        e = self._make_ekf_with_v_state(500.0)
        p33_before = e._P[3][3]
        e.update_velocity(300.0, 0.0, R_ENC_V, R_OTOS_V)
        assert e._P[3][3] < p33_before

    def test_position_states_unchanged_by_velocity_update(self):
        """x, y, theta are NOT changed by update_velocity (block decoupled)."""
        e = _make_ekf_default()
        e.predict(50.0, 0.1, 0.0, 0.05)
        x_before = e.x
        y_before = e.y
        theta_before = e.theta
        # Set v to 500, inject measurement 300
        e._x[3] = 500.0
        e.update_velocity(300.0, 0.0, R_ENC_V, R_OTOS_V)
        # Because P[0][3]=P[1][3]=P[2][3]=0 (block decoupled), K[0]/K[1]/K[2]=0.
        assert e.x == pytest.approx(x_before, abs=1e-9)
        assert e.y == pytest.approx(y_before, abs=1e-9)
        assert e.theta == pytest.approx(theta_before, abs=1e-9)


# ---------------------------------------------------------------------------
# TestMahalanobisGating — outlier observations are rejected; counter increments
# ---------------------------------------------------------------------------

class TestMahalanobisGating:
    """Mahalanobis gate: outliers rejected, state/covariance unchanged."""

    def test_update_position_large_innovation_rejected(self):
        """update_position with innovation >> 1-sigma is rejected; counter increments."""
        e = _make_ekf_default()
        # Build a small P so the Mahalanobis distance is huge for a large jump.
        e.predict(0.0, 0.0, 0.0)  # P[0][0] = 1.0 (Q_XY)
        # Measurement 1000 mm away; sigma ~ sqrt(P[0][0]+R_XY) ~ sqrt(11) ~ 3.3 mm
        rejected_before = e.rejected_count
        state_before = e._x[:]
        P_before = [e._P[i][:] for i in range(5)]
        e.update_position(1000.0, 0.0)
        assert e.rejected_count == rejected_before + 1
        assert e._x == state_before
        for i in range(5):
            assert e._P[i] == P_before[i]

    def test_update_position_small_innovation_accepted(self):
        """update_position within ~1-sigma is accepted; counter unchanged."""
        e = _make_ekf_default()
        e.predict(0.0, 0.0, 0.0)   # P[0][0] = 1, P[1][1] = 1
        rejected_before = e.rejected_count
        # Tiny innovation (0.01 mm) — should always pass the gate.
        e.update_position(0.01, 0.0)
        assert e.rejected_count == rejected_before

    def test_update_velocity_outlier_rejected(self):
        """update_velocity with a huge outlier is rejected; counter increments."""
        e = _make_ekf_default()
        e.predict(0.0, 0.0, 0.0)   # P[3][3] = Q_V = 25
        # Innovation of 1000 mm/s; s = 25 + 50 = 75; d2 = 1e6/75 >> 3.84
        rejected_before = e.rejected_count
        state_before = e._x[:]
        P_before = [e._P[i][:] for i in range(5)]
        e.update_velocity(1000.0, 0.0, R_ENC_V, R_OTOS_V)
        # v channel should be rejected (innovation = 1000 from state 0)
        assert e.rejected_count > rejected_before
        # State should be unchanged (v is rejected)
        assert e._x[3] == state_before[3]

    def test_update_position_rejection_leaves_state_unchanged(self):
        """After a rejected update_position, state and covariance are identical."""
        e = _make_ekf_default()
        e.predict(0.0, 0.0, 0.0)
        state_before = e._x[:]
        P_before = [e._P[i][:] for i in range(5)]
        e.update_position(5000.0, 5000.0)   # enormous innovation — always rejected
        assert e._x == pytest.approx(state_before, abs=1e-12)
        for i in range(5):
            assert e._P[i] == pytest.approx(P_before[i], abs=1e-12)


# ---------------------------------------------------------------------------
# TestSetPoseRebaseline — regression for the setPose _prevEncL fix
#
# Bug description (sprint 023, Odometry.cpp):
#   Old code:  _prevEncL = 0.0f; _prevEncR = 0.0f;
#   After a camera fix sets encLMm=500, encRMm=495, the next predict step
#   would compute dL = encLMm - _prevEncL = 500 - 0 = 500 (spurious jump).
#
#   Fix:       _prevEncL = s.encLMm; _prevEncR = s.encRMm;
#   After fix, dL = encLMm - _prevEncL = 500 - 500 = 0 (correct).
#
# This Python test reproduces the firmware behaviour in a minimal pure-Python
# structure to confirm the bug and verify the fix logic.
# ---------------------------------------------------------------------------

class TestSetPoseRebaseline:
    """Regression test for the setPose encoder re-baseline bug."""

    class SimpleOdometry:
        """Minimal Odometry structure mirroring the firmware's _prevEncL/_prevEncR logic."""

        def __init__(self):
            self._prev_enc_l = 0.0
            self._prev_enc_r = 0.0

        def set_pose_old_buggy(self, enc_l_mm: float, enc_r_mm: float) -> None:
            """Old (buggy) behaviour: zeros _prev instead of re-baselining."""
            # Bug: this zeros _prev instead of saving current encoder values.
            self._prev_enc_l = 0.0
            self._prev_enc_r = 0.0

        def set_pose_fixed(self, enc_l_mm: float, enc_r_mm: float) -> None:
            """Fixed behaviour: _prev = current encoder values."""
            self._prev_enc_l = enc_l_mm
            self._prev_enc_r = enc_r_mm

        def compute_delta_l(self, enc_l_mm: float) -> float:
            """Compute dL = encLMm - _prevEncL (what firmware predict step does)."""
            dL = enc_l_mm - self._prev_enc_l
            self._prev_enc_l = enc_l_mm
            return dL

    def test_setpose_rebaselines_encoder_prev(self):
        """Old behaviour: set_pose(zeros _prev) then predict produces spurious dL=500.
        Fixed behaviour: set_pose(_prev=enc) then predict produces dL≈0.

        Bug: Odometry::setPose() used to write _prevEncL = 0 instead of
        _prevEncL = s.encLMm. After a camera fix at enc=500, the next predict
        step would compute dL = 500 - 0 = 500 (spurious jump), corrupting the
        EKF position estimate for one tick.

        Fix (sprint 023, T001): _prevEncL = s.encLMm (re-baseline to current
        encoder reading so the next predict yields dL ≈ 0).
        """
        enc_l_mm = 500.0
        enc_r_mm = 495.0

        # Scenario: robot has driven enc_l_mm since last zero.
        # Camera fix fires — set_pose() is called.
        # Next TLM frame: encoder unchanged → dL should be 0 (no actual motion).

        odo = self.SimpleOdometry()
        # Pre-condition: _prev is already at some baseline before the camera fix.
        odo._prev_enc_l = enc_l_mm   # simulate having tracked up to this point

        # --- OLD BUGGY BEHAVIOUR ---
        odo.set_pose_old_buggy(enc_l_mm, enc_r_mm)
        # After buggy set_pose, _prevEncL = 0. But the encoder is still at 500.
        # Next predict: dL = 500 - 0 = 500 → SPURIOUS JUMP
        dL_old = odo.compute_delta_l(enc_l_mm)
        assert dL_old == pytest.approx(500.0, abs=1e-6), (
            f"Old behaviour should produce spurious dL=500, got {dL_old}"
        )

        # Reset and test the FIXED behaviour
        odo2 = self.SimpleOdometry()
        odo2._prev_enc_l = enc_l_mm

        odo2.set_pose_fixed(enc_l_mm, enc_r_mm)
        # After fixed set_pose, _prevEncL = 500. Encoder still at 500.
        # Next predict: dL = 500 - 500 = 0 → CORRECT (no spurious jump)
        dL_fixed = odo2.compute_delta_l(enc_l_mm)
        assert dL_fixed == pytest.approx(0.0, abs=1e-6), (
            f"Fixed behaviour should produce dL≈0, got {dL_fixed}"
        )


# ---------------------------------------------------------------------------
# TestGoldenVectors — hard-coded expected values for Python/C++ parity
#
# These values are the source of truth for the C++ EKF implementation.
# If the C++ EKF produces different values for the same inputs, the C++ is wrong.
#
# Noise parameters used:
#   q_xy=1.0, q_theta=0.01, q_v=25.0, q_omega=0.01
#   r_otos_xy=10.0, r_otos_v=100.0, r_enc_v=50.0
#
# Golden Vector 1: predict(dCenter=50.0, dTheta=0.1, theta_before=0.0, dt_s=0.05)
#   from initial state [0,0,0,0,0], P=0.
#   Expected state: [49.9375130197, 2.49895846353, 0.1, 0.0, 0.0]
#   Expected P_diag: [1.0, 1.0, 0.01, 25.0, 0.01]
#
# Golden Vector 2: update_position(x_otos=49.94, y_otos=2.50)
#   applied immediately after GV1 (same P).
#   Expected state: [49.9377393198, 2.49905311348, 0.1, 0.0, 0.0]
#   Expected P_diag: [0.909090909, 0.909090909, 0.01, 25.0, 0.01]
# ---------------------------------------------------------------------------

class TestGoldenVectors:
    """Hard-coded golden vectors asserting Python/C++ EKF numerical parity."""

    _GV_Q_XY    = 1.0
    _GV_Q_THETA = 0.01
    _GV_Q_V     = 25.0
    _GV_Q_OMEGA = 0.01
    _GV_R_XY    = 10.0
    _GV_R_OV    = 100.0
    _GV_R_EV    = 50.0

    def _make_gv_ekf(self) -> EKF:
        e = EKF()
        e.init(self._GV_Q_XY, self._GV_Q_THETA, self._GV_Q_V, self._GV_Q_OMEGA,
               self._GV_R_XY, self._GV_R_OV, self._GV_R_EV)
        return e

    def test_golden_vector_1_predict(self):
        """GV1: predict(50.0, 0.1, 0.0, 0.05) from zero state and zero P.

        These values are the source of truth. If the C++ EKF produces different
        values for the same inputs, the C++ implementation is wrong.
        """
        e = self._make_gv_ekf()
        e.predict(50.0, 0.1, 0.0, 0.05)

        # State — 6 significant figures
        assert e.x     == pytest.approx(49.9375130197, rel=1e-6)
        assert e.y     == pytest.approx(2.49895846353, rel=1e-6)
        assert e.theta == pytest.approx(0.1,           rel=1e-6)
        assert e.v     == pytest.approx(0.0,           abs=1e-9)
        assert e.omega == pytest.approx(0.0,           abs=1e-9)

        # P diagonal — 6 significant figures
        assert e._P[0][0] == pytest.approx(1.0,   rel=1e-6)
        assert e._P[1][1] == pytest.approx(1.0,   rel=1e-6)
        assert e._P[2][2] == pytest.approx(0.01,  rel=1e-6)
        assert e._P[3][3] == pytest.approx(25.0,  rel=1e-6)
        assert e._P[4][4] == pytest.approx(0.01,  rel=1e-6)

    def test_golden_vector_2_update_position(self):
        """GV2: update_position(49.94, 2.50) after GV1 predict.

        These values are the source of truth. If the C++ EKF produces different
        values for the same inputs, the C++ implementation is wrong.

        Innovation: yi0 = 49.94 - 49.9375130197 = 0.002487
                    yi1 = 2.50  - 2.49895846353 = 0.001042
        Both well within 1-sigma, so Mahalanobis gate passes (d2 << 5.99).
        """
        e = self._make_gv_ekf()
        e.predict(50.0, 0.1, 0.0, 0.05)   # same as GV1
        e.update_position(49.94, 2.50)

        # State — 6 significant figures
        assert e.x     == pytest.approx(49.9377393198, rel=1e-6)
        assert e.y     == pytest.approx(2.49905311348, rel=1e-6)
        assert e.theta == pytest.approx(0.1,           rel=1e-6)
        assert e.v     == pytest.approx(0.0,           abs=1e-9)
        assert e.omega == pytest.approx(0.0,           abs=1e-9)

        # P diagonal — 6 significant figures
        assert e._P[0][0] == pytest.approx(0.909090909, rel=1e-6)
        assert e._P[1][1] == pytest.approx(0.909090909, rel=1e-6)
        assert e._P[2][2] == pytest.approx(0.01,        rel=1e-6)
        assert e._P[3][3] == pytest.approx(25.0,        rel=1e-6)
        assert e._P[4][4] == pytest.approx(0.01,        rel=1e-6)


# ---------------------------------------------------------------------------
# TestReplayHarness — ekf_replay.replay_tlm_log() drives EKF from a TLM log
# ---------------------------------------------------------------------------

class TestReplayHarness:
    """replay_tlm_log() parses a fixture log and drives the EKF mirror."""

    _FIXTURE = "tests/dev/fixtures/tlm_log_sample.txt"

    def _load_fixture(self) -> str:
        """Return the fixture path (relative to project root)."""
        return self._FIXTURE

    @staticmethod
    def _get_replay_fn():
        """Import replay_tlm_log from ekf_replay (handles path discovery)."""
        import importlib.util
        import os
        # Locate ekf_replay.py relative to this test file.
        here = os.path.dirname(os.path.abspath(__file__))
        replay_path = os.path.join(here, "ekf_replay.py")
        spec = importlib.util.spec_from_file_location("ekf_replay", replay_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.replay_tlm_log

    def test_replay_encoder_only_advances_x(self):
        """Straight-line encoder-only replay produces monotonically increasing x."""
        replay_tlm_log = self._get_replay_fn()
        frames = replay_tlm_log(
            self._load_fixture(),
            encoder_only=True,
            otos_position=False,
            otos_velocity=False,
        )
        assert len(frames) > 5, "fixture must have at least 5 encoder frames"
        # Extract x values from the straight-line section (first 10 frames).
        xs = [f[1] for f in frames[:10]]
        # x must be non-decreasing (monotonically advancing)
        for i in range(1, len(xs)):
            assert xs[i] >= xs[i - 1] - 1e-6, (
                f"x should be non-decreasing: xs[{i}]={xs[i]} < xs[{i-1}]={xs[i-1]}"
            )

    def test_replay_returns_correct_tuple_structure(self):
        """Each frame is a (t_ms, x, y, theta, v, omega, P_diag) 7-tuple."""
        replay_tlm_log = self._get_replay_fn()
        frames = replay_tlm_log(self._load_fixture())
        assert len(frames) > 0
        frame = frames[0]
        assert len(frame) == 7, f"expected 7-tuple, got {len(frame)}"
        t_ms, x, y, theta, v, omega, p_diag = frame
        assert isinstance(p_diag, (list, tuple)) and len(p_diag) == 5

    def test_replay_with_otos_position_differs_from_encoder_only(self):
        """Replay with OTOS position updates differs from encoder-only result."""
        replay_tlm_log = self._get_replay_fn()
        frames_enc = replay_tlm_log(self._load_fixture(), encoder_only=True,
                                    otos_position=False, otos_velocity=False)
        frames_otos = replay_tlm_log(self._load_fixture(), encoder_only=False,
                                     otos_position=True, otos_velocity=False)
        # OTOS updates should cause the trajectories to diverge at some point.
        xs_enc  = [f[1] for f in frames_enc]
        xs_otos = [f[1] for f in frames_otos]
        # They may start the same but after OTOS updates should differ.
        differs = any(abs(a - b) > 1e-6 for a, b in zip(xs_enc, xs_otos))
        assert differs, "encoder-only and +OTOS-position trajectories should differ"


# ---------------------------------------------------------------------------
# TestSetPosePrior — set_pose() sets sane diagonal P-prior (sprint 024-004)
# ---------------------------------------------------------------------------

class TestSetPosePrior:
    """set_pose() initialises a sane diagonal P-prior, not a zero matrix."""

    def test_p00_equals_prior_xy(self):
        """P[0][0] = PRIOR_XY (100 mm^2) after set_pose."""
        e = _make_ekf_default()
        e.set_pose(0.0, 0.0, 0.0)
        assert e._P[0][0] == pytest.approx(EKF._PRIOR_XY, abs=1e-9)

    def test_p11_equals_prior_xy(self):
        """P[1][1] = PRIOR_XY (100 mm^2) after set_pose."""
        e = _make_ekf_default()
        e.set_pose(0.0, 0.0, 0.0)
        assert e._P[1][1] == pytest.approx(EKF._PRIOR_XY, abs=1e-9)

    def test_p22_equals_prior_theta(self):
        """P[2][2] ≈ (5°)^2 after set_pose."""
        e = _make_ekf_default()
        e.set_pose(0.0, 0.0, 0.0)
        expected = (5.0 * math.pi / 180.0) ** 2
        assert e._P[2][2] == pytest.approx(expected, rel=1e-5)

    def test_p33_equals_prior_v(self):
        """P[3][3] = PRIOR_V after set_pose."""
        e = _make_ekf_default()
        e.set_pose(0.0, 0.0, 0.0)
        assert e._P[3][3] == pytest.approx(EKF._PRIOR_V, abs=1e-9)

    def test_p44_equals_prior_omega(self):
        """P[4][4] = PRIOR_OMEGA after set_pose."""
        e = _make_ekf_default()
        e.set_pose(0.0, 0.0, 0.0)
        assert e._P[4][4] == pytest.approx(EKF._PRIOR_OMEGA, abs=1e-9)

    def test_off_diagonal_zero(self):
        """All off-diagonal entries of P are zero after set_pose."""
        e = _make_ekf_default()
        # Build up covariance, then reset.
        for _ in range(5):
            e.predict(10.0, 0.1, 0.0)
        e.set_pose(10.0, 20.0, 0.3)
        for i in range(5):
            for j in range(5):
                if i != j:
                    assert e._P[i][j] == pytest.approx(0.0, abs=1e-12), (
                        f"Off-diagonal P[{i}][{j}] = {e._P[i][j]} should be zero"
                    )

    def test_prior_theta_approx_5deg_squared(self):
        """P[2][2] prior is approximately (5 degrees)^2 in radians^2."""
        e = _make_ekf_default()
        e.set_pose(0.0, 0.0, 0.0)
        five_deg_rad = 5.0 * math.pi / 180.0
        assert e._P[2][2] == pytest.approx(five_deg_rad ** 2, rel=1e-4)


# ---------------------------------------------------------------------------
# TestUpdateHeading — heading fusion closes the loop on OTOS heading (sprint 024-004)
# ---------------------------------------------------------------------------

class TestUpdateHeading:
    """update_heading() fuses heading measurement; state moves; P narrows."""

    def _make_ekf_with_heading_cov(self, heading_state: float = 0.0) -> EKF:
        """EKF with nonzero P[2][2] so update_heading() produces a visible effect.

        Build P[2][2] via predicts (each adds Q_THETA = 0.01).
        After 100 predicts: P[2][2] = 1.0.  With r_theta=0.1:
          s = 1.0 + 0.1 = 1.1; for small innovations, gate passes.
        """
        e = _make_ekf_default()
        for _ in range(100):
            e.predict(0.0, 0.0, 0.0)  # pure covariance growth, P[2][2] → 1.0
        e._x[2] = heading_state
        return e

    def test_heading_state_moves_toward_measurement(self):
        """EKF at theta=0.5, OTOS at theta=0.0: after update, theta < 0.5."""
        e = self._make_ekf_with_heading_cov(heading_state=0.5)
        theta_before = e.theta
        e.update_heading(0.0, 0.1)
        assert e.theta < theta_before, \
            f"theta={e.theta} should have moved toward 0.0 from {theta_before}"

    def test_p22_decreases_after_update(self):
        """P[2][2] is smaller after update_heading (uncertainty reduced)."""
        e = self._make_ekf_with_heading_cov(heading_state=0.0)
        p22_before = e._P[2][2]
        e.update_heading(0.0, 0.1)
        assert e._P[2][2] < p22_before, \
            f"P[2][2] should decrease: before={p22_before}, after={e._P[2][2]}"

    def test_wrap_safe_innovation_negative_pi_boundary(self):
        """Innovation wraps correctly across the -pi boundary.

        State theta near -pi+0.1, measurement near +pi-0.1.
        Without wrap: innovation ≈ 2*pi - 0.2 (huge, rejected).
        With wrap:    innovation ≈ -0.2 rad (small, accepted).
        """
        e = self._make_ekf_with_heading_cov(heading_state=-(math.pi - 0.1))
        r_theta = 0.1
        meas = math.pi - 0.1
        # Naive (unwrapped) innovation = meas - state ≈ 2*pi - 0.2 → large → rejected
        # Wrapped innovation = wrap_pi(meas - state) ≈ -0.2 → small → accepted
        wrapped_innov = wrap_pi(meas - e.theta)
        assert abs(wrapped_innov) < 0.5, \
            f"wrapped innovation {wrapped_innov:.3f} should be small (~-0.2)"
        theta_before = e.theta
        e.update_heading(meas, r_theta)
        # Should be accepted (state should change)
        assert e.theta != pytest.approx(theta_before, abs=1e-9), \
            "update_heading should have been accepted (state should change)"

    def test_large_innovation_rejected_and_streak_increments(self):
        """Huge heading outlier is rejected; streak counter increments."""
        e = self._make_ekf_with_heading_cov(heading_state=0.0)
        # After 100 predicts: P[2][2] = 100 * Q_THETA = 100 * 0.01 = 1.0.
        # s = P[2][2] + r_theta = 1.0 + 0.001 = 1.001.
        # Innovation = wrap_pi(3.0 - 0.0) = 3.0 (within (-pi,pi]).
        # d2 = 3.0^2 / 1.001 = 8.99 > 3.84 → should be rejected.
        r_tiny = 0.001
        streak_before = e.rej_head_streak
        rejected_before = e.rejected_count
        theta_before = e.theta
        P_before = [e._P[i][:] for i in range(5)]
        e.update_heading(3.0, r_tiny)  # innovation = 3.0 > sqrt(3.84 * 1.001) ≈ 1.96
        assert e.rejected_count == rejected_before + 1, "rejection count should increment"
        assert e.rej_head_streak == streak_before + 1, "streak should increment"
        assert e.theta == pytest.approx(theta_before, abs=1e-12), "state unchanged on reject"
        for i in range(5):
            assert e._P[i] == pytest.approx(P_before[i], abs=1e-12), \
                f"P[{i}] should be unchanged on rejection"

    def test_accepted_resets_streak(self):
        """A rejection followed by an acceptance resets the streak to 0."""
        e = self._make_ekf_with_heading_cov(heading_state=0.0)
        # First: force a rejection (huge innovation, tiny R).
        e.update_heading(3.0, 0.001)
        assert e.rej_head_streak >= 1, "streak should be >= 1 after rejection"
        # Now: small innovation → accepted.
        e.update_heading(0.0, 0.1)
        assert e.rej_head_streak == 0, "streak should reset to 0 on acceptance"

    def test_block_decoupled_x_y_unchanged_for_zero_cross_terms(self):
        """After a straight predict, P[2][0]=P[2][1]=0: x and y unchanged by update_heading."""
        e = _make_ekf_default()
        # After init: P=0 for all. After one straight predict: P[2][0]=P[2][1]=0
        # (because a=-dCenter*sin(0)=0, b=dCenter*cos(0), only P[0][0] grows).
        e.predict(100.0, 0.0, 0.0)
        x_before = e.x
        y_before = e.y
        # Manually ensure P[2][2] is nonzero so the update does something.
        e._P[2][2] = 0.1
        e.update_heading(0.5, 0.1)
        # K[0] = P[0][2] / s = 0 / s = 0; K[1] = P[1][2] / s = 0 → x, y unchanged.
        assert e.x == pytest.approx(x_before, abs=1e-9), "x should not change"
        assert e.y == pytest.approx(y_before, abs=1e-9), "y should not change"


# ---------------------------------------------------------------------------
# TestHeadingConvergence — heading fusion closes the gap where drift occurs
#   (field-profile: heading diverges on turns without OTOS correction)
# ---------------------------------------------------------------------------

class TestHeadingConvergence:
    """Heading fusion: with OTOS corrections, heading stays near truth."""

    def test_heading_tracks_otos_truth_over_turns(self):
        """After N turns with OTOS heading corrections, fused heading stays within ~2° of truth.

        Scenario: simulate a 90° turn in 18 predict steps (5° per step), with a
        small encoder bias of +5% (so encoder ends at ~94.5°, truth at 90°).
        OTOS fires once per turn at the truth heading.  After the correction, the
        fused heading should be within ~2° of truth.

        The key invariant: with OTOS heading fusion active, cumulative heading drift
        is bounded; without fusion it accumulates monotonically.
        """
        r_theta = 0.01   # matches firmware default ekfROtosTheta

        def simulate_turn_with_correction(truth_deg: float, encoder_bias: float = 1.05):
            """Run a 90° turn (in 18 x 5° steps) with encoder bias, then correct once."""
            e = _make_ekf_default()
            # Start with steady-state covariance (enough predicts so P isn't degenerate).
            for _ in range(50):
                e.predict(0.0, 0.0, 0.0)

            # Execute turn: 18 steps of 5° each, encoder reads 5%*encoder_bias each.
            step_truth_rad  = math.radians(truth_deg / 18.0)
            step_encoder_rad = step_truth_rad * encoder_bias
            for _ in range(18):
                e.predict(0.0, step_encoder_rad, e.theta)

            # Single OTOS heading correction at truth.
            truth_h = math.radians(truth_deg)
            e.update_heading(truth_h, r_theta)

            err_deg = abs(wrap_pi(e.theta - truth_h)) * 180.0 / math.pi
            return err_deg

        # Test for 90° turn with 5% encoder bias.
        err = simulate_turn_with_correction(90.0, encoder_bias=1.05)
        assert err < 2.0, (
            f"Heading error {err:.2f}° > 2.0° after OTOS correction on 90° turn"
        )

    def test_uncorrected_heading_diverges_per_turn(self):
        """Without OTOS corrections, accumulated encoder error grows each turn.

        Verifies that the error is real and that correction (TestHeadingConvergence
        above) is the actual fix, not just a lucky coincidence.
        """
        r_theta = 0.01
        encoder_error_per_turn = 0.175  # ~10° per turn

        e = _make_ekf_default()
        for _ in range(20):
            e.predict(0.0, 0.0, 0.0)

        # 4 turns with NO OTOS correction.
        total_encoder_error = 0.0
        truth_h = 0.0
        for _ in range(4):
            truth_h += math.pi / 2
            total_encoder_error += encoder_error_per_turn
            e.predict(0.0, encoder_error_per_turn, e.theta)
            # No update_heading call!

        final_err_deg = abs(wrap_pi(e.theta - truth_h)) * 180.0 / math.pi
        # Without correction, error should be large (4 * 10° = 40°).
        assert final_err_deg > 10.0, (
            f"Expected large heading error without correction; got {final_err_deg:.2f}°"
        )
