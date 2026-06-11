"""ekf_replay.py — Pure-Python 5-state EKF mirror and TLM log replay harness.

This module is intentionally pytest-free so it can be imported from both
test code and Jupyter notebooks without requiring the test framework.

It contains:
  - ``EKF``: Pure-Python mirror of ``source/control/EKF.cpp`` (sprint 023).
  - ``replay_tlm_log()``: Reads a TLM log and drives the EKF through
    predict + update steps, returning per-frame state tuples.

The ``EKF`` class is re-exported by ``tests/dev/test_ekf.py`` so test code
continues to import from there.

Usage from a notebook::

    import importlib.util, sys
    spec = importlib.util.spec_from_file_location(
        'ekf_replay', '/path/to/tests/dev/ekf_replay.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    frames = mod.replay_tlm_log('tests/dev/fixtures/tlm_log_sample.txt')

Sprint 023, Ticket T006.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import NamedTuple

# ---------------------------------------------------------------------------
# Import path setup — project root so protocol.py can be found.
# ---------------------------------------------------------------------------

_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from host.robot_radio.robot.protocol import parse_tlm  # type: ignore[import]


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
    """Python mirror of the C++ EKF class (sprint 023, ticket T006).

    State: [x_mm, y_mm, theta_rad, v_mmps, omega_rads]
    Motion model: position block = arc-segment (midpoint integration);
                  velocity block = random-walk (identity Jacobian).
    Observation channels:
      update_position(x_otos, y_otos): 2D position, Mahalanobis gate 5.99.
      update_velocity(v_meas, omega_meas, r_v, r_omega): two scalar 1-DOF
        updates, each gated at 3.84.
    """

    def __init__(self):
        self._x = [0.0] * 5
        self._P = [[0.0] * 5 for _ in range(5)]
        self._Q = [[0.0] * 5 for _ in range(5)]
        self._r_otos_xy = 0.0
        self._r_otos_v = 0.0
        self._r_enc_v = 0.0
        self._rejected = 0

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

        self._x = [0.0] * 5
        self._P = [[0.0] * 5 for _ in range(5)]

    def set_pose(self, x: float, y: float, theta: float) -> None:
        """Overwrite state with a known pose; zero v and omega; reset covariance."""
        self._x[0] = float(x)
        self._x[1] = float(y)
        self._x[2] = float(theta)
        self._x[3] = 0.0   # v
        self._x[4] = 0.0   # omega
        self._P = [[0.0] * 5 for _ in range(5)]

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
        """Update step: 2D position-only observation from OTOS.

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


# ---------------------------------------------------------------------------
# EKF noise parameters for replay (tuned to typical robot values)
# ---------------------------------------------------------------------------

_Q_XY      = 4.0    # process noise (mm^2 per tick)
_Q_THETA   = 0.001  # process noise (rad^2 per tick)
_Q_V       = 100.0  # velocity process noise (mm/s)^2 per tick
_Q_OMEGA   = 0.1    # angular velocity process noise (rad/s)^2 per tick
_R_OTOS_XY = 25.0   # OTOS position noise variance (mm^2)
_R_OTOS_V  = 200.0  # OTOS velocity noise variance (mm/s)^2
_R_ENC_V   = 100.0  # encoder velocity noise variance (mm/s)^2

_TRACK_WIDTH_MM = 100.0   # nominal differential-drive track width (mm)
_DEFAULT_DT_S   = 0.1     # fallback dt when timestamps are unavailable


# ---------------------------------------------------------------------------
# Replay result type
# ---------------------------------------------------------------------------

class ReplayFrame(NamedTuple):
    """One frame of EKF replay output."""
    t_ms: float
    x: float
    y: float
    theta: float
    v: float
    omega: float
    p_diag: list[float]   # [P[0][0], P[1][1], P[2][2], P[3][3], P[4][4]]


def replay_tlm_log(
    log_path: str,
    *,
    encoder_only: bool = False,
    otos_position: bool = True,
    otos_velocity: bool = True,
) -> list[ReplayFrame]:
    """Replay a TLM log file through the Python EKF mirror.

    Reads a newline-delimited TLM log (each line is a raw firmware TLM string).
    Comments (lines starting with '#') and blank lines are skipped.

    For each parsed TLMFrame:
      1. Compute encoder deltas (dL, dR) from the absolute encoder totals.
      2. Compute dCenter and dTheta from dL and dR using the nominal track width.
      3. Call ekf.predict(dCenter, dTheta, theta_before, dt_s).
      4. If ``otos_position`` and OTOS pose is present: call
         ekf.update_position(x_otos, y_otos).  Mahalanobis-gated.
      5. If ``otos_velocity`` and twist is present: call
         ekf.update_velocity(v_meas, omega_meas, r_v, r_omega).  Gated.
      6. If ``encoder_only`` is True, steps 4 and 5 are skipped regardless.

    Args:
        log_path:      Path to the TLM log file (absolute or relative to cwd).
        encoder_only:  If True, skip all OTOS updates (position and velocity).
        otos_position: If True (and not encoder_only), apply OTOS position updates.
        otos_velocity: If True (and not encoder_only), apply OTOS velocity updates.

    Returns:
        List of ReplayFrame named tuples, one per successfully parsed TLM line.
    """
    ekf = EKF()
    ekf.init(_Q_XY, _Q_THETA, _Q_V, _Q_OMEGA, _R_OTOS_XY, _R_OTOS_V, _R_ENC_V)

    frames: list[ReplayFrame] = []
    prev_enc_l: float | None = None
    prev_enc_r: float | None = None
    prev_t_ms:  float | None = None

    path = Path(log_path)
    # If the path is relative, try both cwd and project root.
    if not path.is_absolute():
        if not path.exists():
            candidate = _PROJECT_ROOT / log_path
            if candidate.exists():
                path = candidate

    with path.open("r", encoding="utf-8") as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            frame = parse_tlm(line)
            if frame is None:
                continue

            t_ms = float(frame.t) if frame.t is not None else (
                (prev_t_ms or 0.0) + _DEFAULT_DT_S * 1000.0
            )

            dt_s = max((t_ms - prev_t_ms) * 0.001, 1e-6) if prev_t_ms is not None else _DEFAULT_DT_S

            # Compute encoder deltas from cumulative encoder values.
            dL = 0.0
            dR = 0.0
            if frame.enc is not None:
                enc_l = float(frame.enc[0])
                enc_r = float(frame.enc[1])
                if prev_enc_l is not None:
                    dL = enc_l - prev_enc_l
                    dR = enc_r - prev_enc_r
                prev_enc_l = enc_l
                prev_enc_r = enc_r

            # Differential-drive kinematics.
            dCenter = (dL + dR) * 0.5
            dTheta  = (dR - dL) / _TRACK_WIDTH_MM

            theta_before = ekf.theta
            ekf.predict(dCenter, dTheta, theta_before, dt_s)

            if not encoder_only and otos_position and frame.pose is not None:
                ekf.update_position(float(frame.pose[0]), float(frame.pose[1]))

            if not encoder_only and otos_velocity and frame.twist is not None:
                v_meas     = float(frame.twist[0])          # mm/s
                omega_meas = float(frame.twist[1]) * 0.001  # mrad/s → rad/s
                ekf.update_velocity(v_meas, omega_meas, _R_ENC_V, _R_OTOS_V)

            p_diag = [ekf._P[i][i] for i in range(5)]
            frames.append(ReplayFrame(
                t_ms=t_ms, x=ekf.x, y=ekf.y, theta=ekf.theta,
                v=ekf.v, omega=ekf.omega, p_diag=p_diag,
            ))
            prev_t_ms = t_ms

    return frames
