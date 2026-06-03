#!/usr/bin/env python3
"""test_otos_fusion.py — Unit tests for Odometry::correct() OTOS complementary fusion (010-006).

Pure Python mirror of the correct() method implemented in source/control/Odometry.cpp.

Tests verify:
  - In-gate OTOS sample: position blended with alphaPos fraction; heading blended
    angle-wrap-safely with alphaYaw fraction.
  - Out-of-gate OTOS sample: pose unchanged; rejected counter incremented.
  - Heading blend is angle-wrap-safe across the ±π discontinuity.
  - Chained predict/correct: fusion applied after midpoint integration.
  - Zero alpha: correct() does nothing to pose.
  - Alpha == 1.0: pose jumps directly to OTOS measurement.

Sprint 010, Ticket 006.
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Pure-Python Odometry mirror (predict + correct)
# ---------------------------------------------------------------------------

def wrap_pi(theta: float) -> float:
    """Keep heading in (-π, π] using atan2 identity."""
    return math.atan2(math.sin(theta), math.cos(theta))


class Odometry:
    """Python mirror of the Odometry class with predict() and correct()."""

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0   # radians
        self.prev_enc_l = 0.0
        self.prev_enc_r = 0.0
        self.otos_rejected = 0  # mirrors _otosRejected counter

    def predict(self, enc_l_mm: float, enc_r_mm: float, trackwidth_mm: float) -> None:
        """Midpoint (exact-arc) integration step."""
        dL = enc_l_mm - self.prev_enc_l
        dR = enc_r_mm - self.prev_enc_r
        self.prev_enc_l = enc_l_mm
        self.prev_enc_r = enc_r_mm

        dC = (dL + dR) / 2.0
        dTheta = (dR - dL) / trackwidth_mm
        theta_mid = self.heading + dTheta / 2.0

        self.x += dC * math.cos(theta_mid)
        self.y += dC * math.sin(theta_mid)
        self.heading = wrap_pi(self.heading + dTheta)

    def correct(self, x_otos: float, y_otos: float, theta_otos_rad: float,
                alpha_pos: float, alpha_yaw: float, otos_gate: float) -> None:
        """OTOS complementary correction — mirror of Odometry::correct().

        Outlier gate: if distance(otos, predicted) > otos_gate, reject sample.
        Position blend: _x += alphaPos * (x_otos - _x)
        Heading blend: _heading += alphaYaw * wrapPi(theta_otos - _heading)
        """
        dx = x_otos - self.x
        dy = y_otos - self.y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > otos_gate:
            self.otos_rejected += 1
            return

        # Accepted: blend position
        self.x += alpha_pos * dx
        self.y += alpha_pos * dy

        # Heading blend — angle-wrap-safe
        dh = wrap_pi(theta_otos_rad - self.heading)
        self.heading = wrap_pi(self.heading + alpha_yaw * dh)

    def zero(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.prev_enc_l = 0.0
        self.prev_enc_r = 0.0
        self.otos_rejected = 0


# ---------------------------------------------------------------------------
# Tests — in-gate sample (blend applied)
# ---------------------------------------------------------------------------

class TestInGateSample:
    """OTOS sample within gate threshold: pose is blended, counter unchanged."""

    ALPHA_POS = 0.15
    ALPHA_YAW = 0.10
    GATE = 50.0  # mm — default from RobotConfig

    def test_position_x_blended_with_alpha(self):
        """x moves alphaPos fraction of the error toward OTOS x."""
        odo = Odometry()
        # Predicted pose: (0, 0, 0); OTOS: (20, 0, 0) — within 50 mm gate.
        odo.correct(20.0, 0.0, 0.0, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        # Expected: x = 0 + 0.15 * (20 - 0) = 3.0
        assert odo.x == pytest.approx(3.0, abs=1e-6)
        assert odo.y == pytest.approx(0.0, abs=1e-6)

    def test_position_y_blended_with_alpha(self):
        """y moves alphaPos fraction of the error toward OTOS y."""
        odo = Odometry()
        odo.correct(0.0, 30.0, 0.0, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        assert odo.y == pytest.approx(0.15 * 30.0, abs=1e-6)

    def test_both_xy_blended_independently(self):
        """Both x and y are blended using the same alphaPos."""
        odo = Odometry()
        odo.correct(10.0, 20.0, 0.0, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        assert odo.x == pytest.approx(0.15 * 10.0, abs=1e-6)
        assert odo.y == pytest.approx(0.15 * 20.0, abs=1e-6)

    def test_heading_blended_with_alpha_yaw(self):
        """Heading moves alphaYaw fraction of angular difference toward OTOS heading."""
        odo = Odometry()
        target_h = math.pi / 4  # 45°
        odo.correct(0.0, 0.0, target_h, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        # dh = wrap_pi(pi/4 - 0) = pi/4; result = 0 + 0.10 * pi/4
        expected_h = 0.10 * (math.pi / 4)
        assert odo.heading == pytest.approx(expected_h, abs=1e-6)

    def test_no_rejection_counter_increment_for_in_gate(self):
        """Accepted sample does not increment the rejected counter."""
        odo = Odometry()
        odo.correct(10.0, 10.0, 0.0, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        assert odo.otos_rejected == 0

    def test_alpha_one_jumps_to_otos(self):
        """With alpha=1.0, pose jumps completely to OTOS measurement."""
        odo = Odometry()
        odo.correct(40.0, 25.0, 0.5, 1.0, 1.0, self.GATE)
        assert odo.x == pytest.approx(40.0, abs=1e-6)
        assert odo.y == pytest.approx(25.0, abs=1e-6)
        assert odo.heading == pytest.approx(0.5, abs=1e-6)

    def test_alpha_zero_leaves_pose_unchanged(self):
        """With alpha=0.0, correct() is a no-op for pose."""
        odo = Odometry()
        # Set initial pose via predict
        odo.predict(100.0, 100.0, 120.0)
        x_before = odo.x
        y_before = odo.y
        h_before = odo.heading
        # OTOS within gate but alpha=0
        odo.correct(50.0, 50.0, 1.0, 0.0, 0.0, self.GATE)
        assert odo.x == pytest.approx(x_before, abs=1e-6)
        assert odo.y == pytest.approx(y_before, abs=1e-6)
        assert odo.heading == pytest.approx(h_before, abs=1e-6)

    def test_in_gate_boundary_exact(self):
        """Sample at exactly the gate distance is accepted (not rejected)."""
        odo = Odometry()
        # Distance = gate exactly: sqrt(50^2 + 0^2) = 50.0 mm
        odo.correct(50.0, 0.0, 0.0, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        # Not rejected → counter stays 0
        assert odo.otos_rejected == 0
        # Pose was blended
        assert odo.x == pytest.approx(0.15 * 50.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Tests — out-of-gate sample (rejected)
# ---------------------------------------------------------------------------

class TestOutOfGateSample:
    """OTOS sample beyond gate threshold: pose unchanged, counter incremented."""

    ALPHA_POS = 0.15
    ALPHA_YAW = 0.10
    GATE = 50.0  # mm

    def test_out_of_gate_x_leaves_pose_unchanged(self):
        """OTOS x beyond gate: x stays at predicted value."""
        odo = Odometry()
        # OTOS x = 100 mm, gate = 50 mm → distance = 100 > 50: rejected
        odo.correct(100.0, 0.0, 0.0, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        assert odo.x == pytest.approx(0.0, abs=1e-6)
        assert odo.y == pytest.approx(0.0, abs=1e-6)
        assert odo.heading == pytest.approx(0.0, abs=1e-6)

    def test_out_of_gate_increments_rejected_counter(self):
        """Rejected sample increments _otosRejected by 1."""
        odo = Odometry()
        odo.correct(100.0, 0.0, 0.0, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        assert odo.otos_rejected == 1

    def test_multiple_rejections_accumulate(self):
        """Multiple out-of-gate samples accumulate in the rejected counter."""
        odo = Odometry()
        for _ in range(5):
            odo.correct(100.0, 0.0, 0.0, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        assert odo.otos_rejected == 5

    def test_out_of_gate_diagonal(self):
        """Out-of-gate check uses Euclidean distance, not per-axis."""
        odo = Odometry()
        # Each axis 40 mm → distance = sqrt(40²+40²) ≈ 56.6 > 50: rejected
        odo.correct(40.0, 40.0, 0.0, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        assert odo.otos_rejected == 1
        assert odo.x == pytest.approx(0.0, abs=1e-6)

    def test_out_of_gate_heading_not_changed(self):
        """Rejected sample does not change heading either."""
        odo = Odometry()
        odo.heading = math.pi / 6  # 30°
        initial_h = odo.heading
        odo.correct(100.0, 0.0, math.pi, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        assert odo.heading == pytest.approx(initial_h, abs=1e-6)

    def test_counter_not_incremented_for_in_gate(self):
        """After an in-gate sample, counter stays at its previous value."""
        odo = Odometry()
        # One rejection
        odo.correct(100.0, 0.0, 0.0, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        assert odo.otos_rejected == 1
        # One acceptance
        odo.correct(5.0, 0.0, 0.0, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        assert odo.otos_rejected == 1  # unchanged after accepted sample


# ---------------------------------------------------------------------------
# Tests — heading wrap-safety across ±π boundary
# ---------------------------------------------------------------------------

class TestHeadingWrapSafety:
    """Blend across the ±π discontinuity must use angular difference, not raw difference."""

    ALPHA_POS = 0.0   # no position change — isolate heading blend
    ALPHA_YAW = 0.50
    GATE = 1000.0     # large gate to always accept

    def test_blend_across_pi_ccw(self):
        """Predicted at +π-0.1, OTOS at -π+0.1: correct path is +0.2 rad (not -2π+0.2)."""
        # Predicted heading near +π; OTOS heading near -π (same direction, across wrap).
        pred_h = math.pi - 0.1  # just below +π
        otos_h = -(math.pi - 0.1)  # equivalent to just above -π (same angle opposite sign)
        # wrap_pi(otos_h - pred_h) = wrap_pi(-2*(pi-0.1)) = wrap_pi(-2pi+0.2) = +0.2 rad
        # With alpha=0.5: new heading = pred_h + 0.5 * 0.2 = (pi-0.1) + 0.1 = pi
        # Alternatively: the short-path difference is only 0.2 rad, not -(2pi - 0.2).
        odo = Odometry()
        odo.heading = pred_h
        odo.correct(0.0, 0.0, otos_h, self.ALPHA_POS, self.ALPHA_YAW, self.GATE)
        # The blend should be the short-path: angular diff ≈ 0.2 rad → move half of that
        # Final heading ≈ (π - 0.1) + 0.5 * 0.2 = π, which wraps to ≈ -π or +π
        # Key property: |result - pred_h| ≤ 0.15 rad (close, not jumping by 2π)
        delta = abs(wrap_pi(odo.heading - pred_h))
        assert delta < 0.15, (
            f"Heading moved {math.degrees(delta):.2f}° — expected short-path blend (~0.1 rad)"
        )

    def test_blend_no_wrap_needed(self):
        """No ±π crossing: blend is straightforward."""
        odo = Odometry()
        odo.heading = 0.5
        odo.correct(0.0, 0.0, 1.0, 0.0, 0.5, 1000.0)
        # dh = wrap_pi(1.0 - 0.5) = 0.5; result = 0.5 + 0.5*0.5 = 0.75
        assert odo.heading == pytest.approx(0.75, abs=1e-6)

    def test_blend_zero_heading_error(self):
        """OTOS heading equals predicted: no change."""
        odo = Odometry()
        odo.heading = 1.2
        odo.correct(0.0, 0.0, 1.2, 0.0, 0.5, 1000.0)
        assert odo.heading == pytest.approx(1.2, abs=1e-6)


# ---------------------------------------------------------------------------
# Tests — chained predict + correct
# ---------------------------------------------------------------------------

class TestPredictThenCorrect:
    """Verify predict() followed by correct() works as the combined predict/correct cycle."""

    TRACKWIDTH = 120.0

    def test_correct_reduces_x_drift(self):
        """After forward predict, an OTOS reading closer to truth reduces x error."""
        odo = Odometry()
        # Predict: 100 mm forward on both wheels
        odo.predict(100.0, 100.0, self.TRACKWIDTH)
        x_predicted = odo.x  # should be ≈ 100 mm

        # Suppose ground truth is 98 mm. OTOS reports 98 mm.
        # With alpha=0.5, gate=50: x_new = 100 + 0.5*(98 - 100) = 99 mm
        odo.correct(98.0, 0.0, 0.0, 0.5, 0.0, 50.0)
        assert odo.x < x_predicted   # moved toward OTOS (98 < 100)
        assert odo.x == pytest.approx(99.0, abs=1e-5)

    def test_multiple_cycles_converge(self):
        """Repeated correct() calls converge toward OTOS truth exponentially."""
        odo = Odometry()
        odo.x = 100.0  # simulate predicted drift

        target_x = 0.0
        alpha = 0.15
        gate = 200.0

        for _ in range(30):
            odo.correct(target_x, 0.0, 0.0, alpha, 0.0, gate)

        # After 30 iterations: x = 100 * (1-0.15)^30 ≈ 100 * 0.85^30 ≈ 0.76
        expected = 100.0 * ((1 - alpha) ** 30)
        assert odo.x == pytest.approx(expected, abs=0.01)

    def test_rejected_sample_followed_by_accepted(self):
        """After a rejected sample, a valid in-gate sample still applies correctly."""
        odo = Odometry()
        # Out-of-gate: rejected
        odo.correct(200.0, 0.0, 0.0, 0.15, 0.10, 50.0)
        assert odo.otos_rejected == 1
        assert odo.x == pytest.approx(0.0, abs=1e-6)

        # In-gate: accepted
        odo.correct(20.0, 0.0, 0.0, 0.15, 0.10, 50.0)
        assert odo.otos_rejected == 1  # counter not incremented again
        assert odo.x == pytest.approx(0.15 * 20.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Tests — OTOS unit conversion (verify constants)
# ---------------------------------------------------------------------------

class TestOtosUnitConversion:
    """Verify the LSB-to-mm and LSB-to-rad conversion factors match the ticket spec.

    Ticket spec (from OtosSensor.h register map comments):
      Position: 1 LSB = 0.305 mm  → x_mm = raw_x * 0.305
      Heading:  1 LSB = 0.00549°  → θ_rad = raw_h * 0.00549 * (π/180)

    These are the constants DriveController.cpp uses before calling correct().
    """

    POS_MM_PER_LSB  = 0.305
    HDG_DEG_PER_LSB = 0.00549

    def test_position_conversion_1000_lsb(self):
        """1000 LSB → 305.0 mm."""
        mm = 1000 * self.POS_MM_PER_LSB
        assert mm == pytest.approx(305.0, rel=1e-4)

    def test_heading_conversion_1000_lsb(self):
        """1000 LSB → 0.00549 * 1000 ° → in radians."""
        deg = 1000 * self.HDG_DEG_PER_LSB
        rad = deg * (math.pi / 180.0)
        assert rad == pytest.approx(math.radians(5.49), rel=1e-4)

    def test_heading_one_full_revolution_lsb_count(self):
        """360 / 0.00549 ≈ 65573 LSBs for a full revolution."""
        lsb_per_rev = 360.0 / self.HDG_DEG_PER_LSB
        assert lsb_per_rev == pytest.approx(65573.0, rel=1e-3)


# ---------------------------------------------------------------------------
# OTOS mounting-offset transform tests (012-007)
# ---------------------------------------------------------------------------
# Pure-Python mirror of the chip-frame → robot-center transform applied in
# DriveController::tick() before calling _odo.correct().
# Mirrors poseRobotFrame() from src/otos.ts.
# ---------------------------------------------------------------------------

def otos_to_robot_frame(
    x_chip: float,
    y_chip: float,
    h_chip_rad: float,
    odom_off_x: float,
    odom_off_y: float,
    odom_yaw_deg: float,
    odom_upside_down: bool,
) -> tuple[float, float, float]:
    """Python mirror of the DriveController 012-007 mounting-offset transform.

    Steps (match DriveController.cpp exactly):
      1. If upside-down: negate x, y, heading (Z-axis flip).
      2. Rotate chip frame by -odomYawDeg, subtract mounting offset.
      3. Heading += odomYawDeg (in radians).

    At defaults (all zeros/false): returns (x_chip, y_chip, h_chip_rad) unchanged.
    """
    xF, yF, hF = x_chip, y_chip, h_chip_rad

    if odom_upside_down:
        xF, yF, hF = -xF, -yF, -hF

    ang_rad = -odom_yaw_deg * (math.pi / 180.0)
    c = math.cos(ang_rad)
    s = math.sin(ang_rad)
    x_mm = c * xF - s * yF - odom_off_x
    y_mm = s * xF + c * yF - odom_off_y
    h_rad = hF + odom_yaw_deg * (math.pi / 180.0)

    return x_mm, y_mm, h_rad


class TestOtosMountingOffsetTransform:
    """Unit tests for the OTOS chip-frame → robot-center mounting offset transform.

    Sprint 012, Ticket 007.
    """

    # ── Identity (default config) ──────────────────────────────────────────

    def test_identity_at_defaults_x(self):
        """At default offsets (all zero, no flip), x passes through unchanged."""
        x, _y, _h = otos_to_robot_frame(100.0, 0.0, 0.0, 0.0, 0.0, 0.0, False)
        assert x == pytest.approx(100.0, abs=1e-6)

    def test_identity_at_defaults_y(self):
        """At default offsets, y passes through unchanged."""
        _x, y, _h = otos_to_robot_frame(0.0, 200.0, 0.0, 0.0, 0.0, 0.0, False)
        assert y == pytest.approx(200.0, abs=1e-6)

    def test_identity_at_defaults_heading(self):
        """At default offsets, heading passes through unchanged."""
        h_in = math.pi / 4
        _x, _y, h = otos_to_robot_frame(0.0, 0.0, h_in, 0.0, 0.0, 0.0, False)
        assert h == pytest.approx(h_in, abs=1e-6)

    def test_identity_all_nonzero_input_defaults(self):
        """With all offsets at default, arbitrary chip pose passes through unchanged."""
        x, y, h = otos_to_robot_frame(37.5, -18.2, 0.523, 0.0, 0.0, 0.0, False)
        assert x == pytest.approx(37.5, abs=1e-6)
        assert y == pytest.approx(-18.2, abs=1e-6)
        assert h == pytest.approx(0.523, abs=1e-6)

    # ── Translation offset only ────────────────────────────────────────────

    def test_x_offset_subtracts_from_x(self):
        """odomOffX is subtracted from x (with yaw=0)."""
        x, y, h = otos_to_robot_frame(100.0, 0.0, 0.0, 10.0, 0.0, 0.0, False)
        assert x == pytest.approx(90.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)

    def test_y_offset_subtracts_from_y(self):
        """odomOffY is subtracted from y (with yaw=0)."""
        x, y, h = otos_to_robot_frame(0.0, 50.0, 0.0, 0.0, 5.0, 0.0, False)
        assert x == pytest.approx(0.0, abs=1e-6)
        assert y == pytest.approx(45.0, abs=1e-6)

    def test_both_offsets(self):
        """Both offsets applied simultaneously."""
        x, y, h = otos_to_robot_frame(100.0, 50.0, 0.0, 10.0, 5.0, 0.0, False)
        assert x == pytest.approx(90.0, abs=1e-6)
        assert y == pytest.approx(45.0, abs=1e-6)

    # ── Yaw rotation ──────────────────────────────────────────────────────

    def test_yaw_90_rotates_x_to_minus_y(self):
        """90° yaw: chip +X maps to robot -Y (rotation by -90°)."""
        # angRad = -90°; cos(-90)=0, sin(-90)=-1
        # x_mm = 0*xF - (-1)*yF - 0 = yF; y_mm = -1*xF + 0*yF - 0 = -xF
        # With xF=100, yF=0: x_mm=0, y_mm=-100
        x, y, h = otos_to_robot_frame(100.0, 0.0, 0.0, 0.0, 0.0, 90.0, False)
        assert x == pytest.approx(0.0, abs=1e-5)
        assert y == pytest.approx(-100.0, abs=1e-5)

    def test_yaw_90_heading_correction(self):
        """90° yaw adds 90° (π/2 rad) to heading."""
        _, _, h = otos_to_robot_frame(0.0, 0.0, 0.0, 0.0, 0.0, 90.0, False)
        assert h == pytest.approx(math.pi / 2.0, abs=1e-6)

    def test_yaw_45_combined_rotation(self):
        """45° yaw rotates the chip (1,0) vector by -45° = (cos(-45), sin(-45))."""
        c45 = math.cos(math.radians(-45.0))
        s45 = math.sin(math.radians(-45.0))
        x, y, h = otos_to_robot_frame(1.0, 0.0, 0.0, 0.0, 0.0, 45.0, False)
        assert x == pytest.approx(c45, abs=1e-6)
        assert y == pytest.approx(s45, abs=1e-6)

    def test_yaw_and_offset_combined(self):
        """Yaw rotation then translation offset are both applied correctly."""
        # 90° yaw: chip (100, 0) → robot (0, -100); then subtract offX=5, offY=3
        x, y, h = otos_to_robot_frame(100.0, 0.0, 0.0, 5.0, 3.0, 90.0, False)
        assert x == pytest.approx(-5.0, abs=1e-5)
        assert y == pytest.approx(-103.0, abs=1e-5)

    # ── Upside-down flip ──────────────────────────────────────────────────

    def test_upside_down_negates_x(self):
        """Upside-down flag negates chip X before the rotation."""
        # upsideDown negates xF; then with yaw=0 and no offset: x_mm = -x_chip
        x, y, h = otos_to_robot_frame(100.0, 0.0, 0.0, 0.0, 0.0, 0.0, True)
        assert x == pytest.approx(-100.0, abs=1e-6)

    def test_upside_down_negates_y(self):
        """Upside-down flag negates chip Y before the rotation."""
        x, y, h = otos_to_robot_frame(0.0, 50.0, 0.0, 0.0, 0.0, 0.0, True)
        assert y == pytest.approx(-50.0, abs=1e-6)

    def test_upside_down_negates_heading(self):
        """Upside-down flag negates chip heading before the yaw addition."""
        h_in = math.pi / 4
        _, _, h = otos_to_robot_frame(0.0, 0.0, h_in, 0.0, 0.0, 0.0, True)
        assert h == pytest.approx(-h_in, abs=1e-6)

    def test_upside_down_false_leaves_sign_unchanged(self):
        """Without upside-down, positive chip X stays positive."""
        x, _, _ = otos_to_robot_frame(80.0, 0.0, 0.0, 0.0, 0.0, 0.0, False)
        assert x > 0.0

    def test_upside_down_with_yaw(self):
        """Flip then rotate: chip (100, 0) → flip → (-100, 0) → rotate -90° → (0, 100)."""
        # After flip: xF=-100, yF=0
        # angRad = -90°: cos=0, sin=-1
        # x_mm = 0*(-100) - (-1)*0 = 0; y_mm = (-1)*(-100) + 0*0 = 100
        x, y, _ = otos_to_robot_frame(100.0, 0.0, 0.0, 0.0, 0.0, 90.0, True)
        assert x == pytest.approx(0.0, abs=1e-5)
        assert y == pytest.approx(100.0, abs=1e-5)

    # ── Nezha robot: defaults are a no-op ──────────────────────────────────

    def test_nezha_defaults_are_identity_for_typical_pose(self):
        """Nezha robot defaults (all zero, no flip): realistic pose is unchanged."""
        # Typical pose after 200 mm forward travel, slight yaw
        x_chip, y_chip = 198.5, 3.2
        h_chip = math.radians(1.5)
        x, y, h = otos_to_robot_frame(x_chip, y_chip, h_chip, 0.0, 0.0, 0.0, False)
        assert x == pytest.approx(x_chip, abs=1e-6)
        assert y == pytest.approx(y_chip, abs=1e-6)
        assert h == pytest.approx(h_chip, abs=1e-6)
