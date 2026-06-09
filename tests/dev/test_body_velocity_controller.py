#!/usr/bin/env python3
"""test_body_velocity_controller.py — Unit tests for BodyVelocityController math (017-002).

Pure Python implementation of the same algorithm defined in:
  source/control/BodyVelocityController.h / .cpp

Tests verify the body-level (v, ω) trapezoid profiler:
  - Linear ramp slope = aMax (acceleration) and aDecel (deceleration).
  - Yaw ramp slope = yawAccMax_rad * dt.
  - vBodyMax clamping on live _v.
  - yawRateMax clamping on live _omega.
  - Spin-in-place (v=0, omega>0) produces non-zero, opposite-sign wheel targets.
  - Straight (omega=0) produces equal left/right wheel targets.
  - atTarget() semantics: false while ramping, true after convergence.
  - reset() zeroes all state.
  - seedCurrent() sets _v/_omega; next advance ramps from seeded values.
  - Wheel math: (v, omega) -> inverse -> saturate -> (sL, sR) matches manual Python.

Unit conventions (match source/control/BodyKinematics.h):
  v      : mm/s (body forward speed)
  omega  : rad/s CCW-positive (body yaw rate)
  vL, vR : mm/s (wheel speeds, signed)
  b      : mm (track width)
  aMax   : mm/s² (acceleration limit)
  aDecel : mm/s² (deceleration limit)
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Pure Python mirrors
# ---------------------------------------------------------------------------

def bk_inverse(v: float, omega: float, b: float) -> tuple[float, float]:
    """vL = v - omega*(b/2), vR = v + omega*(b/2)."""
    half_b = b / 2.0
    return v - omega * half_b, v + omega * half_b


def bk_saturate(vL: float, vR: float,
                vWheelMax: float, steerHeadroom: float) -> tuple[float, float]:
    """Scale both wheel speeds when max(|vL|, |vR|) > (vWheelMax - steerHeadroom)."""
    ceiling = vWheelMax - steerHeadroom
    max_abs = max(abs(vL), abs(vR))
    if max_abs > ceiling:
        s = ceiling / max_abs
        return s * vL, s * vR
    return vL, vR


def approach(cur: float, tgt: float, step: float) -> float:
    """cur + clamp(tgt - cur, -step, +step)."""
    delta = tgt - cur
    delta = max(-step, min(+step, delta))
    return cur + delta


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class RobotConfig:
    """Minimal config mirror — only fields used by BodyVelocityController."""

    def __init__(self, **kwargs):
        # Defaults match defaultRobotConfig() in source/types/Config.h
        self.vBodyMax       = kwargs.get('vBodyMax',       400.0)
        self.yawRateMax     = kwargs.get('yawRateMax',     180.0)   # deg/s
        self.yawAccMax      = kwargs.get('yawAccMax',      720.0)   # deg/s²
        self.jMax           = kwargs.get('jMax',             0.0)
        self.yawJerkMax     = kwargs.get('yawJerkMax',       0.0)
        self.aMax           = kwargs.get('aMax',           300.0)   # mm/s²
        self.aDecel         = kwargs.get('aDecel',         250.0)   # mm/s²
        self.trackwidthMm   = kwargs.get('trackwidthMm',   126.0)
        self.vWheelMax      = kwargs.get('vWheelMax',      400.0)
        self.steerHeadroom  = kwargs.get('steerHeadroom',   20.0)


class BodyVelocityController:
    """Python mirror of source/control/BodyVelocityController.cpp.

    Simulates MotorController by storing the last setTarget call in
    last_sL / last_sR so tests can inspect the wheel outputs.
    """

    DEG_TO_RAD = math.pi / 180.0

    def __init__(self, cfg: RobotConfig):
        self.cfg        = cfg
        self._v         = 0.0
        self._omega     = 0.0
        self._vTgt      = 0.0
        self._omegaTgt  = 0.0
        self._aLive     = 0.0     # live linear acceleration (S-curve channel), mm/s²
        self._omegaALive = 0.0   # live yaw acceleration (S-curve channel), rad/s²
        # Simulated MotorController output (last setTarget call).
        self.last_sL  = 0.0
        self.last_sR  = 0.0

    def set_target(self, v_mms: float, omega_rads: float) -> None:
        self._vTgt    = v_mms
        self._omegaTgt = omega_rads

    def advance(self, dt_s: float) -> bool:
        """Step the profiler one control tick. Returns True while still ramping."""
        if dt_s <= 0.0:
            return not self.at_target()

        # Linear channel — asymmetric accel/decel with optional jerk limit.
        #
        # At jMax == 0 (default): pure trapezoid — approach v directly under
        # the per-tick dv_max step (identical to pre-018 behaviour).
        #
        # At jMax > 0: S-curve — slew _aLive toward the demanded acceleration
        # under the jerk bound (jMax * dt_s), then integrate v += _aLive*dt_s.
        vTgt_clamped = clamp(self._vTgt, -self.cfg.vBodyMax, +self.cfg.vBodyMax)

        if self.cfg.jMax > 0.0:
            # S-curve path: jerk-limit the acceleration, then integrate toward
            # the target using approach() so the integration cannot overshoot.
            #
            # aTarget: +aMax when v must increase to reach vTgt_clamped, else -aDecel.
            a_target = self.cfg.aMax if vTgt_clamped >= self._v else -self.cfg.aDecel
            jerk_step = self.cfg.jMax * dt_s
            self._aLive = approach(self._aLive, a_target, jerk_step)
            # Integrate, but cap the step so we never overshoot vTgt_clamped.
            self._v = approach(self._v, vTgt_clamped, abs(self._aLive * dt_s))
        else:
            # Trapezoid path (jMax == 0): identical to pre-018 behaviour.
            # Use aDecel when target is closer to zero than current v (decelerating).
            if abs(vTgt_clamped) >= abs(self._v):
                dv_max = self.cfg.aMax * dt_s
            else:
                dv_max = self.cfg.aDecel * dt_s
            self._v = approach(self._v, vTgt_clamped, dv_max)

        # Yaw channel — symmetric trapezoid with optional jerk limit.
        # Convert deg-based limits to rad at use site.
        yaw_rate_max_rad = self.cfg.yawRateMax * self.DEG_TO_RAD
        yaw_acc_max_rad  = self.cfg.yawAccMax  * self.DEG_TO_RAD
        omega_tgt_clamped = clamp(self._omegaTgt, -yaw_rate_max_rad, +yaw_rate_max_rad)

        if self.cfg.yawJerkMax > 0.0:
            # S-curve path for yaw.  Same approach-based integration as linear
            # channel: prevents overshoot while preserving jerk-limited ramp.
            yaw_jerk_max_rad = self.cfg.yawJerkMax * self.DEG_TO_RAD
            omega_a_target = +yaw_acc_max_rad if omega_tgt_clamped >= self._omega else -yaw_acc_max_rad
            self._omegaALive = approach(self._omegaALive, omega_a_target, yaw_jerk_max_rad * dt_s)
            self._omega = approach(self._omega, omega_tgt_clamped, abs(self._omegaALive * dt_s))
        else:
            # Trapezoid path (yawJerkMax == 0): identical to pre-018 behaviour.
            domega_max = yaw_acc_max_rad * dt_s
            self._omega = approach(self._omega, omega_tgt_clamped, domega_max)

        # Per-tick ordering invariant: profile → inverse → saturate → setTarget.
        vL, vR = bk_inverse(self._v, self._omega, self.cfg.trackwidthMm)
        sL, sR = bk_saturate(vL, vR, self.cfg.vWheelMax, self.cfg.steerHeadroom)
        self.last_sL = sL
        self.last_sR = sR

        return not self.at_target()

    def reset(self) -> None:
        """Zero all profiler state."""
        self._v         = 0.0
        self._omega     = 0.0
        self._vTgt      = 0.0
        self._omegaTgt  = 0.0
        self._aLive     = 0.0
        self._omegaALive = 0.0

    def seed_current(self, v_mms: float, omega_rads: float) -> None:
        """Seed the live profiler position without touching targets."""
        self._v     = v_mms
        self._omega = omega_rads

    def current_v(self)     -> float: return self._v
    def current_omega(self) -> float: return self._omega
    def target_v(self)      -> float: return self._vTgt
    def target_omega(self)  -> float: return self._omegaTgt

    def at_target(self) -> bool:
        """True when within convergence thresholds of the (clamped) target."""
        yaw_rate_max_rad  = self.cfg.yawRateMax * self.DEG_TO_RAD
        vTgt_clamped      = clamp(self._vTgt, -self.cfg.vBodyMax, +self.cfg.vBodyMax)
        omega_tgt_clamped = clamp(self._omegaTgt, -yaw_rate_max_rad, +yaw_rate_max_rad)
        return (abs(self._v     - vTgt_clamped)     < 0.5 and
                abs(self._omega - omega_tgt_clamped) < 0.001)


# ---------------------------------------------------------------------------
# Shared fixture / factory
# ---------------------------------------------------------------------------

def make_bvc(**overrides) -> BodyVelocityController:
    """Create a BodyVelocityController with sensible test defaults."""
    cfg = RobotConfig(**overrides)
    return BodyVelocityController(cfg)


# ---------------------------------------------------------------------------
# Tests — linear ramp slope (acceleration)
# ---------------------------------------------------------------------------

class TestLinearRampSlope:
    """Verify that v advances by exactly aMax*dt per tick while ramping up."""

    DT = 0.01  # s — 10 ms control tick

    def test_accel_slope_each_tick(self):
        """Step v 0→300 mm/s; each tick advances by aMax*dt until target reached."""
        aMax = 300.0
        bvc = make_bvc(aMax=aMax, vBodyMax=400.0)
        bvc.set_target(300.0, 0.0)

        expected_step = aMax * self.DT  # 3.0 mm/s per tick
        v_prev = bvc.current_v()

        ramp_ticks = 0
        for _ in range(200):
            ramping = bvc.advance(self.DT)
            v_now = bvc.current_v()

            if v_prev < 300.0:
                # Still ramping: each step must be exactly expected_step (or reach target).
                actual_step = v_now - v_prev
                if v_now < 300.0:
                    assert actual_step == pytest.approx(expected_step, rel=1e-5), (
                        f"Linear accel step wrong: got {actual_step}, expected {expected_step}"
                    )
                ramp_ticks += 1
            v_prev = v_now

            if not ramping:
                break

        assert ramp_ticks > 0, "No ramp ticks observed"
        assert bvc.current_v() == pytest.approx(300.0, abs=0.5)

    def test_accel_slope_continuous(self):
        """v increases monotonically from 0 to target."""
        bvc = make_bvc(aMax=300.0, vBodyMax=400.0)
        bvc.set_target(200.0, 0.0)
        v_prev = 0.0
        for _ in range(100):
            bvc.advance(self.DT)
            v_now = bvc.current_v()
            assert v_now >= v_prev - 1e-6, (
                f"v decreased during acceleration: {v_now} < {v_prev}"
            )
            v_prev = v_now


# ---------------------------------------------------------------------------
# Tests — linear ramp slope (deceleration)
# ---------------------------------------------------------------------------

class TestLinearDecelSlope:
    """Verify that v decreases by exactly aDecel*dt per tick while ramping down."""

    DT = 0.01  # s

    def test_decel_slope_each_tick(self):
        """Step v 300→0 mm/s; slope = aDecel."""
        aDecel = 250.0
        bvc = make_bvc(aMax=300.0, aDecel=aDecel, vBodyMax=400.0)

        # Seed at 300 mm/s and target 0.
        bvc.seed_current(300.0, 0.0)
        bvc.set_target(0.0, 0.0)

        expected_step = aDecel * self.DT  # 2.5 mm/s per tick

        v_prev = bvc.current_v()
        for _ in range(200):
            bvc.advance(self.DT)
            v_now = bvc.current_v()
            if v_prev > 0.5:
                actual_step = v_prev - v_now  # positive when decelerating
                if v_now > 0.5:
                    assert actual_step == pytest.approx(expected_step, rel=1e-5), (
                        f"Decel step wrong: got {actual_step}, expected {expected_step}"
                    )
            v_prev = v_now
            if bvc.at_target():
                break

        assert bvc.current_v() == pytest.approx(0.0, abs=0.5)

    def test_decel_slope_monotonic(self):
        """v decreases monotonically from 300 to 0."""
        bvc = make_bvc(aDecel=250.0, vBodyMax=400.0)
        bvc.seed_current(300.0, 0.0)
        bvc.set_target(0.0, 0.0)
        v_prev = 300.0
        for _ in range(200):
            bvc.advance(self.DT)
            v_now = bvc.current_v()
            assert v_now <= v_prev + 1e-6, (
                f"v increased during deceleration: {v_now} > {v_prev}"
            )
            v_prev = v_now


# ---------------------------------------------------------------------------
# Tests — yaw ramp slope
# ---------------------------------------------------------------------------

class TestYawRampSlope:
    """Verify that omega advances by yawAccMax_rad * dt per tick."""

    DT = 0.01  # s

    def test_yaw_slope_each_tick(self):
        """Step omega 0→yawRateMax; slope = yawAccMax_rad * dt."""
        yawRateMax_deg  = 180.0   # deg/s
        yawAccMax_deg   = 720.0   # deg/s²
        yaw_rate_max_rad = yawRateMax_deg * math.pi / 180.0
        yaw_acc_max_rad  = yawAccMax_deg  * math.pi / 180.0

        bvc = make_bvc(yawRateMax=yawRateMax_deg, yawAccMax=yawAccMax_deg)
        bvc.set_target(0.0, yaw_rate_max_rad)  # set to max yaw rate

        expected_step = yaw_acc_max_rad * self.DT

        omega_prev = bvc.current_omega()
        for _ in range(200):
            bvc.advance(self.DT)
            omega_now = bvc.current_omega()
            if omega_prev < yaw_rate_max_rad - 0.001:
                actual_step = omega_now - omega_prev
                if omega_now < yaw_rate_max_rad - 0.001:
                    assert actual_step == pytest.approx(expected_step, rel=1e-5), (
                        f"Yaw accel step wrong: got {actual_step}, expected {expected_step}"
                    )
            omega_prev = omega_now
            if bvc.at_target():
                break

        assert bvc.current_omega() == pytest.approx(yaw_rate_max_rad, abs=0.001)

    def test_yaw_slope_monotonic(self):
        """omega increases monotonically from 0 to yawRateMax."""
        yaw_rate_max_rad = 180.0 * math.pi / 180.0
        bvc = make_bvc(yawRateMax=180.0, yawAccMax=720.0)
        bvc.set_target(0.0, yaw_rate_max_rad)

        omega_prev = 0.0
        for _ in range(200):
            bvc.advance(self.DT)
            omega_now = bvc.current_omega()
            assert omega_now >= omega_prev - 1e-6, (
                f"omega decreased while ramping up: {omega_now} < {omega_prev}"
            )
            omega_prev = omega_now


# ---------------------------------------------------------------------------
# Tests — spin-in-place
# ---------------------------------------------------------------------------

class TestSpinInPlace:
    """v=0, omega>0: both wheel targets are non-zero (and opposite sign)."""

    def test_spin_produces_nonzero_wheels(self):
        """Spin command results in non-zero wheel speeds."""
        bvc = make_bvc()
        omega = 1.0  # rad/s
        bvc.set_target(0.0, omega)

        # Ramp up to steady state.
        for _ in range(200):
            bvc.advance(0.01)
            if bvc.at_target():
                break

        # Both wheels non-zero.
        assert bvc.last_sL != pytest.approx(0.0, abs=1.0)
        assert bvc.last_sR != pytest.approx(0.0, abs=1.0)

    def test_spin_wheel_signs_opposite(self):
        """Spin CCW: left wheel backward, right wheel forward."""
        bvc = make_bvc()
        omega = 1.0  # rad/s CCW
        bvc.set_target(0.0, omega)

        for _ in range(200):
            bvc.advance(0.01)
            if bvc.at_target():
                break

        # For CCW spin (omega > 0): vL < 0, vR > 0 (from inverse kinematics).
        assert bvc.last_sL < 0.0, f"sL should be negative for CCW spin, got {bvc.last_sL}"
        assert bvc.last_sR > 0.0, f"sR should be positive for CCW spin, got {bvc.last_sR}"


# ---------------------------------------------------------------------------
# Tests — straight (omega=0)
# ---------------------------------------------------------------------------

class TestStraight:
    """omega=0: both wheel targets must be equal (within float tolerance)."""

    def test_straight_equal_wheels(self):
        """Straight drive produces equal left/right wheel speeds."""
        bvc = make_bvc()
        bvc.set_target(200.0, 0.0)

        for _ in range(200):
            bvc.advance(0.01)
            if bvc.at_target():
                break

        assert bvc.last_sL == pytest.approx(bvc.last_sR, rel=1e-5), (
            f"Straight drive: sL={bvc.last_sL} != sR={bvc.last_sR}"
        )

    def test_straight_each_tick_equal_wheels(self):
        """sL == sR on every tick while driving straight."""
        bvc = make_bvc()
        bvc.set_target(150.0, 0.0)
        for _ in range(50):
            bvc.advance(0.01)
            assert bvc.last_sL == pytest.approx(bvc.last_sR, rel=1e-5), (
                f"sL != sR during straight drive tick: {bvc.last_sL}, {bvc.last_sR}"
            )


# ---------------------------------------------------------------------------
# Tests — vBodyMax clamping
# ---------------------------------------------------------------------------

class TestVBodyMaxClamp:
    """Live _v must never exceed vBodyMax even if target is higher."""

    def test_v_never_exceeds_vbodymax(self):
        """Target v=600, vBodyMax=400: _v stays <= 400."""
        vBodyMax = 400.0
        bvc = make_bvc(vBodyMax=vBodyMax, aMax=300.0)
        bvc.set_target(600.0, 0.0)

        for _ in range(200):
            bvc.advance(0.01)
            assert bvc.current_v() <= vBodyMax + 1e-4, (
                f"_v exceeded vBodyMax: {bvc.current_v()} > {vBodyMax}"
            )

    def test_v_converges_to_vbodymax_not_target(self):
        """When target > vBodyMax, _v converges to vBodyMax."""
        vBodyMax = 400.0
        bvc = make_bvc(vBodyMax=vBodyMax, aMax=300.0)
        bvc.set_target(600.0, 0.0)

        for _ in range(300):
            bvc.advance(0.01)
            if bvc.at_target():
                break

        assert bvc.current_v() == pytest.approx(vBodyMax, abs=0.5)


# ---------------------------------------------------------------------------
# Tests — yawRateMax clamping
# ---------------------------------------------------------------------------

class TestYawRateMaxClamp:
    """Live _omega must never exceed yawRateMax (in rad/s)."""

    def test_omega_never_exceeds_yawratemax(self):
        """Target omega above limit: _omega stays within ±yawRateMax_rad."""
        yawRateMax_deg = 90.0
        yaw_rate_max_rad = yawRateMax_deg * math.pi / 180.0
        big_omega = 10.0  # well above limit

        bvc = make_bvc(yawRateMax=yawRateMax_deg, yawAccMax=720.0)
        bvc.set_target(0.0, big_omega)

        for _ in range(200):
            bvc.advance(0.01)
            assert bvc.current_omega() <= yaw_rate_max_rad + 1e-5, (
                f"_omega exceeded limit: {bvc.current_omega()} > {yaw_rate_max_rad}"
            )

    def test_omega_converges_to_yawratemax(self):
        """Target omega above limit: _omega converges to yawRateMax_rad."""
        yawRateMax_deg = 90.0
        yaw_rate_max_rad = yawRateMax_deg * math.pi / 180.0

        bvc = make_bvc(yawRateMax=yawRateMax_deg, yawAccMax=720.0)
        bvc.set_target(0.0, 10.0)

        for _ in range(300):
            bvc.advance(0.01)
            if bvc.at_target():
                break

        assert bvc.current_omega() == pytest.approx(yaw_rate_max_rad, abs=0.001)


# ---------------------------------------------------------------------------
# Tests — atTarget()
# ---------------------------------------------------------------------------

class TestAtTarget:
    """atTarget() is false while ramping, true once converged."""

    def test_at_target_false_while_ramping(self):
        """atTarget() returns False immediately after setTarget on a fresh controller."""
        bvc = make_bvc()
        bvc.set_target(300.0, 0.0)
        # Before any advance, _v=0, target=300 -> not at target.
        assert bvc.at_target() is False

    def test_at_target_true_after_ramp(self):
        """atTarget() returns True once _v has converged on vTgt."""
        bvc = make_bvc(aMax=300.0, vBodyMax=400.0)
        bvc.set_target(200.0, 0.0)

        reached = False
        for _ in range(300):
            bvc.advance(0.01)
            if bvc.at_target():
                reached = True
                break

        assert reached, "atTarget() never returned True after sufficient advance ticks"

    def test_advance_returns_false_when_at_target(self):
        """advance() returns False (not-ramping) once at target."""
        bvc = make_bvc(aMax=300.0, vBodyMax=400.0)
        bvc.set_target(100.0, 0.0)

        last_result = True
        for _ in range(200):
            last_result = bvc.advance(0.01)
            if not last_result:
                break

        assert last_result is False, "advance() should return False when at target"

    def test_at_target_initially_true_for_zero_target(self):
        """Fresh controller with target (0, 0) is already at target."""
        bvc = make_bvc()
        # _v=0, _omega=0, _vTgt=0, _omegaTgt=0 — already converged.
        assert bvc.at_target() is True


# ---------------------------------------------------------------------------
# Tests — reset()
# ---------------------------------------------------------------------------

class TestReset:
    """reset() zeros all four state variables."""

    def test_reset_zeroes_current_v(self):
        """After driving, reset() → currentV() == 0."""
        bvc = make_bvc()
        bvc.seed_current(200.0, 0.5)
        bvc.set_target(300.0, 1.0)
        bvc.advance(0.01)
        bvc.reset()
        assert bvc.current_v() == pytest.approx(0.0)

    def test_reset_zeroes_current_omega(self):
        """After driving, reset() → currentOmega() == 0."""
        bvc = make_bvc()
        bvc.seed_current(0.0, 1.5)
        bvc.advance(0.01)
        bvc.reset()
        assert bvc.current_omega() == pytest.approx(0.0)

    def test_reset_zeroes_targets(self):
        """reset() clears _vTgt and _omegaTgt."""
        bvc = make_bvc()
        bvc.set_target(300.0, 2.0)
        bvc.reset()
        assert bvc.target_v()     == pytest.approx(0.0)
        assert bvc.target_omega() == pytest.approx(0.0)

    def test_reset_makes_at_target_true(self):
        """After reset(), at_target() returns True (all zeros)."""
        bvc = make_bvc()
        bvc.seed_current(200.0, 1.0)
        bvc.set_target(200.0, 1.0)
        bvc.advance(0.01)
        bvc.reset()
        assert bvc.at_target() is True


# ---------------------------------------------------------------------------
# Tests — seedCurrent()
# ---------------------------------------------------------------------------

class TestSeedCurrent:
    """seedCurrent() sets _v/_omega; advance ramps from seeded values."""

    def test_seed_sets_current_v(self):
        """seedCurrent sets _v correctly."""
        bvc = make_bvc()
        bvc.seed_current(150.0, 0.0)
        assert bvc.current_v() == pytest.approx(150.0)

    def test_seed_sets_current_omega(self):
        """seedCurrent sets _omega correctly."""
        bvc = make_bvc()
        bvc.seed_current(0.0, 0.75)
        assert bvc.current_omega() == pytest.approx(0.75)

    def test_seed_ramps_from_seeded_v(self):
        """After seed(100), next advance(dt) starts at 100, not 0."""
        aMax = 300.0
        dt   = 0.01
        bvc  = make_bvc(aMax=aMax, vBodyMax=400.0)
        bvc.seed_current(100.0, 0.0)
        bvc.set_target(200.0, 0.0)
        bvc.advance(dt)
        expected = 100.0 + aMax * dt  # 103.0
        assert bvc.current_v() == pytest.approx(expected, rel=1e-5), (
            f"Expected v={expected}, got {bvc.current_v()}"
        )

    def test_seed_ramps_from_seeded_omega(self):
        """After seed omega, next advance ramps from the seeded value."""
        yaw_acc_rad = 720.0 * math.pi / 180.0
        dt = 0.01
        bvc = make_bvc(yawAccMax=720.0, yawRateMax=180.0)
        seed_omega = 0.5
        bvc.seed_current(0.0, seed_omega)
        target_omega = 2.0  # above yawRateMax (π rad/s ≈ 3.14) — will be clamped
        bvc.set_target(0.0, target_omega)
        bvc.advance(dt)
        yaw_rate_max_rad = 180.0 * math.pi / 180.0
        # Target is clamped to yawRateMax; seed_omega < max, so we ramp up.
        expected = seed_omega + yaw_acc_rad * dt
        assert bvc.current_omega() == pytest.approx(expected, rel=1e-5)


# ---------------------------------------------------------------------------
# Tests — wheel math (inverse → saturate → setTarget)
# ---------------------------------------------------------------------------

class TestWheelMath:
    """Verify (sL, sR) from advance matches manual Python computation."""

    def test_straight_wheel_math(self):
        """Straight drive: sL and sR match manual inverse + saturate computation."""
        cfg = RobotConfig(vBodyMax=400.0, aMax=300.0, trackwidthMm=126.0,
                          vWheelMax=400.0, steerHeadroom=20.0)
        bvc = BodyVelocityController(cfg)

        # Seed at v=200, omega=0 so we're already at target after one tick.
        bvc.seed_current(200.0, 0.0)
        bvc.set_target(200.0, 0.0)
        bvc.advance(0.01)

        # Manual computation.
        vL, vR = bk_inverse(200.0, 0.0, 126.0)
        sL_expected, sR_expected = bk_saturate(vL, vR, 400.0, 20.0)

        assert bvc.last_sL == pytest.approx(sL_expected, rel=1e-5)
        assert bvc.last_sR == pytest.approx(sR_expected, rel=1e-5)

    def test_spin_wheel_math(self):
        """Spin-in-place: sL and sR match manual inverse + saturate computation."""
        omega = 1.0  # rad/s CCW
        cfg = RobotConfig(vBodyMax=400.0, yawRateMax=180.0, yawAccMax=720.0,
                          trackwidthMm=126.0, vWheelMax=400.0, steerHeadroom=20.0)
        bvc = BodyVelocityController(cfg)
        bvc.seed_current(0.0, omega)
        bvc.set_target(0.0, omega)
        bvc.advance(0.01)

        vL, vR = bk_inverse(0.0, omega, 126.0)
        sL_expected, sR_expected = bk_saturate(vL, vR, 400.0, 20.0)

        assert bvc.last_sL == pytest.approx(sL_expected, rel=1e-5)
        assert bvc.last_sR == pytest.approx(sR_expected, rel=1e-5)

    def test_arc_wheel_math(self):
        """Arc (v>0, omega>0): sL/sR match inverse + saturate for the live (v, omega)."""
        v     = 150.0
        omega = 0.5   # rad/s
        cfg = RobotConfig(vBodyMax=400.0, yawRateMax=180.0, yawAccMax=720.0,
                          aMax=300.0, trackwidthMm=126.0,
                          vWheelMax=400.0, steerHeadroom=20.0)
        bvc = BodyVelocityController(cfg)
        bvc.seed_current(v, omega)
        bvc.set_target(v, omega)
        bvc.advance(0.01)

        # Live (v, omega) after advance — seeded = target, so no ramp.
        v_live     = bvc.current_v()
        omega_live = bvc.current_omega()
        vL, vR     = bk_inverse(v_live, omega_live, 126.0)
        sL_exp, sR_exp = bk_saturate(vL, vR, 400.0, 20.0)

        assert bvc.last_sL == pytest.approx(sL_exp, rel=1e-5)
        assert bvc.last_sR == pytest.approx(sR_exp, rel=1e-5)

    def test_saturated_wheel_math(self):
        """When inverse output exceeds ceiling, saturate scales both wheels."""
        # v large enough that vR would exceed vWheelMax - steerHeadroom.
        # With trackwidthMm=126, omega=2.0: vR = v + omega*(63) = 350+126 = 476 > 380.
        v     = 350.0
        omega = 2.0
        cfg = RobotConfig(vBodyMax=500.0, yawRateMax=360.0, yawAccMax=720.0,
                          aMax=300.0, trackwidthMm=126.0,
                          vWheelMax=400.0, steerHeadroom=20.0)
        bvc = BodyVelocityController(cfg)
        bvc.seed_current(v, omega)
        bvc.set_target(v, omega)
        bvc.advance(0.01)

        v_live, omega_live = bvc.current_v(), bvc.current_omega()
        vL, vR = bk_inverse(v_live, omega_live, 126.0)
        sL_exp, sR_exp = bk_saturate(vL, vR, 400.0, 20.0)

        assert bvc.last_sL == pytest.approx(sL_exp, rel=1e-5)
        assert bvc.last_sR == pytest.approx(sR_exp, rel=1e-5)
        # The faster wheel should be at the ceiling (380 mm/s).
        assert max(abs(bvc.last_sL), abs(bvc.last_sR)) == pytest.approx(380.0, rel=1e-5)


# ---------------------------------------------------------------------------
# Tests — monotonic convergence
# ---------------------------------------------------------------------------

class TestMonotonicConvergence:
    """Both channels converge monotonically from zero to target."""

    def test_v_converges_monotonically(self):
        """v increases monotonically from 0 to 200 mm/s."""
        bvc = make_bvc(aMax=300.0, vBodyMax=400.0)
        bvc.set_target(200.0, 0.0)
        v_prev = 0.0
        for _ in range(100):
            bvc.advance(0.01)
            v_now = bvc.current_v()
            assert v_now >= v_prev - 1e-6
            v_prev = v_now
        assert bvc.current_v() == pytest.approx(200.0, abs=0.5)

    def test_omega_converges_monotonically(self):
        """omega increases monotonically from 0 to yawRateMax."""
        yaw_rate_max_rad = 90.0 * math.pi / 180.0
        bvc = make_bvc(yawRateMax=90.0, yawAccMax=360.0)
        bvc.set_target(0.0, yaw_rate_max_rad)
        omega_prev = 0.0
        for _ in range(200):
            bvc.advance(0.01)
            omega_now = bvc.current_omega()
            assert omega_now >= omega_prev - 1e-6
            omega_prev = omega_now
        assert bvc.current_omega() == pytest.approx(yaw_rate_max_rad, abs=0.001)


# ---------------------------------------------------------------------------
# Tests — S-curve (jerk-limited) activation (018-007)
# ---------------------------------------------------------------------------

class TestSCurveLinear:
    """jMax > 0: acceleration ramps rather than steps; v reaches target later than
    trapezoid with same aMax; degenerates to trapezoid at jMax=0."""

    DT = 0.010  # s — 10 ms control tick
    N_TICKS = 50

    def _simulate_v(self, **kwargs) -> list:
        """Run N_TICKS ticks and return list of v after each tick."""
        bvc = make_bvc(**kwargs)
        bvc.set_target(300.0, 0.0)
        vs = []
        for _ in range(self.N_TICKS):
            bvc.advance(self.DT)
            vs.append(bvc.current_v())
        return vs

    def test_jmax_zero_degenerates_to_trapezoid(self):
        """At jMax=0 (default), advance() is byte-identical to pure trapezoid."""
        aMax = 300.0
        dt   = self.DT
        # Build reference trapezoid by hand.
        ref_v = 0.0
        vBodyMax = 400.0
        trap_vs = []
        for _ in range(self.N_TICKS):
            ref_v = approach(ref_v, min(300.0, vBodyMax), aMax * dt)
            trap_vs.append(ref_v)

        bvc_vs = self._simulate_v(aMax=aMax, jMax=0.0, vBodyMax=vBodyMax)
        for i, (bv, tv) in enumerate(zip(bvc_vs, trap_vs)):
            assert bv == pytest.approx(tv, rel=1e-6), (
                f"tick {i}: jMax=0 BVC v={bv} != trapezoid v={tv}"
            )

    def test_scurve_slower_than_trapezoid_at_tick20(self):
        """With jMax > 0, v at tick 20 is less than trapezoid v at tick 20 (accel ramps)."""
        aMax  = 300.0
        jMax  = 1000.0   # mm/s³ — jerk bound activates S-curve

        scurve_vs  = self._simulate_v(aMax=aMax, jMax=jMax,  vBodyMax=400.0)
        trap_vs    = self._simulate_v(aMax=aMax, jMax=0.0,   vBodyMax=400.0)

        # S-curve must be strictly slower at tick 20 (index 19) because the
        # acceleration has not yet reached aMax (it is still ramping up via jerk).
        assert scurve_vs[19] < trap_vs[19], (
            f"S-curve v at tick 20 ({scurve_vs[19]:.2f}) should be < trapezoid "
            f"({trap_vs[19]:.2f}) when jMax={jMax}"
        )

    def test_scurve_converges_to_target(self):
        """jMax > 0: v eventually converges to the target (given enough ticks)."""
        bvc = make_bvc(aMax=300.0, jMax=1000.0, vBodyMax=400.0)
        bvc.set_target(200.0, 0.0)
        for _ in range(500):
            bvc.advance(self.DT)
            if bvc.at_target():
                break
        assert bvc.current_v() == pytest.approx(200.0, abs=0.5), (
            f"S-curve did not converge: v={bvc.current_v()}"
        )

    def test_scurve_acceleration_ramps_not_steps(self):
        """With jMax active, the acceleration (dv/tick) grows gradually, not instantly."""
        aMax = 300.0
        jMax = 500.0   # mm/s³ — moderate jerk bound
        dt   = self.DT

        bvc = make_bvc(aMax=aMax, jMax=jMax, vBodyMax=400.0)
        bvc.set_target(300.0, 0.0)

        # At jMax=0 (trapezoid), the first tick already steps by aMax*dt = 3 mm/s.
        # With jMax active, the first tick steps by at most jMax*dt*dt = 0.05 mm/s
        # (because _aLive starts at 0 and can only increase by jMax*dt per tick).
        bvc.advance(dt)
        first_tick_v = bvc.current_v()
        # S-curve first step << trapezoid first step (3.0 mm/s).
        assert first_tick_v < aMax * dt, (
            f"First tick v={first_tick_v:.4f} should be < aMax*dt={aMax*dt:.2f} with jMax active"
        )

    def test_scurve_vbodymax_respected(self):
        """With jMax active, live v never exceeds vBodyMax."""
        vBodyMax = 200.0
        bvc = make_bvc(aMax=300.0, jMax=800.0, vBodyMax=vBodyMax)
        bvc.set_target(500.0, 0.0)  # target above vBodyMax

        for _ in range(200):
            bvc.advance(self.DT)
            assert bvc.current_v() <= vBodyMax + 1e-4, (
                f"v={bvc.current_v():.4f} exceeded vBodyMax={vBodyMax}"
            )

    def test_scurve_reset_zeroes_alive(self):
        """reset() clears _aLive so the next S-curve run starts from zero."""
        bvc = make_bvc(aMax=300.0, jMax=1000.0, vBodyMax=400.0)
        bvc.set_target(200.0, 0.0)
        for _ in range(20):
            bvc.advance(self.DT)
        # _aLive is now non-zero.
        bvc.reset()
        assert bvc._aLive == pytest.approx(0.0), "_aLive not zeroed by reset()"
        assert bvc._omegaALive == pytest.approx(0.0), "_omegaALive not zeroed by reset()"


class TestSCurveYaw:
    """yawJerkMax > 0: omega acceleration ramps; degenerates to trapezoid at 0."""

    DT = 0.010
    N_TICKS = 50

    def _simulate_omega(self, **kwargs) -> list:
        yaw_rate_max_rad = 180.0 * math.pi / 180.0
        bvc = make_bvc(**kwargs)
        bvc.set_target(0.0, yaw_rate_max_rad)
        omegas = []
        for _ in range(self.N_TICKS):
            bvc.advance(self.DT)
            omegas.append(bvc.current_omega())
        return omegas

    def test_yawjerkmax_zero_degenerates_to_trapezoid(self):
        """At yawJerkMax=0 (default), yaw channel is byte-identical to trapezoid."""
        yaw_acc_max_deg = 720.0
        dt = self.DT
        yaw_rate_max_rad = 180.0 * math.pi / 180.0
        yaw_acc_max_rad  = yaw_acc_max_deg * math.pi / 180.0

        ref_omega = 0.0
        trap_omegas = []
        for _ in range(self.N_TICKS):
            ref_omega = approach(ref_omega, yaw_rate_max_rad, yaw_acc_max_rad * dt)
            trap_omegas.append(ref_omega)

        bvc_omegas = self._simulate_omega(yawRateMax=180.0, yawAccMax=yaw_acc_max_deg,
                                          yawJerkMax=0.0)
        for i, (bv, tv) in enumerate(zip(bvc_omegas, trap_omegas)):
            assert bv == pytest.approx(tv, rel=1e-6), (
                f"tick {i}: yawJerkMax=0 BVC omega={bv} != trapezoid omega={tv}"
            )

    def test_yaw_scurve_slower_than_trapezoid(self):
        """With yawJerkMax > 0, omega at tick 10 is less than trapezoid omega."""
        scurve  = self._simulate_omega(yawRateMax=180.0, yawAccMax=720.0, yawJerkMax=5000.0)
        trap    = self._simulate_omega(yawRateMax=180.0, yawAccMax=720.0, yawJerkMax=0.0)
        assert scurve[9] < trap[9], (
            f"S-curve omega at tick 10 ({scurve[9]:.4f} rad/s) should be < trapezoid "
            f"({trap[9]:.4f} rad/s)"
        )

    def test_yaw_scurve_converges(self):
        """yawJerkMax > 0: omega converges to yawRateMax given enough ticks."""
        yaw_rate_max_rad = 180.0 * math.pi / 180.0
        bvc = make_bvc(yawRateMax=180.0, yawAccMax=720.0, yawJerkMax=3000.0)
        bvc.set_target(0.0, yaw_rate_max_rad)
        for _ in range(500):
            bvc.advance(self.DT)
            if bvc.at_target():
                break
        assert bvc.current_omega() == pytest.approx(yaw_rate_max_rad, abs=0.001), (
            f"Yaw S-curve did not converge: omega={bvc.current_omega()}"
        )

    def test_yaw_scurve_rate_max_respected(self):
        """With yawJerkMax active, live omega never exceeds yawRateMax."""
        yaw_rate_max_rad = 90.0 * math.pi / 180.0
        bvc = make_bvc(yawRateMax=90.0, yawAccMax=720.0, yawJerkMax=5000.0)
        bvc.set_target(0.0, 10.0)  # well above limit
        for _ in range(200):
            bvc.advance(self.DT)
            assert bvc.current_omega() <= yaw_rate_max_rad + 1e-5, (
                f"omega={bvc.current_omega():.5f} exceeded yawRateMax={yaw_rate_max_rad:.5f}"
            )
