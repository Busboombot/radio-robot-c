"""
test_bench_otos.py — BenchOtosSensor sim hook tests.

031-002 (wiring tests, lines below this docstring through end of file):
  - BenchOtosSensor starts at zero pose.
  - Ticking with a straight-ahead commanded velocity advances the ideal X
    accumulator (pose moves forward).
  - After reset() both accumulators return to zero.
  - Ticking with zero velocity (bench mode off) leaves pose unchanged.
  - With no noise set, errored == ideal (noiseless at default parameters).

031-004 (integrator correctness tests, near end of file):
  - Zero-error oracle: with noise=0 the ERRORED pose == IDEAL pose ==
    closed-form analytic integral for straight drive, pure spin, and curved arc.
  - Noise band: with linSigma/yawSigma set, the errored pose deviates from ideal
    (noise is actually applied) but stays within a statistically sane bound.
  - Drift accumulates: with yaw-drift set and zero noise, errored heading minus
    ideal heading grows ~linearly at the drift rate (correct sign and magnitude).
  - Ideal integrator matches ExactPoseTracker / Odometry midpoint convention:
    closed-form midpoint oracle implemented in Python agrees with idealX/Y/H
    returned by the C++ integrator.

These tests exercise the sim-side BenchOtosSensor directly via the
sim_bench_otos_* hooks (no firmware loop involved).
"""

import math
import pytest
from firmware import Sim


def test_bench_otos_starts_at_zero():
    """Fresh sim has bench OTOS accumulators at (0, 0, 0)."""
    with Sim() as s:
        x, y, h = s.get_bench_otos_ideal()
        assert x == pytest.approx(0.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)
        assert h == pytest.approx(0.0, abs=1e-6)


def test_bench_otos_straight_drive_advances_x():
    """Ticking with forward velocity on both wheels advances X (heading=0 → north)."""
    with Sim() as s:
        # Straight drive: both wheels at 100 mm/s, dt = 1000 ms → expect ~100 mm forward.
        # BenchOtosSensor heading=0 convention: forward = +X axis (check BenchOtosSensor.cpp).
        trackwidth = 120.0  # mm, typical robot trackwidth
        vel = 100.0          # mm/s
        dt_ms = 1000         # 1 second

        s.bench_otos_tick(vel, vel, trackwidth, dt_ms)

        x, y, h = s.get_bench_otos_ideal()
        # At heading=0, straight drive should move in +X direction.
        # Expected: x ≈ 100 mm, y ≈ 0 mm, h ≈ 0 rad
        assert x == pytest.approx(100.0, abs=1.0), f"Expected x≈100 mm, got {x}"
        assert y == pytest.approx(0.0, abs=1.0), f"Expected y≈0 mm, got {y}"
        assert h == pytest.approx(0.0, abs=0.01), f"Expected h≈0 rad, got {h}"


def test_bench_otos_zero_velocity_no_motion():
    """Ticking with zero velocity leaves the pose unchanged."""
    with Sim() as s:
        # One tick at zero
        s.bench_otos_tick(0.0, 0.0, 120.0, 1000)
        x, y, h = s.get_bench_otos_ideal()
        assert x == pytest.approx(0.0, abs=1e-6)
        assert y == pytest.approx(0.0, abs=1e-6)
        assert h == pytest.approx(0.0, abs=1e-6)


def test_bench_otos_reset_clears_accumulators():
    """reset() returns both ideal and errored accumulators to zero."""
    with Sim() as s:
        # Advance pose.
        s.bench_otos_tick(100.0, 100.0, 120.0, 1000)

        x, y, h = s.get_bench_otos_ideal()
        assert abs(x) > 1.0, "Pose did not advance before reset"

        # Reset and verify both accumulators are zero.
        s.bench_otos_reset()

        x2, y2, h2 = s.get_bench_otos_ideal()
        assert x2 == pytest.approx(0.0, abs=1e-6), f"After reset ideal x={x2}"
        assert y2 == pytest.approx(0.0, abs=1e-6), f"After reset ideal y={y2}"
        assert h2 == pytest.approx(0.0, abs=1e-6), f"After reset ideal h={h2}"

        ex, ey, eh = s.get_bench_otos_errored()
        assert ex == pytest.approx(0.0, abs=1e-6), f"After reset errored x={ex}"
        assert ey == pytest.approx(0.0, abs=1e-6), f"After reset errored y={ey}"
        assert eh == pytest.approx(0.0, abs=1e-6), f"After reset errored h={eh}"


def test_bench_otos_noiseless_errored_equals_ideal():
    """With noise=0 and drift=0, errored accumulator == ideal accumulator."""
    with Sim() as s:
        # Default noise is 0; just confirm.
        s.bench_otos_set_noise(0.0, 0.0, 0.0)

        s.bench_otos_tick(80.0, 80.0, 120.0, 500)

        xi, yi, hi = s.get_bench_otos_ideal()
        xe, ye, he = s.get_bench_otos_errored()

        assert xi == pytest.approx(xe, abs=1e-4), f"ideal x={xi} errored x={xe}"
        assert yi == pytest.approx(ye, abs=1e-4), f"ideal y={yi} errored y={ye}"
        assert hi == pytest.approx(he, abs=1e-4), f"ideal h={hi} errored h={he}"


def test_bench_otos_multiple_ticks_accumulate():
    """Multiple ticks accumulate correctly (10 × 100 ms at 100 mm/s ≈ 1 s drive)."""
    with Sim() as s:
        s.bench_otos_set_noise(0.0, 0.0, 0.0)
        for _ in range(10):
            s.bench_otos_tick(100.0, 100.0, 120.0, 100)

        x, y, h = s.get_bench_otos_ideal()
        assert x == pytest.approx(100.0, abs=2.0), f"10x100ms at 100mm/s: x≈100, got {x}"
        assert y == pytest.approx(0.0, abs=1.0)
        assert h == pytest.approx(0.0, abs=0.01)


def test_bench_otos_zero_dt_is_noop():
    """dt_ms=0 must not advance the pose (BenchOtosSensor::tick guards dt==0)."""
    with Sim() as s:
        s.bench_otos_tick(100.0, 100.0, 120.0, 0)
        x, y, h = s.get_bench_otos_ideal()
        assert x == pytest.approx(0.0, abs=1e-6), f"dt=0 should not advance x, got {x}"


def test_bench_otos_turn_changes_heading():
    """Differential velocity (right > left) should increase heading (CCW turn)."""
    with Sim() as s:
        s.bench_otos_set_noise(0.0, 0.0, 0.0)
        # Spin in place: left=−50, right=50 → pure left turn
        s.bench_otos_tick(-50.0, 50.0, 120.0, 1000)

        _, _, h = s.get_bench_otos_ideal()
        # 1 second of spin at omega = (50 − (−50)) / 120 = 100/120 ≈ 0.833 rad/s
        # Expected heading ≈ 0.833 rad (≈ 48°), must be clearly positive
        assert h > 0.5, f"Spin left should produce positive heading, got h={h}"


# =============================================================================
# 031-004  Integrator-correctness tests
# =============================================================================
# Conventions reproduced from BenchOtosSensor.cpp tick():
#
#   dC   = (velL + velR) / 2 * dt_s          [arc distance, mm]
#   dTh  = (velR - velL) / trackwidth * dt_s [heading change, rad]
#   hMid = h + dTh / 2                        [midpoint heading]
#   x   += dC * cos(hMid)
#   y   += dC * sin(hMid)
#   h    = wrap_pi(h + dTh)
#
# This is the identical midpoint-arc formula used by Odometry::predict() and
# ExactPoseTracker (see source/hal/mock/MockHAL.h).
# =============================================================================


def _midpoint_oracle(steps):
    """Pure-Python midpoint-arc integrator — mirrors BenchOtosSensor::tick() exactly.

    ``steps`` is a list of (vel_l, vel_r, trackwidth_mm, dt_ms) tuples.
    Returns (x, y, h) after integrating all steps.
    """
    x, y, h = 0.0, 0.0, 0.0
    for vel_l, vel_r, trackwidth, dt_ms in steps:
        if dt_ms == 0 or trackwidth <= 0:
            continue
        dt_s = dt_ms / 1000.0
        dC   = (vel_l + vel_r) / 2.0 * dt_s
        dTh  = (vel_r - vel_l) / trackwidth * dt_s
        hMid = h + dTh / 2.0
        x   += dC * math.cos(hMid)
        y   += dC * math.sin(hMid)
        h   += dTh
    # wrap to [-pi, pi]
    while h >  math.pi: h -= 2 * math.pi
    while h < -math.pi: h += 2 * math.pi
    return x, y, h


# ---------------------------------------------------------------------------
# 031-004 Test 1a: Zero-noise straight-drive oracle
# ---------------------------------------------------------------------------

def test_031_004_zero_noise_straight_oracle():
    """031-004-1a: With noise=0, straight drive matches closed-form oracle exactly.

    Profile: velL=velR=100 mm/s, trackwidth=120 mm, dt=10 ms, 100 ticks (1 s total).
    Expected: x = 100 mm, y = 0 mm, h = 0 rad.

    Both IDEAL and ERRORED must match (zero noise → errored == ideal == analytic).
    Tolerance: ±0.1 mm on position, ±0.001 rad on heading.
    """
    TRACKWIDTH = 120.0
    VEL        = 100.0
    DT_MS      = 10
    N_TICKS    = 100  # 1 second at 100 mm/s → 100 mm

    # Build oracle
    steps = [(VEL, VEL, TRACKWIDTH, DT_MS)] * N_TICKS
    ox, oy, oh = _midpoint_oracle(steps)

    with Sim() as s:
        s.bench_otos_set_noise(0.0, 0.0, 0.0)
        for _ in range(N_TICKS):
            s.bench_otos_tick(VEL, VEL, TRACKWIDTH, DT_MS)

        xi, yi, hi = s.get_bench_otos_ideal()
        xe, ye, he = s.get_bench_otos_errored()

    # Oracle values: x ≈ 100 mm, y = 0, h = 0
    assert ox == pytest.approx(100.0, abs=1e-6), f"Oracle x={ox}"
    assert oy == pytest.approx(0.0,   abs=1e-6), f"Oracle y={oy}"
    assert oh == pytest.approx(0.0,   abs=1e-6), f"Oracle h={oh}"

    # IDEAL matches oracle
    assert xi == pytest.approx(ox, abs=0.1),   f"ideal x={xi} vs oracle {ox}"
    assert yi == pytest.approx(oy, abs=0.1),   f"ideal y={yi} vs oracle {oy}"
    assert hi == pytest.approx(oh, abs=0.001), f"ideal h={hi} vs oracle {oh}"

    # ERRORED == IDEAL (zero noise)
    assert xe == pytest.approx(xi, abs=0.1),   f"errored x={xe} vs ideal {xi}"
    assert ye == pytest.approx(yi, abs=0.1),   f"errored y={ye} vs ideal {yi}"
    assert he == pytest.approx(hi, abs=0.001), f"errored h={he} vs ideal {hi}"


# ---------------------------------------------------------------------------
# 031-004 Test 1b: Zero-noise pure-spin oracle
# ---------------------------------------------------------------------------

def test_031_004_zero_noise_spin_oracle():
    """031-004-1b: With noise=0, pure spin matches closed-form arc oracle.

    Profile: velL=-50 mm/s, velR=+50 mm/s, trackwidth=100 mm, dt=10 ms, 100 ticks.
    omega = (50 - (-50)) / 100 = 1.0 rad/s.
    After 1 s: h = 1.0 rad.  Net translation = 0 (pure spin around centre).

    Tolerance: ±0.1 mm on position, ±0.001 rad on heading.
    """
    TRACKWIDTH = 100.0
    VEL_L      = -50.0
    VEL_R      =  50.0
    DT_MS      = 10
    N_TICKS    = 100

    steps = [(VEL_L, VEL_R, TRACKWIDTH, DT_MS)] * N_TICKS
    ox, oy, oh = _midpoint_oracle(steps)

    with Sim() as s:
        s.bench_otos_set_noise(0.0, 0.0, 0.0)
        for _ in range(N_TICKS):
            s.bench_otos_tick(VEL_L, VEL_R, TRACKWIDTH, DT_MS)

        xi, yi, hi = s.get_bench_otos_ideal()
        xe, ye, he = s.get_bench_otos_errored()

    # Oracle: omega=1.0 rad/s for 1s → h=1.0 rad; x≈0, y≈0 (pure spin)
    # (Small midpoint integration error is negligible for 10ms steps at 1 rad/s.)
    assert oh == pytest.approx(1.0, abs=0.001), f"Oracle spin heading={oh}, expected ~1.0 rad"
    assert abs(ox) < 0.5, f"Pure spin: oracle x should be ~0, got {ox}"
    assert abs(oy) < 0.5, f"Pure spin: oracle y should be ~0, got {oy}"

    # IDEAL matches oracle
    assert xi == pytest.approx(ox, abs=0.1),   f"ideal x={xi} vs oracle {ox}"
    assert yi == pytest.approx(oy, abs=0.1),   f"ideal y={yi} vs oracle {oy}"
    assert hi == pytest.approx(oh, abs=0.001), f"ideal h={hi} vs oracle {oh}"

    # ERRORED == IDEAL (zero noise)
    assert xe == pytest.approx(xi, abs=0.1),   f"errored x={xe} vs ideal {xi}"
    assert ye == pytest.approx(yi, abs=0.1),   f"errored y={ye} vs ideal {yi}"
    assert he == pytest.approx(hi, abs=0.001), f"errored h={he} vs ideal {hi}"


# ---------------------------------------------------------------------------
# 031-004 Test 1c: Zero-noise curved arc (ticket spec case 2)
# ---------------------------------------------------------------------------

def test_031_004_zero_noise_arc_oracle():
    """031-004-1c: With noise=0, curved arc matches analytic arc formula.

    Profile: velL=100 mm/s, velR=0 mm/s, trackwidth=100 mm, dt=10 ms, 100 ticks.
    radius = trackwidth/2 = 50 mm (left wheel traces arc, right wheel stationary).
    omega = (0 - 100) / 100 = -1.0 rad/s (clockwise).
    After 0.1 s (10 ticks at 10ms):
        dh   = -0.1 rad
        x    ≈ sin(|dh|) * r = sin(0.1) * 50
        y    ≈ -(1 - cos(dh)) * r (negative because CW turn)

    Use 10 ticks instead of 100 to stay in small-angle regime where the
    analytic formula is an excellent approximation.

    Tolerance: ±0.1 mm on position, ±0.001 rad on heading.
    """
    TRACKWIDTH = 100.0
    VEL_L      = 100.0
    VEL_R      =   0.0
    DT_MS      = 10
    N_TICKS    = 10   # 0.1 s

    steps = [(VEL_L, VEL_R, TRACKWIDTH, DT_MS)] * N_TICKS
    ox, oy, oh = _midpoint_oracle(steps)

    # Analytic: radius = (v_l + v_r) / (2 * omega) = 100/2 / |-1| = 50
    # omega = (v_r - v_l) / trackwidth = (0 - 100) / 100 = -1.0 rad/s
    # after dt_total = 0.1 s: dh_total = -0.1 rad
    dh_total = -1.0 * (N_TICKS * DT_MS / 1000.0)  # -0.1 rad
    radius   = 50.0
    # Arc formula: robot turns CW by |dh|; at heading=0, +x is forward
    analytic_x = math.sin(abs(dh_total)) * radius       # ≈ 4.998 mm
    analytic_y = -(1.0 - math.cos(dh_total)) * radius   # ≈ -0.25 mm (CW → negative y)

    with Sim() as s:
        s.bench_otos_set_noise(0.0, 0.0, 0.0)
        for _ in range(N_TICKS):
            s.bench_otos_tick(VEL_L, VEL_R, TRACKWIDTH, DT_MS)

        xi, yi, hi = s.get_bench_otos_ideal()
        xe, ye, he = s.get_bench_otos_errored()

    # Oracle matches analytic (small angle ≈ exact)
    assert ox == pytest.approx(analytic_x,  abs=0.1),   f"Oracle x={ox} vs analytic {analytic_x}"
    assert oy == pytest.approx(analytic_y,  abs=0.1),   f"Oracle y={oy} vs analytic {analytic_y}"
    assert oh == pytest.approx(dh_total,    abs=0.001), f"Oracle h={oh} vs analytic {dh_total}"

    # IDEAL matches oracle
    assert xi == pytest.approx(ox, abs=0.1),   f"ideal x={xi} vs oracle {ox}"
    assert yi == pytest.approx(oy, abs=0.1),   f"ideal y={yi} vs oracle {oy}"
    assert hi == pytest.approx(oh, abs=0.001), f"ideal h={hi} vs oracle {oh}"

    # ERRORED == IDEAL (zero noise)
    assert xe == pytest.approx(xi, abs=0.1),   f"errored x={xe} vs ideal {xi}"
    assert ye == pytest.approx(yi, abs=0.1),   f"errored y={ye} vs ideal {yi}"
    assert he == pytest.approx(hi, abs=0.001), f"errored h={he} vs ideal {hi}"


# ---------------------------------------------------------------------------
# 031-004 Test 2: Noise band
# ---------------------------------------------------------------------------

def test_031_004_noise_band():
    """031-004-2: With Gaussian noise set, errored pose deviates from ideal
    but stays within a statistically sane band.

    Profile: velL=velR=100 mm/s (straight), dt=10 ms, 100 ticks (1 s, 100 mm).
    Noise: noiseXY=0.02 (2% of arc), noiseH=0.001 (0.1% of dTh).

    Each tick the arc noise sigma = 0.02 * |dC| = 0.02 * 1.0 = 0.02 mm.
    After 100 independent Gaussian steps accumulated via midpoint:
      std(x_errored) ≈ noiseXY * dC_per_tick * sqrt(N) = 0.02 * 1.0 * 10 = 0.2 mm

    We use a 5-sigma bound to keep the test stable with the fixed LCG seed:
      bound = 5 * 0.02 * sqrt(100) = 1.0 mm

    Asserts:
      1. errored_x != ideal_x  (noise is actually applied — delta > 0.001 mm).
      2. |errored_x - ideal_x| < 1.0 mm  (bounded within 5*expected_std).

    The LCG seed is fixed in HOST_BUILD (see BenchOtosSensor.h _lcgState = 12345u)
    so the result is deterministic and always the same value for this profile.
    """
    TRACKWIDTH  = 120.0
    VEL         = 100.0
    DT_MS       = 10
    N_TICKS     = 100
    NOISE_XY    = 0.02
    NOISE_H     = 0.001

    # Expected std(x) per-step ≈ noiseXY * dC = 0.02 * 1.0 mm
    # After N steps (random walk): std(x_total) ≈ noiseXY * dC * sqrt(N) = 0.02 * 10 = 0.2 mm
    # 5-sigma bound = 1.0 mm
    dC_per_tick = VEL * (DT_MS / 1000.0)            # 1.0 mm
    expected_std = NOISE_XY * dC_per_tick * math.sqrt(N_TICKS)  # 0.2 mm
    K = 5.0
    bound = K * expected_std                         # 1.0 mm

    with Sim() as s:
        s.bench_otos_set_noise(NOISE_XY, NOISE_H, 0.0)
        for _ in range(N_TICKS):
            s.bench_otos_tick(VEL, VEL, TRACKWIDTH, DT_MS)

        xi, yi, hi = s.get_bench_otos_ideal()
        xe, ye, he = s.get_bench_otos_errored()

    delta_x = abs(xe - xi)
    # Noise is applied: errored must differ from ideal by at least a tiny amount.
    assert delta_x > 1e-3, (
        f"Expected noise to be applied (|errored_x - ideal_x| > 0.001 mm), "
        f"got delta_x={delta_x:.6f} mm"
    )
    # Noise is bounded: within K * expected_std.
    assert delta_x < bound, (
        f"Expected |errored_x - ideal_x| < {bound:.3f} mm ({K}-sigma bound), "
        f"got delta_x={delta_x:.4f} mm (expected_std={expected_std:.3f} mm)"
    )


# ---------------------------------------------------------------------------
# 031-004 Test 3: Drift accumulates
# ---------------------------------------------------------------------------

def test_031_004_drift_accumulates():
    """031-004-3: With yaw-drift set and zero noise, errored heading minus ideal
    grows ~linearly with time at the drift rate (correct sign and magnitude).

    Drift = +0.1 rad/s (positive → CCW drift).
    After T seconds: errored_h - ideal_h ≈ drift * T.

    Profile: pure straight drive (no actual heading change, so ideal_h stays near 0).
    Test at 3 checkpoints: 0.5 s, 1.0 s, 2.0 s.

    Tolerance: ±20% relative error on accumulated drift at each checkpoint.
    """
    DRIFT_RADS  = 0.1   # rad/s positive drift
    TRACKWIDTH  = 120.0
    VEL         = 50.0  # straight drive to accumulate some distance
    DT_MS       = 10
    TICKS_PER_S = 1000 // DT_MS  # 100 ticks per second

    def run_and_check(total_ticks, expected_drift):
        with Sim() as s:
            s.bench_otos_set_noise(0.0, 0.0, DRIFT_RADS)
            for _ in range(total_ticks):
                s.bench_otos_tick(VEL, VEL, TRACKWIDTH, DT_MS)
            xi, _, hi = s.get_bench_otos_ideal()
            xe, _, he = s.get_bench_otos_errored()
        heading_error = he - hi
        return heading_error

    # Checkpoint 1: 0.5 s → expected drift = 0.05 rad
    err_05 = run_and_check(TICKS_PER_S // 2, DRIFT_RADS * 0.5)
    assert err_05 > 0, f"Drift should be positive (CCW), got err_05={err_05:.4f}"
    assert err_05 == pytest.approx(DRIFT_RADS * 0.5, rel=0.20), (
        f"0.5s drift: expected ~{DRIFT_RADS*0.5:.3f} rad, got {err_05:.4f} rad"
    )

    # Checkpoint 2: 1.0 s → expected drift = 0.1 rad
    err_10 = run_and_check(TICKS_PER_S, DRIFT_RADS * 1.0)
    assert err_10 > 0, f"Drift should be positive (CCW), got err_10={err_10:.4f}"
    assert err_10 == pytest.approx(DRIFT_RADS * 1.0, rel=0.20), (
        f"1.0s drift: expected ~{DRIFT_RADS:.3f} rad, got {err_10:.4f} rad"
    )

    # Checkpoint 3: 2.0 s → expected drift = 0.2 rad
    err_20 = run_and_check(TICKS_PER_S * 2, DRIFT_RADS * 2.0)
    assert err_20 > 0, f"Drift should be positive (CCW), got err_20={err_20:.4f}"
    assert err_20 == pytest.approx(DRIFT_RADS * 2.0, rel=0.20), (
        f"2.0s drift: expected ~{DRIFT_RADS*2:.3f} rad, got {err_20:.4f} rad"
    )

    # Linearity check: drift at 2s should be approximately 2x drift at 1s.
    assert err_20 == pytest.approx(2.0 * err_10, rel=0.10), (
        f"Drift not linear: 2s={err_20:.4f}, 2*1s={2*err_10:.4f}"
    )


# ---------------------------------------------------------------------------
# 031-004 Test 4: Ideal integrator matches ExactPoseTracker / midpoint convention
# ---------------------------------------------------------------------------

def test_031_004_ideal_matches_midpoint_oracle():
    """031-004-4: BenchOtosSensor ideal accumulator uses the same midpoint-arc
    formula as Odometry::predict and ExactPoseTracker.

    Cross-check: Python _midpoint_oracle (identical formula) must agree with
    the C++ idealX/Y/H outputs from BenchOtosSensor::tick() to within floating-
    point precision for three distinct profiles.

    This verifies that the bench pose is consistent with the encoder-odometry
    the EKF also sees — a requirement for the bench sensor to be a useful
    stand-in during stand-based sessions.

    Tolerance: ±0.002 mm on position, ±1e-5 rad on heading.
    The C++ integrator uses float32 while Python uses float64, so accumulated
    rounding for 200 ticks over ~240 mm is ~1e-4 mm; 0.002 mm gives 20x margin.
    """
    TRACKWIDTH = 120.0

    profiles = [
        # (name, vel_l, vel_r, dt_ms, n_ticks)
        ("straight_fast",  200.0, 200.0, 24, 50),   # 2.4 s straight at 200 mm/s
        ("spin_ccw",       -30.0,  30.0, 10, 200),  # 200 ticks of 60 mm/s spin
        ("arc_curve",      100.0,  60.0, 20, 100),  # 2 s curved arc
    ]

    for name, vel_l, vel_r, dt_ms, n_ticks in profiles:
        steps = [(vel_l, vel_r, TRACKWIDTH, dt_ms)] * n_ticks
        ox, oy, oh = _midpoint_oracle(steps)

        with Sim() as s:
            s.bench_otos_set_noise(0.0, 0.0, 0.0)
            for _ in range(n_ticks):
                s.bench_otos_tick(vel_l, vel_r, TRACKWIDTH, dt_ms)

            xi, yi, hi = s.get_bench_otos_ideal()

        assert xi == pytest.approx(ox, abs=0.002), (
            f"[{name}] ideal x={xi:.6f} vs oracle {ox:.6f}"
        )
        assert yi == pytest.approx(oy, abs=0.002), (
            f"[{name}] ideal y={yi:.6f} vs oracle {oy:.6f}"
        )
        assert hi == pytest.approx(oh, abs=1e-5), (
            f"[{name}] ideal h={hi:.8f} vs oracle {oh:.8f}"
        )
