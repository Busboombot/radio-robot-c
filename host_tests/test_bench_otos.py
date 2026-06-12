"""
test_bench_otos.py — 031-002 minimal wiring test for BenchOtosSensor sim hooks.

Tests:
  - BenchOtosSensor starts at zero pose.
  - Ticking with a straight-ahead commanded velocity advances the ideal X
    accumulator (pose moves forward).
  - After reset() both accumulators return to zero.
  - Ticking with zero velocity (bench mode off) leaves pose unchanged.
  - With no noise set, errored == ideal (noiseless at default parameters).

These tests exercise the sim-side BenchOtosSensor directly via the
sim_bench_otos_* hooks.  Full integrator-correctness tests are in 031-004.
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
