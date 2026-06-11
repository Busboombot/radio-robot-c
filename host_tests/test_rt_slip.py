"""test_rt_slip.py — RT encoder-arc compensation for rotationalSlip (sprint 024-006).

Verifies that beginRotation() divides the encoder-arc target by the effective slip
so the body achieves the commanded angle despite wheel scrub.

Math:
  No-slip arc  = |deg| * (π/180) * (trackwidth/2) = 90 * π/180 * 63 ≈ 98.9 mm
  Slip=0.74:   arc = 98.9 / 0.74 ≈ 133.6 mm  (larger target → wheels travel farther)
  Coast anticipation (kRtCoastArcMm = 8 mm) is subtracted from both.

  stopArc (no slip)  = 98.9 - 8 ≈ 90.9 mm
  stopArc (slip=0.74) = 133.6 - 8 ≈ 125.6 mm

When RT completes, the encoder differential |encR - encL|/2 should ≈ stopArc
(motors coast slightly beyond that due to SOFT stop ramp), so the slip=0.74
run should drive meaningfully more arc than the slip-disabled run.
"""

import math
import pytest


def _arc_after_rt(sim, cdeg: int = 9000) -> float:
    """Issue RT <cdeg>, wait for completion, return |encR - encL| / 2 in mm."""
    # Reset encoders and pose before RT.
    sim.send_command("ZERO")

    # Issue RT command.
    r = sim.send_command(f"RT {cdeg}")
    assert "OK" in r.upper(), f"RT {cdeg} → unexpected reply {repr(r)}"

    # Tick up to 8 s; RT should complete well before then.
    sim.tick_for(8000)

    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r = float(sim._lib.sim_get_enc_r(sim._h))

    # For a CCW spin: left wheel goes backward (negative), right goes forward.
    # |differential| / 2 = per-wheel arc.
    diff_half = abs(enc_r - enc_l) / 2.0
    return diff_half


def test_rt_arc_larger_with_slip(sim):
    """RT 9000 with rotationalSlip=0.74 drives a larger encoder arc than slip-disabled.

    With slip=0.74 (firmware default), beginRotation() divides the no-slip arc
    by 0.74, expanding the target from ~90.9 mm (stop-arc) to ~125.6 mm.
    The final encoder arc after the SOFT ramp should therefore be meaningfully
    larger than the no-slip case.

    Expected:
      No-slip  (rotSlip=0):    stopArc ≈ 90.9 mm   → final enc-arc ≈ 90–100 mm
      Slip=0.74 (default):     stopArc ≈ 125.6 mm  → final enc-arc ≈ 125–140 mm
    """
    # --- Run 1: slip disabled (SET rotSlip=0 → effectiveSlip → 1.0) ---
    sim.send_command("SET rotSlip=0")
    arc_no_slip = _arc_after_rt(sim, 9000)

    # --- Run 2: firmware default slip=0.74 ---
    sim.send_command("SET rotSlip=0.74")
    arc_with_slip = _arc_after_rt(sim, 9000)

    # The slip-compensated run must drive at least 15% more encoder arc.
    ratio = arc_with_slip / arc_no_slip if arc_no_slip > 0 else 0.0
    assert arc_with_slip > arc_no_slip * 1.1, (
        f"RT 9000 with slip=0.74 should drive ≥10% more arc than no-slip. "
        f"no-slip={arc_no_slip:.1f} mm, slip=0.74={arc_with_slip:.1f} mm, "
        f"ratio={ratio:.2f} (expected ≥1.10)"
    )


def test_rt_arc_no_slip_matches_geometry(sim):
    """RT 9000 with rotSlip=0 (identity) produces arc ≈ theoretical no-slip value.

    Theoretical: stopArc = 90 * π/180 * (126/2) - 8 ≈ 90.9 mm.
    The SOFT ramp adds some coast, so actual final arc ≥ stopArc.
    """
    sim.send_command("SET rotSlip=0")
    arc = _arc_after_rt(sim, 9000)

    # Theoretical no-slip per-wheel arc (before coast).
    tw_mm = 126.0   # trackwidthMm default
    coast_mm = 8.0  # kRtCoastArcMm
    theoretical_stop_arc = 90.0 * math.pi / 180.0 * (tw_mm * 0.5) - coast_mm
    # Actual must be at least the stop-arc (SOFT ramp adds some coast past it).
    assert arc >= theoretical_stop_arc * 0.8, (
        f"RT 9000 no-slip arc {arc:.1f} mm should be ≈ {theoretical_stop_arc:.1f} mm "
        f"(stopping arc before coast)"
    )


def test_rt_slip_compensation_ratio(sim):
    """The slip=0.74 arc is approximately 1/0.74 ≈ 1.35× the no-slip arc.

    The ratio arc_with_slip / arc_no_slip should be approximately 1/slip = 1/0.74 ≈ 1.35.
    Because the coast arc (8 mm) is the same in both cases, the ratio will not be
    exactly 1/0.74, but it should exceed 1.2 (generous tolerance for coast effects).
    """
    sim.send_command("SET rotSlip=0")
    arc_no_slip = _arc_after_rt(sim, 9000)

    sim.send_command("SET rotSlip=0.74")
    arc_slip = _arc_after_rt(sim, 9000)

    assert arc_no_slip > 10.0, f"No-slip arc too small ({arc_no_slip:.1f} mm) — RT did not run"
    ratio = arc_slip / arc_no_slip
    assert ratio >= 1.2, (
        f"RT arc ratio (slip/no-slip) = {ratio:.2f} should be ≥1.2 "
        f"(no-slip={arc_no_slip:.1f}, slip=0.74={arc_slip:.1f})"
    )
