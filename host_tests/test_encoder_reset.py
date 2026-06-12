"""
test_encoder_reset.py — N1 regression tests for atomic encoder reset (030-001).

These tests catch two failure modes that existed before the atomic resetEncoders()
fix:

1. D-command backward pose teleport (test_encoder_reset_pose_continuity):
   Every D command used to teleport the fused pose backward by the prior segment's
   travel because Odometry::_prevEncL/_prevEncR were not re-baselined.  On the
   first tick after beginDistance(), Odometry::predict() computed dL = 0 - _prevEncL
   (large negative) and fed that into the EKF.  With OTOS OFF this corruption was
   permanent.  After the fix, pose delta after D completes must be < 5 mm.

2. ZERO enc frozen-encoder window (test_zero_enc_no_frozen_window):
   ZERO enc used to reset hardware accumulators but leave state.inputs.encLMm/R
   stale, so the outlier filter froze encoder reads until the fresh accumulator
   climbed back to the stale value.  After the fix, the outlier filter baseline is
   zeroed atomically with the hardware, so the first tick after ZERO enc must
   accept a clean (non-negative, near-zero delta) encoder read.

3. EKF rejection count after D with fusion ON (test_ekf_rej_zero_after_d):
   With OTOS fusion enabled, the pre-fix large negative encoder delta triggered
   Mahalanobis gate rejections for ~10 ticks after each D command (ekf_rej climbed).
   After the fix, a clean D drive must produce zero new rejections.
"""
import ctypes

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get_ekf_rej(sim) -> int:
    """Return the cumulative EKF gate rejection count from the sim."""
    return int(sim._lib.sim_get_ekf_rej_count(sim._h))


def _get_pose(sim):
    """Return (x_mm, y_mm) from the current fused pose."""
    x = float(sim._lib.sim_get_pose_x(sim._h))
    y = float(sim._lib.sim_get_pose_y(sim._h))
    return x, y


# ---------------------------------------------------------------------------
# Test 1: D-then-G pose continuity (OTOS fusion OFF)
#
# Acceptance criterion: pose delta immediately after D completes is < 5 mm.
# A backward teleport of ~prior-segment-length would be hundreds of mm.
# ---------------------------------------------------------------------------

def test_encoder_reset_pose_continuity(sim):
    """D-then-G with OTOS fusion OFF: no backward pose teleport after D.

    Drive 300 mm via D command.  After it completes, snapshot pose.  Tick one
    more step and re-read pose.  The delta must be < 5 mm (not ~300 mm backward).

    OTOS fusion is intentionally OFF so that any corruption from the encoder
    delta is permanent (no EKF correction to mask it).
    """
    # Extend watchdog so the D command isn't killed early.
    sim.send_command("SET sTimeout=60000")

    # Enable ENC and POSE fields in TLM so we can diagnose on failure.
    sim.send_command("SET fields=enc,pose")

    # OTOS fusion off (default — MockOtosSensor not initialised, fuseOtos=False).
    # No explicit sim.set_otos_fusion call needed; the fixture leaves it off.

    # Drive 300 mm at 200 mm/s.
    r = sim.send_command("D 200 200 300")
    assert "OK" in r.upper(), f"Expected OK from D command, got {repr(r)}"

    # Tick until D completes (up to 10 s simulated).
    sim.tick_for(10000)

    # Verify D completed via EVT done.
    evts = sim.get_async_evts()
    assert "EVT done D" in evts, f"D command did not complete: {repr(evts)}"

    # Snapshot pose immediately after D completes.
    x0, y0 = _get_pose(sim)

    # Tick one more step — this is the tick where the backward teleport used to
    # appear (the first predict() after beginDistance zeroed the hardware).
    sim.tick_for(24)

    x1, y1 = _get_pose(sim)
    delta = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5

    assert delta < 5.0, (
        f"Pose jumped {delta:.1f} mm after D command — backward teleport not fixed.\n"
        f"  Pose before extra tick: ({x0:.1f}, {y0:.1f})\n"
        f"  Pose after  extra tick: ({x1:.1f}, {y1:.1f})"
    )


# ---------------------------------------------------------------------------
# Test 2: EKF rejection count after D with OTOS fusion ON
#
# Acceptance criterion: no new EKF rejections during and after a D drive
# with OTOS enabled.
# ---------------------------------------------------------------------------

def test_ekf_rej_zero_after_d(sim):
    """D command with OTOS fusion ON produces zero EKF gate rejections.

    Before the fix, the large negative encoder delta injected by the first
    tick after beginDistance() was far outside the Mahalanobis gate, triggering
    ~10 consecutive rejections.  After the fix, the delta is zero so no
    rejection occurs.
    """
    sim.send_command("SET sTimeout=60000")

    # Enable OTOS fusion + sim model so correctEKF() runs on each tick.
    sim._lib.sim_enable_otos_model(sim._h)
    sim._lib.sim_set_otos_fusion(sim._h, ctypes.c_int(1))

    # Snapshot rejection count before the drive.
    rej_before = _get_ekf_rej(sim)

    # Drive 200 mm.
    r = sim.send_command("D 200 200 200")
    assert "OK" in r.upper(), f"Expected OK from D, got {repr(r)}"

    # Tick for a few ticks — enough to see any post-D spurious rejection.
    # 5 ticks × 24 ms = 120 ms; the old code rejected for ~10 ticks (~240 ms).
    sim.tick_for(10 * 24)

    rej_after = _get_ekf_rej(sim)
    new_rej = rej_after - rej_before

    assert new_rej == 0, (
        f"EKF accumulated {new_rej} rejections in the 10 ticks after D — "
        f"spurious negative encoder delta not fixed.\n"
        f"  Rejections before: {rej_before}, after: {rej_after}"
    )


# ---------------------------------------------------------------------------
# Test 3: ZERO enc no frozen-encoder window
#
# Acceptance criterion: the tick immediately after ZERO enc (with nonzero
# prior travel) must NOT cause the outlier filter to reject the encoder read
# (i.e. encLMm/R must update to ~0, not stay at the old stale value).
# ---------------------------------------------------------------------------

def test_zero_enc_no_frozen_window(sim):
    """ZERO enc with nonzero prior travel: no frozen-encoder window on next tick.

    Sequence:
      1. Drive forward a little so encoders accumulate (encLMm/R > 0).
      2. Stop the robot.
      3. Send ZERO enc.
      4. Tick once.
      5. Assert encLMm and encRMm are near 0 (outlier filter accepted the reset,
         not frozen at the pre-reset value).

    Before the fix, ZERO enc reset the hardware but left state.inputs.encLMm/R
    stale, so the outlier filter saw delta ≈ −prior_travel on the next read and
    rejected it, leaving encLMm/R frozen at the stale value until the hardware
    accumulator climbed back.
    """
    sim.send_command("SET sTimeout=60000")

    # Drive for 500 ms to accumulate encoder travel.
    r = sim.send_command("VW 150 0")
    assert "OK" in r.upper(), f"VW failed: {repr(r)}"
    sim.tick_for(500)

    # Verify encoders moved.
    enc_l_before = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r_before = float(sim._lib.sim_get_enc_r(sim._h))
    assert enc_l_before > 10.0 and enc_r_before > 10.0, (
        f"Expected encoder travel > 10 mm before ZERO enc, "
        f"got enc_l={enc_l_before:.2f}, enc_r={enc_r_before:.2f}"
    )

    # Stop the robot first so VW watchdog doesn't interfere.
    sim.send_command("X")
    sim.tick_for(24)

    # Issue ZERO enc.
    r = sim.send_command("ZERO enc")
    assert "OK" in r.upper(), f"ZERO enc failed: {repr(r)}"

    # Tick once — this is where the frozen-encoder window used to appear.
    sim.tick_for(24)

    enc_l_after = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r_after = float(sim._lib.sim_get_enc_r(sim._h))

    # After atomic reset, the outlier filter baseline is 0, so the hardware
    # read of ~0 is accepted and enc{L,R}Mm should be near 0.
    # The hardware accumulator starts at 0 after reset; one tick of motion
    # at speed=0 (stopped) should give near-zero delta.
    # Tolerance: 5 mm (generous for one tick of stopped motion).
    assert abs(enc_l_after) < 5.0, (
        f"enc_l={enc_l_after:.2f} after ZERO enc — outlier filter may be frozen.\n"
        f"  Was {enc_l_before:.2f} before ZERO enc."
    )
    assert abs(enc_r_after) < 5.0, (
        f"enc_r={enc_r_after:.2f} after ZERO enc — outlier filter may be frozen.\n"
        f"  Was {enc_r_before:.2f} before ZERO enc."
    )
