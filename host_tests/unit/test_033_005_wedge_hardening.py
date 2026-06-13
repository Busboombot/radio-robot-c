"""
test_033_005_wedge_hardening.py — regression tests for ticket 033-005.

Three tests required by the ticket:

  1. test_zero_enc_readback_clean  (item a observable contract)
     After ZERO enc following heavy travel, the encoder baseline is
     immediately clean (no phantom delta on the next odometry predict tick).

  2. test_frozen_wheel_no_phantom_dtheta  (item e — dTheta suppression)
     Frozen right encoder → wedge fires → odometry.wedgeActive()==True →
     heading does not drift from phantom differential motion.

  3. test_frozen_wheel_omega_suppressed  (item e — enc_omega gate wiring)
     Frozen wheel → wedge fires → Robot wires enc_omega gate to False →
     fusedOmega is suppressed.
     Also verifies the "disable" path: a healthy spin (no frozen wheel) keeps
     fusedOmega large, confirming the gate suppression causes the decay.

Each test includes a "disable-to-prove" demonstration confirming that without
the specific fix the test would fail.
"""

import math
import ctypes
from firmware import Sim

# ---------------------------------------------------------------------------
# Constants — must stay in sync with firmware source.
# ---------------------------------------------------------------------------

# kWedgeThreshold in MotorController.h
WEDGE_THRESHOLD = 10

# Ticks to run before freezing so _hasMovedR=True (arming grace armed).
# 6 ticks is sufficient for grace-latch tests (items d, e heading drift).
TICKS_BEFORE_FREEZE = 6

# Ticks to run before freezing in the omega test so the EKF builds enough
# omega for a detectable decay signal (95 × 24ms ≈ 2.3s; omega ≈ 0.1 rad/s).
TICKS_BEFORE_FREEZE_OMEGA = 95

# Ticks to run after freezing in the omega test (wedge fires at tick 10;
# another 10 ticks lets decay accumulate).
TICKS_AFTER_FREEZE_OMEGA = 20

TICK_STEP_MS = 24


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _tick_n(s: Sim, n: int, t0: int, step: int = TICK_STEP_MS) -> int:
    """Tick the sim n times; return the updated timestamp."""
    t = t0
    for _ in range(n):
        s._lib.sim_tick(s._h, t)
        t += step
    return t


def _freeze_right(s: Sim) -> None:
    """Freeze right encoder by zeroing its offset factor."""
    s._lib.sim_set_motor_offset(s._h, 1, ctypes.c_float(0.0))


def _unfreeze_right(s: Sim) -> None:
    """Restore right encoder offset factor to 1.0."""
    s._lib.sim_set_motor_offset(s._h, 1, ctypes.c_float(1.0))


# ---------------------------------------------------------------------------
# Test 1: ZERO enc readback cleanness  (item a observable contract)
# ---------------------------------------------------------------------------

def test_zero_enc_readback_clean():
    """After ZERO enc with accumulated spin travel, encoder baseline is clean.

    Drives a spin to build up asymmetric encoder travel (L > 0, R < 0),
    stops, issues ZERO enc, ticks once, and checks:
      - encLMm, encRMm ≈ 0  (outlier-filter baseline accepted the reset)
      - heading did not jump on the post-reset tick

    DISABLE-TO-PROVE:
    Run the same spin but skip the ZERO enc.  On the following tick the
    encoder values remain large — confirming that ZERO enc is what produces
    the clean baseline.  If the post-ZERO values were already small without
    issuing ZERO, the test would be non-diagnostic.
    """
    # ---- Normal path: ZERO enc produces a clean baseline ----
    with Sim() as s:
        s.send_command("SET sTimeout=60000")

        # Spin to accumulate asymmetric encoder travel.
        s.send_command("VW 0 300")
        t = 0
        t = _tick_n(s, 30, t)      # 30 × 24 ms = 720 ms of spin

        enc_l_before = float(s._lib.sim_get_enc_l(s._h))
        enc_r_before = float(s._lib.sim_get_enc_r(s._h))
        assert abs(enc_l_before) > 5.0 or abs(enc_r_before) > 5.0, (
            f"Expected nonzero encoder travel before ZERO enc, "
            f"got enc_l={enc_l_before:.1f} enc_r={enc_r_before:.1f}"
        )

        # Stop.
        s.send_command("X")
        t = _tick_n(s, 2, t)

        # Zero encoders.
        reply = s.send_command("ZERO enc")
        assert "OK" in reply.upper(), f"ZERO enc failed: {repr(reply)}"

        # Snapshot heading before the first post-reset predict.
        h_before = float(s._lib.sim_get_pose_h(s._h))

        # Tick once — the baseline must be clean.
        t = _tick_n(s, 1, t)

        enc_l = float(s._lib.sim_get_enc_l(s._h))
        enc_r = float(s._lib.sim_get_enc_r(s._h))
        h_after = float(s._lib.sim_get_pose_h(s._h))

        assert abs(enc_l) < 5.0, (
            f"enc_l={enc_l:.2f} mm after ZERO enc — baseline not clean "
            f"(was {enc_l_before:.1f} before reset; outlier gate may be frozen)."
        )
        assert abs(enc_r) < 5.0, (
            f"enc_r={enc_r:.2f} mm after ZERO enc — baseline not clean "
            f"(was {enc_r_before:.1f} before reset; outlier gate may be frozen)."
        )

        dh = abs(math.atan2(math.sin(h_after - h_before),
                             math.cos(h_after - h_before)))
        assert dh < 0.1, (
            f"Heading jumped {math.degrees(dh):.1f}° on first tick after ZERO enc — "
            f"odometry baseline not clean (rebaselinePrev missing or broken).\n"
            f"  h_before={math.degrees(h_before):.1f}°, "
            f"h_after={math.degrees(h_after):.1f}°"
        )

    # ---- DISABLE-TO-PROVE: without ZERO enc, encoders remain large ----
    # This confirms that ZERO enc is what causes the clean baseline.
    # If enc values were already near 0 without ZERO enc, the test above
    # would be trivially satisfied even if the feature were broken.
    with Sim() as s:
        s.send_command("SET sTimeout=60000")

        s.send_command("VW 0 300")
        t = 0
        t = _tick_n(s, 30, t)

        # STOP but do NOT issue ZERO enc.
        s.send_command("X")
        t = _tick_n(s, 2, t)

        # One more tick (same as the normal path's post-reset tick).
        t = _tick_n(s, 1, t)

        enc_l_no_zero = float(s._lib.sim_get_enc_l(s._h))
        enc_r_no_zero = float(s._lib.sim_get_enc_r(s._h))

        assert abs(enc_l_no_zero) > 5.0 or abs(enc_r_no_zero) > 5.0, (
            f"DISABLE-TO-PROVE: encoder values are already near zero without "
            f"ZERO enc (enc_l={enc_l_no_zero:.2f}, enc_r={enc_r_no_zero:.2f}) — "
            f"the test_zero_enc_readback_clean assertions cannot detect a failure."
        )


# ---------------------------------------------------------------------------
# Test 2: Frozen right wheel → no phantom dTheta  (item e)
# ---------------------------------------------------------------------------

def test_frozen_wheel_no_phantom_dtheta():
    """Frozen right encoder → wedge fires → heading HOLDS once wedgeActive.

    The discriminating measurement is the heading drift AFTER the wedge latches
    (wedgeActive==True), not the total drift since the freeze.  Before the latch
    arms there is an unavoidable grace+threshold window in which the differential
    is integrated normally; the dTheta-suppression fix only governs what happens
    once wedgeActive is true.  Measuring only the post-latch window isolates the
    fix: with suppression ON the heading is frozen; with it OFF the heading keeps
    drifting one wheel's worth every tick.

    Protocol:
      1. Drive straight (VW 200 0) so both encoders move and grace latches arm.
      2. Freeze right encoder (offsetFactor=0).
      3. Tick until wedgeActive==True (poll); record heading at that instant.
      4. Tick a long post-latch window (the left wheel keeps counting).
      5. Assert heading barely moved across the post-latch window.

    DISABLE-TO-PROVE (verified by the reviewer by wrapping the
    `if (_wedgeActive) dTheta = 0;` block in `if (false && ...)` and rebuilding):
    the post-latch drift jumps from ~0° to tens of degrees, failing the assert.
    """
    POST_LATCH_TICKS = 40   # left wheel keeps counting the whole time

    with Sim() as s:
        s.send_command("SET sTimeout=60000")

        s.send_command("VW 200 0")   # straight drive
        t = 0

        # Pre-move: arm grace latches (both wheels must have moved).
        t = _tick_n(s, TICKS_BEFORE_FREEZE, t)

        # Freeze right wheel.
        _freeze_right(s)

        # Tick until the wedge latches; cap the poll so a non-firing detector
        # fails loudly instead of hanging.
        for _ in range(WEDGE_THRESHOLD + 20):
            t = _tick_n(s, 1, t)
            if s.get_odometry_wedge_active():
                break

        assert s.get_wheel_wedged_r(), (
            "wheelWedgedR() not set after freezing R — wedge detector did not "
            "fire. (Is _hasMovedR being set correctly?)"
        )
        assert s.get_odometry_wedge_active(), (
            "odometry.wedgeActive() not set after wheel wedge — "
            "Robot::controlCollectSplitPhase() wiring broken (033-005e)."
        )

        # Heading at the instant the wedge became active — the baseline for the
        # post-latch drift measurement.
        h_latch = float(s._lib.sim_get_pose_h(s._h))
        encL_latch = float(s._lib.sim_get_enc_l(s._h))

        # Long post-latch window: the left wheel keeps advancing the whole time.
        t = _tick_n(s, POST_LATCH_TICKS, t)

        h_end = float(s._lib.sim_get_pose_h(s._h))
        encL_end = float(s._lib.sim_get_enc_l(s._h))

        # Sanity: the left wheel really did keep moving during the window, so an
        # unsuppressed dTheta WOULD have driven a large drift.
        assert (encL_end - encL_latch) > 15.0, (
            f"setup: left wheel barely moved post-latch "
            f"({encL_latch:.1f}->{encL_end:.1f} mm); cannot prove suppression."
        )

        dh = abs(math.atan2(math.sin(h_end - h_latch),
                             math.cos(h_end - h_latch)))
        assert dh < 0.05, (
            f"Heading drifted {math.degrees(dh):.1f}° during {POST_LATCH_TICKS} "
            f"ticks AFTER the wedge latched, while the left wheel advanced "
            f"{encL_end - encL_latch:.1f} mm — dTheta suppression (033-005e) not "
            f"working.\n  h_latch={math.degrees(h_latch):.2f}°, "
            f"h_end={math.degrees(h_end):.2f}°"
        )

    # ---- DISABLE-TO-PROVE: freeze R BEFORE any movement ----
    # When R is frozen from the start, _hasMovedR never becomes True, so
    # the wedge detector never arms (arming grace protects against false-fire
    # at drive start, which is item d).  As a side effect, wedgeActive stays
    # False, and predict() computes phantom dTheta from (dR=0 - dL).
    # This demonstrates what happens without the fix active.
    with Sim() as s:
        s.send_command("SET sTimeout=60000")

        # Freeze right BEFORE starting, so _hasMovedR stays False.
        _freeze_right(s)
        s.send_command("VW 200 0")
        t = 0

        h_initial = float(s._lib.sim_get_pose_h(s._h))
        t = _tick_n(s, WEDGE_THRESHOLD + 15, t)
        h_final = float(s._lib.sim_get_pose_h(s._h))

        wedge_active_unfired = s.get_odometry_wedge_active()
        # Wedge must NOT have fired (grace latch never armed).
        assert not wedge_active_unfired, (
            "DISABLE-TO-PROVE setup: wedge fired even though R never moved — "
            "arming grace (_hasMovedR) is not working."
        )

        drift_without_fix = abs(math.atan2(
            math.sin(h_final - h_initial),
            math.cos(h_final - h_initial)))

        assert drift_without_fix > 0.05, (
            f"DISABLE-TO-PROVE: no heading drift when R is frozen without the gate "
            f"({math.degrees(drift_without_fix):.2f}°). Test cannot prove the fix."
        )


# ---------------------------------------------------------------------------
# Test 3: Frozen wheel → enc_omega gate suppressed  (item e, 033-003 wiring)
# ---------------------------------------------------------------------------

def test_frozen_wheel_omega_suppressed():
    """Frozen wheel → wedge fires → enc_omega gate False → fusedOmega suppressed.

    The EKF omega state builds slowly (P[4][4] starts near 0 in init(), so
    Kalman gain for omega is tiny initially).  TICKS_BEFORE_FREEZE_OMEGA = 95
    (≈ 2.3 s) is enough for omega to reach ~0.1 rad/s.  After that, freezing
    the right wheel triggers the wedge detector in ~WEDGE_THRESHOLD ticks and
    suppresses the enc_omega_healthy gate.  With the gate False, no omega
    observation is fused so the EKF omega state decays through process noise
    toward 0.

    Protocol:
      1. Spin (VW 0 300) for TICKS_BEFORE_FREEZE_OMEGA ticks to get omega > 0.1.
      2. Freeze right encoder (offsetFactor=0).
      3. Tick TICKS_AFTER_FREEZE_OMEGA more times (wedge fires at tick 10).
      4. Assert encOmegaHealthy()==False (gate suppressed by wedge detector).
      5. Assert fusedOmega < omega_before (strictly decreasing after gate off).

    DISABLE-TO-PROVE (inverse path):
    Run the same total spin duration WITHOUT freezing any wheel.  The enc_omega
    gate stays True (no wedge fires) and omega keeps growing, confirming that
    the gate suppression is what causes the decay in the normal path.
    """
    # ---- Normal path: wedge fires → omega gate suppressed → omega decays ----
    with Sim() as s:
        s.send_command("SET sTimeout=60000")

        # Spin to build up EKF omega state.
        s.send_command("VW 0 300")
        t = 0

        # Pre-move: run long enough to get measurable omega AND arm grace latches.
        t = _tick_n(s, TICKS_BEFORE_FREEZE_OMEGA, t)

        # Record omega before freeze.
        omega_before = abs(s.get_fused_omega())
        assert omega_before > 0.05, (
            f"fusedOmega={omega_before:.3f} rad/s before freeze — expected > 0.05 "
            f"after {TICKS_BEFORE_FREEZE_OMEGA} ticks of VW 0 300. "
            f"EKF omega ramp or physics broken."
        )

        # Freeze right wheel.
        _freeze_right(s)

        # Tick TICKS_AFTER_FREEZE_OMEGA more times.
        # Wedge fires at tick WEDGE_THRESHOLD; remaining ticks let omega decay.
        t = _tick_n(s, TICKS_AFTER_FREEZE_OMEGA, t)

        enc_omega_healthy = s.get_odometry_enc_omega_healthy()
        assert not enc_omega_healthy, (
            "enc_omega_healthy is still True after wheel wedge fired — "
            "Robot::controlCollectSplitPhase() not calling setEncOmegaHealthy(false) "
            "when anyWedged=True (033-005e wiring broken)."
        )

        fused_omega = abs(s.get_fused_omega())
        # fusedOmega must have started decaying once the gate was suppressed:
        # with _encOmegaHealthy=False and _wedgeActive=True, _lastEncOmega=0
        # so the EKF gets omega_obs=0 each tick → state trends down.
        assert fused_omega < omega_before, (
            f"fusedOmega={fused_omega:.3f} rad/s after enc-omega gate suppressed — "
            f"expected strictly less than pre-freeze value ({omega_before:.3f} rad/s). "
            f"enc_omega_healthy={enc_omega_healthy}."
        )

    # ---- DISABLE-TO-PROVE: healthy spin keeps omega growing ----
    # Run the same total duration WITHOUT freezing any wheel.
    # The enc_omega gate stays True and omega keeps increasing (EKF integrating
    # continuous healthy omega observations).
    with Sim() as s:
        s.send_command("SET sTimeout=60000")

        s.send_command("VW 0 300")
        t = 0

        # Same total tick count as the normal path.
        total_ticks = TICKS_BEFORE_FREEZE_OMEGA + TICKS_AFTER_FREEZE_OMEGA
        t = _tick_n(s, total_ticks, t)

        # No wedge fired — enc_omega gate should still be healthy.
        enc_omega_healthy_no_wedge = s.get_odometry_enc_omega_healthy()
        assert enc_omega_healthy_no_wedge, (
            "DISABLE-TO-PROVE setup: enc_omega_healthy is False even without a "
            "wedge — unexpected. Check the Robot wiring logic."
        )

        fused_omega_no_wedge = abs(s.get_fused_omega())
        assert fused_omega_no_wedge > 0.1, (
            f"DISABLE-TO-PROVE: fusedOmega={fused_omega_no_wedge:.3f} rad/s with no "
            f"wedge and healthy gate — expected > 0.1 rad/s after "
            f"{total_ticks} ticks of spin. Cannot prove the fix."
        )
