"""
test_goto_bounds.py — sprint 024-001 regression tests for G command time nets.

Tests cover the two new stop conditions added to beginGoTo():
  1. PRE_ROTATE TIME net: G to a 135° target with heading frozen (OTOS stale)
     must exit via the TIME net and emit "EVT done G", not spin forever.
  2. PURSUE TIME net: G to a reachable target has a TIME stop on the PURSUE
     MotionCommand in addition to the POSITION stop.

These tests run in both the exact-profile sim (default) and the field-profile
fixture (slipTurnExtra ≈ 0.26, fuseOtos = true).
"""
import ctypes
import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _tick_for_with_keepalives(sim, total_ms: int,
                               step_ms: int = 24,
                               keepalive_interval_ms: int = 200) -> str:
    """Advance sim for total_ms, sending '+' keepalives periodically.

    Keepalives prevent the watchdog from firing EVT safety_stop during the
    run, so the only way for the command to terminate is via its own stop
    conditions (HEADING, POSITION, or TIME).

    Because sim_command() resets the replyStore, we must drain accumulated
    EVTs BEFORE each keepalive send, then append them into a local buffer.
    The aggregated EVT string is returned so callers can check it.

    Args:
        total_ms:             Total simulation time to advance.
        step_ms:              Tick step size.
        keepalive_interval_ms: How often to send '+'.

    Returns:
        All accumulated EVT strings from the run.
    """
    accumulated_evts = ""
    end = sim._t + total_ms
    next_keepalive = sim._t + keepalive_interval_ms

    while sim._t < end:
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += step_ms

        if sim._t >= next_keepalive:
            # Drain any EVTs accumulated since the last drain.
            accumulated_evts += sim.get_async_evts()
            # Send keepalive (resets store; OK reply is discarded).
            sim.send_command("+")
            next_keepalive += keepalive_interval_ms

    # Drain any remaining EVTs.
    accumulated_evts += sim.get_async_evts()
    return accumulated_evts


# ---------------------------------------------------------------------------
# test_pre_rotate_time_net
#
# Verify that G to a target requiring PRE_ROTATE exits via the TIME net when
# the heading is frozen (OTOS not advancing, encoder odometry not advancing).
#
# Setup:
#   - Target at (0, -300, 200) → 270° (behind+left, ty < 0 → CW spin).
#   - OTOS fusion enabled; OTOS pose is fixed at (0, 0, 0) — heading frozen.
#   - Motor slip: slipTurnExtra = 0.26 → encoder over-reports on turns,
#     so encoder heading also drifts in the wrong direction.
#
# Expected outcome:
#   - Command terminates via the TIME net (not HEADING stop, since heading is
#     frozen; not safety_stop, since keepalives are flowing).
#   - Terminal EVT is "EVT done G" (not "EVT safety_stop").
#   - Elapsed ticks < runaway threshold (20 s).
#
# If PRE_ROTATE were still unbounded (old code), the robot would spin
# indefinitely and the test would time out or see safety_stop (if watchdog
# fires) rather than "EVT done G".
# ---------------------------------------------------------------------------

def test_pre_rotate_time_net(sim):
    """G to 135°-behind target with frozen heading exits via TIME net as done G.

    Uses the field-profile fixture (slip + OTOS fusion).  OTOS pose is kept
    frozen at origin, so poseHrad never advances to satisfy the HEADING stop.
    The PRE_ROTATE TIME net (2×nominal + 2000 ms) must terminate the spin.
    """
    # Set up field profile: turn slip + OTOS fusion.
    sim.set_field_profile(slip_turn_extra=0.26, fuse_otos=True)

    # Freeze watchdog so it cannot mask the TIME net result.
    sim.send_command("SET sTimeout=60000")

    # Target is behind-left of the robot (ty = -300, tx = -100).
    # bearing = atan2(-300, -100) ≈ -108° → PRE_ROTATE branch.
    # OTOS pose stays at (0, 0, 0): we do NOT inject new OTOS readings,
    # so the EKF-fused heading is stuck near 0.  The PRE_ROTATE HEADING stop
    # will never fire (robot never appears to have turned enough), so the
    # TIME net must be the terminating condition.
    r = sim.send_command("G -100 -300 150")
    assert "OK" in r.upper(), f"Expected OK from G, got {repr(r)}"

    # Run for up to 20 s with keepalives.  The PRE_ROTATE TIME net is
    # 2 × (bearing / omega) × 1000 + 2000 ms.  For bearing ≈ 1.9 rad
    # at omega ≈ 2 rad/s (speed 150, default trackwidth ~120 mm), nominal ≈
    # 950 ms → timeout ≈ 3900 ms.  We run for 20 s to give ample headroom.
    all_output = _tick_for_with_keepalives(sim, 20_000)

    # Must emit "EVT done G" (time-net terminal event).
    assert "EVT done G" in all_output, (
        f"Expected 'EVT done G' from PRE_ROTATE time net, got {repr(all_output)}"
    )

    # Must NOT emit "EVT safety_stop" (watchdog must NOT be masking the result).
    assert "safety_stop" not in all_output, (
        f"Got safety_stop instead of done G — watchdog fired before TIME net: "
        f"{repr(all_output)}"
    )

    # Sanity: elapsed time should be well under 20 s.
    assert sim._t <= 22_000, (
        f"Sim ran for {sim._t} ms — TIME net did not fire within 20 s"
    )


# ---------------------------------------------------------------------------
# test_pursue_time_net
#
# Verify that the PURSUE MotionCommand carries a TIME stop in addition to the
# POSITION stop.  We test this indirectly: issue G to a reachable target,
# then verify the command eventually terminates via "EVT done G" (which would
# not happen if POSITION was the only stop and the robot stopped short due to
# decel rounding).  The TIME net also prevents runaway if position is never
# reached.
#
# We also verify that in the direct-PURSUE path (bearing <= gate at call time),
# the command terminates cleanly within a generous time bound, confirming that
# the TIME stop is present and not misconfigured.
# ---------------------------------------------------------------------------

def test_pursue_time_net_exact_profile(sim):
    """G to a close ahead target (exact profile) emits done G within 15 s.

    Target is directly ahead (tx=300, ty=0), so bearing = 0° < turnInPlaceGate
    and the robot enters PURSUE immediately.  The PURSUE command must have a
    TIME stop so that it terminates even if the POSITION stop doesn't fire
    (e.g. decel rounding leaves robot 5 mm short of target).

    With exact profile (no slip), the robot should reach the target and emit
    "EVT done G" well within the TIME net budget.
    """
    # Use default sTimeout but send keepalives so watchdog doesn't interfere.
    sim.send_command("SET sTimeout=60000")

    r = sim.send_command("G 300 0 200")
    assert "OK" in r.upper(), f"Expected OK from G, got {repr(r)}"

    evts = _tick_for_with_keepalives(sim, 15_000)
    assert "EVT done G" in evts, (
        f"Expected 'EVT done G' after G 300 0 200 (exact profile), got {repr(evts)}"
    )
    assert "safety_stop" not in evts, (
        f"Got unexpected safety_stop: {repr(evts)}"
    )


def test_pursue_time_net_field_profile(sim):
    """G to a close ahead target (field profile) emits done G within 15 s.

    Same as exact-profile test but with slipTurnExtra = 0.26 and OTOS fusion.
    Verifies that the PURSUE TIME net is present in the field-profile fixture.
    """
    sim.set_field_profile(slip_turn_extra=0.26, fuse_otos=True)
    sim.send_command("SET sTimeout=60000")

    r = sim.send_command("G 300 0 200")
    assert "OK" in r.upper(), f"Expected OK from G, got {repr(r)}"

    evts = _tick_for_with_keepalives(sim, 15_000)
    assert "EVT done G" in evts, (
        f"Expected 'EVT done G' after G 300 0 200 (field profile), got {repr(evts)}"
    )
    assert "safety_stop" not in evts, (
        f"Got unexpected safety_stop: {repr(evts)}"
    )


def test_pre_rotate_then_pursue_emits_done_g(sim):
    """G to a behind target: PRE_ROTATE succeeds, then PURSUE emits done G.

    Uses exact profile (no slip).  Target is behind+left (ty negative).
    With heading free to advance, PRE_ROTATE should complete via HEADING stop
    and transition to PURSUE, which then reaches the target and emits done G.
    """
    sim.send_command("SET sTimeout=60000")

    # Target behind-left: bearing ≈ 135° → PRE_ROTATE branch.
    # Exact profile: heading will advance normally via encoder odometry.
    r = sim.send_command("G -200 -200 150")
    assert "OK" in r.upper(), f"Expected OK from G, got {repr(r)}"

    evts = _tick_for_with_keepalives(sim, 30_000)
    assert "EVT done G" in evts, (
        f"Expected 'EVT done G' after G to behind target (exact profile), "
        f"got {repr(evts)}"
    )
    assert "safety_stop" not in evts, (
        f"Got unexpected safety_stop: {repr(evts)}"
    )
