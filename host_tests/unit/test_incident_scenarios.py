"""
test_incident_scenarios.py — sprint 027-001 named regression tests for the
four §4 sim2real incident scenarios.

Each test encodes one real failure mode observed in the field-024 and earlier
sessions.  Two tests are immediate regression guards (pass against current
code); two are xfail gates for D6 and D8 fixes that land later in sprint 027.

| Scenario | Ticket | Expected state before fix | After fix |
|----------|--------|--------------------------|-----------|
| test_scenario_g_into_boards     | D8 (027-004) | PURSUE orbits > 3 rev or TIME fires  | converges ≤ 1.5 rev |
| test_scenario_turn_under_rotate | 024 (already fixed) | completes at ~67° physical | OTOS fusion corrects to ≥ 85° |
| test_scenario_keepalive_kills_turn | D6 (027-003) | TURN stopped at wrong heading | TURN reaches commanded heading |
| test_scenario_spin_on_placement | D5 (024, already fixed) | spins forever | exits via TIME net |

These tests use the ``sim_field_profile`` fixture from conftest.py (turn slip
0.26 + OTOS fusion ON) to reproduce field conditions.
"""
import ctypes
import math

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _tick_for_with_keepalives(sim, total_ms: int,
                               step_ms: int = 24,
                               keepalive_interval_ms: int = 200) -> str:
    """Advance sim for total_ms, sending '+' keepalives periodically.

    Drains EVTs before each keepalive so the store is never overwritten.
    Returns all accumulated EVT strings.
    """
    accumulated_evts = ""
    end = sim._t + total_ms
    next_keepalive = sim._t + keepalive_interval_ms

    while sim._t < end:
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += step_ms

        if sim._t >= next_keepalive:
            accumulated_evts += sim.get_async_evts()
            sim.send_command("+")
            next_keepalive += keepalive_interval_ms

    accumulated_evts += sim.get_async_evts()
    return accumulated_evts


def _tick_tracking_heading(sim, total_ms: int,
                            step_ms: int = 24,
                            keepalive_interval_ms: int = 200):
    """Advance sim, tracking total cumulative angular displacement.

    Returns (accumulated_evts: str, total_abs_heading_change_rad: float).
    The total heading change is computed by summing |delta| between consecutive
    heading samples, which captures orbiting (continuous same-direction rotation)
    as well as oscillation.
    """
    accumulated_evts = ""
    end = sim._t + total_ms
    next_keepalive = sim._t + keepalive_interval_ms

    prev_h = float(sim._lib.sim_get_pose_h(sim._h))
    total_delta = 0.0

    while sim._t < end:
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += step_ms

        cur_h = float(sim._lib.sim_get_pose_h(sim._h))
        # Shortest-path delta to handle wrapping.
        dh = cur_h - prev_h
        # Wrap delta to [-π, π].
        while dh > math.pi:
            dh -= 2.0 * math.pi
        while dh < -math.pi:
            dh += 2.0 * math.pi
        total_delta += abs(dh)
        prev_h = cur_h

        if sim._t >= next_keepalive:
            accumulated_evts += sim.get_async_evts()
            sim.send_command("+")
            next_keepalive += keepalive_interval_ms

    accumulated_evts += sim.get_async_evts()
    return accumulated_evts, total_delta


# ---------------------------------------------------------------------------
# test_scenario_g_into_boards
#
# §4.1: G to a target requiring PURSUE in field profile.  The unbounded
# curvature law κ = 2·dy/d² near the target and the narrow 5 mm arrival disc
# (unreachable on carpet / in the slip sim) cause PURSUE to orbit the target
# instead of converging.  With D8 (027-004) the curvature is clamped and the
# arrival disc is widened to 25 mm.
#
# Setup:
#   - Field profile: slipTurnExtra=0.26, OTOS fusion ON.
#   - Target: (80, 0) — directly ahead, bearing = 0° → enters PURSUE immediately.
#     Close enough that d → small triggers κ explosion if unclamped.
#   - Run for up to 20 s with keepalives.
#
# Assertion:
#   - Total angular displacement < 1.5 × 2π (1.5 orbits) before EVT done G or
#     TIME net fires.
#
# D8 (027-004) landed: xfail mark removed.  The curvature clamp and widened
# arriveTolMm=25 allow PURSUE to converge within 1.5 revolutions in sim.
# ---------------------------------------------------------------------------

def test_scenario_g_into_boards(sim_field_profile):
    """G to close-ahead target in field profile: PURSUE must converge within 1.5 orbits.

    §4.1 regression guard.  Without D8, the unbounded curvature law drives the
    robot into a tight orbit around the target.  With D8, the clamp and wider
    arrival disc allow convergence well within 1.5 revolutions.
    """
    sim = sim_field_profile

    # Target directly ahead at 80 mm (bearing = 0 → immediate PURSUE).
    # Speed 150 mm/s gives enough deceleration headroom.
    r = sim.send_command("G 80 0 150")
    assert "OK" in r.upper(), f"Expected OK from G, got {repr(r)}"

    # Tick up to 20 s while tracking heading displacement.
    evts, total_arc_rad = _tick_tracking_heading(sim, 20_000)

    # 1.5 revolutions = 1.5 × 2π ≈ 9.42 rad
    max_orbits_rad = 1.5 * 2.0 * math.pi

    assert total_arc_rad < max_orbits_rad, (
        f"PURSUE orbit guard: total heading displacement {total_arc_rad:.2f} rad "
        f"({total_arc_rad / (2 * math.pi):.2f} rev) exceeds 1.5-orbit limit "
        f"({max_orbits_rad:.2f} rad). D8 (curvature clamp) not yet in effect. "
        f"EVTs: {repr(evts)}"
    )
    assert "safety_stop" not in evts, (
        f"Got unexpected safety_stop in G-into-boards scenario: {repr(evts)}"
    )


# ---------------------------------------------------------------------------
# test_scenario_turn_under_rotate
#
# §4.3: TURN with field-profile turn slip.  Without OTOS heading fusion, the
# encoder-odometry heading under-reports the physical rotation (encoders
# over-report arc due to scrub → poseH arrives at the target before the body
# does, so TURN fires early at ~67° physical).  With OTOS fusion the EKF
# corrects poseH to match the OTOS reading, and TURN fires at ≥ 85°.
#
# This is a regression guard for the sprint-024 OTOS fusion fix — it should
# PASS against current code.
# ---------------------------------------------------------------------------

def test_scenario_turn_under_rotate(sim_field_profile):
    """TURN 9000 in field profile: OTOS fusion ensures heading ≥ 85° at completion.

    §4.3 regression guard (sprint-024 fix already landed).  Turn slip causes
    the encoder to over-report arc, making poseH reach 90° while the body is
    at ~67°.  OTOS EKF fusion corrects poseH so it tracks the true body heading.
    The TURN HEADING stop should fire at ≥ 85° (OTOS-corrected) rather than
    ~67° (encoder-only).
    """
    sim = sim_field_profile

    target_cdeg = 9000  # 90 degrees
    r = sim.send_command(f"TURN {target_cdeg}")
    assert "OK" in r.upper(), f"Expected OK from TURN, got {repr(r)}"

    # Tick up to 10 s with keepalives.  A 90-degree turn should complete in < 3 s.
    evts = _tick_for_with_keepalives(sim, 10_000)

    assert "EVT done TURN" in evts, (
        f"Expected 'EVT done TURN' after TURN 9000 (field profile), got {repr(evts)}"
    )
    assert "safety_stop" not in evts, (
        f"Got unexpected safety_stop: {repr(evts)}"
    )

    # Check final EKF-fused heading: must be >= 85 degrees.
    final_h_rad = float(sim._lib.sim_get_pose_h(sim._h))
    min_acceptable_rad = math.radians(85.0)

    assert final_h_rad >= min_acceptable_rad, (
        f"TURN under-rotate guard: final heading {math.degrees(final_h_rad):.1f} deg "
        f"< 85 deg. OTOS fusion did not correct the encoder under-report. "
        f"(Encoder-only would give ~67 deg.)"
    )


# ---------------------------------------------------------------------------
# test_scenario_keepalive_kills_turn
#
# §4.4: TURN 9000 + mid-flight VW 0 0 keepalive.  The open-ended handleVW
# branch calls MotionCommand::setTarget(0, 0), zeroing ω on the active TURN
# command.  The HEADING stop can never fire; the TIME net fires 2×nominal+2 s
# later and emits EVT done TURN at the wrong heading.
#
# Correct behaviour (after D6 / 027-003): handleVW must detect that a TURN
# (non-VW command) is active and skip setTarget(), leaving ω unchanged.
#
# xfail(strict=True): this test MUST fail before D6 lands (proving the defect
# is real) and MUST pass after.  The strict mark will cause the CI to fail if
# the test unexpectedly passes before the fix is in — which would indicate
# either the test is wrong or the defect was silently fixed.
# ---------------------------------------------------------------------------

def test_scenario_keepalive_kills_turn(sim):
    """TURN 9000 reaches ≥ 85° despite a VW 0 0 keepalive injected mid-flight.

    §4.4 regression gate for D6.  Without the fix, VW 0 0 zeroes omega on the
    active TURN, halting rotation at ~38° and causing the robot to navigate
    from a wrong pose estimate.  With D6 fixed, TURN is protected from VW
    keepalive interference and reaches the commanded heading.
    """
    target_cdeg = 9000  # 90 degrees
    min_acceptable_rad = math.radians(85.0)

    r = sim.send_command(f"TURN {target_cdeg}")
    assert "OK" in r.upper(), f"Expected OK from TURN, got {repr(r)}"

    # Let the turn run for 300 ms (about 22° into rotation at 60 deg/s).
    sim.tick_for(300)

    # Inject the stomping keepalive.  This is the D6 trigger.
    sim.send_command("VW 0 0")

    # Let the command run to completion.
    evts = _tick_for_with_keepalives(sim, 8_000)

    assert "EVT done TURN" in evts, (
        f"Expected 'EVT done TURN' after TURN 9000 + VW keepalive, got {repr(evts)}"
    )
    assert "safety_stop" not in evts, (
        f"Got unexpected safety_stop: {repr(evts)}"
    )

    final_h_rad = float(sim._lib.sim_get_pose_h(sim._h))
    assert final_h_rad >= min_acceptable_rad, (
        f"D6 keepalive-kills-turn: final heading {math.degrees(final_h_rad):.1f} deg "
        f"< 85 deg. VW 0 0 zeroed omega via setTarget(0,0), stopping rotation early."
    )


# ---------------------------------------------------------------------------
# test_scenario_spin_on_placement
#
# §4.2: OTOS pose frozen at (0,0,0) mid-PRE_ROTATE in field profile.  When the
# sensor is invalid (lifted robot) the OTOS reports stale pose.  In PRE_ROTATE,
# the fused poseH never advances to satisfy the HEADING stop, causing the robot
# to spin until power-off.
#
# D5 (sprint 024) added a TIME net to PRE_ROTATE.  This test verifies that net
# is present and bounds the spin.  Should PASS against current code.
#
# The OTOS pose is kept frozen at (0,0,0) by not calling sim_set_otos_pose,
# so the EKF-fused heading stays near 0 (no HEADING stop will fire),
# but the TIME net must terminate the command within its budget.
# ---------------------------------------------------------------------------

def test_scenario_spin_on_placement(sim_field_profile):
    """G to behind target with frozen OTOS exits via TIME net, not infinite spin.

    §4.2 regression guard (D5 sprint-024 fix already landed).  OTOS pose is
    frozen at (0,0,0) so the EKF-fused heading never reaches the PRE_ROTATE
    target.  Without the D5 TIME net the robot spins forever.  With D5 the
    TIME net fires and emits 'EVT done G' within the budget.
    """
    sim = sim_field_profile

    # Target behind-left: bearing ≈ atan2(-300, -100) ≈ -108° → PRE_ROTATE branch.
    # OTOS pose stays frozen at (0,0,0) — we do NOT inject new OTOS readings,
    # so the EKF-fused heading is stuck near 0.  The HEADING stop will never
    # fire; only the TIME net can terminate this command.
    r = sim.send_command("G -100 -300 150")
    assert "OK" in r.upper(), f"Expected OK from G, got {repr(r)}"

    # Run for up to 20 s with keepalives.  The PRE_ROTATE TIME net should fire
    # within a few seconds (2×nominal + 2000 ms budget).
    evts = _tick_for_with_keepalives(sim, 20_000)

    # Must emit "EVT done G" (TIME-net terminal event).
    assert "EVT done G" in evts, (
        f"§4.2 spin-on-placement: expected 'EVT done G' from PRE_ROTATE TIME net "
        f"with frozen OTOS pose, got {repr(evts)}"
    )
    # Must NOT spin forever past the TIME net budget (20 s is generous).
    assert "safety_stop" not in evts, (
        f"Got safety_stop instead of done G — watchdog fired before TIME net: "
        f"{repr(evts)}"
    )
    assert sim._t <= 22_000, (
        f"§4.2 spin-on-placement: sim ran for {sim._t} ms — "
        f"TIME net did not fire within 20 s (spin was unbounded)"
    )
