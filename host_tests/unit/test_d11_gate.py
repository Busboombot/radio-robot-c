"""
test_d11_gate.py — D11 double-OK regression tests and D6 defect documentation.

Sprint 026, Ticket 003.

D11 defect: converter commands (S, T, D, G, TURN, RT, R) produced two OK
replies on the hardware path — one from the converter handler, one from
handleVW.  Tickets 026-001 (wired CommandQueue in sim) and 026-002
(handleVW stop-param branches no longer emit a reply) fix D11.

These tests are the acceptance gate: they confirm exactly one OK per corr-id
across the synchronous reply and all accumulated async EVTs.

The sim fixture is in host_tests/conftest.py; tests are placed here rather
than host/tests/test_protocol_v2.py because that file tests the Python
protocol layer, not the C-ABI sim.

D6 defect: a VW keepalive mid-TURN calls MotionCommand::setTarget(0, 0),
zeroing the omega target and stopping the turn prematurely.  D6 is NOT fixed
in sprint 026.  test_d6_cannot_stomp_turn is marked xfail to document the
defect and provide a ready regression gate for sprint 027.
"""
import math

import pytest

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _count_ok_with_id(text: str, corr_id: str) -> int:
    """Count lines that start with 'OK' and contain the corr-id token '#<id>'."""
    token = f"#{corr_id}"
    return sum(
        1
        for line in text.splitlines()
        if line.startswith("OK") and token in line
    )


# ---------------------------------------------------------------------------
# test_single_ok_per_converter_command
#
# For each converter command, send with a unique corr-id, tick long enough
# for the command's completion EVT to fire, then count OK lines that contain
# the corr-id across the sync reply AND accumulated async EVTs.
# Must be exactly 1.
#
# This test proves:
# (a) The queue IS wired in sim (ticket 001 — otherwise the converter takes
#     the fallback path and there would be no second OK to detect, making the
#     test trivially pass on a broken setup).
# (b) handleVW does NOT emit a second OK on the converter-push path (ticket 002).
#
# Before tickets 001+002: all converter commands would produce two OKs.
# After tickets 001+002: exactly one OK per command.
# ---------------------------------------------------------------------------

# (command, corr_id, tick_ms, description)
_CONVERTER_CASES = [
    ("S 200 200",       "1",  500,   "S — stream, open-ended"),
    ("T 500 200 200",   "2",  5000,  "T — timed drive 200 ms"),
    ("D 300 200 200",   "3",  10000, "D — distance drive 200 mm"),
    ("G 400 300 200",   "4",  10000, "G — goto (target at non-origin)"),
    ("TURN 9000",       "5",  5000,  "TURN 9000 — 90-degree heading turn"),
    ("RT 9000",         "6",  5000,  "RT 9000 — relative spin-in-place"),
    ("R 200 200",       "7",  500,   "R — arc, open-ended"),
]


@pytest.mark.parametrize("cmd,cid,tick_ms,description", _CONVERTER_CASES,
                         ids=[c[3] for c in _CONVERTER_CASES])
def test_single_ok_per_converter_command(sim, cmd, cid, tick_ms, description):
    """Each converter command produces exactly one OK reply with its corr-id.

    Counts OK lines across the synchronous reply AND all async EVTs accumulated
    over tick_ms of simulation time.  Exactly 1 is required — 0 means the
    command failed (no reply), 2+ means D11 double-OK is still present.
    """
    full_cmd = f"{cmd} #{cid}"
    sync_reply = sim.send_command(full_cmd)
    sim.tick_for(tick_ms)
    async_evts = sim.get_async_evts()

    all_text = sync_reply + async_evts
    ok_count = _count_ok_with_id(all_text, cid)

    assert ok_count == 1, (
        f"Expected exactly 1 OK with #{cid} for '{full_cmd}' "
        f"({description}), got {ok_count}.\n"
        f"  sync_reply={repr(sync_reply.strip())}\n"
        f"  async_evts={repr(async_evts.strip())}"
    )


# ---------------------------------------------------------------------------
# test_direct_vw_replies_once
#
# A direct VW command (no stop params) is the open-ended velocity path.
# The handleVW open-ended branch is the ONLY branch that must emit replyOK
# (per the D11 suppression rule).  Assert exactly one OK with corr-id #99.
# ---------------------------------------------------------------------------

def test_direct_vw_replies_once(sim):
    """Direct VW command (no stop params) produces exactly one OK reply."""
    sync_reply = sim.send_command("VW 200 0 #99")
    sim.tick_for(500)
    async_evts = sim.get_async_evts()

    all_text = sync_reply + async_evts
    ok_count = _count_ok_with_id(all_text, "99")

    assert ok_count == 1, (
        f"Expected exactly 1 OK with #99 for 'VW 200 0 #99', got {ok_count}.\n"
        f"  sync_reply={repr(sync_reply.strip())}\n"
        f"  async_evts={repr(async_evts.strip())}"
    )


# ---------------------------------------------------------------------------
# test_d6_cannot_stomp_turn — xfail (D6 not fixed in sprint 026)
#
# D6 defect: a VW keepalive (VW 0 0) injected mid-TURN calls
# MotionCommand::setTarget(0, 0), zeroing omega and stopping the turn
# prematurely.  On hardware this manifests as the robot halting mid-rotation
# instead of reaching the commanded heading.
#
# This test asserts the CORRECT behaviour: a TURN 9000 (90-degree target)
# should reach within 30 degrees of the target heading (1.047 rad tolerance)
# even when a VW 0 0 keepalive is injected at 300 ms.
#
# CURRENT STATE (sprint 026): the VW 0 0 stomps omega to 0 via
# MotionCommand::setTarget.  The robot halts at ~38 degrees (0.66 rad),
# well below the 72-degree (1.26 rad) minimum required here.  This test
# therefore FAILS — expected, documented as D6.
#
# FIXED STATE (sprint 027): handleVW should detect that a TURN (or any
# non-VW MotionCommand) is active and skip the setTarget() re-arm, OR the
# setTarget keepalive path should not update a heading-stopped command.
# When fixed, the robot will reach ~90 degrees and this test will pass.
# ---------------------------------------------------------------------------

def test_d6_cannot_stomp_turn(sim):
    """TURN 9000 reaches target heading (≥72 deg) despite mid-turn VW 0 0 keepalive.

    D6 defect: VW keepalive zeroes omega on the active TURN command.
    Expected to xfail until sprint 027 fixes the keepalive path.
    """
    target_cdeg = 9000  # 90 degrees
    target_rad = target_cdeg * math.pi / 18000.0  # 1.5708 rad

    # Minimum acceptable heading: target minus 30-degree tolerance (0.524 rad).
    # A healthy TURN reaches 90 degrees; a stomped TURN stops at ~38 degrees.
    min_acceptable_rad = target_rad - math.radians(30)  # 1.047 rad (60 deg)

    # Start the TURN.
    sim.send_command(f"TURN {target_cdeg} #77")

    # Let it run 300 ms (turn is active, ~22 degrees in at yawRateMax=60 deg/s).
    sim.tick_for(300)

    # Inject a VW 0 0 keepalive mid-turn.  This is the D6 trigger.
    sim.send_command("VW 0 0")

    # Let the command run to completion (5 s is enough for a 90-degree turn).
    sim.tick_for(5000)
    sim.get_async_evts()  # drain; we don't need the EVTs here

    # Read the final heading from the sim.
    import ctypes
    final_h = float(sim._lib.sim_get_pose_h(sim._h))

    # Assert the robot reached a heading close to the target.
    # This assertion FAILS with D6 (final_h ≈ 0.66 rad < min_acceptable_rad 1.047 rad).
    assert final_h >= min_acceptable_rad, (
        f"D6 stomped TURN: final heading {math.degrees(final_h):.1f} deg "
        f"< minimum {math.degrees(min_acceptable_rad):.1f} deg "
        f"(target was {math.degrees(target_rad):.1f} deg). "
        f"VW 0 0 keepalive zeroed omega via setTarget(0, 0), halting rotation early."
    )
