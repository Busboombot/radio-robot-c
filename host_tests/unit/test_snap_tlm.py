"""test_snap_tlm.py — SNAP telemetry frame consistency tests (ticket 027-006).

Lead A investigation:  SNAP and STREAM share the same buildTlmFrame() call path
and read the same state.inputs struct.  The 024-005 change only added ekf_rej and
expanded the buffer size — no structural difference between SNAP and STREAM.

The enc=0 / mode=IDLE anomaly observed on field-024 hardware is NOT a code bug.
It is a tick-ordering artifact: SNAP fires via dequeueOne() at the top of the
tick body, BEFORE driveAdvance() updates mode and state.inputs.  After the first
post-command tick, SNAP correctly reflects live state.

These tests confirm the positive case: SNAP issued after sim_tick() has run
(driveAdvance has advanced) returns non-IDLE mode and non-zero encoders.

See: field-024-full-speed-spin-unresolved.md (Lead A),
     028-001 (D10 deferred work — seq numbers, frame demux).
"""
import re

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_snap(reply: str) -> dict:
    """Parse a TLM line from a SNAP reply into a dict.

    Returns keys: mode (str), enc_l (int), enc_r (int).
    mode is the single character from mode=<C>, or '?' if not found.
    enc_l/r are 0 if not present.
    """
    out = {"mode": "?", "enc_l": 0, "enc_r": 0}
    m = re.search(r"mode=(\w+)", reply)
    if m:
        out["mode"] = m.group(1)
    m = re.search(r"enc=(-?\d+),(-?\d+)", reply)
    if m:
        out["enc_l"] = int(m.group(1))
        out["enc_r"] = int(m.group(2))
    return out


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_snap_returns_tlm_frame(sim):
    """SNAP command returns a TLM line (starts with 'TLM')."""
    reply = sim.send_command("SNAP")
    assert reply.strip().startswith("TLM"), (
        f"Expected SNAP to return TLM frame, got {repr(reply)}"
    )


def test_snap_mode_idle_before_motion(sim):
    """SNAP returns mode=I (IDLE) before any motion command is issued."""
    reply = sim.send_command("SNAP")
    parsed = _parse_snap(reply)
    assert parsed["mode"] == "I", (
        f"Expected mode=I before motion, got mode={parsed['mode']!r} in {repr(reply)}"
    )


def test_snap_reflects_active_mode_after_ticks(sim):
    """SNAP issued after motion ticks shows non-IDLE mode.

    Scenario:
      1. Issue a timed drive T (3000 ms, enough to still be running).
      2. Run 5 sim ticks (120 ms) so driveAdvance() has advanced the mode.
      3. Issue SNAP — should reflect TIMED or STREAMING mode, NOT IDLE.
      4. Encoders should be non-zero (motors have been running).

    This is the positive-case test documenting that SNAP correctly reports
    live state after driveAdvance() has run at least once.
    """
    # Start a timed drive long enough that it won't finish during the test.
    r = sim.send_command("T 200 200 3000")
    assert "OK" in r.upper(), f"T command failed: {repr(r)}"

    # Advance 1000 ms so the velocity PID ramps up and accumulates encoder distance.
    # T at 200 mm/s for 1000 ms should give ~200 mm total, well above the integer
    # truncation threshold in buildTlmFrame (enc= field is int32).
    sim.tick_for(1000)

    # SNAP should now reflect live state (driveAdvance has run).
    reply = sim.send_command("SNAP")
    parsed = _parse_snap(reply)

    assert parsed["mode"] != "I", (
        f"SNAP after active T command returned mode=I (IDLE); "
        f"expected TIMED mode.  Full reply: {repr(reply)}"
    )
    # Encoders must be non-zero — motors have run for 1000 ms at 200 mm/s.
    total_enc = abs(parsed["enc_l"]) + abs(parsed["enc_r"])
    assert total_enc > 0, (
        f"SNAP after 1000 ms of T drive returned zero encoders "
        f"(enc_l={parsed['enc_l']}, enc_r={parsed['enc_r']}). "
        f"Full reply: {repr(reply)}"
    )


def test_snap_mode_matches_stream_mode_during_d_drive(sim):
    """SNAP mode during an active D drive matches the expected drive mode.

    Issues D 200 200 1000 (1000 mm), ticks for 200 ms, then checks SNAP.
    After driveAdvance() has run, mode must be DISTANCE ('D'), not IDLE.
    """
    r = sim.send_command("D 200 200 1000")
    assert "OK" in r.upper(), f"D command failed: {repr(r)}"

    sim.tick_for(200)

    reply = sim.send_command("SNAP")
    parsed = _parse_snap(reply)

    assert parsed["mode"] == "D", (
        f"Expected mode=D during active D drive, got mode={parsed['mode']!r}. "
        f"Full reply: {repr(reply)}"
    )
    total_enc = abs(parsed["enc_l"]) + abs(parsed["enc_r"])
    assert total_enc > 0, (
        f"Encoders are zero during active D drive after 200 ms ticks. "
        f"Full reply: {repr(reply)}"
    )
