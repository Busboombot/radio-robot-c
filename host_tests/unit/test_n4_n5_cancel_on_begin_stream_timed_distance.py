"""
test_n4_n5_cancel_on_begin_stream_timed_distance.py — N4 + N5 regression tests
for sprint 030-004.

N4: beginStream() and beginRawVelocity() must cancel any active MotionCommand
    before seeding the BVC.  An S issued while TURN/G/T/D is running previously
    left _activeCmd alive (zombie supervisor) — the old command's TIME/HEADING
    stop would fire, soft-stop the robot, and emit a stale EVT done, silently
    killing the stream.  The old command also never got EVT cancelled.

N5: beginTimed() and beginDistance() must emit EVT cancelled for the preempted
    command's corrId before configuring the new command.  Previously they called
    configure() immediately, resetting the reply sink and leaving the preempted
    command without a terminal event.

Tests:
  1. test_s_mid_turn_emits_cancelled_p1_1
       P1.1 verify scenario: start TURN, inject S 0 0 mid-turn on the queue
       path.  TURN must get EVT cancelled; the S stream must NOT kill the TURN
       before it gets cancelled (no stale EVT done TURN after S takes over).
       The S 0 0 should stop the robot cleanly.

  2. test_g_preempted_by_t_emits_cancelled
       Start G, inject T mid-flight.  Host must receive EVT cancelled for the G
       corrId before any T-related event.  No EVT done G after the cancel.

  3. test_s_mid_distance_emits_cancelled
       Start D (distance drive), inject S 0 0 mid-flight.
       D must emit EVT cancelled; S takes over.

  4. test_t_preempts_running_turn_emits_cancelled
       Start TURN (active), issue T mid-flight.  TURN must emit EVT cancelled;
       T runs to completion (EVT done T).

  5. test_d_preempts_running_turn_emits_cancelled
       Start TURN (active), issue D mid-flight.  TURN must emit EVT cancelled;
       D runs (EVT done D).

  6. test_s_keepalive_during_vw_not_cancelled
       D6 regression: a plain VW keepalive during an active VW-origin command
       must NOT cancel it (the origin guard sends it to setTarget instead).
       The active command must remain running after the keepalive.
"""
import ctypes
import pytest


# ---------------------------------------------------------------------------
# Helper: tick with keepalives, accumulating all EVTs
# ---------------------------------------------------------------------------

def _tick_with_keepalives(sim, total_ms: int,
                           step_ms: int = 24,
                           keepalive_interval_ms: int = 200) -> str:
    """Advance sim for total_ms, injecting '+' keepalives periodically.

    Returns all accumulated EVT strings.
    """
    accumulated = ""
    end = sim._t + total_ms
    next_keepalive = sim._t + keepalive_interval_ms

    while sim._t < end:
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += step_ms

        if sim._t >= next_keepalive:
            accumulated += sim.get_async_evts()
            sim.send_command("+")
            next_keepalive += keepalive_interval_ms

    accumulated += sim.get_async_evts()
    return accumulated


# ---------------------------------------------------------------------------
# 1. test_s_mid_turn_emits_cancelled_p1_1
#
# P1.1 verify scenario (N4): start TURN to 9000 cdeg (90 deg), inject S 0 0
# mid-turn on the queue path.  The TURN must receive EVT cancelled immediately;
# the robot must NOT produce a stale EVT done TURN later (the zombie stop).
#
# Expected:
#   - "EVT cancelled" in the synchronous reply from the S command (since cancel
#     fires inside beginStream() which is called during cmd.dequeueOne() inside
#     sim_command()).
#   - No "EVT done TURN" in any subsequent ticks.
#   - No "EVT done D" or other stale motion EVTs.
# ---------------------------------------------------------------------------

def test_s_mid_turn_emits_cancelled_p1_1(sim):
    """P1.1: S 0 0 mid-TURN on queue path → EVT cancelled for TURN; no stale done TURN."""
    # Start a 90-degree TURN — takes ~500 ms to arrive at default yawRateMax.
    sim.send_command("TURN 9000")

    # Let a few ticks pass so the TURN is genuinely active mid-flight.
    sim.tick_for(96)
    sim.get_async_evts()   # drain any early EVTs (none expected yet)

    # Inject S 0 0 mid-TURN on the queue path.
    # beginStream() fires cancel() synchronously inside dequeueOne() during
    # sim_command(), so EVT cancelled appears in the sync reply.
    s_reply = sim.send_command("S 0 0")

    # Run for 3 s with keepalives so any stale zombie EVTs would fire.
    all_evts = s_reply + _tick_with_keepalives(sim, 3_000)

    # The preempted TURN must have emitted "EVT cancelled".
    assert "EVT cancelled" in all_evts, (
        f"Expected 'EVT cancelled' for preempted TURN; got {repr(all_evts)}"
    )

    # There must be NO stale "EVT done TURN" (zombie stop firing after S took over).
    assert "EVT done TURN" not in all_evts, (
        f"Got stale 'EVT done TURN' — zombie TURN supervisor still active: "
        f"{repr(all_evts)}"
    )

    # No other unexpected motion completions.
    assert "EVT done D" not in all_evts, (
        f"Got unexpected 'EVT done D': {repr(all_evts)}"
    )
    assert "safety_stop" not in all_evts, (
        f"Got unexpected safety_stop: {repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 2. test_g_preempted_by_t_emits_cancelled
#
# N5 scenario: start G to a directly-ahead target (PURSUE branch), then issue
# T mid-flight.  Host must receive EVT cancelled for the G; T runs to completion.
#
# Expected:
#   - "EVT cancelled" in the T command's sync reply.
#   - "EVT done T" after the T duration.
#   - No "EVT done G" (G was cancelled before it could complete).
# ---------------------------------------------------------------------------

def test_g_preempted_by_t_emits_cancelled(sim):
    """G (PURSUE) preempted by T → EVT cancelled for G; EVT done T; no done G."""
    # Start G directly ahead (no PRE_ROTATE).
    sim.send_command("G 500 0 200")

    # Let G enter PURSUE for a bit.
    sim.tick_for(200)
    sim.get_async_evts()

    # Issue T mid-flight — beginTimed() must cancel G and emit EVT cancelled.
    t_reply = sim.send_command("T 100 100 500")

    # Run to completion with keepalives.
    all_evts = t_reply + _tick_with_keepalives(sim, 5_000)

    # The preempted G must have emitted "EVT cancelled".
    assert "EVT cancelled" in all_evts, (
        f"Expected 'EVT cancelled' for preempted G; got {repr(all_evts)}"
    )

    # T must complete normally.
    assert "EVT done T" in all_evts, (
        f"Expected 'EVT done T'; got {repr(all_evts)}"
    )

    # G must NOT have completed normally (it was cancelled).
    assert "EVT done G" not in all_evts, (
        f"Got unexpected 'EVT done G' — G should have been cancelled: "
        f"{repr(all_evts)}"
    )

    assert "safety_stop" not in all_evts, (
        f"Got unexpected safety_stop: {repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 3. test_s_mid_distance_emits_cancelled
#
# N4 scenario: start D (distance drive), then inject S 0 0 mid-flight.
# D must emit EVT cancelled; the stream takes over cleanly.
#
# Expected:
#   - "EVT cancelled" in S's sync reply.
#   - No "EVT done D" (D was cancelled before completion).
# ---------------------------------------------------------------------------

def test_s_mid_distance_emits_cancelled(sim):
    """S 0 0 mid-D → EVT cancelled for D; no stale EVT done D."""
    # Start a distance drive (100 mm at 150 mm/s — would take ~700 ms).
    sim.send_command("D 150 150 1000")

    # Let D run for a bit.
    sim.tick_for(200)
    sim.get_async_evts()

    # Inject S 0 0 mid-flight — beginStream() must cancel D.
    s_reply = sim.send_command("S 0 0")

    # Run for 3 s with keepalives to catch any zombie EVTs.
    all_evts = s_reply + _tick_with_keepalives(sim, 3_000)

    # D must have been cancelled.
    assert "EVT cancelled" in all_evts, (
        f"Expected 'EVT cancelled' for preempted D; got {repr(all_evts)}"
    )

    # No stale EVT done D (zombie stop must not fire).
    assert "EVT done D" not in all_evts, (
        f"Got stale 'EVT done D' — zombie D supervisor still active: "
        f"{repr(all_evts)}"
    )

    assert "safety_stop" not in all_evts, (
        f"Got unexpected safety_stop: {repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 4. test_t_preempts_running_turn_emits_cancelled
#
# N5 scenario: start TURN (active MotionCommand), then issue T mid-flight.
# beginTimed() must cancel the TURN and emit EVT cancelled.
#
# Expected:
#   - "EVT cancelled" in T's sync reply.
#   - "EVT done T" after the T duration.
#   - Exactly one "EVT done T"; no "EVT done TURN".
# ---------------------------------------------------------------------------

def test_t_preempts_running_turn_emits_cancelled(sim):
    """TURN preempted by T → EVT cancelled + EVT done T; no EVT done TURN."""
    # Start a 90-degree TURN.
    sim.send_command("TURN 9000")
    sim.tick_for(96)
    sim.get_async_evts()

    # Issue T mid-flight.
    t_reply = sim.send_command("T 100 100 400")
    all_evts = t_reply + _tick_with_keepalives(sim, 5_000)

    assert "EVT cancelled" in all_evts, (
        f"Expected 'EVT cancelled' for preempted TURN; got {repr(all_evts)}"
    )
    assert "EVT done T" in all_evts, (
        f"Expected 'EVT done T'; got {repr(all_evts)}"
    )
    assert "EVT done TURN" not in all_evts, (
        f"Got unexpected 'EVT done TURN' — TURN should have been cancelled: "
        f"{repr(all_evts)}"
    )
    assert "safety_stop" not in all_evts, (
        f"Got unexpected safety_stop: {repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 5. test_d_preempts_running_turn_emits_cancelled
#
# N5 scenario: start TURN, then issue D mid-flight.  beginDistance() must
# cancel the TURN and emit EVT cancelled before starting D.
#
# Expected:
#   - "EVT cancelled" in D's sync reply.
#   - "EVT done D" when distance is reached.
#   - No "EVT done TURN".
# ---------------------------------------------------------------------------

def test_d_preempts_running_turn_emits_cancelled(sim):
    """TURN preempted by D → EVT cancelled + EVT done D; no EVT done TURN."""
    # Start a 90-degree TURN.
    sim.send_command("TURN 9000")
    sim.tick_for(96)
    sim.get_async_evts()

    # Issue D mid-flight (short distance so it completes quickly).
    d_reply = sim.send_command("D 150 150 200")
    all_evts = d_reply + _tick_with_keepalives(sim, 8_000)

    assert "EVT cancelled" in all_evts, (
        f"Expected 'EVT cancelled' for preempted TURN; got {repr(all_evts)}"
    )
    assert "EVT done D" in all_evts, (
        f"Expected 'EVT done D'; got {repr(all_evts)}"
    )
    assert "EVT done TURN" not in all_evts, (
        f"Got unexpected 'EVT done TURN' — TURN should have been cancelled: "
        f"{repr(all_evts)}"
    )
    assert "safety_stop" not in all_evts, (
        f"Got unexpected safety_stop: {repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 6. test_s_keepalive_during_vw_not_cancelled  (D6 regression)
#
# A plain VW keepalive (no stream=1) during an active VW-origin command must
# NOT cancel it — the D6 origin guard in handleVW intercepts it and calls
# setTarget() instead of beginStream().  This verifies the D6 guard is intact.
#
# Protocol: after the initial VW the robot is in VELOCITY/VW mode.
# Subsequent VW commands (keepalives) call handleVW → origin guard → setTarget.
# The robot must still be running (encoders growing) after the keepalive.
#
# Expected:
#   - No "EVT cancelled" after a VW keepalive.
#   - Encoders growing after the keepalive (robot still driving).
# ---------------------------------------------------------------------------

def test_s_keepalive_during_vw_not_cancelled(sim):
    """D6 regression: VW keepalive during active VW-origin command must not cancel it."""
    # Start an open-ended VW command (no stop params → beginVelocity, VW origin).
    sim.send_command("VW 150 0")

    # Let it run for a bit.
    sim.tick_for(200)
    sim.get_async_evts()

    enc_before = float(sim._lib.sim_get_enc_l(sim._h))

    # Send another VW (keepalive) — must NOT cancel the active command.
    vw_reply = sim.send_command("VW 150 0")

    # Tick a bit more.
    sim.tick_for(200)
    enc_after = float(sim._lib.sim_get_enc_l(sim._h))

    all_evts = vw_reply + sim.get_async_evts()

    # No EVT cancelled — keepalive must not preempt.
    assert "EVT cancelled" not in all_evts, (
        f"Got unexpected 'EVT cancelled' from VW keepalive — D6 guard broken: "
        f"{repr(all_evts)}"
    )

    # Encoders must have grown (robot still driving).
    assert enc_after > enc_before, (
        f"Robot stopped after VW keepalive: enc_before={enc_before:.1f} "
        f"enc_after={enc_after:.1f}"
    )

    assert "safety_stop" not in all_evts, (
        f"Got unexpected safety_stop: {repr(all_evts)}"
    )
