"""
test_cancel_on_begin.py -- sprint 024-002 tests for cancel-on-begin* guards.

Every begin*() entry point must cancel any active MotionCommand before
configuring the new one.  These tests verify:

  1. test_cancel_on_begin_goto_pre_rotate:
       Start a TURN, issue G mid-flight -> TURN emits "EVT cancelled",
       exactly one active command exists afterward (the new PRE_ROTATE G),
       and the PRE_ROTATE structure from ticket 001 is in place (terminates
       via TIME net with "EVT done G" rather than spinning forever).

  2. test_cancel_on_begin_goto_pursue:
       Start a TURN, issue G to a directly-ahead target (no PRE_ROTATE) ->
       TURN emits "EVT cancelled", new PURSUE command emits "EVT done G".

  3. test_cancel_on_begin_turn:
       Start a TURN (active command), issue a second TURN mid-flight ->
       first TURN emits "EVT cancelled", second TURN completes with
       "EVT done TURN".

  4. test_cancel_on_begin_arc:
       Start a TURN (active MotionCommand), issue R (arc) mid-flight ->
       TURN emits "EVT cancelled", R drives motors.

  5. test_back_to_back_g_no_duplicate_evt:
       Field-profile: issue G, then issue G again before it finishes ->
       exactly one "EVT done G" total in the output; no duplicate or
       mismatched EVT labels.

Note on beginVelocity:
  The handleVW() open-ended path uses a "keepalive re-send" optimisation:
  when any MotionCommand is already active, VW updates the target in-place
  rather than calling beginVelocity() again.  This is intentional -- it is
  the keepalive protocol that prevents the watchdog from firing.  As a
  result, the cancel guard in beginVelocity() is not exercised via the VW
  wire command in the sim (no queue).  It is exercised whenever a direct
  caller (e.g. a future composite command or a testing harness) calls
  beginVelocity() directly with an active command already present.  The guard
  is present and correctly placed; the sim does not provide a natural path to
  trigger it through the wire command layer.
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
# 1. test_cancel_on_begin_goto_pre_rotate
#
# Setup: issue TURN to 90 degrees (active, not yet arrived).
#        After a few ticks (TURN still running), issue G to a behind+left target
#        (ty = -200, tx = -200) which triggers the PRE_ROTATE branch.
#
# cancel() fires synchronously inside beginGoTo() during cmd.process() for the
# G command, so "EVT cancelled" appears in the sync reply from sim_command("G").
#
# Expected:
#   - "EVT cancelled" appears in the G command's sync reply.
#   - "EVT done G" eventually appears (PRE_ROTATE TIME net or HEADING success).
#   - No "EVT done TURN" (TURN was cancelled, not completed).
# ---------------------------------------------------------------------------

def test_cancel_on_begin_goto_pre_rotate(sim):
    """TURN preempted by G (PRE_ROTATE) -> EVT cancelled + EVT done G; no done TURN."""
    sim.send_command("SET sTimeout=60000")

    # Start TURN to 9000 cdeg (90 degrees) -- will take several hundred ms to arrive.
    sim.send_command("TURN 9000")

    # Let a few ticks pass so the TURN is genuinely active mid-flight.
    sim.tick_for(96)

    # Drain any early EVTs (should be none yet for a 90-degree TURN after 96 ms).
    sim.get_async_evts()

    # Issue G to a behind+left target -- enters PRE_ROTATE (cancels the TURN).
    # cancel() fires synchronously inside beginGoTo() -> the "EVT cancelled" is
    # written to the reply store during cmd.process() and returned in the sync
    # reply from sim_command().  We must capture it here, not via get_async_evts()
    # (which only sees EVTs emitted during subsequent sim_tick() calls).
    g_reply = sim.send_command("G -200 -200 150")

    # Run for up to 30 s with keepalives; PRE_ROTATE + PURSUE should finish.
    all_evts = g_reply + _tick_with_keepalives(sim, 30_000)

    # The preempted TURN must have emitted "EVT cancelled".
    assert "EVT cancelled" in all_evts, (
        f"Expected 'EVT cancelled' for preempted TURN, got {repr(all_evts)}"
    )

    # G must eventually produce "EVT done G".
    assert "EVT done G" in all_evts, (
        f"Expected 'EVT done G' from G after TURN cancellation, got {repr(all_evts)}"
    )

    # TURN must NOT have completed normally.
    assert "EVT done TURN" not in all_evts, (
        f"Got unexpected 'EVT done TURN' -- TURN should have been cancelled: "
        f"{repr(all_evts)}"
    )

    # Watchdog must not have fired (keepalives were flowing).
    assert "safety_stop" not in all_evts, (
        f"Got unexpected safety_stop: {repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 2. test_cancel_on_begin_goto_pursue
#
# Setup: start a TURN, then issue G to a directly-ahead target (ty=0, tx=300)
#        so G enters the PURSUE branch immediately (no PRE_ROTATE).
#
# Expected:
#   - "EVT cancelled" in G command's sync reply (from the preempted TURN).
#   - "EVT done G" when PURSUE reaches the target.
#   - No "EVT done TURN".
# ---------------------------------------------------------------------------

def test_cancel_on_begin_goto_pursue(sim):
    """TURN preempted by G (PURSUE branch) -> EVT cancelled + EVT done G."""
    sim.send_command("SET sTimeout=60000")

    # Start TURN to 9000 cdeg (90 degrees).
    sim.send_command("TURN 9000")
    sim.tick_for(96)
    sim.get_async_evts()

    # Issue G directly ahead -- enters PURSUE (no PRE_ROTATE); cancels TURN.
    # "EVT cancelled" fires synchronously in the G reply (see comment in test 1).
    g_reply = sim.send_command("G 300 0 200")

    all_evts = g_reply + _tick_with_keepalives(sim, 15_000)

    assert "EVT cancelled" in all_evts, (
        f"Expected 'EVT cancelled' for preempted TURN, got {repr(all_evts)}"
    )
    assert "EVT done G" in all_evts, (
        f"Expected 'EVT done G' from G (PURSUE), got {repr(all_evts)}"
    )
    assert "EVT done TURN" not in all_evts, (
        f"Got unexpected 'EVT done TURN': {repr(all_evts)}"
    )
    assert "safety_stop" not in all_evts, (
        f"Got unexpected safety_stop: {repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 3. test_cancel_on_begin_turn
#
# Setup: start a TURN to a large angle, then issue a second TURN mid-flight.
#        beginTurn() has the cancel guard; the second TURN cancels the first.
#
# Expected:
#   - "EVT cancelled" in the second TURN command's sync reply.
#   - "EVT done TURN" when the second TURN arrives at its target.
#   - Exactly one "EVT done TURN" (not two).
# ---------------------------------------------------------------------------

def test_cancel_on_begin_turn(sim):
    """Second TURN preempts first TURN -> EVT cancelled + exactly one EVT done TURN."""
    sim.send_command("SET sTimeout=60000")

    # Start TURN to 9000 cdeg (90 degrees) -- takes ~500 ms to arrive.
    sim.send_command("TURN 9000")
    sim.tick_for(96)
    sim.get_async_evts()

    # Issue second TURN to 0 cdeg (home heading) -- cancels the first TURN.
    turn2_reply = sim.send_command("TURN 0")

    # Run to completion.
    all_evts = turn2_reply + _tick_with_keepalives(sim, 10_000)

    # First TURN must have been cancelled.
    assert "EVT cancelled" in all_evts, (
        f"Expected 'EVT cancelled' for first TURN, got {repr(all_evts)}"
    )

    # Second TURN must complete.
    assert "EVT done TURN" in all_evts, (
        f"Expected 'EVT done TURN' from second TURN, got {repr(all_evts)}"
    )

    # Exactly one done TURN (not two from ghost completion of the first).
    done_turn_count = all_evts.count("EVT done TURN")
    assert done_turn_count == 1, (
        f"Expected exactly 1 'EVT done TURN', got {done_turn_count}: {repr(all_evts)}"
    )

    assert "safety_stop" not in all_evts, (
        f"Got unexpected safety_stop: {repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 4. test_cancel_on_begin_arc
#
# Setup: start a TURN (active MotionCommand), then issue R (arc) mid-flight.
#        beginArc() has the cancel guard; TURN is cancelled before R starts.
#
# Note: handleR() calls beginArc() directly (no keepalive optimisation),
#       so the cancel guard in beginArc() is exercised.
#
# Expected:
#   - "EVT cancelled" in the R command's sync reply.
#   - Motors are running after R starts (encoders grow).
#   - No "EVT done TURN" (TURN was cancelled).
# ---------------------------------------------------------------------------

def test_cancel_on_begin_arc(sim):
    """TURN preempted by R -> EVT cancelled for TURN; R drives motors; no done TURN."""
    sim.send_command("SET sTimeout=60000")

    # Start TURN to 9000 cdeg -- active command.
    sim.send_command("TURN 9000")
    sim.tick_for(96)
    sim.get_async_evts()

    # Issue R (arc) -- beginArc cancels the active TURN command.
    r_reply = sim.send_command("R 200 500")

    # Tick 500 ms so R has time to drive the motors.
    sim.tick_for(500)
    enc_l = float(sim._lib.sim_get_enc_l(sim._h))

    all_evts = r_reply + sim.get_async_evts()

    # TURN must have been cancelled.
    assert "EVT cancelled" in all_evts, (
        f"Expected 'EVT cancelled' for preempted TURN, got {repr(all_evts)}"
    )

    # R must have driven motors.
    assert enc_l > 0.0, (
        f"Expected R to drive encoders after TURN cancel: enc_l={enc_l:.1f}"
    )

    # TURN must NOT have completed normally.
    assert "EVT done TURN" not in all_evts, (
        f"Got unexpected 'EVT done TURN' -- TURN should have been cancelled: "
        f"{repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 5. test_back_to_back_g_no_duplicate_evt (field-profile)
#
# Issue G, let it run briefly (PRE_ROTATE spinning), then issue a second G
# before it finishes.  The first G is in PRE_ROTATE which has no reply sink
# (by design), so no "EVT cancelled" fires.  The stale PRE_ROTATE command is
# silently cancelled by the cancel guard, and the second G runs to completion.
#
# Invariant: exactly one "EVT done G" in the final output.
# Forbidden: more than one "EVT done G", any "EVT done TURN", or any other
#            spurious labels that would indicate ghost commands.
# ---------------------------------------------------------------------------

def test_back_to_back_g_no_duplicate_evt(sim):
    """Back-to-back G commands: exactly one EVT done G; no duplicate labels."""
    sim.set_field_profile(slip_turn_extra=0.26, fuse_otos=True)
    sim.send_command("SET sTimeout=60000")

    # First G: target is behind-left (PRE_ROTATE path).
    sim.send_command("G -200 -200 150")
    # Let it run briefly (PRE_ROTATE spinning).
    sim.tick_for(500)
    sim.get_async_evts()

    # Second G: directly ahead (PURSUE path) -- preempts the first G.
    # PRE_ROTATE has no reply sink so no "EVT cancelled" fires here.
    g2_reply = sim.send_command("G 300 0 200")

    # Run to completion with keepalives.
    all_evts = g2_reply + _tick_with_keepalives(sim, 20_000)

    # Exactly one "EVT done G" must appear.
    done_g_count = all_evts.count("EVT done G")
    assert done_g_count == 1, (
        f"Expected exactly 1 'EVT done G', got {done_g_count}: {repr(all_evts)}"
    )

    # No "EVT done TURN" or other spurious motion EVTs (no ghost commands).
    assert "EVT done TURN" not in all_evts, (
        f"Got spurious 'EVT done TURN': {repr(all_evts)}"
    )

    # Watchdog must not have fired.
    assert "safety_stop" not in all_evts, (
        f"Got unexpected safety_stop: {repr(all_evts)}"
    )
