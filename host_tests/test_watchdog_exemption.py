"""
test_watchdog_exemption.py — sprint 024-003 tests for:

  1. TIME-stop exemption: G/TURN/D/T with a TIME stop run to completion
     with ZERO keepalives sent and safety ON in sim.

  2. Open-ended S without keepalives still safety-stops at sTimeoutMs.

  3. SAFE one-shot re-arm: SAFE off + a new motion command emits
     "EVT safety re-armed" and restores safety for that command.

All tests use the sim fixture (safety ON by default via conftest).
"""
import ctypes
import pytest


# ---------------------------------------------------------------------------
# 1. test_time_stop_exempt_turn
#
# Issue TURN 9000 (90 degrees) with safety ON and NO keepalives.
# The MotionCommand carries HEADING + TIME stops.
# With the TIME-stop exemption the watchdog must NOT fire.
# Command must complete with "EVT done TURN".
# ---------------------------------------------------------------------------

def test_time_stop_exempt_turn(sim):
    """TURN with TIME stop runs to completion; zero keepalives; no safety_stop."""
    # Use real sTimeoutMs (500 ms) to make the exemption meaningful.
    sim.send_command("SET sTimeout=500")

    # Issue TURN — creates a MotionCommand with HEADING + TIME stops.
    sim.send_command("TURN 9000")

    # Tick for 5 s with NO keepalives.  A 90-degree TURN takes ~500 ms.
    all_evts = ""
    for _ in range(5000 // 24):
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += 24

    all_evts = sim.get_async_evts()

    assert "EVT done TURN" in all_evts, (
        f"Expected 'EVT done TURN' with no keepalives (TIME-stop exemption), "
        f"got {repr(all_evts)}"
    )
    assert "safety_stop" not in all_evts, (
        f"Got unexpected 'EVT safety_stop' — TIME-stop exemption should have "
        f"blocked the watchdog: {repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 2. test_time_stop_exempt_d
#
# Issue D 200 200 300 (300 mm drive) with safety ON and NO keepalives.
# D has both DISTANCE and TIME stops.  Watchdog must not fire.
# ---------------------------------------------------------------------------

def test_time_stop_exempt_d(sim):
    """D command with TIME stop completes; zero keepalives; no safety_stop."""
    sim.send_command("SET sTimeout=500")

    sim.send_command("D 200 200 300")

    all_evts = ""
    for _ in range(10000 // 24):
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += 24

    all_evts = sim.get_async_evts()

    assert "EVT done D" in all_evts, (
        f"Expected 'EVT done D' with no keepalives (TIME-stop exemption), "
        f"got {repr(all_evts)}"
    )
    assert "safety_stop" not in all_evts, (
        f"Got unexpected 'EVT safety_stop': {repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 3. test_streaming_watchdog_fires_without_keepalive
#
# Issue S (streaming, NO stop conditions) with safety ON, no keepalives.
# S is open-ended and has no TIME stop — the watchdog must still fire.
# ---------------------------------------------------------------------------

def test_streaming_watchdog_fires_without_keepalive(sim):
    """S (open-ended) with no keepalives still safety-stops at sTimeoutMs."""
    sim.send_command("SET sTimeout=500")

    sim.send_command("S 200 200")

    # Tick for 3 s with NO keepalives — well past the 500 ms window.
    for _ in range(3000 // 24):
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += 24

    evts = sim.get_async_evts()

    assert "EVT safety_stop" in evts, (
        f"Expected 'EVT safety_stop' for S without keepalives, got {repr(evts)}"
    )


# ---------------------------------------------------------------------------
# 4. test_safe_oneshot_rearm_on_begin
#
# SAFE off → issue G (directly ahead, PURSUE branch) → assert:
#   - "EVT safety re-armed" appears (from _checkSafeOneShot in beginGoTo).
#   - "EVT done G" eventually appears (safety is on, but G has TIME stop so
#     it completes normally without keepalives).
#   - No safety_stop during the G command.
# ---------------------------------------------------------------------------

def test_safe_oneshot_rearm_on_begin(sim):
    """SAFE off + motion command emits 'EVT safety re-armed'; safety restored."""
    # Real sTimeoutMs so the exemption matters.
    sim.send_command("SET sTimeout=500")

    # Disable safety (one-shot).
    safe_reply = sim.send_command("SAFE off")
    assert "off" in safe_reply.lower(), (
        f"Expected 'off' in SAFE off reply, got {repr(safe_reply)}"
    )

    # Issue G to a directly-ahead target (PURSUE branch, no PRE_ROTATE).
    # beginGoTo() calls _checkSafeOneShot() which re-arms safety and emits
    # "EVT safety re-armed" via the command reply sink.
    g_reply = sim.send_command("G 300 0 200")

    # "EVT safety re-armed" must appear in the synchronous reply from G
    # (emitted via fn before configure() is called).
    assert "safety re-armed" in g_reply, (
        f"Expected 'EVT safety re-armed' in G command reply, got {repr(g_reply)}"
    )

    # Run to completion without keepalives (G has TIME stop → exempt from watchdog).
    all_evts = ""
    for _ in range(15_000 // 24):
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += 24

    all_evts = sim.get_async_evts()

    assert "EVT done G" in all_evts, (
        f"Expected 'EVT done G' after SAFE off + G, got {repr(all_evts)}"
    )
    assert "safety_stop" not in all_evts, (
        f"Got unexpected safety_stop (TIME-stop exemption should apply): "
        f"{repr(all_evts)}"
    )


# ---------------------------------------------------------------------------
# 5. test_safe_oneshot_rearm_emitted_on_turn
#
# Same as test 4 but via TURN — verifies the re-arm is not limited to beginGoTo.
# SAFE off → TURN 0 → expect "EVT safety re-armed" in the sync reply.
# ---------------------------------------------------------------------------

def test_safe_oneshot_rearm_emitted_on_turn(sim):
    """SAFE off + TURN emits 'EVT safety re-armed' in the sync reply."""
    sim.send_command("SET sTimeout=500")
    sim.send_command("SAFE off")

    turn_reply = sim.send_command("TURN 0")

    assert "safety re-armed" in turn_reply, (
        f"Expected 'EVT safety re-armed' in TURN command reply after SAFE off, "
        f"got {repr(turn_reply)}"
    )

    # TURN 0 is a degenerate zero-degree turn; it may complete instantly or
    # with a single tick.  Just verify no safety_stop fires.
    for _ in range(2000 // 24):
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += 24

    evts = sim.get_async_evts()
    assert "safety_stop" not in evts, (
        f"Unexpected safety_stop after TURN with re-armed safety + TIME stop: "
        f"{repr(evts)}"
    )
