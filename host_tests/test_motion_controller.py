"""
test_motion_controller.py — tests for the MotionController state machine.

Tests exercise the D (distance drive) and VW (body velocity) commands via the
simulation.  sim_tick() calls both controlCollectSplitPhase() and driveAdvance(),
so the full motion command pipeline runs.
"""
import pytest


def test_ping_sanity(sim):
    """Sanity check: PING returns OK before any motion tests."""
    r = sim.send_command("PING")
    assert "OK" in r.upper()


def test_d_command_drives_distance(sim):
    """D command 500 mm: motors stop and encoders sum to ~1000 mm."""
    r = sim.send_command("D 200 200 500")
    assert "OK" in r.upper(), f"Expected OK from D command, got {repr(r)}"

    # Tick up to 10 s; D should complete well before then.
    sim.tick_for(10000)

    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r = float(sim._lib.sim_get_enc_r(sim._h))
    total = enc_l + enc_r

    # Both wheels targeted at 500 mm so total should be ≈ 1000 mm.
    assert total >= 800.0, (
        f"Expected enc_l + enc_r >= 800 mm after D 500mm, got {total:.2f}"
    )

    # Motor should have stopped (D command completed).
    pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
    assert pwm_l == 0.0, f"Expected motor stopped after D completes, pwm_l={pwm_l}"


def test_d_command_emits_done_evt(sim):
    """D command emits EVT done D upon completion."""
    sim.send_command("D 200 200 200")

    # Tick enough for a 200mm drive to complete.
    sim.tick_for(10000)

    evts = sim.get_async_evts()
    assert "EVT done D" in evts, (
        f"Expected 'EVT done D' in async EVTs, got {repr(evts)}"
    )


def test_vw_command_drives_encoder(sim):
    """VW 200 0 command (forward 200 mm/s) makes encoder grow over 100 ticks."""
    r = sim.send_command("VW 200 0")
    assert "OK" in r.upper(), f"Expected OK from VW command, got {repr(r)}"

    # Tick for 100 steps (2.4 s simulated).
    sim.tick_for(2400)

    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r = float(sim._lib.sim_get_enc_r(sim._h))

    # At 200 mm/s with PID ramp-up, expect at least 50 mm per wheel in 2.4 s.
    assert enc_l >= 50.0, f"Expected enc_l >= 50 mm after VW 200 for 2.4s, got {enc_l:.2f}"
    assert enc_r >= 50.0, f"Expected enc_r >= 50 mm after VW 200 for 2.4s, got {enc_r:.2f}"


def test_vw_keepalive_timeout_stops_motor(sim):
    """VW with no keepalive for > sTimeoutMs should emit EVT safety_stop.

    The MotionCommand TIME stop fires when the elapsed simulated time (from
    sim_tick now_ms) minus the command-start time (from systemTime() at issue)
    exceeds sTimeoutMs.  In practice the real-time overhead is small (<< 500 ms),
    so the simulated 2 s window reliably triggers the timeout.
    """
    # Use the real default sTimeout (500 ms) for this test.
    sim.send_command("SET sTimeout=500")

    sim.send_command("VW 200 0")

    # Tick for 2 s simulated — well beyond 500 ms sTimeoutMs.
    sim.tick_for(2000)

    evts = sim.get_async_evts()
    assert "EVT safety_stop" in evts, (
        f"Expected 'EVT safety_stop' after keepalive timeout, got {repr(evts)}"
    )
    pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
    assert pwm_l == 0.0, f"Expected motor stopped after safety_stop, pwm_l={pwm_l}"
