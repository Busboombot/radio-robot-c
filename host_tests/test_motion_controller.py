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
    # Disable the system watchdog so the encoder-accumulation test isn't cut off.
    sim.send_command("SET sTimeout=30000")
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


def test_raw_vw_command_no_ramp(sim):
    """_VW immediately seeds BVC current state — no trapezoid ramp from zero.

    After _VW 300 0, the BVC seeds current speed to 300 mm/s and sets target.
    Because there is no ramp from zero, the motor target is applied from the
    very first driveAdvance tick, so the encoders accumulate much faster than
    they would with a VW 300 0 command starting from zero.

    Compare: after 300 ms, _VW 300 0 should reach near-target speed, while
    VW 300 0 would still be ramping.  We verify _VW drives encoders in 300 ms.
    """
    sim.send_command("SET sTimeout=30000")

    r = sim.send_command("_VW 300 0")
    assert "OK" in r.upper(), f"Expected OK from _VW command, got {repr(r)}"

    # Tick for 300 ms.  Motor ramp-up and PID convergence take a few ticks;
    # 300 ms at 300 mm/s yields ~90 mm per wheel at full speed.
    # We expect at least 5 mm to confirm the seed took effect.
    sim.tick_for(300)

    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r = float(sim._lib.sim_get_enc_r(sim._h))

    assert enc_l >= 5.0 and enc_r >= 5.0, (
        f"Expected encoder to grow after _VW 300 0 in 300 ms, "
        f"got enc_l={enc_l:.3f}, enc_r={enc_r:.3f}"
    )


def test_plus_keepalive_is_quiet(sim):
    """+ command (quiet keepalive, sprint 024-003) produces no reply and no motion.

    After sprint 024-003, the '+' keepalive handler no longer emits 'OK keepalive'.
    At 6.7 Hz the acks competed with TLM frames for the 250-byte TX buffer; the
    host already filtered them so suppressing them on the firmware side is safe.
    The watchdog is still reset by the command (via sim_command's watchdogMs update).
    """
    r = sim.send_command("+")
    # Reply must be empty (quiet keepalive).
    assert r.strip() == "", (
        f"Expected empty reply from '+' (quiet keepalive), got {repr(r)}"
    )

    # Encoders should not have moved.
    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r = float(sim._lib.sim_get_enc_r(sim._h))
    assert enc_l == 0.0 and enc_r == 0.0, (
        f"Expected no motion after +, got enc_l={enc_l}, enc_r={enc_r}"
    )


def test_x_soft_ramps_to_zero_and_emits_done(sim):
    """X soft ramps BVC to zero and emits EVT done (via open-ended VW → soft stop).

    Send VW to start open-ended motion, then X soft.  The BVC should ramp to
    zero under aMax.  The MotionCommand SOFT ramp-down should fire EVT done.
    """
    sim.send_command("SET sTimeout=30000")
    sim.send_command("VW 200 0")

    # Let the robot accelerate for 500 ms.
    sim.tick_for(500)

    r = sim.send_command("X soft")
    assert "OK" in r.upper(), f"Expected OK from X soft, got {repr(r)}"

    # Tick up to 4 s for the ramp-down and EVT done to fire.
    sim.tick_for(4000)

    # Motor should have ramped to zero.
    pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
    pwm_r = float(sim._lib.sim_get_pwm_r(sim._h))
    assert pwm_l == 0.0 and pwm_r == 0.0, (
        f"Expected motors stopped after X soft, got pwm_l={pwm_l}, pwm_r={pwm_r}"
    )

    # EVT done should have been emitted.
    evts = sim.get_async_evts()
    assert "EVT done" in evts, (
        f"Expected 'EVT done' after X soft ramp-down, got {repr(evts)}"
    )


def test_x_hard_stops_immediately(sim):
    """Hard X (no suffix) still stops immediately — unchanged behavior."""
    sim.send_command("SET sTimeout=30000")
    sim.send_command("VW 200 0")
    sim.tick_for(500)

    r = sim.send_command("X")
    assert "OK" in r.upper(), f"Expected OK from X, got {repr(r)}"

    # After hard stop, PWM should go to 0 within one tick.
    sim.tick_for(24)
    pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
    assert pwm_l == 0.0, f"Expected PWM 0 immediately after X, got pwm_l={pwm_l}"
