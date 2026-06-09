"""
test_mock_hal.py — tests for MockMotor physics and encoder injection.

MockMotor integrates commanded speed into encoder mm on each tick:
    encoderMm += (cmdSpeed / 100.0) * kNominalMaxMms * offsetFactor * (dt_ms / 1000.0)
where kNominalMaxMms = 400.0 mm/s.

Motor speed is set via the S command (left_mms right_mms dist_mm).  We use
VW (body velocity + angular velocity) to drive at a known forward speed with
no rotation, which sets both wheels at the requested body speed.
"""
import ctypes

# kNominalMaxMms from MockMotor.h
_NOMINAL_MAX_MMS = 400.0


def test_motor_forward_accumulates(sim):
    """At full forward speed (400 mm/s), encoder grows after 1 s of ticks.

    The PID velocity controller ramps up over the first few hundred ms, so the
    encoder won't reach the theoretical 400mm after 1 s.  We assert >= 70% of
    the nominal max to confirm the motor is actually running (not stuck at 0).
    """
    # S command: left_mms right_mms — use very large dist_mm so it
    # doesn't stop early.  S takes mm/s values for L and R directly.
    # Drive both wheels at ~400 mm/s forward for 1 second.
    reply = sim.send_command("S 400 400 9000")
    assert "OK" in reply.upper() or reply == ""  # some builds emit no OK on S

    sim.tick_for(1000)

    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    threshold = 0.7 * _NOMINAL_MAX_MMS * 1.0
    assert enc_l >= threshold, (
        f"Expected enc_l >= {threshold:.1f} mm after 1 s "
        f"at full speed, got {enc_l:.2f} mm"
    )


def test_motor_zero_speed_stable(sim):
    """At 0% speed (no command issued), encoder stays near zero after 1 s."""
    sim.tick_for(1000)
    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    assert abs(enc_l) < 1.0, (
        f"Encoder should be ~0 at zero speed, got {enc_l:.4f} mm"
    )


def test_motor_reverse_decreases(sim):
    """At -400 mm/s both wheels, encoder decreases after 1 s of ticks.

    Same PID ramp-up applies; use 70% threshold (same reasoning as forward test).
    """
    # Drive both wheels in reverse.
    sim.send_command("S -400 -400 9000")
    sim.tick_for(1000)
    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    threshold = -0.7 * _NOMINAL_MAX_MMS * 1.0
    assert enc_l <= threshold, (
        f"Expected enc_l <= {threshold:.1f} mm in reverse, "
        f"got {enc_l:.2f} mm"
    )


def test_encoder_injection(sim):
    """sim_set_enc_l injects an encoder value into Robot state."""
    # Inject a known position into the left encoder.
    sim._lib.sim_set_enc_l(sim._h, ctypes.c_float(123.4))

    # The injection writes directly to robot.state.inputs.encLMm.
    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    assert abs(enc_l - 123.4) < 0.1, (
        f"Expected enc_l ≈ 123.4 after injection, got {enc_l:.4f}"
    )
