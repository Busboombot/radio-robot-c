"""
test_motor_controller.py — tests for the velocity PID and motor controller.

The sim runs a closed-loop velocity PID (MotorController) that drives MockMotor.
All tests drive via the S command (set wheel speeds in mm/s) and observe the
PWM and encoder output.
"""
import ctypes


def test_pwm_nonzero_at_target_speed(sim):
    """After commanding S 200 200 and settling, PWM is nonzero (motor running)."""
    # Disable the system watchdog for this motor-behavior test — the watchdog
    # is tested separately in test_motion_controller.py.
    sim.send_command("SET sTimeout=30000")
    sim.send_command("S 200 200 9000")
    # Let the PID settle for 2 s.
    sim.tick_for(2000)
    pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
    pwm_r = float(sim._lib.sim_get_pwm_r(sim._h))
    assert pwm_l != 0.0, f"Expected nonzero PWM_L at 200 mm/s target, got {pwm_l}"
    assert pwm_r != 0.0, f"Expected nonzero PWM_R at 200 mm/s target, got {pwm_r}"


def test_encoder_grows_at_target_speed(sim):
    """With 200 mm/s target, encoder accumulates over 2 s."""
    # Disable the system watchdog for this motor-behavior test.
    sim.send_command("SET sTimeout=30000")
    sim.send_command("S 200 200 9000")
    sim.tick_for(2000)
    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    # At 200 mm/s for 2 s = 400 mm nominal.  The PID ramps up slowly so
    # accept >= 100 mm (generous lower bound — confirms motor is actually moving).
    assert enc_l >= 100.0, (
        f"Expected enc_l >= 100 mm after 2 s at 200 mm/s, got {enc_l:.2f}"
    )


def test_integral_windup_clamped(sim):
    """With encoder forced to zero, integrator saturates but does not overflow.

    velIMax = 20.0 PWM%.  The total output is kFF*setpoint + kP*err + iMax,
    but the hardware clamp bounds the result to [-100, +100].  The test confirms
    the pwm stays finite (no NaN/Inf) and within the hardware range.
    """
    # Disable the system watchdog for this PID-behavior test.
    sim.send_command("SET sTimeout=30000")
    sim.send_command("S 400 400 9000")

    # Tick for 2 s, forcibly zeroing the encoder after each step so the
    # integrator accumulates error continuously.
    for _ in range(84):  # 84 * 24 ms ≈ 2 s
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += 24
        sim._lib.sim_set_enc_l(sim._h, ctypes.c_float(0.0))
        sim._lib.sim_set_enc_r(sim._h, ctypes.c_float(0.0))

    pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
    # PWM must be within the hardware range and not NaN.
    assert pwm_l == pwm_l, f"PWM is NaN after windup test"
    assert -100.0 <= pwm_l <= 100.0, (
        f"PWM out of hardware range: {pwm_l:.2f}"
    )


def test_stop_zeroes_pwm(sim):
    """After X (cancel) command, PWM reaches 0 within one tick."""
    # Disable the system watchdog so tick_for(500) doesn't trigger it.
    sim.send_command("SET sTimeout=30000")
    sim.send_command("S 400 400 9000")
    sim.tick_for(500)  # Build up motion

    r = sim.send_command("X")
    assert "OK" in r.upper(), f"Expected OK from X, got {repr(r)}"

    # One more tick to let the stop propagate.
    sim.tick_for(24)
    pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
    assert pwm_l == 0.0, f"Expected PWM_L = 0 after X (stop), got {pwm_l:.2f}"
