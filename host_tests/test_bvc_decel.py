"""
test_bvc_decel.py — regression tests for BVC deceleration quality.

These tests catch the "dogleg" / velocity plateau at the start of SOFT
ramp-down, caused by _v overrunning the saturation ceiling during cruise.
"""
import ctypes


def _collect_velocity(sim, duration_ms: int, step_ms: int = 24) -> list[float]:
    """Tick sim for duration_ms and return per-tick vel_l samples."""
    samples = []
    end_t = sim._t + duration_ms
    while sim._t < end_t:
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += step_ms
        samples.append(float(sim._lib.sim_get_vel_l(sim._h)))
    return samples


def _find_plateau(samples: list[float],
                  flat_threshold: float = 1.0,
                  min_vel: float = 15.0,
                  run_len: int = 3) -> tuple[int, list[float]] | None:
    """Return (start_index, window) of the first plateau, or None."""
    run = 0
    for i in range(1, len(samples)):
        if samples[i - 1] > min_vel and (samples[i - 1] - samples[i]) < flat_threshold:
            run += 1
        else:
            run = 0
        if run >= run_len:
            start = max(0, i - run - 1)
            return start, samples[start: i + 3]
    return None


def test_straight_decel_no_plateau(sim):
    """After T 200 200 1200, velocity descends without flat spot."""
    sim.send_command("T 200 200 1200")

    # Cruise phase — tick until just before stop fires
    _collect_velocity(sim, 1200)

    # Decel phase — should ramp cleanly to zero
    decel = _collect_velocity(sim, 1500)

    result = _find_plateau(decel)
    if result is not None:
        idx, window = result
        pretty = [f"{v:.1f}" for v in window]
        assert False, (
            f"Velocity plateau during straight decel at sample {idx}: "
            f"{pretty} mm/s"
        )


def test_turn_decel_no_plateau(sim):
    """After T -200 200 600 (90-deg turn), velocity descends without flat spot."""
    sim.send_command("T -200 200 600")

    _collect_velocity(sim, 600)

    # For turns, watch the right-wheel velocity (positive)
    decel = []
    end_t = sim._t + 1500
    step = 24
    while sim._t < end_t:
        sim._lib.sim_tick(sim._h, ctypes.c_uint32(sim._t))
        sim._t += step
        decel.append(float(sim._lib.sim_get_vel_r(sim._h)))

    result = _find_plateau(decel)
    if result is not None:
        idx, window = result
        pretty = [f"{v:.1f}" for v in window]
        assert False, (
            f"Velocity plateau during turn decel at sample {idx}: "
            f"{pretty} mm/s"
        )
