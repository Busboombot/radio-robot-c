"""
test_ekf_encoder_velocity_unconditional.py — 033-003 regression tests.

fusedV / fusedOmega were permanently 0 whenever OTOS was not fusing (lifted
bench stand, real-world dropout, or — as in the default sim — OTOS simply not
enabled).  Root cause: the EKF velocity states (v, omega) are a random walk in
predict(); they were only ever set by updateVelocity(), which was only called
inside correctEKF() — the OTOS-gated path.  No OTOS → no velocity observation →
twist reads 0 even while the wheels turn.

Fix: Odometry::predict() fuses encoder-derived velocity into the EKF every tick,
unconditionally.  correctEKF() now fuses only the OTOS observations.

The default Sim does NOT enable OTOS fusion (test_n8_n9 has to enable it
explicitly), so a plain drive here exercises exactly the OTOS-invalid path.
"""

from firmware import Sim


def test_fusedv_nonzero_while_driving_without_otos():
    """A straight drive with no OTOS fusion still produces a nonzero fusedV."""
    with Sim() as s:
        # Drive straight at 200 mm/s for 1 s.  No OTOS model enabled → correctEKF
        # never runs → only predict() can keep the velocity state alive.
        s.send_command("T 200 200 3000")
        s.tick_for(1000)
        fused_v = s.get_fused_v()
        assert fused_v > 50.0, (
            f"fusedV stuck near 0 while driving 200 mm/s with OTOS off "
            f"(got {fused_v:.1f}); encoder velocity must fuse in predict() "
            f"regardless of OTOS health (033-003)."
        )


def test_fusedomega_nonzero_while_spinning_without_otos():
    """An in-place spin with no OTOS fusion still produces a nonzero fusedOmega."""
    with Sim() as s:
        # Spin in place: left +200, right -200 → large yaw rate, ~zero linear.
        s.send_command("T 200 -200 3000")
        s.tick_for(1000)
        fused_omega = s.get_fused_omega()
        assert abs(fused_omega) > 0.3, (
            f"fusedOmega stuck near 0 while spinning with OTOS off "
            f"(got {fused_omega:.3f} rad/s); encoder yaw rate must fuse in "
            f"predict() (033-003)."
        )


def test_enc_omega_suppressed_when_wedged():
    """With the encoder-omega health gate OFF, fusedOmega is suppressed.

    Simulates a wedged wheel (033-005 will drive this gate from the wedge
    detector): predict() must fuse omega_obs = 0 so a frozen encoder cannot
    inject phantom yaw rate into the fused velocity state.
    """
    with Sim() as s:
        # Baseline: healthy spin → large fusedOmega.
        s.send_command("T 200 -200 3000")
        s.tick_for(1000)
        omega_healthy = s.get_fused_omega()
        assert abs(omega_healthy) > 0.3, (
            f"setup: expected large fusedOmega when healthy, got {omega_healthy:.3f}"
        )

    with Sim() as s:
        # Same spin, but the encoder-omega gate is OFF the whole time.
        s.set_enc_omega_healthy(False)
        s.send_command("T 200 -200 3000")
        s.tick_for(1000)
        omega_wedged = s.get_fused_omega()
        assert abs(omega_wedged) < 0.1, (
            f"fusedOmega not suppressed with the health gate off "
            f"(got {omega_wedged:.3f} rad/s); a wedged encoder must not inject "
            f"phantom omega (033-003)."
        )


def test_fusedv_still_fuses_when_omega_suppressed():
    """Suppressing omega must NOT suppress linear velocity fusion."""
    with Sim() as s:
        s.set_enc_omega_healthy(False)
        # Straight drive: linear v should still fuse even with the omega gate off.
        s.send_command("T 200 200 3000")
        s.tick_for(1000)
        fused_v = s.get_fused_v()
        assert fused_v > 50.0, (
            f"fusedV suppressed along with omega (got {fused_v:.1f}); the gate "
            f"must only affect the yaw-rate observation, not linear v."
        )
