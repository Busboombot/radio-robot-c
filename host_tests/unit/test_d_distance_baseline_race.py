"""
test_d_distance_baseline_race.py — 033-004 regression test.

A `D` command following a `TURN` (with no `ZERO enc` between them) must NOT
instant-complete with zero motion.

Root cause (fixed): MotionController::beginDistance() snapshotted the DISTANCE
baseline (enc0) from state.inputs.encLMm/R while those still held the PREVIOUS
command's accumulated encoder average — Robot::distanceDrive() only zeroed the
mirror AFTER beginDistance() returned.  On the first evaluate the DISTANCE stop
computed traveled = |0 - enc0| (enc0 ~= the stale prior average); if that
exceeded the new target, the stop fired immediately and the robot never moved.

The fix zeroes the encoder mirror inside beginDistance() BEFORE the baseline
snapshot, so enc0 starts at 0 regardless of the prior command's residue.
"""

from firmware import Sim


def test_d_after_turn_does_not_instant_complete():
    """D after a TURN (no ZERO enc) drives the full distance, not instant-stop."""
    with Sim() as s:
        # --- D1: drive 250 mm so the encoders accumulate a large baseline. ---
        s.send_command("D 200 200 250")
        s.tick_for(10000)

        # --- TURN in place to 90 deg.  TURN does NOT reset encoders (only D
        #     does), so the accumulated baseline survives into the next D. ---
        s.send_command("TURN 9000")
        s.tick_for(6000)

        # Precondition: the encoder average that the bug would capture as enc0 is
        # large (>> the D2 target).  If this fails the test setup is wrong, not
        # the fix.
        encL = float(s._lib.sim_get_enc_l(s._h))
        encR = float(s._lib.sim_get_enc_r(s._h))
        stale_avg = (encL + encR) * 0.5
        assert abs(stale_avg) >= 120.0, (
            f"test setup: expected accumulated |avg enc| >= 120 before D2, "
            f"got {stale_avg:.1f} (L={encL:.1f}, R={encR:.1f})"
        )

        # Clear D1/TURN events so the next read sees only D2's events.
        s.get_async_evts()

        # --- D2: target 100 mm, strictly less than the stale baseline. ---
        s.send_command("D 200 200 100")

        # 100 mm at 200 mm/s needs ~0.5 s + ramp; 300 ms is far too short to
        # finish legitimately.  With the bug, EVT done D fires on the first
        # evaluate (within one tick).
        s.tick_for(300)
        evts_early = s.get_async_evts()
        assert "EVT done D" not in evts_early, (
            f"D2 instant-completed (baseline race): EVT done D within 300 ms "
            f"before it could travel 100 mm. evts={evts_early!r}"
        )

        # --- Let D2 finish legitimately; it must travel ~the full 100 mm. ---
        s.tick_for(10000)
        evts_done = s.get_async_evts()
        assert "EVT done D" in evts_done, f"D2 never completed: {evts_done!r}"

        # D2's own resetEncoderAccumulators zeroed the hardware encoders at its
        # start, so these reflect D2's travel only.
        encL2 = float(s._lib.sim_get_enc_l(s._h))
        encR2 = float(s._lib.sim_get_enc_r(s._h))
        travel = (encL2 + encR2) * 0.5
        assert travel >= 80.0, (
            f"D2 traveled only {travel:.1f} mm (expected ~100); baseline race "
            f"would leave it near 0 (instant stop)."
        )


def test_d_after_d_does_not_instant_complete():
    """Same race via D -> D (no intervening ZERO enc): the simplest reproduction."""
    with Sim() as s:
        s.send_command("D 200 200 250")
        s.tick_for(10000)
        # After D1, state.inputs holds ~250 mm (the stale baseline).
        s.get_async_evts()

        s.send_command("D 200 200 100")
        s.tick_for(300)
        evts_early = s.get_async_evts()
        assert "EVT done D" not in evts_early, (
            f"second D instant-completed (baseline race): EVT done D within "
            f"300 ms. evts={evts_early!r}"
        )

        s.tick_for(10000)
        assert "EVT done D" in s.get_async_evts()
        travel = (float(s._lib.sim_get_enc_l(s._h))
                  + float(s._lib.sim_get_enc_r(s._h))) * 0.5
        assert travel >= 80.0, (
            f"second D traveled only {travel:.1f} mm (expected ~100)."
        )
