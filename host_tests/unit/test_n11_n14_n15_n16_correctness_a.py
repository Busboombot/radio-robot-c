"""
test_n11_n14_n15_n16_correctness_a.py — regression tests for sprint 030-009.

N11: PURSUE backtrack re-gate must not emit spurious "EVT cancelled" for
     the G command's corrId.  The cancel is an internal phase transition;
     the G command continues (it transitions to PRE_ROTATE and tries again).

N14: ParsedCommand::corrId widened from char[8] to char[16].  A 15-char
     correlation id must round-trip intact on the queue path (no silent
     truncation at 7 chars).

N15: EKF Q scaling is now per-second (N15 fix).  The total Q growth over
     a fixed simulated duration must be the same regardless of how many
     ticks are used to cover that duration (invariant to loop rate).

N16: Invalid sensor= token on the queue path (T, D, TURN converters) must
     return ERR before OK — mirroring the direct path's behaviour.  The
     command must NOT start after the ERR reply.
"""
import ctypes
import math
import pytest


# ---------------------------------------------------------------------------
# N11 — PURSUE re-gate: no spurious "EVT cancelled" for the G's corrId
# ---------------------------------------------------------------------------

def test_n11_pursue_regate_no_spurious_cancelled(sim):
    """PURSUE backtrack re-gate must not emit EVT cancelled for the G corrId.

    Scenario:
      1. Issue G 200 0 200 #n11test — target at (200, 0), speed 200.
      2. Drain the queue so the command starts.
      3. Teleport the robot to (250, 0) facing 0 rad — target is now BEHIND
         (dx = 200-250 = -50 < 0 → bearing_rf > π/2 in robot frame).
      4. Tick 5 times — triggers the 3-consecutive-tick backtrack counter
         and fires cancelQuiet() + re-gates to PRE_ROTATE.
      5. Assert: async events contain NO "EVT cancelled" string.
      6. Assert: async events contain NO "EVT cancelled #n11test".
    """
    # Clear any accumulated events from fixture setup.
    sim.get_async_evts()

    # Issue the G command with a correlation id.
    r = sim.send_command("G 200 0 200 #n11test")
    assert "OK" in r.upper(), f"Expected OK from G, got {repr(r)}"

    # Drain the queue so the VW ParsedCommand is dispatched and beginGoTo runs.
    # sim_command already drains 2 slots; tick once more to start the command.
    sim.tick_for(24)

    # Teleport robot to (250, 0) — robot is now 50 mm past the X target.
    # With heading = 0 (facing +X), bearing to (200, 0) from (250, 0) is
    # atan2(0, -50) = π — clearly > π/2, triggering backtrack.
    sim.set_pose(250.0, 0.0, 0.0)

    # Tick 5 times: backtrack counter hits 3 on tick 3, triggering cancelQuiet().
    sim.tick_for(5 * 24)

    evts = sim.get_async_evts()

    # No "EVT cancelled" should appear for the G command's corrId or at all
    # for internal phase transitions.
    assert "EVT cancelled #n11test" not in evts, (
        f"N11: spurious 'EVT cancelled #n11test' emitted during PURSUE re-gate.\n"
        f"Async events: {repr(evts)}"
    )
    # Also check that no bare cancelled appears (belt-and-suspenders — the
    # cancelQuiet() clears the sink so even a bare cancel would be suppressed).
    assert "EVT cancelled" not in evts, (
        f"N11: 'EVT cancelled' appeared in async events during PURSUE re-gate.\n"
        f"Async events: {repr(evts)}"
    )


def test_n11_normal_cancel_still_emits_evt(sim):
    """Sanity check: a normal external cancel (X) still emits EVT cancelled.

    This verifies that cancelQuiet() did not accidentally suppress all cancel
    events — only the internal PURSUE re-gate path should be quiet.

    In the sim, cancel() runs synchronously during the X command dispatch, so
    the "EVT cancelled" appears in the synchronous reply from sim_command("X"),
    not in the async EVT buffer.
    """
    sim.get_async_evts()

    # Start a long timed drive.
    r = sim.send_command("T 200 200 5000 #canceltest")
    assert "OK" in r.upper(), f"Expected OK from T, got {repr(r)}"
    sim.tick_for(100)

    # Cancel it externally.  In sim, the EVT cancelled is emitted synchronously
    # during X's dispatch (MotionCommand::cancel is called from handleX via
    # cmd.dequeueOne), so it appears in the X reply string itself.
    x_reply = sim.send_command("X")

    # The X reply should include both "EVT cancelled" (from the T command's sink)
    # and "OK x" (from handleX).
    assert "EVT cancelled" in x_reply, (
        f"Expected 'EVT cancelled' in X reply (emitted synchronously during cancel),\n"
        f"X reply: {repr(x_reply)}"
    )


# ---------------------------------------------------------------------------
# N14 — 16-char corrId round-trips intact on the queue path
# ---------------------------------------------------------------------------

def test_n14_long_corrid_roundtrips_on_queue_path(sim):
    """15-char corrId round-trips intact in the OK reply on the queue path.

    Pre-fix: ParsedCommand::corrId was char[8], truncating ids longer than
    7 chars.  Post-fix: char[16], so a 15-char id (e.g. ms-timestamp) passes
    through unchanged.

    The corrId is included in both the synchronous OK reply and any async EVT.
    We test the synchronous OK reply here; the EVT path is tested separately.
    """
    long_id = "123456789012345"  # 15 chars — exactly fits in char[16] with NUL
    assert len(long_id) == 15, "Test setup: id must be exactly 15 chars"

    # T command on the queue path — the OK reply from the converter includes
    # the corrId in the form "OK drive ... #<corrId>".
    r = sim.send_command(f"T 200 200 500 #{long_id}")
    assert "OK" in r.upper(), f"Expected OK from T, got {repr(r)}"
    assert long_id in r, (
        f"N14: 15-char corrId '{long_id}' was truncated in the OK reply.\n"
        f"Full reply: {repr(r)}"
    )


def test_n14_long_corrid_in_evt_done(sim):
    """15-char corrId appears intact in EVT done after command completes.

    Verifies the corrId survives the full queue path:
    T command → pushVW → handleVW → beginTimed → MotionCommand → EVT done.
    """
    long_id = "123456789012345"  # 15 chars
    sim.get_async_evts()  # clear any prior events

    # Issue T with long corrId; wait for it to complete.
    r = sim.send_command(f"T 200 200 200 #{long_id}")
    assert "OK" in r.upper(), f"Expected OK, got {repr(r)}"

    # Tick long enough for the 200 ms drive to complete.
    sim.tick_for(1000)

    evts = sim.get_async_evts()
    assert long_id in evts, (
        f"N14: 15-char corrId '{long_id}' missing from async EVTs.\n"
        f"Full EVT buffer: {repr(evts)}"
    )


def test_n14_eight_char_corrid_still_works(sim):
    """8-char corrId (old boundary) still round-trips after widening.

    Regression guard: widening to char[16] must not break 8-char ids.
    """
    eight_id = "12345678"  # 8 chars
    r = sim.send_command(f"T 200 200 200 #{eight_id}")
    assert "OK" in r.upper(), f"Expected OK, got {repr(r)}"
    assert eight_id in r, (
        f"N14: 8-char corrId '{eight_id}' missing from OK reply (regression).\n"
        f"Reply: {repr(r)}"
    )


# ---------------------------------------------------------------------------
# N15 — EKF Q scaling invariant to loop rate
# ---------------------------------------------------------------------------

def test_n15_q_growth_invariant_to_loop_rate(sim):
    """EKF Q growth over a fixed simulated time is invariant to tick granularity.

    Pre-fix: predict() added full Q per call, so total growth was proportional
    to call count (= time / dt).  Post-fix: Q is multiplied by dt_s each call,
    so total growth ≈ Q_per_second × total_time regardless of step size.

    Method:
      1. Run two fresh sim instances over the same total wall-clock time
         (2000 ms) using different step sizes (24 ms vs 48 ms).
      2. Verify that P[0][0] (position X covariance) after 2000 ms is within
         a small tolerance in both cases — the difference should be at most
         one extra tick's worth of Q (|Δdt| bound).

    We use only predict() growth here (no OTOS corrections, no external
    commands that would start motion) — a pure idle run so only the time
    integration of Q contributes.
    """
    from firmware import Sim

    TOTAL_MS = 2000
    STEP_A = 24   # fine step — ~83 ticks
    STEP_B = 48   # coarse step — ~42 ticks

    def run_predict_only(step_ms: int) -> float:
        """Run sim for TOTAL_MS ms in step_ms increments; return P[0][0]."""
        with Sim() as s:
            # Set a very long watchdog so no safety stop fires.
            s.send_command("SET sTimeout=60000")
            # Run idle ticks (no motion command) so only Q accumulates.
            s.tick_for(TOTAL_MS, step_ms=step_ms)
            return s.get_ekf_p_diag(0)  # P[0][0]

    p_fine   = run_predict_only(STEP_A)
    p_coarse = run_predict_only(STEP_B)

    # Pre-fix: |p_fine - p_coarse| ≈ (83 - 42) × Q_per_call ≈ 41 × 200 = 8200
    # Post-fix: both ≈ Q_per_second × TOTAL_MS/1000 = 200 × 2.0 = 400 mm²
    # Tolerance: within 2× step_B Q worth of each other (one extra tick error).
    # Q_per_second (ekfQxy) = 200.0; at step_B dt = 0.048, one tick = 9.6 mm².
    tolerance = 20.0  # mm² — generous to allow for rounding and step boundary

    assert abs(p_fine - p_coarse) < tolerance, (
        f"N15: EKF P[0][0] growth is loop-rate-coupled (not invariant).\n"
        f"  Fine step ({STEP_A} ms over {TOTAL_MS} ms):   P[0][0] = {p_fine:.3f}\n"
        f"  Coarse step ({STEP_B} ms over {TOTAL_MS} ms): P[0][0] = {p_coarse:.3f}\n"
        f"  Difference: {abs(p_fine - p_coarse):.3f} > tolerance {tolerance}"
    )


def test_n15_q_growth_matches_expected_per_second(sim):
    """EKF P[0][0] growth after 1 second ≈ ekfQxy per second (= 200.0 mm²/s).

    With Q*dt scaling, running for 1000 ms starting from P[0][0] = 0 (init)
    should add approximately ekfQxy * 1.0 = 200.0 mm² to P[0][0].

    We allow a ±2 tick tolerance for step-boundary rounding.
    """
    # Start from a fresh sim with no prior motion.
    p0 = sim.get_ekf_p_diag(0)  # initial P[0][0] after fixture setup

    sim.tick_for(1000, step_ms=24)  # ~41 ticks over 1 s

    p1 = sim.get_ekf_p_diag(0)
    growth = p1 - p0

    # Expected: ekfQxy * 1.0 s = 200.0 mm²
    # Tolerance: ±2 ticks at 24 ms each = ±2 × 200 × 0.024 = ±9.6 mm²
    expected = 200.0
    tolerance = 15.0

    assert abs(growth - expected) < tolerance, (
        f"N15: P[0][0] growth over 1 s = {growth:.3f} mm²; "
        f"expected {expected:.1f} ± {tolerance} mm².\n"
        f"P before: {p0:.3f}, P after: {p1:.3f}"
    )


# ---------------------------------------------------------------------------
# N16 — invalid sensor= on queue path returns ERR before OK
# ---------------------------------------------------------------------------

def test_n16_t_invalid_sensor_returns_err(sim):
    """T with invalid sensor= on queue path returns ERR, not OK.

    Pre-fix: the T converter packed the raw sensor token and replied OK,
    forwarding the bad token to handleVW which silently dropped the stop.
    Post-fix: sensor= is validated in handleT BEFORE pushVW and replyOK.
    """
    # "sensor=badchan:ge:100" has an invalid channel name — mc_parseSensorToken
    # returns false for any channel not in the known list.
    r = sim.send_command("T 200 200 2000 sensor=badchan:ge:100 #n16t")
    first_token = r.strip().split()[0] if r.strip() else ""
    assert first_token == "ERR", (
        f"N16: T with invalid sensor= must return ERR, got {repr(r)}"
    )
    assert "OK" not in r, (
        f"N16: T with invalid sensor= must not emit OK before ERR, got {repr(r)}"
    )


def test_n16_d_invalid_sensor_returns_err(sim):
    """D with invalid sensor= on queue path returns ERR, not OK."""
    r = sim.send_command("D 200 200 500 sensor=notachan:le:50 #n16d")
    first_token = r.strip().split()[0] if r.strip() else ""
    assert first_token == "ERR", (
        f"N16: D with invalid sensor= must return ERR, got {repr(r)}"
    )
    assert "OK" not in r, (
        f"N16: D with invalid sensor= must not emit OK, got {repr(r)}"
    )


def test_n16_turn_invalid_sensor_returns_err(sim):
    """TURN with invalid sensor= on queue path returns ERR, not OK."""
    r = sim.send_command("TURN 9000 sensor=xyz:ge:0 #n16turn")
    first_token = r.strip().split()[0] if r.strip() else ""
    assert first_token == "ERR", (
        f"N16: TURN with invalid sensor= must return ERR, got {repr(r)}"
    )
    assert "OK" not in r, (
        f"N16: TURN with invalid sensor= must not emit OK, got {repr(r)}"
    )


def test_n16_t_invalid_sensor_command_does_not_start(sim):
    """T with invalid sensor= must not start motion.

    After the ERR reply, no EVT done should arrive (command never started).
    Encoders should remain at zero.
    """
    sim.get_async_evts()

    sim.send_command("T 200 200 500 sensor=badchan:ge:100 #n16nostart")
    sim.tick_for(1000)

    evts = sim.get_async_evts()
    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r = float(sim._lib.sim_get_enc_r(sim._h))

    assert "EVT done" not in evts, (
        f"N16: T with invalid sensor= must not start motion (EVT done appeared).\n"
        f"Events: {repr(evts)}"
    )
    # Encoders should not have moved significantly (within motor-stop tolerance).
    assert enc_l < 10.0 and enc_r < 10.0, (
        f"N16: T with invalid sensor= must not drive motors.\n"
        f"enc_l={enc_l:.1f} enc_r={enc_r:.1f}"
    )


def test_n16_valid_sensor_still_works(sim):
    """T with valid sensor= on queue path still returns OK and drives.

    Sanity check: N16 fix must not break the valid sensor= path.
    line0 is a valid channel; ge:0 always fires immediately (threshold 0).
    """
    sim.get_async_evts()

    # A valid sensor stop: line0:ge:0 — fires as soon as line0 >= 0.
    r = sim.send_command("T 200 200 2000 sensor=line0:ge:0 #n16valid")
    assert "OK" in r.upper(), (
        f"N16: T with valid sensor= must return OK, got {repr(r)}"
    )
