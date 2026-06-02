"""Unit tests for robot_radio.robot.clock_sync.ClockSync.

All tests are fully offline — no serial port, no hardware.
We inject a fake clock and a fake send_fn so the math is deterministic.

Test plan:
- Min-RTT selection: five samples with distinct RTTs; best offset uses min-RTT.
- Offset math: known T0, T1, t_r → expected offset.
- Equal-RTT samples: any selection is valid (first is fine).
- ping_burst: fake send_fn returning canned pong lines; verify samples recorded.
- ping_burst partial failure: timed-out pings are skipped; surviving sample used.
- ping_burst all failure: prior estimate left unchanged.
- Skew regression: samples with known a/b; recovered coefficients within tolerance.
- to_host_time: with and without skew; None before calibration.
- stale(): True before any burst; False immediately after; True after elapsed.
- reset(): clears all state back to uncalibrated.
- _parse_pong_t helper: various valid/invalid input strings.
"""

from __future__ import annotations

import math
from typing import Iterator

import pytest

from robot_radio.robot.clock_sync import ClockSync, _parse_pong_t


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class FakeClock:
    """Controllable monotonic clock for test injection.

    ``advance(dt_s)`` moves the clock forward by dt_s seconds.
    The clock is callable and returns the current time in seconds.
    """

    def __init__(self, start: float = 1_000_000.0) -> None:
        # Start at a large value so host_ms is clearly distinct from robot_ms.
        self._t = start

    def __call__(self) -> float:
        return self._t

    def advance(self, dt_s: float) -> None:
        self._t += dt_s

    def advance_ms(self, dt_ms: float) -> None:
        self._t += dt_ms / 1000.0

    @property
    def now_ms(self) -> float:
        return self._t * 1000.0


def _make_pong(t_robot_ms: int) -> str:
    """Return a pong reply line for the given robot timestamp."""
    return f"OK pong t={t_robot_ms}"


def _make_cs(start_s: float = 1_000_000.0) -> tuple[ClockSync, FakeClock]:
    """Return (ClockSync, FakeClock) pair with shared clock."""
    clock = FakeClock(start_s)
    cs = ClockSync(clock_fn=clock)
    return cs, clock


# ---------------------------------------------------------------------------
# Helper: record a synthetic PING sample
# ---------------------------------------------------------------------------

def _record(cs: ClockSync, clock: FakeClock,
            rtt_ms: float, t_robot_ms: float,
            send_delay_ms: float | None = None) -> None:
    """Record one synthetic sample.

    The robot stamp ``t_robot_ms`` is recorded at mid-time, so:
        t0 = clock now
        t1 = t0 + rtt_ms
    We advance the clock by rtt_ms to simulate the round-trip.
    ``send_delay_ms`` is ignored (t_robot_ms is supplied directly).
    """
    t0_ms = clock.now_ms
    clock.advance_ms(rtt_ms)
    t1_ms = clock.now_ms
    cs.record_ping(t0_ms=t0_ms, t1_ms=t1_ms, t_robot_ms=t_robot_ms)


# ===========================================================================
# _parse_pong_t
# ===========================================================================

class TestParsePongT:
    """Unit tests for the module-level _parse_pong_t helper."""

    def test_plain_pong(self) -> None:
        assert _parse_pong_t("OK pong t=12345") == 12345

    def test_relay_prefix(self) -> None:
        assert _parse_pong_t("< OK pong t=99") == 99

    def test_with_corr_id(self) -> None:
        assert _parse_pong_t("OK pong t=500 #7") == 500

    def test_zero(self) -> None:
        assert _parse_pong_t("OK pong t=0") == 0

    def test_large_value(self) -> None:
        assert _parse_pong_t("OK pong t=3600000") == 3600000

    def test_empty_string(self) -> None:
        assert _parse_pong_t("") is None

    def test_no_t_field(self) -> None:
        assert _parse_pong_t("OK pong") is None

    def test_non_integer_t(self) -> None:
        assert _parse_pong_t("OK pong t=abc") is None

    def test_wrong_tag(self) -> None:
        # Even if the line has t=, but it's not a pong format, we still parse t=.
        assert _parse_pong_t("ERR t=12") == 12  # _parse_pong_t looks for any t= token

    def test_bare_t_token(self) -> None:
        assert _parse_pong_t("t=42") == 42


# ===========================================================================
# No-sample state
# ===========================================================================

class TestNoCalibraton:
    """ClockSync with no samples must return None from all output methods."""

    def test_best_offset_ms_none(self) -> None:
        cs, _ = _make_cs()
        assert cs.best_offset_ms() is None

    def test_to_host_time_none(self) -> None:
        cs, _ = _make_cs()
        assert cs.to_host_time(1000) is None

    def test_min_rtt_none(self) -> None:
        cs, _ = _make_cs()
        assert cs.min_rtt_ms is None

    def test_offset_ms_alias_none(self) -> None:
        cs, _ = _make_cs()
        assert cs.offset_ms is None

    def test_skew_none(self) -> None:
        cs, _ = _make_cs()
        assert cs.skew is None

    def test_sample_count_zero(self) -> None:
        cs, _ = _make_cs()
        assert cs.sample_count == 0

    def test_stale_before_any_sync(self) -> None:
        cs, _ = _make_cs()
        assert cs.stale() is True

    def test_stale_zero_threshold(self) -> None:
        cs, _ = _make_cs()
        assert cs.stale(max_age_s=0.0) is True


# ===========================================================================
# Min-RTT selection and offset math
# ===========================================================================

class TestMinRTTSelection:
    """Verify the min-RTT sample is selected and offset is computed correctly."""

    def test_five_samples_min_rtt_selected(self) -> None:
        """Simulate 5 pings with RTTs 80, 60, 40, 50, 70 ms; best is sample 3 (RTT=40)."""
        cs, clock = _make_cs()

        # We define t_robot_ms as the robot stamp at mid-time for each sample.
        # To keep the expected offset easy to compute, set t_robot_ms = mid-time.
        # Then offset = mid_time - t_robot_ms = 0 (trivially).  Instead, use
        # a fixed robot "boot" reference so offset is host_start - robot_zero.
        #
        # Layout (all times in ms relative to clock start):
        # host_clock_start = 1_000_000_000 ms (from FakeClock(1_000_000.0) in seconds)
        HOST_START_MS = 1_000_000.0 * 1000.0  # = 1e9 ms

        rtts = [80.0, 60.0, 40.0, 50.0, 70.0]
        # Robot boot offset: robot runs at same rate but started 5000 ms earlier.
        ROBOT_OFFSET = 5000.0

        for i, rtt in enumerate(rtts):
            t0_ms = clock.now_ms
            # Robot stamp: at mid-time of this exchange,
            # robot_ms = host_mid - ROBOT_OFFSET = (t0 + rtt/2) - ROBOT_OFFSET
            robot_ms = t0_ms + rtt / 2.0 - ROBOT_OFFSET
            clock.advance_ms(rtt)
            t1_ms = clock.now_ms
            cs.record_ping(t0_ms=t0_ms, t1_ms=t1_ms, t_robot_ms=robot_ms)
            clock.advance_ms(10.0)  # small inter-ping gap

        assert cs.sample_count == 5
        assert cs.min_rtt_ms == pytest.approx(40.0, abs=1e-6)

        # Offset should equal ROBOT_OFFSET (host is 5000 ms ahead of robot).
        assert cs.best_offset_ms() == pytest.approx(ROBOT_OFFSET, abs=1e-6)

    def test_single_sample_offset_math(self) -> None:
        """Exact arithmetic check for one sample."""
        cs, clock = _make_cs(start_s=0.0)  # start at time 0 for simplicity

        t0_ms = 100.0
        t1_ms = 140.0   # RTT = 40 ms
        t_robot_ms = 110.0
        # expected: (100+140)/2 - 110 = 120 - 110 = 10.0
        cs.record_ping(t0_ms=t0_ms, t1_ms=t1_ms, t_robot_ms=t_robot_ms)
        assert cs.best_offset_ms() == pytest.approx(10.0)

    def test_min_rtt_updates_on_better_sample(self) -> None:
        """Adding a sample with smaller RTT updates the best."""
        cs, clock = _make_cs(start_s=0.0)

        cs.record_ping(t0_ms=0.0, t1_ms=100.0, t_robot_ms=50.0)  # RTT=100
        assert cs.min_rtt_ms == pytest.approx(100.0)

        cs.record_ping(t0_ms=200.0, t1_ms=240.0, t_robot_ms=220.0)  # RTT=40
        assert cs.min_rtt_ms == pytest.approx(40.0)

    def test_larger_rtt_does_not_replace_best(self) -> None:
        """After the min-RTT sample, a worse sample leaves the best unchanged."""
        cs, clock = _make_cs(start_s=0.0)

        cs.record_ping(t0_ms=0.0, t1_ms=40.0, t_robot_ms=20.0)   # RTT=40  best
        cs.record_ping(t0_ms=100.0, t1_ms=180.0, t_robot_ms=140.0)  # RTT=80 worse

        assert cs.min_rtt_ms == pytest.approx(40.0)

    def test_equal_rtt_keeps_first(self) -> None:
        """When two samples have equal RTT, the first one stays as best."""
        cs, clock = _make_cs(start_s=0.0)

        # Both RTT=50.  First offset=10, second offset=20.
        cs.record_ping(t0_ms=0.0, t1_ms=50.0, t_robot_ms=15.0)
        first_offset = cs.best_offset_ms()

        cs.record_ping(t0_ms=100.0, t1_ms=150.0, t_robot_ms=115.0)
        # Second RTT also 50 → strict < fails → best unchanged.
        assert cs.best_offset_ms() == pytest.approx(first_offset)
        assert cs.min_rtt_ms == pytest.approx(50.0)


# ===========================================================================
# to_host_time (offset-only path)
# ===========================================================================

class TestToHostTimeOffsetOnly:
    """Verify to_host_time() using the simple offset-only model."""

    def test_translation_uses_offset(self) -> None:
        cs, clock = _make_cs(start_s=0.0)
        # offset = 10 ms (host 10 ms ahead of robot)
        cs.record_ping(t0_ms=100.0, t1_ms=140.0, t_robot_ms=110.0)
        # offset = (100+140)/2 - 110 = 10
        assert cs.best_offset_ms() == pytest.approx(10.0)
        assert cs.to_host_time(500) == pytest.approx(510.0)

    def test_translation_negative_offset(self) -> None:
        """Host can be *behind* robot (offset negative)."""
        cs, clock = _make_cs(start_s=0.0)
        # offset = (0 + 40)/2 - 30 = 20 - 30 = -10
        cs.record_ping(t0_ms=0.0, t1_ms=40.0, t_robot_ms=30.0)
        assert cs.to_host_time(100) == pytest.approx(90.0)

    def test_to_host_time_none_before_samples(self) -> None:
        cs, _ = _make_cs()
        assert cs.to_host_time(1234) is None

    def test_translation_zero_robot_time(self) -> None:
        cs, _ = _make_cs(start_s=0.0)
        cs.record_ping(t0_ms=1000.0, t1_ms=1040.0, t_robot_ms=0.0)
        # offset = 1020 - 0 = 1020
        assert cs.to_host_time(0) == pytest.approx(1020.0)


# ===========================================================================
# Skew regression
# ===========================================================================

class TestSkewRegression:
    """Verify linear regression recovers a known skew (a, b)."""

    def _make_samples_with_skew(
        self,
        cs: ClockSync,
        a: float,
        b: float,
        n: int = 10,
        rtt_ms: float = 20.0,
        step_ms: float = 1000.0,
    ) -> None:
        """Inject *n* samples consistent with host_mid = a·t_robot + b.

        For each sample:
            t_robot = i * step_ms
            host_mid = a * t_robot + b
            t0 = host_mid - rtt_ms/2
            t1 = host_mid + rtt_ms/2
        """
        for i in range(n):
            t_r = float(i) * step_ms
            host_mid = a * t_r + b
            t0_ms = host_mid - rtt_ms / 2.0
            t1_ms = host_mid + rtt_ms / 2.0
            cs.record_ping(t0_ms=t0_ms, t1_ms=t1_ms, t_robot_ms=t_r)

    def test_skew_one_recovers_perfect_clock(self) -> None:
        """No drift (a=1): regression should return a≈1, b≈offset."""
        cs, _ = _make_cs()
        OFFSET = 5000.0
        self._make_samples_with_skew(cs, a=1.0, b=OFFSET, n=10)

        assert cs.skew is not None
        assert cs.skew == pytest.approx(1.0, abs=1e-6)

    def test_skew_slightly_fast_robot(self) -> None:
        """Robot clock 50 ppm fast (a=1.00005)."""
        cs, _ = _make_cs()
        A, B = 1.00005, 3000.0
        self._make_samples_with_skew(cs, a=A, b=B, n=20, step_ms=5000.0)

        assert cs.skew == pytest.approx(A, rel=1e-4)

    def test_skew_b_intercept_recovered(self) -> None:
        """Intercept b is recovered accurately."""
        cs, _ = _make_cs()
        A, B = 1.0, 12345.0
        self._make_samples_with_skew(cs, a=A, b=B, n=10)

        # Reconstruct b from fitted a, b
        assert cs._skew_b == pytest.approx(B, abs=0.1)

    def test_skew_translation_accuracy(self) -> None:
        """to_host_time() using skew model matches ground truth."""
        cs, _ = _make_cs()
        A, B = 1.00002, 8000.0
        self._make_samples_with_skew(cs, a=A, b=B, n=15, step_ms=3000.0)

        for t_r in [0.0, 5000.0, 30000.0, 100000.0]:
            expected = A * t_r + B
            got = cs.to_host_time(t_r)
            assert got is not None
            assert got == pytest.approx(expected, abs=0.5), \
                f"to_host_time({t_r}) = {got}, expected {expected}"

    def test_skew_none_with_one_sample(self) -> None:
        """One sample → no skew model (need ≥2 distinct t_robot values)."""
        cs, _ = _make_cs()
        cs.record_ping(t0_ms=0.0, t1_ms=20.0, t_robot_ms=1000.0)
        assert cs.skew is None

    def test_skew_none_when_samples_too_close(self) -> None:
        """Samples clustered in a burst (< 1 ms robot span) → no skew."""
        cs, _ = _make_cs()
        # All robot stamps at exactly 1000.0 ms (span = 0).
        for _ in range(5):
            cs.record_ping(t0_ms=0.0, t1_ms=20.0, t_robot_ms=1000.0)
        assert cs.skew is None

    def test_skew_falls_back_to_offset_when_none(self) -> None:
        """to_host_time() uses offset when skew is not available."""
        cs, _ = _make_cs(start_s=0.0)
        cs.record_ping(t0_ms=100.0, t1_ms=140.0, t_robot_ms=110.0)
        # offset = (100+140)/2 - 110 = 10
        # No skew (only 1 sample), so: to_host_time(500) = 500 + 10 = 510
        assert cs.skew is None
        assert cs.to_host_time(500) == pytest.approx(510.0)


# ===========================================================================
# ping_burst
# ===========================================================================

class TestPingBurst:
    """Verify ping_burst() with a fake send_fn."""

    def _make_send_fn(
        self,
        replies: list[str | None],
        clock: FakeClock,
        rtt_ms: float = 30.0,
    ) -> object:
        """Return a send_fn that consumes replies from the list.

        Each call advances the clock by rtt_ms and returns the next reply.
        """
        iterator: Iterator[str | None] = iter(replies)

        def send_fn(cmd: str) -> str | None:
            clock.advance_ms(rtt_ms)
            return next(iterator, None)

        return send_fn

    def test_burst_records_all_successful_pings(self) -> None:
        cs, clock = _make_cs()
        replies = [_make_pong(t) for t in [1000, 1030, 1060, 1090, 1120]]
        fn = self._make_send_fn(replies, clock, rtt_ms=30.0)
        cs.ping_burst(fn, n=5)
        assert cs.sample_count == 5

    def test_burst_selects_min_rtt(self) -> None:
        """ping_burst with varying RTTs selects the minimum."""
        cs, clock = _make_cs(start_s=0.0)
        rtts = [80.0, 60.0, 40.0, 50.0, 70.0]
        # Robot clock assumed = host_mid for simplicity (offset=0).
        # We need a send_fn where each call advances the clock by the corresponding rtt.
        call_idx = [0]
        t_robot_counter = [0.0]

        def send_fn(cmd: str) -> str | None:
            rtt = rtts[call_idx[0]]
            clock.advance_ms(rtt)
            t_robot_counter[0] += rtt
            t_r = int(t_robot_counter[0] - rtt / 2)
            call_idx[0] += 1
            return _make_pong(t_r)

        cs.ping_burst(send_fn, n=5)
        assert cs.min_rtt_ms == pytest.approx(40.0, abs=1e-6)

    def test_burst_skips_none_replies(self) -> None:
        """Timed-out PINGs (None reply) are skipped; surviving samples are used."""
        cs, clock = _make_cs()
        replies: list[str | None] = [None, _make_pong(500), None, _make_pong(560), None]
        fn = self._make_send_fn(replies, clock, rtt_ms=30.0)
        cs.ping_burst(fn, n=5)
        assert cs.sample_count == 2

    def test_burst_skips_empty_replies(self) -> None:
        """Empty string replies are treated as failure."""
        cs, clock = _make_cs()
        replies: list[str | None] = ["", _make_pong(800), ""]
        fn = self._make_send_fn(replies, clock, rtt_ms=30.0)
        cs.ping_burst(fn, n=3)
        assert cs.sample_count == 1

    def test_burst_all_fail_leaves_prior_estimate(self) -> None:
        """If all pings fail, the prior estimate is preserved."""
        cs, clock = _make_cs(start_s=0.0)
        # Establish a prior estimate.
        cs.record_ping(t0_ms=0.0, t1_ms=40.0, t_robot_ms=10.0)
        prior_offset = cs.best_offset_ms()
        assert prior_offset is not None

        # Burst with all failures.
        replies: list[str | None] = [None, None, None]
        fn = self._make_send_fn(replies, clock, rtt_ms=20.0)
        cs.ping_burst(fn, n=3)

        # Prior offset must be intact.
        assert cs.best_offset_ms() == pytest.approx(prior_offset)
        assert cs.sample_count == 1  # only the pre-burst sample

    def test_burst_updates_last_sync(self) -> None:
        """ping_burst sets _last_sync_s so stale() is False immediately after."""
        cs, clock = _make_cs()
        assert cs.stale() is True

        replies = [_make_pong(1000)]
        fn = self._make_send_fn(replies, clock, rtt_ms=30.0)
        cs.ping_burst(fn, n=1)

        assert cs.stale() is False

    def test_burst_all_fail_does_not_update_last_sync(self) -> None:
        """If all pings fail, _last_sync_s is NOT updated."""
        cs, clock = _make_cs()
        replies: list[str | None] = [None, None]
        fn = self._make_send_fn(replies, clock, rtt_ms=20.0)
        cs.ping_burst(fn, n=2)
        assert cs.stale() is True


# ===========================================================================
# stale()
# ===========================================================================

class TestStale:
    """Verify stale() reports staleness based on time elapsed."""

    def test_stale_before_any_sync(self) -> None:
        cs, _ = _make_cs()
        assert cs.stale() is True

    def test_not_stale_immediately_after_burst(self) -> None:
        cs, clock = _make_cs()
        cs.record_ping(t0_ms=0.0, t1_ms=20.0, t_robot_ms=10.0)
        # Manually set _last_sync_s to now.
        cs._last_sync_s = clock()
        assert cs.stale(max_age_s=60.0) is False

    def test_stale_after_elapsed_time(self) -> None:
        cs, clock = _make_cs()
        cs._last_sync_s = clock()
        clock.advance(61.0)  # advance 61 seconds
        assert cs.stale(max_age_s=60.0) is True

    def test_not_stale_within_threshold(self) -> None:
        cs, clock = _make_cs()
        cs._last_sync_s = clock()
        clock.advance(30.0)  # only 30 seconds elapsed
        assert cs.stale(max_age_s=60.0) is False

    def test_custom_threshold(self) -> None:
        cs, clock = _make_cs()
        cs._last_sync_s = clock()
        clock.advance(10.0)
        assert cs.stale(max_age_s=5.0) is True
        assert cs.stale(max_age_s=15.0) is False


# ===========================================================================
# reset()
# ===========================================================================

class TestReset:
    """Verify reset() returns ClockSync to pristine uncalibrated state."""

    def test_reset_clears_samples(self) -> None:
        cs, _ = _make_cs(start_s=0.0)
        cs.record_ping(t0_ms=0.0, t1_ms=40.0, t_robot_ms=10.0)
        assert cs.sample_count == 1
        cs.reset()
        assert cs.sample_count == 0

    def test_reset_clears_offset(self) -> None:
        cs, _ = _make_cs(start_s=0.0)
        cs.record_ping(t0_ms=0.0, t1_ms=40.0, t_robot_ms=10.0)
        cs.reset()
        assert cs.best_offset_ms() is None

    def test_reset_clears_skew(self) -> None:
        cs, _ = _make_cs()
        cs.record_ping(t0_ms=0.0, t1_ms=20.0, t_robot_ms=0.0)
        cs.record_ping(t0_ms=1000.0, t1_ms=1020.0, t_robot_ms=1000.0)
        cs.reset()
        assert cs.skew is None

    def test_reset_makes_stale(self) -> None:
        cs, clock = _make_cs()
        cs._last_sync_s = clock()
        cs.reset()
        assert cs.stale() is True

    def test_reset_allows_fresh_calibration(self) -> None:
        cs, _ = _make_cs(start_s=0.0)
        cs.record_ping(t0_ms=0.0, t1_ms=40.0, t_robot_ms=10.0)
        cs.reset()
        cs.record_ping(t0_ms=100.0, t1_ms=120.0, t_robot_ms=105.0)
        # offset = (100+120)/2 - 105 = 110 - 105 = 5
        assert cs.best_offset_ms() == pytest.approx(5.0)


# ===========================================================================
# Properties / aliases
# ===========================================================================

class TestProperties:
    """Verify the convenience property aliases behave correctly."""

    def test_offset_ms_alias(self) -> None:
        cs, _ = _make_cs(start_s=0.0)
        cs.record_ping(t0_ms=100.0, t1_ms=140.0, t_robot_ms=110.0)
        assert cs.offset_ms == cs.best_offset_ms()

    def test_min_rtt_ms_property(self) -> None:
        cs, _ = _make_cs(start_s=0.0)
        cs.record_ping(t0_ms=0.0, t1_ms=60.0, t_robot_ms=30.0)  # RTT=60
        cs.record_ping(t0_ms=100.0, t1_ms=130.0, t_robot_ms=115.0)  # RTT=30
        assert cs.min_rtt_ms == pytest.approx(30.0)

    def test_sample_count_increments(self) -> None:
        cs, _ = _make_cs(start_s=0.0)
        for i in range(7):
            cs.record_ping(t0_ms=float(i * 100), t1_ms=float(i * 100 + 30),
                           t_robot_ms=float(i * 100 + 15))
        assert cs.sample_count == 7
