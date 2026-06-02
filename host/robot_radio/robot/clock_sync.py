"""ClockSync — NTP-style host-side clock offset estimator for robot timestamps.

The robot runs a free-running clock (`uBit.systemTime()`, ms since boot) that
we cannot and do not set from the host.  Instead we keep an *offset estimate*
on the host and use it to translate robot `t=` timestamps into host-monotonic
time so that robot events can be correlated with host events.

Algorithm (NTP-style min-RTT filtering)
---------------------------------------
For each PING exchange:
  - T0  = host monotonic time (ms) *before* sending PING
  - T1  = host monotonic time (ms) *after* receiving the reply
  - t_r = robot clock (ms) parsed from "OK pong t=<t_r>"

Assuming a roughly symmetric link delay, the robot clock stamp `t_r`
corresponds to host mid-time ``(T0+T1)/2``.  Therefore:

    offset_ms = (T0 + T1) / 2 − t_r

We fire N PINGs and keep the sample with the smallest RTT (= T1 − T0)
because that sample has the least relay/queuing jitter — this is the
classic NTP min-RTT trick.  Accuracy is bounded by ~½ the minimum RTT.

Skew compensation
-----------------
The micro:bit crystal drifts ~tens of ppm (a few ms/min).  After
accumulating enough samples we fit a linear model:

    host_mid ≈ a · t_robot + b

``a`` is the skew factor (ideally ≈ 1.0 for a perfect clock); ``b`` is
the intercept.  ``to_host_time()`` uses the skew model when available,
falling back to the offset-only estimate.

Usage example::

    cs = ClockSync()
    cs.ping_burst(lambda cmd: proto.ping_and_raw(cmd))
    host_ms = cs.to_host_time(tlm_frame.t)
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable


# ---------------------------------------------------------------------------
# Internal sample record
# ---------------------------------------------------------------------------

@dataclass
class _PingSample:
    """One recorded PING exchange."""
    t0_ms: float       # host monotonic time before send (ms)
    t1_ms: float       # host monotonic time after receive (ms)
    t_robot_ms: float  # robot clock at pong (ms)

    @property
    def rtt_ms(self) -> float:
        """Round-trip time in ms."""
        return self.t1_ms - self.t0_ms

    @property
    def offset_ms(self) -> float:
        """Estimated host-minus-robot clock offset (ms)."""
        return (self.t0_ms + self.t1_ms) / 2.0 - self.t_robot_ms


# ---------------------------------------------------------------------------
# ClockSync
# ---------------------------------------------------------------------------

class ClockSync:
    """Host-side NTP-style clock offset estimator.

    Typical use:
    1. Call ``ping_burst(send_fn)`` on connect and every ~30–60 s.
    2. Call ``to_host_time(t_robot_ms)`` to translate robot timestamps.

    Args:
        clock_fn: Callable[[], float] that returns host monotonic time in
            **seconds** (default: ``time.monotonic``).  Inject a fake for
            unit tests.
    """

    def __init__(self, clock_fn: Callable[[], float] | None = None) -> None:
        self._clock = clock_fn if clock_fn is not None else time.monotonic
        self._samples: list[_PingSample] = []
        self._best: _PingSample | None = None
        self._last_sync_s: float | None = None  # host monotonic seconds of last burst

        # Skew model (populated after ≥2 samples with distinct t_robot values)
        self._skew_a: float | None = None  # slope  (host_mid per robot_ms)
        self._skew_b: float | None = None  # intercept (ms)

    # ------------------------------------------------------------------
    # Core recording
    # ------------------------------------------------------------------

    def record_ping(
        self,
        t0_ms: float,
        t1_ms: float,
        t_robot_ms: float,
    ) -> None:
        """Record one PING sample.

        Args:
            t0_ms: Host monotonic time in ms *before* the PING was sent.
            t1_ms: Host monotonic time in ms *after* the pong reply arrived.
            t_robot_ms: Robot clock stamp (ms) from ``OK pong t=<n>``.
        """
        sample = _PingSample(t0_ms=t0_ms, t1_ms=t1_ms, t_robot_ms=t_robot_ms)
        self._samples.append(sample)

        # Update the best (min-RTT) sample.
        if self._best is None or sample.rtt_ms < self._best.rtt_ms:
            self._best = sample

        # Recompute skew model when we have ≥2 samples.
        self._fit_skew()

    def _fit_skew(self) -> None:
        """Fit linear model host_mid ≈ a·t_robot + b using all samples.

        Uses ordinary least squares (no external dependencies).
        Only applied if samples span at least 1 ms of robot time (to avoid
        degenerate regression when all samples arrive in a burst).
        """
        n = len(self._samples)
        if n < 2:
            self._skew_a = None
            self._skew_b = None
            return

        xs = [s.t_robot_ms for s in self._samples]
        ys = [(s.t0_ms + s.t1_ms) / 2.0 for s in self._samples]  # host_mid

        x_span = max(xs) - min(xs)
        if x_span < 1.0:
            # Samples too close in robot time — skip skew, keep offset only.
            self._skew_a = None
            self._skew_b = None
            return

        # OLS: minimise Σ(y − a·x − b)²
        x_mean = sum(xs) / n
        y_mean = sum(ys) / n
        ss_xx = sum((x - x_mean) ** 2 for x in xs)
        ss_xy = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))

        if ss_xx == 0.0:
            self._skew_a = None
            self._skew_b = None
            return

        self._skew_a = ss_xy / ss_xx
        self._skew_b = y_mean - self._skew_a * x_mean

    # ------------------------------------------------------------------
    # Ping burst convenience
    # ------------------------------------------------------------------

    def ping_burst(
        self,
        send_fn: Callable[[str], str | None],
        n: int = 5,
    ) -> None:
        """Fire *n* PINGs and update the internal offset estimate.

        ``send_fn`` must accept a command string and return the raw reply
        line (e.g. ``"OK pong t=12345"``), or ``None``/empty on timeout.

        Samples from this burst are *appended* to any prior samples so the
        skew regression can span multiple bursts.  If fewer than 1 sample
        succeeds the existing estimate is left unchanged.

        Args:
            send_fn: Callable(cmd: str) -> str | None
            n: Number of PINGs to fire (default 5).
        """
        new_samples: list[_PingSample] = []

        for _ in range(n):
            t0_s = self._clock()
            reply = send_fn("PING")
            t1_s = self._clock()

            if not reply:
                continue

            t_robot_ms = _parse_pong_t(reply)
            if t_robot_ms is None:
                continue

            t0_ms = t0_s * 1000.0
            t1_ms = t1_s * 1000.0
            sample = _PingSample(t0_ms=t0_ms, t1_ms=t1_ms, t_robot_ms=float(t_robot_ms))
            new_samples.append(sample)

        if not new_samples:
            return  # No successful pings — leave prior estimate intact.

        for s in new_samples:
            self._samples.append(s)
            if self._best is None or s.rtt_ms < self._best.rtt_ms:
                self._best = s

        self._last_sync_s = self._clock()
        self._fit_skew()

    # ------------------------------------------------------------------
    # Query API
    # ------------------------------------------------------------------

    def best_offset_ms(self) -> float | None:
        """Return the clock offset from the min-RTT sample (ms).

        Returns:
            offset_ms = (T0+T1)/2 − t_robot  from the best sample,
            or ``None`` if no samples have been recorded yet.
        """
        if self._best is None:
            return None
        return self._best.offset_ms

    def to_host_time(self, t_robot_ms: float) -> float | None:
        """Translate a robot timestamp to host-monotonic time (ms).

        Uses the skew-corrected model when available (≥2 samples spanning
        enough time), otherwise falls back to the offset-only estimate.

        Args:
            t_robot_ms: Robot clock value (ms from robot boot).

        Returns:
            Estimated host monotonic time in ms, or ``None`` if no
            calibration data is available.
        """
        if self._skew_a is not None and self._skew_b is not None:
            return self._skew_a * t_robot_ms + self._skew_b

        offset = self.best_offset_ms()
        if offset is None:
            return None
        return t_robot_ms + offset

    def stale(self, max_age_s: float = 60.0) -> bool:
        """Return True if no ping burst has been recorded within *max_age_s* seconds.

        Args:
            max_age_s: Staleness threshold in seconds (default 60 s).

        Returns:
            True if last sync is older than *max_age_s* (or never synced).
        """
        if self._last_sync_s is None:
            return True
        return (self._clock() - self._last_sync_s) > max_age_s

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------

    @property
    def min_rtt_ms(self) -> float | None:
        """Minimum RTT observed across all recorded samples (ms), or None."""
        if self._best is None:
            return None
        return self._best.rtt_ms

    @property
    def offset_ms(self) -> float | None:
        """Alias for ``best_offset_ms()``."""
        return self.best_offset_ms()

    @property
    def skew(self) -> float | None:
        """Skew factor *a* from the linear model, or None if not yet fitted.

        A value close to 1.0 means the two clocks tick at almost the same rate.
        Values slightly above/below 1.0 indicate crystal drift.
        """
        return self._skew_a

    @property
    def sample_count(self) -> int:
        """Total number of PING samples recorded."""
        return len(self._samples)

    def reset(self) -> None:
        """Discard all samples and reset to uncalibrated state."""
        self._samples.clear()
        self._best = None
        self._last_sync_s = None
        self._skew_a = None
        self._skew_b = None


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _parse_pong_t(line: str) -> int | None:
    """Extract ``t`` integer from an ``OK pong t=<n>`` reply line.

    Returns the robot timestamp (int ms) or None if parsing fails.
    Handles relay prefix stripping (leading ``< ``).
    """
    stripped = line.strip().lstrip("<# ").strip()
    # Fast path: look for "t=" token.
    for tok in stripped.split():
        if tok.startswith("t="):
            val = tok[2:]
            try:
                return int(val)
            except ValueError:
                return None
    return None
