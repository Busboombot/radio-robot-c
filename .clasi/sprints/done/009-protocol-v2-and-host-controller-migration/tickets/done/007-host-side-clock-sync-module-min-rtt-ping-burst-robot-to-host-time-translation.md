---
id: '007'
title: Host-side clock-sync module (min-RTT PING burst, robot-to-host time translation)
status: done
use-cases:
- SUC-005
depends-on:
- 008
issue: protocol-v2-raw250-hard-break.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 009-007: Host-side clock-sync module

## Description

Implement `host/robot_radio/robot/clock_sync.py` — a pure-Python NTP-style clock
offset estimator. The robot clock is free-running `uBit.systemTime()` (ms since
boot) and must not be set from the host. The host keeps an offset estimate and
uses it to translate robot `t=` timestamps into host time.

**Algorithm** (from the issue — locked):
1. Fire N `PING` commands (default 5) in rapid succession.
2. For each: record `t0 = time.monotonic_ns()` before send, `t1` after reply,
   parse `t_r` from `OK pong t=<t_r>`.
3. Select the sample with minimum RTT (`t1 − t0`).
4. `offset_ms = (t0_best_ms + t1_best_ms) / 2 − t_r_best`
   (robot's clock corresponds to host mid-time at `t_r`).
5. `to_host_time(t_robot_ms)` = `t_robot_ms + offset_ms`.

**Skew (optional, implement if time permits)**: linear regression over several
samples spread in time to fit `host ≈ a·t_robot + b`. Deferred if not in scope.

**API**:
```python
class ClockSync:
    def ping_burst(self, send_fn, n=5) -> None:
        """Fire n PINGs, updating the internal offset estimate."""

    def record_ping(self, t0_ms: float, t1_ms: float, t_robot_ms: int) -> None:
        """Record one PING sample (t0/t1 in host ms, t_robot in robot ms)."""

    def best_offset_ms(self) -> float | None:
        """Return offset from min-RTT sample, or None if no samples."""

    def to_host_time(self, t_robot_ms: int) -> float | None:
        """Translate robot timestamp to host time (ms). None if no calibration."""

    def stale(self, max_age_s: float = 60.0) -> bool:
        """True if last ping burst was more than max_age_s ago."""
```

`ping_burst()` is a convenience wrapper that calls `send_fn("PING")`, waits for
`OK pong t=<n>`, calls `record_ping()` for each reply, then selects the best sample.
`send_fn` is a callable that takes a command string and returns the raw reply line.

## Acceptance Criteria

- [x] `ClockSync` class exists at `host/robot_radio/robot/clock_sync.py`.
- [x] `record_ping(t0, t1, t_robot)` stores samples; `best_offset_ms()` returns the offset from the min-RTT sample.
- [x] `to_host_time(t_robot_ms)` returns `t_robot_ms + best_offset_ms()` (float ms).
- [x] `stale()` returns True after 60 s with no new pings.
- [x] Unit tests (no hardware): simulate 5 PING samples with known RTTs; verify the min-RTT sample is selected and the offset is computed correctly.
- [ ] [BENCH] After a PING burst over the relay, the host's translation of a known robot event (e.g. `EVT done T` from a 1-second timed drive) aligns with the host clock to within ½ the measured minimum RTT.
- [ ] Offset stays stable (< 20 ms drift) over a 3-minute run with re-pings every 30 s.

## Implementation Plan

**Approach**: Pure Python; no external dependencies. Uses `time.monotonic()` for
host-side timestamps.

**Files to create**:
- `host/robot_radio/robot/clock_sync.py`

**Test file**:
- `host/tests/test_clock_sync.py` — unit tests with simulated PING samples.

**Integration**: `Nezha` (ticket 008) will hold a `ClockSync` instance and call
`ping_burst()` on connect and periodically. `TLMFrame.host_t` will call
`clock_sync.to_host_time(frame.t)`.

**Edge cases**:
- No samples yet → `to_host_time()` returns `None`.
- All samples have equal RTT → any sample is valid (first or last).
- `ping_burst()` with `send_fn` that times out → skip failed pings; if < 1 sample,
  leave the existing offset unchanged.

**Testing**:
```python
cs = ClockSync()
# Simulate 5 pings: RTTs 80, 60, 40, 50, 70 ms; min is sample 2 (RTT=40)
# Verify cs.best_offset_ms() matches expected value for that sample.
```
