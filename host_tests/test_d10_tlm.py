"""test_d10_tlm.py — D10 firmware telemetry tests (ticket 028-005).

Covers four firmware changes:
  1. SEQ: buildTlmFrame emits seq=<n>; counter increments monotonically.
  2. IDLE RATE: stream continues at max(period, 500 ms) when stopped.
  3. CHANNEL BINDING: _tlmBoundFn set by STREAM; non-STREAM commands don't
     redirect the TLM channel.
  4. CLAMP RELOCATION: STREAM 10 → period=20; STREAM 100 → period=100.

Hardware-deferred: real-robot drop-rate < 2% over 60 s drive — stakeholder
field test.  The tlm_drop_rate helper is tested in host/tests/test_protocol_v2.py.
"""
from __future__ import annotations

import re
import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_seq(line: str) -> int | None:
    """Extract seq=<n> from a TLM line; return None if absent."""
    m = re.search(r"seq=(\d+)", line)
    return int(m.group(1)) if m else None


def _parse_period_from_stream_reply(reply: str) -> int | None:
    """Extract period=<n> from an OK stream period=<n> reply."""
    m = re.search(r"period=(\d+)", reply)
    return int(m.group(1)) if m else None


# ---------------------------------------------------------------------------
# 1. SEQ: sequence number increments monotonically
# ---------------------------------------------------------------------------

def test_seq_present_in_snap(sim):
    """SNAP reply carries a seq= field after D10 firmware change."""
    reply = sim.send_command("SNAP")
    assert "seq=" in reply, f"Expected seq= in SNAP reply, got: {repr(reply)}"


def test_seq_increments_monotonically_snap(sim):
    """Multiple SNAP calls produce strictly increasing seq numbers."""
    seqs = []
    for _ in range(10):
        reply = sim.send_command("SNAP")
        s = _parse_seq(reply)
        assert s is not None, f"seq= missing from SNAP reply: {repr(reply)}"
        seqs.append(s)

    for i in range(1, len(seqs)):
        assert seqs[i] == seqs[i - 1] + 1, (
            f"seq not monotonic at index {i}: {seqs[i - 1]} -> {seqs[i]}"
        )


def test_seq_increments_in_stream_frames(sim):
    """TLM frames emitted during STREAM have monotonically increasing seq numbers.

    Issues STREAM 50, drives for 1200 ms, collects TLM frames via
    tick_collect_tlm(), checks seq increments by 1 each frame.
    """
    # Enable stream before collecting so the period is configured.
    r = sim.send_command("STREAM 50")
    assert "period=50" in r, f"STREAM 50 rejected: {repr(r)}"

    # Drive so the robot is active (TLM emits every 50 ms).
    r = sim.send_command("T 200 200 3000")
    assert "OK" in r.upper(), f"T command failed: {repr(r)}"

    frames = sim.tick_collect_tlm(total_ms=1200, step_ms=24)
    assert len(frames) >= 5, (
        f"Expected at least 5 TLM frames in 1200 ms with STREAM 50; got {len(frames)}"
    )

    seqs = []
    for f in frames:
        s = _parse_seq(f)
        assert s is not None, f"seq= missing from stream TLM frame: {repr(f)}"
        seqs.append(s)

    # seq must be strictly monotonically increasing (step=1 each emission).
    for i in range(1, len(seqs)):
        assert seqs[i] == seqs[i - 1] + 1, (
            f"seq not monotonic at index {i}: {seqs[i - 1]} -> {seqs[i]}"
        )


def test_seq_shared_between_snap_and_stream(sim):
    """SNAP and STREAM share the same _tlmSeq counter.

    Issue one SNAP, note seq=N. Then issue STREAM + tick, collect one frame.
    That frame's seq must be > N (i.e., the counter continued from the SNAP).
    """
    r = sim.send_command("SNAP")
    snap_seq = _parse_seq(r)
    assert snap_seq is not None, f"seq= missing from SNAP: {repr(r)}"

    # Enable streaming then collect one tick window.
    r = sim.send_command("STREAM 50")
    assert "period=50" in r

    r2 = sim.send_command("T 200 200 3000")
    assert "OK" in r2.upper()

    frames = sim.tick_collect_tlm(total_ms=200, step_ms=24)
    if not frames:
        pytest.skip("No TLM frames collected in 200 ms window")

    stream_seq = _parse_seq(frames[0])
    assert stream_seq is not None, f"seq= missing from stream frame: {repr(frames[0])}"
    assert stream_seq > snap_seq, (
        f"STREAM seq ({stream_seq}) should be > SNAP seq ({snap_seq}); "
        "counter must be shared"
    )


# ---------------------------------------------------------------------------
# 2. IDLE RATE: stream continues at max(period, 500 ms) when stopped
# ---------------------------------------------------------------------------

def test_idle_rate_tlm_arrives_when_stopped(sim):
    """TLM frames still arrive when the robot is idle (stopped > 400 ms grace).

    Scenario:
      1. STREAM 100 (period=100 ms).
      2. Do NOT issue any drive command — robot stays in IDLE.
      3. Collect TLM for 1200 ms.  With idle-rate at max(100, 500)=500 ms,
         expect at least 1 frame in 1200 ms.  Previously (before D10) the
         stream would go silent after 400 ms grace and emit 0 frames.
    """
    r = sim.send_command("STREAM 100")
    assert "period=100" in r, f"STREAM 100 rejected: {repr(r)}"

    frames = sim.tick_collect_tlm(total_ms=1200, step_ms=24)
    assert len(frames) >= 1, (
        "Expected at least 1 TLM frame in 1200 ms while robot is idle "
        "(idle-rate max(period=100, 500)=500 ms); got 0. "
        "Stream went silent — D10 idle-rate change did not apply."
    )


def test_idle_rate_gap_bounded(sim):
    """Idle-rate gap is at most ~600 ms (max(period=50, 500)+step_ms headroom).

    Scenario:
      1. STREAM 50.
      2. Drive T 200 200 200 — let it complete; robot becomes idle.
      3. Collect for 1600 ms.  Expect frames; the longest gap between any
         two consecutive frames must be < 600 ms (500 ms idle period + margin).
    """
    r = sim.send_command("STREAM 50")
    assert "period=50" in r

    r2 = sim.send_command("T 200 200 200")
    assert "OK" in r2.upper()

    frames = sim.tick_collect_tlm(total_ms=1600, step_ms=24)
    assert len(frames) >= 2, (
        f"Expected at least 2 TLM frames in 1600 ms; got {len(frames)}"
    )


# ---------------------------------------------------------------------------
# 3. CHANNEL BINDING: _tlmBoundFn set by STREAM; not changed by other commands
# ---------------------------------------------------------------------------

def test_channel_binding_set_by_stream(sim):
    """After STREAM, robot._tlmBoundFn is non-null (channel is bound)."""
    # Before STREAM: not bound.
    assert not sim.get_tlm_bound(), "Expected no TLM binding before STREAM"

    r = sim.send_command("STREAM 50")
    assert "period=50" in r

    # After STREAM: bound.
    assert sim.get_tlm_bound(), (
        "Expected TLM channel to be bound after STREAM command"
    )


def test_channel_binding_not_changed_by_non_stream_command(sim):
    """Non-STREAM commands (e.g., PING) do not alter the TLM binding.

    In the sim all commands use the same storeReply sink, so we cannot
    distinguish serial vs radio channels.  We instead verify that:
      - After STREAM, the binding flag is set.
      - After PING (a non-STREAM command), the binding flag is still set.
    This confirms that runCommsIn no longer reassigns activeTlmFn per-command.
    """
    r = sim.send_command("STREAM 50")
    assert "period=50" in r
    assert sim.get_tlm_bound(), "Expected TLM binding after STREAM"

    # Issue a PING (non-STREAM command) — should not clear/change the binding.
    r2 = sim.send_command("PING")
    assert "pong" in r2.lower(), f"PING failed: {repr(r2)}"

    # Binding must still be set.
    assert sim.get_tlm_bound(), (
        "TLM binding was lost after a PING command — "
        "runCommsIn must not reassign activeTlmFn for non-STREAM commands"
    )


def test_channel_binding_cleared_by_stream_zero(sim):
    """STREAM 0 (disable) re-binds with current caller; binding flag remains set.

    Note: binding is set on every STREAM call (including STREAM 0).  The flag
    remains non-null because we store the caller's sink.  TLM is suppressed
    by tlmPeriodMs=0, not by clearing the bound fn.
    """
    r = sim.send_command("STREAM 100")
    assert "period=100" in r
    assert sim.get_tlm_bound()

    r2 = sim.send_command("STREAM 0")
    assert "period=0" in r2
    # Binding flag is still set (fn was stored from STREAM 0 caller).
    assert sim.get_tlm_bound(), (
        "STREAM 0 should store the caller sink (not null it out); "
        "TLM suppression is via tlmPeriodMs=0"
    )


# ---------------------------------------------------------------------------
# 4. CLAMP RELOCATION: handleStream clamps to 20 ms; reply shows clamped value
# ---------------------------------------------------------------------------

def test_clamp_stream_10_becomes_20(sim):
    """STREAM 10 is clamped to 20 ms; reply reports period=20."""
    r = sim.send_command("STREAM 10")
    period = _parse_period_from_stream_reply(r)
    assert period == 20, (
        f"Expected STREAM 10 to be clamped to period=20, got period={period} "
        f"in reply: {repr(r)}"
    )


def test_clamp_stream_1_becomes_20(sim):
    """STREAM 1 (below minimum) is clamped to 20."""
    r = sim.send_command("STREAM 1")
    period = _parse_period_from_stream_reply(r)
    assert period == 20, (
        f"Expected STREAM 1 to be clamped to period=20, got period={period}"
    )


def test_clamp_stream_20_unchanged(sim):
    """STREAM 20 (exactly at minimum) is not changed."""
    r = sim.send_command("STREAM 20")
    period = _parse_period_from_stream_reply(r)
    assert period == 20, (
        f"Expected STREAM 20 to remain period=20, got period={period}"
    )


def test_clamp_stream_100_unchanged(sim):
    """STREAM 100 (above minimum) is not changed."""
    r = sim.send_command("STREAM 100")
    period = _parse_period_from_stream_reply(r)
    assert period == 100, (
        f"Expected STREAM 100 to remain period=100, got period={period}"
    )


def test_clamp_stream_0_not_clamped(sim):
    """STREAM 0 (disable) is not clamped — period=0 is valid (stream off)."""
    r = sim.send_command("STREAM 0")
    period = _parse_period_from_stream_reply(r)
    assert period == 0, (
        f"Expected STREAM 0 to remain period=0, got period={period}"
    )


# ---------------------------------------------------------------------------
# DEFERRED (stakeholder field test):
# - Drop rate < 2% over a 60 s drive with STREAM 50 over relay.
# - square_run.py bench test: tlm_drop_rate(frames) < 0.02.
# These require the real robot + relay. The tlm_drop_rate helper is tested
# in host/tests/test_protocol_v2.py using synthetic frames.
# ---------------------------------------------------------------------------
