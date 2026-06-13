"""
test_dbg_otos_commands.py — 031-003 command-table and parse tests for
DBG OTOS BENCH and DBG OTOS.

Verifies:
  - "DBG OTOS BENCH" and "DBG OTOS" are present in the command table
    (reported by HELP).
  - "DBG OTOS BENCH" appears before "DBG OTOS" in the table (longest
    prefix wins).
  - "DBG OTOS BENCH 1" replies OK dbg otos bench=<n>  (no crash in sim).
  - "DBG OTOS BENCH 0" replies OK dbg otos bench=0.
  - "DBG OTOS BENCH 1 noiseXY=0.5 noiseH=0.01 drift=0.001" is accepted
    without error.
  - "DBG OTOS" (bare) replies with a line containing ideal=, otos=, fused=.
"""

import pytest
from firmware import Sim


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def send(s: Sim, cmd: str) -> str:
    return s.send_command(cmd)


def get_help_text(s: Sim) -> str:
    """Return the HELP reply (multi-line command listing)."""
    return s.send_command("HELP")


# ---------------------------------------------------------------------------
# Table registration — verified via dispatch behavior, not HELP text.
# HELP is a hardcoded string and does not enumerate registered commands.
# We verify registration by checking that the commands are dispatched
# correctly (OK reply, expected fields) rather than appearance in HELP.
# ---------------------------------------------------------------------------

def test_dbg_otos_bench_registered():
    """DBG OTOS BENCH is registered: replies OK (not ERR unknown)."""
    with Sim() as s:
        r = send(s, "DBG OTOS BENCH 1")
        assert "ERR unknown" not in r, (
            f"'DBG OTOS BENCH' not registered (got ERR unknown): {repr(r)}"
        )
        assert "OK" in r.upper(), (
            f"'DBG OTOS BENCH' must reply OK when registered: {repr(r)}"
        )


def test_dbg_otos_registered():
    """DBG OTOS is registered: replies OK with pose fields (not ERR unknown)."""
    with Sim() as s:
        r = send(s, "DBG OTOS")
        assert "ERR unknown" not in r, (
            f"'DBG OTOS' not registered (got ERR unknown): {repr(r)}"
        )
        assert "ideal=" in r, (
            f"'DBG OTOS' must reply with pose fields when registered: {repr(r)}"
        )


def test_dbg_otos_bench_before_dbg_otos_dispatch():
    """Prefix ordering: 'DBG OTOS BENCH 1' dispatches to bench handler (not query).

    If DBG OTOS matched before DBG OTOS BENCH, then 'DBG OTOS BENCH 1' would
    be dispatched to the query handler (which outputs ideal=...), but the
    bench handler outputs 'otos bench=' — not 'ideal='.
    This verifies longest-prefix-first ordering in the command table.
    """
    with Sim() as s:
        r = send(s, "DBG OTOS BENCH 1")
        # Bench handler reply contains "otos bench=" — not "ideal=".
        assert "otos bench=" in r, (
            f"'DBG OTOS BENCH 1' must be dispatched to the bench handler "
            f"(expected 'otos bench=' in reply): {repr(r)}"
        )
        # If misrouted to query handler, reply would contain "ideal=".
        assert "ideal=" not in r, (
            f"'DBG OTOS BENCH 1' was incorrectly dispatched to the query "
            f"handler (should not contain 'ideal='): {repr(r)}"
        )


# ---------------------------------------------------------------------------
# DBG OTOS BENCH command parsing / reply
# ---------------------------------------------------------------------------

def test_dbg_otos_bench_enable():
    """DBG OTOS BENCH 1 must actually ENABLE bench mode -> reply bench=1.

    Regression for 033-002: a union-aliasing parse bug in parseDbgOtosBench
    (writing fval=0.0f after ival on the shared union) zeroed the enable flag,
    so the handler always saw enable=0 and replied bench=0 even for `BENCH 1`.
    The HOST handler now mirrors the toggle via Robot::setBenchOtosEnabled so the
    sim observes the real enable state.
    """
    with Sim() as s:
        r = send(s, "DBG OTOS BENCH 1")
        assert "OK" in r.upper(), f"Expected OK reply, got: {repr(r)}"
        assert "otos bench=1" in r, f"BENCH 1 must report bench=1, got: {repr(r)}"


def test_dbg_otos_bench_enable_round_trip():
    """Enable -> disable -> enable round-trips the bench flag, incl. with noise args."""
    with Sim() as s:
        assert "otos bench=1" in send(s, "DBG OTOS BENCH 1")
        assert "otos bench=0" in send(s, "DBG OTOS BENCH 0")
        # Positional noise args after the enable flag must not break the enable parse.
        assert "otos bench=1" in send(s, "DBG OTOS BENCH 1 20 10")


def test_dbg_otos_bench_disable():
    """DBG OTOS BENCH 0 replies OK dbg otos bench=0."""
    with Sim() as s:
        r = send(s, "DBG OTOS BENCH 0")
        assert "OK" in r.upper(), f"Expected OK reply, got: {repr(r)}"
        assert "otos bench=0" in r, f"Expected 'otos bench=0' in reply, got: {repr(r)}"


def test_dbg_otos_bench_with_noise_params():
    """DBG OTOS BENCH 1 noiseXY=0.5 noiseH=0.01 drift=0.001 is accepted without error."""
    with Sim() as s:
        r = send(s, "DBG OTOS BENCH 1 noiseXY=0.5 noiseH=0.01 drift=0.001")
        assert "OK" in r.upper(), f"Expected OK reply, got: {repr(r)}"
        assert "otos bench=" in r, f"Expected 'otos bench=' in reply, got: {repr(r)}"
        # Must NOT reply ERR.
        assert "ERR" not in r.upper(), f"Unexpected ERR in reply: {repr(r)}"


def test_dbg_otos_bench_no_args():
    """DBG OTOS BENCH with no args defaults to disable (bench=0), no crash."""
    with Sim() as s:
        r = send(s, "DBG OTOS BENCH")
        assert "OK" in r.upper(), f"Expected OK reply, got: {repr(r)}"
        assert "otos bench=" in r, f"Expected 'otos bench=' in reply, got: {repr(r)}"


# ---------------------------------------------------------------------------
# DBG OTOS query
# ---------------------------------------------------------------------------

def test_dbg_otos_query_returns_pose_fields():
    """DBG OTOS reply contains ideal=, otos=, fused=, and OK dbg otos."""
    with Sim() as s:
        r = send(s, "DBG OTOS")
        assert "ideal=" in r, f"Expected 'ideal=' in reply, got: {repr(r)}"
        assert "otos=" in r, f"Expected 'otos=' in reply, got: {repr(r)}"
        assert "fused=" in r, f"Expected 'fused=' in reply, got: {repr(r)}"
        assert "OK" in r.upper(), f"Expected 'OK' in reply, got: {repr(r)}"


def test_dbg_otos_query_contains_err_field():
    """DBG OTOS reply includes err= field."""
    with Sim() as s:
        r = send(s, "DBG OTOS")
        assert "err=" in r, f"Expected 'err=' in reply, got: {repr(r)}"


def test_dbg_otos_query_zero_at_start():
    """At startup all pose fields are zero."""
    with Sim() as s:
        r = send(s, "DBG OTOS")
        # F1 fix (034-004): integer format: ideal=0,0,0 otos=0,0,0 fused=0,0,0
        # 'ideal=0' and 'fused=0' are still present in 'ideal=0,0,0' / 'fused=0,0,0'.
        assert "ideal=0" in r, f"Expected 'ideal=0...' at start, got: {repr(r)}"
        assert "fused=0" in r, f"Expected 'fused=0...' at start, got: {repr(r)}"


def test_dbg_otos_not_dispatched_as_bench():
    """Bare 'DBG OTOS' must NOT be dispatched as DBG OTOS BENCH.

    DBG OTOS BENCH expects an enable-flag; a bare DBG OTOS must route to
    the query handler (which accepts 0 args).  Both replies must contain
    OK, but only the query reply has 'ideal='.
    """
    with Sim() as s:
        r = send(s, "DBG OTOS")
        assert "ideal=" in r, (
            f"DBG OTOS was NOT dispatched to query handler "
            f"(possibly mis-routed to BENCH handler): {repr(r)}"
        )
        assert "otos bench=" not in r, (
            f"DBG OTOS was incorrectly dispatched to BENCH handler: {repr(r)}"
        )


# ---------------------------------------------------------------------------
# F1 integer format — 034-004 new host sim test
# ---------------------------------------------------------------------------

def test_dbg_otos_query_integer_format_non_zero_fused():
    """DBG OTOS reply has integer (not float) format; fused= is non-zero when
    OTOS pose has been injected and fused via otosCorrect().

    034-004 (F1 fix): CODAL/newlib-nano has no float printf; the reply was
    changed to scaled integers (mm for position, cdeg for heading).
    This test verifies:
      1. The reply does NOT contain '.' characters in the numeric fields
         (integer-only format).
      2. The fused= field is non-zero after a known OTOS pose is injected
         and fused — confirms the integer-format path produces real numbers,
         not the empty strings that %f produced on hardware.
    """
    import ctypes
    import math

    with Sim() as s:
        # Inject a known OTOS pose: x=500 mm, y=-100 mm, h=pi/2 rad (90°).
        # Enable OTOS fusion so otosCorrect() fuses the injected pose into
        # state.inputs.otosX/Y/H (the "fused" fields that DBG OTOS reads).
        s._lib.sim_set_otos_pose(
            s._h,
            ctypes.c_float(500.0),
            ctypes.c_float(-100.0),
            ctypes.c_float(math.pi / 2),
        )
        # enable_otos_model + otos_fusion marks MockOtosSensor initialized and
        # enables the otosCorrect() EKF path each tick.
        s._lib.sim_enable_otos_model(s._h)
        s._lib.sim_set_otos_fusion(s._h, ctypes.c_int(1))

        # Advance one tick so loopTickOnce calls otosCorrect and fuses the pose.
        s.tick_for(24)

        r = send(s, "DBG OTOS")

        # Field-presence sanity.
        assert "fused=" in r, f"Expected 'fused=' in reply, got: {repr(r)}"

        # F1 check: the pose triple for fused must not contain a decimal point.
        # Locate 'fused=' and check the triple that follows.
        fused_idx = r.find("fused=")
        assert fused_idx >= 0, f"'fused=' not found in: {repr(r)}"
        triple_start = fused_idx + len("fused=")
        # The triple ends at the next space or end-of-line.
        triple_end = r.find(" ", triple_start)
        if triple_end < 0:
            triple_end = len(r)
        fused_triple = r[triple_start:triple_end]
        assert "." not in fused_triple, (
            f"fused= triple contains '.' (float format, not integer): "
            f"fused_triple={repr(fused_triple)}, full reply={repr(r)}"
        )

        # Non-zero check: after injecting x=500 the fused x should be close to 500.
        # Extract the first integer in the triple.
        parts = fused_triple.split(",")
        assert len(parts) >= 3, f"fused triple should have 3 parts, got: {repr(fused_triple)}"
        fused_x_mm = int(parts[0])
        assert abs(fused_x_mm) > 10, (
            f"fused_x should be non-zero after pose injection, got {fused_x_mm} mm; "
            f"full reply: {repr(r)}"
        )
