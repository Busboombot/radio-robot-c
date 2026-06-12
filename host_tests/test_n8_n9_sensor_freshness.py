"""test_n8_n9_sensor_freshness.py — N8+N9 sensor validity fixes (ticket 030-008).

N8: TLM freshness gate for line/color/otos fields.
    A stalled sensor (mock frozen) must stop appearing in TLM within ~2×lagMs
    of the last successful read.

    Default lag values (DefaultConfig.cpp):
      lagLineMs  =  50 ms  → gate = now - lastUpdMs <= 100 ms
      lagColorMs = 100 ms  → gate = now - lastUpdMs <= 200 ms
      lagOtosMs  = 100 ms  → gate = now - lastUpdMs <= 200 ms

N9: Same-tick OTOS read failure must not fuse a zero-filled pose/velocity.
    When MockOtosSensor returns false from readTransformed(), the EKF state
    (fusedV, fusedOmega) and the stored otosX/Y/H must be unchanged from
    the tick before the failure.

Timing notes
~~~~~~~~~~~~
lineRead / colorRead are gated by lagLineMs / lagColorMs in loopTickOnce
(they run at most once per lag period).  TLM fires at cfg.tlmPeriodMs (50ms)
while the robot is in motion; when IDLE it clamps to 500ms.

Tests issue "T 200 200 9999" to keep the robot active (non-idle) so TLM
fires at the normal 50ms rate throughout the collection window.

After tick_for() phases, drain_reply_store() is called to discard TLM frames
accumulated in replyStore (which tick_collect_tlm would otherwise pick up
from its first drain on entry).
"""
from __future__ import annotations

import re
import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _has_field(tlm_line: str, field: str) -> bool:
    """Return True if `field=` appears in the TLM line."""
    return f" {field}=" in tlm_line or tlm_line.startswith(f"{field}=")


def _parse_field(tlm_line: str, field: str) -> str | None:
    """Return the value string after `field=`, or None if absent."""
    m = re.search(rf"{re.escape(field)}=([^\s]+)", tlm_line)
    return m.group(1) if m else None


# ---------------------------------------------------------------------------
# N8 — line sensor freshness gate
# ---------------------------------------------------------------------------

def test_n8_line_field_present_while_sensor_live(sim):
    """line= appears in TLM while the line sensor is producing fresh data.

    Scenario:
      1. Initialize the MockLineSensor; issue T (keeps robot active → TLM at 50ms).
      2. Enable STREAM 50.
      3. Warm up for 5×lagLineMs (250 ms) so lineRead has run several times.
      4. Drain accumulated reply store, then collect 200 ms of TLM.
      5. All collected frames must contain a line= field.
    """
    LAG_MS = 50   # lagLineMs default

    sim.init_line_sensor()
    r = sim.send_command("STREAM 50")
    assert "period=50" in r, f"STREAM 50 rejected: {repr(r)}"
    # Keep robot active so TLM fires at 50ms (not 500ms idle rate).
    r2 = sim.send_command("T 200 200 30000")
    assert "OK" in r2.upper(), f"T command failed: {repr(r2)}"

    # Warm up.
    sim.tick_for(5 * LAG_MS)
    sim.drain_reply_store()

    frames = sim.tick_collect_tlm(total_ms=200, step_ms=24)
    assert len(frames) >= 2, (
        f"Expected >= 2 TLM frames in 200 ms with STREAM 50 while active; "
        f"got {len(frames)}"
    )
    for f in frames:
        assert _has_field(f, "line"), (
            f"Expected line= in TLM frame while sensor is live; frame: {repr(f)}"
        )


def test_n8_line_field_absent_after_sensor_freezes(sim):
    """line= disappears from TLM after the line sensor stalls.

    Scenario:
      1. Initialize MockLineSensor; STREAM 50; T drive (active → 50ms TLM rate).
      2. Warm up for 5×lagLineMs (250 ms).
      3. Freeze the sensor; wait 4×lagMs (200 ms, = 2×window).
      4. Drain reply store; collect 200 ms of TLM.
      5. ALL frames in the collection window must lack line=.

    lagLineMs = 50 ms; gate = now − lastUpdMs <= 100 ms.
    After 200 ms of no updates, every TLM frame must be stale.
    """
    LAG_MS = 50
    WARMUP_MS  = 5 * LAG_MS   # 250 ms
    DRAIN_MS   = 4 * LAG_MS   # 200 ms — exhaust 2×lag window
    COLLECT_MS = 4 * LAG_MS   # 200 ms — all frames must be stale

    sim.init_line_sensor()
    r = sim.send_command("STREAM 50")
    assert "period=50" in r
    # Keep robot active so TLM fires at 50ms rate.
    r2 = sim.send_command("T 200 200 30000")
    assert "OK" in r2.upper(), f"T command failed: {repr(r2)}"

    sim.tick_for(WARMUP_MS)

    # Freeze the sensor.
    sim.set_line_frozen(True)

    # Let the freshness window expire.
    sim.tick_for(DRAIN_MS)
    sim.drain_reply_store()

    # Collect TLM frames — all must lack line=.
    frames = sim.tick_collect_tlm(total_ms=COLLECT_MS, step_ms=24)
    assert len(frames) >= 1, (
        f"Expected at least 1 TLM frame in {COLLECT_MS} ms; got {len(frames)}"
    )
    for f in frames:
        assert not _has_field(f, "line"), (
            f"Expected line= to be absent after sensor freeze + {DRAIN_MS} ms; "
            f"frame: {repr(f)}.  "
            f"N8 freshness gate: now−lastUpdMs <= 2×{LAG_MS}ms must be False."
        )


# ---------------------------------------------------------------------------
# N8 — color sensor freshness gate
# ---------------------------------------------------------------------------

def test_n8_color_field_present_while_sensor_live(sim):
    """color= appears in TLM while the color sensor is producing fresh data.

    lagColorMs = 100 ms; warm up 5×lag = 500 ms, then collect.
    Robot is kept active via T drive to use 50ms TLM rate.
    """
    LAG_MS = 100  # lagColorMs default

    sim.init_color_sensor()
    r = sim.send_command("STREAM 50")
    assert "period=50" in r
    r2 = sim.send_command("T 200 200 30000")
    assert "OK" in r2.upper(), f"T command failed: {repr(r2)}"

    sim.tick_for(5 * LAG_MS)
    sim.drain_reply_store()

    frames = sim.tick_collect_tlm(total_ms=300, step_ms=24)
    assert len(frames) >= 2, (
        f"Expected >= 2 TLM frames in 300 ms; got {len(frames)}"
    )
    for f in frames:
        assert _has_field(f, "color"), (
            f"Expected color= in TLM frame while sensor is live; frame: {repr(f)}"
        )


def test_n8_color_field_absent_after_sensor_freezes(sim):
    """color= disappears from TLM after the color sensor stalls.

    lagColorMs = 100 ms; 2×lag window = 200 ms.
    After 4×lag = 400 ms of no updates, every frame must be stale.
    Robot is kept active via T drive (50ms TLM rate).
    """
    LAG_MS = 100  # lagColorMs default
    WARMUP_MS  = 5 * LAG_MS   # 500 ms
    DRAIN_MS   = 4 * LAG_MS   # 400 ms — exhaust 2×lag window
    COLLECT_MS = 4 * LAG_MS   # 400 ms — all frames must be stale

    sim.init_color_sensor()
    r = sim.send_command("STREAM 50")
    assert "period=50" in r
    r2 = sim.send_command("T 200 200 30000")
    assert "OK" in r2.upper(), f"T command failed: {repr(r2)}"

    sim.tick_for(WARMUP_MS)

    sim.set_color_frozen(True)

    sim.tick_for(DRAIN_MS)
    sim.drain_reply_store()

    frames = sim.tick_collect_tlm(total_ms=COLLECT_MS, step_ms=24)
    assert len(frames) >= 2, (
        f"Expected >= 2 TLM frames in {COLLECT_MS} ms; got {len(frames)}"
    )
    for f in frames:
        assert not _has_field(f, "color"), (
            f"Expected color= to be absent after sensor freeze + {DRAIN_MS} ms; "
            f"frame: {repr(f)}.  "
            f"N8 freshness gate: now−lastUpdMs <= 2×{LAG_MS}ms must be False."
        )


# ---------------------------------------------------------------------------
# N9 — same-tick OTOS read failure must not fuse zeros
# ---------------------------------------------------------------------------

def test_n9_otos_read_failure_does_not_fuse_zeros(sim):
    """A same-tick OTOS I2C failure must not drag fusedV to zero.

    Scenario:
      1. Enable OTOS model + fusion.
      2. Drive T 200 200 3000 and tick 500 ms so fusedV > 0.
      3. Record fusedV_before.
      4. Inject an OTOS read failure.
      5. Run exactly ONE tick.
      6. fusedV_after must be > 50% of fusedV_before.

    Pre-fix: readTransformed returned {0,0,0}, correctEKF fused v_otos=0 and
    dragged fusedV toward zero.  Post-fix: return false → skip fusion entirely.
    """
    import ctypes

    sim._lib.sim_enable_otos_model(sim._h)
    sim._lib.sim_set_otos_fusion(sim._h, 1)

    r = sim.send_command("T 200 200 3000")
    assert "OK" in r.upper(), f"T command failed: {repr(r)}"
    sim.tick_for(500)

    fused_v_before = sim.get_fused_v()
    assert fused_v_before > 50.0, (
        f"Expected fusedV > 50 mm/s after 500 ms of T 200 drive; "
        f"got fusedV={fused_v_before:.1f}.  OTOS fusion may not be active."
    )

    sim.set_otos_read_failure(True)

    t = sim._t
    sim._lib.sim_tick(sim._h, ctypes.c_uint32(t))
    sim._t += 24

    fused_v_after = sim.get_fused_v()
    sim.set_otos_read_failure(False)

    assert fused_v_after > 0.5 * fused_v_before, (
        f"fusedV dropped too much after one same-tick OTOS read failure: "
        f"before={fused_v_before:.1f} mm/s, after={fused_v_after:.1f} mm/s.  "
        f"N9 fix: readTransformed must return false → otosCorrect skips fusion."
    )


def test_n9_otos_pose_not_updated_on_read_failure(sim):
    """otosX/Y/H must not be overwritten with zeros on a same-tick OTOS failure.

    Scenario:
      1. Enable OTOS model + fusion; drive 300 ms to build a non-zero pose.
      2. Record SNAP otos= values.
      3. Inject a read failure and run one tick.
      4. otos= must be absent (freshness gate) OR not {0,0,0} if robot was displaced.
    """
    import ctypes

    sim._lib.sim_enable_otos_model(sim._h)
    sim._lib.sim_set_otos_fusion(sim._h, 1)

    r = sim.send_command("T 200 200 3000")
    assert "OK" in r.upper()
    sim.tick_for(300)

    snap_before = sim.send_command("SNAP")
    otos_before = _parse_field(snap_before, "otos")
    assert otos_before is not None, (
        f"Expected otos= in SNAP before failure; got: {repr(snap_before)}"
    )
    parts_before = [float(v) for v in otos_before.split(",")]
    assert len(parts_before) == 3

    sim.set_otos_read_failure(True)
    t = sim._t
    sim._lib.sim_tick(sim._h, ctypes.c_uint32(t))
    sim._t += 24

    snap_after = sim.send_command("SNAP")
    otos_after = _parse_field(snap_after, "otos")
    sim.set_otos_read_failure(False)

    # Either absent (freshness gate) or same values — not zeroed.
    if otos_after is not None:
        parts_after = [float(v) for v in otos_after.split(",")]
        assert len(parts_after) == 3
        all_zero = (abs(parts_after[0]) < 1.0 and
                    abs(parts_after[1]) < 1.0 and
                    abs(parts_after[2]) < 1.0)
        if all_zero and (abs(parts_before[0]) > 5.0 or abs(parts_before[1]) > 5.0):
            pytest.fail(
                f"otos= was overwritten with zeros after a same-tick read failure.  "
                f"Before: otos={otos_before}, After: otos={otos_after}.  "
                f"N9 fix: otosCorrect must NOT update otosX/Y/H on poseOk=false."
            )
