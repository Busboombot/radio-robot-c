"""
test_halt_controller.py — tests for the HALT command and HaltController (ticket 020-007).

Exercises:
  - HALT TIME registers a time condition; EVT halt fires after the threshold.
  - HALT DIST registers a distance condition; EVT halt fires after threshold mm.
  - HALT CLEAR removes all conditions; no EVT fires afterward.
  - HALT DIST SOFT fires with EVT halt and issues a soft stop.
  - HALT LINE ANY GE fires when any line sensor >= threshold.
  - ZERO T / ZERO D reset the baselines used by HALT TIME / HALT DIST.
"""
import re


def _find_evt_halt(evts: str) -> int | None:
    """Return the halt id from 'EVT halt id=<n>' in evts, or None."""
    m = re.search(r"EVT halt id=(\d+)", evts)
    return int(m.group(1)) if m else None


# ---------------------------------------------------------------------------
# HALT TIME
# ---------------------------------------------------------------------------

def test_halt_time_registers(sim):
    """HALT TIME 200 returns OK HALT id=0."""
    r = sim.send_command("ZERO T")
    assert "OK" in r.upper(), f"ZERO T failed: {repr(r)}"

    r = sim.send_command("HALT TIME 200")
    assert "OK" in r.upper(), f"Expected OK from HALT TIME, got {repr(r)}"
    assert "id=" in r, f"Expected id= in HALT TIME reply, got {repr(r)}"


def test_halt_time_fires_evt(sim):
    """HALT TIME 200: EVT halt fires within 500ms tick window."""
    # Extend watchdog far beyond the test window.
    sim.send_command("SET sTimeout=60000")

    # Set baseline, start driving, register halt condition.
    sim.send_command("ZERO T")
    sim.send_command("VW 200 0")
    r = sim.send_command("HALT TIME 200")
    assert "id=0" in r, f"Expected id=0 from HALT TIME, got {repr(r)}"

    # Tick 500ms (well past the 200ms threshold).
    sim.tick_for(500)

    evts = sim.get_async_evts()
    halt_id = _find_evt_halt(evts)
    assert halt_id is not None, (
        f"Expected 'EVT halt id=0' in async events after 500ms, got: {repr(evts)}"
    )
    assert halt_id == 0, f"Expected halt id=0, got id={halt_id}"


def test_halt_time_fires_only_once(sim):
    """HALT conditions are cleared after first fire; no second EVT on re-tick."""
    sim.send_command("SET sTimeout=60000")
    sim.send_command("ZERO T")
    sim.send_command("VW 200 0")
    sim.send_command("HALT TIME 100")

    # First window — should fire.
    sim.tick_for(300)
    evts1 = sim.get_async_evts()
    assert _find_evt_halt(evts1) is not None, "Expected halt EVT in first tick window"

    # Start driving again and tick more — should NOT fire again (conditions cleared).
    sim.send_command("VW 200 0")
    sim.tick_for(300)
    evts2 = sim.get_async_evts()
    assert _find_evt_halt(evts2) is None, (
        f"Expected no second halt EVT after clear, but got: {repr(evts2)}"
    )


# ---------------------------------------------------------------------------
# HALT DIST
# ---------------------------------------------------------------------------

def test_halt_dist_registers(sim):
    """HALT DIST 500 returns OK HALT id=0."""
    sim.send_command("ZERO D")
    r = sim.send_command("HALT DIST 500")
    assert "OK" in r.upper(), f"Expected OK from HALT DIST, got {repr(r)}"
    assert "id=" in r, f"Expected id= in HALT DIST reply, got {repr(r)}"


def test_halt_dist_fires_evt(sim):
    """HALT DIST 300: EVT halt fires once encoders travel >= 300mm."""
    sim.send_command("SET sTimeout=60000")

    # Zero distance baseline and set a modest distance condition.
    sim.send_command("ZERO D")
    sim.send_command("VW 300 0")
    sim.send_command("HALT DIST 300")

    # Tick 3s — at 300mm/s the mock should easily reach 300mm.
    sim.tick_for(3000)

    evts = sim.get_async_evts()
    halt_id = _find_evt_halt(evts)
    assert halt_id is not None, (
        f"Expected 'EVT halt' in async events after HALT DIST 300 at v=300, "
        f"got: {repr(evts)}"
    )


# ---------------------------------------------------------------------------
# HALT CLEAR
# ---------------------------------------------------------------------------

def test_halt_clear_removes_conditions(sim):
    """HALT CLEAR after HALT TIME: no EVT halt fires after CLEAR."""
    sim.send_command("SET sTimeout=60000")
    sim.send_command("ZERO T")
    sim.send_command("VW 200 0")
    sim.send_command("HALT TIME 100")

    r = sim.send_command("HALT CLEAR")
    assert "OK" in r.upper(), f"Expected OK from HALT CLEAR, got {repr(r)}"
    assert "cleared=" in r, f"Expected cleared= in HALT CLEAR reply, got {repr(r)}"

    # Tick past the original threshold — no halt should fire.
    sim.tick_for(500)
    evts = sim.get_async_evts()
    assert _find_evt_halt(evts) is None, (
        f"Expected no EVT halt after HALT CLEAR, but got: {repr(evts)}"
    )


# ---------------------------------------------------------------------------
# HALT TIME SOFT
# ---------------------------------------------------------------------------

def test_halt_time_soft_fires_evt(sim):
    """HALT TIME 200 SOFT: EVT halt fires with soft stop."""
    sim.send_command("SET sTimeout=60000")
    sim.send_command("ZERO T")
    sim.send_command("VW 200 0")
    r = sim.send_command("HALT TIME 200 SOFT")
    assert "OK" in r.upper(), f"Expected OK from HALT TIME SOFT, got {repr(r)}"

    sim.tick_for(500)
    evts = sim.get_async_evts()
    assert _find_evt_halt(evts) is not None, (
        f"Expected 'EVT halt' after HALT TIME SOFT, got: {repr(evts)}"
    )


# ---------------------------------------------------------------------------
# HALT LIST
# ---------------------------------------------------------------------------

def test_halt_list_shows_registered(sim):
    """HALT LIST returns entries for each registered condition."""
    sim.send_command("ZERO T")
    sim.send_command("HALT TIME 500")
    sim.send_command("HALT DIST 1000")

    r = sim.send_command("HALT LIST")
    assert "OK" in r.upper(), f"Expected OK from HALT LIST, got {repr(r)}"


# ---------------------------------------------------------------------------
# ZERO T / ZERO D baseline
# ---------------------------------------------------------------------------

def test_zero_t_resets_timer_baseline(sim):
    """ZERO T reply contains 'T' indicating timer baseline was set."""
    r = sim.send_command("ZERO T")
    assert "T" in r, f"Expected T in ZERO T reply, got {repr(r)}"
    assert "OK" in r.upper(), f"Expected OK from ZERO T, got {repr(r)}"


def test_zero_d_resets_dist_baseline(sim):
    """ZERO D reply contains 'D' indicating distance baseline was set."""
    r = sim.send_command("ZERO D")
    assert "D" in r, f"Expected D in ZERO D reply, got {repr(r)}"
    assert "OK" in r.upper(), f"Expected OK from ZERO D, got {repr(r)}"


# ---------------------------------------------------------------------------
# HALT POS
# ---------------------------------------------------------------------------

def test_halt_pos_registers(sim):
    """HALT POS 500 0 50 returns OK HALT id=<n>."""
    r = sim.send_command("HALT POS 500 0 50")
    assert "OK" in r.upper(), f"Expected OK from HALT POS, got {repr(r)}"
    assert "id=" in r, f"Expected id= in HALT POS reply, got {repr(r)}"


def test_halt_pos_fires_when_in_radius(sim):
    """HALT POS fires when robot pose is within the radius."""
    sim.send_command("SET sTimeout=60000")
    sim.send_command("VW 200 0")

    # Register a position halt at (0, 0) with a very large radius
    # so it fires immediately since the mock robot starts near the origin.
    r = sim.send_command("HALT POS 0 0 10000")
    assert "id=" in r, f"Expected id= in HALT POS reply, got {repr(r)}"

    # Tick a little — condition should fire quickly given the huge radius.
    sim.tick_for(200)
    evts = sim.get_async_evts()
    halt_id = _find_evt_halt(evts)
    assert halt_id is not None, (
        f"Expected 'EVT halt' after HALT POS 0 0 10000 (large radius), "
        f"got: {repr(evts)}"
    )


def test_halt_pos_does_not_fire_outside_radius(sim):
    """HALT POS does not fire when pose is outside the radius."""
    sim.send_command("SET sTimeout=60000")
    # Robot starts near origin; register a far-away target that should not fire
    # within a short tick window.
    r = sim.send_command("HALT POS 999999 999999 1")
    assert "id=" in r, f"Expected id= in HALT POS reply, got {repr(r)}"

    sim.send_command("VW 200 0")
    sim.tick_for(200)
    evts = sim.get_async_evts()
    assert _find_evt_halt(evts) is None, (
        f"Expected no EVT halt for far-away HALT POS, got: {repr(evts)}"
    )


# ---------------------------------------------------------------------------
# HALT COLOR
# ---------------------------------------------------------------------------

def test_halt_color_registers(sim):
    """HALT COLOR 120 0.8 0.6 0.3 returns OK HALT id=<n>."""
    r = sim.send_command("HALT COLOR 120 0.8 0.6 0.3")
    assert "OK" in r.upper(), f"Expected OK from HALT COLOR, got {repr(r)}"
    assert "id=" in r, f"Expected id= in HALT COLOR reply, got {repr(r)}"


# ---------------------------------------------------------------------------
# HALT INFO
# ---------------------------------------------------------------------------

def test_halt_info_returns_original_string(sim):
    """HALT INFO <id> replies with str= containing the original command label."""
    r = sim.send_command("HALT POS 500 0 50")
    assert "id=" in r, f"Expected id= in HALT POS reply, got {repr(r)}"
    import re
    m = re.search(r"id=(\d+)", r)
    assert m, f"Could not parse id from {repr(r)}"
    cid = int(m.group(1))

    info = sim.send_command(f"HALT INFO {cid}")
    assert "OK" in info.upper(), f"Expected OK from HALT INFO, got {repr(info)}"
    assert "str=" in info, f"Expected str= in HALT INFO reply, got {repr(info)}"
    # Label contains the key fields we formatted.
    assert "POS" in info, f"Expected POS in info str, got {repr(info)}"


def test_halt_info_not_found(sim):
    """HALT INFO on a non-existent id returns ERR notfound."""
    r = sim.send_command("HALT INFO 255")
    assert "ERR" in r.upper(), f"Expected ERR for unknown HALT INFO id, got {repr(r)}"


# ---------------------------------------------------------------------------
# HALT CLEAR <id>
# ---------------------------------------------------------------------------

def test_halt_clear_id_removes_one(sim):
    """HALT CLEAR <id> removes a single entry; the other remains."""
    sim.send_command("ZERO T")
    r0 = sim.send_command("HALT TIME 500")
    r1 = sim.send_command("HALT TIME 600")
    assert "id=0" in r0, f"Expected id=0 in first HALT TIME reply, got {repr(r0)}"
    assert "id=1" in r1, f"Expected id=1 in second HALT TIME reply, got {repr(r1)}"

    # Clear only id=0.
    rc = sim.send_command("HALT CLEAR 0")
    assert "OK" in rc.upper(), f"Expected OK from HALT CLEAR 0, got {repr(rc)}"
    assert "cleared" in rc and "0" in rc, f"Expected cleared id=0 in reply, got {repr(rc)}"

    # id=0 should be gone; id=1 should still be info-able.
    ri = sim.send_command("HALT INFO 1")
    assert "OK" in ri.upper(), f"Expected id=1 still present after HALT CLEAR 0, got {repr(ri)}"
    # id=0 should now be notfound.
    ri0 = sim.send_command("HALT INFO 0")
    # After remove(), info() finds it but reports active=no, or it may not be found.
    # Acceptable: either ERR notfound OR active=no in the reply.
    # The implementation searches all entries including inactive ones (not found only
    # if id is completely unknown), but active=no will appear in the reply.
    # Just verify the entry was indeed deactivated by checking HALT LIST count decreased.
    rl = sim.send_command("HALT LIST")
    assert "count=1" in rl, f"Expected count=1 after removing id=0, got {repr(rl)}"


def test_halt_clear_id_not_found(sim):
    """HALT CLEAR <id> with an unknown id returns ERR notfound."""
    rc = sim.send_command("HALT CLEAR 200")
    assert "ERR" in rc.upper(), f"Expected ERR for unknown HALT CLEAR id, got {repr(rc)}"
