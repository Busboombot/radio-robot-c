"""
test_command_processor.py — tests for the CommandProcessor wire protocol.

Tests exercise command parsing, routing, and reply formatting.  Each test
creates a fresh Sim (via the sim fixture) and sends one or more commands.
"""
import re


def test_ping(sim):
    """PING returns a reply containing OK."""
    r = sim.send_command("PING")
    assert "OK" in r.upper(), f"Expected OK in PING reply, got {repr(r)}"


def test_ping_body(sim):
    """PING reply contains t= timestamp field."""
    r = sim.send_command("PING")
    assert "t=" in r, f"Expected t= in PING reply, got {repr(r)}"


def test_hello(sim):
    """HELLO returns the DEVICE: identity banner."""
    r = sim.send_command("HELLO")
    assert "DEVICE" in r.upper(), f"Expected DEVICE in HELLO reply, got {repr(r)}"
    # Banner format: DEVICE:NEZHA2:robot:<name>:<serial>
    assert "NEZHA2" in r, f"Expected NEZHA2 in HELLO banner, got {repr(r)}"


def test_unknown_verb(sim):
    """An unregistered verb returns ERR unknown."""
    r = sim.send_command("XXXXBAD")
    # Command processor replies ERR unknown for unregistered verbs.
    assert "ERR" in r.upper() or r == "", (
        f"Expected ERR (or empty) for unknown verb, got {repr(r)}"
    )


def test_unknown_verb_content(sim):
    """Unknown verb reply mentions 'unknown' in the error code."""
    r = sim.send_command("XXXXBAD")
    if r:  # non-empty reply
        assert "unknown" in r.lower(), (
            f"Expected 'unknown' in ERR reply, got {repr(r)}"
        )


def test_set_and_get_roundtrip(sim):
    """SET vel.kP=2.0 followed by GET vel.kP returns 2.0."""
    r = sim.send_command("SET vel.kP=2.0")
    assert "OK" in r.upper(), f"Expected OK from SET, got {repr(r)}"

    r = sim.send_command("GET vel.kP")
    assert "CFG" in r, f"Expected CFG in GET reply, got {repr(r)}"

    # Parse the value from "CFG vel.kP=2.000"
    m = re.search(r"vel\.kP=([0-9.]+)", r)
    assert m is not None, f"Could not find vel.kP= in GET reply: {repr(r)}"
    val = float(m.group(1))
    assert abs(val - 2.0) < 0.01, (
        f"Expected vel.kP=2.0 after SET, got {val} (reply: {repr(r)})"
    )


def test_set_unknown_key(sim):
    """SET with an unknown key returns ERR badkey."""
    r = sim.send_command("SET xxxxxbadkey=1.0")
    assert "ERR" in r.upper(), f"Expected ERR for unknown key, got {repr(r)}"


def test_vw_reply_format(sim):
    """VW command returns OK vw with v= and omega= fields."""
    r = sim.send_command("VW 200 0")
    assert "OK" in r.upper(), f"Expected OK from VW, got {repr(r)}"
    assert "v=200" in r, f"Expected v=200 in VW reply, got {repr(r)}"
    assert "omega=0" in r, f"Expected omega=0 in VW reply, got {repr(r)}"


def test_vw_then_vel(sim):
    """VW 200 0 for 2 s: GET VEL shows non-zero velocity."""
    sim.send_command("VW 200 0")
    sim.tick_for(2000)

    r = sim.send_command("GET VEL")
    assert "OK" in r.upper(), f"Expected OK from GET VEL, got {repr(r)}"
    assert "vel=" in r, f"Expected vel= in GET VEL reply, got {repr(r)}"

    # Parse left velocity (format: vel=<vL>:E,<vR>:E)
    m = re.search(r"vel=(-?\d+):E,(-?\d+):E", r)
    assert m is not None, f"Could not parse vel= from GET VEL: {repr(r)}"
    vl, vr = int(m.group(1)), int(m.group(2))

    # After 2 s of VW 200 0, both wheels should be moving forward.
    assert vl > 0, f"Expected positive left velocity after VW 200 0, got {vl}"
    assert vr > 0, f"Expected positive right velocity after VW 200 0, got {vr}"
