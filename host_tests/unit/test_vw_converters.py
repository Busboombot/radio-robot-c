"""
test_vw_converters.py — tests for S/T/D/G/R/TURN VW converters and OP cached state.

Sprint 020, Ticket 011.  Updated sprint 026, Ticket 001.

The sim now wires a CommandQueue into both CommandProcessor and MotionController
(matching LoopScheduler's constructor wiring), so converter commands (S, T, D,
G, R, TURN, RT) travel the queue path on the next sim_tick() — the same path as
hardware.  These tests verify the observable behaviour is correct on the queue path.

OP cached-state tests verify the handleOP implementation reads from
HardwareState (hwState->otosX/Y/H) rather than calling the device.
"""
import pytest


# ---------------------------------------------------------------------------
# OP — cached-state read
# ---------------------------------------------------------------------------

def test_op_returns_ok_with_xyz_fields(sim):
    """OP replies OK op x=... y=... h=... from cached state."""
    r = sim.send_command("OP")
    assert r.strip(), f"OP returned empty reply"
    # Must start with OK (no device call = no nodev error even without OTOS init)
    assert r.upper().startswith("OK"), f"Expected OK from OP, got {repr(r)}"
    assert "x=" in r, f"Expected x= field in OP reply, got {repr(r)}"
    assert "y=" in r, f"Expected y= field in OP reply, got {repr(r)}"
    assert "h=" in r, f"Expected h= field in OP reply, got {repr(r)}"


def test_op_no_device_call_needed(sim):
    """OP does not require OTOS to be initialized — reads cached state."""
    # Send OP without ever initializing OTOS.  Old code would return ERR nodev;
    # new code reads hwState directly and returns OK.
    r = sim.send_command("OP")
    assert "ERR" not in r.upper(), (
        f"OP should not require OTOS init (reads cached state), got {repr(r)}"
    )
    assert "OK" in r.upper(), f"Expected OK from OP, got {repr(r)}"


def test_op_returns_zeros_at_boot(sim):
    """OP returns x=0 y=0 h=0 at boot (OTOS pose not yet written)."""
    r = sim.send_command("OP")
    # hwState starts zeroed; expect all three fields to be 0.
    assert "x=0" in r, f"Expected x=0 in OP reply at boot, got {repr(r)}"
    assert "y=0" in r, f"Expected y=0 in OP reply at boot, got {repr(r)}"
    assert "h=0" in r, f"Expected h=0 in OP reply at boot, got {repr(r)}"


# ---------------------------------------------------------------------------
# S — stream command (queue path: S → pushVW → dequeueOne → beginStream)
# ---------------------------------------------------------------------------

def test_s_command_replies_ok(sim):
    """S command replies OK drive l=... r=... even with converter refactor."""
    r = sim.send_command("S 200 200")
    assert "OK" in r.upper(), f"Expected OK from S, got {repr(r)}"
    assert "drive" in r.lower(), f"Expected 'drive' in S reply, got {repr(r)}"


def test_s_command_drives_motors(sim):
    """S 200 200 sets motors running (encoders grow)."""
    sim.send_command("S 200 200")
    sim.tick_for(500)
    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r = float(sim._lib.sim_get_enc_r(sim._h))
    assert enc_l > 0.0 and enc_r > 0.0, (
        f"Expected encoders to grow after S 200 200, got enc_l={enc_l}, enc_r={enc_r}"
    )


# ---------------------------------------------------------------------------
# T — timed command (queue path: T → pushVW → dequeueOne → beginTimed)
# ---------------------------------------------------------------------------

def test_t_command_replies_ok(sim):
    """T command replies OK drive l=... r=... ms=..."""
    r = sim.send_command("T 200 200 2000")
    assert "OK" in r.upper(), f"Expected OK from T, got {repr(r)}"
    assert "drive" in r.lower(), f"Expected 'drive' in T reply, got {repr(r)}"


def test_t_command_emits_done_evt(sim):
    """T command emits EVT done T upon completion."""
    sim.send_command("T 200 200 500")
    sim.tick_for(5000)
    evts = sim.get_async_evts()
    assert "EVT done T" in evts, (
        f"Expected 'EVT done T' in async EVTs after T, got {repr(evts)}"
    )


def test_t_command_motors_stop_after_timeout(sim):
    """T command: motors stop after timeout expires."""
    sim.send_command("T 200 200 500")
    sim.tick_for(5000)
    pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
    assert pwm_l == 0.0, f"Expected motor stopped after T timeout, pwm_l={pwm_l}"


# ---------------------------------------------------------------------------
# D — distance command (queue path: D → pushVW → dequeueOne → distanceDrive)
# ---------------------------------------------------------------------------

def test_d_command_replies_ok(sim):
    """D command replies OK drive l=... r=... mm=..."""
    r = sim.send_command("D 200 200 300")
    assert "OK" in r.upper(), f"Expected OK from D, got {repr(r)}"
    assert "drive" in r.lower(), f"Expected 'drive' in D reply, got {repr(r)}"


def test_d_command_emits_done_evt_unchanged(sim):
    """D command still emits EVT done D (unchanged behaviour after refactor)."""
    sim.send_command("D 200 200 200")
    sim.tick_for(10000)
    evts = sim.get_async_evts()
    assert "EVT done D" in evts, (
        f"Expected 'EVT done D' after D command, got {repr(evts)}"
    )


# ---------------------------------------------------------------------------
# R — arc command (queue path: R → pushVW → dequeueOne → beginArc)
# ---------------------------------------------------------------------------

def test_r_command_replies_ok(sim):
    """R command replies OK arc speed=... radius=..."""
    r = sim.send_command("R 200 500")
    assert "OK" in r.upper(), f"Expected OK from R, got {repr(r)}"
    assert "arc" in r.lower(), f"Expected 'arc' in R reply, got {repr(r)}"


def test_r_command_drives_motors(sim):
    """R 200 500 drives motors (encoders grow)."""
    sim.send_command("R 200 500")
    sim.tick_for(500)
    enc_l = float(sim._lib.sim_get_enc_l(sim._h))
    enc_r = float(sim._lib.sim_get_enc_r(sim._h))
    assert enc_l > 0.0 or enc_r > 0.0, (
        f"Expected encoder to grow after R 200 500, enc_l={enc_l}, enc_r={enc_r}"
    )


# ---------------------------------------------------------------------------
# TURN — heading rotation (queue path: TURN → pushVW → dequeueOne → beginTurn)
# ---------------------------------------------------------------------------

def test_turn_command_replies_ok(sim):
    """TURN command replies OK turn heading=... eps=..."""
    r = sim.send_command("TURN 9000")
    assert "OK" in r.upper(), f"Expected OK from TURN, got {repr(r)}"
    assert "turn" in r.lower(), f"Expected 'turn' in TURN reply, got {repr(r)}"


def test_turn_command_emits_done_evt(sim):
    """TURN command emits EVT done TURN upon arrival at heading."""
    sim.send_command("TURN 0")   # 0 deg = no rotation needed → fires quickly
    sim.tick_for(5000)
    evts = sim.get_async_evts()
    assert "EVT done TURN" in evts, (
        f"Expected 'EVT done TURN' after TURN 0, got {repr(evts)}"
    )


# ---------------------------------------------------------------------------
# G — go-to command (queue path: G → pushVW → dequeueOne → beginGoTo)
# ---------------------------------------------------------------------------

def test_g_command_replies_ok(sim):
    """G command replies OK goto x=... y=... speed=..."""
    r = sim.send_command("G 0 0 200")
    assert "OK" in r.upper(), f"Expected OK from G, got {repr(r)}"
    assert "goto" in r.lower(), f"Expected 'goto' in G reply, got {repr(r)}"


def test_g_command_emits_done_evt(sim):
    """G 0 0 200 (target at origin) emits EVT done G quickly."""
    sim.send_command("G 0 0 200")
    sim.tick_for(5000)
    evts = sim.get_async_evts()
    assert "EVT done G" in evts, (
        f"Expected 'EVT done G' from G 0 0 200 (target at origin), got {repr(evts)}"
    )
