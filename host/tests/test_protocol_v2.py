"""Unit tests for the v2 encode/parse protocol layer.

Tests verify:
- Correct v2 wire encoding of outgoing commands (no sign prefix, space-separated).
- Correct parsing of all response tag types (OK, ERR, EVT, TLM, CFG, ID).
- TLMFrame dataclass field extraction.
- NezhaProtocol method roundtrips via a mock SerialConnection.

No hardware is needed — the mock injects canned v2 response lines.
"""

from __future__ import annotations

import math
import time
from typing import Any
from unittest.mock import MagicMock, patch

import pytest

from robot_radio.robot.protocol import (
    NezhaProtocol,
    ParsedResponse,
    TLMFrame,
    parse_cfg,
    parse_response,
    parse_tlm,
    tlm_drop_rate,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mock_conn(response_lines: list[str] | None = None) -> MagicMock:
    """Create a mock SerialConnection that returns canned response lines."""
    conn = MagicMock()
    conn.is_open = True
    conn.mode = "relay"
    # Default: no responses unless overridden per call.
    responses = response_lines or []
    conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": responses}
    conn.send_fast.return_value = None
    conn.read_lines.return_value = []
    conn.read_pending_lines.return_value = []
    return conn


def _proto(response_lines: list[str] | None = None) -> tuple[NezhaProtocol, MagicMock]:
    """Return (NezhaProtocol, mock_conn) pair with canned responses."""
    conn = _mock_conn(response_lines)
    return NezhaProtocol(conn), conn


# ===========================================================================
# parse_response
# ===========================================================================

class TestParseResponse:
    """Unit tests for the module-level parse_response() function."""

    def test_ok_simple(self) -> None:
        r = parse_response("OK stop")
        assert r is not None
        assert r.tag == "OK"
        assert r.tokens == ["stop"]
        assert r.kv == {}
        assert r.corr_id is None

    def test_ok_with_kv(self) -> None:
        r = parse_response("OK pong t=12345")
        assert r is not None
        assert r.tag == "OK"
        assert r.tokens == ["pong"]
        assert r.kv == {"t": "12345"}

    def test_ok_drive_kv(self) -> None:
        r = parse_response("OK drive l=200 r=150")
        assert r is not None
        assert r.tag == "OK"
        assert r.tokens == ["drive"]
        assert r.kv == {"l": "200", "r": "150"}

    def test_ok_with_corr_id(self) -> None:
        r = parse_response("OK pong t=9999 #7")
        assert r is not None
        assert r.corr_id == "7"
        assert r.kv == {"t": "9999"}

    def test_err_simple(self) -> None:
        r = parse_response("ERR badarg")
        assert r is not None
        assert r.tag == "ERR"
        assert r.tokens == ["badarg"]

    def test_err_with_detail(self) -> None:
        r = parse_response("ERR badkey foo")
        assert r is not None
        assert r.tag == "ERR"
        assert r.tokens == ["badkey", "foo"]

    def test_evt_done_T(self) -> None:
        r = parse_response("EVT done T")
        assert r is not None
        assert r.tag == "EVT"
        assert r.tokens == ["done", "T"]

    def test_evt_done_D(self) -> None:
        r = parse_response("EVT done D")
        assert r is not None
        assert r.tag == "EVT"
        assert r.tokens == ["done", "D"]

    def test_evt_done_G(self) -> None:
        r = parse_response("EVT done G")
        assert r is not None
        assert r.tag == "EVT"
        assert r.tokens == ["done", "G"]

    def test_evt_safety_stop(self) -> None:
        r = parse_response("EVT safety_stop")
        assert r is not None
        assert r.tag == "EVT"
        assert r.tokens == ["safety_stop"]

    def test_id_response(self) -> None:
        r = parse_response("ID model=Nezha2 name=GUTOV serial=12345 fw=1.0 proto=2 caps=otos,line")
        assert r is not None
        assert r.tag == "ID"
        assert r.kv["model"] == "Nezha2"
        assert r.kv["name"] == "GUTOV"
        assert r.kv["proto"] == "2"

    def test_cfg_response(self) -> None:
        r = parse_response("CFG ml=0.487 mr=0.481 tw=120")
        assert r is not None
        assert r.tag == "CFG"
        assert r.kv == {"ml": "0.487", "mr": "0.481", "tw": "120"}

    def test_relay_prefix_stripped(self) -> None:
        # Lines arriving through the relay have a '<' prefix.
        r = parse_response("< OK stop")
        assert r is not None
        assert r.tag == "OK"

    def test_unrecognised_returns_none(self) -> None:
        assert parse_response("GARBAGE line") is None

    def test_empty_line_returns_none(self) -> None:
        assert parse_response("") is None
        assert parse_response("   ") is None


# ===========================================================================
# parse_tlm
# ===========================================================================

class TestParseTLM:
    """Unit tests for parse_tlm()."""

    def test_full_tlm_frame(self) -> None:
        line = "TLM t=12345 mode=S enc=1024,1019 pose=350,-12,1780 line=120,340,330,118 color=21,30,18,80"
        frame = parse_tlm(line)
        assert frame is not None
        assert frame.t == 12345
        assert frame.mode == "S"
        assert frame.enc == (1024, 1019)
        assert frame.pose == (350, -12, 1780)
        assert frame.line == (120, 340, 330, 118)
        assert frame.color == (21, 30, 18, 80)

    def test_enc_only(self) -> None:
        frame = parse_tlm("TLM t=500 enc=100,95")
        assert frame is not None
        assert frame.enc == (100, 95)
        assert frame.pose is None
        assert frame.line is None

    def test_pose_only(self) -> None:
        frame = parse_tlm("TLM t=1000 pose=200,-50,9000")
        assert frame is not None
        assert frame.pose == (200, -50, 9000)
        assert frame.enc is None

    def test_negative_encoder_values(self) -> None:
        frame = parse_tlm("TLM t=100 enc=-50,-48")
        assert frame is not None
        assert frame.enc == (-50, -48)

    def test_vel_field(self) -> None:
        # vel= is 2-value format: vL_mmps, vR_mmps (Sprint 010, Ticket 007)
        frame = parse_tlm("TLM t=100 vel=200,195")
        assert frame is not None
        assert frame.vel == (200, 195)

    def test_vel_field_old_3value_ignored(self) -> None:
        # A 3-value vel= (old format) is not parsed — vel stays None.
        frame = parse_tlm("TLM t=100 vel=200,0,15")
        assert frame is not None
        assert frame.vel is None

    def test_non_tlm_returns_none(self) -> None:
        assert parse_tlm("OK stop") is None
        assert parse_tlm("EVT done T") is None
        assert parse_tlm("") is None

    def test_t_field_missing(self) -> None:
        # TLM without 't' is still valid — t stays None.
        frame = parse_tlm("TLM enc=10,10")
        assert frame is not None
        assert frame.t is None
        assert frame.enc == (10, 10)


# ===========================================================================
# parse_cfg
# ===========================================================================

class TestParseCFG:
    """Unit tests for parse_cfg()."""

    def test_full_cfg(self) -> None:
        d = parse_cfg("CFG ml=0.487 mr=0.481 tw=120 sTimeout=200 pid.kp=2.000")
        assert d is not None
        assert d["ml"] == "0.487"
        assert d["tw"] == "120"
        assert d["pid.kp"] == "2.000"

    def test_non_cfg_returns_none(self) -> None:
        assert parse_cfg("OK stop") is None
        assert parse_cfg("TLM t=1 enc=0,0") is None

    def test_empty_cfg(self) -> None:
        # Bare "CFG" with no keys → empty dict.
        d = parse_cfg("CFG")
        assert d == {}


# ===========================================================================
# NezhaProtocol command encoding (wire format verification)
# ===========================================================================

class TestCommandEncoding:
    """Verify v2 wire encoding: space-separated, no sign prefix, verb upper-cased."""

    # ── drive ────────────────────────────────────────────────────────────────

    def test_drive_positive(self) -> None:
        proto, conn = _proto()
        proto.drive(200, 150)
        conn.send_fast.assert_called_once_with("S 200 150")

    def test_drive_negative(self) -> None:
        proto, conn = _proto()
        proto.drive(-100, -80)
        conn.send_fast.assert_called_once_with("S -100 -80")

    def test_drive_mixed_sign(self) -> None:
        proto, conn = _proto()
        proto.drive(200, -150)
        conn.send_fast.assert_called_once_with("S 200 -150")

    def test_drive_zero(self) -> None:
        proto, conn = _proto()
        proto.drive(0, 0)
        conn.send_fast.assert_called_once_with("S 0 0")

    # ── stop ─────────────────────────────────────────────────────────────────

    def test_stop_sends_STOP(self) -> None:
        proto, conn = _proto()
        proto.stop()
        conn.send_fast.assert_called_once_with("STOP")

    # ── timed ────────────────────────────────────────────────────────────────

    def test_timed_positive(self) -> None:
        proto, conn = _proto(["OK drive l=200 r=200 ms=1000"])
        proto.timed(200, 200, 1000)
        conn.send.assert_called_once_with("T 200 200 1000", read_ms=300)

    def test_timed_negative_speeds(self) -> None:
        proto, conn = _proto(["OK drive l=-100 r=-100 ms=500"])
        proto.timed(-100, -100, 500)
        conn.send.assert_called_once_with("T -100 -100 500", read_ms=300)

    # ── distance ─────────────────────────────────────────────────────────────

    def test_distance(self) -> None:
        proto, conn = _proto(["OK drive l=200 r=200 mm=300"])
        proto.distance(200, 200, 300)
        conn.send.assert_called_once_with("D 200 200 300", read_ms=300)

    # ── go_to ────────────────────────────────────────────────────────────────

    def test_go_to(self) -> None:
        proto, conn = _proto(["OK goto x=300 y=0 speed=200"])
        proto.go_to(300, 0, 200)
        conn.send.assert_called_once_with("G 300 0 200", read_ms=300)

    # ── grip ─────────────────────────────────────────────────────────────────

    def test_grip_with_angle(self) -> None:
        proto, conn = _proto(["OK grip deg=90"])
        result = proto.grip(90)
        conn.send.assert_called_once_with("GRIP 90", read_ms=300)
        assert result == 90

    def test_grip_query(self) -> None:
        proto, conn = _proto(["OK grip deg=45"])
        result = proto.grip()
        conn.send.assert_called_once_with("GRIP", read_ms=300)
        assert result == 45

    # ── zero ─────────────────────────────────────────────────────────────────

    def test_zero_encoders(self) -> None:
        proto, conn = _proto()
        proto.zero_encoders()
        conn.send.assert_called_once_with("ZERO enc", read_ms=200)

    def test_zero_otos(self) -> None:
        proto, conn = _proto()
        proto.zero_otos()
        conn.send.assert_called_once_with("ZERO pose", read_ms=200)

    def test_zero_all(self) -> None:
        proto, conn = _proto()
        proto.zero_all()
        conn.send.assert_called_once_with("ZERO enc pose", read_ms=200)

    # ── stream ───────────────────────────────────────────────────────────────

    def test_stream_period(self) -> None:
        proto, conn = _proto(["OK stream period=40"])
        proto.stream(40)
        conn.send.assert_called_once_with("STREAM 40", read_ms=300)

    def test_stream_off(self) -> None:
        proto, conn = _proto(["OK stream period=0"])
        proto.stream(0)
        conn.send.assert_called_once_with("STREAM 0", read_ms=300)

    def test_stream_fields(self) -> None:
        proto, conn = _proto(["OK stream fields=enc,pose"])
        proto.stream_fields("enc,pose")
        conn.send.assert_called_once_with("STREAM fields=enc,pose", read_ms=300)

    # ── OTOS commands ─────────────────────────────────────────────────────────

    def test_otos_init(self) -> None:
        proto, conn = _proto(["OK oi"])
        proto.otos_init()
        conn.send.assert_called_once_with("OI", read_ms=500)

    def test_otos_zero(self) -> None:
        proto, conn = _proto(["OK oz"])
        proto.otos_zero()
        conn.send.assert_called_once_with("OZ", read_ms=200)

    def test_otos_reset_tracking(self) -> None:
        proto, conn = _proto(["OK or"])
        proto.otos_reset_tracking()
        conn.send.assert_called_once_with("OR", read_ms=200)

    def test_otos_set_linear_scalar(self) -> None:
        proto, conn = _proto(["OK linear scalar=5"])
        val = proto.otos_set_linear_scalar(5)
        conn.send.assert_called_once_with("OL 5", read_ms=500)
        assert val == 5

    def test_otos_get_linear_scalar(self) -> None:
        proto, conn = _proto(["OK linear scalar=3"])
        val = proto.otos_get_linear_scalar()
        conn.send.assert_called_once_with("OL", read_ms=300)
        assert val == 3

    # ── J-port ───────────────────────────────────────────────────────────────

    def test_port_read(self) -> None:
        proto, conn = _proto(["OK port p=1 v=0"])
        val = proto.port_read(1)
        conn.send.assert_called_once_with("P 1", read_ms=300)
        assert val == 0

    def test_port_write(self) -> None:
        proto, conn = _proto(["OK port p=2 v=1"])
        proto.port_write(2, True)
        conn.send.assert_called_once_with("P 2 1", read_ms=200)

    def test_port_write_false(self) -> None:
        proto, conn = _proto()
        proto.port_write(3, False)
        conn.send.assert_called_once_with("P 3 0", read_ms=200)

    def test_port_read_analog(self) -> None:
        proto, conn = _proto(["OK aport p=1 v=512"])
        val = proto.port_read_analog(1)
        conn.send.assert_called_once_with("PA 1", read_ms=300)
        assert val == 512

    def test_port_write_analog(self) -> None:
        proto, conn = _proto()
        proto.port_write_analog(1, 255)
        conn.send.assert_called_once_with("PA 1 255", read_ms=200)

    # ── vw ───────────────────────────────────────────────────────────────────

    def test_vw_straight(self) -> None:
        """vw(200, 0) → send_fast 'VW 200 0'."""
        proto, conn = _proto()
        proto.vw(200, 0)
        conn.send_fast.assert_called_once_with("VW 200 0")

    def test_vw_spin_ccw(self) -> None:
        """vw(0, 500) → send_fast 'VW 0 500'."""
        proto, conn = _proto()
        proto.vw(0, 500)
        conn.send_fast.assert_called_once_with("VW 0 500")

    def test_vw_spin_cw(self) -> None:
        """vw(0, -500) → send_fast 'VW 0 -500'."""
        proto, conn = _proto()
        proto.vw(0, -500)
        conn.send_fast.assert_called_once_with("VW 0 -500")

    def test_vw_curved_arc(self) -> None:
        """vw(200, 300) → send_fast 'VW 200 300'."""
        proto, conn = _proto()
        proto.vw(200, 300)
        conn.send_fast.assert_called_once_with("VW 200 300")

    def test_vw_negative_v(self) -> None:
        """vw(-200, 0) → send_fast 'VW -200 0'."""
        proto, conn = _proto()
        proto.vw(-200, 0)
        conn.send_fast.assert_called_once_with("VW -200 0")

    def test_vw_with_corr_id(self) -> None:
        """vw(200, 0, corr_id='7') → send_fast 'VW 200 0 #7'."""
        proto, conn = _proto()
        proto.vw(200, 0, corr_id="7")
        conn.send_fast.assert_called_once_with("VW 200 0 #7")

    def test_vw_at_max_omega(self) -> None:
        """vw(0, 3142) → send_fast 'VW 0 3142' (≈π rad/s)."""
        proto, conn = _proto()
        proto.vw(0, 3142)
        conn.send_fast.assert_called_once_with("VW 0 3142")

    def test_vw_at_min_omega(self) -> None:
        """vw(0, -3142) → send_fast 'VW 0 -3142'."""
        proto, conn = _proto()
        proto.vw(0, -3142)
        conn.send_fast.assert_called_once_with("VW 0 -3142")

    def test_vw_zero(self) -> None:
        """vw(0, 0) → send_fast 'VW 0 0' (stationary keepalive)."""
        proto, conn = _proto()
        proto.vw(0, 0)
        conn.send_fast.assert_called_once_with("VW 0 0")

    def test_vw_uses_send_fast(self) -> None:
        """vw() uses fire-and-forget (send_fast), not blocking send."""
        proto, conn = _proto()
        proto.vw(200, 0)
        conn.send_fast.assert_called_once()
        conn.send.assert_not_called()

    def test_vw_reply_parsed_ok(self) -> None:
        """OK vw v=200 omega=0 is a valid parsed response."""
        r = parse_response("OK vw v=200 omega=0")
        assert r is not None
        assert r.tag == "OK"
        assert r.tokens == ["vw"]
        assert r.kv == {"v": "200", "omega": "0"}

    def test_vw_reply_with_corr_id_parsed(self) -> None:
        """OK vw v=200 omega=0 #7 extracts corr_id='7'."""
        r = parse_response("OK vw v=200 omega=0 #7")
        assert r is not None
        assert r.corr_id == "7"
        assert r.kv == {"v": "200", "omega": "0"}


# ===========================================================================
# NezhaProtocol response parsing
# ===========================================================================

class TestResponseParsing:
    """Verify that NezhaProtocol methods correctly parse v2 responses."""

    def test_ping_returns_t_and_rtt(self) -> None:
        proto, conn = _proto(["OK pong t=12345"])
        result = proto.ping()
        assert result is not None
        t_robot_ms, rtt_ms = result
        assert t_robot_ms == 12345
        assert rtt_ms >= 0.0  # monotonic time ensures non-negative

    def test_ping_no_response_returns_none(self) -> None:
        proto, conn = _proto([])
        result = proto.ping()
        assert result is None

    def test_ping_with_corr_id(self) -> None:
        proto, conn = _proto(["OK pong t=500 #42"])
        result = proto.ping(corr_id="42")
        assert result is not None
        assert result[0] == 500
        conn.send.assert_called_once_with("PING #42", read_ms=500)

    def test_echo(self) -> None:
        proto, conn = _proto(["OK echo hello world"])
        result = proto.echo("hello world")
        conn.send.assert_called_once_with("ECHO hello world", read_ms=500)
        assert result == "hello world"

    def test_get_id(self) -> None:
        proto, conn = _proto(["ID model=Nezha2 name=GUTOV serial=12345 fw=1.0 proto=2 caps=otos"])
        result = proto.get_id()
        assert result is not None
        assert result["model"] == "Nezha2"
        assert result["proto"] == "2"

    def test_get_config_full(self) -> None:
        proto, conn = _proto(["CFG ml=0.487 mr=0.481 tw=120"])
        result = proto.get_config()
        conn.send.assert_called_once_with("GET", read_ms=500)
        assert result is not None
        assert result["ml"] == "0.487"
        assert result["tw"] == "120"

    def test_get_config_keys(self) -> None:
        proto, conn = _proto(["CFG ml=0.487 pid.kp=2.000"])
        result = proto.get_config("ml", "pid.kp")
        conn.send.assert_called_once_with("GET ml pid.kp", read_ms=500)
        assert result is not None
        assert result["ml"] == "0.487"

    def test_get_config_no_response_returns_none(self) -> None:
        proto, conn = _proto([])
        result = proto.get_config()
        assert result is None

    def test_set_config_float(self) -> None:
        proto, conn = _proto(["OK set ml=0.487"])
        result = proto.set_config(ml=0.487)
        # Verify the sent command contains the key=value pair.
        called_cmd = conn.send.call_args[0][0]
        assert called_cmd.startswith("SET ")
        assert "ml=" in called_cmd
        assert result is not None
        assert result["ml"] == "0.487"

    def test_set_config_int(self) -> None:
        proto, conn = _proto(["OK set sTimeout=200"])
        result = proto.set_config(sTimeout=200)
        called_cmd = conn.send.call_args[0][0]
        assert "sTimeout=200" in called_cmd
        assert result is not None

    def test_set_config_multiple_keys(self) -> None:
        proto, conn = _proto(["OK set ml=0.487 mr=0.481"])
        result = proto.set_config(ml=0.487, mr=0.481)
        called_cmd = conn.send.call_args[0][0]
        assert called_cmd.startswith("SET ")
        assert "ml=" in called_cmd
        assert "mr=" in called_cmd

    def test_set_config_no_kwargs_returns_none(self) -> None:
        proto, conn = _proto()
        result = proto.set_config()
        assert result is None
        conn.send.assert_not_called()


# ===========================================================================
# EVT done parsing
# ===========================================================================

class TestEVTDone:
    """Verify EVT done / EVT safety_stop are parsed correctly."""

    # --- Bare EVT (no corr id) ---

    def test_evt_done_T(self) -> None:
        r = parse_response("EVT done T")
        assert r is not None
        assert r.tag == "EVT"
        assert r.tokens[0] == "done"
        assert r.tokens[1] == "T"
        assert r.corr_id is None

    def test_evt_done_D(self) -> None:
        r = parse_response("EVT done D")
        assert r is not None
        assert r.tokens == ["done", "D"]
        assert r.corr_id is None

    def test_evt_done_G(self) -> None:
        r = parse_response("EVT done G")
        assert r is not None
        assert r.tokens == ["done", "G"]
        assert r.corr_id is None

    def test_evt_safety_stop(self) -> None:
        r = parse_response("EVT safety_stop")
        assert r is not None
        assert r.tag == "EVT"
        assert r.tokens == ["safety_stop"]
        assert r.corr_id is None

    # --- Correlated EVT (originating command carried #id) ---

    def test_evt_done_T_with_corr_id(self) -> None:
        """EVT done T #12 — parse_response extracts corr_id='12'."""
        r = parse_response("EVT done T #12")
        assert r is not None
        assert r.tag == "EVT"
        assert r.tokens == ["done", "T"]
        assert r.corr_id == "12"

    def test_evt_done_D_with_corr_id(self) -> None:
        """EVT done D #5 — corr_id extracted."""
        r = parse_response("EVT done D #5")
        assert r is not None
        assert r.tokens == ["done", "D"]
        assert r.corr_id == "5"

    def test_evt_done_G_with_corr_id(self) -> None:
        """EVT done G #99 — corr_id extracted."""
        r = parse_response("EVT done G #99")
        assert r is not None
        assert r.tokens == ["done", "G"]
        assert r.corr_id == "99"

    def test_evt_safety_stop_with_corr_id(self) -> None:
        """EVT safety_stop #3 — corr_id extracted."""
        r = parse_response("EVT safety_stop #3")
        assert r is not None
        assert r.tokens == ["safety_stop"]
        assert r.corr_id == "3"

    # --- wait_for_evt_done ---

    def test_wait_for_evt_done_returns_done(self) -> None:
        proto, conn = _proto()
        conn.read_lines.return_value = ["EVT done T"]
        outcome = proto.wait_for_evt_done("T", timeout_ms=1000)
        assert outcome == "done"

    def test_wait_for_evt_done_safety_stop(self) -> None:
        proto, conn = _proto()
        conn.read_lines.return_value = ["EVT safety_stop"]
        outcome = proto.wait_for_evt_done("T", timeout_ms=1000)
        assert outcome == "safety_stop"

    def test_wait_for_evt_done_timeout(self) -> None:
        proto, conn = _proto()
        conn.read_lines.return_value = []  # no replies ever
        outcome = proto.wait_for_evt_done("T", timeout_ms=10)  # very short
        assert outcome == "timeout"

    def test_wait_for_evt_done_with_matching_corr_id(self) -> None:
        """wait_for_evt_done accepts EVT done T #12 when corr_id='12'."""
        proto, conn = _proto()
        conn.read_lines.return_value = ["EVT done T #12"]
        outcome = proto.wait_for_evt_done("T", timeout_ms=1000, corr_id="12")
        assert outcome == "done"

    def test_wait_for_evt_done_skips_wrong_corr_id(self) -> None:
        """wait_for_evt_done skips EVT done T #99 when waiting for corr_id='12'."""
        import itertools
        proto, conn = _proto()
        # Always return the wrong id — the filter should skip it and we time out.
        conn.read_lines.side_effect = itertools.repeat(["EVT done T #99"])
        outcome = proto.wait_for_evt_done("T", timeout_ms=20, corr_id="12")
        # Should not accept the wrong id → times out.
        assert outcome == "timeout"

    def test_wait_for_evt_done_accepts_bare_evt_when_filtering(self) -> None:
        """Bare EVT done T (no id) is accepted even when a corr_id filter is set."""
        proto, conn = _proto()
        conn.read_lines.return_value = ["EVT done T"]
        outcome = proto.wait_for_evt_done("T", timeout_ms=1000, corr_id="12")
        assert outcome == "done"

    def test_wait_for_evt_done_corr_id_safety_stop(self) -> None:
        """EVT safety_stop #5 is accepted when waiting for corr_id='5'."""
        proto, conn = _proto()
        conn.read_lines.return_value = ["EVT safety_stop #5"]
        outcome = proto.wait_for_evt_done("T", timeout_ms=1000, corr_id="5")
        assert outcome == "safety_stop"


# ===========================================================================
# v1 artifacts must be absent
# ===========================================================================

class TestNoV1Artifacts:
    """Verify that all v1 protocol code has been removed."""

    def test_no_sign_function(self) -> None:
        """_sign() helper must not exist in protocol module."""
        import robot_radio.robot.protocol as proto_mod
        assert not hasattr(proto_mod, "_sign"), \
            "_sign() is a v1 artifact and must not exist in the v2 protocol module"

    def test_no_sign_in_nezha(self) -> None:
        """_sign() must not exist in the nezha module."""
        import robot_radio.robot.nezha as nezha_mod
        assert not hasattr(nezha_mod, "_sign"), \
            "_sign() is a v1 artifact and must not exist in the v2 nezha module"

    def test_drive_is_space_separated(self) -> None:
        """S command must use space-separated integers, not sign-packed."""
        proto, conn = _proto()
        proto.drive(200, 150)
        cmd = conn.send_fast.call_args[0][0]
        # Must be "S 200 150", not "S+200+150" or "S+200-150"
        assert cmd == "S 200 150"
        assert "+" not in cmd, f"v1 sign prefix '+' found in command: {cmd!r}"

    def test_stop_is_STOP_not_X(self) -> None:
        """stop() must send STOP, not the v1 'X' command."""
        proto, conn = _proto()
        proto.stop()
        cmd = conn.send_fast.call_args[0][0]
        assert cmd == "STOP"
        assert cmd != "X"

    def test_no_parse_enc_v1(self) -> None:
        """parse_enc() (v1 standalone ENC line parser) must not exist."""
        import robot_radio.robot.protocol as proto_mod
        assert not hasattr(proto_mod, "parse_enc"), \
            "parse_enc() is a v1 artifact"

    def test_no_parse_so_v1(self) -> None:
        """parse_so() (v1 SO odometry parser) must not exist."""
        import robot_radio.robot.protocol as proto_mod
        assert not hasattr(proto_mod, "parse_so"), \
            "parse_so() is a v1 artifact"

    def test_no_parse_ls_v1(self) -> None:
        """parse_ls() (v1 LS line sensor parser) must not exist."""
        import robot_radio.robot.protocol as proto_mod
        assert not hasattr(proto_mod, "parse_ls"), \
            "parse_ls() is a v1 artifact"

    def test_no_parse_cs_v1(self) -> None:
        """parse_cs() (v1 CS color sensor parser) must not exist."""
        import robot_radio.robot.protocol as proto_mod
        assert not hasattr(proto_mod, "parse_cs"), \
            "parse_cs() is a v1 artifact"


# ===========================================================================
# TLMFrame edge cases
# ===========================================================================

class TestTLMFrameEdgeCases:
    """Edge-case tests for TLMFrame parsing."""

    def test_tlm_with_relay_prefix(self) -> None:
        """TLM lines arriving via relay have a '<' prefix."""
        frame = parse_tlm("< TLM t=500 enc=100,95")
        assert frame is not None
        assert frame.enc == (100, 95)

    def test_tlm_all_fields(self) -> None:
        # vel= uses 2-value format vL,vR (Sprint 010, Ticket 007)
        line = "TLM t=12345 mode=T enc=1024,1019 pose=350,-12,1780 vel=200,195 line=120,340,330,118 color=21,30,18,80"
        frame = parse_tlm(line)
        assert frame is not None
        assert frame.t == 12345
        assert frame.mode == "T"
        assert frame.enc == (1024, 1019)
        assert frame.pose == (350, -12, 1780)
        assert frame.vel == (200, 195)
        assert frame.line == (120, 340, 330, 118)
        assert frame.color == (21, 30, 18, 80)

    def test_tlm_bad_enc_ignored(self) -> None:
        """Malformed enc field → enc stays None."""
        frame = parse_tlm("TLM t=100 enc=notanumber,95")
        assert frame is not None
        assert frame.enc is None

    def test_tlm_bad_pose_ignored(self) -> None:
        """Malformed pose field → pose stays None."""
        frame = parse_tlm("TLM t=100 pose=x,y,z")
        assert frame is not None
        assert frame.pose is None

    def test_tlm_negative_pose(self) -> None:
        """Negative x/y/heading are valid."""
        frame = parse_tlm("TLM t=100 pose=-200,-50,-9000")
        assert frame is not None
        assert frame.pose == (-200, -50, -9000)


# ===========================================================================
# T002 — ticket-required new tests
# Explicit assertions for v2 wire encodings, wait_for_evt_done paths,
# TLM partial/full parsing, and liveness preflight.
# ===========================================================================

class TestPingEncoding:
    """test_ping_encoding — PING command sends 'PING\n' on the wire."""

    def test_ping_encoding(self) -> None:
        """ping() must send the literal string 'PING' (no-arg form) via conn.send."""
        proto, conn = _proto(["OK pong t=1"])
        proto.ping()
        called_cmd = conn.send.call_args[0][0]
        assert called_cmd == "PING"

    def test_ping_encoding_with_corr_id(self) -> None:
        """ping(corr_id='3') must send 'PING #3' on the wire."""
        proto, conn = _proto(["OK pong t=1 #3"])
        proto.ping(corr_id="3")
        called_cmd = conn.send.call_args[0][0]
        assert called_cmd == "PING #3"


class TestIdEncoding:
    """test_id_encoding — ID command sends 'ID\n' on the wire."""

    def test_id_encoding(self) -> None:
        """get_id() must send the literal string 'ID' via conn.send."""
        proto, conn = _proto(["ID model=Nezha2 name=TOVEZ serial=0 fw=2.0 proto=2"])
        proto.get_id()
        called_cmd = conn.send.call_args[0][0]
        assert called_cmd == "ID"


class TestVerEncoding:
    """test_ver_encoding — VER command sends 'VER\n' on the wire."""

    def test_ver_encoding(self) -> None:
        """get_ver() must send the literal string 'VER' via conn.send."""
        proto, conn = _proto(["OK ver fw=2.0 proto=2"])
        result = proto.get_ver()
        called_cmd = conn.send.call_args[0][0]
        assert called_cmd == "VER"
        assert result is not None
        assert result["fw"] == "2.0"
        assert result["proto"] == "2"

    def test_ver_no_response_returns_none(self) -> None:
        """get_ver() returns None when firmware gives no response."""
        proto, conn = _proto([])
        result = proto.get_ver()
        assert result is None


class TestDriveSpaceDelimited:
    """test_drive_space_delimited — S command uses space-separated integers."""

    def test_drive_space_delimited(self) -> None:
        """drive(100, -50) must encode as 'S 100 -50' (space-separated, no sign prefix)."""
        proto, conn = _proto()
        proto.drive(100, -50)
        cmd = conn.send_fast.call_args[0][0]
        assert cmd == "S 100 -50"
        assert "+" not in cmd
        # Tokens: must be exactly 3 space-separated parts
        parts = cmd.split()
        assert parts == ["S", "100", "-50"]


class TestTimedSpaceDelimited:
    """test_timed_space_delimited — T command uses space-separated integers."""

    def test_timed_space_delimited(self) -> None:
        """timed(100, 100, 1000) must encode as 'T 100 100 1000'."""
        proto, conn = _proto(["OK drive l=100 r=100 ms=1000"])
        proto.timed(100, 100, 1000)
        called_cmd = conn.send.call_args[0][0]
        assert called_cmd == "T 100 100 1000"
        parts = called_cmd.split()
        assert parts == ["T", "100", "100", "1000"]


class TestDistanceSpaceDelimited:
    """test_distance_space_delimited — D command uses space-separated integers."""

    def test_distance_space_delimited(self) -> None:
        """distance(100, 100, 900) must encode as 'D 100 100 900'."""
        proto, conn = _proto(["OK drive l=100 r=100 mm=900"])
        proto.distance(100, 100, 900)
        called_cmd = conn.send.call_args[0][0]
        assert called_cmd == "D 100 100 900"
        parts = called_cmd.split()
        assert parts == ["D", "100", "100", "900"]


class TestZeroEncoding:
    """test_zero_enc / test_zero_pose — ZERO verb uses v2 sub-token format."""

    def test_zero_enc(self) -> None:
        """zero_encoders() must send 'ZERO enc' (not 'EZ' or 'ZERO ENC')."""
        proto, conn = _proto()
        proto.zero_encoders()
        called_cmd = conn.send.call_args[0][0]
        assert called_cmd == "ZERO enc"
        assert "EZ" not in called_cmd

    def test_zero_pose(self) -> None:
        """zero_otos() must send 'ZERO pose' (not 'SZ' or 'ZERO POSE')."""
        proto, conn = _proto()
        proto.zero_otos()
        called_cmd = conn.send.call_args[0][0]
        assert called_cmd == "ZERO pose"
        assert "SZ" not in called_cmd


class TestWaitForEvtDonePaths:
    """test_wait_for_evt_done_success / test_wait_for_evt_done_safety_stop."""

    def test_wait_for_evt_done_success(self) -> None:
        """wait_for_evt_done returns 'done' when EVT done T is received."""
        proto, conn = _proto()
        conn.read_lines.return_value = ["EVT done T"]
        result = proto.wait_for_evt_done("T", timeout_ms=1000)
        assert result == "done"

    def test_wait_for_evt_done_safety_stop(self) -> None:
        """wait_for_evt_done returns 'safety_stop' when EVT safety_stop is received."""
        proto, conn = _proto()
        conn.read_lines.return_value = ["EVT safety_stop"]
        result = proto.wait_for_evt_done("T", timeout_ms=1000)
        assert result == "safety_stop"

    def test_wait_for_evt_done_wrong_verb_keeps_waiting(self) -> None:
        """wait_for_evt_done('T') ignores EVT done D — only accepts matching verb."""
        import itertools
        proto, conn = _proto()
        # Always return the wrong verb — should not match and we time out.
        conn.read_lines.side_effect = itertools.repeat(["EVT done D"])
        result = proto.wait_for_evt_done("T", timeout_ms=20)
        assert result == "timeout"

    def test_wait_for_evt_done_timeout_returns_timeout(self) -> None:
        """wait_for_evt_done returns 'timeout' when no matching event arrives."""
        proto, conn = _proto()
        conn.read_lines.return_value = []
        result = proto.wait_for_evt_done("T", timeout_ms=10)
        assert result == "timeout"

    def test_wait_for_evt_done_corr_id_success(self) -> None:
        """wait_for_evt_done with corr_id accepts EVT done T #7 when id='7'."""
        proto, conn = _proto()
        conn.read_lines.return_value = ["EVT done T #7"]
        result = proto.wait_for_evt_done("T", timeout_ms=1000, corr_id="7")
        assert result == "done"

    def test_wait_for_evt_done_corr_id_safety_stop(self) -> None:
        """wait_for_evt_done with corr_id accepts EVT safety_stop #7 when id='7'."""
        proto, conn = _proto()
        conn.read_lines.return_value = ["EVT safety_stop #7"]
        result = proto.wait_for_evt_done("T", timeout_ms=1000, corr_id="7")
        assert result == "safety_stop"


class TestParseTLMExtended:
    """test_parse_tlm_partial / test_parse_tlm_full — explicit TLM parsing assertions."""

    def test_parse_tlm_partial(self) -> None:
        """TLM with only t= and enc= fields: pose must be None (partial frame)."""
        frame = parse_tlm("TLM t=100 enc=10,10")
        assert frame is not None
        assert frame.t == 100
        assert frame.enc == (10, 10)
        assert frame.pose is None
        assert frame.vel is None
        assert frame.line is None
        assert frame.color is None

    def test_parse_tlm_full(self) -> None:
        """Full TLM line: all fields must be populated."""
        line = "TLM t=99999 mode=S enc=200,198 pose=500,100,18000 vel=150,148 line=10,20,30,40 color=5,10,15,80"
        frame = parse_tlm(line)
        assert frame is not None
        assert frame.t == 99999
        assert frame.mode == "S"
        assert frame.enc == (200, 198)
        assert frame.pose == (500, 100, 18000)
        assert frame.vel == (150, 148)
        assert frame.line == (10, 20, 30, 40)
        assert frame.color == (5, 10, 15, 80)

    def test_parse_tlm_enc_and_pose_no_vel(self) -> None:
        """TLM with enc and pose but no vel: vel must be None."""
        frame = parse_tlm("TLM t=1000 enc=50,48 pose=100,-20,3600")
        assert frame is not None
        assert frame.enc == (50, 48)
        assert frame.pose == (100, -20, 3600)
        assert frame.vel is None

    def test_parse_tlm_mode_field_T(self) -> None:
        """mode=T (timed drive in progress) is parsed correctly."""
        frame = parse_tlm("TLM t=500 mode=T enc=100,100")
        assert frame is not None
        assert frame.mode == "T"

    def test_parse_tlm_mode_field_D(self) -> None:
        """mode=D (distance drive in progress) is parsed correctly."""
        frame = parse_tlm("TLM t=500 mode=D enc=200,198 pose=300,0,0")
        assert frame is not None
        assert frame.mode == "D"


class TestLivenessPreflight:
    """test_liveness_preflight — ping + get_id sequence succeeds with canned responses."""

    def test_liveness_preflight_success(self) -> None:
        """Preflight sequence: ping() returns (t, rtt); get_id() returns robot identity."""
        # Each method call uses conn.send separately, so configure side_effect.
        conn = MagicMock()
        conn.is_open = True
        conn.mode = "relay"
        conn.send.side_effect = [
            {"sent": "PING", "mode": "relay", "responses": ["OK pong t=1"]},
            {"sent": "ID", "mode": "relay", "responses": ["ID model=Nezha2 name=TOVEZ serial=0 fw=2.0 proto=2 caps=otos,line"]},
        ]
        conn.send_fast.return_value = None
        conn.read_lines.return_value = []

        proto = NezhaProtocol(conn)

        # Step 1: ping
        ping_result = proto.ping()
        assert ping_result is not None
        t_robot, rtt = ping_result
        assert t_robot == 1
        assert rtt >= 0.0

        # Step 2: get_id
        id_result = proto.get_id()
        assert id_result is not None
        assert id_result["model"] == "Nezha2"
        assert id_result["name"] == "TOVEZ"
        assert id_result["proto"] == "2"

    def test_liveness_preflight_ping_failure(self) -> None:
        """Preflight: ping() returns None when firmware gives no pong."""
        conn = _mock_conn([])
        proto = NezhaProtocol(conn)
        result = proto.ping()
        assert result is None

    def test_liveness_preflight_id_failure(self) -> None:
        """Preflight: get_id() returns None when firmware gives no ID line."""
        conn = _mock_conn([])
        proto = NezhaProtocol(conn)
        result = proto.get_id()
        assert result is None


class TestStreamDriveKeepalive:
    """stream_drive keepalive cadence: resend at ~30% of watchdog_ms."""

    def test_stream_drive_resends_within_keepalive_window(self) -> None:
        """stream_drive resends S command at <= 30% of watchdog_ms interval.

        Uses a fake monotonic clock and a controlled read_lines to verify that
        the keepalive fires before the watchdog_ms deadline.
        """
        import itertools

        conn = MagicMock()
        conn.is_open = True
        conn.mode = "relay"
        # MUST return a parseable line, NOT []. The real read_lines(duration_ms)
        # BLOCKS ~50 ms on serial; a MagicMock returning [] returns instantly, so
        # stream_drive's `while True` loop (which only exits on safety_stop) spins
        # at full CPU and NEVER yields — next(gen) never returns. Each spin also
        # records a MagicMock call in call_args_list, which grows without bound
        # → multi-GB memory blow-up and a hung test. Returning a non-safety line
        # makes the generator yield once per iteration so next(gen) returns.
        conn.read_lines.return_value = ["TLM t=1 enc=0,0"]
        conn.send.return_value = {"sent": "STREAM 40", "mode": "relay", "responses": ["OK stream period=40"]}
        conn.send_fast.return_value = None

        proto = NezhaProtocol(conn)

        watchdog_ms = 500
        keepalive_threshold_s = watchdog_ms * 0.30 / 1000.0  # 0.150 s

        speeds = [100, 100]

        # Collect all send_fast calls by iterating the generator for a limited
        # number of steps, then close it.
        gen = proto.stream_drive(speeds, period_ms=40, watchdog_ms=watchdog_ms)

        # Let the generator run for just enough time to trigger at least one
        # keepalive resend (sleep slightly longer than the keepalive threshold).
        import threading

        results: list[int] = []

        def _run_gen() -> None:
            try:
                for _ in range(5):
                    next(gen)
            except StopIteration:
                pass
            finally:
                gen.close()

        # The real-time keepalive depends on wall clock, so we allow enough
        # real time to elapse.  Use a short sleep between iterations via
        # a patched monotonic that advances time quickly.
        fake_time = [0.0]

        original_monotonic = time.monotonic

        def fake_monotonic() -> float:
            fake_time[0] += 0.06  # advance 60 ms per call (> 30% of 500 ms after 3 calls)
            return fake_time[0]

        import robot_radio.robot.protocol as proto_mod

        original = proto_mod.time.monotonic  # type: ignore[attr-defined]
        proto_mod.time.monotonic = fake_monotonic  # type: ignore[attr-defined]
        try:
            _run_gen()
        finally:
            proto_mod.time.monotonic = original  # type: ignore[attr-defined]

        # send_fast was called: first send (initial S) + at least one resend.
        assert conn.send_fast.call_count >= 2, (
            f"Expected >= 2 send_fast calls (initial + keepalive resend), "
            f"got {conn.send_fast.call_count}"
        )
        # All calls to send_fast should be 'S 100 100' or 'STOP'.
        for call in conn.send_fast.call_args_list:
            cmd = call[0][0]
            assert cmd.startswith("S ") or cmd == "STOP", f"Unexpected send_fast: {cmd!r}"

    def test_stream_drive_sends_stop_on_close(self) -> None:
        """stream_drive sends STOP when generator is closed."""
        conn = _mock_conn([])
        conn.send.return_value = {"sent": "STREAM 40", "mode": "relay", "responses": []}
        # See note in test_stream_drive_resends_*: read_lines must yield a line,
        # not [], or next(gen) spins forever (no yield) and the mock accumulates
        # calls until OOM. One non-safety line lets next(gen) yield and return.
        conn.read_lines.return_value = ["TLM t=1 enc=0,0"]
        proto = NezhaProtocol(conn)

        speeds = [200, 200]
        gen = proto.stream_drive(speeds, period_ms=40, watchdog_ms=500)
        # Start the generator, then immediately close it.
        try:
            next(gen)
        except StopIteration:
            pass
        gen.close()

        # STOP should have been sent on GeneratorExit.
        fast_calls = [c[0][0] for c in conn.send_fast.call_args_list]
        assert "STOP" in fast_calls, f"Expected STOP in send_fast calls: {fast_calls}"

    def test_stream_drive_stops_on_safety_stop(self) -> None:
        """stream_drive ends naturally when EVT safety_stop is received."""
        import itertools

        conn = MagicMock()
        conn.is_open = True
        conn.mode = "relay"
        conn.send.return_value = {"sent": "STREAM 40", "mode": "relay", "responses": []}
        conn.send_fast.return_value = None
        # First call returns a safety_stop event; subsequent calls return nothing.
        conn.read_lines.side_effect = itertools.chain(
            [["EVT safety_stop"]],
            itertools.repeat([]),
        )

        proto = NezhaProtocol(conn)
        speeds = [100, 100]
        gen = proto.stream_drive(speeds, period_ms=40, watchdog_ms=500)

        frames = list(gen)  # Should terminate when safety_stop is seen.
        # No frames yielded (safety_stop causes return before yield).
        # The generator should have exited cleanly.
        assert frames == []


# ===========================================================================
# D10 seq field + tlm_drop_rate (ticket 028-005)
# ===========================================================================

class TestTLMSeqField:
    """Tests for TLMFrame.seq parsing (D10 firmware 028-005)."""

    def test_seq_parsed_from_tlm_line(self) -> None:
        """seq= field is parsed into TLMFrame.seq as an int."""
        frame = parse_tlm("TLM t=1000 mode=I seq=42")
        assert frame is not None
        assert frame.seq == 42

    def test_seq_absent_on_old_firmware(self) -> None:
        """TLM lines without seq= leave TLMFrame.seq as None."""
        frame = parse_tlm("TLM t=1000 mode=S enc=100,95")
        assert frame is not None
        assert frame.seq is None

    def test_seq_zero(self) -> None:
        """seq=0 is valid."""
        frame = parse_tlm("TLM t=0 mode=I seq=0")
        assert frame is not None
        assert frame.seq == 0

    def test_seq_large_value(self) -> None:
        """seq near uint16 max (65535) is parsed correctly."""
        frame = parse_tlm("TLM t=100 mode=I seq=65534")
        assert frame is not None
        assert frame.seq == 65534

    def test_seq_with_other_fields(self) -> None:
        """seq= coexists correctly with enc, pose, and mode fields."""
        frame = parse_tlm("TLM t=500 mode=D seq=7 enc=200,198 pose=100,-5,900")
        assert frame is not None
        assert frame.seq == 7
        assert frame.enc == (200, 198)
        assert frame.pose == (100, -5, 900)
        assert frame.mode == "D"

    def test_seq_bad_value_ignored(self) -> None:
        """Non-integer seq= value leaves seq as None."""
        frame = parse_tlm("TLM t=100 seq=notanumber")
        assert frame is not None
        assert frame.seq is None


class TestTlmDropRate:
    """Tests for tlm_drop_rate() helper (D10 028-005).

    These tests use synthetic TLMFrame objects.  Real-robot drop-rate
    measurement is DEFERRED — stakeholder field test.
    """

    def _frames(self, seqs: list[int | None]) -> list[TLMFrame]:
        """Build a list of TLMFrame objects with the given seq values."""
        return [TLMFrame(seq=s) for s in seqs]

    def test_zero_frames(self) -> None:
        assert tlm_drop_rate([]) == 0.0

    def test_one_frame(self) -> None:
        assert tlm_drop_rate(self._frames([5])) == 0.0

    def test_no_drops_consecutive(self) -> None:
        """Consecutive seq numbers → 0% drop rate."""
        frames = self._frames([0, 1, 2, 3, 4])
        assert tlm_drop_rate(frames) == 0.0

    def test_one_dropped_frame(self) -> None:
        """Gap of 2 in seq (one dropped frame between adjacent receives)."""
        # seq 0, 2: expected 0,1,2 — 1 dropped out of 2 expected gaps.
        frames = self._frames([0, 2])
        assert tlm_drop_rate(frames) == pytest.approx(0.5)

    def test_all_consecutive_long(self) -> None:
        """Long sequence with no gaps → 0.0."""
        frames = self._frames(list(range(100)))
        assert tlm_drop_rate(frames) == 0.0

    def test_every_other_dropped(self) -> None:
        """Receive every other frame (50% drop rate)."""
        # seq 0, 2, 4, 6: each gap=2, 1 dropped per 2 expected.
        frames = self._frames([0, 2, 4, 6])
        assert tlm_drop_rate(frames) == pytest.approx(0.5)

    def test_all_seq_none(self) -> None:
        """All frames have seq=None (pre-D10 firmware) → 0.0."""
        frames = self._frames([None, None, None])
        assert tlm_drop_rate(frames) == 0.0

    def test_mixed_none_and_seq(self) -> None:
        """Frames with seq=None are skipped; remaining are evaluated."""
        # None frames are excluded; [0, 1, 2] → no drops.
        frames = self._frames([None, 0, None, 1, 2])
        assert tlm_drop_rate(frames) == 0.0

    def test_uint16_wrap_around(self) -> None:
        """uint16 wrap-around from 65535 to 0 is counted as gap=1 (not a drop)."""
        frames = self._frames([65534, 65535, 0, 1])
        assert tlm_drop_rate(frames) == 0.0

    def test_uint16_wrap_with_drops(self) -> None:
        """Wrap-around with one dropped frame across the boundary."""
        # 65535 → 1: expected 65535, 0, 1 → gap=2, 1 dropped.
        frames = self._frames([65535, 1])
        assert tlm_drop_rate(frames) == pytest.approx(0.5)

    def test_large_gap(self) -> None:
        """Large gap → high drop rate."""
        # seq 0 → 100: 99 dropped out of 100 expected.
        frames = self._frames([0, 100])
        assert tlm_drop_rate(frames) == pytest.approx(0.99)

    def test_under_2pct_threshold(self) -> None:
        """Simulate field-test scenario: 1% drop rate over 1200 frames.

        DEFERRED — real-robot validation is a stakeholder field test.
        This verifies the arithmetic at the expected field-test scale.
        """
        # 1200 frames, 12 dropped (1%)
        seqs = list(range(1200))
        # Simulate 12 evenly-spaced drops by removing every 100th frame.
        received = [s for s in seqs if s % 100 != 50]
        frames = self._frames(received)
        rate = tlm_drop_rate(frames)
        assert rate < 0.02, f"Expected rate < 2%, got {rate:.3f}"
