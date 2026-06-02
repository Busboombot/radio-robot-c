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
        frame = parse_tlm("TLM t=100 vel=200,0,15")
        assert frame is not None
        assert frame.vel == (200, 0, 15)

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
        line = "TLM t=12345 mode=T enc=1024,1019 pose=350,-12,1780 vel=200,0,15 line=120,340,330,118 color=21,30,18,80"
        frame = parse_tlm(line)
        assert frame is not None
        assert frame.t == 12345
        assert frame.mode == "T"
        assert frame.enc == (1024, 1019)
        assert frame.pose == (350, -12, 1780)
        assert frame.vel == (200, 0, 15)
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
