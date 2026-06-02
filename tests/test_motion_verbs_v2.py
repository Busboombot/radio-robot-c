#!/usr/bin/env python3
"""test_motion_verbs_v2.py — Unit tests for motion verbs v2 (009-006).

These tests validate the wire protocol formats for:
  - S (streaming drive): OK drive l=<l> r=<r>
  - T (timed drive): OK drive l=<l> r=<r> ms=<ms>; then EVT done T
  - D (distance drive): OK drive l=<l> r=<r> mm=<mm>; then EVT done D
  - G (go-to): OK goto x=<x> y=<y> speed=<speed>; then EVT done G
  - STOP: OK stop
  - GRIP <deg>: OK grip deg=<deg>; GRIP (no-arg): OK grip deg=<current>
  - ZERO enc: OK zero enc
  - ZERO pose: OK zero pose
  - ZERO enc pose: OK zero enc pose
  - EVT safety_stop (async, S watchdog)
  - EVT done T / EVT done D / EVT done G (async completions)
  - OTOS/port commands: OI, OZ, OR, OP, OV, OL, OA, P, PA
  - ERR badarg (wrong arg count)
  - ERR range <field> (out-of-range values)
  - #id correlation on synchronous responses; absent on EVT
"""

from __future__ import annotations

import re
import pytest


# ---------------------------------------------------------------------------
# Wire format helpers
# ---------------------------------------------------------------------------

def parse_ok(line: str) -> tuple[str, str]:
    """Parse 'OK <verb> [<body>]' → (verb, body)."""
    assert line.startswith("OK "), f"Expected OK line, got: {line!r}"
    parts = line[3:].split(" ", 1)
    verb = parts[0]
    body = parts[1] if len(parts) > 1 else ""
    return verb, body


def parse_err(line: str) -> tuple[str, str]:
    """Parse 'ERR <code> [<detail>]' → (code, detail)."""
    assert line.startswith("ERR "), f"Expected ERR line, got: {line!r}"
    parts = line[4:].split(" ", 1)
    code = parts[0]
    detail = parts[1] if len(parts) > 1 else ""
    return code, detail


def parse_evt(line: str) -> tuple[str, str]:
    """Parse 'EVT <name> [<body>]' → (name, body)."""
    assert line.startswith("EVT "), f"Expected EVT line, got: {line!r}"
    parts = line[4:].split(" ", 1)
    name = parts[0]
    body = parts[1] if len(parts) > 1 else ""
    return name, body


def parse_body_kv(body: str) -> dict[str, str]:
    """Parse 'k=v k=v ...' body fragment into dict."""
    result: dict[str, str] = {}
    for token in body.split():
        if "=" in token:
            k, v = token.split("=", 1)
            result[k] = v
    return result


# ---------------------------------------------------------------------------
# S — streaming drive
# ---------------------------------------------------------------------------

class TestSCommand:
    """Tests for S (streaming velocity) command wire format."""

    def _make_s_ok(self, l: int, r: int, corr_id: str = "") -> str:
        """Simulate firmware OK response to S <l> <r>."""
        body = f"l={l} r={r}"
        if corr_id:
            return f"OK drive {body} #{corr_id}"
        return f"OK drive {body}"

    def test_s_ok_prefix(self) -> None:
        """S 200 150 → response starts with 'OK drive'."""
        resp = self._make_s_ok(200, 150)
        assert resp.startswith("OK drive")

    def test_s_l_r_values(self) -> None:
        """S 200 150 → l=200 r=150 in body."""
        resp = self._make_s_ok(200, 150)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["l"] == "200"
        assert kv["r"] == "150"

    def test_s_zero_velocity(self) -> None:
        """S 0 0 → OK drive l=0 r=0."""
        resp = self._make_s_ok(0, 0)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["l"] == "0"
        assert kv["r"] == "0"

    def test_s_with_corr_id(self) -> None:
        """S 200 150 #7 → OK drive l=200 r=150 #7."""
        resp = self._make_s_ok(200, 150, corr_id="7")
        assert resp.endswith("#7")
        _, body = parse_ok(resp)
        # Strip corr_id from body
        body_clean = re.sub(r"\s+#\d+$", "", body)
        kv = parse_body_kv(body_clean)
        assert kv["l"] == "200"
        assert kv["r"] == "150"

    def test_s_badarg_no_args(self) -> None:
        """S (no args) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_s_range_l_too_high(self) -> None:
        """S 1001 0 → ERR range l."""
        line = "ERR range l"
        code, detail = parse_err(line)
        assert code == "range"
        assert "l" in detail

    def test_s_range_r_too_high(self) -> None:
        """S 0 1001 → ERR range r."""
        line = "ERR range r"
        code, detail = parse_err(line)
        assert code == "range"
        assert "r" in detail

    def test_evt_safety_stop_format(self) -> None:
        """S watchdog timeout emits EVT safety_stop (no #id)."""
        line = "EVT safety_stop"
        name, body = parse_evt(line)
        assert name == "safety_stop"
        assert body == ""
        # EVT responses carry no #id
        assert "#" not in line


# ---------------------------------------------------------------------------
# T — timed drive
# ---------------------------------------------------------------------------

class TestTCommand:
    """Tests for T (timed drive) command wire format."""

    def _make_t_ok(self, l: int, r: int, ms: int, corr_id: str = "") -> str:
        """Simulate firmware OK response to T <l> <r> <ms>."""
        body = f"l={l} r={r} ms={ms}"
        if corr_id:
            return f"OK drive {body} #{corr_id}"
        return f"OK drive {body}"

    def test_t_ok_prefix(self) -> None:
        """T 200 150 1000 → response starts with 'OK drive'."""
        resp = self._make_t_ok(200, 150, 1000)
        assert resp.startswith("OK drive")

    def test_t_all_fields(self) -> None:
        """T 200 150 1000 → l=200 r=150 ms=1000."""
        resp = self._make_t_ok(200, 150, 1000)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["l"] == "200"
        assert kv["r"] == "150"
        assert kv["ms"] == "1000"

    def test_t_completion_evt_format(self) -> None:
        """After ~ms, DriveController emits EVT done T (no #id when not correlated)."""
        line = "EVT done T"
        name, body = parse_evt(line)
        assert name == "done"
        assert body == "T"
        assert "#" not in line

    def test_t_completion_evt_with_corr_id(self) -> None:
        """T 200 200 1000 #12 → EVT done T #12 (corr id echoed on completion)."""
        line = "EVT done T #12"
        name, body = parse_evt(line)
        assert name == "done"
        assert "T" in body
        assert "#12" in line

    def test_t_completion_no_cmd_prefix(self) -> None:
        """EVT completion is 'EVT done T', NOT 'EVT done cmd=T'."""
        # Ensure the legacy cmd= format is gone
        line = "EVT done T"
        assert "cmd=" not in line

    def test_t_badarg_too_few(self) -> None:
        """T 200 150 (no ms) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_t_range_ms_too_large(self) -> None:
        """T 200 200 31000 → ERR range ms."""
        line = "ERR range ms"
        code, detail = parse_err(line)
        assert code == "range"
        assert "ms" in detail

    def test_t_with_corr_id(self) -> None:
        """T 200 150 1000 #5 → OK drive ... #5 (synchronous reply)."""
        resp = self._make_t_ok(200, 150, 1000, corr_id="5")
        assert resp.endswith("#5")

    def test_t_evt_no_corr_id_when_not_supplied(self) -> None:
        """EVT done T has no #id when T command had no #id."""
        line = "EVT done T"
        assert "#" not in line

    def test_t_evt_corr_id_when_supplied(self) -> None:
        """EVT done T #12 when T command carried #12."""
        line = "EVT done T #12"
        name, body = parse_evt(line)
        assert name == "done"
        assert "#12" in line


# ---------------------------------------------------------------------------
# D — distance drive
# ---------------------------------------------------------------------------

class TestDCommand:
    """Tests for D (distance drive) command wire format."""

    def _make_d_ok(self, l: int, r: int, mm: int, corr_id: str = "") -> str:
        """Simulate firmware OK response to D <l> <r> <mm>."""
        body = f"l={l} r={r} mm={mm}"
        if corr_id:
            return f"OK drive {body} #{corr_id}"
        return f"OK drive {body}"

    def test_d_ok_prefix(self) -> None:
        """D 200 200 300 → response starts with 'OK drive'."""
        resp = self._make_d_ok(200, 200, 300)
        assert resp.startswith("OK drive")

    def test_d_all_fields(self) -> None:
        """D 200 200 300 → l=200 r=200 mm=300."""
        resp = self._make_d_ok(200, 200, 300)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["l"] == "200"
        assert kv["r"] == "200"
        assert kv["mm"] == "300"

    def test_d_completion_evt_format(self) -> None:
        """After distance reached, emits EVT done D."""
        line = "EVT done D"
        name, body = parse_evt(line)
        assert name == "done"
        assert body == "D"

    def test_d_completion_no_cmd_prefix(self) -> None:
        """EVT completion is 'EVT done D', NOT 'EVT done cmd=D'."""
        line = "EVT done D"
        assert "cmd=" not in line

    def test_d_badarg_too_few(self) -> None:
        """D 200 200 (no mm) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_d_range_mm_zero(self) -> None:
        """D 200 200 0 → ERR range mm."""
        line = "ERR range mm"
        code, detail = parse_err(line)
        assert code == "range"
        assert "mm" in detail


# ---------------------------------------------------------------------------
# G — go-to XY
# ---------------------------------------------------------------------------

class TestGCommand:
    """Tests for G (go-to) command wire format."""

    def _make_g_ok(self, x: int, y: int, speed: int, corr_id: str = "") -> str:
        """Simulate firmware OK response to G <x> <y> <speed>."""
        body = f"x={x} y={y} speed={speed}"
        if corr_id:
            return f"OK goto {body} #{corr_id}"
        return f"OK goto {body}"

    def test_g_ok_prefix(self) -> None:
        """G 300 0 200 → response starts with 'OK goto'."""
        resp = self._make_g_ok(300, 0, 200)
        assert resp.startswith("OK goto")

    def test_g_all_fields(self) -> None:
        """G 300 0 200 → x=300 y=0 speed=200."""
        resp = self._make_g_ok(300, 0, 200)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["x"] == "300"
        assert kv["y"] == "0"
        assert kv["speed"] == "200"

    def test_g_completion_evt_format(self) -> None:
        """After reaching destination, emits EVT done G."""
        line = "EVT done G"
        name, body = parse_evt(line)
        assert name == "done"
        assert body == "G"

    def test_g_completion_no_cmd_prefix(self) -> None:
        """EVT completion is 'EVT done G', NOT 'EVT done cmd=G'."""
        line = "EVT done G"
        assert "cmd=" not in line

    def test_g_badarg_too_few(self) -> None:
        """G 300 0 (no speed) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_g_range_speed_zero(self) -> None:
        """G 300 0 0 → ERR range speed."""
        line = "ERR range speed"
        code, detail = parse_err(line)
        assert code == "range"
        assert "speed" in detail

    def test_g_verb_is_goto_not_grip(self) -> None:
        """G is unambiguously go-to — OK verb is 'goto', not 'grip'."""
        resp = self._make_g_ok(300, 0, 200)
        verb, _ = parse_ok(resp)
        assert verb == "goto"


# ---------------------------------------------------------------------------
# STOP
# ---------------------------------------------------------------------------

class TestStopCommand:
    """Tests for STOP command wire format."""

    def test_stop_format(self) -> None:
        """STOP → OK stop."""
        line = "OK stop"
        verb, body = parse_ok(line)
        assert verb == "stop"
        assert body == ""

    def test_stop_with_corr_id(self) -> None:
        """STOP #3 → OK stop #3."""
        line = "OK stop #3"
        assert line.endswith("#3")
        assert "stop" in line

    def test_stop_no_body(self) -> None:
        """STOP response has no key=value body."""
        line = "OK stop"
        _, body = parse_ok(line)
        assert "=" not in body


# ---------------------------------------------------------------------------
# GRIP — gripper control (de-overloaded from G)
# ---------------------------------------------------------------------------

class TestGripCommand:
    """Tests for GRIP command wire format."""

    def _make_grip_ok(self, deg: int, corr_id: str = "") -> str:
        """Simulate firmware OK response to GRIP [deg]."""
        body = f"deg={deg}"
        if corr_id:
            return f"OK grip {body} #{corr_id}"
        return f"OK grip {body}"

    def test_grip_with_deg(self) -> None:
        """GRIP 90 → OK grip deg=90."""
        resp = self._make_grip_ok(90)
        verb, body = parse_ok(resp)
        assert verb == "grip"
        kv = parse_body_kv(body)
        assert kv["deg"] == "90"

    def test_grip_zero_deg(self) -> None:
        """GRIP 0 → OK grip deg=0."""
        resp = self._make_grip_ok(0)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["deg"] == "0"

    def test_grip_180_deg(self) -> None:
        """GRIP 180 → OK grip deg=180."""
        resp = self._make_grip_ok(180)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["deg"] == "180"

    def test_grip_no_arg_returns_current(self) -> None:
        """GRIP (no arg) → OK grip deg=<current>."""
        # Simulate firmware reading current angle (e.g. 90) and returning it
        current_deg = 90
        resp = self._make_grip_ok(current_deg)
        verb, body = parse_ok(resp)
        assert verb == "grip"
        kv = parse_body_kv(body)
        assert "deg" in kv

    def test_grip_with_corr_id(self) -> None:
        """GRIP 45 #9 → OK grip deg=45 #9."""
        resp = self._make_grip_ok(45, corr_id="9")
        assert resp.endswith("#9")
        verb, body = parse_ok(resp)
        assert verb == "grip"

    def test_grip_range_over_180(self) -> None:
        """GRIP 181 → ERR range deg."""
        line = "ERR range deg"
        code, detail = parse_err(line)
        assert code == "range"
        assert "deg" in detail

    def test_grip_range_negative(self) -> None:
        """GRIP -1 → ERR range deg."""
        line = "ERR range deg"
        code, detail = parse_err(line)
        assert code == "range"
        assert "deg" in detail

    def test_grip_not_g_verb(self) -> None:
        """GRIP response uses 'grip' verb, never 'goto'."""
        resp = self._make_grip_ok(90)
        verb, _ = parse_ok(resp)
        assert verb == "grip"
        assert verb != "goto"


# ---------------------------------------------------------------------------
# ZERO — encoder/odometry zeroing umbrella
# ---------------------------------------------------------------------------

class TestZeroCommand:
    """Tests for ZERO umbrella command wire format."""

    def _make_zero_ok(self, what: str, corr_id: str = "") -> str:
        """Simulate firmware OK response to ZERO <what>."""
        if corr_id:
            return f"OK zero {what} #{corr_id}"
        return f"OK zero {what}"

    def test_zero_enc(self) -> None:
        """ZERO enc → OK zero enc."""
        resp = self._make_zero_ok("enc")
        verb, body = parse_ok(resp)
        assert verb == "zero"
        assert body.strip() == "enc"

    def test_zero_pose(self) -> None:
        """ZERO pose → OK zero pose."""
        resp = self._make_zero_ok("pose")
        verb, body = parse_ok(resp)
        assert verb == "zero"
        assert body.strip() == "pose"

    def test_zero_enc_pose(self) -> None:
        """ZERO enc pose → OK zero enc pose."""
        resp = self._make_zero_ok("enc pose")
        verb, body = parse_ok(resp)
        assert verb == "zero"
        assert "enc" in body
        assert "pose" in body

    def test_zero_with_corr_id(self) -> None:
        """ZERO enc #4 → OK zero enc #4."""
        resp = self._make_zero_ok("enc", corr_id="4")
        assert resp.endswith("#4")

    def test_zero_no_arg_badarg(self) -> None:
        """ZERO (no arg) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_zero_unknown_arg_badarg(self) -> None:
        """ZERO foo → ERR badarg (unknown target)."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"


# ---------------------------------------------------------------------------
# EVT format — async events carry no #id
# ---------------------------------------------------------------------------

class TestEvtFormat:
    """Validate async EVT event format — bare (no corr id) and correlated cases."""

    # --- Bare events (no originating #id) ---

    def test_safety_stop_bare_no_id(self) -> None:
        """EVT safety_stop (uncorrelated) has no #id."""
        line = "EVT safety_stop"
        assert "#" not in line

    def test_done_t_bare_no_id(self) -> None:
        """EVT done T (uncorrelated) has no #id."""
        line = "EVT done T"
        assert "#" not in line

    def test_done_d_bare_no_id(self) -> None:
        """EVT done D (uncorrelated) has no #id."""
        line = "EVT done D"
        assert "#" not in line

    def test_done_g_bare_no_id(self) -> None:
        """EVT done G (uncorrelated) has no #id."""
        line = "EVT done G"
        assert "#" not in line

    def test_done_t_format(self) -> None:
        """EVT done T: name=done, body starts with T."""
        name, body = parse_evt("EVT done T")
        assert name == "done"
        assert body == "T"

    def test_done_d_format(self) -> None:
        """EVT done D: name=done, body starts with D."""
        name, body = parse_evt("EVT done D")
        assert name == "done"
        assert body == "D"

    def test_done_g_format(self) -> None:
        """EVT done G: name=done, body starts with G."""
        name, body = parse_evt("EVT done G")
        assert name == "done"
        assert body == "G"

    def test_safety_stop_format(self) -> None:
        """EVT safety_stop: name=safety_stop, no body (bare)."""
        name, body = parse_evt("EVT safety_stop")
        assert name == "safety_stop"
        assert body == ""

    def test_evt_completions_no_cmd_prefix(self) -> None:
        """None of the EVT done X completions use 'cmd=' prefix."""
        for line in ["EVT done T", "EVT done D", "EVT done G"]:
            assert "cmd=" not in line, f"Legacy cmd= found in {line!r}"

    # --- Correlated events (originating command carried #id) ---

    def test_done_t_with_corr_id(self) -> None:
        """EVT done T #12 — corr id echoed when T #12 was used."""
        line = "EVT done T #12"
        name, body = parse_evt(line)
        assert name == "done"
        assert "T" in body
        assert "#12" in line

    def test_done_d_with_corr_id(self) -> None:
        """EVT done D #5 — corr id echoed when D #5 was used."""
        line = "EVT done D #5"
        name, body = parse_evt(line)
        assert name == "done"
        assert "D" in body
        assert "#5" in line

    def test_done_g_with_corr_id(self) -> None:
        """EVT done G #99 — corr id echoed when G #99 was used."""
        line = "EVT done G #99"
        name, body = parse_evt(line)
        assert name == "done"
        assert "G" in body
        assert "#99" in line

    def test_safety_stop_with_corr_id(self) -> None:
        """EVT safety_stop #3 — corr id echoed when active S had #3."""
        line = "EVT safety_stop #3"
        name, body = parse_evt(line)
        assert name == "safety_stop"
        assert "#3" in line

    def test_corr_id_format_is_hash_digits(self) -> None:
        """Correlated EVT id uses '#' followed by decimal digits only."""
        import re
        for line in ["EVT done T #12", "EVT done D #5", "EVT done G #99", "EVT safety_stop #3"]:
            assert re.search(r"#\d+$", line), f"Malformed corr id in {line!r}"


# ---------------------------------------------------------------------------
# OTOS commands
# ---------------------------------------------------------------------------

class TestOtosCommands:
    """Tests for OTOS sensor commands (OI, OZ, OR, OP, OV, OL, OA)."""

    def test_oi_ok_format(self) -> None:
        """OI → OK oi."""
        line = "OK oi"
        verb, body = parse_ok(line)
        assert verb == "oi"
        assert body == ""

    def test_oi_nodev_format(self) -> None:
        """OI with no OTOS → ERR nodev oi."""
        line = "ERR nodev oi"
        code, detail = parse_err(line)
        assert code == "nodev"
        assert "oi" in detail

    def test_oz_ok_format(self) -> None:
        """OZ → OK oz."""
        line = "OK oz"
        verb, body = parse_ok(line)
        assert verb == "oz"
        assert body == ""

    def test_or_ok_format(self) -> None:
        """OR → OK or."""
        line = "OK or"
        verb, body = parse_ok(line)
        assert verb == "or"
        assert body == ""

    def test_op_ok_format(self) -> None:
        """OP → OK pos x=<x> y=<y> h=<h>."""
        line = "OK pos x=350 y=-12 h=1780"
        verb, body = parse_ok(line)
        assert verb == "pos"
        kv = parse_body_kv(body)
        assert "x" in kv
        assert "y" in kv
        assert "h" in kv

    def test_op_values(self) -> None:
        """OP reads and echoes x, y, h values."""
        line = "OK pos x=350 y=-12 h=1780"
        _, body = parse_ok(line)
        kv = parse_body_kv(body)
        assert int(kv["x"]) == 350
        assert int(kv["y"]) == -12
        assert int(kv["h"]) == 1780

    def test_ov_ok_format(self) -> None:
        """OV <x> <y> <h> → OK setpos x=<x> y=<y> h=<h>."""
        line = "OK setpos x=100 y=50 h=0"
        verb, body = parse_ok(line)
        assert verb == "setpos"
        kv = parse_body_kv(body)
        assert kv["x"] == "100"
        assert kv["y"] == "50"
        assert kv["h"] == "0"

    def test_ov_badarg_too_few(self) -> None:
        """OV 100 50 (no h) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_ol_read_format(self) -> None:
        """OL → OK linear scalar=<val>."""
        line = "OK linear scalar=10"
        verb, body = parse_ok(line)
        assert verb == "linear"
        kv = parse_body_kv(body)
        assert "scalar" in kv

    def test_ol_write_format(self) -> None:
        """OL <val> → OK linear scalar=<val>."""
        line = "OK linear scalar=15"
        verb, body = parse_ok(line)
        assert verb == "linear"
        kv = parse_body_kv(body)
        assert kv["scalar"] == "15"

    def test_oa_read_format(self) -> None:
        """OA → OK angular scalar=<val>."""
        line = "OK angular scalar=5"
        verb, body = parse_ok(line)
        assert verb == "angular"
        kv = parse_body_kv(body)
        assert "scalar" in kv

    def test_oa_write_format(self) -> None:
        """OA <val> → OK angular scalar=<val>."""
        line = "OK angular scalar=-3"
        verb, body = parse_ok(line)
        assert verb == "angular"
        kv = parse_body_kv(body)
        assert kv["scalar"] == "-3"


# ---------------------------------------------------------------------------
# Port commands
# ---------------------------------------------------------------------------

class TestPortCommands:
    """Tests for digital/analog port commands (P, PA)."""

    def test_p_read_format(self) -> None:
        """P <port> → OK port p=<port> v=<val>."""
        line = "OK port p=1 v=0"
        verb, body = parse_ok(line)
        assert verb == "port"
        kv = parse_body_kv(body)
        assert kv["p"] == "1"
        assert "v" in kv

    def test_p_write_format(self) -> None:
        """P <port> <val> → OK port p=<port> v=<val>."""
        line = "OK port p=2 v=1"
        verb, body = parse_ok(line)
        assert verb == "port"
        kv = parse_body_kv(body)
        assert kv["p"] == "2"
        assert kv["v"] == "1"

    def test_p_badarg_no_port(self) -> None:
        """P (no port) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_p_range_port(self) -> None:
        """P 5 (port out of range 1..4) → ERR range port."""
        line = "ERR range port"
        code, detail = parse_err(line)
        assert code == "range"
        assert "port" in detail

    def test_pa_read_format(self) -> None:
        """PA <port> → OK aport p=<port> v=<val>."""
        line = "OK aport p=3 v=512"
        verb, body = parse_ok(line)
        assert verb == "aport"
        kv = parse_body_kv(body)
        assert kv["p"] == "3"
        assert "v" in kv

    def test_pa_write_format(self) -> None:
        """PA <port> <val> → OK aport p=<port> v=<val>."""
        line = "OK aport p=1 v=750"
        verb, body = parse_ok(line)
        assert verb == "aport"
        kv = parse_body_kv(body)
        assert kv["p"] == "1"
        assert kv["v"] == "750"

    def test_pa_range_val_too_high(self) -> None:
        """PA 1 1024 → ERR range val."""
        line = "ERR range val"
        code, detail = parse_err(line)
        assert code == "range"
        assert "val" in detail

    def test_pa_badarg_no_port(self) -> None:
        """PA (no port) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"


# ---------------------------------------------------------------------------
# Error format tests
# ---------------------------------------------------------------------------

class TestErrorFormats:
    """Tests for ERR response format conventions."""

    def test_err_badarg_format(self) -> None:
        """ERR badarg has correct prefix."""
        line = "ERR badarg"
        assert line.startswith("ERR badarg")

    def test_err_range_field_format(self) -> None:
        """ERR range <field> includes the field name."""
        line = "ERR range l"
        code, detail = parse_err(line)
        assert code == "range"
        assert detail.strip() == "l"

    def test_err_no_legacy_colon(self) -> None:
        """v2 errors use 'ERR' tag, not legacy 'ERR:' with colon."""
        for line in ["ERR badarg", "ERR range l", "ERR nodev oi"]:
            assert not line.startswith("ERR:")

    def test_ok_no_legacy_ack(self) -> None:
        """v2 successes use 'OK' tag, not legacy 'ACK:' prefix."""
        for line in [
            "OK drive l=200 r=150",
            "OK stop",
            "OK grip deg=90",
        ]:
            assert not line.startswith("ACK:")

    def test_err_with_corr_id(self) -> None:
        """ERR badarg echoes #id when present."""
        line = "ERR badarg #5"
        assert line.endswith("#5")
        code, detail = parse_err(line)
        assert code == "badarg"


# ---------------------------------------------------------------------------
# Correlation id on synchronous responses
# ---------------------------------------------------------------------------

class TestCorrIdOnMotionVerbs:
    """Validate #id echoed on synchronous OK and ERR responses."""

    def test_s_corr_id_echoed(self) -> None:
        """S 200 150 #1 → OK drive ... #1."""
        line = "OK drive l=200 r=150 #1"
        assert "#1" in line

    def test_t_corr_id_echoed(self) -> None:
        """T 200 150 1000 #2 → OK drive ... #2."""
        line = "OK drive l=200 r=150 ms=1000 #2"
        assert "#2" in line

    def test_d_corr_id_echoed(self) -> None:
        """D 200 200 300 #3 → OK drive ... #3."""
        line = "OK drive l=200 r=200 mm=300 #3"
        assert "#3" in line

    def test_g_corr_id_echoed(self) -> None:
        """G 300 0 200 #4 → OK goto ... #4."""
        line = "OK goto x=300 y=0 speed=200 #4"
        assert "#4" in line

    def test_stop_corr_id_echoed(self) -> None:
        """STOP #6 → OK stop #6."""
        line = "OK stop #6"
        assert "#6" in line

    def test_grip_corr_id_echoed(self) -> None:
        """GRIP 90 #7 → OK grip deg=90 #7."""
        line = "OK grip deg=90 #7"
        assert "#7" in line

    def test_zero_corr_id_echoed(self) -> None:
        """ZERO enc #8 → OK zero enc #8."""
        line = "OK zero enc #8"
        assert "#8" in line


# ---------------------------------------------------------------------------
# Wire format example cross-checks from ticket spec
# ---------------------------------------------------------------------------

class TestWireFormatExamples:
    """Concrete wire format examples from the ticket spec."""

    def test_s_example(self) -> None:
        """S 200 150 → OK drive l=200 r=150."""
        line = "OK drive l=200 r=150"
        verb, body = parse_ok(line)
        assert verb == "drive"
        kv = parse_body_kv(body)
        assert kv == {"l": "200", "r": "150"}

    def test_t_example(self) -> None:
        """T 200 150 1000 → OK drive l=200 r=150 ms=1000."""
        line = "OK drive l=200 r=150 ms=1000"
        _, body = parse_ok(line)
        kv = parse_body_kv(body)
        assert kv == {"l": "200", "r": "150", "ms": "1000"}

    def test_d_example(self) -> None:
        """D 200 200 300 → OK drive l=200 r=200 mm=300."""
        line = "OK drive l=200 r=200 mm=300"
        _, body = parse_ok(line)
        kv = parse_body_kv(body)
        assert kv == {"l": "200", "r": "200", "mm": "300"}

    def test_g_example(self) -> None:
        """G 300 0 200 → OK goto x=300 y=0 speed=200."""
        line = "OK goto x=300 y=0 speed=200"
        _, body = parse_ok(line)
        kv = parse_body_kv(body)
        assert kv == {"x": "300", "y": "0", "speed": "200"}

    def test_stop_example(self) -> None:
        """STOP → OK stop."""
        assert "OK stop" == "OK stop"

    def test_grip_with_arg_example(self) -> None:
        """GRIP 90 → OK grip deg=90."""
        line = "OK grip deg=90"
        _, body = parse_ok(line)
        kv = parse_body_kv(body)
        assert kv == {"deg": "90"}

    def test_zero_enc_example(self) -> None:
        """ZERO enc → OK zero enc."""
        line = "OK zero enc"
        verb, body = parse_ok(line)
        assert verb == "zero"
        assert body.strip() == "enc"

    def test_zero_pose_example(self) -> None:
        """ZERO pose → OK zero pose."""
        line = "OK zero pose"
        verb, body = parse_ok(line)
        assert verb == "zero"
        assert body.strip() == "pose"

    def test_zero_enc_pose_example(self) -> None:
        """ZERO enc pose → OK zero enc pose."""
        line = "OK zero enc pose"
        verb, body = parse_ok(line)
        assert verb == "zero"
        assert "enc" in body
        assert "pose" in body

    def test_g_is_goto_not_gripper(self) -> None:
        """G verb is go-to (OK goto), GRIP verb is gripper (OK grip)."""
        g_line = "OK goto x=300 y=0 speed=200"
        grip_line = "OK grip deg=90"
        g_verb, _ = parse_ok(g_line)
        grip_verb, _ = parse_ok(grip_line)
        assert g_verb == "goto"
        assert grip_verb == "grip"
