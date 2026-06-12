"""Tests for host/robot_radio/io/cli.py — v2 CLI conversion.

All tests mock the serial layer; no hardware required.  Mock read loops
ALWAYS return finite side_effect lists (never empty []) to avoid OOM.

OOM safety rule: every read_lines / send mock must return a non-empty,
terminating sequence so that internal read loops cannot spin forever.
"""

from __future__ import annotations

import math
import sys
from typing import Any
from unittest.mock import MagicMock, patch, call

import pytest

# ---------------------------------------------------------------------------
# Helpers — build a realistic mock connection + robot stack
# ---------------------------------------------------------------------------

_TLM_ENC   = "TLM t=1000 enc=512,490"
_TLM_POSE  = "TLM t=1001 pose=350,-12,1780"
_TLM_LINE  = "TLM t=1002 line=200,210,230,240"
_TLM_COLOR = "TLM t=1003 color=120,80,60,255"
_TLM_ALL   = "TLM t=2000 enc=512,490 pose=350,-12,1780 line=200,210,230,240 color=120,80,60,255"
_OL_OK     = "OK linear scalar=27"
_ID_RESP   = "ID model=Nezha2 name=GUTOV serial=01:02:03 fw=0.9.0 proto=2"
_PING_OK   = "OK pong t=12345"


def _make_mock_conn(
    send_side: list[dict] | None = None,
    read_lines_side: list[list[str]] | None = None,
) -> MagicMock:
    """Return a mock SerialConnection for unit tests.

    ``send_side`` is a list of return dicts for successive conn.send() calls.
    ``read_lines_side`` is a list of return lists for successive read_lines() calls.

    Defaults:
    - send() returns {"responses": [_TLM_ALL, _OL_OK, _ID_RESP, _PING_OK], ...}
    - read_lines() returns [_TLM_ALL] (finite, non-empty — avoids OOM spin)
    - read_pending_lines() returns [] (drain only, does not block)
    """
    conn = MagicMock()
    conn.is_open = True
    conn.mode = "relay"
    conn._mode = "relay"
    conn.in_waiting = 0

    # connect() must return a dict with an announcement so _make_robot
    # doesn't fall into the HELLO retry loop or error out.
    conn.connect.return_value = {
        "port": "/dev/ttyUSB0",
        "mode": "relay",
        "announcement": {
            "role": "RELAY",
            "common_name": "Radio Relay",
            "device_name": "relay-01",
            "serial_field": "00:01:02:03",
        },
        "lines": [],
        "responses": [],
    }

    # Default rich send() response — provides OL scalar for freshness check,
    # ID for push_calibration, PING for liveness.
    _default_send = {
        "sent": "CMD",
        "mode": "relay",
        "responses": [_PING_OK, _ID_RESP, _OL_OK, _TLM_ALL],
    }

    if send_side is not None:
        # Cycle through the provided dicts, repeating last one when exhausted.
        _side = [dict(s) for s in send_side]
        _last = _side[-1] if _side else _default_send
        # side_effect must always return something — avoid IndexError OOM spiral.
        _queue = list(_side)

        def _send_fn(cmd, read_ms=500):  # noqa: ARG001
            return _queue.pop(0) if _queue else _last
        conn.send.side_effect = _send_fn
    else:
        conn.send.return_value = _default_send

    # CRITICAL: read_lines must return a non-empty terminating list.
    # An empty [] return causes infinite loops and OOM.
    if read_lines_side is not None:
        _rl_queue = list(read_lines_side)
        _rl_last = _rl_queue[-1] if _rl_queue else [_TLM_ALL]

        def _rl_fn(duration_ms=100):  # noqa: ARG001
            return _rl_queue.pop(0) if _rl_queue else _rl_last
        conn.read_lines.side_effect = _rl_fn
    else:
        # Default: one meaningful line then stop (finite).
        conn.read_lines.return_value = [_TLM_ALL]

    conn.send_fast.return_value = None
    conn.read_pending_lines.return_value = []
    return conn


def _make_mock_serial_connection(conn: MagicMock) -> MagicMock:
    """Return a SerialConnection *class* mock whose constructor returns conn."""
    cls = MagicMock()
    cls.return_value = conn
    return cls


def _announce_result() -> dict:
    return {
        "port": "/dev/ttyUSB0",
        "mode": "relay",
        "announcement": {
            "role": "RELAY",
            "common_name": "Radio Relay",
            "device_name": "relay-01",
            "serial_field": "00:01:02:03",
        },
        "lines": [],
        "responses": [_PING_OK, _ID_RESP, _OL_OK, _TLM_ALL],
    }


# ---------------------------------------------------------------------------
# Patch context: isolates _make_robot from real hardware
# ---------------------------------------------------------------------------

def _nezha_config() -> MagicMock:
    """Return a minimal mock robot config that selects the Nezha hardware model."""
    cfg = MagicMock(spec_set=[
        "hardware_model", "otos_linear_scale", "otos_angular_scale",
        "calibration", "geometry", "wheels", "mm_per_tick", "vision",
    ])
    cfg.hardware_model = "nezha"
    cfg.otos_linear_scale = 1.027  # yields expected_ol=27, matches _OL_OK
    cfg.otos_angular_scale = 1.0
    cfg.calibration = None
    cfg.geometry = None
    cfg.wheels = None
    cfg.mm_per_tick = None   # prevents _print_enc_dist MagicMock format error
    cfg.vision = MagicMock()
    cfg.vision.robot_tag_id = 100
    return cfg


def _patches(conn: MagicMock, config=None):
    """Return a list of context manager patches to isolate the CLI.

    Patches:
    - SerialConnection class in connection module → returns mock conn.
      (make_robot imports SerialConnection from robot_radio.robot.connection,
       so we patch there rather than in the cli module.)
    - list_serial_ports in the connection module → ["/dev/ttyUSB0"]
    - get_robot_config in the connection module → Nezha config mock
    - get_robot_config in the cli module → Nezha config mock (for _push_calibration)
    - match_robot_by_id in cli → None (ID-based lookup unused in unit tests)
    """
    cfg = config if config is not None else _nezha_config()
    return [
        patch("robot_radio.robot.connection.SerialConnection", return_value=conn),
        patch("robot_radio.robot.connection.list_serial_ports", return_value=["/dev/ttyUSB0"]),
        patch("robot_radio.robot.connection.get_robot_config", return_value=cfg),
        patch("robot_radio.io.cli.get_robot_config", return_value=cfg),
        patch("robot_radio.io.cli.match_robot_by_id", return_value=None),
    ]


def _enter_patches(patches):
    return [p.__enter__() for p in patches]


def _exit_patches(patches):
    for p in patches:
        p.__exit__(None, None, None)


# ---------------------------------------------------------------------------
# Helper: run a CLI command with a mock connection, capture output.
# ---------------------------------------------------------------------------

def _run_cmd(cmd_fn, args_ns, conn: MagicMock):
    """Call a CLI command function with a mocked robot environment.

    Returns (stdout_text, stderr_text) as strings.
    """
    import io
    from contextlib import redirect_stdout, redirect_stderr

    stdout_buf = io.StringIO()
    stderr_buf = io.StringIO()

    # Patch all serial-layer dependencies.
    patches = _patches(conn)
    for p in patches:
        p.__enter__()

    try:
        with redirect_stdout(stdout_buf), redirect_stderr(stderr_buf):
            try:
                cmd_fn(args_ns)
            except SystemExit:
                pass  # Normal exit for some commands (unsupported, errors)
    finally:
        _exit_patches(patches)

    return stdout_buf.getvalue(), stderr_buf.getvalue()


# ---------------------------------------------------------------------------
# Namespace helpers
# ---------------------------------------------------------------------------

class _NS:
    """Simple argparse namespace replacement for tests."""
    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, name):
        return None  # Graceful default for unset fields


# ===========================================================================
# Tests: cmd_enc — v2 SNAP → TLM encoder read
# ===========================================================================

class TestCmdEnc:
    """rogo enc must read encoder values via SNAP → TLM, NOT v1 ENC verb."""

    def test_enc_reads_from_tlm(self, capsys):
        """SNAP response containing TLM enc= field is parsed and printed."""
        conn = _make_mock_conn(
            send_side=[
                # connect() / freshness check responses
                {"responses": [_PING_OK, _ID_RESP, _OL_OK]},
                # SNAP response
                {"responses": [_TLM_ENC]},
            ]
        )

        from robot_radio.io.cli import cmd_enc
        args = _NS(port=None, verbose=False)

        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            cmd_enc(args)
        finally:
            _exit_patches(patches)

        out = capsys.readouterr().out
        assert "ENC 512 490" in out

    def test_enc_snap_sent(self):
        """cmd_enc must send SNAP to the firmware connection."""
        conn = _make_mock_conn()

        from robot_radio.io.cli import cmd_enc
        args = _NS(port=None, verbose=False)
        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            cmd_enc(args)
        finally:
            _exit_patches(patches)

        # Verify that "SNAP" was sent at some point.
        sent_cmds = [str(c.args[0]) for c in conn.send.call_args_list]
        assert any("SNAP" in cmd for cmd in sent_cmds), f"SNAP not sent; calls: {sent_cmds}"

    def test_enc_tlm_parse(self):
        """parse_tlm correctly extracts enc= from a TLM line."""
        from robot_radio.robot.protocol import parse_tlm
        frame = parse_tlm(_TLM_ENC)
        assert frame is not None
        assert frame.enc == (512, 490)


# ===========================================================================
# Tests: cmd_opos — v2 SNAP → TLM robot OTOS pose read
# ===========================================================================

class TestCmdOpos:
    """rogo opos must read robot OTOS fused pose via SNAP → TLM pose= field."""

    def test_opos_reads_from_tlm(self, capsys):
        """SNAP response containing TLM pose= field is parsed and printed."""
        conn = _make_mock_conn(
            send_side=[
                {"responses": [_PING_OK, _ID_RESP, _OL_OK]},
                # SNAP response
                {"responses": [_TLM_POSE]},
            ]
        )

        from robot_radio.io.cli import cmd_opos
        args = _NS(port=None, verbose=False)
        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            cmd_opos(args)
        finally:
            _exit_patches(patches)

        out = capsys.readouterr().out
        # x_mm=350 y_mm=-12 h_cdeg=1780 → h_deg=17.8
        assert "POSE" in out
        assert "350" in out
        assert "-12" in out
        assert "17.8" in out

    def test_opos_tlm_parse_pose(self):
        """parse_tlm correctly extracts pose= field from a TLM line."""
        from robot_radio.robot.protocol import parse_tlm
        frame = parse_tlm(_TLM_POSE)
        assert frame is not None
        assert frame.pose == (350, -12, 1780)

    def test_opos_heading_conversion(self):
        """Heading centi-degrees → degrees conversion is correct."""
        from robot_radio.robot.protocol import parse_tlm
        frame = parse_tlm("TLM t=1 pose=0,0,9000")
        assert frame is not None
        assert frame.pose is not None
        x_mm, y_mm, h_cdeg = frame.pose
        h_deg = h_cdeg / 100.0
        assert abs(h_deg - 90.0) < 0.01


# ===========================================================================
# Tests: cmd_stop — v2 STOP verb
# ===========================================================================

class TestCmdStop:
    """rogo stop must send STOP (v2), not the v1 X verb."""

    def test_stop_prints_STOP(self, capsys):
        """cmd_stop outputs 'STOP' confirming v2 verb."""
        conn = _make_mock_conn()
        from robot_radio.io.cli import cmd_stop
        args = _NS(port=None, verbose=False)
        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            cmd_stop(args)
        finally:
            _exit_patches(patches)

        out = capsys.readouterr().out
        assert "STOP" in out
        assert "X" not in out.strip()

    def test_stop_sends_STOP_verb(self):
        """robot.stop() must invoke the STOP verb on the wire (send_fast('STOP'))."""
        conn = _make_mock_conn()
        from robot_radio.io.cli import cmd_stop
        args = _NS(port=None, verbose=False)
        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            cmd_stop(args)
        finally:
            _exit_patches(patches)

        # stop() uses send_fast; verify the STOP command was fired.
        fast_calls = [str(c.args[0]) for c in conn.send_fast.call_args_list]
        assert any("STOP" in cmd for cmd in fast_calls), \
            f"STOP not in send_fast calls: {fast_calls}"


# ===========================================================================
# Tests: set_config(sTimeout=ms) — v2 watchdog replacement
# ===========================================================================

class TestSetConfig:
    """set_config(sTimeout=ms) replaces the v1 set_watchdog verb."""

    def test_set_config_encodes_correctly(self):
        """NezhaProtocol.set_config(sTimeout=500) sends SET sTimeout=500."""
        from robot_radio.robot.protocol import NezhaProtocol

        conn = MagicMock()
        conn.send.return_value = {"responses": ["OK set sTimeout=500"]}
        proto = NezhaProtocol(conn)
        proto.set_config(sTimeout=500)

        # Verify the exact v2 wire encoding.
        call_args = conn.send.call_args_list
        sent_cmds = [str(c.args[0]) for c in call_args]
        assert any("SET" in cmd and "sTimeout=500" in cmd for cmd in sent_cmds), \
            f"SET sTimeout=500 not found; calls: {sent_cmds}"

    def test_no_set_watchdog_in_proto(self):
        """NezhaProtocol must NOT have a set_watchdog method (v1 removed)."""
        from robot_radio.robot.protocol import NezhaProtocol
        assert not hasattr(NezhaProtocol, "set_watchdog"), \
            "set_watchdog must not exist in v2 NezhaProtocol"


# ===========================================================================
# Tests: cmd_line — v2 SNAP → TLM line sensor read
# ===========================================================================

class TestCmdLine:
    """rogo line must read line sensor via SNAP → TLM, NOT v1 LS verb."""

    def test_line_reads_from_tlm(self, capsys):
        """SNAP response with TLM line= field prints LS output."""
        conn = _make_mock_conn(
            send_side=[
                {"responses": [_PING_OK, _ID_RESP, _OL_OK]},
                {"responses": [_TLM_LINE]},
            ]
        )
        from robot_radio.io.cli import cmd_line
        args = _NS(port=None, verbose=False)
        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            cmd_line(args)
        finally:
            _exit_patches(patches)

        out = capsys.readouterr().out
        assert "LS 200 210 230 240" in out

    def test_line_no_v1_LS_verb_sent(self):
        """cmd_line must not send the v1 LS verb to the robot."""
        conn = _make_mock_conn()
        from robot_radio.io.cli import cmd_line
        args = _NS(port=None, verbose=False)
        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            cmd_line(args)
        finally:
            _exit_patches(patches)

        sent_cmds = [str(c.args[0]) for c in conn.send.call_args_list]
        assert not any(cmd.strip() == "LS" for cmd in sent_cmds), \
            f"v1 LS verb was sent; calls: {sent_cmds}"


# ===========================================================================
# Tests: cmd_color — v2 SNAP → TLM color sensor read
# ===========================================================================

class TestCmdColor:
    """rogo color must read color sensor via SNAP → TLM, NOT v1 CS verb."""

    def test_color_no_v1_CS_verb_sent(self):
        """cmd_color must not send the v1 CS verb to the robot."""
        conn = _make_mock_conn(
            send_side=[
                {"responses": [_PING_OK, _ID_RESP, _OL_OK]},
                {"responses": [_TLM_COLOR]},
            ]
        )
        from robot_radio.io.cli import cmd_color
        args = _NS(port=None, verbose=False, raw=True, name=False,
                   rgb=False, hsv=False, calibrate_white=False)
        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            cmd_color(args)
        finally:
            _exit_patches(patches)

        sent_cmds = [str(c.args[0]) for c in conn.send.call_args_list]
        assert not any(cmd.strip() == "CS" for cmd in sent_cmds), \
            f"v1 CS verb was sent; calls: {sent_cmds}"

    def test_color_tlm_parse(self):
        """parse_tlm correctly extracts color= field from a TLM line."""
        from robot_radio.robot.protocol import parse_tlm
        frame = parse_tlm(_TLM_COLOR)
        assert frame is not None
        assert frame.color == (120, 80, 60, 255)


# ===========================================================================
# Tests: drive stream --resend
# ===========================================================================

class TestDriveStreamResend:
    """rogo drive <L> <R> stream [--resend MS] forwards the resend interval correctly."""

    def _run_drive_stream(self, resend_ms, extra_kw=None):
        """Run cmd_drive in stream mode and return all stream_drive call kwargs."""
        from robot_radio.io.cli import cmd_drive

        # stream_drive must yield at least one item and then raise GeneratorExit.
        # We use a MagicMock that returns one TLM response then stops.
        tlm_resp = MagicMock()
        tlm_resp.tag = "TLM"
        tlm_resp.raw = _TLM_ENC

        conn = _make_mock_conn()

        stream_drive_calls = []

        class _StopAfterOne:
            """Generator that yields one item then exits cleanly."""
            def __init__(self, **kwargs):
                stream_drive_calls.append(kwargs)

            def __iter__(self):
                yield tlm_resp
                # Simulate Ctrl-C / natural end by raising GeneratorExit.
                return

            def close(self):
                pass

        kw = dict(port=None, verbose=False, left=200, right=200,
                  stream_kw="stream", ms=None, mm=None, ez=False,
                  min_speed=None, resend=resend_ms)
        if extra_kw:
            kw.update(extra_kw)
        args = _NS(**kw)

        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            with patch("robot_radio.io.cli.Nezha.stream_drive",
                       side_effect=lambda speeds, **kw: _StopAfterOne(**kw)):
                try:
                    cmd_drive(args)
                except (SystemExit, KeyboardInterrupt):
                    pass
        finally:
            _exit_patches(patches)

        return stream_drive_calls

    def test_resend_150_produces_correct_watchdog(self):
        """--resend 150 → watchdog_ms = round(150/0.30) = 500."""
        calls = self._run_drive_stream(150)
        assert len(calls) >= 1, "stream_drive was not called"
        wdg = calls[0].get("watchdog_ms")
        # keepalive = watchdog_ms * 0.30 → 150ms when watchdog_ms=500
        assert wdg is not None
        assert abs(wdg - 500) <= 2, f"expected watchdog_ms≈500, got {wdg}"

    def test_resend_300_produces_correct_watchdog(self):
        """--resend 300 → watchdog_ms = round(300/0.30) = 1000."""
        calls = self._run_drive_stream(300)
        assert len(calls) >= 1
        wdg = calls[0].get("watchdog_ms")
        assert abs(wdg - 1000) <= 2, f"expected watchdog_ms≈1000, got {wdg}"

    def test_resend_zero_rejected(self):
        """--resend 0 must be rejected with a non-zero exit."""
        from robot_radio.io.cli import cmd_drive
        args = _NS(port=None, verbose=False, left=100, right=100,
                   stream_kw="stream", ms=None, mm=None, ez=False,
                   min_speed=None, resend=0)
        conn = _make_mock_conn()
        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            with pytest.raises(SystemExit) as exc_info:
                cmd_drive(args)
        finally:
            _exit_patches(patches)
        assert exc_info.value.code != 0

    def test_resend_negative_rejected(self):
        """--resend -1 must be rejected."""
        from robot_radio.io.cli import cmd_drive
        args = _NS(port=None, verbose=False, left=100, right=100,
                   stream_kw="stream", ms=None, mm=None, ez=False,
                   min_speed=None, resend=-1)
        conn = _make_mock_conn()
        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            with pytest.raises(SystemExit) as exc_info:
                cmd_drive(args)
        finally:
            _exit_patches(patches)
        assert exc_info.value.code != 0

    def test_stream_and_ms_mutually_exclusive(self):
        """Combining 'stream' with --ms must be rejected."""
        from robot_radio.io.cli import cmd_drive
        args = _NS(port=None, verbose=False, left=100, right=100,
                   stream_kw="stream", ms=500, mm=None, ez=False,
                   min_speed=None, resend=150)
        conn = _make_mock_conn()
        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            with pytest.raises(SystemExit):
                cmd_drive(args)
        finally:
            _exit_patches(patches)


# ===========================================================================
# Tests: drive stream VEL output and --secs auto-stop
# ===========================================================================

_TLM_ENC_VEL = "TLM t=1500 enc=256,244 vel=198,202"


class TestDriveStreamVelAndSecs:
    """rogo drive stream prints VEL and --secs N causes auto-stop."""

    def _run_stream_capture_output(self, extra_kw=None, tlm_line=_TLM_ENC_VEL,
                                   n_frames=3):
        """Run cmd_drive stream mode and return stdout text."""
        import io
        from contextlib import redirect_stdout, redirect_stderr
        from robot_radio.io.cli import cmd_drive

        frame_count = [0]

        class _MultiFrame:
            """Generator that yields n_frames TLM responses then stops."""
            def __init__(self, **kwargs):
                pass

            def __iter__(self):
                while frame_count[0] < n_frames:
                    resp = MagicMock()
                    resp.tag = "TLM"
                    resp.raw = tlm_line
                    frame_count[0] += 1
                    yield resp

            def close(self):
                pass

        kw = dict(port=None, verbose=False, left=200, right=200,
                  stream_kw="stream", ms=None, mm=None, ez=False,
                  min_speed=None, resend=150, secs=None)
        if extra_kw:
            kw.update(extra_kw)
        args = _NS(**kw)

        conn = _make_mock_conn()
        stdout_buf = io.StringIO()
        stderr_buf = io.StringIO()

        patches = _patches(conn)
        for p in patches:
            p.__enter__()
        try:
            with patch("robot_radio.io.cli.Nezha.stream_drive",
                       side_effect=lambda speeds, **kw: _MultiFrame(**kw)):
                with redirect_stdout(stdout_buf), redirect_stderr(stderr_buf):
                    try:
                        cmd_drive(args)
                    except (SystemExit, KeyboardInterrupt):
                        pass
        finally:
            _exit_patches(patches)

        return stdout_buf.getvalue()

    def test_stream_prints_vel(self):
        """Stream output includes VEL after ENC when vel= is in TLM."""
        out = self._run_stream_capture_output()
        # Live stream lines contain both ENC and VEL (e.g. "ENC 256 244  VEL 198 202").
        # The final summary line printed by _print_enc_dist contains only "ENC".
        # Filter to lines that have both ENC and VEL.
        enc_vel_lines = [ln for ln in out.splitlines() if "ENC" in ln and "VEL" in ln]
        assert len(enc_vel_lines) > 0, (
            f"No 'ENC ... VEL ...' live stream lines in output:\n{out!r}"
        )

    def test_stream_vel_values(self):
        """VEL values from TLM vel= field are printed correctly."""
        out = self._run_stream_capture_output()
        enc_vel_lines = [ln for ln in out.splitlines() if "ENC" in ln and "VEL" in ln]
        assert enc_vel_lines, "No ENC+VEL lines"
        # _TLM_ENC_VEL has vel=198,202
        assert "198" in enc_vel_lines[0], f"Left velocity 198 missing: {enc_vel_lines[0]!r}"
        assert "202" in enc_vel_lines[0], f"Right velocity 202 missing: {enc_vel_lines[0]!r}"

    def test_stream_vel_zero_when_no_vel_field(self):
        """VEL is printed as 0 0 when TLM has no vel= field."""
        out = self._run_stream_capture_output(tlm_line=_TLM_ENC)
        enc_lines = [ln for ln in out.splitlines() if "ENC" in ln]
        assert enc_lines, "No ENC lines"
        assert "VEL 0 0" in enc_lines[0], f"Expected 'VEL 0 0': {enc_lines[0]!r}"

    def test_secs_stops_stream(self):
        """--secs N causes the stream loop to exit after N seconds."""
        import time as _time

        # Run with secs=0.0 (deadline already past) — loop should exit immediately.
        # We provide many frames but the deadline check should break after 0.
        out = self._run_stream_capture_output(extra_kw={"secs": 0.0}, n_frames=100)
        # With secs=0.0 the deadline fires immediately; 0 or very few ENC lines expected.
        enc_lines = [ln for ln in out.splitlines() if "ENC" in ln]
        # May print 0 lines (deadline before first frame) or 1 (race).
        assert len(enc_lines) <= 2, (
            f"Expected 0-2 ENC lines with secs=0, got {len(enc_lines)}: {out!r}"
        )


# ===========================================================================
# Tests: v1 verb grep — no v1 verbs in the CLI module's executed code paths
# ===========================================================================

class TestNoV1Verbs:
    """Verify that no v1 verb strings remain in cli.py executed code paths."""

    def _load_cli_source(self) -> str:
        """Return the source of cli.py as a string."""
        import inspect
        from robot_radio.io import cli
        return inspect.getsource(cli)

    def test_no_set_watchdog_calls(self):
        src = self._load_cli_source()
        assert "set_watchdog(" not in src, \
            "set_watchdog() is a v1 verb; use set_config(sTimeout=...) instead"

    def test_no_set_world_pose_calls(self):
        src = self._load_cli_source()
        # proto.set_world_pose() must not be called (v2 has no SI verb on proto)
        assert "proto.set_world_pose" not in src, \
            "proto.set_world_pose() is v1; use robot.set_world_pose() (OV command)"

    def test_no_bare_EZ_verb_string(self):
        """The EZ verb is valid in v2 (ZERO enc), but must not be the print output."""
        # cmd_stop should print 'STOP', not 'X'.
        src = self._load_cli_source()
        # 'print("X")' with exactly X is v1 stop output.
        assert 'print("X")' not in src, \
            "v1 stop output 'X' detected; cmd_stop should print 'STOP'"

    def test_no_LS_verb_sent_directly(self):
        """'robot.send(\"LS\"' must not appear (v1 line sensor verb)."""
        src = self._load_cli_source()
        assert 'send("LS"' not in src, \
            "v1 LS verb sent directly; use SNAP → TLM"

    def test_no_CS_verb_sent_directly(self):
        """'robot.send(\"CS\"' must not appear (v1 color sensor verb)."""
        src = self._load_cli_source()
        assert 'send("CS"' not in src, \
            "v1 CS verb sent directly; use SNAP → TLM"


# ===========================================================================
# Tests: TLM parse helpers (module-level, no mocking required)
# ===========================================================================

class TestTLMParseHelpers:
    """Unit tests for parse_tlm field extraction used by CLI commands."""

    def test_parse_tlm_all_fields(self):
        from robot_radio.robot.protocol import parse_tlm
        frame = parse_tlm(_TLM_ALL)
        assert frame is not None
        assert frame.enc == (512, 490)
        assert frame.pose == (350, -12, 1780)
        assert frame.line == (200, 210, 230, 240)
        assert frame.color == (120, 80, 60, 255)
        assert frame.t == 2000

    def test_parse_tlm_none_for_non_tlm(self):
        from robot_radio.robot.protocol import parse_tlm
        assert parse_tlm("OK pong t=1") is None
        assert parse_tlm("EVT done T") is None
        assert parse_tlm("") is None

    def test_parse_tlm_partial_fields(self):
        """A TLM frame with only enc= and no other fields is valid."""
        from robot_radio.robot.protocol import parse_tlm
        frame = parse_tlm("TLM t=100 enc=100,200")
        assert frame is not None
        assert frame.enc == (100, 200)
        assert frame.pose is None
        assert frame.line is None
        assert frame.color is None
