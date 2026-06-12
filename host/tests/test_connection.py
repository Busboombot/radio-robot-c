"""Tests for robot_radio.robot.connection — shared robot construction module.

Verifies that:
  - session cache read/write round-trips correctly
  - make_robot uses the cache on a cache hit
  - make_robot does a full HELLO on a cache miss
  - make_robot constructs a Nezha robot when the config says 'nezha'
  - get_port returns cached port when available
  - CLI and MCP use the same cache path (session cache parity)
  - _parse_device_line correctly parses DEVICE: announcements
"""

from __future__ import annotations

import json
import os
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock, patch, call

import pytest

from robot_radio.robot.connection import (
    read_session_cache,
    write_session_cache,
    get_port,
    make_robot,
    _parse_device_line,
    _SESSION_CACHE_PATH,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_ANNOUNCE_RELAY = {
    "role": "RELAY",
    "common_name": "Radio Relay",
    "device_name": "relay-01",
    "serial_field": "00:01:02:03",
}

_CONNECT_RESULT = {
    "port": "/dev/ttyUSB0",
    "mode": "relay",
    "announcement": _ANNOUNCE_RELAY,
    "lines": [],
    "responses": [],
}


def _make_mock_conn(announcement=_ANNOUNCE_RELAY, error=None):
    """Return a mock SerialConnection that succeeds on connect()."""
    conn = MagicMock()
    conn.is_open = True
    conn.mode = "relay"
    conn._mode = "relay"
    if error:
        conn.connect.return_value = {"error": error}
    else:
        conn.connect.return_value = {
            "port": "/dev/ttyUSB0",
            "mode": "relay",
            "announcement": announcement,
            "lines": [],
            "responses": [],
        }
    conn.read_lines.return_value = ["DEVICE:RELAY:Radio Relay:relay-01:00:01:02:03"]
    return conn


def _nezha_config():
    cfg = MagicMock(spec_set=[
        "hardware_model", "otos_linear_scale", "otos_angular_scale",
        "calibration", "geometry", "wheels", "mm_per_tick", "vision",
    ])
    cfg.hardware_model = "nezha"
    cfg.otos_linear_scale = 1.0
    cfg.otos_angular_scale = 1.0
    cfg.calibration = None
    cfg.geometry = None
    cfg.wheels = None
    cfg.mm_per_tick = None
    cfg.vision = MagicMock()
    cfg.vision.robot_tag_id = 100
    return cfg


def _simple_args(port=None):
    ns = types.SimpleNamespace()
    ns.port = port
    return ns


def _connection_patches(conn, config=None):
    cfg = config if config is not None else _nezha_config()
    return [
        patch("robot_radio.robot.connection.SerialConnection", return_value=conn),
        patch("robot_radio.robot.connection.list_serial_ports",
              return_value=["/dev/ttyUSB0"]),
        patch("robot_radio.robot.connection.get_robot_config", return_value=cfg),
    ]


# ---------------------------------------------------------------------------
# Session cache tests
# ---------------------------------------------------------------------------

class TestSessionCache:
    """read_session_cache / write_session_cache round-trip and error handling."""

    def test_write_then_read(self, tmp_path, monkeypatch):
        cache_file = tmp_path / ".rogo_session.json"
        monkeypatch.setattr(
            "robot_radio.robot.connection._SESSION_CACHE_PATH", str(cache_file))
        write_session_cache("/dev/ttyUSB0", "relay", "relay-01")
        result = read_session_cache()
        assert result is not None
        assert result["port"] == "/dev/ttyUSB0"
        assert result["mode"] == "relay"
        assert result["device_name"] == "relay-01"

    def test_read_missing_file_returns_none(self, tmp_path, monkeypatch):
        cache_file = tmp_path / ".rogo_session_missing.json"
        monkeypatch.setattr(
            "robot_radio.robot.connection._SESSION_CACHE_PATH", str(cache_file))
        assert read_session_cache() is None

    def test_read_malformed_json_returns_none(self, tmp_path, monkeypatch):
        cache_file = tmp_path / ".rogo_session.json"
        cache_file.write_text("not json!")
        monkeypatch.setattr(
            "robot_radio.robot.connection._SESSION_CACHE_PATH", str(cache_file))
        assert read_session_cache() is None

    def test_read_missing_port_key_returns_none(self, tmp_path, monkeypatch):
        cache_file = tmp_path / ".rogo_session.json"
        cache_file.write_text('{"mode": "relay"}')
        monkeypatch.setattr(
            "robot_radio.robot.connection._SESSION_CACHE_PATH", str(cache_file))
        assert read_session_cache() is None

    def test_write_swallows_errors(self, monkeypatch):
        """write_session_cache must not raise even if the path is unwritable."""
        monkeypatch.setattr(
            "robot_radio.robot.connection._SESSION_CACHE_PATH",
            "/dev/null/impossible/path.json")
        # Must not raise.
        write_session_cache("/dev/ttyUSB0", "relay", "relay-01")


# ---------------------------------------------------------------------------
# Parse device line tests
# ---------------------------------------------------------------------------

class TestParseDeviceLine:
    def test_parses_relay_line(self):
        lines = ["DEVICE:RELAY:Radio Relay:relay-01:00:01:02:03"]
        result = _parse_device_line(lines)
        assert result is not None
        assert result["role"] == "RELAY"
        assert result["common_name"] == "Radio Relay"
        assert result["device_name"] == "relay-01"

    def test_parses_with_garbage_prefix(self):
        lines = ["<some garbage>DEVICE:RELAY:Radio Relay:relay-01:00:01"]
        result = _parse_device_line(lines)
        assert result is not None
        assert result["role"] == "RELAY"

    def test_returns_none_for_no_device_line(self):
        lines = ["OK pong", "TLM t=123"]
        assert _parse_device_line(lines) is None

    def test_returns_none_for_empty_list(self):
        assert _parse_device_line([]) is None


# ---------------------------------------------------------------------------
# get_port tests
# ---------------------------------------------------------------------------

class TestGetPort:
    def test_explicit_port_wins(self):
        args = _simple_args(port="/dev/ttyEXPLICIT")
        with patch("robot_radio.robot.connection.list_serial_ports",
                   return_value=["/dev/ttyUSB0"]):
            result = get_port(args)
        assert result == "/dev/ttyEXPLICIT"

    def test_auto_detect_first_port(self):
        args = _simple_args(port=None)
        with patch("robot_radio.robot.connection.list_serial_ports",
                   return_value=["/dev/ttyUSB0", "/dev/ttyUSB1"]), \
             patch("robot_radio.robot.connection.read_session_cache",
                   return_value=None):
            result = get_port(args)
        assert result == "/dev/ttyUSB0"

    def test_uses_cache_when_present(self, tmp_path, monkeypatch):
        cache_file = tmp_path / ".rogo_session.json"
        monkeypatch.setattr(
            "robot_radio.robot.connection._SESSION_CACHE_PATH", str(cache_file))
        write_session_cache("/dev/ttyUSB1", "relay", "relay-01")
        args = _simple_args(port=None)
        with patch("robot_radio.robot.connection.list_serial_ports",
                   return_value=["/dev/ttyUSB0", "/dev/ttyUSB1"]):
            result = get_port(args)
        assert result == "/dev/ttyUSB1"

    def test_no_ports_exits(self):
        args = _simple_args(port=None)
        with patch("robot_radio.robot.connection.list_serial_ports", return_value=[]):
            with pytest.raises(SystemExit):
                get_port(args)


# ---------------------------------------------------------------------------
# make_robot tests
# ---------------------------------------------------------------------------

class TestMakeRobot:
    def test_returns_nezha_on_hello(self):
        """make_robot returns a Nezha when config says 'nezha'."""
        from robot_radio.robot import Nezha
        conn = _make_mock_conn()
        args = _simple_args(port=None)

        patches = _connection_patches(conn)
        for p in patches:
            p.__enter__()
        try:
            with patch("robot_radio.robot.connection.read_session_cache",
                       return_value=None):
                robot, returned_conn, result = make_robot(
                    port=None, mode=None, verbose=False, args=args)
        finally:
            for p in patches:
                p.__exit__(None, None, None)

        assert isinstance(robot, Nezha)
        assert returned_conn is conn

    def test_cache_hit_skips_hello(self, tmp_path, monkeypatch):
        """When cache matches the port, make_robot uses skip_ping=True."""
        cache_file = tmp_path / ".rogo_session.json"
        monkeypatch.setattr(
            "robot_radio.robot.connection._SESSION_CACHE_PATH", str(cache_file))
        write_session_cache("/dev/ttyUSB0", "relay", "relay-01")

        conn = _make_mock_conn()
        args = _simple_args(port=None)

        patches = _connection_patches(conn)
        for p in patches:
            p.__enter__()
        try:
            robot, returned_conn, result = make_robot(
                port=None, mode=None, verbose=False, args=args)
        finally:
            for p in patches:
                p.__exit__(None, None, None)

        # With cache hit, connect() is called with skip_ping=True.
        call_kwargs = conn.connect.call_args
        assert call_kwargs is not None
        # skip_ping is passed as keyword argument
        kw = call_kwargs[1] if call_kwargs[1] else {}
        pos = call_kwargs[0] if call_kwargs[0] else ()
        assert kw.get("skip_ping") is True or (len(pos) > 0 and pos[0] is True), \
            f"Expected skip_ping=True in connect() call; got args={pos} kwargs={kw}"

    def test_writes_session_cache_on_hello(self, tmp_path, monkeypatch):
        """After a full HELLO, the session cache is written."""
        cache_file = tmp_path / ".rogo_session.json"
        monkeypatch.setattr(
            "robot_radio.robot.connection._SESSION_CACHE_PATH", str(cache_file))

        conn = _make_mock_conn()
        args = _simple_args(port=None)

        patches = _connection_patches(conn)
        for p in patches:
            p.__enter__()
        try:
            # No cache initially.
            with patch("robot_radio.robot.connection.read_session_cache",
                       return_value=None):
                make_robot(port=None, mode=None, verbose=False, args=args)
        finally:
            for p in patches:
                p.__exit__(None, None, None)

        written = json.loads(cache_file.read_text())
        assert written["port"] == "/dev/ttyUSB0"
        assert written["mode"] == "relay"

    def test_explicit_port_bypasses_cache(self, tmp_path, monkeypatch):
        """When an explicit port is passed, cache is not consulted."""
        cache_file = tmp_path / ".rogo_session.json"
        monkeypatch.setattr(
            "robot_radio.robot.connection._SESSION_CACHE_PATH", str(cache_file))
        # Write a cache entry for /dev/ttyUSB0
        write_session_cache("/dev/ttyUSB0", "relay", "relay-01")

        conn = _make_mock_conn()
        # Explicit port "/dev/ttyUSB0" → should bypass cache and do full HELLO.
        args = _simple_args(port=None)

        patches = _connection_patches(conn)
        for p in patches:
            p.__enter__()
        try:
            make_robot(port="/dev/ttyUSB0", mode=None, verbose=False, args=args)
        finally:
            for p in patches:
                p.__exit__(None, None, None)

        # connect() must not be called with skip_ping=True.
        call_kwargs = conn.connect.call_args
        kw = call_kwargs[1] if call_kwargs[1] else {}
        assert kw.get("skip_ping") is not True, \
            "Expected full HELLO (no skip_ping) when explicit port given"


# ---------------------------------------------------------------------------
# Session cache parity test (CLI vs MCP use the same file)
# ---------------------------------------------------------------------------

class TestSessionCacheParity:
    """CLI and MCP must use the same session cache file."""

    def test_cli_and_mcp_use_same_cache_path(self):
        """Both front-ends delegate to connection._SESSION_CACHE_PATH."""
        import robot_radio.robot.connection as conn_mod
        import robot_radio.io.cli as cli_mod

        # CLI imports read_session_cache / write_session_cache from connection.
        # Verify they are the same function objects (not copies).
        assert cli_mod._read_session_cache is conn_mod.read_session_cache, \
            ("cli._read_session_cache is not connection.read_session_cache — "
             "session cache parity broken")
        assert cli_mod._write_session_cache is conn_mod.write_session_cache, \
            ("cli._write_session_cache is not connection.write_session_cache — "
             "session cache parity broken")

    def test_mcp_imports_make_robot_from_connection(self):
        """robot_mcp.py imports make_robot from robot_radio.robot.connection.

        Verified by source inspection — avoids importing the mcp package.
        """
        import inspect
        from pathlib import Path

        mcp_source_path = Path(__file__).parent.parent / "robot_radio" / "io" / "robot_mcp.py"
        source = mcp_source_path.read_text()
        assert "from robot_radio.robot.connection import make_robot" in source, \
            ("robot_mcp.py does not import make_robot from robot_radio.robot.connection — "
             "MCP does not share the robot construction path")
