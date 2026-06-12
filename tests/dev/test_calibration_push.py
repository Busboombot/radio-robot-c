"""Unit tests for robot_radio.calibration.push.push_calibration.

Tests both the NezhaProtocol path and the SerialConnection path.
No hardware required.
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from robot_radio.calibration.push import push_calibration


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_conn():
    """Return a mock SerialConnection that records send() calls."""
    conn = MagicMock()
    conn.send.return_value = {"responses": ["OK set"]}
    return conn


def _make_config(
    ml: float = 0.71659,
    mr: float = 0.70777,
    tw: int = 126,
    ol: float = 1.127,   # int8 = 127
    oa: float = 0.987,   # int8 = -13
):
    """Return a minimal mock RobotConfig with calibration fields."""
    cfg = MagicMock()
    cfg.calibration.mm_per_wheel_deg_left  = ml
    cfg.calibration.mm_per_wheel_deg_right = mr
    cal_spec = {"otos_linear_scale": ol, "otos_angular_scale": oa}
    cfg.calibration.otos_linear_scale  = ol
    cfg.calibration.otos_angular_scale = oa
    cfg.geometry.trackwidth = tw
    cfg.geometry.odometry_offset_mm = None
    cfg.wheels = MagicMock()
    cfg.wheels.wheel_diameter_mm = None
    return cfg


def _sent_cmds(conn: MagicMock) -> list[str]:
    return [c.args[0] for c in conn.send.call_args_list]


# ---------------------------------------------------------------------------
# SerialConnection path
# ---------------------------------------------------------------------------

class TestPushViaSerialConnection:
    """push_calibration(SerialConnection, config) constructs SET commands."""

    def test_emits_set_ml(self):
        from robot_radio.io.serial_conn import SerialConnection
        conn = _make_conn()
        conn.__class__ = SerialConnection
        result = push_calibration(conn, _make_config())
        assert any(c.startswith("SET ml=") for c in _sent_cmds(conn))

    def test_emits_set_mr(self):
        from robot_radio.io.serial_conn import SerialConnection
        conn = _make_conn()
        conn.__class__ = SerialConnection
        push_calibration(conn, _make_config())
        assert any(c.startswith("SET mr=") for c in _sent_cmds(conn))

    def test_emits_set_tw(self):
        from robot_radio.io.serial_conn import SerialConnection
        conn = _make_conn()
        conn.__class__ = SerialConnection
        push_calibration(conn, _make_config())
        assert any(c.startswith("SET tw=") for c in _sent_cmds(conn))

    def test_emits_oi(self):
        from robot_radio.io.serial_conn import SerialConnection
        conn = _make_conn()
        conn.__class__ = SerialConnection
        push_calibration(conn, _make_config())
        assert "OI" in _sent_cmds(conn)

    def test_emits_ol(self):
        from robot_radio.io.serial_conn import SerialConnection
        conn = _make_conn()
        conn.__class__ = SerialConnection
        push_calibration(conn, _make_config(ol=1.127))
        cmds = _sent_cmds(conn)
        ol_cmds = [c for c in cmds if c.startswith("OL ")]
        assert ol_cmds
        assert int(ol_cmds[0].split()[1]) == 127

    def test_emits_oa(self):
        from robot_radio.io.serial_conn import SerialConnection
        conn = _make_conn()
        conn.__class__ = SerialConnection
        push_calibration(conn, _make_config(oa=0.987))
        cmds = _sent_cmds(conn)
        oa_cmds = [c for c in cmds if c.startswith("OA ")]
        assert oa_cmds
        assert int(oa_cmds[0].split()[1]) == -13

    def test_oi_before_ol(self):
        from robot_radio.io.serial_conn import SerialConnection
        conn = _make_conn()
        conn.__class__ = SerialConnection
        push_calibration(conn, _make_config())
        cmds = _sent_cmds(conn)
        oi_idx = cmds.index("OI")
        ol_idx = next(i for i, c in enumerate(cmds) if c.startswith("OL "))
        assert oi_idx < ol_idx

    def test_returns_status_ok(self):
        from robot_radio.io.serial_conn import SerialConnection
        conn = _make_conn()
        conn.__class__ = SerialConnection
        result = push_calibration(conn, _make_config())
        assert result.get("status") == "ok"

    def test_returns_commands_list(self):
        from robot_radio.io.serial_conn import SerialConnection
        conn = _make_conn()
        conn.__class__ = SerialConnection
        result = push_calibration(conn, _make_config())
        assert "commands" in result
        assert isinstance(result["commands"], list)
        assert len(result["commands"]) >= 6  # ml mr tw OI OL OA


# ---------------------------------------------------------------------------
# NezhaProtocol path — no own push_calibration
# ---------------------------------------------------------------------------

class TestPushViaNezhaProtocolFallback:
    """When NezhaProtocol has no push_calibration method, falls back to conn path."""

    def test_extracts_conn_and_sends_commands(self):
        from robot_radio.robot.protocol import NezhaProtocol

        inner_conn = _make_conn()
        # Build a real-class NezhaProtocol but don't give it push_calibration
        proto = MagicMock(spec=NezhaProtocol)
        proto._conn = inner_conn
        # Remove push_calibration from the spec so isinstance check routes correctly
        if hasattr(type(proto), "push_calibration"):
            # It shouldn't be there, but defensive: delete from spec
            pass

        # Ensure NezhaProtocol doesn't have push_calibration
        assert not hasattr(NezhaProtocol, "push_calibration"), (
            "NezhaProtocol.push_calibration was added — update this test for 028-003"
        )

        result = push_calibration(proto, _make_config())
        cmds = _sent_cmds(inner_conn)
        assert any(c.startswith("SET ml=") for c in cmds)
        assert "OI" in cmds
        assert result.get("status") == "ok"


# ---------------------------------------------------------------------------
# NezhaProtocol path — with own push_calibration (forward-compat test)
# ---------------------------------------------------------------------------

class TestPushViaNezhaProtocolDelegates:
    """When NezhaProtocol has push_calibration, it is called instead."""

    def test_delegates_to_proto_push_calibration(self):
        from robot_radio.robot.protocol import NezhaProtocol

        cfg = _make_config()
        expected_result = {"status": "ok", "delegated": True}
        delegate_fn = MagicMock(return_value=expected_result)

        # Patch push_calibration onto the NezhaProtocol class so the
        # hasattr / callable checks in push.py see it, then create a
        # real-class proto instance whose push_calibration is the mock.
        with patch.object(NezhaProtocol, "push_calibration",
                          delegate_fn, create=True):
            proto = MagicMock(spec=NezhaProtocol)
            proto.push_calibration = delegate_fn
            result = push_calibration(proto, cfg)

        delegate_fn.assert_called_once_with(cfg)
        assert result.get("delegated") is True


# ---------------------------------------------------------------------------
# Invalid input
# ---------------------------------------------------------------------------

class TestPushInvalidInput:

    def test_raises_type_error_on_wrong_type(self):
        with pytest.raises(TypeError, match="NezhaProtocol or SerialConnection"):
            push_calibration("not-a-conn", _make_config())
