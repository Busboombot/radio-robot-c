"""Unit tests for _push_calibration() — v2 verb sequence.

Verifies that the function:
  - emits only v2 verbs (SET ml, SET mr, SET tw, OI, OL, OA)
  - does NOT emit any v1 dead verbs (KML, KMR, OO, OK)
  - sends every command with ack-gated blocking (conn.send, not send_fast)
  - correctly resolves robot config via ID + match_robot_by_id

All tests are pure Python — no serial hardware required.
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from unittest.mock import MagicMock, call, patch

import pytest

# Import under test
from robot_radio.io.cli import _push_calibration
from robot_radio.config.robot_config import (
    _reset_robot_config,
    load_robot_config,
)

# ---------------------------------------------------------------------------
# Helpers / fixtures
# ---------------------------------------------------------------------------

_REPO_ROOT = Path(__file__).parent.parent.parent
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"


def _make_conn(id_response: str | None = "ID model=Nezha2 name=tovez serial=89f137c0") -> MagicMock:
    """Return a mock SerialConnection whose send() records all calls.

    If *id_response* is not None, the first call (ID) returns a response
    dict containing that line. Subsequent calls return a generic OK response.
    """
    conn = MagicMock()

    def _send_side_effect(cmd: str, read_ms: int = 500, **_kwargs):
        if cmd == "ID" and id_response is not None:
            return {"responses": [id_response]}
        return {"responses": ["OK set"]}

    conn.send.side_effect = _send_side_effect
    return conn


@pytest.fixture(autouse=True)
def reset_config_cache():
    """Clear the singleton cache before and after each test."""
    _reset_robot_config()
    yield
    _reset_robot_config()


@pytest.fixture(autouse=True)
def clear_env(monkeypatch):
    """Ensure ROBOT_CONFIG env does not pollute test isolation."""
    monkeypatch.delenv("ROBOT_CONFIG", raising=False)


# ---------------------------------------------------------------------------
# Helper: collect all commands sent
# ---------------------------------------------------------------------------

def _sent_cmds(conn: MagicMock) -> list[str]:
    """Return list of all positional first-arg strings passed to conn.send()."""
    return [c.args[0] for c in conn.send.call_args_list]


# ---------------------------------------------------------------------------
# Core v2 verb sequence tests
# ---------------------------------------------------------------------------

class TestPushCalibrationV2Verbs:
    """_push_calibration emits the correct v2 verb sequence."""

    def test_emits_set_ml(self):
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        assert any(c.startswith("SET ml=") for c in cmds), \
            f"Expected 'SET ml=...' in {cmds}"

    def test_emits_set_mr(self):
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        assert any(c.startswith("SET mr=") for c in cmds), \
            f"Expected 'SET mr=...' in {cmds}"

    def test_emits_set_tw(self):
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        assert any(c.startswith("SET tw=") for c in cmds), \
            f"Expected 'SET tw=...' in {cmds}"

    def test_emits_oi(self):
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        assert "OI" in cmds, f"Expected 'OI' in {cmds}"

    def test_emits_ol(self):
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        assert any(c.startswith("OL ") for c in cmds), \
            f"Expected 'OL <int8>' in {cmds}"

    def test_emits_oa(self):
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        assert any(c.startswith("OA ") for c in cmds), \
            f"Expected 'OA <int8>' in {cmds}"


class TestPushCalibrationDeadVerbs:
    """_push_calibration must NOT emit any v1 dead verbs."""

    _DEAD = ("KML", "KMR", "K+ML", "K+MR", "OO", "OK")

    def test_no_dead_verbs(self):
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        # ID command is sent — that is expected; filter it out
        non_id = [c for c in cmds if c != "ID"]
        for cmd in non_id:
            for dead in self._DEAD:
                assert not cmd.startswith(dead), \
                    f"Dead verb {dead!r} found in command: {cmd!r}"


class TestPushCalibrationAckGated:
    """Every command must be sent with conn.send() (ack-gated), not send_fast."""

    def test_uses_send_not_send_fast(self):
        conn = _make_conn()
        _push_calibration(conn)
        # send_fast must never be called
        assert conn.send_fast.call_count == 0, \
            f"send_fast was called {conn.send_fast.call_count} time(s); expected 0"

    def test_send_called_multiple_times(self):
        conn = _make_conn()
        _push_calibration(conn)
        # At minimum: ID + SET ml + SET mr + SET tw + OI + OL + OA = 7
        assert conn.send.call_count >= 7, \
            f"Expected at least 7 send() calls, got {conn.send.call_count}"


# ---------------------------------------------------------------------------
# Config values roundtrip (tovez)
# ---------------------------------------------------------------------------

class TestPushCalibrationValues:
    """The values sent match the tovez.json config."""

    def test_set_ml_value(self):
        """SET ml encodes mm_per_wheel_deg_left from tovez (0.487)."""
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        ml_cmds = [c for c in cmds if c.startswith("SET ml=")]
        assert ml_cmds, "No SET ml= command found"
        val_str = ml_cmds[0].split("=", 1)[1]
        val = float(val_str)
        assert abs(val - 0.487) < 1e-4, f"Expected ml≈0.487, got {val}"

    def test_set_mr_value(self):
        """SET mr encodes mm_per_wheel_deg_right from tovez (0.481)."""
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        mr_cmds = [c for c in cmds if c.startswith("SET mr=")]
        assert mr_cmds, "No SET mr= command found"
        val_str = mr_cmds[0].split("=", 1)[1]
        val = float(val_str)
        assert abs(val - 0.481) < 1e-4, f"Expected mr≈0.481, got {val}"

    def test_set_tw_value(self):
        """SET tw encodes trackwidth from tovez (126 mm)."""
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        tw_cmds = [c for c in cmds if c.startswith("SET tw=")]
        assert tw_cmds, "No SET tw= command found"
        val = int(tw_cmds[0].split("=", 1)[1])
        assert val == 126, f"Expected tw=126, got {val}"

    def test_ol_value(self):
        """OL value matches tovez otos_linear_scale=1.05 → int8=50."""
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        ol_cmds = [c for c in cmds if c.startswith("OL ")]
        assert ol_cmds, "No OL command found"
        val = int(ol_cmds[0].split()[1])
        assert val == 50, f"Expected OL 50, got {val}"

    def test_oa_value(self):
        """OA value matches tovez otos_angular_scale=0.987 → int8=-13."""
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        oa_cmds = [c for c in cmds if c.startswith("OA ")]
        assert oa_cmds, "No OA command found"
        val = int(oa_cmds[0].split()[1])
        assert val == -13, f"Expected OA -13, got {val}"


# ---------------------------------------------------------------------------
# Robot ID resolution
# ---------------------------------------------------------------------------

class TestPushCalibrationIdResolution:
    """_push_calibration sends ID first and uses match_robot_by_id."""

    def test_sends_id_first(self):
        """ID must be the first command sent."""
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        assert cmds[0] == "ID", \
            f"Expected first command to be 'ID', got {cmds[0]!r}"

    def test_falls_back_when_no_id_response(self):
        """When ID returns no response, falls back to get_robot_config() and still pushes."""
        conn = _make_conn(id_response=None)
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        # Should still push v2 verbs
        assert any(c.startswith("SET ml=") for c in cmds), \
            "Expected SET ml even when ID fails (fallback config)"
        assert "OI" in cmds

    def test_no_config_skips_push(self):
        """When no config is found at all, push is skipped gracefully."""
        conn = _make_conn(id_response="ID model=Nezha2 name=UNKNOWN_ROBOT")
        # Override match_robot_by_id fallback to return None
        with patch("robot_radio.io.cli.match_robot_by_id", return_value=None), \
             patch("robot_radio.io.cli.get_robot_config", return_value=None):
            _push_calibration(conn)
        cmds = _sent_cmds(conn)
        # Only ID should have been sent; no calibration commands
        non_id = [c for c in cmds if c != "ID"]
        assert non_id == [], \
            f"Expected no calibration commands when cfg=None, got: {non_id}"


# ---------------------------------------------------------------------------
# OI ordering — must precede OL/OA
# ---------------------------------------------------------------------------

class TestOiOrdering:
    """OI must appear before OL and OA in the send sequence."""

    def test_oi_before_ol(self):
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        oi_idx = cmds.index("OI")
        ol_cmds = [i for i, c in enumerate(cmds) if c.startswith("OL ")]
        assert ol_cmds, "OL not found"
        assert oi_idx < ol_cmds[0], \
            f"OI at {oi_idx} must precede OL at {ol_cmds[0]}"

    def test_oi_before_oa(self):
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        oi_idx = cmds.index("OI")
        oa_cmds = [i for i, c in enumerate(cmds) if c.startswith("OA ")]
        assert oa_cmds, "OA not found"
        assert oi_idx < oa_cmds[0], \
            f"OI at {oi_idx} must precede OA at {oa_cmds[0]}"


# ---------------------------------------------------------------------------
# Nonzero odom offsets are sent via SET keys
# ---------------------------------------------------------------------------

class TestOdomOffsets:
    """Nonzero odom offsets are pushed via SET odomOffX/odomOffY/odomYaw."""

    def test_zero_offsets_not_sent(self):
        """tovez has all-zero offsets; SET odomOff* should be absent."""
        conn = _make_conn()
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        odom = [c for c in cmds if "odomOff" in c or "odomYaw" in c]
        assert odom == [], \
            f"Expected no odomOff* commands for tovez (offsets=0), got: {odom}"

    def test_nonzero_offsets_sent(self, monkeypatch, tmp_path):
        """When config has nonzero offsets, SET odomOffX/Y/Yaw are sent."""
        cfg_data = json.loads(_TOVEZ_JSON.read_text())
        cfg_data["geometry"]["odometry_offset_mm"] = {"x": 10.0, "y": 5.0, "yaw_rad": 0.1}
        tmp_cfg = tmp_path / "robot_offset.json"
        tmp_cfg.write_text(json.dumps(cfg_data))
        monkeypatch.setenv("ROBOT_CONFIG", str(tmp_cfg))
        _reset_robot_config()

        # ID response won't match the temp file name — fall back via get_robot_config
        conn = _make_conn(id_response=None)
        _push_calibration(conn)
        cmds = _sent_cmds(conn)
        odom_x = [c for c in cmds if c.startswith("SET odomOffX=")]
        odom_y = [c for c in cmds if c.startswith("SET odomOffY=")]
        odom_yaw = [c for c in cmds if c.startswith("SET odomYaw=")]
        assert odom_x, f"Expected SET odomOffX in {cmds}"
        assert odom_y, f"Expected SET odomOffY in {cmds}"
        assert odom_yaw, f"Expected SET odomYaw in {cmds}"
