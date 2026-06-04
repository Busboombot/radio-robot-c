"""Unit tests for the Nezha high-level driver (v2).

Tests verify:
- connect() liveness preflight (PING + ID)
- speed_for_time: sends T command, waits for EVT done T
- speed_for_distance: hop loop with D commands
- go_to: sends G command, waits for EVT done G
- stop: sends STOP
- stream_drive: updates state from TLM, terminates on EVT safety_stop
- NezhaState: heading_rad updated from TLMFrame (cdeg → radians)
- No v1 artifacts in nezha.py

All tests must be sub-second. CRITICAL: read_lines side_effect must never
return [] in an infinite loop — always use a finite side_effect that ends
with a terminal event line or a small repeat. See test_protocol_v2.py
TestStreamDrive for the safety pattern.

Timing safety: any test that exercises wait_for_evt_done with read_lines=[]
MUST patch time.time to advance past the deadline immediately. Otherwise the
real clock makes the test wait for the full timeout (up to 6 seconds).
"""

from __future__ import annotations

import math
import itertools
import time
from unittest.mock import MagicMock, call, patch

import pytest

from robot_radio.robot.protocol import NezhaProtocol, TLMFrame, parse_tlm
from robot_radio.robot.nezha import Nezha, RobotNotFoundError
from robot_radio.robot.nezha_state import NezhaState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mock_conn() -> MagicMock:
    """Create a mock SerialConnection with safe defaults."""
    conn = MagicMock()
    conn.is_open = True
    conn.mode = "relay"
    conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": []}
    conn.send_fast.return_value = None
    conn.read_lines.return_value = []
    return conn


def _nezha_with_conn(conn: MagicMock) -> tuple[Nezha, NezhaProtocol]:
    proto = NezhaProtocol(conn)
    robot = Nezha(proto)
    return robot, proto


def _canned_send(responses_per_call: list[list[str]]) -> MagicMock:
    """Create a conn whose send() returns successive canned responses."""
    conn = _mock_conn()
    conn.send.side_effect = [
        {"sent": "CMD", "mode": "relay", "responses": lines}
        for lines in responses_per_call
    ]
    return conn


# ===========================================================================
# connect() — liveness preflight
# ===========================================================================

class TestConnectPreflight:
    """connect() sends PING then ID; raises RobotNotFoundError on failure."""

    def test_connect_preflight_success(self) -> None:
        """connect() returns identity dict when PING and ID both succeed."""
        conn = MagicMock()
        conn.is_open = True
        conn.mode = "relay"
        conn.send.side_effect = [
            {"sent": "PING", "mode": "relay", "responses": ["OK pong t=1"]},
            {"sent": "ID", "mode": "relay", "responses": [
                "ID model=Nezha2 name=TOVEZ serial=0 fw=2.0 proto=2 caps=otos,line"
            ]},
        ]
        conn.send_fast.return_value = None
        conn.read_lines.return_value = []

        robot, _ = _nezha_with_conn(conn)
        identity = robot.connect()

        assert identity is not None
        assert identity["model"] == "Nezha2"
        assert identity["name"] == "TOVEZ"

        # Verify PING and ID were sent in order
        calls = [c[0][0] for c in conn.send.call_args_list]
        assert calls[0] == "PING"
        assert calls[1] == "ID"

    def test_connect_preflight_ping_timeout_raises(self) -> None:
        """connect() raises RobotNotFoundError when PING returns None."""
        conn = _canned_send([
            [],  # PING — no response
        ])
        robot, _ = _nezha_with_conn(conn)
        with pytest.raises(RobotNotFoundError, match="PING"):
            robot.connect()

    def test_connect_preflight_id_timeout_raises(self) -> None:
        """connect() raises RobotNotFoundError when ID returns None."""
        conn = MagicMock()
        conn.is_open = True
        conn.mode = "relay"
        conn.send.side_effect = [
            {"sent": "PING", "mode": "relay", "responses": ["OK pong t=1"]},
            {"sent": "ID", "mode": "relay", "responses": []},  # ID silent
        ]
        conn.send_fast.return_value = None
        conn.read_lines.return_value = []
        robot, _ = _nezha_with_conn(conn)
        with pytest.raises(RobotNotFoundError, match="ID"):
            robot.connect()


# ===========================================================================
# speed_for_time — T command + wait_for_evt_done("T")
# ===========================================================================

class TestSpeedForTime:
    """speed_for_time sends T l r ms and blocks until EVT done T."""

    def test_speed_for_time_sends_T_command(self) -> None:
        """speed_for_time(200, 200, 1000) must send 'T 200 200 1000'."""
        conn = _mock_conn()
        # send() for T command initial response
        conn.send.return_value = {"sent": "T 200 200 1000", "mode": "relay",
                                  "responses": ["OK drive l=200 r=200 ms=1000"]}
        # read_lines returns EVT done T so it doesn't block
        conn.read_lines.return_value = ["EVT done T"]

        robot, _ = _nezha_with_conn(conn)
        result = robot.speed_for_time(200, 200, 1000)

        # Verify T command was sent
        sent_cmds = [c[0][0] for c in conn.send.call_args_list]
        assert any("T 200 200 1000" in cmd for cmd in sent_cmds), \
            f"Expected 'T 200 200 1000' in send calls: {sent_cmds}"

    def test_speed_for_time_waits_for_evt_done_T(self) -> None:
        """speed_for_time blocks until 'EVT done T' and returns encoders."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "T", "mode": "relay",
                                  "responses": ["OK drive l=100 r=100 ms=500"]}
        conn.read_lines.return_value = ["EVT done T"]

        robot, _ = _nezha_with_conn(conn)
        result = robot.speed_for_time(100, 100, 500)

        # Should return encoder tuple
        assert isinstance(result, tuple)
        assert len(result) == 2

    def test_speed_for_time_safety_stop_returns_normally(self) -> None:
        """speed_for_time returns if safety_stop received (does not raise)."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "T", "mode": "relay",
                                  "responses": ["OK drive"]}
        # Return safety_stop — wait_for_evt_done returns "safety_stop"
        conn.read_lines.return_value = ["EVT safety_stop"]

        robot, _ = _nezha_with_conn(conn)
        # speed_for_time delegates to wait_for_evt_done but doesn't raise on safety_stop
        result = robot.speed_for_time(100, 100, 500)
        assert isinstance(result, tuple)

    def test_speed_for_time_clamps_min_speed(self) -> None:
        """Very slow non-zero speed is clamped to MIN_SPEED_MMS."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "T", "mode": "relay", "responses": []}
        conn.read_lines.return_value = ["EVT done T"]

        robot, _ = _nezha_with_conn(conn)
        robot.speed_for_time(1, 1, 200)  # speed=1 < MIN_SPEED_MMS

        sent_cmds = [c[0][0] for c in conn.send.call_args_list]
        t_cmd = next((c for c in sent_cmds if c.startswith("T ")), None)
        assert t_cmd is not None
        parts = t_cmd.split()
        # l and r should be clamped to at least MIN_SPEED_MMS (12)
        assert int(parts[1]) >= Nezha.MIN_SPEED_MMS
        assert int(parts[2]) >= Nezha.MIN_SPEED_MMS

    def test_speed_for_time_no_v1_commands(self) -> None:
        """speed_for_time must not use v1 sign-prefix format (T+200+200+1000)."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "T", "mode": "relay", "responses": []}
        conn.read_lines.return_value = ["EVT done T"]

        robot, _ = _nezha_with_conn(conn)
        robot.speed_for_time(200, 200, 1000)

        sent_cmds = [c[0][0] for c in conn.send.call_args_list]
        for cmd in sent_cmds:
            assert "+" not in cmd, f"v1 sign-prefix found in command: {cmd!r}"


# ===========================================================================
# speed_for_distance — hop loop with D commands
# ===========================================================================

class TestSpeedForDistance:
    """speed_for_distance uses a hop loop: D l r hop_mm + wait_for_evt_done('D')."""

    def test_speed_for_distance_sends_D_command(self) -> None:
        """speed_for_distance must send a D command on each hop."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "D", "mode": "relay",
                                  "responses": ["OK drive l=200 r=200 mm=100"]}
        conn.read_lines.return_value = ["EVT done D"]

        robot, _ = _nezha_with_conn(conn)
        robot.speed_for_distance(200, 200, 100)

        sent_cmds = [c[0][0] for c in conn.send.call_args_list]
        assert any(cmd.startswith("D ") for cmd in sent_cmds), \
            f"Expected D command in: {sent_cmds}"

    def test_speed_for_distance_multiple_hops(self) -> None:
        """Large distance should produce multiple D hops."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "D", "mode": "relay", "responses": []}
        conn.read_lines.return_value = ["EVT done D"]

        robot, _ = _nezha_with_conn(conn)
        # 1000mm at 200mm/s → hop_mm_max = max(40, 300) = 300mm, so 4 hops
        robot.speed_for_distance(200, 200, 1000)

        d_cmds = [c[0][0] for c in conn.send.call_args_list
                  if c[0][0].startswith("D ")]
        assert len(d_cmds) >= 2, f"Expected multiple D hops, got: {d_cmds}"

    def test_speed_for_distance_zero_returns_immediately(self) -> None:
        """speed_for_distance(_, _, 0) returns without sending any commands."""
        conn = _mock_conn()
        robot, _ = _nezha_with_conn(conn)
        result = robot.speed_for_distance(200, 200, 0)

        # No D commands should be sent
        d_cmds = [c[0][0] for c in conn.send.call_args_list
                  if c[0][0].startswith("D ")]
        assert len(d_cmds) == 0
        assert result == (0, 0)

    def test_speed_for_distance_timeout_raises(self) -> None:
        """speed_for_distance raises TimeoutError if a hop times out.

        Patches time.time to expire the deadline immediately so the test
        returns in < 1 ms rather than waiting the full 6 s default.
        """
        conn = _mock_conn()
        conn.send.return_value = {"sent": "D", "mode": "relay", "responses": []}
        # read_lines returns [] — but with time patched to past-deadline,
        # wait_for_evt_done exits the while loop immediately → returns "timeout".
        conn.read_lines.return_value = []

        robot, _ = _nezha_with_conn(conn)
        # Patch time.time in the protocol module so the deadline is already expired.
        import robot_radio.robot.protocol as proto_mod
        _real_time = proto_mod.time.time
        _calls = [0]

        def _instant_expire() -> float:
            # First call → set deadline; second call → already past it.
            _calls[0] += 1
            return float(_calls[0] * 10000)  # large jumps ensure deadline expires

        proto_mod.time.time = _instant_expire  # type: ignore[attr-defined]
        try:
            with pytest.raises(TimeoutError):
                robot.speed_for_distance(200, 200, 100)
        finally:
            proto_mod.time.time = _real_time  # type: ignore[attr-defined]


# ===========================================================================
# go_to — G command + wait_for_evt_done("G")
# ===========================================================================

class TestGoTo:
    """go_to sends G x y speed and blocks until EVT done G."""

    def test_go_to_sends_G_command(self) -> None:
        """go_to(300, 0, 200) must send 'G 300 0 200'."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "G", "mode": "relay",
                                  "responses": ["OK goto x=300 y=0 speed=200"]}
        conn.read_lines.return_value = ["EVT done G"]

        robot, _ = _nezha_with_conn(conn)
        with patch("robot_radio.robot.nezha.time.sleep"):
            robot.go_to(300, 0, 200)

        sent_cmds = [c[0][0] for c in conn.send.call_args_list]
        assert any(cmd == "G 300 0 200" for cmd in sent_cmds), \
            f"Expected 'G 300 0 200' in: {sent_cmds}"

    def test_go_to_returns_outcome_done(self) -> None:
        """go_to returns (left_enc, right_enc, 'done') when EVT done G received."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "G", "mode": "relay", "responses": []}
        conn.read_lines.return_value = ["EVT done G"]

        robot, _ = _nezha_with_conn(conn)
        with patch("robot_radio.robot.nezha.time.sleep"):
            left, right, outcome = robot.go_to(100, 100, 150)

        assert outcome == "done"
        assert isinstance(left, int)
        assert isinstance(right, int)

    def test_go_to_returns_outcome_safety_stop(self) -> None:
        """go_to returns (enc, enc, 'safety_stop') on safety_stop event."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "G", "mode": "relay", "responses": []}
        conn.read_lines.return_value = ["EVT safety_stop"]

        robot, _ = _nezha_with_conn(conn)
        with patch("robot_radio.robot.nezha.time.sleep"):
            left, right, outcome = robot.go_to(100, 100, 150)

        assert outcome == "safety_stop"


# ===========================================================================
# stop() — sends STOP
# ===========================================================================

class TestStop:
    """stop() sends STOP to the protocol."""

    def test_stop_sends_STOP(self) -> None:
        """stop() must call protocol.stop() which sends 'STOP'."""
        conn = _mock_conn()
        robot, _ = _nezha_with_conn(conn)
        robot.stop()

        conn.send_fast.assert_called_once_with("STOP")

    def test_stop_does_not_send_X(self) -> None:
        """stop() must not use v1 'X' command."""
        conn = _mock_conn()
        robot, _ = _nezha_with_conn(conn)
        robot.stop()

        fast_calls = [c[0][0] for c in conn.send_fast.call_args_list]
        assert "X" not in fast_calls, f"v1 X command found in: {fast_calls}"


# ===========================================================================
# stream_drive — TLM state updates and safety_stop termination
# ===========================================================================

class TestStreamDrive:
    """stream_drive updates state from TLM, terminates on EVT safety_stop.

    CRITICAL memory safety: read_lines side_effect must NEVER return [] in
    an infinite iterator when the generator only exits on safety_stop.
    Always end the side_effect with the terminal event.
    """

    def test_stream_drive_updates_encoders_from_tlm(self) -> None:
        """stream_drive updates self.encoders from TLM enc field."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "STREAM 40", "mode": "relay",
                                  "responses": ["OK stream period=40"]}
        # Finite side_effect: TLM frame then safety_stop to terminate
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=50,48"],
            ["EVT safety_stop"],
        ]

        robot, _ = _nezha_with_conn(conn)
        speeds = [100, 100]
        # Consume the generator
        frames = list(robot.stream_drive(speeds))

        assert robot.encoders == (50, 48)

    def test_stream_drive_updates_otos_pose_from_tlm(self) -> None:
        """stream_drive updates self.otos_pose from TLM pose field."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "STREAM 40", "mode": "relay",
                                  "responses": ["OK stream period=40"]}
        conn.read_lines.side_effect = [
            ["TLM t=200 pose=100,50,9000"],
            ["EVT safety_stop"],
        ]

        robot, _ = _nezha_with_conn(conn)
        speeds = [100, 100]
        list(robot.stream_drive(speeds))

        x_mm, y_mm, yaw_rad = robot.otos_pose
        assert x_mm == pytest.approx(100.0)
        assert y_mm == pytest.approx(50.0)
        # 9000 cdeg = 90 degrees = pi/2 radians
        assert yaw_rad == pytest.approx(math.pi / 2, rel=1e-4)

    def test_stream_drive_heading_converted_to_radians(self) -> None:
        """Heading in TLM centidegrees is converted to radians in otos_pose."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "STREAM 40", "mode": "relay",
                                  "responses": ["OK stream period=40"]}
        # 18000 cdeg = 180 degrees = pi radians
        conn.read_lines.side_effect = [
            ["TLM t=300 pose=0,0,18000"],
            ["EVT safety_stop"],
        ]

        robot, _ = _nezha_with_conn(conn)
        list(robot.stream_drive([100, 100]))

        _, _, yaw_rad = robot.otos_pose
        assert yaw_rad == pytest.approx(math.pi, rel=1e-5)

    def test_stream_drive_terminates_on_safety_stop(self) -> None:
        """stream_drive ends when EVT safety_stop is received."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "STREAM 40", "mode": "relay",
                                  "responses": ["OK stream period=40"]}
        conn.read_lines.side_effect = itertools.chain(
            [["EVT safety_stop"]],
            itertools.repeat([]),
        )

        robot, _ = _nezha_with_conn(conn)
        frames = list(robot.stream_drive([100, 100]))
        # safety_stop causes return before yield → no frames
        assert frames == []

    def test_stream_drive_yields_tlm_parsed_responses(self) -> None:
        """stream_drive yields ParsedResponse objects for each incoming line."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "STREAM 40", "mode": "relay",
                                  "responses": ["OK stream period=40"]}
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            ["EVT safety_stop"],
        ]

        robot, _ = _nezha_with_conn(conn)
        frames = list(robot.stream_drive([50, 50]))

        # One TLM frame yielded before safety_stop
        assert len(frames) == 1
        assert frames[0].tag == "TLM"


# ===========================================================================
# NezhaState — heading_rad updated from TLMFrame
# ===========================================================================

class TestNezhaStateHeadingRad:
    """NezhaState.heading_rad is set from TLM pose cdeg → radians."""

    def _make_state(self) -> tuple[NezhaState, MagicMock]:
        conn = _mock_conn()
        proto = NezhaProtocol(conn)
        state = NezhaState(proto)
        return state, conn

    def test_heading_rad_initial_zero(self) -> None:
        """heading_rad starts at 0.0."""
        state, _ = self._make_state()
        assert state.heading_rad == 0.0

    def test_heading_rad_updated_from_tlm_pose(self) -> None:
        """_process_line with TLM pose= updates heading_rad in radians."""
        state, _ = self._make_state()
        # 9000 cdeg = 90 degrees = pi/2 radians
        state._process_line("TLM t=1 pose=0,0,9000")
        assert state.heading_rad == pytest.approx(math.pi / 2, rel=1e-4)

    def test_heading_rad_180_degrees(self) -> None:
        """18000 cdeg = pi radians."""
        state, _ = self._make_state()
        state._process_line("TLM t=1 pose=100,50,18000")
        assert state.heading_rad == pytest.approx(math.pi, rel=1e-5)

    def test_heading_rad_negative(self) -> None:
        """Negative centidegrees produce negative radians."""
        state, _ = self._make_state()
        # -9000 cdeg = -90 degrees = -pi/2 radians
        state._process_line("TLM t=1 pose=0,0,-9000")
        assert state.heading_rad == pytest.approx(-math.pi / 2, rel=1e-4)

    def test_heading_rad_conversion_formula(self) -> None:
        """heading_rad = cdeg / 18000.0 * math.pi for arbitrary values."""
        state, _ = self._make_state()
        cdeg = 5000
        state._process_line(f"TLM t=1 pose=0,0,{cdeg}")
        expected = cdeg / 18000.0 * math.pi
        assert state.heading_rad == pytest.approx(expected, rel=1e-6)

    def test_encoders_updated_from_tlm(self) -> None:
        """NezhaState.encoders updated from TLM enc field."""
        state, _ = self._make_state()
        state._process_line("TLM t=1 enc=123,119")
        assert state.encoders == (123, 119)

    def test_tlm_without_pose_does_not_reset_heading_rad(self) -> None:
        """TLM line without pose= does not change heading_rad."""
        state, _ = self._make_state()
        state._process_line("TLM t=1 pose=0,0,9000")  # set to pi/2
        heading_before = state.heading_rad
        state._process_line("TLM t=2 enc=50,50")   # no pose field
        assert state.heading_rad == heading_before


# ===========================================================================
# No v1 artifacts in nezha module
# ===========================================================================

class TestNoV1Artifacts:
    """nezha.py must not contain v1 command strings or helpers."""

    def test_no_sign_function(self) -> None:
        """_sign() must not exist in nezha module."""
        import robot_radio.robot.nezha as nezha_mod
        assert not hasattr(nezha_mod, "_sign"), \
            "_sign() is a v1 artifact and must not exist in nezha.py"

    def test_speed_command_space_separated(self) -> None:
        """stream_drive/speed uses 'S l r' (space-separated), not sign-prefix."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "STREAM 40", "mode": "relay",
                                  "responses": ["OK stream period=40"]}
        # Return safety_stop immediately after one TLM frame so generator exits
        conn.read_lines.side_effect = [
            ["TLM t=1 enc=0,0"],
            ["EVT safety_stop"],
        ]
        robot, _ = _nezha_with_conn(conn)
        list(robot.stream_drive([200, 150]))

        # All send_fast calls must be 'S <l> <r>' format
        for c in conn.send_fast.call_args_list:
            cmd = c[0][0]
            if cmd.startswith("S "):
                assert "+" not in cmd, f"v1 sign prefix in: {cmd!r}"
                parts = cmd.split()
                assert len(parts) == 3, f"S command must have 3 tokens: {cmd!r}"

    def test_RobotNotFoundError_importable(self) -> None:
        """RobotNotFoundError must be importable from nezha module."""
        from robot_radio.robot.nezha import RobotNotFoundError
        assert issubclass(RobotNotFoundError, Exception)
