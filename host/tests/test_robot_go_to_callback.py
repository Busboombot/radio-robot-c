"""Tests for callback-driven go_to and turn with _run_until_done loop.

Covers the 6 cases from the Sprint 036 ticket 003 plan:
1. test_go_to_no_callback_blocking      — on_tick=None uses wait_for_evt_done; STREAM not enabled.
2. test_go_to_callback_receives_ticks   — callback called per TLM; state updated; outcome "done".
3. test_go_to_callback_abort_on_false   — callback returns False; X sent; outcome "aborted".
4. test_go_to_callback_safety_stop      — EVT safety_stop; outcome "safety_stop"; STREAM disabled.
5. test_turn_no_callback                — turn(9000) sends TURN; waits for EVT done TURN; "done".
6. test_turn_callback_abort             — turn callback returns False; outcome "aborted".

All tests must be sub-second. read_lines side_effect is always finite — never
an infinite iterator — to avoid the OOM / infinite-loop hazard documented in
test_nezha_drive.py.
"""

from __future__ import annotations

import time
from unittest.mock import MagicMock, call, patch

import pytest

from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.robot.nezha import Nezha


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mock_conn() -> MagicMock:
    """Create a mock SerialConnection with safe, non-blocking defaults."""
    conn = MagicMock()
    conn.is_open = True
    conn.mode = "relay"
    conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": []}
    conn.send_fast.return_value = None
    conn.read_lines.return_value = []
    return conn


def _nezha(conn: MagicMock) -> Nezha:
    proto = NezhaProtocol(conn)
    return Nezha(proto)


# ---------------------------------------------------------------------------
# Case 1: no-callback blocking path
# ---------------------------------------------------------------------------

class TestGoToNoCallback:
    """go_to(x, y, speed) with on_tick=None uses the blocking wait_for_evt_done path."""

    def test_go_to_no_callback_sends_G_and_waits(self) -> None:
        """on_tick=None: sends G command, calls wait_for_evt_done, returns (enc, enc, outcome)."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "G", "mode": "relay",
                                  "responses": ["OK goto x=300 y=0 speed=200"]}
        conn.read_lines.return_value = ["EVT done G"]

        robot = _nezha(conn)
        with patch("robot_radio.robot.nezha.time.sleep"):
            left, right, outcome = robot.go_to(300, 0, 200)

        assert outcome == "done"
        assert isinstance(left, int)
        assert isinstance(right, int)

        # Verify G command sent
        sent_cmds = [c[0][0] for c in conn.send.call_args_list]
        assert any("G 300 0 200" in cmd for cmd in sent_cmds), \
            f"Expected 'G 300 0 200' in send calls: {sent_cmds}"

    def test_go_to_no_callback_does_not_enable_stream(self) -> None:
        """on_tick=None: STREAM must NOT be enabled (no 'STREAM' send calls)."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "G", "mode": "relay", "responses": []}
        conn.read_lines.return_value = ["EVT done G"]

        robot = _nezha(conn)
        with patch("robot_radio.robot.nezha.time.sleep"):
            robot.go_to(100, 100, 150)

        # No send call should contain STREAM
        all_sends = [c[0][0] for c in conn.send.call_args_list]
        assert not any(cmd.startswith("STREAM") for cmd in all_sends), \
            f"STREAM should not be called in blocking path, got: {all_sends}"

    def test_go_to_no_callback_returns_safety_stop(self) -> None:
        """on_tick=None: returns (enc, enc, 'safety_stop') on EVT safety_stop."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "G", "mode": "relay", "responses": []}
        conn.read_lines.return_value = ["EVT safety_stop"]

        robot = _nezha(conn)
        with patch("robot_radio.robot.nezha.time.sleep"):
            left, right, outcome = robot.go_to(100, 100, 150)

        assert outcome == "safety_stop"


# ---------------------------------------------------------------------------
# Case 2: callback receives ticks and state is updated
# ---------------------------------------------------------------------------

class TestGoToCallbackTicks:
    """go_to with on_tick: callback called per TLM frame; state updated each time."""

    def test_go_to_callback_called_per_tlm(self) -> None:
        """Callback is called once per TLM frame; outcome is 'done'."""
        conn = _mock_conn()
        # STREAM enable + G command both return OK immediately
        conn.send.return_value = {"sent": "CMD", "mode": "relay",
                                  "responses": ["OK"]}

        # Two TLM frames then EVT done G — finite, never loops
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9 pose=50,0,0"],
            ["TLM t=200 enc=20,19 pose=100,0,0"],
            ["EVT done G"],
        ]

        tick_calls: list = []
        def on_tick(robot: Nezha) -> None:
            tick_calls.append(robot.encoders)

        robot = _nezha(conn)
        left, right, outcome = robot.go_to(300, 0, 200, on_tick=on_tick)

        assert outcome == "done"
        assert len(tick_calls) == 2, f"Expected 2 tick calls, got {len(tick_calls)}"

    def test_go_to_callback_state_updated_each_tick(self) -> None:
        """Robot state (encoders) is updated from each TLM frame before callback."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}

        observed_encoders: list = []
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=50,48"],
            ["TLM t=200 enc=100,99"],
            ["EVT done G"],
        ]

        def on_tick(robot: Nezha) -> None:
            observed_encoders.append(robot.encoders)

        robot = _nezha(conn)
        robot.go_to(300, 0, 200, on_tick=on_tick)

        assert observed_encoders[0] == (50, 48)
        assert observed_encoders[1] == (100, 99)

    def test_go_to_callback_enables_stream(self) -> None:
        """on_tick provided: STREAM 80 is enabled before issuing G."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.side_effect = [
            ["EVT done G"],
        ]

        def on_tick(robot: Nezha) -> None:
            pass

        robot = _nezha(conn)
        robot.go_to(300, 0, 200, on_tick=on_tick)

        all_sends = [c[0][0] for c in conn.send.call_args_list]
        assert any(cmd.startswith("STREAM 80") for cmd in all_sends), \
            f"Expected 'STREAM 80' in send calls: {all_sends}"


# ---------------------------------------------------------------------------
# Case 3: abort on False
# ---------------------------------------------------------------------------

class TestGoToCallbackAbort:
    """go_to with on_tick returning False: sends X, outcome 'aborted'."""

    def test_abort_sends_X(self) -> None:
        """When on_tick returns False, send_fast('X') is called."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            # Would not be reached after abort, but guard against runaway
            ["EVT done G"],
        ]

        def on_tick(robot: Nezha):
            return False

        robot = _nezha(conn)
        left, right, outcome = robot.go_to(300, 0, 200, on_tick=on_tick)

        assert outcome == "aborted"
        fast_calls = [c[0][0] for c in conn.send_fast.call_args_list]
        assert "X" in fast_calls, f"Expected X in send_fast calls: {fast_calls}"

    def test_abort_state_reflects_last_tlm(self) -> None:
        """After abort, robot.state reflects the last received TLM frame."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=42,41"],
            ["EVT done G"],  # not reached
        ]

        def on_tick(robot: Nezha):
            return False

        robot = _nezha(conn)
        robot.go_to(300, 0, 200, on_tick=on_tick)

        assert robot.encoders == (42, 41)

    def test_abort_after_first_tick_not_second(self) -> None:
        """Callback returns True on first tick, False on second → aborted."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            ["TLM t=200 enc=20,19"],
            ["EVT done G"],  # not reached
        ]

        call_count = [0]
        def on_tick(robot: Nezha):
            call_count[0] += 1
            return call_count[0] < 2  # True first time, False second

        robot = _nezha(conn)
        left, right, outcome = robot.go_to(300, 0, 200, on_tick=on_tick)

        assert outcome == "aborted"
        assert call_count[0] == 2


# ---------------------------------------------------------------------------
# Case 4: safety_stop via callback path
# ---------------------------------------------------------------------------

class TestGoToCallbackSafetyStop:
    """EVT safety_stop during callback-driven go_to: outcome 'safety_stop', STREAM disabled."""

    def test_safety_stop_outcome(self) -> None:
        """EVT safety_stop → outcome 'safety_stop'."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=10,9"],
            ["EVT safety_stop"],
        ]

        tick_calls = [0]
        def on_tick(robot: Nezha) -> None:
            tick_calls[0] += 1

        robot = _nezha(conn)
        left, right, outcome = robot.go_to(300, 0, 200, on_tick=on_tick)

        assert outcome == "safety_stop"
        assert tick_calls[0] == 1  # one TLM tick before safety_stop

    def test_safety_stop_disables_stream(self) -> None:
        """EVT safety_stop: STREAM 0 is sent to disable streaming."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.side_effect = [
            ["EVT safety_stop"],
        ]

        def on_tick(robot: Nezha) -> None:
            pass

        robot = _nezha(conn)
        robot.go_to(300, 0, 200, on_tick=on_tick)

        all_sends = [c[0][0] for c in conn.send.call_args_list]
        assert any(cmd == "STREAM 0" for cmd in all_sends), \
            f"Expected 'STREAM 0' to disable stream after safety_stop: {all_sends}"


# ---------------------------------------------------------------------------
# Case 5: turn no-callback blocking path
# ---------------------------------------------------------------------------

class TestTurnNoCallback:
    """turn(heading_cdeg) with no on_tick: sends TURN, waits for EVT done TURN."""

    def test_turn_sends_TURN_command(self) -> None:
        """turn(9000) sends 'TURN 9000' command."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "TURN 9000", "mode": "relay",
                                  "responses": ["OK turn heading=9000 eps=300"]}
        conn.read_lines.return_value = ["EVT done TURN"]

        robot = _nezha(conn)
        outcome = robot.turn(9000)

        sent_cmds = [c[0][0] for c in conn.send.call_args_list]
        assert any("TURN 9000" in cmd for cmd in sent_cmds), \
            f"Expected 'TURN 9000' in send calls: {sent_cmds}"

    def test_turn_returns_done(self) -> None:
        """turn() returns 'done' when EVT done TURN received."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "TURN", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.return_value = ["EVT done TURN"]

        robot = _nezha(conn)
        outcome = robot.turn(9000)

        assert outcome == "done"

    def test_turn_returns_safety_stop(self) -> None:
        """turn() returns 'safety_stop' on EVT safety_stop."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "TURN", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.return_value = ["EVT safety_stop"]

        robot = _nezha(conn)
        outcome = robot.turn(9000)

        assert outcome == "safety_stop"

    def test_turn_no_callback_does_not_enable_stream(self) -> None:
        """turn with no callback must NOT enable STREAM."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "TURN", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.return_value = ["EVT done TURN"]

        robot = _nezha(conn)
        robot.turn(9000)

        all_sends = [c[0][0] for c in conn.send.call_args_list]
        assert not any(cmd.startswith("STREAM") for cmd in all_sends), \
            f"STREAM must not be called in blocking turn path: {all_sends}"

    def test_turn_with_eps_cdeg(self) -> None:
        """turn(9000, eps_cdeg=100) includes eps=100 in TURN command."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "TURN", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.return_value = ["EVT done TURN"]

        robot = _nezha(conn)
        robot.turn(9000, eps_cdeg=100)

        all_sends = [c[0][0] for c in conn.send.call_args_list]
        assert any("eps=100" in cmd for cmd in all_sends), \
            f"Expected eps=100 in TURN command: {all_sends}"


# ---------------------------------------------------------------------------
# Case 6: turn callback abort
# ---------------------------------------------------------------------------

class TestTurnCallbackAbort:
    """turn with on_tick returning False: sends X, outcome 'aborted'."""

    def test_turn_callback_abort_outcome(self) -> None:
        """turn on_tick returns False → outcome 'aborted'."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=5,5"],
            ["EVT done TURN"],  # not reached
        ]

        def on_tick(robot: Nezha):
            return False

        robot = _nezha(conn)
        outcome = robot.turn(9000, on_tick=on_tick)

        assert outcome == "aborted"

    def test_turn_callback_abort_sends_X(self) -> None:
        """turn abort: send_fast('X') is called."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=5,5"],
            ["EVT done TURN"],
        ]

        def on_tick(robot: Nezha):
            return False

        robot = _nezha(conn)
        robot.turn(9000, on_tick=on_tick)

        fast_calls = [c[0][0] for c in conn.send_fast.call_args_list]
        assert "X" in fast_calls, f"Expected X in send_fast calls: {fast_calls}"

    def test_turn_callback_enables_stream(self) -> None:
        """turn with on_tick: STREAM 80 is enabled before TURN command."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.side_effect = [
            ["EVT done TURN"],
        ]

        def on_tick(robot: Nezha) -> None:
            pass

        robot = _nezha(conn)
        robot.turn(9000, on_tick=on_tick)

        all_sends = [c[0][0] for c in conn.send.call_args_list]
        assert any(cmd.startswith("STREAM 80") for cmd in all_sends), \
            f"Expected 'STREAM 80' in send calls for turn with callback: {all_sends}"

    def test_turn_callback_done(self) -> None:
        """turn callback path: EVT done TURN → outcome 'done'."""
        conn = _mock_conn()
        conn.send.return_value = {"sent": "CMD", "mode": "relay", "responses": ["OK"]}
        conn.read_lines.side_effect = [
            ["TLM t=100 enc=5,5"],
            ["EVT done TURN"],
        ]

        tick_count = [0]
        def on_tick(robot: Nezha):
            tick_count[0] += 1
            return True

        robot = _nezha(conn)
        outcome = robot.turn(9000, on_tick=on_tick)

        assert outcome == "done"
        assert tick_count[0] == 1
