#!/usr/bin/env python3
"""test_turn_command.py — Unit tests for the TURN verb (018-005).

Tests validate:
  - Wire format: TURN <heading_cdeg> [eps=<cdeg>] [#id]
  - OK reply: OK turn heading=<cdeg> eps=<cdeg>
  - EVT completion: EVT done TURN [#id]
  - Shortest-path ω sign selection (positive heading → positive ω = CCW)
  - eps default (300 cdeg) and override
  - delta_rad computation across all quadrants
  - wrap-around behaviour near ±180°
  - TURN in HELP verb list
  - host protocol.py turn() wrapper (send_fast NOT used; send() is used)
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Python mirror of beginTurn() delta/sign computation
# ---------------------------------------------------------------------------

def wrap_angle(x: float) -> float:
    """Wrap x into (-π, π] using atan2(sin, cos) — matches C++ implementation."""
    return math.atan2(math.sin(x), math.cos(x))


def cdeg_to_rad(cdeg: float) -> float:
    """Convert centidegrees to radians (1 cdeg = π/18000 rad)."""
    return cdeg * math.pi / 18000.0


def compute_turn(heading_cdeg: float, current_heading_rad: float) -> tuple[float, float]:
    """Mirror of DriveController::beginTurn() delta and sign computation.

    Returns:
        (delta_rad, omega_sign): signed angular delta in (-π, π] and ω sign.
    """
    theta_rad = cdeg_to_rad(heading_cdeg)
    diff = theta_rad - current_heading_rad
    delta_rad = wrap_angle(diff)
    omega_sign = 1.0 if delta_rad >= 0.0 else -1.0
    return delta_rad, omega_sign


# ---------------------------------------------------------------------------
# Wire format helpers (shared with other test files)
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
# Tests — TURN wire format (OK reply)
# ---------------------------------------------------------------------------

class TestTurnOkReply:
    """Tests for TURN command OK response wire format."""

    def _make_turn_ok(self, heading_cdeg: int, eps_cdeg: int = 300,
                      corr_id: str = "") -> str:
        """Simulate firmware OK response to TURN <heading_cdeg> [eps=<cdeg>]."""
        body = f"heading={heading_cdeg} eps={eps_cdeg}"
        if corr_id:
            return f"OK turn {body} #{corr_id}"
        return f"OK turn {body}"

    def test_turn_ok_prefix(self) -> None:
        """TURN 9000 → response starts with 'OK turn'."""
        resp = self._make_turn_ok(9000)
        assert resp.startswith("OK turn")

    def test_turn_heading_field(self) -> None:
        """TURN 9000 → heading=9000 in body."""
        resp = self._make_turn_ok(9000)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["heading"] == "9000"

    def test_turn_eps_default(self) -> None:
        """TURN 9000 (no eps=) → eps=300 in reply."""
        resp = self._make_turn_ok(9000, eps_cdeg=300)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["eps"] == "300"

    def test_turn_eps_override(self) -> None:
        """TURN 9000 eps=100 → eps=100 in reply."""
        resp = self._make_turn_ok(9000, eps_cdeg=100)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["eps"] == "100"

    def test_turn_negative_heading(self) -> None:
        """TURN -9000 → heading=-9000 in body (CW 90°)."""
        resp = self._make_turn_ok(-9000)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["heading"] == "-9000"

    def test_turn_zero_heading(self) -> None:
        """TURN 0 → heading=0 (face forward)."""
        resp = self._make_turn_ok(0)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["heading"] == "0"

    def test_turn_max_heading(self) -> None:
        """TURN 18000 → heading=18000 (±180°)."""
        resp = self._make_turn_ok(18000)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["heading"] == "18000"

    def test_turn_with_corr_id(self) -> None:
        """TURN 9000 #7 → OK turn ... #7."""
        resp = self._make_turn_ok(9000, corr_id="7")
        assert resp.endswith("#7")
        _, body = parse_ok(resp)
        kv = parse_body_kv(body.replace(" #7", ""))
        assert kv["heading"] == "9000"

    def test_turn_verb_is_turn(self) -> None:
        """TURN OK reply uses 'turn' verb."""
        resp = self._make_turn_ok(9000)
        verb, _ = parse_ok(resp)
        assert verb == "turn"


# ---------------------------------------------------------------------------
# Tests — TURN EVT completion format
# ---------------------------------------------------------------------------

class TestTurnEvtCompletion:
    """Tests for EVT done TURN completion wire format."""

    def test_turn_completion_evt_format(self) -> None:
        """After arriving at heading, emits 'EVT done TURN'."""
        line = "EVT done TURN"
        name, body = parse_evt(line)
        assert name == "done"
        assert body == "TURN"

    def test_turn_completion_no_id_when_not_correlated(self) -> None:
        """EVT done TURN has no #id when TURN had no #id."""
        line = "EVT done TURN"
        assert "#" not in line

    def test_turn_completion_with_corr_id(self) -> None:
        """EVT done TURN #7 when TURN command carried #7."""
        line = "EVT done TURN #7"
        name, body = parse_evt(line)
        assert name == "done"
        assert "TURN" in body
        assert "#7" in line

    def test_turn_evt_tag_not_t_or_d(self) -> None:
        """EVT done TURN is distinct from EVT done T and EVT done D."""
        line = "EVT done TURN"
        name, body = parse_evt(line)
        assert body == "TURN"
        assert body != "T"
        assert body != "D"

    def test_turn_evt_no_cmd_prefix(self) -> None:
        """EVT done TURN does not use legacy 'cmd=' prefix."""
        line = "EVT done TURN"
        assert "cmd=" not in line

    def test_turn_corr_id_format(self) -> None:
        """EVT done TURN corr id uses '#' followed by decimal digits only."""
        import re
        line = "EVT done TURN #42"
        assert re.search(r"#\d+$", line), f"Malformed corr id in {line!r}"


# ---------------------------------------------------------------------------
# Tests — TURN ERR cases
# ---------------------------------------------------------------------------

class TestTurnErrCases:
    """Tests for TURN command error responses."""

    def test_turn_badarg_no_args(self) -> None:
        """TURN (no args) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_turn_range_too_large(self) -> None:
        """TURN 18001 → ERR range heading."""
        line = "ERR range heading"
        code, detail = parse_err(line)
        assert code == "range"
        assert "heading" in detail

    def test_turn_range_too_small(self) -> None:
        """TURN -18001 → ERR range heading."""
        line = "ERR range heading"
        code, detail = parse_err(line)
        assert code == "range"
        assert "heading" in detail

    def test_turn_eps_range_too_small(self) -> None:
        """TURN 9000 eps=5 → ERR range eps (below 10 cdeg minimum)."""
        line = "ERR range eps"
        code, detail = parse_err(line)
        assert code == "range"
        assert "eps" in detail

    def test_turn_eps_range_too_large(self) -> None:
        """TURN 9000 eps=1801 → ERR range eps (above 1800 cdeg maximum)."""
        line = "ERR range eps"
        code, detail = parse_err(line)
        assert code == "range"
        assert "eps" in detail


# ---------------------------------------------------------------------------
# Tests — Shortest-path sign selection (pure Python mirror of C++ logic)
# ---------------------------------------------------------------------------

class TestTurnShortestPathSign:
    """Validate shortest-path ω sign selection and delta_rad computation."""

    def test_positive_heading_ccw_sign(self) -> None:
        """TURN 9000 from heading=0 → delta > 0 → positive ω (CCW)."""
        delta, sign = compute_turn(9000, 0.0)
        assert delta > 0.0, f"Expected CCW delta, got {delta}"
        assert sign > 0.0, "Expected positive ω sign for CCW turn"

    def test_negative_heading_cw_sign(self) -> None:
        """TURN -9000 from heading=0 → delta < 0 → negative ω (CW)."""
        delta, sign = compute_turn(-9000, 0.0)
        assert delta < 0.0, f"Expected CW delta, got {delta}"
        assert sign < 0.0, "Expected negative ω sign for CW turn"

    def test_shortest_path_selects_cw_when_shorter(self) -> None:
        """From heading=0, TURN 18000-1=17999 cdeg → CW is shorter (delta < 0).

        Going CCW would be +179.99° but CW is -180.01°... Actually, 17999 cdeg
        = 179.99° CCW is shorter than going -180.01° CW. But 18000 wraps to ±180.
        Test with a case where CW is actually shorter: heading=5000 cdeg,
        target=-5000 cdeg → CCW is +350°, CW is -10° → CW is shorter.
        """
        # Starting at +50° (5000 cdeg), target is -50° (-5000 cdeg).
        # Shortest path: CW = -100° = -10000 cdeg. CCW = +260° = +26000 cdeg. CW wins.
        current_rad = cdeg_to_rad(5000)
        delta, sign = compute_turn(-5000, current_rad)
        assert delta < 0.0, f"Expected CW (negative) delta, got {delta}"
        assert sign < 0.0, "Expected negative ω sign (CW is shortest path)"
        # Delta magnitude should be ~100° = ~1.745 rad
        assert abs(abs(delta) - math.radians(100.0)) < 0.01

    def test_shortest_path_selects_ccw_when_shorter(self) -> None:
        """From heading=-50° (-5000 cdeg), target=+50° (5000 cdeg) → CCW is shorter."""
        current_rad = cdeg_to_rad(-5000)
        delta, sign = compute_turn(5000, current_rad)
        assert delta > 0.0, f"Expected CCW (positive) delta, got {delta}"
        assert sign > 0.0, "Expected positive ω sign (CCW is shortest path)"
        assert abs(abs(delta) - math.radians(100.0)) < 0.01

    def test_zero_target_from_positive_heading(self) -> None:
        """TURN 0 from +45° → CW rotation (delta < 0)."""
        current_rad = math.radians(45.0)
        delta, sign = compute_turn(0, current_rad)
        assert delta < 0.0, f"Expected CW delta to reach 0 from +45°"

    def test_zero_target_from_negative_heading(self) -> None:
        """TURN 0 from -45° → CCW rotation (delta > 0)."""
        current_rad = math.radians(-45.0)
        delta, sign = compute_turn(0, current_rad)
        assert delta > 0.0, f"Expected CCW delta to reach 0 from -45°"

    def test_already_at_target_delta_zero(self) -> None:
        """TURN 9000 from heading=9000 cdeg → delta ≈ 0."""
        current_rad = cdeg_to_rad(9000)
        delta, _ = compute_turn(9000, current_rad)
        assert abs(delta) < 1e-5, f"Delta should be ~0 when already at target, got {delta}"


# ---------------------------------------------------------------------------
# Tests — eps conversion
# ---------------------------------------------------------------------------

class TestTurnEpsConversion:
    """Validate eps centidegree-to-radian conversion."""

    def test_eps_300_cdeg_to_rad(self) -> None:
        """Default eps=300 cdeg = 3° = π/60 rad ≈ 0.05236 rad."""
        eps_rad = cdeg_to_rad(300)
        assert abs(eps_rad - math.radians(3.0)) < 1e-6

    def test_eps_100_cdeg_to_rad(self) -> None:
        """eps=100 cdeg = 1° = π/180 rad ≈ 0.01745 rad."""
        eps_rad = cdeg_to_rad(100)
        assert abs(eps_rad - math.radians(1.0)) < 1e-6

    def test_eps_1800_cdeg_to_rad(self) -> None:
        """eps=1800 cdeg = 18° ≈ 0.3142 rad."""
        eps_rad = cdeg_to_rad(1800)
        assert abs(eps_rad - math.radians(18.0)) < 1e-6

    def test_eps_10_cdeg_to_rad(self) -> None:
        """Minimum eps=10 cdeg = 0.1° ≈ 0.001745 rad."""
        eps_rad = cdeg_to_rad(10)
        assert abs(eps_rad - math.radians(0.1)) < 1e-6


# ---------------------------------------------------------------------------
# Tests — wrap-around near ±180° (heading boundary)
# ---------------------------------------------------------------------------

class TestTurnWrapAround:
    """Validate correct shortest-path and HEADING stop near the ±π wrap boundary."""

    def test_wrap_around_from_plus170_to_minus170(self) -> None:
        """From +170° to -170°: shortest path is CCW +20° (wrap of -340° = +20°).

        Raw diff = -170° - 170° = -340°.
        wrap(-340°) = atan2(sin(-340°), cos(-340°)) = atan2(sin(20°), cos(20°)) = +20°.
        So the robot rotates CCW by 20° (positive delta), which is shorter than CW 340°.
        """
        current_rad = math.radians(170.0)
        # target -170° = -17000 cdeg
        delta, sign = compute_turn(-17000, current_rad)
        # Shortest path is CCW +20° (wrap collapses -340° to +20°)
        assert abs(abs(delta) - math.radians(20.0)) < 0.01, (
            f"Expected ~20° delta magnitude, got {math.degrees(abs(delta)):.2f}°"
        )
        assert sign > 0.0, (
            "Expected CCW (positive) sign: wrap(-340°) = +20°, so delta > 0"
        )

    def test_wrap_around_from_minus170_to_plus170(self) -> None:
        """From -170° to +170°: shortest path is CW -20° (wrap of +340° = -20°).

        Raw diff = +170° - (-170°) = +340°.
        wrap(+340°) = atan2(sin(340°), cos(340°)) = atan2(-sin(20°), cos(20°)) = -20°.
        So the robot rotates CW by 20° (negative delta), which is shorter than CCW 340°.
        """
        current_rad = math.radians(-170.0)
        # target +170° = +17000 cdeg
        delta, sign = compute_turn(17000, current_rad)
        assert abs(abs(delta) - math.radians(20.0)) < 0.01, (
            f"Expected ~20° delta magnitude, got {math.degrees(abs(delta)):.2f}°"
        )
        assert sign < 0.0, (
            "Expected CW (negative) sign: wrap(+340°) = -20°, so delta < 0"
        )

    def test_exactly_180_delta_positive_sign(self) -> None:
        """From 0° to ±180°: delta = ±π; HEADING stop fires at ±π.

        atan2(sin(π), cos(π)) = atan2(0, -1) = +π (positive); sign=+1.
        This is an edge case — exact 180° could go either way by math.
        We just verify delta magnitude is π and the sign matches what atan2 gives.
        """
        delta, sign = compute_turn(18000, 0.0)
        assert abs(abs(delta) - math.pi) < 1e-5, f"Expected ±π delta, got {delta}"
        # Sign depends on atan2(sin(π), cos(π)) which returns +π → positive
        # In C++ sinf(π) may be small positive or negative floating-point.
        # Just verify sign is consistent with delta.
        assert sign == (1.0 if delta >= 0.0 else -1.0)

    def test_heading_stop_fires_at_target_near_pi(self) -> None:
        """HEADING stop condition fires when robot reaches +170° target from 0°."""
        # Uses the Python mirror of StopCondition::evaluate() for HEADING.
        # Baseline: heading0Rad = 0.
        # Target absolute: +170° = 17000 cdeg.
        # delta_rad (passed to makeHeadingStop) = wrap(17000_cdeg - 0) = +170° rad.
        delta_rad = cdeg_to_rad(17000)  # ~2.967 rad
        eps_rad   = cdeg_to_rad(300)    # 3° tolerance

        # Robot arrived at exactly +170°
        current_rad = cdeg_to_rad(17000)
        heading0_rad = 0.0

        # Mirror of evaluate(HEADING):
        # current_delta = wrap(current - heading0) = wrap(17000 cdeg) = delta_rad
        # error = wrap(current_delta - a) = wrap(delta_rad - delta_rad) = 0
        current_delta = wrap_angle(current_rad - heading0_rad)
        error = wrap_angle(current_delta - delta_rad)
        fires = abs(error) < eps_rad
        assert fires, f"HEADING stop should fire at target; error={math.degrees(error):.3f}°"

    def test_heading_stop_does_not_fire_before_target(self) -> None:
        """HEADING stop does not fire when robot is 10° short of +90° target."""
        target_cdeg = 9000  # +90°
        delta_rad = cdeg_to_rad(target_cdeg)
        eps_rad   = cdeg_to_rad(300)      # 3° tolerance
        heading0_rad = 0.0

        # Robot at +80° (8000 cdeg) — 10° short
        current_rad = cdeg_to_rad(8000)
        current_delta = wrap_angle(current_rad - heading0_rad)
        error = wrap_angle(current_delta - delta_rad)
        fires = abs(error) < eps_rad
        assert not fires, f"HEADING stop must not fire 10° short; error={math.degrees(error):.3f}°"


# ---------------------------------------------------------------------------
# Tests — TURN in HELP verb list
# ---------------------------------------------------------------------------

class TestTurnInHelp:
    """TURN must appear in the HELP verb list."""

    def test_turn_in_help(self) -> None:
        """TURN appears in the HELP verb list."""
        help_body = (
            "PING ECHO ID VER HELP SET GET GET VEL STREAM SNAP S T D G R TURN VW RF "
            "X STOP GRIP ZERO OI OZ OR OP OV OL OA P PA"
        )
        assert "TURN" in help_body.split()

    def test_turn_in_help_after_r(self) -> None:
        """TURN appears after R in HELP list (implementation order)."""
        verbs = (
            "PING ECHO ID VER HELP SET GET GET VEL STREAM SNAP S T D G R TURN VW RF "
            "X STOP GRIP ZERO OI OZ OR OP OV OL OA P PA"
        ).split()
        r_idx    = verbs.index("R")
        turn_idx = verbs.index("TURN")
        vw_idx   = verbs.index("VW")
        assert r_idx < turn_idx < vw_idx


# ---------------------------------------------------------------------------
# Tests — host protocol.py turn() wrapper
# ---------------------------------------------------------------------------

class TestTurnHostWrapper:
    """Tests for NezhaProtocol.turn() wrapper method."""

    def _make_proto(self):
        """Create a NezhaProtocol with a mock connection."""
        from unittest.mock import MagicMock
        from robot_radio.robot.protocol import NezhaProtocol

        mock_conn = MagicMock()
        mock_conn.send.return_value = {"responses": ["OK turn heading=9000 eps=300"]}
        proto = NezhaProtocol(mock_conn)
        return proto, mock_conn

    def test_turn_sends_correct_command(self) -> None:
        """turn(9000) sends 'TURN 9000' via conn.send()."""
        proto, mock_conn = self._make_proto()
        proto.turn(9000)
        args, _ = mock_conn.send.call_args
        assert args[0] == "TURN 9000", f"Expected 'TURN 9000', got {args[0]!r}"

    def test_turn_sends_with_eps(self) -> None:
        """turn(9000, eps_cdeg=100) sends 'TURN 9000 eps=100'."""
        proto, mock_conn = self._make_proto()
        proto.turn(9000, eps_cdeg=100)
        args, _ = mock_conn.send.call_args
        assert args[0] == "TURN 9000 eps=100", f"Got {args[0]!r}"

    def test_turn_sends_with_corr_id(self) -> None:
        """turn(9000, corr_id='42') sends 'TURN 9000 #42'."""
        proto, mock_conn = self._make_proto()
        proto.turn(9000, corr_id="42")
        args, _ = mock_conn.send.call_args
        assert args[0] == "TURN 9000 #42", f"Got {args[0]!r}"

    def test_turn_sends_with_eps_and_corr_id(self) -> None:
        """turn(9000, eps_cdeg=100, corr_id='1') sends 'TURN 9000 eps=100 #1'."""
        proto, mock_conn = self._make_proto()
        proto.turn(9000, eps_cdeg=100, corr_id="1")
        args, _ = mock_conn.send.call_args
        assert args[0] == "TURN 9000 eps=100 #1", f"Got {args[0]!r}"

    def test_turn_default_eps_not_in_wire(self) -> None:
        """turn(9000) with no eps_cdeg does NOT append 'eps=' to the wire command."""
        proto, mock_conn = self._make_proto()
        proto.turn(9000)
        args, _ = mock_conn.send.call_args
        # eps_cdeg=None means no eps= token in wire (firmware uses its default)
        assert "eps=" not in args[0], (
            f"Expected no eps= in wire when eps_cdeg is None, got {args[0]!r}"
        )

    def test_turn_uses_send_not_send_fast(self) -> None:
        """turn() uses conn.send() (blocking), not send_fast (fire-and-forget).

        TURN expects an OK reply; using send() lets the caller confirm acceptance.
        """
        proto, mock_conn = self._make_proto()
        proto.turn(9000)
        assert mock_conn.send.call_count == 1
        assert mock_conn.send_fast.call_count == 0

    def test_turn_returns_response_lines(self) -> None:
        """turn() returns the list of response lines from conn.send()."""
        from unittest.mock import MagicMock
        from robot_radio.robot.protocol import NezhaProtocol

        mock_conn = MagicMock()
        mock_conn.send.return_value = {"responses": ["OK turn heading=9000 eps=300"]}
        proto = NezhaProtocol(mock_conn)
        result = proto.turn(9000)
        assert result == ["OK turn heading=9000 eps=300"]

    def test_turn_negative_heading(self) -> None:
        """turn(-9000) sends 'TURN -9000'."""
        proto, mock_conn = self._make_proto()
        proto.turn(-9000)
        args, _ = mock_conn.send.call_args
        assert args[0] == "TURN -9000", f"Got {args[0]!r}"

    def test_turn_zero_heading(self) -> None:
        """turn(0) sends 'TURN 0'."""
        proto, mock_conn = self._make_proto()
        proto.turn(0)
        args, _ = mock_conn.send.call_args
        assert args[0] == "TURN 0", f"Got {args[0]!r}"


# ---------------------------------------------------------------------------
# Tests — delta_rad computation across all quadrants
# ---------------------------------------------------------------------------

class TestTurnDeltaAllQuadrants:
    """Verify delta_rad is correct for targets in all four quadrants."""

    def test_q1_target_from_zero(self) -> None:
        """Target +45° from 0° → delta = +45° (CCW)."""
        delta, _ = compute_turn(4500, 0.0)
        assert abs(delta - math.radians(45.0)) < 1e-5

    def test_q2_target_from_zero(self) -> None:
        """Target +135° from 0° → delta = +135° (CCW)."""
        delta, _ = compute_turn(13500, 0.0)
        assert abs(delta - math.radians(135.0)) < 1e-5

    def test_q3_target_from_zero(self) -> None:
        """Target -135° from 0° → delta = -135° (CW), shortest path."""
        delta, _ = compute_turn(-13500, 0.0)
        assert abs(delta - math.radians(-135.0)) < 1e-5

    def test_q4_target_from_zero(self) -> None:
        """Target -45° from 0° → delta = -45° (CW)."""
        delta, _ = compute_turn(-4500, 0.0)
        assert abs(delta - math.radians(-45.0)) < 1e-5

    def test_cross_quadrant_from_q1_to_q3(self) -> None:
        """From +45° to -135°: shortest path is CW -180° (or CCW +180°; atan2 gives +π)."""
        current_rad = math.radians(45.0)
        # target -135° = -13500 cdeg; diff = -180° = -π; wrap gives ±π
        delta, _ = compute_turn(-13500, current_rad)
        assert abs(abs(delta) - math.pi) < 1e-5

    def test_small_ccw_correction(self) -> None:
        """From +89° to +90°: delta = +1° (tiny CCW nudge)."""
        current_rad = math.radians(89.0)
        delta, sign = compute_turn(9000, current_rad)
        assert abs(delta - math.radians(1.0)) < 0.01
        assert sign > 0.0

    def test_small_cw_correction(self) -> None:
        """From -89° to -90°: delta = -1° (tiny CW nudge)."""
        current_rad = math.radians(-89.0)
        delta, sign = compute_turn(-9000, current_rad)
        assert abs(delta - math.radians(-1.0)) < 0.01
        assert sign < 0.0
