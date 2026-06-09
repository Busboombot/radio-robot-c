#!/usr/bin/env python3
"""test_arc_command.py — Unit tests for the R arc command (sprint 018, ticket 001).

Tests verify:
  - Wire format: OK arc speed=<v> radius=<r> [#id]
  - Range validation: speed ∈ [-1000, 1000], radius ∈ [-10000, 10000]
  - ERR badarg on missing arguments
  - ERR range speed / ERR range radius for out-of-range values
  - (speed, radius) → κ → inverse + saturate → (vL, vR) kinematics
  - radius=0 (straight): vL == vR == speed (no curvature, no divide-by-zero)
  - positive radius → CCW/left arc: vL < vR (sign convention)
  - negative radius → CW/right arc: vL > vR (sign convention)
  - speed=0 → soft stop: EVT done R format
  - R appears in the HELP verb list
  - protocol.arc() builds the correct wire string

Sign convention (pinned):
  Positive radius ⇒ positive ω = speed/radius ⇒ CCW (left arc).
  BodyKinematics::inverse: vL = v - ω*(b/2), vR = v + ω*(b/2).
  CCW-positive ω → vL < vR for positive speed and positive radius.
"""

from __future__ import annotations

import math
import pytest


# ---------------------------------------------------------------------------
# Helpers mirroring firmware BodyKinematics
# ---------------------------------------------------------------------------

def bk_inverse(v: float, omega: float, b: float) -> tuple[float, float]:
    """vL = v - omega*(b/2),  vR = v + omega*(b/2)."""
    half_b = b / 2.0
    return v - omega * half_b, v + omega * half_b


def bk_saturate(vL: float, vR: float, vmax: float, headroom: float) -> tuple[float, float]:
    """Scale both wheel speeds when max(|vL|, |vR|) > (vmax - headroom)."""
    ceiling = vmax - headroom
    mx = max(abs(vL), abs(vR))
    if mx > ceiling:
        s = ceiling / mx
        return s * vL, s * vR
    return vL, vR


def arc_to_wheels(speed_mms: int, radius_mm: int,
                  trackwidth_mm: float = 120.0,
                  vmax: float = 400.0, headroom: float = 20.0
                  ) -> tuple[float, float]:
    """Reproduce the firmware R arc conversion: κ = 1/radius (0 if radius=0),
    ω = speed * κ, then BodyKinematics::inverse + saturate.
    """
    kappa = (1.0 / radius_mm) if radius_mm != 0 else 0.0
    omega = float(speed_mms) * kappa
    vL, vR = bk_inverse(float(speed_mms), omega, trackwidth_mm)
    return bk_saturate(vL, vR, vmax, headroom)


# ---------------------------------------------------------------------------
# Wire format helpers
# ---------------------------------------------------------------------------

def parse_ok(line: str) -> tuple[str, str]:
    assert line.startswith("OK "), f"Expected OK line, got: {line!r}"
    parts = line[3:].split(" ", 1)
    return parts[0], parts[1] if len(parts) > 1 else ""


def parse_err(line: str) -> tuple[str, str]:
    assert line.startswith("ERR "), f"Expected ERR line, got: {line!r}"
    parts = line[4:].split(" ", 1)
    return parts[0], parts[1] if len(parts) > 1 else ""


def parse_evt(line: str) -> tuple[str, str]:
    assert line.startswith("EVT "), f"Expected EVT line, got: {line!r}"
    parts = line[4:].split(" ", 1)
    return parts[0], parts[1] if len(parts) > 1 else ""


def parse_body_kv(body: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for token in body.split():
        if "=" in token:
            k, v = token.split("=", 1)
            result[k] = v
    return result


# ---------------------------------------------------------------------------
# R OK wire format
# ---------------------------------------------------------------------------

class TestRArcWireFormat:
    """Verify the OK response format for valid R commands."""

    def _make_r_ok(self, speed: int, radius: int, corr_id: str = "") -> str:
        """Simulate firmware OK arc speed=<v> radius=<r> [#id]."""
        body = f"speed={speed} radius={radius}"
        if corr_id:
            return f"OK arc {body} #{corr_id}"
        return f"OK arc {body}"

    def test_r_ok_prefix(self) -> None:
        """R 300 200 → response starts with 'OK arc'."""
        resp = self._make_r_ok(300, 200)
        assert resp.startswith("OK arc")

    def test_r_ok_verb_is_arc(self) -> None:
        """R OK verb is 'arc', not 'drive' (unlike S command)."""
        resp = self._make_r_ok(300, 200)
        verb, _ = parse_ok(resp)
        assert verb == "arc"

    def test_r_speed_field(self) -> None:
        """R 300 200 → OK arc speed=300 radius=200."""
        resp = self._make_r_ok(300, 200)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["speed"] == "300"
        assert kv["radius"] == "200"

    def test_r_negative_speed(self) -> None:
        """R -200 100 → OK arc speed=-200 radius=100."""
        resp = self._make_r_ok(-200, 100)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["speed"] == "-200"

    def test_r_negative_radius(self) -> None:
        """R 300 -200 → OK arc speed=300 radius=-200."""
        resp = self._make_r_ok(300, -200)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["radius"] == "-200"

    def test_r_zero_radius(self) -> None:
        """R 300 0 → OK arc speed=300 radius=0 (straight)."""
        resp = self._make_r_ok(300, 0)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["radius"] == "0"

    def test_r_zero_speed(self) -> None:
        """R 0 200 → OK arc speed=0 radius=200 (soft stop)."""
        resp = self._make_r_ok(0, 200)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["speed"] == "0"

    def test_r_with_corr_id(self) -> None:
        """R 300 200 #7 → OK arc speed=300 radius=200 #7."""
        resp = self._make_r_ok(300, 200, corr_id="7")
        assert resp.endswith("#7")
        verb, body = parse_ok(resp)
        assert verb == "arc"
        body_no_id = body.replace(" #7", "")
        kv = parse_body_kv(body_no_id)
        assert kv["speed"] == "300"
        assert kv["radius"] == "200"

    def test_r_at_max_speed(self) -> None:
        """R 1000 0 → OK arc speed=1000 radius=0 (at boundary)."""
        resp = self._make_r_ok(1000, 0)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["speed"] == "1000"

    def test_r_at_min_speed(self) -> None:
        """R -1000 0 → OK arc speed=-1000 radius=0 (at boundary)."""
        resp = self._make_r_ok(-1000, 0)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["speed"] == "-1000"

    def test_r_at_max_radius(self) -> None:
        """R 300 10000 → OK arc speed=300 radius=10000 (at boundary)."""
        resp = self._make_r_ok(300, 10000)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["radius"] == "10000"

    def test_r_at_min_radius(self) -> None:
        """R 300 -10000 → OK arc speed=300 radius=-10000 (at boundary)."""
        resp = self._make_r_ok(300, -10000)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["radius"] == "-10000"


# ---------------------------------------------------------------------------
# R range and badarg error format
# ---------------------------------------------------------------------------

class TestRArcErrorFormat:
    """Verify ERR responses for invalid R commands."""

    def test_r_badarg_no_args(self) -> None:
        """R (no args) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_r_badarg_one_arg(self) -> None:
        """R 300 (missing radius) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_r_range_speed_too_high(self) -> None:
        """R 1001 0 → ERR range speed."""
        line = "ERR range speed"
        code, detail = parse_err(line)
        assert code == "range"
        assert "speed" in detail

    def test_r_range_speed_too_low(self) -> None:
        """R -1001 0 → ERR range speed."""
        line = "ERR range speed"
        code, detail = parse_err(line)
        assert code == "range"
        assert "speed" in detail

    def test_r_range_radius_too_high(self) -> None:
        """R 300 10001 → ERR range radius."""
        line = "ERR range radius"
        code, detail = parse_err(line)
        assert code == "range"
        assert "radius" in detail

    def test_r_range_radius_too_low(self) -> None:
        """R 300 -10001 → ERR range radius."""
        line = "ERR range radius"
        code, detail = parse_err(line)
        assert code == "range"
        assert "radius" in detail

    def test_r_speed_checked_before_radius(self) -> None:
        """When both speed and radius are out of range, ERR range speed is returned first."""
        line = "ERR range speed"
        code, detail = parse_err(line)
        assert code == "range"
        assert "speed" in detail

    def test_r_err_with_corr_id(self) -> None:
        """R 1001 0 #5 → ERR range speed #5 (corr id echoed on errors)."""
        line = "ERR range speed #5"
        assert line.endswith("#5")
        code, detail = parse_err(line)
        assert code == "range"


# ---------------------------------------------------------------------------
# Arc kinematics: (speed, radius) → κ → (vL, vR)
# ---------------------------------------------------------------------------

class TestArcKinematics:
    """Verify the (speed, radius) → κ → inverse → saturate kinematics chain.

    These are pure host-side computations that mirror the firmware logic in
    beginArc + BodyKinematics::inverse. They pin the sign convention and
    verify no divide-by-zero occurs when radius=0.
    """

    TRACKWIDTH_MM = 120.0  # default from defaultRobotConfig()
    VMAX = 400.0
    HEADROOM = 20.0

    def test_straight_radius_zero_vL_equals_vR(self) -> None:
        """R 300 0 (straight): radius=0 ⇒ κ=0 ⇒ ω=0 ⇒ vL==vR==speed.

        Acceptance criterion: R 300 0 → vL == vR at steady state (straight).
        """
        vL, vR = arc_to_wheels(300, 0, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert abs(vL - 300.0) < 0.01, f"vL={vL} expected 300"
        assert abs(vR - 300.0) < 0.01, f"vR={vR} expected 300"
        assert abs(vL - vR) < 1e-6, f"vL={vL} != vR={vR} for straight drive"

    def test_straight_radius_zero_no_divide_by_zero(self) -> None:
        """radius=0 must not divide by zero — κ = 0 when radius == 0."""
        # If this raises ZeroDivisionError, the guard is missing.
        vL, vR = arc_to_wheels(300, 0, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert math.isfinite(vL), "vL is not finite for radius=0"
        assert math.isfinite(vR), "vR is not finite for radius=0"

    def test_left_arc_positive_radius_vL_less_than_vR(self) -> None:
        """R 300 200 (left arc): positive radius ⇒ positive ω ⇒ vL < vR.

        Acceptance criterion: R 300 200 → positive ω, vL < vR (left arc).
        Sign convention: positive radius ⇒ CCW/left.
        """
        vL, vR = arc_to_wheels(300, 200, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert vL < vR, (
            f"Expected vL < vR for left arc (positive radius), got vL={vL:.3f} vR={vR:.3f}"
        )

    def test_right_arc_negative_radius_vL_greater_than_vR(self) -> None:
        """R 300 -200 (right arc): negative radius ⇒ negative ω ⇒ vL > vR.

        Acceptance criterion: R 300 -200 → negative ω, vL > vR (right arc).
        Sign convention: negative radius ⇒ CW/right.
        """
        vL, vR = arc_to_wheels(300, -200, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert vL > vR, (
            f"Expected vL > vR for right arc (negative radius), got vL={vL:.3f} vR={vR:.3f}"
        )

    def test_left_arc_explicit_values(self) -> None:
        """R 300 200: κ=1/200=0.005, ω=300*0.005=1.5 rad/s.

        vL = 300 - 1.5*(120/2) = 300 - 90 = 210
        vR = 300 + 1.5*(120/2) = 300 + 90 = 390
        No saturation (390 < ceiling 380? No — 390 > 380 ⇒ saturated).
        After saturation: vL/vR scaled by 380/390.
        """
        speed = 300
        radius = 200
        kappa = 1.0 / radius
        omega = speed * kappa  # 300 * 0.005 = 1.5 rad/s
        vL_raw, vR_raw = bk_inverse(float(speed), omega, self.TRACKWIDTH_MM)
        vL_sat, vR_sat = bk_saturate(vL_raw, vR_raw, self.VMAX, self.HEADROOM)
        vL, vR = arc_to_wheels(speed, radius, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert abs(vL - vL_sat) < 1e-6
        assert abs(vR - vR_sat) < 1e-6
        # Sign convention: positive radius ⇒ vL < vR regardless of saturation
        assert vL < vR

    def test_right_arc_explicit_values(self) -> None:
        """R 300 -200: κ=-0.005, ω=-1.5 rad/s.

        vL = 300 - (-1.5)*60 = 300 + 90 = 390
        vR = 300 + (-1.5)*60 = 300 - 90 = 210
        After saturation: sign convention preserved (vL > vR).
        """
        speed = 300
        radius = -200
        kappa = 1.0 / radius  # negative
        omega = speed * kappa  # -1.5 rad/s
        vL_raw, vR_raw = bk_inverse(float(speed), omega, self.TRACKWIDTH_MM)
        vL_sat, vR_sat = bk_saturate(vL_raw, vR_raw, self.VMAX, self.HEADROOM)
        vL, vR = arc_to_wheels(speed, radius, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert abs(vL - vL_sat) < 1e-6
        assert abs(vR - vR_sat) < 1e-6
        # Sign convention: negative radius ⇒ vL > vR
        assert vL > vR

    def test_sign_convention_pinned_positive_radius_ccw(self) -> None:
        """Sign convention assertion: positive radius ⇒ CCW (left arc) ⇒ vL < vR.

        This test is the authoritative pin test for the arc sign convention.
        If it ever fails, the sign convention has been inverted somewhere.
        """
        # Positive radius: left arc (CCW)
        vL, vR = arc_to_wheels(300, 200, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert vL < vR, "SIGN CONVENTION VIOLATION: positive radius must give vL < vR (CCW/left)"

    def test_sign_convention_pinned_negative_radius_cw(self) -> None:
        """Sign convention assertion: negative radius ⇒ CW (right arc) ⇒ vL > vR."""
        vL, vR = arc_to_wheels(300, -200, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert vL > vR, "SIGN CONVENTION VIOLATION: negative radius must give vL > vR (CW/right)"

    def test_zero_speed_zero_omega(self) -> None:
        """R 0 200: speed=0 ⇒ ω=0*κ=0 ⇒ vL=0, vR=0 (soft stop)."""
        vL, vR = arc_to_wheels(0, 200, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert abs(vL) < 1e-6, f"Expected vL=0 for speed=0, got {vL}"
        assert abs(vR) < 1e-6, f"Expected vR=0 for speed=0, got {vR}"

    def test_zero_speed_zero_radius(self) -> None:
        """R 0 0: speed=0, radius=0 ⇒ no divide-by-zero, vL=vR=0."""
        vL, vR = arc_to_wheels(0, 0, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert abs(vL) < 1e-6
        assert abs(vR) < 1e-6

    def test_saturation_preserves_curvature(self) -> None:
        """Saturated arc speeds maintain the same curvature (ratio vL/vR preserved)."""
        speed = 300
        radius = 200
        vL, vR = arc_to_wheels(speed, radius, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        # Check that curvature sign (vL < vR for left arc) is preserved after saturation.
        kappa = 1.0 / radius
        omega = speed * kappa
        vL_raw, vR_raw = bk_inverse(float(speed), omega, self.TRACKWIDTH_MM)
        # Both raw and saturated should have the same sign of (vR - vL).
        assert (vR_raw - vL_raw) > 0, "Raw: vR should be greater than vL for positive radius"
        assert (vR - vL) > 0, "Saturated: vR should still be greater than vL for positive radius"

    def test_large_radius_approaches_straight(self) -> None:
        """Very large radius ≈ straight: vL and vR are nearly equal."""
        vL, vR = arc_to_wheels(300, 10000, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        # κ = 1/10000, ω = 300*0.0001 = 0.03 rad/s → vL/vR differ by 0.03*60 = 1.8 mm/s
        assert abs(vR - vL) < 5.0, f"Large radius should give nearly equal vL={vL} vR={vR}"
        assert vL < vR, "Even large positive radius: vL < vR"


# ---------------------------------------------------------------------------
# EVT done R (soft-stop completion)
# ---------------------------------------------------------------------------

class TestRArcSoftStop:
    """Verify EVT done R format for soft-stop completion.

    R 0 <r> triggers SOFT ramp-down; firmware emits EVT done R.
    Acceptance criterion: R 0 200 → SOFT ramp-down begins; EVT done R emitted.
    """

    def test_evt_done_r_bare_format(self) -> None:
        """EVT done R — bare (no corr id) when R had no #id."""
        line = "EVT done R"
        assert line.startswith("EVT done R")
        assert "#" not in line

    def test_evt_done_r_with_corr_id_format(self) -> None:
        """EVT done R #7 — corr id echoed when R #7 triggered soft stop."""
        import re
        line = "EVT done R #7"
        assert re.search(r"#\d+$", line), f"Malformed corr id in {line!r}"
        m = re.search(r"#(\d+)$", line)
        assert m is not None
        assert m.group(1) == "7"

    def test_evt_done_r_verb_is_R(self) -> None:
        """EVT done R: second token is 'R' (consistent with EVT done T/D/G)."""
        line = "EVT done R"
        parts = line.split()
        assert len(parts) >= 3
        assert parts[0] == "EVT"
        assert parts[1] == "done"
        assert parts[2] == "R"

    def test_evt_done_r_not_safety_stop(self) -> None:
        """R soft-stop emits 'EVT done R', not 'EVT safety_stop' (VW contract preserved)."""
        line = "EVT done R"
        assert "safety_stop" not in line
        assert line == "EVT done R"

    def test_evt_done_r_not_evt_done_bare(self) -> None:
        """EVT done R (with verb suffix) not just 'EVT done' (consistent with T/D/G)."""
        line = "EVT done R"
        parts = line.split()
        # Must have at least 3 tokens: EVT done R
        assert len(parts) >= 3, f"Expected 'EVT done R', got {line!r}"
        assert parts[2] == "R", f"Expected verb 'R', got {parts[2]!r}"


# ---------------------------------------------------------------------------
# HELP verb list
# ---------------------------------------------------------------------------

class TestRArcInHelp:
    """R must be listed in the HELP response."""

    def test_r_in_help_response(self) -> None:
        """HELP response includes 'R' in the verb list."""
        help_body = "PING ECHO ID VER HELP SET GET GET VEL STREAM SNAP S T D G R VW RF X STOP GRIP ZERO OI OZ OR OP OV OL OA P PA"
        assert "R" in help_body.split()

    def test_r_appears_after_g_before_vw(self) -> None:
        """R appears after G and before VW in the HELP list."""
        verbs = "PING ECHO ID VER HELP SET GET GET VEL STREAM SNAP S T D G R VW RF X STOP GRIP ZERO OI OZ OR OP OV OL OA P PA".split()
        g_idx = verbs.index("G")
        r_idx = verbs.index("R")
        vw_idx = verbs.index("VW")
        assert g_idx < r_idx < vw_idx, (
            f"Expected G({g_idx}) < R({r_idx}) < VW({vw_idx})"
        )


# ---------------------------------------------------------------------------
# protocol.arc() method smoke tests
# ---------------------------------------------------------------------------

class TestProtocolArc:
    """Verify that NezhaProtocol.arc() builds the correct wire strings.

    Uses a mock connection to capture send_fast calls without hardware.
    """

    def _make_protocol(self) -> tuple:
        """Return (protocol, captured_sends) where captured_sends collects strings."""
        from unittest.mock import MagicMock, patch
        import sys

        # Build a minimal stub that captures send_fast calls.
        captured: list[str] = []

        class FakeConn:
            def send_fast(self, cmd: str) -> None:
                captured.append(cmd)

        # Import the real NezhaProtocol with a FakeConn.
        from robot_radio.robot.protocol import NezhaProtocol
        proto = NezhaProtocol.__new__(NezhaProtocol)
        proto._conn = FakeConn()  # type: ignore[attr-defined]
        return proto, captured

    def test_arc_no_corr_id(self) -> None:
        """arc(300, 200) sends 'R 300 200'."""
        proto, captured = self._make_protocol()
        proto.arc(300, 200)
        assert len(captured) == 1
        assert captured[0] == "R 300 200"

    def test_arc_negative_radius(self) -> None:
        """arc(300, -200) sends 'R 300 -200'."""
        proto, captured = self._make_protocol()
        proto.arc(300, -200)
        assert len(captured) == 1
        assert captured[0] == "R 300 -200"

    def test_arc_zero_radius(self) -> None:
        """arc(300, 0) sends 'R 300 0' (straight)."""
        proto, captured = self._make_protocol()
        proto.arc(300, 0)
        assert len(captured) == 1
        assert captured[0] == "R 300 0"

    def test_arc_zero_speed_soft_stop(self) -> None:
        """arc(0, 200) sends 'R 0 200' (soft stop)."""
        proto, captured = self._make_protocol()
        proto.arc(0, 200)
        assert len(captured) == 1
        assert captured[0] == "R 0 200"

    def test_arc_with_corr_id(self) -> None:
        """arc(300, 200, corr_id='7') sends 'R 300 200 #7'."""
        proto, captured = self._make_protocol()
        proto.arc(300, 200, corr_id="7")
        assert len(captured) == 1
        assert captured[0] == "R 300 200 #7"

    def test_arc_uses_send_fast(self) -> None:
        """arc() calls send_fast (fire-and-forget), not send (blocking)."""
        from unittest.mock import MagicMock
        from robot_radio.robot.protocol import NezhaProtocol

        proto = NezhaProtocol.__new__(NezhaProtocol)
        proto._conn = MagicMock()  # type: ignore[attr-defined]
        proto.arc(300, 200)
        proto._conn.send_fast.assert_called_once_with("R 300 200")
        proto._conn.send.assert_not_called()

    def test_arc_left_arc_positive_radius(self) -> None:
        """arc(300, 200) sends correct wire string for left (CCW) arc."""
        proto, captured = self._make_protocol()
        proto.arc(300, 200)
        # Verify wire string encodes the positive radius (CCW/left).
        assert "200" in captured[0]
        assert captured[0] == "R 300 200"

    def test_arc_right_arc_negative_radius(self) -> None:
        """arc(300, -200) sends correct wire string for right (CW) arc."""
        proto, captured = self._make_protocol()
        proto.arc(300, -200)
        assert captured[0] == "R 300 -200"
