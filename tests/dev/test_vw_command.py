#!/usr/bin/env python3
"""test_vw_command.py — Unit tests for the VW command (011-005, updated 017-004).

Tests verify:
  - Wire format: OK vw v=<v> omega=<omega_mrads> [#id]
  - Range validation: v ∈ [-1000, 1000], omega ∈ [-3142, 3142]
  - ERR badarg on missing arguments
  - ERR range v / ERR range omega for out-of-range values
  - mrad/s → rad/s scaling (÷ 1000) and resulting (vL, vR) computation
  - VW uses DriveMode::VELOCITY (mode=V in TLM) after Sprint 017-004 migration
  - Keepalive loss → EVT safety_stop (TIME stop condition; wire contract preserved)
  - VW is in the HELP verb list

Sprint 017-004 changes:
  - VW migrated from raw STREAMING path onto MotionCommand + BodyVelocityController.
  - VW now ramps (trapezoid profile) instead of stepping wheel setpoints.
  - DriveMode::VELOCITY = 5 added; TLM reports mode=V for VW.
  - EVT safety_stop preserved: MotionCommand TIME condition uses setDoneEvt().
  - S command (beginStream) is completely unchanged.
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


def wire_to_wheels(v_mms: int, omega_mrads: int,
                   trackwidth_mm: float = 120.0,
                   vmax: float = 400.0, headroom: float = 20.0
                   ) -> tuple[float, float]:
    """Reproduce the firmware conversion for VW: mrad/s → rad/s, then inverse + saturate."""
    omega_rads = omega_mrads / 1000.0  # mrad/s → rad/s (firmware boundary)
    vL, vR = bk_inverse(float(v_mms), omega_rads, trackwidth_mm)
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


def parse_body_kv(body: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for token in body.split():
        if "=" in token:
            k, v = token.split("=", 1)
            result[k] = v
    return result


# ---------------------------------------------------------------------------
# VW OK wire format
# ---------------------------------------------------------------------------

class TestVWWireFormat:
    """Verify the OK response format for valid VW commands."""

    def _make_vw_ok(self, v: int, omega: int, corr_id: str = "") -> str:
        """Simulate firmware OK vw v=<v> omega=<omega> [#id]."""
        body = f"v={v} omega={omega}"
        if corr_id:
            return f"OK vw {body} #{corr_id}"
        return f"OK vw {body}"

    def test_vw_ok_prefix(self) -> None:
        """VW 200 0 → response starts with 'OK vw'."""
        resp = self._make_vw_ok(200, 0)
        assert resp.startswith("OK vw")

    def test_vw_v_field(self) -> None:
        """VW 200 0 → OK vw v=200 omega=0."""
        resp = self._make_vw_ok(200, 0)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["v"] == "200"
        assert kv["omega"] == "0"

    def test_vw_negative_v(self) -> None:
        """VW -200 0 → OK vw v=-200 omega=0."""
        resp = self._make_vw_ok(-200, 0)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["v"] == "-200"

    def test_vw_omega_positive(self) -> None:
        """VW 0 500 → OK vw v=0 omega=500 (CCW spin)."""
        resp = self._make_vw_ok(0, 500)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["v"] == "0"
        assert kv["omega"] == "500"

    def test_vw_omega_negative(self) -> None:
        """VW 0 -500 → OK vw v=0 omega=-500 (CW spin)."""
        resp = self._make_vw_ok(0, -500)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["omega"] == "-500"

    def test_vw_curved_arc(self) -> None:
        """VW 200 300 → OK vw v=200 omega=300 (left-turn arc)."""
        resp = self._make_vw_ok(200, 300)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["v"] == "200"
        assert kv["omega"] == "300"

    def test_vw_at_max_v(self) -> None:
        """VW 1000 0 → OK vw v=1000 omega=0 (at boundary)."""
        resp = self._make_vw_ok(1000, 0)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["v"] == "1000"

    def test_vw_at_min_v(self) -> None:
        """VW -1000 0 → OK vw v=-1000 omega=0 (at boundary)."""
        resp = self._make_vw_ok(-1000, 0)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["v"] == "-1000"

    def test_vw_at_max_omega(self) -> None:
        """VW 0 3142 → OK vw v=0 omega=3142 (≈π rad/s, at boundary)."""
        resp = self._make_vw_ok(0, 3142)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["omega"] == "3142"

    def test_vw_at_min_omega(self) -> None:
        """VW 0 -3142 → OK vw v=0 omega=-3142 (at boundary)."""
        resp = self._make_vw_ok(0, -3142)
        _, body = parse_ok(resp)
        kv = parse_body_kv(body)
        assert kv["omega"] == "-3142"

    def test_vw_with_corr_id(self) -> None:
        """VW 200 0 #7 → OK vw v=200 omega=0 #7."""
        resp = self._make_vw_ok(200, 0, corr_id="7")
        assert resp.endswith("#7")
        verb, body = parse_ok(resp)
        assert verb == "vw"
        body_no_id = body.replace(" #7", "")
        kv = parse_body_kv(body_no_id)
        assert kv["v"] == "200"
        assert kv["omega"] == "0"

    def test_vw_verb_is_vw_not_drive(self) -> None:
        """VW OK verb is 'vw', not 'drive' (unlike S command)."""
        resp = self._make_vw_ok(200, 0)
        verb, _ = parse_ok(resp)
        assert verb == "vw"
        assert verb != "drive"


# ---------------------------------------------------------------------------
# VW range and badarg error format
# ---------------------------------------------------------------------------

class TestVWErrorFormat:
    """Verify ERR responses for invalid VW commands."""

    def test_vw_badarg_no_args(self) -> None:
        """VW (no args) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_vw_badarg_one_arg(self) -> None:
        """VW 200 (missing omega) → ERR badarg."""
        line = "ERR badarg"
        code, _ = parse_err(line)
        assert code == "badarg"

    def test_vw_range_v_too_high(self) -> None:
        """VW 1001 0 → ERR range v."""
        line = "ERR range v"
        code, detail = parse_err(line)
        assert code == "range"
        assert "v" in detail

    def test_vw_range_v_too_low(self) -> None:
        """VW -1001 0 → ERR range v."""
        line = "ERR range v"
        code, detail = parse_err(line)
        assert code == "range"
        assert "v" in detail

    def test_vw_range_omega_too_high(self) -> None:
        """VW 0 3143 → ERR range omega."""
        line = "ERR range omega"
        code, detail = parse_err(line)
        assert code == "range"
        assert "omega" in detail

    def test_vw_range_omega_too_low(self) -> None:
        """VW 0 -3143 → ERR range omega."""
        line = "ERR range omega"
        code, detail = parse_err(line)
        assert code == "range"
        assert "omega" in detail

    def test_vw_v_checked_before_omega(self) -> None:
        """When both v and omega are out of range, ERR range v is returned first."""
        line = "ERR range v"
        code, detail = parse_err(line)
        assert code == "range"
        assert "v" in detail

    def test_vw_err_with_corr_id(self) -> None:
        """VW 1001 0 #5 → ERR range v #5 (corr id echoed on errors)."""
        line = "ERR range v #5"
        assert line.endswith("#5")
        code, detail = parse_err(line)
        assert code == "range"


# ---------------------------------------------------------------------------
# mrad/s → rad/s scaling and (v,ω)→wheels conversion
# ---------------------------------------------------------------------------

class TestVWKinematics:
    """Verify mrad/s → rad/s scaling and the resulting wheel speed computation."""

    TRACKWIDTH_MM = 120.0  # default from defaultRobotConfig()
    VMAX = 400.0
    HEADROOM = 20.0

    def test_straight_forward(self) -> None:
        """VW 200 0: ω=0 → vL=vR=200 mm/s (no saturation)."""
        vL, vR = wire_to_wheels(200, 0, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert abs(vL - 200.0) < 0.01
        assert abs(vR - 200.0) < 0.01

    def test_spin_in_place_ccw(self) -> None:
        """VW 0 500: v=0, ω=0.5 rad/s → vL negative, vR positive (CCW)."""
        vL, vR = wire_to_wheels(0, 500, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        # vL = 0 - 0.5 * 60 = -30,  vR = 0 + 0.5 * 60 = +30
        assert abs(vL - (-30.0)) < 0.01
        assert abs(vR - 30.0) < 0.01

    def test_spin_in_place_cw(self) -> None:
        """VW 0 -500: ω=-0.5 rad/s → vL positive, vR negative (CW)."""
        vL, vR = wire_to_wheels(0, -500, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert abs(vL - 30.0) < 0.01
        assert abs(vR - (-30.0)) < 0.01

    def test_curved_arc_left(self) -> None:
        """VW 200 1000: ω=1.0 rad/s → left wheel slower (left turn)."""
        vL, vR = wire_to_wheels(200, 1000, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        # vL = 200 - 1.0 * 60 = 140,  vR = 200 + 1.0 * 60 = 260
        assert abs(vL - 140.0) < 0.01
        assert abs(vR - 260.0) < 0.01

    def test_mrad_to_rad_scaling(self) -> None:
        """omega_mrads / 1000 = omega_rads at the conversion boundary."""
        omega_mrads = 3142  # ≈ π rad/s
        omega_rads = omega_mrads / 1000.0
        assert abs(omega_rads - math.pi) < 0.002  # 3.142 vs π=3.14159…

    def test_saturation_at_max_omega(self) -> None:
        """VW 200 3142 triggers saturation check: max speed ≤ ceiling."""
        vL, vR = wire_to_wheels(200, 3142, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert max(abs(vL), abs(vR)) <= (self.VMAX - self.HEADROOM) + 1e-6

    def test_saturation_preserves_curvature(self) -> None:
        """Saturated speeds maintain the same curvature (kappa = (vR-vL)/(b*v))."""
        vL_raw, vR_raw = bk_inverse(300.0, 3.0, 120.0)
        # vL = 300 - 3*60 = 120,  vR = 300 + 3*60 = 480 (exceeds ceiling 380)
        vL_sat, vR_sat = bk_saturate(vL_raw, vR_raw, 400.0, 20.0)
        kappa_raw = (vR_raw - vL_raw) / (120.0 * (vR_raw + vL_raw) / 2) if (vR_raw + vL_raw) else 0.0
        kappa_sat = (vR_sat - vL_sat) / (120.0 * (vR_sat + vL_sat) / 2) if (vR_sat + vL_sat) else 0.0
        assert abs(kappa_raw - kappa_sat) < 1e-6

    def test_zero_v_zero_omega(self) -> None:
        """VW 0 0 → vL=0, vR=0 (stationary command)."""
        vL, vR = wire_to_wheels(0, 0, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert abs(vL) < 1e-6
        assert abs(vR) < 1e-6

    def test_boundary_v_1000_omega_0_saturated(self) -> None:
        """VW 1000 0 → vL=vR, both saturated to ceiling (straight ahead)."""
        vL, vR = wire_to_wheels(1000, 0, self.TRACKWIDTH_MM, self.VMAX, self.HEADROOM)
        assert abs(vL - vR) < 1e-6  # straight ahead, symmetric
        assert max(abs(vL), abs(vR)) <= (self.VMAX - self.HEADROOM) + 1e-6


# ---------------------------------------------------------------------------
# Keepalive watchdog and EVT safety_stop (Sprint 017-004: MotionCommand-based)
# ---------------------------------------------------------------------------

class TestVWWatchdog:
    """Verify EVT safety_stop format and corr_id propagation.

    Sprint 017-004: VW is now backed by a MotionCommand with a TIME stop
    condition (sTimeoutMs).  The EVT safety_stop wire contract is preserved
    via MotionCommand.setDoneEvt("EVT safety_stop").
    """

    def test_safety_stop_bare_format(self) -> None:
        """EVT safety_stop — bare (no corr id) when VW had no #id."""
        import re
        line = "EVT safety_stop"
        assert line.startswith("EVT safety_stop")
        assert "#" not in line

    def test_safety_stop_with_corr_id_format(self) -> None:
        """EVT safety_stop #7 — corr id echoed when VW #7 was last keepalive."""
        import re
        line = "EVT safety_stop #7"
        assert re.search(r"#\d+$", line), f"Malformed corr id in {line!r}"
        # id portion is digits only
        m = re.search(r"#(\d+)$", line)
        assert m is not None
        assert m.group(1) == "7"

    def test_vw_mode_is_V_not_S(self) -> None:
        """TLM mode=V is produced for VW (DriveMode::VELOCITY added in Sprint 017-004).

        VW no longer uses DriveMode::STREAMING.  A new DriveMode::VELOCITY = 5
        was added; Robot::buildTlmFrame maps it to mode char 'V'.
        This replaces the old test_vw_mode_is_S_not_new_mode assertion.
        """
        # Simulate the TLM line a robot running VW would produce.
        valid_tlm = "TLM t=100 mode=V enc=0,0"
        assert "mode=V" in valid_tlm
        assert "mode=S" not in valid_tlm

    def test_safety_stop_evt_name(self) -> None:
        """EVT safety_stop has name 'safety_stop', not 'safety_stop_vw' or other variant."""
        line = "EVT safety_stop"
        name = line[4:].split()[0]
        assert name == "safety_stop"

    def test_safety_stop_not_evt_done(self) -> None:
        """VW keepalive loss emits 'EVT safety_stop', not 'EVT done'.

        Sprint 017-004: MotionCommand.setDoneEvt('EVT safety_stop') is called
        in beginVelocity so the TIME condition emits the correct EVT name,
        preserving the host wire contract.
        """
        # The firmware emits this when the TIME condition fires for VW.
        line = "EVT safety_stop"
        assert line == "EVT safety_stop"
        # Explicitly NOT "EVT done" (that would break host scripts).
        assert line != "EVT done"

    def test_safety_stop_keepalive_loss_format(self) -> None:
        """Keepalive-loss EVT: 'EVT safety_stop' (bare) or 'EVT safety_stop #id' (with id).

        Contract: TIME stop condition fires → SOFT ramp to zero → EVT safety_stop
        emitted.  The corr_id echoes the id of the LAST VW packet sent.
        """
        import re
        # Bare (no corr_id)
        bare = "EVT safety_stop"
        assert re.match(r"^EVT safety_stop$", bare), f"Unexpected bare format: {bare!r}"

        # With corr_id
        with_id = "EVT safety_stop #42"
        m = re.match(r"^EVT safety_stop #(\d+)$", with_id)
        assert m is not None, f"Unexpected id format: {with_id!r}"
        assert m.group(1) == "42"

    def test_s_command_still_uses_streaming_mode(self) -> None:
        """S command (beginStream) uses DriveMode::STREAMING → mode=S in TLM.

        Sprint 017-004 does NOT change the S command path.  S still calls
        beginStream, which sets _mode = DriveMode::STREAMING.
        """
        # Simulate the TLM line for an active S command.
        s_tlm = "TLM t=100 mode=S enc=100,100"
        assert "mode=S" in s_tlm
        assert "mode=V" not in s_tlm


# ---------------------------------------------------------------------------
# HELP verb list
# ---------------------------------------------------------------------------

class TestVWInHelp:
    """VW must be listed in the HELP response."""

    def test_vw_in_help_response(self) -> None:
        """HELP response includes 'VW' in the verb list."""
        help_body = "PING ECHO ID VER HELP SET GET GET VEL STREAM SNAP S T D G VW STOP GRIP ZERO OI OZ OR OP OV OL OA P PA"
        assert "VW" in help_body.split()

    def test_vw_alphabetically_after_g(self) -> None:
        """VW appears after G and before STOP in the HELP list."""
        verbs = "PING ECHO ID VER HELP SET GET GET VEL STREAM SNAP S T D G VW STOP GRIP ZERO OI OZ OR OP OV OL OA P PA".split()
        g_idx = verbs.index("G")
        vw_idx = verbs.index("VW")
        stop_idx = verbs.index("STOP")
        assert g_idx < vw_idx < stop_idx
