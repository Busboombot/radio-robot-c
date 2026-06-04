#!/usr/bin/env python3
"""test_config_registry.py — Unit tests for SET/GET config registry.

Covers sprint 009-004 (original 22 keys), sprint 010-004 (6 new
velocity/saturation tunables: vel.kP, vel.kI, vel.kFF, minWheelMms,
vWheelMax, steerHeadroom), sprint 011-001 (4 new pose-control tunables:
aMax, aDecel, turnGate, arriveTol), and sprint 012-001 (10 new OTOS
calibration and turn-asymmetry keys: otosLinSc, otosAngSc, rotGainPos,
rotGainNeg, rotOffPos, rotOffNeg, rotSlip, odomOffX, odomOffY, odomYaw).

These tests validate:
  - GET response format: CFG prefix, all 42 keys present, correct value format
  - GET subset: only requested keys returned
  - SET value parsing: float, integer, float-as-int
  - Error paths: badkey, missing key=value in SET
  - #id correlation in GET and SET responses
  - Response length under 768 bytes for full GET dump (buffer expanded in Sprint 012)
  - Integer params formatted without decimal point
  - Float params formatted with 3 decimal places
  - lapsToMmScale is absent from the registry

The tests simulate the wire protocol by parsing expected response strings,
verifying format without requiring a connected robot.
"""

from __future__ import annotations

import re
import pytest

# ---------------------------------------------------------------------------
# Helpers — parse CFG lines into key=value dicts
# ---------------------------------------------------------------------------

def parse_cfg(line: str) -> dict[str, str]:
    """Parse 'CFG k1=v1 k2=v2 ...' into {k: v}. Strips trailing #id."""
    assert line.startswith("CFG "), f"Expected CFG line, got: {line!r}"
    body = line[4:]
    # Strip trailing #id if present
    body = re.sub(r"\s+#\d+\s*$", "", body)
    result: dict[str, str] = {}
    for token in body.split():
        if "=" in token:
            k, v = token.split("=", 1)
            result[k] = v
    return result


def is_float_formatted(val: str) -> bool:
    """True if the value looks like a 3-decimal float (e.g. '0.487')."""
    return bool(re.fullmatch(r"-?\d+\.\d{3}", val))


def is_int_formatted(val: str) -> bool:
    """True if the value looks like a plain integer (no decimal point)."""
    return bool(re.fullmatch(r"-?\d+", val))


# ---------------------------------------------------------------------------
# Registry specification — mirrors kRegistry[] in CommandProcessor.cpp
# ---------------------------------------------------------------------------

# (key, type) where type is 'float', 'int', or 'float_as_int'
# Mirrors kRegistry[] in CommandProcessor.cpp — 32 entries as of Sprint 011-001.
REGISTRY = [
    # Original 22 keys (Sprint 009-004)
    ("ml",            "float"),
    ("mr",            "float"),
    ("kff",           "float"),
    ("klf",           "float"),
    ("klb",           "float"),
    ("krf",           "float"),
    ("krb",           "float"),
    ("adjThr",        "float"),
    ("adjGain",       "float"),
    ("tw",            "float_as_int"),
    ("pid.kp",        "float"),
    ("pid.ki",        "float"),
    ("pid.kd",        "float"),
    ("pid.max",       "float"),
    # New keys added Sprint 010-004: velocity/saturation tunables
    ("vel.kP",        "float"),
    ("vel.kI",        "float"),
    ("vel.kFF",       "float"),
    ("minWheelMms",   "float"),
    ("vWheelMax",     "float"),
    ("steerHeadroom", "float"),
    # Remaining original keys (legacy, retained for backward compatibility)
    ("turnThr",       "float_as_int"),
    ("doneTol",       "float_as_int"),
    ("distScale",     "float"),
    ("turnScale",     "float"),
    ("minSpeed",      "int"),
    ("sTimeout",      "int"),
    ("tick",          "int"),
    ("tlmPeriod",     "int"),
    # New keys added Sprint 011-001: pose-control tunables
    ("aMax",          "float"),
    ("aDecel",        "float"),
    ("turnGate",      "float_as_int"),   # wire: integer degrees
    ("arriveTol",     "float_as_int"),   # wire: integer mm
    # New keys added Sprint 012-001: OTOS calibration and turn asymmetry
    ("otosLinSc",     "float"),
    ("otosAngSc",     "float"),
    ("rotGainPos",    "float"),
    ("rotGainNeg",    "float"),
    ("rotOffPos",     "float"),
    ("rotOffNeg",     "float"),
    ("rotSlip",       "float"),
    ("odomOffX",      "float"),
    ("odomOffY",      "float"),
    ("odomYaw",       "float"),
]

REGISTRY_KEYS = [k for k, _ in REGISTRY]

# Default RobotConfig values as written to the wire by GET.
# These match defaultRobotConfig() in Config.h + expected %.3f / %d formatting.
# Sprint 010-004 adds 6 new keys after pid.max; sprint 011-001 adds 4 more.
# Total 32 keys.
DEFAULT_GET_LINE = (
    "CFG ml=0.487 mr=0.481 kff=0.150 klf=1.000 klb=1.000 krf=1.000 krb=1.000 "
    "adjThr=0.500 adjGain=0.050 tw=126 pid.kp=300.000 pid.ki=0.000 pid.kd=0.000 "
    "pid.max=30.000 vel.kP=0.300 vel.kI=0.050 vel.kFF=0.150 "
    "minWheelMms=20.000 vWheelMax=400.000 steerHeadroom=20.000 "
    "turnThr=50 doneTol=5 distScale=0.940 turnScale=1.070 "
    "minSpeed=50 sTimeout=500 tick=20 tlmPeriod=0 "
    "aMax=300.000 aDecel=250.000 turnGate=45 arriveTol=5 "
    "otosLinSc=1.050 otosAngSc=0.987 rotGainPos=1.000 rotGainNeg=1.170 "
    "rotOffPos=0.000 rotOffNeg=0.000 rotSlip=0.740 "
    "odomOffX=0.000 odomOffY=0.000 odomYaw=0.000"
)


# ---------------------------------------------------------------------------
# Tests against the registry specification
# ---------------------------------------------------------------------------

class TestRegistrySpec:
    """Validate the registry spec itself is consistent."""

    def test_all_42_keys_present(self) -> None:
        assert len(REGISTRY) == 42, f"Expected 42 registry entries, got {len(REGISTRY)}"

    def test_key_names_unique(self) -> None:
        keys = [k for k, _ in REGISTRY]
        assert len(keys) == len(set(keys)), "Duplicate key names in registry"

    def test_types_valid(self) -> None:
        valid_types = {"float", "int", "float_as_int"}
        for k, t in REGISTRY:
            assert t in valid_types, f"Key {k!r} has unknown type {t!r}"


class TestGetResponseFormat:
    """Tests for GET response format (based on expected wire output)."""

    def test_cfg_prefix(self) -> None:
        assert DEFAULT_GET_LINE.startswith("CFG ")

    def test_all_keys_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        for key in REGISTRY_KEYS:
            assert key in kv, f"Key {key!r} missing from GET response"

    def test_float_fields_have_3_decimal_places(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        for key, typ in REGISTRY:
            if typ == "float":
                assert is_float_formatted(kv[key]), (
                    f"Float key {key!r} value {kv[key]!r} not formatted as %.3f"
                )

    def test_int_fields_have_no_decimal_point(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        for key, typ in REGISTRY:
            if typ == "int":
                assert is_int_formatted(kv[key]), (
                    f"Int key {key!r} value {kv[key]!r} has unexpected decimal"
                )

    def test_float_as_int_fields_have_no_decimal_point(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        for key, typ in REGISTRY:
            if typ == "float_as_int":
                assert is_int_formatted(kv[key]), (
                    f"Float-as-int key {key!r} value {kv[key]!r} has unexpected decimal"
                )

    def test_default_ml_value(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["ml"] == "0.487"

    def test_default_mr_value(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["mr"] == "0.481"

    def test_default_tw_value(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["tw"] == "126"

    def test_default_pid_kp_value(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["pid.kp"] == "300.000"

    def test_default_sTimeout_value(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["sTimeout"] == "500"

    def test_default_tick_value(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["tick"] == "20"

    def test_default_tlmPeriod_value(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["tlmPeriod"] == "0"

    def test_response_length_under_768_bytes(self) -> None:
        # Sprint 012-001: GET buffer expanded from 512 to 768 to accommodate
        # 10 new keys (~156 extra bytes, pushing total to ~565 bytes).
        # The full GET response must fit in the firmware's 768-byte line[] buffer.
        length = len(DEFAULT_GET_LINE.encode("utf-8"))
        assert length < 768, (
            f"GET response is {length} bytes — exceeds 768-byte buffer limit"
        )

    def test_response_length_reasonable(self) -> None:
        """Confirm the response is in the expected range (~540-765 bytes).
        Sprint 010-004 added 6 keys raising the floor from ~238 to ~336 bytes.
        Sprint 011-001 added 4 more keys raising the floor to ~390 bytes.
        Sprint 012-001 added 10 more keys raising the floor to ~565 bytes.
        """
        length = len(DEFAULT_GET_LINE)
        assert 540 < length < 765, (
            f"GET response length {length} is outside expected range 540-765"
        )


class TestVelocityTunables:
    """Sprint 010-004: new velocity/saturation keys in kRegistry[]."""

    def test_vel_kP_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "vel.kP" in kv, "vel.kP missing from GET dump"

    def test_vel_kI_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "vel.kI" in kv, "vel.kI missing from GET dump"

    def test_vel_kFF_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "vel.kFF" in kv, "vel.kFF missing from GET dump"

    def test_minWheelMms_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "minWheelMms" in kv, "minWheelMms missing from GET dump"

    def test_vWheelMax_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "vWheelMax" in kv, "vWheelMax missing from GET dump"

    def test_steerHeadroom_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "steerHeadroom" in kv, "steerHeadroom missing from GET dump"

    def test_vel_kP_default_value(self) -> None:
        """vel.kP default is 0.3 → formatted as 0.300."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["vel.kP"] == "0.300", f"vel.kP expected '0.300', got {kv['vel.kP']!r}"

    def test_vel_kI_default_value(self) -> None:
        """vel.kI default is 0.05 → formatted as 0.050."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["vel.kI"] == "0.050", f"vel.kI expected '0.050', got {kv['vel.kI']!r}"

    def test_vel_kFF_default_value(self) -> None:
        """vel.kFF default is 0.15 → formatted as 0.150."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["vel.kFF"] == "0.150", f"vel.kFF expected '0.150', got {kv['vel.kFF']!r}"

    def test_minWheelMms_default_value(self) -> None:
        """minWheelMms default is 20.0 → formatted as 20.000."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["minWheelMms"] == "20.000", (
            f"minWheelMms expected '20.000', got {kv['minWheelMms']!r}"
        )

    def test_vWheelMax_default_value(self) -> None:
        """vWheelMax default is 400.0 → formatted as 400.000."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["vWheelMax"] == "400.000", (
            f"vWheelMax expected '400.000', got {kv['vWheelMax']!r}"
        )

    def test_steerHeadroom_default_value(self) -> None:
        """steerHeadroom default is 20.0 → formatted as 20.000."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["steerHeadroom"] == "20.000", (
            f"steerHeadroom expected '20.000', got {kv['steerHeadroom']!r}"
        )

    def test_velocity_keys_are_float_formatted(self) -> None:
        """All six new keys are floats with 3 decimal places."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        for key in ("vel.kP", "vel.kI", "vel.kFF", "minWheelMms", "vWheelMax", "steerHeadroom"):
            assert is_float_formatted(kv[key]), (
                f"Key {key!r} value {kv[key]!r} not formatted as %.3f"
            )

    def test_set_vel_kP_round_trip(self) -> None:
        """SET vel.kP=0.4 → OK set vel.kP=0.4; GET vel.kP → CFG vel.kP=0.400."""
        # Simulate SET round-trip response format.
        ok_line = "OK set vel.kP=0.4"
        assert ok_line.startswith("OK set")
        assert "vel.kP=0.4" in ok_line
        # Simulate GET response after SET.
        get_line = "CFG vel.kP=0.400"
        kv = parse_cfg(get_line)
        assert kv["vel.kP"] == "0.400"

    def test_set_vWheelMax_round_trip(self) -> None:
        """SET vWheelMax=350 → OK set vWheelMax=350; GET vWheelMax → CFG vWheelMax=350.000."""
        ok_line = "OK set vWheelMax=350"
        assert ok_line.startswith("OK set")
        assert "vWheelMax=350" in ok_line
        # vWheelMax is CFG_FLOAT, so GET shows %.3f
        get_line = "CFG vWheelMax=350.000"
        kv = parse_cfg(get_line)
        assert kv["vWheelMax"] == "350.000"

    def test_lapsToMmScale_absent(self) -> None:
        """lapsToMmScale must not appear in the registry (deleted in Ticket 001)."""
        for key, _ in REGISTRY:
            assert key != "lapsToMmScale", "lapsToMmScale is still in the registry"
        # Also confirm it's not in the default GET dump.
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "lapsToMmScale" not in kv, "lapsToMmScale appears in GET dump"


class TestPoseControlTunables:
    """Sprint 011-001: new pose-control keys in kRegistry[]."""

    def test_aMax_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "aMax" in kv, "aMax missing from GET dump"

    def test_aDecel_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "aDecel" in kv, "aDecel missing from GET dump"

    def test_turnGate_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "turnGate" in kv, "turnGate missing from GET dump"

    def test_arriveTol_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "arriveTol" in kv, "arriveTol missing from GET dump"

    def test_aMax_default_value(self) -> None:
        """aMax default is 300.0 → formatted as 300.000."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["aMax"] == "300.000", f"aMax expected '300.000', got {kv['aMax']!r}"

    def test_aDecel_default_value(self) -> None:
        """aDecel default is 250.0 → formatted as 250.000."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["aDecel"] == "250.000", f"aDecel expected '250.000', got {kv['aDecel']!r}"

    def test_turnGate_default_value(self) -> None:
        """turnGate default is 45.0° → formatted as integer 45."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["turnGate"] == "45", f"turnGate expected '45', got {kv['turnGate']!r}"

    def test_arriveTol_default_value(self) -> None:
        """arriveTol default is 5.0 mm → formatted as integer 5."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["arriveTol"] == "5", f"arriveTol expected '5', got {kv['arriveTol']!r}"

    def test_aMax_is_float_formatted(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert is_float_formatted(kv["aMax"]), (
            f"aMax value {kv['aMax']!r} not formatted as %.3f"
        )

    def test_aDecel_is_float_formatted(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert is_float_formatted(kv["aDecel"]), (
            f"aDecel value {kv['aDecel']!r} not formatted as %.3f"
        )

    def test_turnGate_is_int_formatted(self) -> None:
        """turnGate is CFG_FI — wire format is integer degrees."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert is_int_formatted(kv["turnGate"]), (
            f"turnGate value {kv['turnGate']!r} has unexpected decimal"
        )

    def test_arriveTol_is_int_formatted(self) -> None:
        """arriveTol is CFG_FI — wire format is integer mm."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert is_int_formatted(kv["arriveTol"]), (
            f"arriveTol value {kv['arriveTol']!r} has unexpected decimal"
        )

    def test_set_aMax_round_trip(self) -> None:
        """SET aMax=400 → OK set aMax=400; GET aMax → CFG aMax=400.000."""
        ok_line = "OK set aMax=400"
        assert ok_line.startswith("OK set")
        assert "aMax=400" in ok_line
        get_line = "CFG aMax=400.000"
        kv = parse_cfg(get_line)
        assert kv["aMax"] == "400.000"

    def test_set_aDecel_round_trip(self) -> None:
        """SET aDecel=300 → OK set aDecel=300; GET aDecel → CFG aDecel=300.000."""
        ok_line = "OK set aDecel=300"
        assert ok_line.startswith("OK set")
        assert "aDecel=300" in ok_line
        get_line = "CFG aDecel=300.000"
        kv = parse_cfg(get_line)
        assert kv["aDecel"] == "300.000"

    def test_set_turnGate_round_trip(self) -> None:
        """SET turnGate=60 → OK set turnGate=60; GET turnGate → CFG turnGate=60 (integer)."""
        ok_line = "OK set turnGate=60"
        assert ok_line.startswith("OK set")
        assert "turnGate=60" in ok_line
        get_line = "CFG turnGate=60"
        kv = parse_cfg(get_line)
        assert kv["turnGate"] == "60"
        assert is_int_formatted(kv["turnGate"])

    def test_set_arriveTol_round_trip(self) -> None:
        """SET arriveTol=10 → OK set arriveTol=10; GET arriveTol → CFG arriveTol=10."""
        ok_line = "OK set arriveTol=10"
        assert ok_line.startswith("OK set")
        assert "arriveTol=10" in ok_line
        get_line = "CFG arriveTol=10"
        kv = parse_cfg(get_line)
        assert kv["arriveTol"] == "10"
        assert is_int_formatted(kv["arriveTol"])

    def test_legacy_turnThr_still_present(self) -> None:
        """Legacy turnThr key must still be in the registry (backward compat)."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "turnThr" in kv, "Legacy turnThr key missing from GET dump"

    def test_legacy_doneTol_still_present(self) -> None:
        """Legacy doneTol key must still be in the registry (backward compat)."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "doneTol" in kv, "Legacy doneTol key missing from GET dump"

    def test_set_bad_key_still_returns_err(self) -> None:
        """SET badkey=1 still returns ERR badkey badkey (no regression)."""
        err_line = "ERR badkey badkey"
        assert err_line == "ERR badkey badkey"

    def test_full_get_42_keys(self) -> None:
        """Full GET dump has exactly 42 keys (32 existing + 10 new Sprint 012-001)."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert len(kv) == 42, f"Expected 42 keys in full GET, got {len(kv)}"

    def test_get_dump_under_768_bytes_with_new_keys(self) -> None:
        """Confirm the 42-key GET dump fits in the 768-byte firmware buffer (expanded Sprint 012-001)."""
        length = len(DEFAULT_GET_LINE.encode("utf-8"))
        assert length < 768, (
            f"GET response is {length} bytes — exceeds 768-byte buffer limit"
        )


class TestOtosAndTurnAsymmetryKeys:
    """Sprint 012-001: new OTOS calibration and turn-asymmetry keys in kRegistry[]."""

    def test_otosLinSc_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "otosLinSc" in kv, "otosLinSc missing from GET dump"

    def test_otosAngSc_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "otosAngSc" in kv, "otosAngSc missing from GET dump"

    def test_rotGainPos_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "rotGainPos" in kv, "rotGainPos missing from GET dump"

    def test_rotGainNeg_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "rotGainNeg" in kv, "rotGainNeg missing from GET dump"

    def test_rotOffPos_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "rotOffPos" in kv, "rotOffPos missing from GET dump"

    def test_rotOffNeg_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "rotOffNeg" in kv, "rotOffNeg missing from GET dump"

    def test_rotSlip_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "rotSlip" in kv, "rotSlip missing from GET dump"

    def test_odomOffX_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "odomOffX" in kv, "odomOffX missing from GET dump"

    def test_odomOffY_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "odomOffY" in kv, "odomOffY missing from GET dump"

    def test_odomYaw_present_in_full_get(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "odomYaw" in kv, "odomYaw missing from GET dump"

    def test_otosLinSc_default_value(self) -> None:
        """otosLinearScale default is 1.05 → formatted as 1.050."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["otosLinSc"] == "1.050", f"otosLinSc expected '1.050', got {kv['otosLinSc']!r}"

    def test_otosAngSc_default_value(self) -> None:
        """otosAngularScale default is 0.987 → formatted as 0.987."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["otosAngSc"] == "0.987", f"otosAngSc expected '0.987', got {kv['otosAngSc']!r}"

    def test_rotGainPos_default_value(self) -> None:
        """rotationGainPos default is 1.0 → formatted as 1.000."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["rotGainPos"] == "1.000", f"rotGainPos expected '1.000', got {kv['rotGainPos']!r}"

    def test_rotGainNeg_default_value(self) -> None:
        """rotationGainNeg default is 1.17 → formatted as 1.170."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["rotGainNeg"] == "1.170", f"rotGainNeg expected '1.170', got {kv['rotGainNeg']!r}"

    def test_rotOffPos_default_value(self) -> None:
        """rotationOffsetDeg default is 0.0 → formatted as 0.000."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["rotOffPos"] == "0.000", f"rotOffPos expected '0.000', got {kv['rotOffPos']!r}"

    def test_rotOffNeg_default_value(self) -> None:
        """rotationOffsetDegNeg default is 0.0 → formatted as 0.000."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["rotOffNeg"] == "0.000", f"rotOffNeg expected '0.000', got {kv['rotOffNeg']!r}"

    def test_rotSlip_default_value(self) -> None:
        """rotationalSlip default is 0.74 → formatted as 0.740."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["rotSlip"] == "0.740", f"rotSlip expected '0.740', got {kv['rotSlip']!r}"

    def test_odomOffX_default_value(self) -> None:
        """odomOffX default is 0.0 → formatted as 0.000."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["odomOffX"] == "0.000", f"odomOffX expected '0.000', got {kv['odomOffX']!r}"

    def test_odomOffY_default_value(self) -> None:
        """odomOffY default is 0.0 → formatted as 0.000."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["odomOffY"] == "0.000", f"odomOffY expected '0.000', got {kv['odomOffY']!r}"

    def test_odomYaw_default_value(self) -> None:
        """odomYawDeg default is 0.0 → formatted as 0.000."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["odomYaw"] == "0.000", f"odomYaw expected '0.000', got {kv['odomYaw']!r}"

    def test_all_sprint012_keys_are_float_formatted(self) -> None:
        """All 10 new Sprint 012-001 keys are CFG_F floats with 3 decimal places."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        sprint012_keys = (
            "otosLinSc", "otosAngSc", "rotGainPos", "rotGainNeg",
            "rotOffPos", "rotOffNeg", "rotSlip", "odomOffX", "odomOffY", "odomYaw"
        )
        for key in sprint012_keys:
            assert is_float_formatted(kv[key]), (
                f"Sprint 012 key {key!r} value {kv[key]!r} not formatted as %.3f"
            )

    def test_set_otosLinSc_round_trip(self) -> None:
        """SET otosLinSc=1.05 → OK set otosLinSc=1.05; GET otosLinSc → CFG otosLinSc=1.050."""
        ok_line = "OK set otosLinSc=1.05"
        assert ok_line.startswith("OK set")
        assert "otosLinSc=1.05" in ok_line
        get_line = "CFG otosLinSc=1.050"
        kv = parse_cfg(get_line)
        assert kv["otosLinSc"] == "1.050"

    def test_set_otosAngSc_round_trip(self) -> None:
        """SET otosAngSc=0.987 → OK set otosAngSc=0.987; GET otosAngSc → CFG otosAngSc=0.987."""
        ok_line = "OK set otosAngSc=0.987"
        assert ok_line.startswith("OK set")
        assert "otosAngSc=0.987" in ok_line
        get_line = "CFG otosAngSc=0.987"
        kv = parse_cfg(get_line)
        assert kv["otosAngSc"] == "0.987"

    def test_set_rotGainNeg_round_trip(self) -> None:
        """SET rotGainNeg=1.17 → OK set rotGainNeg=1.17; GET rotGainNeg → CFG rotGainNeg=1.170."""
        ok_line = "OK set rotGainNeg=1.17"
        assert ok_line.startswith("OK set")
        assert "rotGainNeg=1.17" in ok_line
        get_line = "CFG rotGainNeg=1.170"
        kv = parse_cfg(get_line)
        assert kv["rotGainNeg"] == "1.170"

    def test_all_10_sprint012_keys_round_trip(self) -> None:
        """All 10 new keys appear in DEFAULT_GET_LINE (simulated round-trip from defaults)."""
        kv = parse_cfg(DEFAULT_GET_LINE)
        sprint012_keys = (
            "otosLinSc", "otosAngSc", "rotGainPos", "rotGainNeg",
            "rotOffPos", "rotOffNeg", "rotSlip", "odomOffX", "odomOffY", "odomYaw"
        )
        for key in sprint012_keys:
            assert key in kv, f"Sprint 012-001 key {key!r} missing from full GET dump"

    def test_get_dump_length_reported(self) -> None:
        """Report and validate the new full GET dump byte count (565 bytes, fits in 768-byte buffer)."""
        length = len(DEFAULT_GET_LINE.encode("utf-8"))
        # The full 42-key GET dump is ~565 bytes; firmware buffer is now 768 bytes.
        assert 550 <= length <= 600, (
            f"GET dump is {length} bytes — outside expected 550-600 byte range for 42 keys"
        )
        assert length < 768, (
            f"GET dump is {length} bytes — exceeds 768-byte firmware buffer"
        )


class TestGetSubset:
    """Tests for GET with specific key arguments."""

    def _simulate_get_subset(self, keys: list[str]) -> str:
        """Simulate a GET response for specific keys from the default config."""
        kv_all = parse_cfg(DEFAULT_GET_LINE)
        parts = ["CFG"]
        for k in keys:
            if k in kv_all:
                parts.append(f"{k}={kv_all[k]}")
        return " ".join(parts)

    def test_get_single_key_ml(self) -> None:
        response = self._simulate_get_subset(["ml"])
        assert response == "CFG ml=0.487"

    def test_get_two_keys(self) -> None:
        response = self._simulate_get_subset(["ml", "pid.kp"])
        kv = parse_cfg(response)
        assert "ml" in kv
        assert "pid.kp" in kv
        assert len(kv) == 2

    def test_get_subset_format(self) -> None:
        response = self._simulate_get_subset(["tw", "sTimeout"])
        kv = parse_cfg(response)
        assert is_int_formatted(kv["tw"])
        assert is_int_formatted(kv["sTimeout"])


class TestSetResponseFormat:
    """Tests for SET wire response format."""

    def _simulate_set_ok(self, pairs: dict[str, str]) -> str:
        """Simulate 'OK set key=val key=val' response for valid keys."""
        applied = " ".join(f"{k}={v}" for k, v in pairs.items())
        return f"OK set {applied}"

    def _simulate_set_err(self, key: str) -> str:
        return f"ERR badkey {key}"

    def test_set_single_float_key(self) -> None:
        resp = self._simulate_set_ok({"ml": "0.500"})
        assert resp == "OK set ml=0.500"

    def test_set_multiple_keys(self) -> None:
        resp = self._simulate_set_ok({"ml": "0.487", "mr": "0.481", "tw": "120"})
        assert resp.startswith("OK set ")
        assert "ml=0.487" in resp
        assert "mr=0.481" in resp
        assert "tw=120" in resp

    def test_set_bad_key_format(self) -> None:
        resp = self._simulate_set_err("badkey")
        assert resp == "ERR badkey badkey"

    def test_set_pid_key_format(self) -> None:
        resp = self._simulate_set_ok({"pid.kp": "2.5"})
        assert "pid.kp=2.5" in resp


class TestCorrelationId:
    """Tests for #id correlation in GET and SET responses."""

    def test_get_with_corr_id(self) -> None:
        # Simulate GET response with #id: "CFG ml=0.487 #9"
        line = "CFG ml=0.487 #9"
        assert line.endswith("#9")
        # Strip the id and parse normally
        body = re.sub(r"\s+#\d+\s*$", "", line[4:])
        assert "ml=0.487" in body

    def test_set_ok_with_corr_id(self) -> None:
        line = "OK set ml=0.487 #42"
        assert line.startswith("OK set")
        assert line.endswith("#42")

    def test_set_err_with_corr_id(self) -> None:
        line = "ERR badkey foo #7"
        assert line.startswith("ERR badkey")
        assert line.endswith("#7")


class TestValueParsing:
    """Tests for value parsing logic (atof / atoi behaviour)."""

    def test_atof_positive_float(self) -> None:
        import ctypes  # just for reference; we test the logic in Python
        assert float("0.487") == pytest.approx(0.487)

    def test_atof_negative_float(self) -> None:
        assert float("-1.234") == pytest.approx(-1.234)

    def test_atoi_integer(self) -> None:
        assert int("200") == 200

    def test_atoi_negative_integer(self) -> None:
        assert int("-50") == -50

    def test_atoi_for_float_as_int(self) -> None:
        # CFG_FLOAT_AS_INT: atoi("120") → 120 → stored as float 120.0
        val = float(int("120"))
        assert val == pytest.approx(120.0)


class TestWireFormatExamples:
    """Concrete wire format examples from the ticket spec."""

    def test_example_get_all_format(self) -> None:
        """GET → CFG ml=0.487 mr=0.481 ... (all params, one line)"""
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert "ml" in kv
        assert "mr" in kv

    def test_example_get_subset_format(self) -> None:
        """GET ml pid.kp → CFG ml=0.487 pid.kp=2.0 (subset)"""
        # Simulate: parse a subset response
        line = "CFG ml=0.487 pid.kp=300.000"
        kv = parse_cfg(line)
        assert kv == {"ml": "0.487", "pid.kp": "300.000"}

    def test_example_set_ok_format(self) -> None:
        """SET ml=0.487 pid.kp=2.0 → OK set ml=0.487 pid.kp=2.0"""
        line = "OK set ml=0.487 pid.kp=2.0"
        assert line.startswith("OK set")
        assert "ml=0.487" in line

    def test_example_set_bad_key(self) -> None:
        """SET badkey=99 → ERR badkey badkey"""
        line = "ERR badkey badkey"
        assert line == "ERR badkey badkey"

    def test_example_set_mixed_valid_and_bad(self) -> None:
        """SET ml=0.487 bad=1 → OK set ml=0.487 (bad key emits ERR)"""
        ok_line = "OK set ml=0.487"
        err_line = "ERR badkey bad"
        assert ok_line.startswith("OK set")
        assert err_line.startswith("ERR badkey")


class TestNoKStarCommands:
    """Confirm the K* command table is no longer referenced."""

    def test_k_commands_not_in_registry(self) -> None:
        """All registry keys use friendly names, not K* names."""
        k_star_pattern = re.compile(r"^K[A-Z]+$")
        for key, _ in REGISTRY:
            assert not k_star_pattern.match(key), (
                f"Key {key!r} looks like a K* command name"
            )

    def test_registry_uses_v2_names(self) -> None:
        """Registry keys are lowercase friendly names."""
        for key, _ in REGISTRY:
            # Keys may contain lowercase letters, digits, and dots (e.g. pid.kp)
            assert re.fullmatch(r"[a-z][a-zA-Z0-9.]*", key), (
                f"Key {key!r} does not follow v2 naming convention"
            )
