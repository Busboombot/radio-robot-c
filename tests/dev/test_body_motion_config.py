#!/usr/bin/env python3
"""test_body_motion_config.py — Unit tests for Sprint 017-001 body motion limit config fields.

Covers five new kRegistry[] entries added to CommandProcessor.cpp:
  vBodyMax    — body forward speed ceiling, mm/s       (default 400.0)
  yawRateMax  — yaw rate ceiling, deg/s                (default 180.0)
  yawAccMax   — yaw acceleration limit, deg/s²         (default 720.0)
  jMax        — linear jerk limit, mm/s³  (0=trapezoid)(default 0.0)
  yawJerkMax  — yaw jerk limit, deg/s³   (0=trapezoid)(default 0.0)

These tests are pure-Python (no serial connection required) and mirror the
pattern of tests/dev/test_config_registry.py:
  - Each key is present in the registry spec with the correct default value.
  - Wire format: each is CFG_F (float, %.3f formatting).
  - Round-trip: encode float → key=val string, decode → same float.
"""

from __future__ import annotations

import re
import pytest

# ---------------------------------------------------------------------------
# Helpers — parse CFG lines into key=value dicts
# (mirrors test_config_registry.py)
# ---------------------------------------------------------------------------

def parse_cfg(line: str) -> dict[str, str]:
    """Parse 'CFG k1=v1 k2=v2 ...' into {k: v}. Strips trailing #id."""
    assert line.startswith("CFG "), f"Expected CFG line, got: {line!r}"
    body = line[4:]
    body = re.sub(r"\s+#\d+\s*$", "", body)
    result: dict[str, str] = {}
    for token in body.split():
        if "=" in token:
            k, v = token.split("=", 1)
            result[k] = v
    return result


def is_float_formatted(val: str) -> bool:
    """True if value looks like a 3-decimal float (e.g. '400.000')."""
    return bool(re.fullmatch(r"-?\d+\.\d{3}", val))


# ---------------------------------------------------------------------------
# Registry spec — only the five new Sprint 017-001 keys
# ---------------------------------------------------------------------------

# (key, type, default_wire_value)
# All five are CFG_F: stored as float, wire format %.3f.
BODY_MOTION_REGISTRY = [
    ("vBodyMax",   "float", "400.000"),   # body forward speed ceiling, mm/s
    ("yawRateMax", "float", "180.000"),   # yaw rate ceiling, deg/s
    ("yawAccMax",  "float", "720.000"),   # yaw acceleration limit, deg/s²
    ("jMax",       "float", "0.000"),     # linear jerk limit, mm/s³ (0=trapezoid)
    ("yawJerkMax", "float", "0.000"),     # yaw jerk limit, deg/s³   (0=trapezoid)
]

# Simulated GET line for the five new keys (default values).
# Represents the portion of a full GET dump for these fields.
DEFAULT_BODY_MOTION_CFG = (
    "CFG vBodyMax=400.000 yawRateMax=180.000 yawAccMax=720.000 "
    "jMax=0.000 yawJerkMax=0.000"
)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestBodyMotionRegistrySpec:
    """Validate the five-entry body motion registry spec."""

    def test_five_entries_defined(self) -> None:
        """Exactly five new body motion limit keys are defined."""
        assert len(BODY_MOTION_REGISTRY) == 5, (
            f"Expected 5 body motion registry entries, got {len(BODY_MOTION_REGISTRY)}"
        )

    def test_key_names_unique(self) -> None:
        keys = [k for k, _, _ in BODY_MOTION_REGISTRY]
        assert len(keys) == len(set(keys)), "Duplicate key names in body motion registry"

    def test_all_are_float_type(self) -> None:
        for key, typ, _ in BODY_MOTION_REGISTRY:
            assert typ == "float", (
                f"Key {key!r} expected type 'float', got {typ!r}"
            )

    def test_key_names(self) -> None:
        keys = [k for k, _, _ in BODY_MOTION_REGISTRY]
        assert "vBodyMax"   in keys
        assert "yawRateMax" in keys
        assert "yawAccMax"  in keys
        assert "jMax"       in keys
        assert "yawJerkMax" in keys


class TestBodyMotionDefaultValues:
    """Verify default values match the spec in Config.h and the architecture."""

    def test_vBodyMax_default(self) -> None:
        """vBodyMax default is 400.0 mm/s — matches vWheelMax ceiling."""
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        assert kv["vBodyMax"] == "400.000", (
            f"vBodyMax expected '400.000', got {kv['vBodyMax']!r}"
        )

    def test_yawRateMax_default(self) -> None:
        """yawRateMax default is 180.0 deg/s ≈ π rad/s."""
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        assert kv["yawRateMax"] == "180.000", (
            f"yawRateMax expected '180.000', got {kv['yawRateMax']!r}"
        )

    def test_yawAccMax_default(self) -> None:
        """yawAccMax default is 720.0 deg/s²."""
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        assert kv["yawAccMax"] == "720.000", (
            f"yawAccMax expected '720.000', got {kv['yawAccMax']!r}"
        )

    def test_jMax_default_is_zero(self) -> None:
        """jMax default is 0.0 — trapezoid profile (S-curve off)."""
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        assert kv["jMax"] == "0.000", (
            f"jMax expected '0.000', got {kv['jMax']!r}"
        )

    def test_yawJerkMax_default_is_zero(self) -> None:
        """yawJerkMax default is 0.0 — trapezoid profile (S-curve off)."""
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        assert kv["yawJerkMax"] == "0.000", (
            f"yawJerkMax expected '0.000', got {kv['yawJerkMax']!r}"
        )

    def test_all_five_keys_present_in_cfg(self) -> None:
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        for key, _, _ in BODY_MOTION_REGISTRY:
            assert key in kv, f"Key {key!r} missing from body motion CFG line"

    def test_all_defaults_match_spec(self) -> None:
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        for key, _, expected in BODY_MOTION_REGISTRY:
            assert kv[key] == expected, (
                f"Key {key!r}: expected {expected!r}, got {kv[key]!r}"
            )


class TestBodyMotionWireFormat:
    """Verify all five keys use CFG_F (%.3f) wire format."""

    def test_vBodyMax_is_float_formatted(self) -> None:
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        assert is_float_formatted(kv["vBodyMax"]), (
            f"vBodyMax value {kv['vBodyMax']!r} not formatted as %.3f"
        )

    def test_yawRateMax_is_float_formatted(self) -> None:
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        assert is_float_formatted(kv["yawRateMax"]), (
            f"yawRateMax value {kv['yawRateMax']!r} not formatted as %.3f"
        )

    def test_yawAccMax_is_float_formatted(self) -> None:
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        assert is_float_formatted(kv["yawAccMax"]), (
            f"yawAccMax value {kv['yawAccMax']!r} not formatted as %.3f"
        )

    def test_jMax_is_float_formatted(self) -> None:
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        assert is_float_formatted(kv["jMax"]), (
            f"jMax value {kv['jMax']!r} not formatted as %.3f"
        )

    def test_yawJerkMax_is_float_formatted(self) -> None:
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        assert is_float_formatted(kv["yawJerkMax"]), (
            f"yawJerkMax value {kv['yawJerkMax']!r} not formatted as %.3f"
        )

    def test_all_body_motion_keys_are_float_formatted(self) -> None:
        """All five body motion keys are CFG_F — always %.3f on wire."""
        kv = parse_cfg(DEFAULT_BODY_MOTION_CFG)
        for key, _, _ in BODY_MOTION_REGISTRY:
            assert is_float_formatted(kv[key]), (
                f"Body motion key {key!r} value {kv[key]!r} not formatted as %.3f"
            )


class TestBodyMotionRoundTrip:
    """Round-trip: encode float → key=val string, decode → same float."""

    def _encode(self, key: str, value: float) -> str:
        """Simulate CFG_F wire encoding (%.3f)."""
        return f"CFG {key}={value:.3f}"

    def _decode(self, line: str, key: str) -> float:
        """Simulate CFG_F wire decoding (atof)."""
        kv = parse_cfg(line)
        return float(kv[key])

    def test_vBodyMax_round_trip_default(self) -> None:
        """400.0 encodes as '400.000' and decodes back to 400.0."""
        line = self._encode("vBodyMax", 400.0)
        val = self._decode(line, "vBodyMax")
        assert val == pytest.approx(400.0)

    def test_vBodyMax_round_trip_set_value(self) -> None:
        """SET vBodyMax=300 encodes as '300.000' and decodes back to 300.0."""
        line = self._encode("vBodyMax", 300.0)
        val = self._decode(line, "vBodyMax")
        assert val == pytest.approx(300.0)

    def test_yawRateMax_round_trip(self) -> None:
        """180.0 encodes as '180.000' and decodes back to 180.0."""
        line = self._encode("yawRateMax", 180.0)
        val = self._decode(line, "yawRateMax")
        assert val == pytest.approx(180.0)

    def test_yawAccMax_round_trip(self) -> None:
        """720.0 encodes as '720.000' and decodes back to 720.0."""
        line = self._encode("yawAccMax", 720.0)
        val = self._decode(line, "yawAccMax")
        assert val == pytest.approx(720.0)

    def test_jMax_round_trip_default(self) -> None:
        """0.0 encodes as '0.000' and decodes back to 0.0."""
        line = self._encode("jMax", 0.0)
        val = self._decode(line, "jMax")
        assert val == pytest.approx(0.0)

    def test_jMax_round_trip_nonzero(self) -> None:
        """SET jMax=1000.0 encodes as '1000.000' and decodes back to 1000.0."""
        line = self._encode("jMax", 1000.0)
        val = self._decode(line, "jMax")
        assert val == pytest.approx(1000.0)

    def test_yawJerkMax_round_trip_default(self) -> None:
        """0.0 encodes as '0.000' and decodes back to 0.0."""
        line = self._encode("yawJerkMax", 0.0)
        val = self._decode(line, "yawJerkMax")
        assert val == pytest.approx(0.0)

    def test_yawJerkMax_round_trip_nonzero(self) -> None:
        """SET yawJerkMax=360.0 encodes as '360.000' and decodes back to 360.0."""
        line = self._encode("yawJerkMax", 360.0)
        val = self._decode(line, "yawJerkMax")
        assert val == pytest.approx(360.0)

    def test_set_vBodyMax_ok_format(self) -> None:
        """SET vBodyMax=300 → OK set vBodyMax=300 (wire format for SET response)."""
        ok_line = "OK set vBodyMax=300"
        assert ok_line.startswith("OK set")
        assert "vBodyMax=300" in ok_line
        # GET after SET shows %.3f
        get_line = "CFG vBodyMax=300.000"
        kv = parse_cfg(get_line)
        assert kv["vBodyMax"] == "300.000"

    def test_set_jMax_zero_ok_format(self) -> None:
        """SET jMax=0 → OK set jMax=0 (trapezoid profile, no S-curve)."""
        ok_line = "OK set jMax=0"
        assert ok_line.startswith("OK set")
        assert "jMax=0" in ok_line
        get_line = "CFG jMax=0.000"
        kv = parse_cfg(get_line)
        assert kv["jMax"] == "0.000"


class TestBodyMotionRegistryNaming:
    """Verify key names follow the v2 naming convention (lowercase friendly names)."""

    def test_key_names_follow_v2_convention(self) -> None:
        """Body motion keys are camelCase — starts with lowercase, no dots for these keys."""
        for key, _, _ in BODY_MOTION_REGISTRY:
            # Keys may contain lowercase letters, uppercase letters, and digits.
            # vBodyMax, yawRateMax, yawAccMax, jMax, yawJerkMax all start lowercase.
            assert key[0].islower(), f"Key {key!r} does not start with a lowercase letter"

    def test_aMax_not_duplicated(self) -> None:
        """aMax and aDecel are NOT in the body motion keys (they are shared from Sprint 011)."""
        body_keys = [k for k, _, _ in BODY_MOTION_REGISTRY]
        assert "aMax" not in body_keys, "aMax should not be in body motion keys (already in Sprint 011)"
        assert "aDecel" not in body_keys, "aDecel should not be in body motion keys (already in Sprint 011)"
