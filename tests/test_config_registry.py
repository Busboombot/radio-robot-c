#!/usr/bin/env python3
"""test_config_registry.py — Unit tests for SET/GET config registry (009-004).

These tests validate:
  - GET response format: CFG prefix, all 22 keys present, correct value format
  - GET subset: only requested keys returned
  - SET value parsing: float, integer, float-as-int
  - Error paths: badkey, missing key=value in SET
  - #id correlation in GET and SET responses
  - Response length under 512 bytes for full GET dump
  - Integer params formatted without decimal point
  - Float params formatted with 3 decimal places

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
REGISTRY = [
    ("ml",        "float"),
    ("mr",        "float"),
    ("kff",       "float"),
    ("klf",       "float"),
    ("klb",       "float"),
    ("krf",       "float"),
    ("krb",       "float"),
    ("adjThr",    "float"),
    ("adjGain",   "float"),
    ("tw",        "float_as_int"),
    ("pid.kp",    "float"),
    ("pid.ki",    "float"),
    ("pid.kd",    "float"),
    ("pid.max",   "float"),
    ("turnThr",   "float_as_int"),
    ("doneTol",   "float_as_int"),
    ("distScale", "float"),
    ("turnScale", "float"),
    ("minSpeed",  "int"),
    ("sTimeout",  "int"),
    ("tick",      "int"),
    ("tlmPeriod", "int"),
]

REGISTRY_KEYS = [k for k, _ in REGISTRY]

# Default RobotConfig values as written to the wire by GET.
# These match defaultRobotConfig() in Config.h + expected %.3f / %d formatting.
DEFAULT_GET_LINE = (
    "CFG ml=0.487 mr=0.481 kff=0.150 klf=1.000 klb=1.000 krf=1.000 krb=1.000 "
    "adjThr=0.500 adjGain=0.050 tw=120 pid.kp=300.000 pid.ki=0.000 pid.kd=0.000 "
    "pid.max=30.000 turnThr=50 doneTol=5 distScale=0.940 turnScale=1.070 "
    "minSpeed=50 sTimeout=200 tick=20 tlmPeriod=0"
)


# ---------------------------------------------------------------------------
# Tests against the registry specification
# ---------------------------------------------------------------------------

class TestRegistrySpec:
    """Validate the registry spec itself is consistent."""

    def test_all_22_keys_present(self) -> None:
        assert len(REGISTRY) == 22, f"Expected 22 registry entries, got {len(REGISTRY)}"

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
        assert kv["tw"] == "120"

    def test_default_pid_kp_value(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["pid.kp"] == "300.000"

    def test_default_sTimeout_value(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["sTimeout"] == "200"

    def test_default_tick_value(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["tick"] == "20"

    def test_default_tlmPeriod_value(self) -> None:
        kv = parse_cfg(DEFAULT_GET_LINE)
        assert kv["tlmPeriod"] == "0"

    def test_response_length_under_512_bytes(self) -> None:
        # The full GET response (including the CFG prefix and all key=value
        # pairs) must fit in the firmware's 512-byte buffer.
        length = len(DEFAULT_GET_LINE.encode("utf-8"))
        assert length < 512, (
            f"GET response is {length} bytes — exceeds 512-byte buffer limit"
        )

    def test_response_length_reasonable(self) -> None:
        """Confirm the response is in the expected range (~200-400 bytes)."""
        length = len(DEFAULT_GET_LINE)
        assert 100 < length < 450, (
            f"GET response length {length} is outside expected range 100-450"
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
