"""Tests for per-robot config loading, schema validation, and match_robot_by_id.

All tests are pure Python — no serial hardware required.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from robot_radio.config.robot_config import (
    RobotConfig,
    _reset_robot_config,
    get_robot_config,
    load_robot_config,
    match_robot_by_id,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# Resolve the project root the same way robot_config.py does:
# robot_config.py is at  host/robot_radio/config/robot_config.py
# _PROJECT_ROOT = Path(__file__).parent.parent.parent  => repo root
_REPO_ROOT = Path(__file__).parent.parent.parent
_ROBOTS_DIR = _REPO_ROOT / "data" / "robots"
_TOVEZ_JSON = _ROBOTS_DIR / "tovez.json"
_ACTIVE_JSON = _ROBOTS_DIR / "active_robot.json"
_SCHEMA_JSON = _ROBOTS_DIR / "robot_config.schema.json"


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def reset_config_cache():
    """Clear the singleton cache before and after each test."""
    _reset_robot_config()
    yield
    _reset_robot_config()


@pytest.fixture(autouse=True)
def clear_env(monkeypatch):
    """Remove ROBOT_CONFIG from env so tests don't inherit it."""
    monkeypatch.delenv("ROBOT_CONFIG", raising=False)


# ---------------------------------------------------------------------------
# Data-file presence tests
# ---------------------------------------------------------------------------

class TestDataFiles:
    def test_tovez_json_exists(self):
        assert _TOVEZ_JSON.exists(), f"Missing: {_TOVEZ_JSON}"

    def test_active_robot_json_exists(self):
        assert _ACTIVE_JSON.exists(), f"Missing: {_ACTIVE_JSON}"

    def test_schema_json_exists(self):
        assert _SCHEMA_JSON.exists(), f"Missing: {_SCHEMA_JSON}"


# ---------------------------------------------------------------------------
# Schema validation tests (using jsonschema if available)
# ---------------------------------------------------------------------------

class TestSchemaValidation:
    def test_schema_is_valid_json(self):
        schema = json.loads(_SCHEMA_JSON.read_text())
        assert schema.get("type") == "object"
        assert "properties" in schema

    def test_tovez_validates_against_schema(self):
        """Validate tovez.json against robot_config.schema.json."""
        pytest.importorskip("jsonschema")
        import jsonschema

        schema = json.loads(_SCHEMA_JSON.read_text())
        instance = json.loads(_TOVEZ_JSON.read_text())
        # Should not raise
        jsonschema.validate(instance, schema)

    def test_schema_has_calibration_fields(self):
        """Schema must contain all rotation_gain_neg / mm_per_wheel_deg fields."""
        schema = json.loads(_SCHEMA_JSON.read_text())
        calib = schema["properties"]["calibration"]["properties"]
        required_fields = [
            "mm_per_wheel_deg_left",
            "mm_per_wheel_deg_right",
            "rotational_slip",
            "rotation_gain",
            "rotation_offset_deg",
            "rotation_gain_neg",
            "rotation_offset_deg_neg",
        ]
        for field in required_fields:
            assert field in calib, f"Schema missing calibration field: {field}"


# ---------------------------------------------------------------------------
# Pydantic model load tests
# ---------------------------------------------------------------------------

class TestLoadRobotConfig:
    def test_load_tovez(self):
        cfg = load_robot_config(_TOVEZ_JSON)
        assert isinstance(cfg, RobotConfig)
        assert cfg.robot_name == "tovez"

    def test_tovez_calibration_values(self):
        cfg = load_robot_config(_TOVEZ_JSON)
        assert cfg.calibration.otos_linear_scale == pytest.approx(1.05)
        assert cfg.calibration.otos_angular_scale == pytest.approx(0.987)
        assert cfg.calibration.mm_per_wheel_deg_left == pytest.approx(0.487)
        assert cfg.calibration.mm_per_wheel_deg_right == pytest.approx(0.481)
        assert cfg.calibration.rotational_slip == pytest.approx(0.74)
        assert cfg.calibration.rotation_gain == pytest.approx(1.0)
        assert cfg.calibration.rotation_offset_deg == pytest.approx(0.0)
        assert cfg.calibration.rotation_gain_neg == pytest.approx(1.17)
        assert cfg.calibration.rotation_offset_deg_neg == pytest.approx(0.0)

    def test_tovez_geometry(self):
        cfg = load_robot_config(_TOVEZ_JSON)
        assert cfg.trackwidth == pytest.approx(126)

    def test_tovez_wheels(self):
        cfg = load_robot_config(_TOVEZ_JSON)
        assert cfg.wheels.wheel_diameter_mm == pytest.approx(80.77)
        assert cfg.wheels.ticks_per_rev == pytest.approx(360)
        assert cfg.wheels.ticks_per_mm == pytest.approx(1.4187)

    def test_tovez_connection(self):
        cfg = load_robot_config(_TOVEZ_JSON)
        assert cfg.connection.device_announcement_name == "tovez"
        assert cfg.connection.serial_last_6 == "f137c0"

    def test_tovez_identity(self):
        cfg = load_robot_config(_TOVEZ_JSON)
        assert cfg.identity.hardware_model == "DFRobot Nezha"
        assert cfg.identity.common_name == "classroom-bot"

    def test_tovez_schema_version(self):
        cfg = load_robot_config(_TOVEZ_JSON)
        assert cfg.schema_version == 2

    def test_otos_scalars(self):
        cfg = load_robot_config(_TOVEZ_JSON)
        # otos_linear_scale=1.05 → scalar = round((1.05-1)/0.001) = 50
        assert cfg.otos_linear_scalar == 50
        # otos_angular_scale=0.987 → scalar = round((0.987-1)/0.001) = -13
        assert cfg.otos_angular_scalar == -13


# ---------------------------------------------------------------------------
# get_robot_config() via active_robot.json pointer
# ---------------------------------------------------------------------------

class TestGetRobotConfig:
    def test_resolves_active_robot(self):
        cfg = get_robot_config()
        assert cfg is not None
        assert cfg.robot_name == "tovez"

    def test_returns_singleton(self):
        cfg1 = get_robot_config()
        cfg2 = get_robot_config()
        assert cfg1 is cfg2

    def test_env_var_overrides(self, monkeypatch, tmp_path):
        """ROBOT_CONFIG env var takes priority over active_robot.json."""
        # Write a minimal config to a temp file
        tmp_cfg = tmp_path / "tmp_robot.json"
        tmp_cfg.write_text(json.dumps({
            "schema_version": 2,
            "identity": {"robot_name": "test-bot", "uid": "test-bot"},
        }))
        monkeypatch.setenv("ROBOT_CONFIG", str(tmp_cfg))
        cfg = get_robot_config()
        assert cfg is not None
        assert cfg.robot_name == "test-bot"


# ---------------------------------------------------------------------------
# match_robot_by_id() tests
# ---------------------------------------------------------------------------

class TestMatchRobotById:
    def test_match_by_exact_name(self):
        """ID response with name=tovez returns the tovez config."""
        cfg = match_robot_by_id("ID model=Nezha2 name=tovez serial=89f137c0")
        assert cfg is not None
        assert cfg.robot_name == "tovez"

    def test_match_case_insensitive(self):
        """Match should be case-insensitive (firmware may uppercase the name)."""
        cfg = match_robot_by_id("ID model=Nezha2 name=TOVEZ serial=89f137c0")
        assert cfg is not None
        assert cfg.robot_name == "tovez"

    def test_match_name_only_token(self):
        """name= field can appear anywhere in the line."""
        cfg = match_robot_by_id("ID name=TOVEZ model=Nezha2")
        assert cfg is not None
        assert cfg.robot_name == "tovez"

    def test_no_name_field_falls_back(self):
        """When there is no name= field, falls back to get_robot_config()."""
        cfg = match_robot_by_id("ID model=Nezha2 serial=89f137c0")
        # Should fall back to active robot (tovez)
        assert cfg is not None
        assert cfg.robot_name == "tovez"

    def test_unknown_name_falls_back(self):
        """When name= doesn't match any file, falls back to get_robot_config()."""
        cfg = match_robot_by_id("ID model=Nezha2 name=UNKNOWN_ROBOT serial=000000")
        # Should fall back to active robot (tovez)
        assert cfg is not None
        assert cfg.robot_name == "tovez"

    def test_returns_correct_calibration(self):
        """Matched config carries the full calibration data."""
        cfg = match_robot_by_id("ID model=Nezha2 name=TOVEZ serial=89f137c0")
        assert cfg is not None
        assert cfg.calibration.otos_linear_scale == pytest.approx(1.05)
        assert cfg.calibration.rotation_gain_neg == pytest.approx(1.17)
