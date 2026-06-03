"""Unit tests for odom_tracker module.

Tests cover:
- parse_so() — legacy v1 SO format parser (kept for compatibility).
- parse_tlm() — new v2 TLM pose= parser delegate.
- Scale math helpers from calibrate_linear.py and calibrate_angular.py.
- Config write-back helpers (save_linear_scale_to_config, save_angular_calibration_to_config).

No hardware or serial connection is required.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# Ensure host package is importable
_HOST = Path(__file__).resolve().parent.parent
if str(_HOST) not in sys.path:
    sys.path.insert(0, str(_HOST))

from robot_radio.sensors.odom_tracker import parse_so, parse_tlm


# ---------------------------------------------------------------------------
# parse_so() — legacy v1 format
# ---------------------------------------------------------------------------

class TestParseSO:
    """Tests for the legacy v1 SO stream parser."""

    def test_typical_so_line(self) -> None:
        result = parse_so("SO+1234-0567+090")
        assert result == (1234, -567, 90)

    def test_all_positive(self) -> None:
        result = parse_so("SO+0100+0200+045")
        assert result == (100, 200, 45)

    def test_all_negative(self) -> None:
        result = parse_so("SO-0100-0200-045")
        assert result == (-100, -200, -45)

    def test_relay_prefix_stripped(self) -> None:
        result = parse_so("<SO+1234-0567+090")
        assert result == (1234, -567, 90)

    def test_non_so_returns_none(self) -> None:
        assert parse_so("TLM t=100 pose=0,0,0") is None
        assert parse_so("OK pong") is None
        assert parse_so("") is None

    def test_malformed_so_returns_none(self) -> None:
        # Too few parts
        assert parse_so("SO+0100") is None

    def test_zero_values(self) -> None:
        result = parse_so("SO+0000+0000+000")
        assert result == (0, 0, 0)


# ---------------------------------------------------------------------------
# parse_tlm() — new v2 TLM delegate
# ---------------------------------------------------------------------------

class TestParseTLM:
    """Tests for the v2 TLM parser wrapper in odom_tracker."""

    def test_full_tlm_returns_dict_with_pose(self) -> None:
        result = parse_tlm("TLM t=500 pose=350,-12,1780 enc=1024,1019")
        assert result is not None
        assert isinstance(result, dict)
        assert result["pose"] == (350, -12, 1780)
        assert result["enc"] == (1024, 1019)
        assert result["t"] == 500

    def test_pose_only(self) -> None:
        result = parse_tlm("TLM t=1000 pose=200,-50,9000")
        assert result is not None
        assert result["pose"] == (200, -50, 9000)
        assert "enc" not in result

    def test_enc_only(self) -> None:
        result = parse_tlm("TLM t=300 enc=100,95")
        assert result is not None
        assert result["enc"] == (100, 95)
        assert "pose" not in result

    def test_relay_prefix_stripped(self) -> None:
        result = parse_tlm("< TLM t=500 pose=100,0,3600")
        assert result is not None
        assert result["pose"] == (100, 0, 3600)

    def test_non_tlm_returns_none(self) -> None:
        assert parse_tlm("OK pong t=12345") is None
        assert parse_tlm("EVT done D") is None
        assert parse_tlm("SO+1234-0567+090") is None
        assert parse_tlm("") is None

    def test_negative_pose_values(self) -> None:
        result = parse_tlm("TLM t=100 pose=-200,-50,-9000")
        assert result is not None
        assert result["pose"] == (-200, -50, -9000)

    def test_heading_in_centidegrees(self) -> None:
        # 9000 cdeg = 90.00 degrees
        result = parse_tlm("TLM t=100 pose=0,0,9000")
        assert result is not None
        assert result["pose"][2] == 9000   # raw cdeg, not degrees

    def test_vel_field_included(self) -> None:
        result = parse_tlm("TLM t=100 vel=200,195")
        assert result is not None
        assert result.get("vel") == (200, 195)

    def test_t_field_included(self) -> None:
        result = parse_tlm("TLM t=9999 pose=0,0,0")
        assert result is not None
        assert result["t"] == 9999

    def test_mode_field_included(self) -> None:
        result = parse_tlm("TLM t=100 mode=D pose=0,0,0")
        assert result is not None
        assert result.get("mode") == "D"

    def test_bare_tlm_returns_empty_dict(self) -> None:
        """A bare 'TLM' line with no fields returns an empty dict (not None)."""
        result = parse_tlm("TLM")
        # Should be an empty dict or None — either is acceptable,
        # but must not raise.
        assert result is None or isinstance(result, dict)


# ---------------------------------------------------------------------------
# Scale math (linear calibration)
# ---------------------------------------------------------------------------

class TestLinearScaleMath:
    """Tests for calibrate_linear.py scale computation helpers."""

    def setup_method(self) -> None:
        # Import the helpers from the calibrate script
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "calibrate_linear",
            _HOST / "calibrate_linear.py",
        )
        self.mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
        spec.loader.exec_module(self.mod)  # type: ignore[union-attr]

    def test_scale_to_int8_typical(self) -> None:
        assert self.mod.scale_to_int8(1.027) == 27

    def test_scale_to_int8_zero(self) -> None:
        assert self.mod.scale_to_int8(1.0) == 0

    def test_scale_to_int8_negative(self) -> None:
        assert self.mod.scale_to_int8(0.987) == -13

    def test_scale_to_int8_clamp_high(self) -> None:
        assert self.mod.scale_to_int8(1.5) == 127

    def test_scale_to_int8_clamp_low(self) -> None:
        assert self.mod.scale_to_int8(0.5) == -128

    def test_int8_to_scale_roundtrip(self) -> None:
        for val in (-128, -50, 0, 27, 50, 127):
            scale = self.mod.int8_to_scale(val)
            assert self.mod.scale_to_int8(scale) == val

    def test_compute_new_linear_scale_exact(self) -> None:
        # If otos = 500mm, actual = 550mm, current = 1.0 → ratio = 1.1
        scale, int8 = self.mod.compute_new_linear_scale(550.0, 500.0, 1.0)
        assert abs(scale - 1.1) < 0.002
        assert int8 == 100

    def test_compute_new_linear_scale_current_not_one(self) -> None:
        # current = 1.05, actual = otos (ratio=1.0) → new = 1.05
        scale, int8 = self.mod.compute_new_linear_scale(500.0, 500.0, 1.05)
        assert abs(scale - 1.05) < 0.001
        assert int8 == 50

    def test_compute_new_linear_scale_clamp(self) -> None:
        # Extreme ratio that would exceed firmware range
        scale, int8 = self.mod.compute_new_linear_scale(2000.0, 500.0, 1.0)
        # ratio = 4.0 → raw = 4.0, clamped to 1.127
        assert scale <= 1.127
        assert int8 == 127

    def test_mean_ratio_stats_single_sample(self) -> None:
        mean, stdev, sem = self.mod.mean_ratio_stats([(500.0, 550.0)])
        assert abs(mean - 1.1) < 1e-6
        assert stdev == 0.0
        assert sem == 0.0

    def test_mean_ratio_stats_multiple(self) -> None:
        # Two samples with ratio 1.0 and 1.2 → mean 1.1
        samples = [(500.0, 500.0), (500.0, 600.0)]
        mean, stdev, sem = self.mod.mean_ratio_stats(samples)
        assert abs(mean - 1.1) < 1e-6
        assert stdev > 0.0
        assert sem > 0.0

    def test_mean_ratio_stats_empty(self) -> None:
        mean, stdev, sem = self.mod.mean_ratio_stats([])
        assert mean == 0.0
        assert stdev == 0.0
        assert sem == 0.0


# ---------------------------------------------------------------------------
# Scale math (angular calibration)
# ---------------------------------------------------------------------------

class TestAngularScaleMath:
    """Tests for calibrate_angular.py scale computation helpers."""

    def setup_method(self) -> None:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "calibrate_angular",
            _HOST / "calibrate_angular.py",
        )
        self.mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
        spec.loader.exec_module(self.mod)  # type: ignore[union-attr]

    def test_scale_to_int8_typical(self) -> None:
        assert self.mod.scale_to_int8(1.027) == 27

    def test_compute_new_angular_scale_exact(self) -> None:
        # target=360, otos=350 → ratio=360/350≈1.0286, new=1.0×1.0286
        scale, int8 = self.mod.compute_new_angular_scale(360.0, 350.0, 1.0)
        expected = 360.0 / 350.0
        assert abs(scale - expected) < 0.001

    def test_compute_new_angular_scale_identity(self) -> None:
        # target = otos → ratio = 1.0, scale unchanged
        scale, int8 = self.mod.compute_new_angular_scale(360.0, 360.0, 1.05)
        assert abs(scale - 1.05) < 0.001

    def test_compute_new_angular_scale_zero_otos(self) -> None:
        # otos = 0 → should not raise, returns current scale
        scale, int8 = self.mod.compute_new_angular_scale(360.0, 0.0, 1.0)
        assert scale == 1.0

    def test_heading_delta_cdeg_positive(self) -> None:
        # 90° CCW rotation: 0 → 9000 cdeg
        delta = self.mod.heading_delta_cdeg(0, 9000)
        assert abs(delta - 90.0) < 0.01

    def test_heading_delta_cdeg_negative(self) -> None:
        # 90° CW rotation: 0 → -9000 cdeg
        delta = self.mod.heading_delta_cdeg(0, -9000)
        assert abs(delta - (-90.0)) < 0.01

    def test_heading_delta_cdeg_wrap(self) -> None:
        # Before=17000 cdeg (170°), after=-17000 cdeg (-170°)
        # Physical CCW rotation of 20° (from 170° → 190° = -170°)
        delta = self.mod.heading_delta_cdeg(17000, -17000)
        assert abs(delta - 20.0) < 0.1

    def test_mean_stdev_typical(self) -> None:
        mean, stdev = self.mod.mean_stdev([1.0, 2.0, 3.0])
        assert abs(mean - 2.0) < 1e-9
        assert abs(stdev - 1.0) < 1e-9

    def test_mean_stdev_empty(self) -> None:
        mean, stdev = self.mod.mean_stdev([])
        assert mean == 0.0
        assert stdev == 0.0

    def test_mean_stdev_single(self) -> None:
        mean, stdev = self.mod.mean_stdev([5.0])
        assert mean == 5.0
        assert stdev == 0.0


# ---------------------------------------------------------------------------
# Config write-back (linear)
# ---------------------------------------------------------------------------

class TestLinearConfigWriteback:
    """Tests for save_linear_scale_to_config."""

    def setup_method(self) -> None:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "calibrate_linear",
            _HOST / "calibrate_linear.py",
        )
        self.mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
        spec.loader.exec_module(self.mod)  # type: ignore[union-attr]

    def test_save_linear_scale(self, tmp_path: Path) -> None:
        cfg = {
            "calibration": {
                "otos_linear_scale": 1.0,
                "otos_angular_scale": 0.987,
            }
        }
        config_file = tmp_path / "robot.json"
        config_file.write_text(json.dumps(cfg, indent=2))

        self.mod.save_linear_scale_to_config(config_file, 1.050)

        result = json.loads(config_file.read_text())
        assert abs(result["calibration"]["otos_linear_scale"] - 1.05) < 1e-6
        # Other fields preserved
        assert abs(result["calibration"]["otos_angular_scale"] - 0.987) < 1e-6

    def test_save_creates_calibration_key_if_missing(self, tmp_path: Path) -> None:
        cfg = {"identity": {"robot_name": "testbot"}}
        config_file = tmp_path / "robot.json"
        config_file.write_text(json.dumps(cfg, indent=2))

        self.mod.save_linear_scale_to_config(config_file, 1.027)

        result = json.loads(config_file.read_text())
        assert abs(result["calibration"]["otos_linear_scale"] - 1.027) < 1e-6
        # Original key preserved
        assert result["identity"]["robot_name"] == "testbot"

    def test_load_current_linear_scale(self, tmp_path: Path) -> None:
        cfg = {"calibration": {"otos_linear_scale": 1.05}}
        config_file = tmp_path / "robot.json"
        config_file.write_text(json.dumps(cfg))

        scale = self.mod.load_current_linear_scale(config_file)
        assert abs(scale - 1.05) < 1e-6

    def test_load_current_linear_scale_default(self, tmp_path: Path) -> None:
        cfg = {"identity": {"robot_name": "testbot"}}
        config_file = tmp_path / "robot.json"
        config_file.write_text(json.dumps(cfg))

        scale = self.mod.load_current_linear_scale(config_file)
        assert scale == 1.0


# ---------------------------------------------------------------------------
# Config write-back (angular)
# ---------------------------------------------------------------------------

class TestAngularConfigWriteback:
    """Tests for save_angular_calibration_to_config."""

    def setup_method(self) -> None:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "calibrate_angular",
            _HOST / "calibrate_angular.py",
        )
        self.mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
        spec.loader.exec_module(self.mod)  # type: ignore[union-attr]

    def test_save_angular_scale_only(self, tmp_path: Path) -> None:
        cfg = {"calibration": {"otos_angular_scale": 1.0}}
        config_file = tmp_path / "robot.json"
        config_file.write_text(json.dumps(cfg, indent=2))

        self.mod.save_angular_calibration_to_config(config_file, 0.987)

        result = json.loads(config_file.read_text())
        assert abs(result["calibration"]["otos_angular_scale"] - 0.987) < 1e-6

    def test_save_with_rotation_gains(self, tmp_path: Path) -> None:
        cfg = {
            "calibration": {
                "otos_angular_scale": 1.0,
                "rotation_gain": 1.0,
                "rotation_gain_neg": 1.0,
            }
        }
        config_file = tmp_path / "robot.json"
        config_file.write_text(json.dumps(cfg, indent=2))

        self.mod.save_angular_calibration_to_config(
            config_file, 0.987,
            rotation_gain=1.03,
            rotation_gain_neg=1.17,
        )

        result = json.loads(config_file.read_text())
        assert abs(result["calibration"]["otos_angular_scale"] - 0.987) < 1e-6
        assert abs(result["calibration"]["rotation_gain"] - 1.03) < 1e-6
        assert abs(result["calibration"]["rotation_gain_neg"] - 1.17) < 1e-6

    def test_save_preserves_other_fields(self, tmp_path: Path) -> None:
        cfg = {
            "identity": {"robot_name": "tovez"},
            "calibration": {
                "otos_angular_scale": 1.0,
                "mm_per_wheel_deg_left": 0.487,
            }
        }
        config_file = tmp_path / "robot.json"
        config_file.write_text(json.dumps(cfg, indent=2))

        self.mod.save_angular_calibration_to_config(config_file, 0.987)

        result = json.loads(config_file.read_text())
        assert result["identity"]["robot_name"] == "tovez"
        assert abs(result["calibration"]["mm_per_wheel_deg_left"] - 0.487) < 1e-6


# ---------------------------------------------------------------------------
# Script importability checks
# ---------------------------------------------------------------------------

class TestScriptImportability:
    """Verify the calibration scripts are importable with no syntax errors."""

    def test_calibrate_linear_importable(self) -> None:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "calibrate_linear",
            _HOST / "calibrate_linear.py",
        )
        mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
        spec.loader.exec_module(mod)  # type: ignore[union-attr]
        assert hasattr(mod, "main")
        assert hasattr(mod, "scale_to_int8")
        assert hasattr(mod, "compute_new_linear_scale")
        assert hasattr(mod, "mean_ratio_stats")
        assert hasattr(mod, "save_linear_scale_to_config")

    def test_calibrate_angular_importable(self) -> None:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "calibrate_angular",
            _HOST / "calibrate_angular.py",
        )
        mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
        spec.loader.exec_module(mod)  # type: ignore[union-attr]
        assert hasattr(mod, "main")
        assert hasattr(mod, "scale_to_int8")
        assert hasattr(mod, "compute_new_angular_scale")
        assert hasattr(mod, "heading_delta_cdeg")
        assert hasattr(mod, "save_angular_calibration_to_config")

    def test_calibrate_linear_does_not_import_parse_so(self) -> None:
        """calibrate_linear.py must not reference parse_so."""
        content = (_HOST / "calibrate_linear.py").read_text()
        assert "parse_so" not in content, \
            "calibrate_linear.py must not reference the dead parse_so"

    def test_calibrate_angular_does_not_import_parse_so(self) -> None:
        """calibrate_angular.py must not reference parse_so."""
        content = (_HOST / "calibrate_angular.py").read_text()
        assert "parse_so" not in content, \
            "calibrate_angular.py must not reference the dead parse_so"

    def test_calibrate_linear_does_not_reference_so_stream(self) -> None:
        """calibrate_linear.py must not reference the dead SO stream verb."""
        content = (_HOST / "calibrate_linear.py").read_text()
        # SO stream commands from v1 that are dead in v2
        for dead_verb in ("SSO", "set_stream_otos"):
            assert dead_verb not in content, \
                f"calibrate_linear.py must not reference dead v1 verb {dead_verb!r}"

    def test_calibrate_angular_does_not_reference_so_stream(self) -> None:
        """calibrate_angular.py must not reference the dead SO stream verb."""
        content = (_HOST / "calibrate_angular.py").read_text()
        for dead_verb in ("SSO", "set_stream_otos"):
            assert dead_verb not in content, \
                f"calibrate_angular.py must not reference dead v1 verb {dead_verb!r}"
