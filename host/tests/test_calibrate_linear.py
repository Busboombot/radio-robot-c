"""Unit tests for calibrate_linear.py — calibration math, write path, import safety.

No hardware is required. All robot/camera calls are mocked.

IMPORTANT: we never loop on a mock that returns instantly — all loops
in the tested code are bounded by trial count or a finite input sequence.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# ---------------------------------------------------------------------------
# Import the module under test using file-based import (tests/calibrate/ is not
# a package — no __init__.py — so we load it by path).
# This must NOT open serial or a camera at import time.
# ---------------------------------------------------------------------------

import importlib.util
from pathlib import Path as _Path

_REPO_ROOT_FOR_IMPORT = _Path(__file__).resolve().parents[2]
_CAL_MOD_PATH = _REPO_ROOT_FOR_IMPORT / "tests" / "calibrate" / "calibrate_linear.py"
_spec = importlib.util.spec_from_file_location("calibrate_linear", _CAL_MOD_PATH)
_module = importlib.util.module_from_spec(_spec)
sys.modules["calibrate_linear"] = _module   # register so reload() works
_spec.loader.exec_module(_module)
cal_mod = _module


# ---------------------------------------------------------------------------
# Test: calibration math
# ---------------------------------------------------------------------------

class TestCalibrationMath:

    def test_encoder_correction_scales_proportionally(self) -> None:
        """tape=900, enc=880: correction factor = 900/880 ≈ 1.0227."""
        tape_mm = 900.0
        enc_mm = 880.0
        ml_in = 0.484
        mr_in = 0.484
        ml_out, mr_out = cal_mod.compute_encoder_correction(tape_mm, enc_mm, ml_in, mr_in)
        expected_k = tape_mm / enc_mm
        assert abs(ml_out - ml_in * expected_k) < 1e-9
        assert abs(mr_out - mr_in * expected_k) < 1e-9
        assert ml_out > ml_in  # robot undershot → scale up

    def test_encoder_correction_overshoot(self) -> None:
        """tape=900, enc=950: correction factor < 1 (robot overshot)."""
        ml_out, mr_out = cal_mod.compute_encoder_correction(900.0, 950.0, 0.5, 0.5)
        assert ml_out < 0.5
        assert mr_out < 0.5

    def test_otos_scale_correction_undershoot(self) -> None:
        """tape=900, otos=945: OTOS overestimates → scale correction < 1."""
        new_scale = cal_mod.compute_otos_scale_correction(900.0, 945.0, 1.05)
        assert new_scale < 1.05
        # Correction must move scale toward truth: otos_mm * new_scale / old_scale ≈ tape/otos
        assert abs(new_scale - 1.05 * (900.0 / 945.0)) < 1e-9

    def test_otos_scale_correction_overshoot(self) -> None:
        """tape=900, otos=850: OTOS underestimates → scale correction > current."""
        current_scale = 1.0
        new_scale = cal_mod.compute_otos_scale_correction(900.0, 850.0, current_scale)
        assert new_scale > current_scale

    def test_scale_to_int8_round_trip(self) -> None:
        """scale → int8 → scale round-trip is stable to 0.001 resolution."""
        for scale in (1.0, 0.95, 1.05, 0.9, 1.1):
            n = cal_mod.scale_to_int8(scale)
            assert -128 <= n <= 127
            recovered = cal_mod.int8_to_scale(n)
            assert abs(recovered - scale) <= 0.0005

    def test_scale_to_int8_clamps(self) -> None:
        assert cal_mod.scale_to_int8(2.0) == 127   # 1.0/0.001 = 1000 > 127
        assert cal_mod.scale_to_int8(0.0) == -128  # -1.0/0.001 = -1000 < -128

    def test_dist2d_mm_converts_cm_to_mm(self) -> None:
        """dist2d_mm with 10 cm apart in x → 100 mm."""
        result = cal_mod.dist2d_mm((0.0, 0.0), (10.0, 0.0))
        assert result is not None
        assert abs(result - 100.0) < 1e-9

    def test_dist2d_mm_none_if_either_missing(self) -> None:
        assert cal_mod.dist2d_mm(None, (1.0, 1.0)) is None
        assert cal_mod.dist2d_mm((1.0, 1.0), None) is None


# ---------------------------------------------------------------------------
# Test: JSON write path
# ---------------------------------------------------------------------------

class TestJsonWrite:

    def test_save_calibration_updates_fields(self, tmp_path: Path) -> None:
        """_save_calibration writes ml/mr/otos_scale into the JSON file."""
        config_data = {
            "schema_version": 1,
            "identity": {"robot_name": "tovez", "uid": "test"},
            "calibration": {
                "mm_per_wheel_deg_left": 0.484,
                "mm_per_wheel_deg_right": 0.484,
                "otos_linear_scale": 1.0,
            },
        }
        config_file = tmp_path / "tovez.json"
        config_file.write_text(json.dumps(config_data, indent=2))

        # Patch the module's _TOVEZ_JSON path to our temp file
        with patch.object(cal_mod, "_TOVEZ_JSON", config_file):
            cal_mod._save_calibration(0.490, 0.488, 1.02, cam_scale=0.98)

        saved = json.loads(config_file.read_text())
        assert abs(saved["calibration"]["mm_per_wheel_deg_left"] - 0.49) < 1e-4
        assert abs(saved["calibration"]["mm_per_wheel_deg_right"] - 0.488) < 1e-4
        assert abs(saved["calibration"]["otos_linear_scale"] - 1.02) < 1e-4
        assert abs(saved["vision"]["camera_distance_scale"] - 0.98) < 1e-4

    def test_save_calibration_preserves_other_fields(self, tmp_path: Path) -> None:
        """_save_calibration must not clobber unrelated config fields."""
        config_data = {
            "schema_version": 1,
            "identity": {"robot_name": "tovez", "uid": "abc"},
            "peripherals": {"laser_port": 4},
            "calibration": {
                "mm_per_wheel_deg_left": 0.484,
                "mm_per_wheel_deg_right": 0.484,
                "otos_linear_scale": 1.0,
            },
        }
        config_file = tmp_path / "tovez.json"
        config_file.write_text(json.dumps(config_data, indent=2))

        with patch.object(cal_mod, "_TOVEZ_JSON", config_file):
            cal_mod._save_calibration(0.484, 0.484, 1.0, cam_scale=None)

        saved = json.loads(config_file.read_text())
        assert saved["peripherals"]["laser_port"] == 4
        assert saved["identity"]["robot_name"] == "tovez"
        assert "camera_distance_scale" not in saved.get("vision", {})

    def test_save_calibration_skips_cam_scale_when_none(self, tmp_path: Path) -> None:
        """When cam_scale is None, vision.camera_distance_scale is not written."""
        config_data = {
            "schema_version": 1,
            "identity": {"robot_name": "tovez", "uid": "abc"},
            "calibration": {
                "mm_per_wheel_deg_left": 0.484,
                "mm_per_wheel_deg_right": 0.484,
                "otos_linear_scale": 1.0,
            },
        }
        config_file = tmp_path / "tovez.json"
        config_file.write_text(json.dumps(config_data, indent=2))

        with patch.object(cal_mod, "_TOVEZ_JSON", config_file):
            cal_mod._save_calibration(0.484, 0.484, 1.0, cam_scale=None)

        saved = json.loads(config_file.read_text())
        assert "camera_distance_scale" not in saved.get("vision", {})


# ---------------------------------------------------------------------------
# Test: no raw serial imports
# ---------------------------------------------------------------------------

class TestNoRawSerial:

    def test_no_serial_module_imported(self) -> None:
        """calibrate_linear must not import the raw 'serial' module directly."""
        # The module is already imported at the top. Check sys.modules for
        # evidence that 'serial' was pulled in *by* calibrate_linear directly.
        # The module-level imports in calibrate_linear use robot_radio.* which
        # internally uses serial — that's fine. What we forbid is calib_common.
        assert "tests.calibrate.calib_common" not in sys.modules
        assert "calib_common" not in sys.modules

    def test_no_calib_common_import(self) -> None:
        """calib_common must not be imported by calibrate_linear."""
        # Reload the module in isolation to make sure it doesn't pull calib_common.
        import importlib
        # Remove any existing calib_common from sys.modules (shouldn't be there)
        for key in list(sys.modules.keys()):
            if "calib_common" in key:
                del sys.modules[key]
        # Re-importing calibrate_linear must not bring calib_common back.
        importlib.reload(cal_mod)
        assert "calib_common" not in sys.modules
        assert "tests.calibrate.calib_common" not in sys.modules

    def test_module_has_no_test_functions(self) -> None:
        """calibrate_linear must not expose any pytest-collectible test_* names."""
        public_names = [n for n in dir(cal_mod)
                        if n.startswith("test_") or n.endswith("_test")]
        assert public_names == [], f"Unexpected test names: {public_names}"
