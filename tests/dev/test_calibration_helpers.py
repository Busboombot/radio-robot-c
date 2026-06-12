"""Unit tests for robot_radio.calibration.helpers.

All tests are pure Python — no hardware required.

Covers:
  - scale_to_int8 / int8_to_scale encoding
  - mean_stdev statistics
  - deep_merge dict merging
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from unittest.mock import patch

import pytest

from robot_radio.calibration.helpers import (
    deep_merge,
    int8_to_scale,
    mean_stdev,
    resolve_save_path,
    save_config,
    scale_to_int8,
)


# ---------------------------------------------------------------------------
# scale_to_int8 / int8_to_scale
# ---------------------------------------------------------------------------

class TestScaleToInt8:
    """Ticket 028-002 acceptance criteria cases."""

    def test_scale_1_027_gives_27(self):
        assert scale_to_int8(1.027) == 27

    def test_scale_1_0_gives_0(self):
        assert scale_to_int8(1.0) == 0

    def test_scale_0_973_gives_neg27(self):
        assert scale_to_int8(0.973) == -27

    def test_clamp_high(self):
        assert scale_to_int8(2.0) == 127

    def test_clamp_low(self):
        assert scale_to_int8(0.0) == -128

    def test_round_trip_27(self):
        """Round-trip via int8_to_scale should recover 1.027."""
        n = scale_to_int8(1.027)
        assert n == 27
        recovered = int8_to_scale(n)
        assert abs(recovered - 1.027) < 1e-9


class TestInt8ToScale:
    """Ticket 028-002 acceptance criteria cases."""

    def test_27_gives_1_027(self):
        assert abs(int8_to_scale(27) - 1.027) < 1e-9

    def test_0_gives_1_0(self):
        assert int8_to_scale(0) == 1.0

    def test_neg27_gives_0_973(self):
        assert abs(int8_to_scale(-27) - 0.973) < 1e-9

    def test_127_gives_1_127(self):
        assert abs(int8_to_scale(127) - 1.127) < 1e-9

    def test_neg128_gives_0_872(self):
        assert abs(int8_to_scale(-128) - 0.872) < 1e-9


# ---------------------------------------------------------------------------
# mean_stdev
# ---------------------------------------------------------------------------

class TestMeanStdev:
    """Ticket 028-002 acceptance criteria cases."""

    def test_three_values(self):
        m, s = mean_stdev([1.0, 2.0, 3.0])
        assert abs(m - 2.0) < 1e-9
        assert abs(s - 1.0) < 1e-9

    def test_single_value(self):
        m, s = mean_stdev([5.0])
        assert m == 5.0
        assert s == 0.0

    def test_empty(self):
        m, s = mean_stdev([])
        assert m == 0.0
        assert s == 0.0

    def test_two_values(self):
        m, s = mean_stdev([0.0, 2.0])
        assert abs(m - 1.0) < 1e-9
        # Bessel: stdev([0,2]) = sqrt(((0-1)^2 + (2-1)^2) / 1) = sqrt(2)
        assert abs(s - math.sqrt(2)) < 1e-9

    def test_identical_values(self):
        m, s = mean_stdev([3.0, 3.0, 3.0])
        assert m == 3.0
        assert s == 0.0


# ---------------------------------------------------------------------------
# deep_merge
# ---------------------------------------------------------------------------

class TestDeepMerge:
    """Ticket 028-002 acceptance criteria cases."""

    def test_nested_merge_does_not_overwrite_sibling(self):
        """deep_merge({'a': {'b': 1}}, {'a': {'c': 2}}) merges without losing b."""
        dst = {"a": {"b": 1}}
        src = {"a": {"c": 2}}
        deep_merge(dst, src)
        assert dst == {"a": {"b": 1, "c": 2}}

    def test_top_level_overwrite(self):
        dst = {"x": 1}
        deep_merge(dst, {"x": 99})
        assert dst["x"] == 99

    def test_adds_new_keys(self):
        dst = {"a": 1}
        deep_merge(dst, {"b": 2})
        assert dst == {"a": 1, "b": 2}

    def test_deeply_nested(self):
        dst = {"a": {"b": {"c": 1}}}
        deep_merge(dst, {"a": {"b": {"d": 2}}})
        assert dst == {"a": {"b": {"c": 1, "d": 2}}}

    def test_scalar_replaces_scalar(self):
        dst = {"a": {"b": 1}}
        deep_merge(dst, {"a": {"b": 99}})
        assert dst["a"]["b"] == 99


# ---------------------------------------------------------------------------
# save_config / resolve_save_path
# ---------------------------------------------------------------------------

class TestSaveConfig:

    def test_save_config_deep_merges(self, tmp_path: Path):
        cfg = {"identity": {"robot_name": "tovez"}, "calibration": {"otos_linear_scale": 1.0}}
        p = tmp_path / "robot.json"
        p.write_text(json.dumps(cfg))
        save_config(p, {"calibration": {"otos_linear_scale": 1.027}})
        result = json.loads(p.read_text())
        assert abs(result["calibration"]["otos_linear_scale"] - 1.027) < 1e-9
        assert result["identity"]["robot_name"] == "tovez"

    def test_save_config_preserves_other_sections(self, tmp_path: Path):
        cfg = {"identity": {"robot_name": "tovez"}, "geometry": {"trackwidth": 126}}
        p = tmp_path / "robot.json"
        p.write_text(json.dumps(cfg))
        save_config(p, {"calibration": {"otos_linear_scale": 1.05}})
        result = json.loads(p.read_text())
        assert result["geometry"]["trackwidth"] == 126


class TestResolveSavePath:

    def test_returns_none_when_no_active_robot(self, tmp_path: Path):
        # project_root with no data/robots/active_robot.json
        result = resolve_save_path(project_root=tmp_path)
        assert result is None

    def test_env_var_overrides(self, tmp_path: Path, monkeypatch):
        cfg_file = tmp_path / "my_robot.json"
        cfg_file.write_text("{}")
        monkeypatch.setenv("ROBOT_CONFIG", str(cfg_file))
        result = resolve_save_path(project_root=tmp_path)
        assert result == cfg_file

    def test_follows_active_robot_pointer(self, tmp_path: Path, monkeypatch):
        monkeypatch.delenv("ROBOT_CONFIG", raising=False)
        robots_dir = tmp_path / "data" / "robots"
        robots_dir.mkdir(parents=True)
        robot_cfg = robots_dir / "tovez.json"
        robot_cfg.write_text("{}")
        active = robots_dir / "active_robot.json"
        active.write_text(json.dumps({"path": "data/robots/tovez.json"}))
        result = resolve_save_path(project_root=tmp_path)
        assert result == tmp_path / "data" / "robots" / "tovez.json"
