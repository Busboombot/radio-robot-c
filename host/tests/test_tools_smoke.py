"""test_tools_smoke.py — smoke tests for tests/tools/velocity_chart.py and
tests/tools/playfield_tour.py (ticket 037-003).

All tests run headless against ``make_target("sim")`` — no hardware, no
camera, no matplotlib display required.

Tests:
  1. test_velocity_chart_importable          — import with no display.
  2. test_velocity_chart_arg_parsing         — arg parser covers --target/--real-time/--full-speed.
  3. test_velocity_chart_sim_headless        — headless sim run collects rows.
  4. test_playfield_tour_importable          — import with no camera/display.
  5. test_playfield_tour_arg_parsing         — arg parser covers --target/--pose/--real-time.
  6. test_playfield_tour_load_waypoints_fallback — fallback waypoints when no playfield.json.
  7. test_playfield_tour_sim_smoke           — sim run drives at least one hop.
  8. test_playfield_tour_compute_robot_relative — world→robot math is correct.
"""

from __future__ import annotations

import importlib
import math
import sys
import pathlib

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _repo_root() -> pathlib.Path:
    """Return the repository root (three levels up from host/tests/)."""
    return pathlib.Path(__file__).resolve().parent.parent.parent


def _import_tool(tool_name: str):
    """Import tests/tools/<tool_name>.py by file path (avoids sys.path conflicts)."""
    import importlib.util

    repo = _repo_root()
    tool_path = repo / "tests" / "tools" / f"{tool_name}.py"
    if not tool_path.exists():
        raise ImportError(f"Tool not found: {tool_path}")

    module_key = f"_test_tools_{tool_name}"
    if module_key in sys.modules:
        return sys.modules[module_key]

    spec = importlib.util.spec_from_file_location(module_key, tool_path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[module_key] = mod
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# velocity_chart smoke tests
# ---------------------------------------------------------------------------

class TestVelocityChartImport:
    """velocity_chart imports without matplotlib or camera."""

    def test_velocity_chart_importable(self) -> None:
        """import tests.tools.velocity_chart works with no display."""
        mod = _import_tool("velocity_chart")
        assert hasattr(mod, "main")
        assert hasattr(mod, "_parse_args")

    def test_velocity_chart_no_matplotlib_on_import(self) -> None:
        """Importing velocity_chart must not trigger a matplotlib import."""
        # Remove cached module so we get a fresh import.
        for key in list(sys.modules):
            if "_test_tools_velocity_chart" in key:
                del sys.modules[key]

        mpl_before = "matplotlib" in sys.modules
        _import_tool("velocity_chart")
        if not mpl_before:
            # matplotlib must not have been imported by the module-level import.
            # (It may have been imported by a previous test in the session —
            # only assert when it was absent before.)
            assert "matplotlib" not in sys.modules, (
                "velocity_chart imported matplotlib eagerly at module level"
            )


class TestVelocityChartArgParsing:
    """_parse_args covers the documented CLI flags."""

    def test_default_target_is_sim(self) -> None:
        mod = _import_tool("velocity_chart")
        args = mod._parse_args([])
        assert args.target == "sim"

    def test_target_bench(self) -> None:
        mod = _import_tool("velocity_chart")
        args = mod._parse_args(["--target", "bench"])
        assert args.target == "bench"

    def test_real_time_flag(self) -> None:
        mod = _import_tool("velocity_chart")
        args = mod._parse_args(["--real-time"])
        assert args.real_time is True

    def test_full_speed_flag(self) -> None:
        mod = _import_tool("velocity_chart")
        args = mod._parse_args(["--full-speed"])
        assert args.real_time is False

    def test_headless_flag(self) -> None:
        mod = _import_tool("velocity_chart")
        args = mod._parse_args(["--headless"])
        assert args.headless is True

    def test_duration_and_speed(self) -> None:
        mod = _import_tool("velocity_chart")
        args = mod._parse_args(["--duration", "5", "--speed", "300"])
        assert args.duration == pytest.approx(5.0)
        assert args.speed == 300


class TestVelocityChartSimRun:
    """Headless sim run completes without error."""

    def test_velocity_chart_sim_headless_short(self, tmp_path) -> None:
        """main(--target sim --full-speed --headless --duration 2) exits 0."""
        mod = _import_tool("velocity_chart")
        csv_path = str(tmp_path / "vc_test.csv")
        rc = mod.main([
            "--target", "sim",
            "--full-speed",
            "--headless",
            "--duration", "2",
            "--speed", "200",
            "--csv", csv_path,
        ])
        assert rc == 0, f"velocity_chart.main returned non-zero: {rc}"

    def test_velocity_chart_sim_collects_rows(self, tmp_path) -> None:
        """Headless run accumulates at least a few dashboard rows."""
        mod = _import_tool("velocity_chart")
        csv_path = str(tmp_path / "vc_rows.csv")
        mod.main([
            "--target", "sim",
            "--full-speed",
            "--headless",
            "--duration", "1",
            "--speed", "150",
            "--csv", csv_path,
        ])
        # CSV must exist and have at least one data row.
        import csv as csv_mod
        with open(csv_path) as fh:
            rows = list(csv_mod.DictReader(fh))
        assert len(rows) >= 1, "Expected at least 1 row in velocity_chart CSV output"
        assert "vL" in rows[0], "Expected 'vL' column in CSV"
        assert "vR" in rows[0], "Expected 'vR' column in CSV"


# ---------------------------------------------------------------------------
# playfield_tour smoke tests
# ---------------------------------------------------------------------------

class TestPlayfieldTourImport:
    """playfield_tour imports without camera or matplotlib."""

    def test_playfield_tour_importable(self) -> None:
        """import tests.tools.playfield_tour works with no camera/display."""
        mod = _import_tool("playfield_tour")
        assert hasattr(mod, "main")
        assert hasattr(mod, "_parse_args")
        assert hasattr(mod, "_load_waypoints")
        assert hasattr(mod, "_compute_robot_relative")

    def test_playfield_tour_no_aprilcam_on_import(self) -> None:
        """Importing playfield_tour must not trigger an aprilcam import."""
        for key in list(sys.modules):
            if "_test_tools_playfield_tour" in key:
                del sys.modules[key]

        cam_before = "aprilcam" in sys.modules
        _import_tool("playfield_tour")
        if not cam_before:
            assert "aprilcam" not in sys.modules, (
                "playfield_tour imported aprilcam eagerly at module level"
            )


class TestPlayfieldTourArgParsing:
    """_parse_args covers --target, --pose, --real-time."""

    def test_default_target_is_sim(self) -> None:
        mod = _import_tool("playfield_tour")
        args = mod._parse_args([])
        assert args.target == "sim"

    def test_target_bench(self) -> None:
        mod = _import_tool("playfield_tour")
        args = mod._parse_args(["--target", "bench"])
        assert args.target == "bench"

    def test_pose_auto_default(self) -> None:
        mod = _import_tool("playfield_tour")
        args = mod._parse_args([])
        assert args.pose == "auto"

    def test_pose_camera(self) -> None:
        mod = _import_tool("playfield_tour")
        args = mod._parse_args(["--pose", "camera"])
        assert args.pose == "camera"

    def test_pose_firmware(self) -> None:
        mod = _import_tool("playfield_tour")
        args = mod._parse_args(["--pose", "firmware"])
        assert args.pose == "firmware"

    def test_real_time_flag(self) -> None:
        mod = _import_tool("playfield_tour")
        args = mod._parse_args(["--real-time"])
        assert args.real_time is True

    def test_full_speed_flag(self) -> None:
        mod = _import_tool("playfield_tour")
        args = mod._parse_args(["--full-speed"])
        assert args.real_time is False

    def test_hops_and_speed(self) -> None:
        mod = _import_tool("playfield_tour")
        args = mod._parse_args(["--hops", "3", "--speed", "200"])
        assert args.hops == 3
        assert args.speed == 200


class TestPlayfieldTourWaypoints:
    """_load_waypoints returns fallback waypoints when no playfield.json."""

    def test_fallback_when_no_file(self, tmp_path) -> None:
        """_load_waypoints with a non-existent path returns fallback list."""
        mod = _import_tool("playfield_tour")
        waypoints = mod._load_waypoints(str(tmp_path / "nonexistent.json"))
        assert len(waypoints) >= 4, "Fallback should have at least 4 waypoints"
        # Each entry is (slug, x_cm, y_cm).
        for slug, x, y in waypoints:
            assert isinstance(slug, str)
            assert isinstance(x, float)
            assert isinstance(y, float)

    def test_loads_from_json(self, tmp_path) -> None:
        """_load_waypoints parses a minimal playfield.json correctly."""
        import json as json_mod

        data = {
            "rectangles": [
                {"slug": "red-N", "x": 10.0, "y": 20.0},
                {"slug": "blue-S", "x": -10.0, "y": -20.0},
            ]
        }
        pf = tmp_path / "playfield.json"
        pf.write_text(json_mod.dumps(data))

        mod = _import_tool("playfield_tour")
        waypoints = mod._load_waypoints(str(pf))
        assert len(waypoints) == 2
        assert waypoints[0] == ("red-N", 10.0, 20.0)
        assert waypoints[1] == ("blue-S", -10.0, -20.0)


class TestPlayfieldTourMath:
    """_compute_robot_relative: world → robot-relative (fwd_mm, left_mm)."""

    def test_forward_only(self) -> None:
        """Robot facing east (yaw=0), target due east: fwd>0, lft≈0."""
        mod = _import_tool("playfield_tour")
        # Robot at origin facing east (yaw=0), target at (10cm, 0).
        fwd, lft = mod._compute_robot_relative(0.0, 0.0, 0.0, 10.0, 0.0)
        assert fwd == pytest.approx(100.0, abs=1e-3), f"fwd={fwd}, expected 100mm"
        assert abs(lft) < 1e-3, f"lft={lft}, expected ~0"

    def test_target_to_left(self) -> None:
        """Robot facing east, target directly north: negated lateral → lft<0."""
        mod = _import_tool("playfield_tour")
        # Target is 10cm north (y+), robot facing east (yaw=0).
        # Standard math: lft = +100mm (left), but the convention negates it.
        # compute_g from playfield_random_tour: lft = dx*sin(H) - dy*cos(H)
        # dx=0, dy=10 → lft = 0*sin(0) - 10*cos(0) = -10 → -100mm.
        fwd, lft = mod._compute_robot_relative(0.0, 0.0, 0.0, 0.0, 10.0)
        assert abs(fwd) < 1e-3, f"fwd={fwd}, expected ~0"
        assert lft == pytest.approx(-100.0, abs=1e-3), f"lft={lft}, expected -100mm"

    def test_facing_north(self) -> None:
        """Robot facing north (yaw=pi/2), target directly north: fwd>0, lft≈0."""
        mod = _import_tool("playfield_tour")
        # Robot at (0,0) facing north, target at (0,10cm).
        fwd, lft = mod._compute_robot_relative(0.0, 0.0, math.pi / 2, 0.0, 10.0)
        assert fwd == pytest.approx(100.0, abs=0.1), f"fwd={fwd}, expected 100mm"
        assert abs(lft) < 0.1, f"lft={lft}, expected ~0"


class TestPlayfieldTourSimSmoke:
    """Sim run drives at least one hop without hardware."""

    def test_playfield_tour_sim_smoke(self, tmp_path) -> None:
        """main(--target sim --full-speed --hops 2) completes without error."""
        import json as json_mod

        # Write a simple two-waypoint playfield.json in a temp dir so the tool
        # doesn't rely on the real playfield.json being present.
        pf_data = {
            "rectangles": [
                {"slug": "north", "x": 0.0, "y": 30.0},
                {"slug": "south", "x": 0.0, "y": -30.0},
            ]
        }
        pf_json = tmp_path / "playfield.json"
        pf_json.write_text(json_mod.dumps(pf_data))

        mod = _import_tool("playfield_tour")
        rc = mod.main([
            "--target", "sim",
            "--full-speed",
            "--hops", "2",
            "--speed", "200",
            "--playfield-json", str(pf_json),
        ])
        assert rc == 0, f"playfield_tour.main returned non-zero: {rc}"

    def test_playfield_tour_sim_multi_hop(self, tmp_path) -> None:
        """Sim run with 4 hops from 4-waypoint playfield completes without error."""
        import json as json_mod

        pf_data = {
            "rectangles": [
                {"slug": "N",  "x":  0.0, "y":  25.0},
                {"slug": "E",  "x": 25.0, "y":   0.0},
                {"slug": "S",  "x":  0.0, "y": -25.0},
                {"slug": "W",  "x": -25.0, "y":  0.0},
            ]
        }
        pf_json = tmp_path / "playfield4.json"
        pf_json.write_text(json_mod.dumps(pf_data))

        mod = _import_tool("playfield_tour")
        rc = mod.main([
            "--target", "sim",
            "--full-speed",
            "--hops", "4",
            "--speed", "200",
            "--playfield-json", str(pf_json),
        ])
        assert rc == 0, f"playfield_tour.main returned non-zero: {rc}"
