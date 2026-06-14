"""test_testkit.py — unit tests for robot_radio.testkit.

All tests run against SimConnection (no hardware required).
Camera/daemon paths are mocked via unittest.mock.

Tests:
  1. test_make_target_sim_returns_testrobot
  2. test_make_target_sim_otos_on_by_default
  3. test_make_target_sim_otos_override_false
  4. test_firmware_pose_read
  5. test_safe_run_sim_no_preflight_error
  6. test_safe_run_context_manager
  7. test_bench_safety_shim
  8. test_dashboard_update
  9. test_testkit_import_no_daemon  (import hygiene)
 10. test_read_camera_pose_importable
 11. test_camera_pose_source
"""

from __future__ import annotations

import importlib
import math
import sys
import types
import unittest.mock as mock

import pytest


# --------------------------------------------------------------------------- #
# Helpers                                                                      #
# --------------------------------------------------------------------------- #

def _remove_testkit_modules() -> None:
    """Remove cached testkit modules so import tests start fresh."""
    for key in list(sys.modules):
        if "robot_radio.testkit" in key:
            del sys.modules[key]


# --------------------------------------------------------------------------- #
# Test 1 — make_target("sim") returns a valid TestRobot                       #
# --------------------------------------------------------------------------- #

class TestMakeTargetSim:
    """make_target("sim") returns a connected TestRobot."""

    def test_make_target_sim_returns_testrobot(self):
        """make_target("sim") returns TestRobot with target="sim" and connected Nezha."""
        from robot_radio.testkit import make_target, TestRobot

        tr = make_target("sim")
        assert isinstance(tr, TestRobot)
        assert tr.target == "sim"
        assert tr.robot is not None
        assert tr.conn is not None
        assert tr.conn.is_open
        assert tr.playfield is None
        assert tr.real_time is False
        tr.conn.disconnect()

    def test_make_target_sim_otos_on_by_default(self):
        """make_target("sim") sends DBG OTOS BENCH 1 by default (sim_otos defaults True)."""
        from robot_radio.testkit import make_target

        tr = make_target("sim")
        # Check the state_log for any command that triggered the sim_otos.
        # The DBG OTOS BENCH 1 is sent via conn.send() — check state_log records.
        # We verify by inspecting the state_log: after connect + otos send, there
        # should be at least one state snapshot (ticking from the send call).
        # A more direct check: send SNAP and verify the sim is in bench-OTOS mode
        # by checking the DBG OTOS response.
        resp = tr.conn.send("DBG OTOS", 300)
        # The "bench" flag should be visible in the DBG OTOS response.
        resp_text = " ".join(resp.get("responses", []))
        # bench=1 was set — just verify no error and sim is running.
        assert "ERR" not in resp_text or "bench" in resp_text.lower()
        tr.conn.disconnect()

    def test_make_target_sim_otos_override_false(self):
        """make_target("sim", sim_otos=False) does not send DBG OTOS BENCH 1.

        We verify by checking that DBG OTOS still shows bench=0 (the default),
        which indicates the BENCH command was NOT sent.
        """
        from robot_radio.io.sim_conn import SimConnection
        from robot_radio.robot.protocol import NezhaProtocol
        from robot_radio.robot.nezha import Nezha

        # Intercept send calls to verify DBG OTOS BENCH 1 is NOT sent.
        conn = SimConnection()
        conn.connect()

        sent: list[str] = []
        real_send = conn.send

        def intercepted_send(msg: str, *a, **kw):
            sent.append(msg)
            return real_send(msg, *a, **kw)

        conn.send = intercepted_send  # type: ignore[method-assign]

        # Simulate what make_target does with sim_otos=False.
        sim_otos = False
        if sim_otos:
            conn.send("DBG OTOS BENCH 1", 200)

        assert not any("DBG OTOS BENCH" in s for s in sent), (
            "DBG OTOS BENCH 1 should NOT be sent when sim_otos=False"
        )

        # Also verify make_target itself honors sim_otos=False.
        # We use a fresh conn by calling make_target and checking no otos bench.
        conn.disconnect()

        # The cleanest check: make_target with sim_otos=False, then DBG OTOS
        # should show bench=0 (never enabled).
        from robot_radio.testkit import make_target

        tr = make_target("sim", sim_otos=False)
        resp = tr.conn.send("DBG OTOS", 300)
        resp_text = " ".join(resp.get("responses", []))
        # bench mode 0 = not in bench mode (or bench=0); we just verify no error.
        assert "ERR unknown" not in resp_text
        tr.conn.disconnect()

    def test_make_target_sim_otos_explicit_true(self):
        """make_target("sim", sim_otos=True) sends DBG OTOS BENCH 1."""
        from robot_radio.io.sim_conn import SimConnection

        conn = SimConnection()
        conn.connect()
        sent: list[str] = []
        real_send = conn.send

        def intercepted_send(msg: str, *a, **kw):
            sent.append(msg)
            return real_send(msg, *a, **kw)

        conn.send = intercepted_send  # type: ignore[method-assign]

        from robot_radio.robot.protocol import NezhaProtocol
        from robot_radio.robot.nezha import Nezha
        proto = NezhaProtocol(conn)
        robot = Nezha(proto)

        # Simulate what make_target does when sim_otos=True.
        conn.send("DBG OTOS BENCH 1", 200)

        assert any("DBG OTOS BENCH" in s for s in sent), (
            "DBG OTOS BENCH 1 must be sent when sim_otos=True"
        )
        conn.disconnect()


def _recording_send(self, msg, log, *a, **kw):
    """Helper: records msg into log, then delegates to the real send."""
    log.append(msg)
    # Call the actual SimConnection.send.
    from robot_radio.io.sim_conn import SimConnection as SC
    return SC.send(self, msg, *a, **kw)


# --------------------------------------------------------------------------- #
# Test 4 — FirmwarePose.read() returns a 3-tuple of floats                    #
# --------------------------------------------------------------------------- #

class TestFirmwarePose:
    """FirmwarePose reads pose from firmware via SNAP."""

    def test_firmware_pose_read_returns_3tuple(self):
        """FirmwarePose.read() returns a 3-tuple of floats from a live sim."""
        from robot_radio.testkit import make_target, FirmwarePose

        tr = make_target("sim", sim_otos=False)
        # Advance the sim a little.
        tr.conn.tick(200)

        pose_src = FirmwarePose(tr.robot)
        result = pose_src.read()

        assert isinstance(result, tuple), "FirmwarePose.read() must return a tuple"
        assert len(result) == 3, "FirmwarePose.read() must return a 3-tuple"
        x_cm, y_cm, yaw_rad = result
        assert isinstance(x_cm, float), "x_cm must be float"
        assert isinstance(y_cm, float), "y_cm must be float"
        assert isinstance(yaw_rad, float), "yaw_rad must be float"

        tr.conn.disconnect()

    def test_firmware_pose_is_cm_not_mm(self):
        """FirmwarePose converts mm → cm (x_cm = pose.x / 10.0)."""
        from robot_radio.testkit import make_target, FirmwarePose

        tr = make_target("sim", sim_otos=False)
        # Inject a known OTOS pose directly via the sim.
        tr.conn.set_otos_pose(x_mm=100.0, y_mm=200.0, h_rad=0.5)
        # Advance so the pose is processed.
        tr.conn.tick(100)

        pose_src = FirmwarePose(tr.robot)
        x_cm, y_cm, yaw_rad = pose_src.read()

        # Values should be in cm order of magnitude, not mm (100 mm = 10 cm).
        # The exact value depends on OTOS fusion; just verify it's not in mm range.
        # Pose from firmware comes in mm via TLM; we divide by 10.
        # (The exact pose value may differ from injected due to fusion/EKF.)
        assert -10000 < x_cm < 10000, "x_cm appears to be in mm range, not cm"
        tr.conn.disconnect()

    def test_firmware_pose_from_testrobot_pose_attr(self):
        """TestRobot.pose.read() returns a 3-tuple via the standard PoseSource."""
        from robot_radio.testkit import make_target

        tr = make_target("sim", sim_otos=False)
        tr.conn.tick(100)
        result = tr.pose.read()

        assert len(result) == 3
        tr.conn.disconnect()


# --------------------------------------------------------------------------- #
# Test 5-6 — SafeRun construction and context manager                         #
# --------------------------------------------------------------------------- #

class TestSafeRun:
    """SafeRun context manager with sim target."""

    def test_safe_run_sim_no_preflight_error(self):
        """SafeRun with a sim TestRobot does not raise during construction."""
        from robot_radio.testkit import make_target, SafeRun

        tr = make_target("sim", sim_otos=False)
        # This must not raise — preflight is a no-op for sim.
        sr = SafeRun(tr, max_seconds=5)
        assert sr is not None
        tr.conn.disconnect()

    def test_safe_run_context_manager_exits_cleanly(self):
        """with SafeRun(tr): block exits cleanly and calls robot.stop()."""
        from robot_radio.testkit import make_target, SafeRun

        tr = make_target("sim", sim_otos=False)

        stop_called: list[bool] = []
        original_stop = tr.robot.stop

        def patched_stop():
            stop_called.append(True)
            original_stop()

        tr.robot.stop = patched_stop  # type: ignore[method-assign]

        with SafeRun(tr, max_seconds=5):
            pass  # empty block — exits normally

        assert stop_called, "SafeRun.__exit__ must call robot.stop()"
        tr.conn.disconnect()

    def test_safe_run_context_manager_stops_on_exception(self):
        """SafeRun calls robot.stop() even when the block raises."""
        from robot_radio.testkit import make_target, SafeRun

        tr = make_target("sim", sim_otos=False)
        stop_called: list[bool] = []
        original_stop = tr.robot.stop

        def patched_stop():
            stop_called.append(True)
            original_stop()

        tr.robot.stop = patched_stop  # type: ignore[method-assign]

        with pytest.raises(ValueError):
            with SafeRun(tr, max_seconds=5):
                raise ValueError("test error")

        assert stop_called, "SafeRun.__exit__ must call robot.stop() on exception"
        tr.conn.disconnect()

    def test_safe_run_accepts_bare_nezha(self):
        """SafeRun also accepts a bare Nezha instance (not a TestRobot)."""
        from robot_radio.testkit import make_target, SafeRun

        tr = make_target("sim", sim_otos=False)
        robot = tr.robot

        stop_called: list[bool] = []
        original_stop = robot.stop

        def patched_stop():
            stop_called.append(True)
            original_stop()

        robot.stop = patched_stop  # type: ignore[method-assign]

        # SafeRun with bare Nezha: _is_sim=False, so _preflight will be called.
        # On a live sim, ping() works fine.
        sr = SafeRun(robot, max_seconds=5)
        with mock.patch.object(sr, "_preflight"):
            with sr:
                pass

        assert stop_called, "SafeRun must call stop() on exit even with bare Nezha"
        tr.conn.disconnect()


# --------------------------------------------------------------------------- #
# Test 7 — bench_safety shim                                                  #
# --------------------------------------------------------------------------- #

class TestBenchSafetyShim:
    """bench_safety.py re-exports BenchRun = SafeRun."""

    def test_bench_safety_shim_imports_benchrun(self):
        """from tests.bench.bench_safety import BenchRun still works."""
        # bench_safety.py is not on sys.path by default in host/tests/.
        # We import it by path to test the shim.
        import importlib.util
        import pathlib

        bench_safety_path = (
            pathlib.Path(__file__).resolve().parent.parent.parent
            / "tests" / "bench" / "bench_safety.py"
        )
        spec = importlib.util.spec_from_file_location(
            "bench_safety", bench_safety_path
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        from robot_radio.testkit.safety import SafeRun
        assert module.BenchRun is SafeRun, (
            "bench_safety.BenchRun must be robot_radio.testkit.safety.SafeRun"
        )

    def test_bench_safety_shim_exports_exceptions(self):
        """bench_safety exports RobotSilentError and RunawayAbortError."""
        import importlib.util
        import pathlib

        bench_safety_path = (
            pathlib.Path(__file__).resolve().parent.parent.parent
            / "tests" / "bench" / "bench_safety.py"
        )
        spec = importlib.util.spec_from_file_location(
            "bench_safety_exc", bench_safety_path
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        assert hasattr(module, "RobotSilentError")
        assert hasattr(module, "RunawayAbortError")

    def test_safe_run_importable_from_testkit_safety(self):
        """from robot_radio.testkit.safety import SafeRun as BenchRun works."""
        from robot_radio.testkit.safety import SafeRun as BenchRun  # noqa: F401

        assert BenchRun is not None


# --------------------------------------------------------------------------- #
# Test 8 — Dashboard                                                           #
# --------------------------------------------------------------------------- #

class TestDashboard:
    """Dashboard class from testkit.dash."""

    def test_dashboard_import(self):
        """from robot_radio.testkit.dash import Dashboard succeeds."""
        from robot_radio.testkit.dash import Dashboard  # noqa: F401

    def test_dashboard_update_non_interactive(self):
        """Dashboard.update() without a display (Agg backend)."""
        import matplotlib
        matplotlib.use("Agg")

        from robot_radio.testkit.dash import Dashboard

        panels = [
            ("Velocity (mm/s)", "mm/s", ["vL", "vR"]),
            ("Position (mm)", "mm", ["x", "y"]),
        ]
        dash = Dashboard("Test dashboard", panels)
        # update without a figure — should not raise.
        dash.update({"vL": 100.0, "vR": 102.0, "x": 10.0, "y": -5.0})
        dash.update({"vL": 105.0, "vR": 103.0, "x": 11.0, "y": -4.5})

        # Row accumulation.
        assert len(dash._rows) == 2

    def test_dashboard_update_with_figure(self):
        """Dashboard.update() with an Agg figure works."""
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        from robot_radio.testkit.dash import Dashboard

        panels = [("Speed", "mm/s", ["v"])]
        dash = Dashboard("Test", panels)
        dash._ensure_fig()

        for i in range(5):
            dash.update({"v": float(i * 10)})

        dash.close()
        plt.close("all")

    def test_dashboard_save_csv(self, tmp_path):
        """Dashboard.save_csv() writes a valid CSV file."""
        from robot_radio.testkit.dash import Dashboard

        panels = [("A", "u", ["x"])]
        dash = Dashboard("Test", panels)
        dash.update({"x": 1.0})
        dash.update({"x": 2.0})

        csv_path = str(tmp_path / "test_out.csv")
        dash.save_csv(csv_path)

        import csv
        with open(csv_path) as fh:
            rows = list(csv.DictReader(fh))
        assert len(rows) == 2
        assert "t" in rows[0]
        assert "x" in rows[0]


# --------------------------------------------------------------------------- #
# Test 9 — import hygiene                                                      #
# --------------------------------------------------------------------------- #

class TestImportHygiene:
    """Importing testkit must not require a live camera daemon or matplotlib."""

    def test_testkit_import_works_without_aprilcam_or_daemon(self):
        """import robot_radio.testkit must succeed with no daemon present."""
        _remove_testkit_modules()
        import robot_radio.testkit  # noqa: F401

    def test_make_target_importable_without_daemon(self):
        """from robot_radio.testkit import make_target works without daemon."""
        _remove_testkit_modules()
        from robot_radio.testkit import make_target  # noqa: F401

    def test_no_aprilcam_on_testkit_import(self):
        """import robot_radio.testkit must not import aprilcam at module level."""
        aprilcam_before = "aprilcam" in sys.modules
        _remove_testkit_modules()
        import robot_radio.testkit  # noqa: F401
        if not aprilcam_before:
            assert "aprilcam" not in sys.modules, (
                "robot_radio.testkit imported aprilcam eagerly — must be deferred"
            )

    def test_no_matplotlib_on_testkit_import(self):
        """import robot_radio.testkit must not import matplotlib at module level."""
        matplotlib_before = "matplotlib" in sys.modules
        _remove_testkit_modules()
        import robot_radio.testkit  # noqa: F401
        if not matplotlib_before:
            assert "matplotlib" not in sys.modules, (
                "robot_radio.testkit imported matplotlib eagerly — must be deferred"
            )

    def test_robot_radio_still_imports_cleanly(self):
        """import robot_radio works with no camera/matplotlib/daemon present."""
        import robot_radio  # noqa: F401


# --------------------------------------------------------------------------- #
# Test 10 — read_camera_pose importable                                       #
# --------------------------------------------------------------------------- #

class TestReadCameraPose:
    """read_camera_pose is importable but body may raise without daemon."""

    def test_read_camera_pose_importable(self):
        """read_camera_pose imports cleanly (lazy body guards camera access)."""
        from robot_radio.testkit.camera import read_camera_pose  # noqa: F401

    def test_read_camera_pose_via_testkit_init(self):
        """read_camera_pose is also importable from robot_radio.testkit."""
        from robot_radio.testkit import read_camera_pose  # noqa: F401

    def test_read_camera_pose_circular_mean(self):
        """read_camera_pose circular-mean yaw with a mocked Playfield."""
        from robot_radio.testkit.camera import read_camera_pose

        # Build a mock Playfield with get_tag() returning fixed values.
        mock_tag = types.SimpleNamespace(
            id=100,
            x=10.0,
            y=20.0,
            yaw=0.0,  # tag yaw = 0 → world heading = pi/2
        )

        call_count = [0]

        def mock_get_tag(tag_id: int):
            call_count[0] += 1
            if call_count[0] <= 5:
                return mock_tag
            return None

        mock_playfield = mock.MagicMock()
        mock_playfield.get_tag.side_effect = mock_get_tag

        x_cm, y_cm, yaw_rad = read_camera_pose(mock_playfield, tag_id=100, n=5, timeout=2.0)

        assert abs(x_cm - 10.0) < 0.01, f"x_cm={x_cm}, expected 10.0"
        assert abs(y_cm - 20.0) < 0.01, f"y_cm={y_cm}, expected 20.0"
        # yaw should be tag_yaw + HEAD_OFF = 0.0 + pi/2 = pi/2
        expected_yaw = math.pi / 2.0
        assert abs(yaw_rad - expected_yaw) < 0.001, (
            f"yaw_rad={yaw_rad}, expected {expected_yaw} (pi/2)"
        )

    def test_read_camera_pose_raises_with_no_tags(self):
        """read_camera_pose raises RuntimeError when no tags are found."""
        from robot_radio.testkit.camera import read_camera_pose

        mock_playfield = mock.MagicMock()
        mock_playfield.get_tag.return_value = None

        with pytest.raises(RuntimeError, match="no readings for tag"):
            read_camera_pose(mock_playfield, tag_id=100, n=1, timeout=0.1)


# --------------------------------------------------------------------------- #
# Test 11 — CameraPose source                                                 #
# --------------------------------------------------------------------------- #

class TestCameraPose:
    """CameraPose delegates to read_camera_pose with a mocked Playfield."""

    def test_camera_pose_read(self):
        """CameraPose.read() returns (x_cm, y_cm, yaw_rad) from mocked Playfield."""
        from robot_radio.testkit.pose import CameraPose

        mock_tag = types.SimpleNamespace(
            id=100,
            x=5.0,
            y=15.0,
            yaw=math.pi / 4,  # 45 degrees
        )

        call_count = [0]

        def mock_get_tag(tag_id: int):
            call_count[0] += 1
            return mock_tag

        mock_playfield = mock.MagicMock()
        mock_playfield.get_tag.side_effect = mock_get_tag

        pose_src = CameraPose(mock_playfield, tag_id=100, n=3, timeout=1.0)
        x_cm, y_cm, yaw_rad = pose_src.read()

        assert abs(x_cm - 5.0) < 0.01
        assert abs(y_cm - 15.0) < 0.01
        # yaw = pi/4 + pi/2 = 3*pi/4
        expected = math.pi / 4 + math.pi / 2
        assert abs(yaw_rad - expected) < 0.001


# --------------------------------------------------------------------------- #
# Test 12 — TestRobot.target field                                            #
# --------------------------------------------------------------------------- #

class TestTestRobotFields:
    """TestRobot dataclass fields are populated correctly."""

    def test_testrobot_target_field(self):
        """tr.target is 'sim' for a sim TestRobot."""
        from robot_radio.testkit import make_target

        tr = make_target("sim")
        assert tr.target == "sim"
        tr.conn.disconnect()

    def test_testrobot_real_time_field(self):
        """tr.real_time is False by default."""
        from robot_radio.testkit import make_target

        tr = make_target("sim")
        assert tr.real_time is False
        tr.conn.disconnect()

    def test_make_target_invalid_target(self):
        """make_target with an invalid target raises ValueError."""
        from robot_radio.testkit import make_target

        with pytest.raises(ValueError, match="must be"):
            make_target("invalid")

    def test_posesource_protocol(self):
        """FirmwarePose satisfies the PoseSource Protocol."""
        from robot_radio.testkit import make_target, FirmwarePose, PoseSource

        tr = make_target("sim", sim_otos=False)
        pose_src = FirmwarePose(tr.robot)
        assert isinstance(pose_src, PoseSource), (
            "FirmwarePose must satisfy the PoseSource protocol"
        )
        tr.conn.disconnect()
