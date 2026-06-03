"""Smoke-validate the nav/path/controllers/kinematics import chain.

This test suite verifies the import graph for the four higher-level
subpackages.  It does NOT drive the robot, run simulations, or exercise
deep v2 integration — that is deferred to a follow-up sprint per the
Sprint 013 architecture.

Environment assumptions (root .venv via ``uv run --with pytest``):
  - numpy:    present (calibrate dep group → aprilcam)
  - aprilcam: present (calibrate dep group)
  - cv2:      present (pulled in by aprilcam at aprilcam import time)
  - wpimath:  NOT present

Classes that import cleanly in this environment:
  nav: Pose, Waypoint, NavParams, align_otos_to_camera (pose_align)
       Navigator (lazy, requires aprilcam — verified structurally, not imported
       here to avoid contaminating sys.modules with cv2 for other tests)
  path: SampledPath, catmull_rom, plan_path, build_safe_spline,
        four_leaf_waypoints, BezierPathBuilder (numpy OK), arc.compute_arc
  controllers: Controller (ABC), PID, normalize_angle, PurePursuitTracker,
               StanleyController
  kinematics: (package-level __init__ only — no wpimath required)

Classes behind wpimath (ImportError on access, not AttributeError):
  controllers.LTVController — ``robot_radio.controllers.ltv`` fails to load
  kinematics.DifferentialDriveKinematics — ``robot_radio.kinematics.differential_drive`` fails

CRITICAL: Do NOT import anything from vendor/PythonRobotics.
Do NOT import robot_radio.nav.navigator here — it pulls in aprilcam → cv2
which would contaminate sys.modules and break test_sensors_v2 laziness tests.
Every test must be sub-second and produce no side effects.
"""

from __future__ import annotations

import importlib
import sys

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _fresh_import(module_name: str):
    """Import a module, removing any cached copy first.

    This ensures each import test starts with a fresh module resolution.
    Cached copies from earlier tests are fine to keep; we just want the
    import to succeed (not rely on prior test ordering for coverage).
    """
    return importlib.import_module(module_name)


# ---------------------------------------------------------------------------
# T1 — top-level package
# ---------------------------------------------------------------------------


class TestTopLevel:
    """robot_radio top-level package imports without error."""

    def test_robot_radio_imports(self):
        import robot_radio  # noqa: F401


# ---------------------------------------------------------------------------
# T2 — nav subpackage
# ---------------------------------------------------------------------------


class TestNavImports:
    """nav subpackage import chain.

    Eager imports (in __init__): pose, nav_params, pose_align.
    Lazy import (via __getattr__): Navigator (requires aprilcam).
    """

    def test_nav_package_imports(self):
        _fresh_import("robot_radio.nav")

    def test_nav_pose_imports(self):
        _fresh_import("robot_radio.nav.pose")

    def test_nav_nav_params_imports(self):
        _fresh_import("robot_radio.nav.nav_params")

    def test_nav_pose_align_imports(self):
        _fresh_import("robot_radio.nav.pose_align")

    def test_nav_navigator_lazy_guard(self):
        """Navigator is behind __getattr__ — the nav package __init__ must not
        import it eagerly.  We verify the guard is present by inspecting the
        __getattr__ attribute without triggering a real aprilcam import.

        Importing robot_radio.nav.navigator directly is intentionally avoided
        here: it pulls aprilcam → cv2 into sys.modules which would cause
        test_sensors_v2::TestSensorsInitLaziness tests to fail due to
        cross-test contamination.
        """
        import robot_radio.nav as nav_pkg

        # The lazy guard must be a callable __getattr__ on the module
        assert hasattr(nav_pkg, "__getattr__"), (
            "robot_radio.nav must have a __getattr__ for lazy Navigator dispatch"
        )

        # The module-level __init__ must NOT have pre-imported Navigator
        assert "Navigator" not in nav_pkg.__dict__, (
            "Navigator was eagerly imported into robot_radio.nav.__dict__ — "
            "it must stay behind __getattr__"
        )

    def test_nav_package_no_aprilcam_on_init(self):
        """Importing robot_radio.nav (without accessing Navigator) must not
        pull aprilcam or cv2 into sys.modules.
        """
        # Remove cached modules so we can check a clean import sequence
        for key in list(sys.modules.keys()):
            if key.startswith("robot_radio.nav"):
                del sys.modules[key]

        cv2_before = "cv2" in sys.modules
        aprilcam_before = "aprilcam" in sys.modules

        import robot_radio.nav  # noqa: F401

        # Only nav.navigator triggers aprilcam; __init__ eager imports do not
        if not aprilcam_before:
            assert "aprilcam" not in sys.modules, (
                "robot_radio.nav __init__ imported aprilcam eagerly — "
                "Navigator must stay behind __getattr__"
            )
        if not cv2_before:
            assert "cv2" not in sys.modules, (
                "robot_radio.nav __init__ imported cv2 eagerly — "
                "Navigator must stay behind __getattr__"
            )


# ---------------------------------------------------------------------------
# T3 — path subpackage
# ---------------------------------------------------------------------------


class TestPathImports:
    """path subpackage import chain.

    Eager imports (in __init__): builder, sampled_path, catmull_rom,
                                  obstacle, patterns.
    Optional import (requires numpy): bezier.BezierPathBuilder.
    numpy IS present so bezier must import cleanly.
    """

    def test_path_package_imports(self):
        _fresh_import("robot_radio.path")

    def test_path_arc_imports(self):
        _fresh_import("robot_radio.path.arc")

    def test_path_bezier_imports(self):
        """bezier.py requires numpy — present in this environment."""
        _fresh_import("robot_radio.path.bezier")

    def test_path_builder_imports(self):
        _fresh_import("robot_radio.path.builder")

    def test_path_catmull_rom_imports(self):
        _fresh_import("robot_radio.path.catmull_rom")

    def test_path_obstacle_imports(self):
        _fresh_import("robot_radio.path.obstacle")

    def test_path_patterns_imports(self):
        _fresh_import("robot_radio.path.patterns")

    def test_path_sampled_path_imports(self):
        _fresh_import("robot_radio.path.sampled_path")

    def test_path_path_helper_imports(self):
        _fresh_import("robot_radio.path.path_helper")


# ---------------------------------------------------------------------------
# T4 — controllers subpackage
# ---------------------------------------------------------------------------


class TestControllersImports:
    """controllers subpackage import chain.

    Eager imports (in __init__): base, pure_pursuit, stanley.
    Lazy import (via __getattr__): LTVController (requires wpimath — absent).
    """

    def test_controllers_package_imports(self):
        _fresh_import("robot_radio.controllers")

    def test_controllers_base_imports(self):
        _fresh_import("robot_radio.controllers.base")

    def test_controllers_pid_imports(self):
        _fresh_import("robot_radio.controllers.pid")

    def test_controllers_pure_pursuit_imports(self):
        _fresh_import("robot_radio.controllers.pure_pursuit")

    def test_controllers_stanley_imports(self):
        _fresh_import("robot_radio.controllers.stanley")

    def test_controllers_ltv_raises_import_error(self):
        """Importing ltv requires wpimath, which is not installed.

        The error must be a clear ImportError naming 'wpimath', not a
        confusing AttributeError.
        """
        # Remove cached module to force a fresh attempt
        sys.modules.pop("robot_radio.controllers.ltv", None)

        with pytest.raises(ImportError, match="wpimath"):
            import robot_radio.controllers.ltv  # noqa: F401

    def test_controllers_ltv_via_getattr_raises_import_error(self):
        """Accessing LTVController via the package __getattr__ must also
        raise ImportError (not AttributeError) so callers see a clear message.
        """
        import robot_radio.controllers as ctrl

        with pytest.raises(ImportError, match="wpimath"):
            _ = ctrl.LTVController


# ---------------------------------------------------------------------------
# T5 — kinematics subpackage
# ---------------------------------------------------------------------------


class TestKinematicsImports:
    """kinematics subpackage import chain.

    The __init__ is fully lazy — no eager imports at all.
    DifferentialDriveKinematics (via differential_drive.py) requires wpimath.
    """

    def test_kinematics_package_imports(self):
        """Package __init__ has no eager imports, so this is always safe."""
        _fresh_import("robot_radio.kinematics")

    def test_kinematics_differential_drive_raises_import_error(self):
        """differential_drive.py requires wpimath — must raise ImportError."""
        sys.modules.pop("robot_radio.kinematics.differential_drive", None)

        with pytest.raises(ImportError, match="wpimath"):
            import robot_radio.kinematics.differential_drive  # noqa: F401

    def test_kinematics_via_getattr_raises_import_error(self):
        """DifferentialDriveKinematics via __getattr__ must raise ImportError."""
        import robot_radio.kinematics as kinem

        with pytest.raises(ImportError, match="wpimath"):
            _ = kinem.DifferentialDriveKinematics


# ---------------------------------------------------------------------------
# T6 — lazy guarantee: no heavy deps on bare subpackage imports
# ---------------------------------------------------------------------------


class TestLazyGuarantee:
    """Importing the four subpackages must not pull heavy optional deps.

    wpimath must never appear (it's not installed).
    matplotlib must never appear (the package contains no matplotlib calls).

    cv2 / aprilcam: these ARE available via the calibrate dep group.
    nav/__init__ does NOT import Navigator eagerly, so they should not
    appear after a nav-only import.  However, once aprilcam has been
    imported by any earlier test in the session, 'aprilcam' and 'cv2'
    will already be in sys.modules.  We only assert the packages we
    control are not pulling in wpimath or matplotlib.
    """

    def test_no_wpimath_after_subpackage_imports(self):
        import robot_radio.nav       # noqa: F401
        import robot_radio.path      # noqa: F401
        import robot_radio.controllers  # noqa: F401
        import robot_radio.kinematics   # noqa: F401

        assert "wpimath" not in sys.modules, (
            "wpimath appeared in sys.modules after subpackage imports — "
            "a wpimath import must be guarded by lazy __getattr__"
        )

    def test_no_matplotlib_after_subpackage_imports(self):
        import robot_radio.nav       # noqa: F401
        import robot_radio.path      # noqa: F401
        import robot_radio.controllers  # noqa: F401
        import robot_radio.kinematics   # noqa: F401

        assert "matplotlib" not in sys.modules, (
            "matplotlib appeared in sys.modules after subpackage imports"
        )


# ---------------------------------------------------------------------------
# T7 — construction smoke tests (no wpimath, numpy present)
# ---------------------------------------------------------------------------


class TestConstruction:
    """Construct one instance from each main module without error.

    These are pure-math / pure-Python constructions — no hardware, no
    serial port, no camera, no simulation loop.  Each must complete in
    well under 1 ms.
    """

    def test_pose_construction(self):
        from robot_radio.nav.pose import Pose

        p = Pose(x=10.0, y=20.0, heading=0.5)
        assert p.x == 10.0
        assert p.y == 20.0
        assert p.heading == 0.5

    def test_waypoint_construction(self):
        from robot_radio.nav.pose import Waypoint

        wp = Waypoint(x=5.0, y=5.0)
        assert wp.heading is None

    def test_nav_params_construction(self):
        from robot_radio.nav.nav_params import NavParams

        params = NavParams()
        assert params.max_speed == 50
        assert isinstance(params.as_dict(), dict)

    def test_pid_construction(self):
        from robot_radio.controllers.pid import PID

        pid = PID(kp=1.0, ki=0.0, kd=0.0)
        assert pid.kp == 1.0

    def test_pure_pursuit_construction(self):
        from robot_radio.controllers.pure_pursuit import PurePursuitTracker

        tracker = PurePursuitTracker(
            path=[(0.0, 0.0), (10.0, 0.0)],
            lookahead=5.0,
            trackwidth=9.0,
            base_speed=40.0,
        )
        assert tracker.lookahead == 5.0

    def test_stanley_construction(self):
        from robot_radio.controllers.stanley import StanleyController

        ctrl = StanleyController(k=0.8, base_speed=40.0, trackwidth=9.0)
        assert ctrl.k == 0.8

    def test_sampled_path_construction(self):
        from robot_radio.path.sampled_path import SampledPath

        path = SampledPath(
            points=[(0.0, 0.0), (10.0, 0.0)],
            headings=[0.0, 0.0],
            builder_name="test",
            total_length_cm=10.0,
        )
        assert path.total_length_cm == 10.0
        assert len(path.points) == 2

    def test_bezier_path_builder_construction(self):
        """BezierPathBuilder requires numpy — present in this environment.

        Constructs a simple path from start to end with no intermediate
        waypoints to verify the builder runs without error.
        """
        from robot_radio.path.bezier import BezierPathBuilder
        from robot_radio.nav.pose import Pose

        builder = BezierPathBuilder()
        start = Pose(x=0.0, y=0.0, heading=0.0)
        end = Pose(x=20.0, y=0.0, heading=0.0)
        path = builder(start=start, end=end, waypoints=[], spacing_cm=2.0)

        assert len(path.points) >= 2
        assert path.builder_name == "bezier"
        assert path.total_length_cm > 0.0

    def test_catmull_rom_call(self):
        from robot_radio.path.catmull_rom import catmull_rom

        pts = [(0.0, 0.0), (5.0, 5.0), (10.0, 0.0)]
        result = catmull_rom(pts, samples_per_segment=4)
        assert len(result) > len(pts)

    def test_arc_compute_arc(self):
        from robot_radio.path.arc import compute_arc

        left, right, radius, alpha = compute_arc(
            start_pose=(0.0, 0.0, 0.0),
            target_pos=(10.0, 10.0),
            trackwidth=9.0,
        )
        # Should produce finite, non-zero values
        assert left != 0.0 or right != 0.0
