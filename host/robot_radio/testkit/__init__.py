"""robot_radio.testkit — uniform test/tool helpers for sim, bench, and production.

Public API
----------
All imports here are LAZY (via __getattr__) so that ``import robot_radio``
and ``from robot_radio.testkit import make_target`` work in environments
without a live camera daemon, matplotlib, or aprilcam installed.

    from robot_radio.testkit import (
        make_target, TestRobot,
        PoseSource, FirmwarePose, CameraPose,
        SafeRun,
        read_camera_pose,
    )

See individual submodules for full documentation:
  - target.py   : make_target, TestRobot
  - pose.py     : PoseSource, FirmwarePose, CameraPose
  - safety.py   : SafeRun, RobotSilentError, RunawayAbortError
  - camera.py   : read_camera_pose
  - dash.py     : Dashboard
"""

from __future__ import annotations


def __getattr__(name: str):
    """Lazy import all public symbols from testkit submodules."""
    if name in ("make_target", "TestRobot"):
        from robot_radio.testkit import target as _target
        return getattr(_target, name)

    if name in ("PoseSource", "FirmwarePose", "CameraPose"):
        from robot_radio.testkit import pose as _pose
        return getattr(_pose, name)

    if name in ("SafeRun", "BenchRun", "RobotSilentError", "RunawayAbortError"):
        from robot_radio.testkit import safety as _safety
        return getattr(_safety, name)

    if name == "read_camera_pose":
        from robot_radio.testkit.camera import read_camera_pose
        return read_camera_pose

    if name == "Dashboard":
        from robot_radio.testkit.dash import Dashboard
        return Dashboard

    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
