"""robot_radio.testkit.pose — PoseSource protocol, FirmwarePose, CameraPose.

Provides a uniform interface for reading robot pose across all targets
(sim, bench, production).  All implementations return (x_cm, y_cm, yaw_rad).

Usage::

    from robot_radio.testkit.pose import FirmwarePose, CameraPose

    # Firmware pose from SNAP telemetry:
    pose_src = FirmwarePose(robot)
    x_cm, y_cm, yaw_rad = pose_src.read()

    # Camera pose from aprilcam daemon (deferred import):
    pose_src = CameraPose(playfield, tag_id=100)
    x_cm, y_cm, yaw_rad = pose_src.read()
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Protocol, runtime_checkable

if TYPE_CHECKING:
    from robot_radio.robot.nezha import Nezha
    from robot_radio.field.playfield import Playfield


# --------------------------------------------------------------------------- #
# Protocol                                                                     #
# --------------------------------------------------------------------------- #

@runtime_checkable
class PoseSource(Protocol):
    """Uniform pose-read interface.

    Returns (x_cm, y_cm, yaw_rad) where:
      - x_cm, y_cm  : world position in centimetres (A1-centred, y-up)
      - yaw_rad     : world heading in radians, CCW-positive, 0 = east
    """

    def read(self) -> tuple[float, float, float]:
        """Return the current pose as (x_cm, y_cm, yaw_rad)."""
        ...


# --------------------------------------------------------------------------- #
# FirmwarePose                                                                 #
# --------------------------------------------------------------------------- #

class FirmwarePose:
    """PoseSource that reads from firmware SNAP telemetry.

    Calls ``robot.refresh()`` to issue a SNAP command, then reads the
    OTOS pose from ``robot.state.pose``.  Converts mm → cm for the return
    value.

    Note: before the first SNAP the pose is (0.0, 0.0, 0.0) — firmware
    reports the boot origin until OTOS is initialised or a world-pose is
    pushed.
    """

    def __init__(self, robot: "Nezha") -> None:
        self._robot = robot

    def read(self) -> tuple[float, float, float]:
        """Issue SNAP, update state, return (x_cm, y_cm, yaw_rad).

        pose.x and pose.y are in mm (from TLM wire format); converted to cm.
        pose.heading is already in radians.
        """
        state = self._robot.refresh()
        x_cm = state.pose.x / 10.0
        y_cm = state.pose.y / 10.0
        yaw_rad = state.pose.heading
        return (x_cm, y_cm, yaw_rad)


# --------------------------------------------------------------------------- #
# CameraPose                                                                   #
# --------------------------------------------------------------------------- #

class CameraPose:
    """PoseSource that reads pose from the aprilcam daemon.

    Thin wrapper around ``read_camera_pose`` — calls it on each ``read()``
    and returns the averaged (x_cm, y_cm, yaw_rad) result.

    The aprilcam import is deferred (inside ``read()``) so that importing
    this module does not require a live daemon.
    """

    def __init__(
        self,
        playfield: "Playfield",
        tag_id: int = 100,
        n: int = 5,
        timeout: float = 4.0,
    ) -> None:
        self._playfield = playfield
        self._tag_id = tag_id
        self._n = n
        self._timeout = timeout

    def read(self) -> tuple[float, float, float]:
        """Return (x_cm, y_cm, yaw_rad) averaged from n camera observations.

        Delegates to read_camera_pose — imports aprilcam lazily.

        Raises
        ------
        RuntimeError
            If no tag readings were obtained within the timeout.
        """
        from robot_radio.testkit.camera import read_camera_pose  # lazy

        return read_camera_pose(
            self._playfield,
            self._tag_id,
            n=self._n,
            timeout=self._timeout,
        )
