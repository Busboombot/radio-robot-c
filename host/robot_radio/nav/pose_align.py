"""Camera-to-OTOS alignment helper.

Reads the current camera pose for a robot tag and calls
:meth:`Otos.align_to` so that subsequent OTOS readings are returned in
the same world frame as the camera.

Usage::

    from robot_radio.nav.pose_align import align_otos_to_camera

    result = align_otos_to_camera(otos, odometry)
    if result["success"]:
        print("Aligned at", result["pose"])
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

from robot_radio.nav.pose import Pose

if TYPE_CHECKING:
    from robot_radio.sensors.otos import Otos
    from robot_radio.sensors.odometry import Odometry


def align_otos_to_camera(
    otos: "Otos",
    odometry: "Odometry",
    settle_frames: int = 5,
    timeout_s: float = 4.0,
) -> dict[str, Any]:
    """Read the current camera pose and align the OTOS to it.

    Collects *settle_frames* valid camera poses and averages them before
    calling :meth:`~robot_radio.sensors.otos.Otos.align_to`.  This reduces the
    effect of single-frame measurement noise.

    Parameters
    ----------
    otos:
        An initialised :class:`~robot_radio.sensors.otos.Otos` instance.
    odometry:
        An :class:`~robot_radio.sensors.odometry.Odometry` instance attached to a
        running playfield.
    settle_frames:
        Number of valid frames to average (default 5).
    timeout_s:
        Maximum seconds to wait for *settle_frames* valid poses.

    Returns
    -------
    dict
        ``{"success": True, "pose": {"x": ..., "y": ..., "heading": ...},
        "frames": N}`` on success, or ``{"success": False, "error": "..."}``
        on timeout.
    """
    import math

    deadline = time.monotonic() + timeout_s
    xs: list[float] = []
    ys: list[float] = []
    sins: list[float] = []
    coss: list[float] = []

    while time.monotonic() < deadline and len(xs) < settle_frames:
        odometry.update()
        if odometry.is_valid:
            xs.append(odometry.x)  # type: ignore[arg-type]
            ys.append(odometry.y)  # type: ignore[arg-type]
            sins.append(math.sin(odometry.yaw))  # type: ignore[arg-type]
            coss.append(math.cos(odometry.yaw))  # type: ignore[arg-type]

    if not xs:
        return {"success": False, "error": "No valid camera pose within timeout"}

    # Average position; circular mean for heading
    avg_x = sum(xs) / len(xs)
    avg_y = sum(ys) / len(ys)
    avg_h = math.atan2(sum(sins) / len(sins), sum(coss) / len(coss))

    avg_pose = Pose(avg_x, avg_y, avg_h)
    otos.align_to(avg_pose)

    return {
        "success": True,
        "pose": {"x": avg_x, "y": avg_y, "heading": avg_h},
        "frames": len(xs),
    }
