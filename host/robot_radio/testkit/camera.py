"""robot_radio.testkit.camera — read_camera_pose: circular-mean tag averaging.

Consolidates the tag-averaging logic duplicated across:
  - host_tests/playfield_tour/playfield_tour_camera.py
  - host_tests/playfield_tour/playfield_random_tour.py
  - tests/playfield_tour/tour_goto.py

All three sites average x, y linearly and average yaw circularly using
atan2(mean(sin), mean(cos)) to handle angle wrap-around correctly.

The camera convention is:
  - world heading = tag yaw + 90 degrees (HEAD_OFF = pi/2)
  - confirmed by empirical bench verification (documented in MEMORY.md)

Usage::

    from robot_radio.testkit.camera import read_camera_pose

    x_cm, y_cm, yaw_rad = read_camera_pose(playfield, tag_id=100, n=5)

aprilcam imports are deferred inside the Playfield.get_tag() call so that
importing this module does not require a live camera daemon.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from robot_radio.field.playfield import Playfield

# Camera heading offset: world heading = tag yaw + 90 degrees.
# Confirmed by bench: a +forward nudge moves the robot along yaw+90deg.
_HEAD_OFF = math.pi / 2.0


def read_camera_pose(
    playfield: "Playfield",
    tag_id: int = 100,
    n: int = 5,
    timeout: float = 4.0,
) -> tuple[float, float, float]:
    """Return averaged robot world pose (x_cm, y_cm, yaw_rad) from camera tags.

    Polls the aprilcam daemon for tag readings via the Playfield interface,
    averaging linearly for x and y and circularly for yaw (to handle angle
    wrap-around).

    The world heading applies the camera heading convention:
        world_yaw = atan2(mean(sin(tag_yaw)), mean(cos(tag_yaw))) + pi/2

    Parameters
    ----------
    playfield:
        Open Playfield that provides daemon access via ``get_tag()``.
    tag_id:
        AprilTag ID to read.  Default 100 (robot tag).
    n:
        Number of valid readings to collect before returning.
    timeout:
        Maximum seconds to wait for n readings.

    Returns
    -------
    tuple[float, float, float]
        (x_cm, y_cm, yaw_rad) in world (A1-centred, y-up, CCW+) coordinates.

    Raises
    ------
    RuntimeError
        If no tag readings were obtained within the timeout.
    """
    xs: list[float] = []
    ys: list[float] = []
    sin_yaws: list[float] = []
    cos_yaws: list[float] = []

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline and len(xs) < n:
        try:
            tag = playfield.get_tag(tag_id)
        except Exception:
            time.sleep(0.05)
            continue

        if tag is not None:
            xs.append(tag.x)
            ys.append(tag.y)
            sin_yaws.append(math.sin(tag.yaw))
            cos_yaws.append(math.cos(tag.yaw))
        else:
            time.sleep(0.05)

    if not xs:
        raise RuntimeError(
            f"read_camera_pose: no readings for tag {tag_id} within {timeout:.1f}s"
        )

    n_got = len(xs)
    x_cm = sum(xs) / n_got
    y_cm = sum(ys) / n_got
    mean_sin = sum(sin_yaws) / n_got
    mean_cos = sum(cos_yaws) / n_got
    yaw_rad = math.atan2(mean_sin, mean_cos) + _HEAD_OFF

    return (x_cm, y_cm, yaw_rad)
