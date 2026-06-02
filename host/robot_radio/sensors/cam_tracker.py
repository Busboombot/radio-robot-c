"""Camera-based robot pose tracker.

Reads the robot's AprilTag from the camera daemon and maintains a running
world-frame position + heading + path.

Usage:
    cam = CamTracker.wait_for_robot(dc, cam_id, robot_tag)
    if cam is None:
        # tag not seen within timeout
        ...
    cam.update()          # refresh from latest camera frame
    print(cam.pos)        # (x, y) in world cm
    print(cam.yaw)        # heading in radians
    print(cam.path)       # list of (x, y) world cm positions
"""

from __future__ import annotations

import math
import time


class CamTracker:
    """Reads robot AprilTag from the camera daemon, tracks world position."""

    MIN_MOVE_CM = 0.5

    def __init__(self, pos: tuple, yaw: float, robot_tag: int, dc, cam_id: str):
        self.pos = pos
        self.yaw = yaw
        self.path: list[tuple] = [pos]
        self._robot_tag = robot_tag
        self._dc = dc
        self._cam_id = cam_id

    def update(self) -> bool:
        """Refresh pose from latest camera frame. Returns True if tag seen."""
        tf = self._dc.get_tags(self._cam_id)
        for t in tf.tags:
            if t.id == self._robot_tag and t.world_xy is not None:
                new_pos = (float(t.world_xy[0]), float(t.world_xy[1]))
                new_yaw = float(t.yaw)
                if (not self.path or
                        math.hypot(new_pos[0] - self.path[-1][0],
                                   new_pos[1] - self.path[-1][1]) > self.MIN_MOVE_CM):
                    self.path.append(new_pos)
                self.pos = new_pos
                self.yaw = new_yaw
                return True
        return False

    @classmethod
    def wait_for_robot(cls, dc, cam_id: str, robot_tag: int,
                       retries: int = 20, pause_s: float = 0.1) -> "CamTracker | None":
        """Wait up to retries×pause_s for the robot tag to appear.

        Returns a CamTracker anchored at the first seen pose, or None on timeout.
        """
        for _ in range(retries):
            tf = dc.get_tags(cam_id)
            for t in tf.tags:
                if t.id == robot_tag and t.world_xy is not None:
                    pos = (float(t.world_xy[0]), float(t.world_xy[1]))
                    yaw = float(t.yaw)
                    return cls(pos, yaw, robot_tag, dc, cam_id)
            time.sleep(pause_s)
        return None
