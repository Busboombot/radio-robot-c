"""Python wrapper for the SparkFun OTOS (Optical Tracking Odometry Sensor).

Communicates with the robot firmware via v2 command names only:

    O   — status: ``O <conn> <pid> <hw> <fw> <status>``
    OI  — init (enable signal processing): ``ACK:OI <spcfg>``
    OZ  — zero tracked position: ``ACK:OZ``
    OR  — reset tracking: ``ACK:OR``
    OC  — config dump: ``OC <spcfg> <self_test>``
    OP  — pose: ``OP <x_mm> <y_mm> <h_deg>``
    OV  — velocity: ``OV <vx_mms> <vy_mms> <vh_dps>``
    OL  — set linear scalar: ``OL <n>``
    OA  — set angular scalar: ``OA <n>``

Note: ``OK`` (v1 OTOS IMU calibrate ack) is a v1 verb and is NOT used here.

All firmware values are already in engineering units (mm, deg, mm/s, deg/s).
This module converts to the world-frame convention used by the rest of the stack:
centimetres + radians.

SE(2) frame offset
------------------
``align_to(world_pose)`` zeros the OTOS sensor (OZ) and records a frame offset
so that ``read_world_pose()`` returns coordinates in the same frame as the
camera.  The offset is stored as ``(tx_cm, ty_cm, theta_rad)``.

The forward transform from sensor frame to world frame is:

    x_w = tx + cos(θ) * x_s − sin(θ) * y_s
    y_w = ty + sin(θ) * x_s + cos(θ) * y_s
    h_w = θ  + h_s
"""

from __future__ import annotations

import math
from typing import Any

from robot_radio.nav.pose import Pose
from robot_radio.io.serial_conn import SerialConnection

# Default timeout for commands that just need a short ACK
_ACK_MS = 500
# Timeout for pose read — firmware replies quickly
_POSE_MS = 300


def _find_line(responses: list[str], prefix: str) -> str | None:
    """Return the first response line that starts with *prefix*.

    Strips the relay forwarding prefix (``<``) before matching so this
    works in both direct and relay connection modes.
    """
    for line in responses:
        stripped = line[1:] if line.startswith("<") else line
        if stripped.startswith(prefix):
            return stripped
    return None


class Otos:
    """Thin Python wrapper around the OTOS firmware commands.

    Parameters
    ----------
    conn:
        An open :class:`~robot_radio.io.serial_conn.SerialConnection`.

    The instance holds an SE(2) frame offset ``(tx_cm, ty_cm, theta_rad)``
    that is set by :meth:`align_to` and applied by :meth:`read_world_pose`.
    Initially the offset is the identity (zero displacement, zero rotation)
    so raw sensor values pass through unchanged.
    """

    def __init__(self, conn: SerialConnection) -> None:
        self._conn = conn
        # SE(2) offset from sensor frame to world frame
        self._tx: float = 0.0  # cm
        self._ty: float = 0.0  # cm
        self._theta: float = 0.0  # radians

    # ------------------------------------------------------------------
    # Wire helpers
    # ------------------------------------------------------------------

    def _send(self, cmd: str, read_ms: int = _ACK_MS) -> dict[str, Any]:
        """Send *cmd* and return the raw result dict from SerialConnection."""
        return self._conn.send(cmd, read_ms)

    # ------------------------------------------------------------------
    # Command methods
    # ------------------------------------------------------------------

    def connect_status(self) -> dict[str, Any]:
        """Send ``O`` and parse the status reply.

        Reply format: ``O <conn> <pid> <hw> <fw> <status>``

        Returns a dict with keys ``conn``, ``pid``, ``hw``, ``fw``,
        ``status``, or ``error`` if the reply could not be parsed.
        """
        result = self._send("O")
        line = _find_line(result.get("responses", []), "O ")
        if line is None:
            return {"error": "No O reply", "raw": result}
        parts = line.split()
        if len(parts) < 6:
            return {"error": "Malformed O reply", "raw": line}
        return {
            "conn": parts[1],
            "pid": parts[2],
            "hw": parts[3],
            "fw": parts[4],
            "status": parts[5],
        }

    def init(self) -> dict[str, Any]:
        """Send ``OI`` (init / enable signal processing).

        Reply: ``ACK:OI <spcfg>``
        """
        result = self._send("OI")
        line = _find_line(result.get("responses", []), "ACK:OI")
        if line is None:
            return {"error": "No ACK:OI reply", "raw": result}
        return {"ack": line}

    def zero(self) -> dict[str, Any]:
        """Send ``OZ`` — zero the tracked position.

        Reply: ``ACK:OZ``
        """
        result = self._send("OZ")
        line = _find_line(result.get("responses", []), "ACK:OZ")
        if line is None:
            return {"error": "No ACK:OZ reply", "raw": result}
        return {"ack": line}

    def reset_tracking(self) -> dict[str, Any]:
        """Send ``OR`` — reset tracking (clears Kalman filters, no position zero).

        Reply: ``ACK:OR``
        """
        result = self._send("OR")
        line = _find_line(result.get("responses", []), "ACK:OR")
        if line is None:
            return {"error": "No ACK:OR reply", "raw": result}
        return {"ack": line}

    def config_dump(self) -> dict[str, Any]:
        """Send ``OC`` — dump config registers.

        Reply: ``OC <spcfg> <self_test>``
        """
        result = self._send("OC")
        line = _find_line(result.get("responses", []), "OC ")
        if line is None:
            return {"error": "No OC reply", "raw": result}
        parts = line.split()
        if len(parts) < 3:
            return {"error": "Malformed OC reply", "raw": line}
        return {"spcfg": parts[1], "self_test": parts[2]}

    def read_pose_raw(self) -> tuple[float, float, float] | None:
        """Send ``OP`` and return ``(x_mm, y_mm, h_deg)`` in sensor frame.

        Returns ``None`` if the reply cannot be parsed.
        """
        result = self._send("OP", read_ms=_POSE_MS)
        line = _find_line(result.get("responses", []), "OP ")
        if line is None:
            return None
        parts = line.split()
        if len(parts) < 4:
            return None
        try:
            return (float(parts[1]), float(parts[2]), float(parts[3]))
        except ValueError:
            return None

    def read_velocity_raw(self) -> tuple[float, float, float] | None:
        """Send ``OV`` and return ``(vx_mms, vy_mms, vh_dps)`` in sensor frame.

        Returns ``None`` if the reply cannot be parsed.
        """
        result = self._send("OV", read_ms=_POSE_MS)
        line = _find_line(result.get("responses", []), "OV ")
        if line is None:
            return None
        parts = line.split()
        if len(parts) < 4:
            return None
        try:
            return (float(parts[1]), float(parts[2]), float(parts[3]))
        except ValueError:
            return None

    # ------------------------------------------------------------------
    # Frame alignment
    # ------------------------------------------------------------------

    def align_to(self, world_pose: Pose) -> None:
        """Zero the OTOS sensor and record a SE(2) frame offset.

        After this call, :meth:`read_world_pose` will return positions and
        headings in the same world frame as the camera.

        Parameters
        ----------
        world_pose:
            The current robot pose as measured by the camera (cm, radians).
            This becomes the new origin / orientation of the OTOS frame.
        """
        # Zero the sensor so raw output is near (0, 0, 0)
        self.zero()
        # Record the SE(2) offset: sensor origin = world_pose
        self._tx = world_pose.x
        self._ty = world_pose.y
        self._theta = world_pose.heading

    def read_world_pose(self) -> Pose | None:
        """Read the OTOS and return a ``Pose`` in world frame (cm, radians).

        Applies the SE(2) offset set by :meth:`align_to`.

        Returns ``None`` if the sensor reply cannot be parsed.
        """
        raw = self.read_pose_raw()
        if raw is None:
            return None
        x_mm, y_mm, h_deg = raw
        # Convert sensor units to cm and radians
        x_s = x_mm / 10.0
        y_s = y_mm / 10.0
        h_s = math.radians(h_deg)
        # Apply SE(2) rotation + translation
        cos_t = math.cos(self._theta)
        sin_t = math.sin(self._theta)
        x_w = self._tx + cos_t * x_s - sin_t * y_s
        y_w = self._ty + sin_t * x_s + cos_t * y_s
        h_w = self._theta + h_s
        # Normalise heading to [-π, π]
        h_w = (h_w + math.pi) % (2 * math.pi) - math.pi
        return Pose(x_w, y_w, h_w)
