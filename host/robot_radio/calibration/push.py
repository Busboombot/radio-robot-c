"""push_calibration — send calibration values to firmware.

Resolves the interface duality between the MCP path (NezhaProtocol) and the
CLI path (SerialConnection):

- When passed a ``NezhaProtocol``: if the proto already has a
  ``push_calibration`` method, delegates to it.  Otherwise extracts the
  underlying ``SerialConnection`` (``proto._conn``) and falls through to the
  direct-SET path.  This ensures forward-compatibility with the wiring that
  ticket 028-003 adds to NezhaProtocol.

- When passed a ``SerialConnection``: constructs and sends the v2 SET command
  sequence directly.

Both paths return a result dict with at minimum a ``"status"`` key.

Note: this module does NOT wire push_calibration into cli.py or robot_mcp.py.
That is ticket 028-003.
"""

from __future__ import annotations

import math
import sys
from typing import Any

from robot_radio.calibration.helpers import scale_to_int8


def push_calibration(conn_or_proto: Any, config: Any) -> dict[str, Any]:
    """Push calibration values to firmware.

    Parameters
    ----------
    conn_or_proto:
        Either a :class:`robot_radio.robot.protocol.NezhaProtocol` or a
        :class:`robot_radio.io.serial_conn.SerialConnection`.
    config:
        A :class:`robot_radio.config.robot_config.RobotConfig` (or any object
        with the same attribute structure).

    Returns
    -------
    dict
        ``{"status": "ok", ...}`` on success.  The dict may carry additional
        diagnostic keys (e.g. ``"commands"`` listing the verbs that were sent).
    """
    # Resolve duality: NezhaProtocol vs SerialConnection.
    from robot_radio.robot.protocol import NezhaProtocol
    from robot_radio.io.serial_conn import SerialConnection

    if isinstance(conn_or_proto, NezhaProtocol):
        proto = conn_or_proto
        # If NezhaProtocol has its own push_calibration method (added by
        # ticket 028-003), delegate to it.  Check both the instance and the
        # class so that either a monkey-patched instance attribute or a real
        # class method is found.
        try:
            _push_fn = object.__getattribute__(proto, "push_calibration")
        except AttributeError:
            _push_fn = getattr(type(proto), "push_calibration", None)
        if _push_fn is not None and callable(_push_fn):
            return _push_fn(config)
        # Otherwise extract the underlying connection and fall through.
        conn = proto._conn
    elif isinstance(conn_or_proto, SerialConnection):
        conn = conn_or_proto
    else:
        raise TypeError(
            f"push_calibration expects NezhaProtocol or SerialConnection, "
            f"got {type(conn_or_proto).__name__}"
        )

    return _push_via_conn(conn, config)


def _push_via_conn(conn: Any, config: Any) -> dict[str, Any]:
    """Build and send the v2 SET / OI / OL / OA sequence over *conn*.

    Mirrors the logic in ``robot_radio.io.cli._push_calibration`` so the two
    stay in sync.  ``_push_calibration`` in cli.py remains authoritative until
    ticket 028-003 replaces it; changes there should be ported here.

    The sequence:
      1. ``SET ml=<float>``  — mm_per_wheel_deg_left
      2. ``SET mr=<float>``  — mm_per_wheel_deg_right
      3. ``SET tw=<int>``    — trackwidth mm
      4. ``OI``              — OTOS init (must precede OL/OA)
      5. ``OL <int8>``       — otos_linear_scale encoded
      6. ``OA <int8>``       — otos_angular_scale encoded
      7. ``SET odomOffX/Y/Yaw`` — only when nonzero

    Returns a dict with ``"status": "ok"`` and ``"commands"`` listing sent verbs.
    """
    sent: list[str] = []

    # ── Wheel encoder calibration and trackwidth ──────────────────────────
    cal = getattr(config, "calibration", None)

    wd = getattr(getattr(config, "wheels", None), "wheel_diameter_mm", None)
    default_mm_per_deg = (math.pi * wd / 360.0) if wd is not None else None

    left_mm_per_deg  = getattr(cal, "mm_per_wheel_deg_left",  None) if cal else None
    right_mm_per_deg = getattr(cal, "mm_per_wheel_deg_right", None) if cal else None
    left_mm_per_deg  = left_mm_per_deg  if left_mm_per_deg  is not None else default_mm_per_deg
    right_mm_per_deg = right_mm_per_deg if right_mm_per_deg is not None else default_mm_per_deg

    if left_mm_per_deg is not None:
        cmd = f"SET ml={left_mm_per_deg:.6f}"
        conn.send(cmd, read_ms=200)
        sent.append(cmd)

    if right_mm_per_deg is not None:
        cmd = f"SET mr={right_mm_per_deg:.6f}"
        conn.send(cmd, read_ms=200)
        sent.append(cmd)

    geom = getattr(config, "geometry", None)
    tw = getattr(geom, "trackwidth", None) if geom else None
    if tw is not None:
        tw_int = int(round(float(tw)))
        cmd = f"SET tw={tw_int}"
        conn.send(cmd, read_ms=200)
        sent.append(cmd)

    # ── OTOS init (must precede scalar writes) ────────────────────────────
    conn.send("OI", read_ms=500)
    sent.append("OI")

    # ── OTOS scalars ──────────────────────────────────────────────────────
    lin_scale = getattr(cal, "otos_linear_scale",  None) if cal else None
    ang_scale = getattr(cal, "otos_angular_scale", None) if cal else None
    lin_scale = float(lin_scale) if lin_scale is not None else 1.0
    ang_scale = float(ang_scale) if ang_scale is not None else 1.0

    lin_int8 = scale_to_int8(lin_scale)
    ang_int8 = scale_to_int8(ang_scale)

    cmd = f"OL {lin_int8}"
    conn.send(cmd, read_ms=200)
    sent.append(cmd)

    cmd = f"OA {ang_int8}"
    conn.send(cmd, read_ms=200)
    sent.append(cmd)

    # ── OTOS mounting offset (skip if all zero) ───────────────────────────
    off = getattr(geom, "odometry_offset_mm", None) if geom else None
    if off is not None:
        ox = float(off.x) if hasattr(off, "x") else 0.0
        oy = float(off.y) if hasattr(off, "y") else 0.0
        oyaw_deg = math.degrees(float(off.yaw_rad)) if hasattr(off, "yaw_rad") else 0.0
        if ox != 0.0 or oy != 0.0 or oyaw_deg != 0.0:
            for cmd in (
                f"SET odomOffX={ox:.3f}",
                f"SET odomOffY={oy:.3f}",
                f"SET odomYaw={oyaw_deg:.3f}",
            ):
                conn.send(cmd, read_ms=200)
                sent.append(cmd)

    return {"status": "ok", "commands": sent}
