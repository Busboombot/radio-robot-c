"""Odometry tracker: converts firmware readings to world-frame positions.

Contains two parsers:
- ``parse_so()`` — legacy v1 SO stream format (kept for compatibility).
- ``parse_tlm()`` — v2 TLM ``pose=x,y,h`` format (primary, heading in cdeg).

For new code, use ``parse_tlm()`` which delegates to the v2 protocol module.

Usage:
    tracker = OdomTracker(world_pos_cm, world_yaw_rad)
    tracker.anchor(conn)           # block until first SO arrives
    tracker.drain(conn, ms=40)     # read available SO, grow path
    print(tracker.world_pos)       # latest world position in cm
    print(tracker.path)            # full path in world cm
"""

from __future__ import annotations

import math


def parse_tlm(line: str) -> dict | None:
    """Parse a v2 TLM line and return a dict with sensor fields, or None.

    Delegates to ``robot_radio.robot.protocol.parse_tlm`` for the actual
    parsing.  Returns a dict with the available fields from the TLM frame,
    for example::

        {'pose': (x_mm, y_mm, h_cdeg), 'enc': (left_mm, right_mm), 't': 12345}

    Returns ``None`` if the line is not a recognisable TLM frame.

    Example::

        result = parse_tlm("TLM t=500 pose=350,-12,1780 enc=1024,1019")
        # {'t': 500, 'pose': (350, -12, 1780), 'enc': (1024, 1019)}
    """
    from robot_radio.robot.protocol import parse_tlm as _proto_parse_tlm

    frame = _proto_parse_tlm(line)
    if frame is None:
        return None

    out: dict = {}
    if frame.t is not None:
        out["t"] = frame.t
    if frame.pose is not None:
        out["pose"] = frame.pose
    if frame.enc is not None:
        out["enc"] = frame.enc
    if frame.vel is not None:
        out["vel"] = frame.vel
    if frame.mode is not None:
        out["mode"] = frame.mode
    return out if out else {}   # empty dict (not None) for bare "TLM" line


def parse_so(line) -> tuple | None:
    """Parse 'SO+1234-0567+090' → (x_mm, y_mm, h_deg) or None."""
    s = str(line).lstrip("<# ").strip()
    if not s.startswith("SO"):
        return None
    body = s[2:]
    try:
        parts, cur = [], ""
        for ch in body:
            if ch in "+-":
                if cur:
                    parts.append(int(cur))
                cur = ch
            else:
                cur += ch
        if cur:
            parts.append(int(cur))
        if len(parts) >= 3:
            return parts[0], parts[1], parts[2]
    except ValueError:
        return None
    return None


class OdomTracker:
    """Converts firmware SO readings to world-frame positions in cm.

    The firmware emits SO values in robot frame (chip-mounting reflection
    and translation offset are baked in via the OO command).  This class
    rotates robot-frame deltas into camera world coordinates using the
    camera-derived yaw captured at anchor time.
    """

    MIN_MOVE_CM = 0.3

    def __init__(self, world_pos_cm: tuple, world_yaw_rad: float):
        self._ref_world = world_pos_cm
        self._ref_yaw = world_yaw_rad
        self._ref_otos: tuple | None = None
        self._last: tuple | None = None
        self.path: list[tuple] = []

    @property
    def anchored(self) -> bool:
        return self._ref_otos is not None

    @property
    def world_pos(self) -> tuple | None:
        if not self.anchored or self._last is None:
            return None
        return self._to_world(self._last)

    @property
    def world_yaw(self) -> float | None:
        """Current robot heading in world frame (CW-positive radians)."""
        if not self.anchored or self._last is None:
            return None
        # SO heading is CCW-positive degrees; world yaw is CW-positive radians.
        # A CW turn increases world_yaw and decreases SO h_deg.
        return self._ref_yaw - math.radians(self._last[2] - self._ref_otos[2])

    def anchor(self, conn, timeout_s: float = 1.0) -> bool:
        """Block until a SO reading arrives and use it as the reference.

        Returns True if anchored within timeout, False otherwise.
        """
        import time
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for line in conn.read_lines(duration_ms=100):
                v = parse_so(line)
               
                if v is not None:
                    self._ref_otos = v
                    self._last = v
                    w0 = self._to_world(v)
                    if w0 is not None:
                        self.path.append(w0)
                    return True
        return False

    def drain(self, conn, duration_ms: int = 40) -> None:
        """Read all available SO lines and append new world positions to path."""
        self.feed(conn.read_lines(duration_ms=duration_ms))

    def feed(self, lines) -> None:
        """Process an already-read iterable of lines (same logic as drain without the read)."""
        for line in lines:
            v = parse_so(line)
            if v is not None:
                self._last = v
                w = self._to_world(v)
                if w is not None:
                    if (not self.path or
                            math.hypot(w[0] - self.path[-1][0],
                                       w[1] - self.path[-1][1]) > self.MIN_MOVE_CM):
                        self.path.append(w)

    def _to_world(self, o: tuple) -> tuple | None:
        if self._ref_otos is None:
            return None
        fwd_mm   = o[1] - self._ref_otos[1]   # SO y = forward
        right_mm = o[0] - self._ref_otos[0]   # SO x = right
        a = self._ref_yaw - math.radians(self._ref_otos[2])
        c, s = math.cos(a), math.sin(a)
        # CW-positive yaw, body Y=forward X=right: body→world is [[c,-s],[-s,-c]]
        rx_mm = c * fwd_mm - s * right_mm
        ry_mm = -s * fwd_mm - c * right_mm
        return (self._ref_world[0] + rx_mm / 10.0,
                self._ref_world[1] + ry_mm / 10.0)
