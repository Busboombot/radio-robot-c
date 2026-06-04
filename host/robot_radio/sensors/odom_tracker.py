"""Odometry tracker: converts firmware TLM frames to world-frame positions.

Primary interface (v2):
- ``parse_tlm()`` — delegates to the v2 protocol module; returns a dict.
- ``OdomTracker.update_from_tlm(frame)`` — feed a ``TLMFrame`` directly.

Legacy (kept for backward compatibility only, NOT on the v2 hot path):
- ``parse_so()`` — v1 SO stream format parser.  Marked ``@deprecated``.
  No internal v2 code calls this function.

Usage (v2)::

    from robot_radio.robot.protocol import parse_tlm as proto_parse_tlm, TLMFrame
    from robot_radio.sensors.odom_tracker import OdomTracker

    tracker = OdomTracker(world_pos_mm=(0.0, 0.0), world_yaw_rad=0.0)
    # Feed a TLMFrame (obtained from NezhaProtocol or parse_tlm):
    tracker.update_from_tlm(tlm_frame)
    print(tracker.x_mm, tracker.y_mm, tracker.heading_cdeg)
"""

from __future__ import annotations

import math
import warnings


# ---------------------------------------------------------------------------
# v2 TLM parse helper (delegates to protocol module)
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Legacy v1 SO parser — DEPRECATED, not used on the v2 hot path
# ---------------------------------------------------------------------------

def parse_so(line) -> tuple | None:
    """[DEPRECATED] Parse 'SO+1234-0567+090' → (x_mm, y_mm, h_deg) or None.

    This is the v1 SO stream format.  In v2 the SO stream does not exist.
    Kept for backward compatibility only — do not use in new code.
    Use ``parse_tlm()`` or ``OdomTracker.update_from_tlm()`` instead.
    """
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


# ---------------------------------------------------------------------------
# OdomTracker — v2 primary interface
# ---------------------------------------------------------------------------

class OdomTracker:
    """Tracks robot pose from v2 TLMFrame.pose (x_mm, y_mm, heading_cdeg).

    The primary update path is :meth:`update_from_tlm`, which accepts a
    ``TLMFrame`` from the v2 protocol layer.

    World-frame conversion is computed from the pose at anchor time.
    Units: all millimetres and centidegrees internally; the ``world_pos``
    property converts to cm for legacy callers.

    Constructor accepts either a ``RobotConfig`` instance (``config=``)
    or bare keyword args (``trackwidth_mm``, ``mm_per_deg_l``,
    ``mm_per_deg_r``) for the kinematic parameters.  If neither is
    supplied the tracker operates in a simple pose-forwarding mode without
    wheel-based dead-reckoning.

    Parameters
    ----------
    world_pos_mm:
        Initial world position in mm (x_mm, y_mm).  Defaults to (0, 0).
    world_yaw_rad:
        Initial world heading in radians (CW-positive).  Defaults to 0.
    config:
        Optional ``RobotConfig``.  When supplied, ``trackwidth_mm``,
        ``mm_per_deg_l``, and ``mm_per_deg_r`` are read from it.
    trackwidth_mm:
        Track width in mm (overrides config if supplied directly).
    mm_per_deg_l:
        Left-wheel mm per encoder degree (calibration param).
    mm_per_deg_r:
        Right-wheel mm per encoder degree (calibration param).
    """

    # Minimum movement (mm) before a new world position is appended to path.
    MIN_MOVE_MM = 3.0

    def __init__(
        self,
        world_pos_mm: tuple[float, float] = (0.0, 0.0),
        world_yaw_rad: float = 0.0,
        *,
        config=None,
        trackwidth_mm: float | None = None,
        mm_per_deg_l: float | None = None,
        mm_per_deg_r: float | None = None,
    ) -> None:
        self._ref_world_mm: tuple[float, float] = world_pos_mm
        self._ref_yaw: float = world_yaw_rad

        # Kinematic parameters (optional; sourced from RobotConfig when supplied)
        if config is not None:
            tw = getattr(config, "trackwidth", None)
            self.trackwidth_mm: float | None = float(tw) if tw is not None else trackwidth_mm
            cal = getattr(config, "calibration", None)
            if cal is not None:
                self.mm_per_deg_l: float | None = getattr(cal, "mm_per_wheel_deg_left", None)
                self.mm_per_deg_r: float | None = getattr(cal, "mm_per_wheel_deg_right", None)
            else:
                self.mm_per_deg_l = mm_per_deg_l
                self.mm_per_deg_r = mm_per_deg_r
        else:
            self.trackwidth_mm = trackwidth_mm
            self.mm_per_deg_l = mm_per_deg_l
            self.mm_per_deg_r = mm_per_deg_r

        # State from TLM
        self._ref_pose: tuple[int, int, int] | None = None   # (x_mm, y_mm, cdeg) at anchor
        self._last_pose: tuple[int, int, int] | None = None  # (x_mm, y_mm, cdeg) latest

        # Path in world mm (list of (x_mm, y_mm))
        self.path: list[tuple[float, float]] = []

    # ------------------------------------------------------------------
    # Primary v2 update path
    # ------------------------------------------------------------------

    def update_from_tlm(self, frame) -> bool:
        """Update pose from a ``TLMFrame``.

        Parameters
        ----------
        frame:
            A ``TLMFrame`` object (from ``robot_radio.robot.protocol``).
            Must have a ``pose`` field ``(x_mm, y_mm, heading_cdeg)``.

        Returns
        -------
        bool
            ``True`` if the frame contained a ``pose`` field and state was
            updated; ``False`` otherwise.
        """
        pose = getattr(frame, "pose", None)
        if pose is None:
            return False
        self._last_pose = pose  # (x_mm, y_mm, heading_cdeg)

        # Auto-anchor on first pose received
        if not self.anchored:
            self.anchor_pose(pose)
            return True

        w = self._to_world_mm(pose)
        if w is not None:
            if (not self.path or
                    math.hypot(w[0] - self.path[-1][0],
                               w[1] - self.path[-1][1]) > self.MIN_MOVE_MM):
                self.path.append(w)
        return True

    def anchor_pose(self, pose: tuple[int, int, int]) -> None:
        """Set the reference pose (anchor point).

        Parameters
        ----------
        pose:
            ``(x_mm, y_mm, heading_cdeg)`` from a ``TLMFrame``.
        """
        self._ref_pose = pose
        self._last_pose = pose
        w0 = self._to_world_mm(pose)
        if w0 is not None and not self.path:
            self.path.append(w0)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def anchored(self) -> bool:
        """True once the tracker has received its first pose."""
        return self._ref_pose is not None

    @property
    def x_mm(self) -> int | None:
        """Raw x position from the latest TLM frame (mm)."""
        return self._last_pose[0] if self._last_pose is not None else None

    @property
    def y_mm(self) -> int | None:
        """Raw y position from the latest TLM frame (mm)."""
        return self._last_pose[1] if self._last_pose is not None else None

    @property
    def heading_cdeg(self) -> int | None:
        """Raw heading from the latest TLM frame (centidegrees)."""
        return self._last_pose[2] if self._last_pose is not None else None

    @property
    def heading_deg(self) -> float | None:
        """Heading in degrees (centidegrees / 100)."""
        if self._last_pose is None:
            return None
        return self._last_pose[2] / 100.0

    @property
    def heading_rad(self) -> float | None:
        """Heading in radians."""
        if self._last_pose is None:
            return None
        return math.radians(self._last_pose[2] / 100.0)

    @property
    def world_pos(self) -> tuple[float, float] | None:
        """Current world position in **cm** (legacy unit; use x_mm/y_mm for mm)."""
        if not self.anchored or self._last_pose is None:
            return None
        w = self._to_world_mm(self._last_pose)
        if w is None:
            return None
        return (w[0] / 10.0, w[1] / 10.0)

    @property
    def world_yaw(self) -> float | None:
        """Current heading in world frame (CW-positive radians)."""
        if not self.anchored or self._last_pose is None:
            return None
        assert self._ref_pose is not None
        # TLM heading is CCW-positive centidegrees.
        # Delta in radians (CCW positive), then flip sign for CW world convention.
        delta_rad = math.radians((self._last_pose[2] - self._ref_pose[2]) / 100.0)
        return self._ref_yaw - delta_rad

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _to_world_mm(self, pose: tuple[int, int, int]) -> tuple[float, float] | None:
        """Convert a TLM pose to world-frame mm using the anchor pose."""
        if self._ref_pose is None:
            return None
        # TLM: x = right, y = forward (robot frame at anchor time)
        fwd_mm = pose[1] - self._ref_pose[1]
        right_mm = pose[0] - self._ref_pose[0]
        # Rotate from robot-anchor-frame to world frame.
        # a = world yaw at anchor time (CW-positive radians)
        # Robot body Y=forward, X=right → world transform:
        #   world_x = cos(a)*fwd - sin(a)*right
        #   world_y = -sin(a)*fwd - cos(a)*right   (CW rotation convention)
        a = self._ref_yaw - math.radians(self._ref_pose[2] / 100.0)
        c, s = math.cos(a), math.sin(a)
        rx_mm = c * fwd_mm - s * right_mm
        ry_mm = -s * fwd_mm - c * right_mm
        return (
            self._ref_world_mm[0] + rx_mm,
            self._ref_world_mm[1] + ry_mm,
        )

    # ------------------------------------------------------------------
    # Legacy SO-based interface (v1 hot path — DEPRECATED, no v2 callers)
    # ------------------------------------------------------------------

    def anchor(self, conn, timeout_s: float = 1.0) -> bool:
        """[DEPRECATED] Block until a SO reading arrives and use it as the reference.

        This method uses the v1 SO stream and is preserved for backward
        compatibility only.  In v2, use ``update_from_tlm()`` instead.

        Returns True if anchored within timeout, False otherwise.
        """
        import time
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for line in conn.read_lines(duration_ms=100):
                v = parse_so(line)
                if v is not None:
                    # Convert SO (mm) to TLM-like pose for anchoring.
                    # SO heading is in integer degrees; store as cdeg.
                    pose_cdeg = (v[0], v[1], v[2] * 100)
                    self.anchor_pose(pose_cdeg)
                    return True
        return False

    def drain(self, conn, duration_ms: int = 40) -> None:
        """[DEPRECATED] Read all available SO lines (v1 legacy path)."""
        self.feed(conn.read_lines(duration_ms=duration_ms))

    def feed(self, lines) -> None:
        """[DEPRECATED] Process SO lines (v1 legacy path).

        For v2, feed TLMFrame objects via ``update_from_tlm()`` instead.
        """
        for line in lines:
            v = parse_so(line)
            if v is not None:
                # SO: (x_mm, y_mm, h_deg); convert heading to cdeg
                pose_cdeg = (v[0], v[1], v[2] * 100)
                self._last_pose = pose_cdeg
                w = self._to_world_mm(pose_cdeg)
                if w is not None:
                    if (not self.path or
                            math.hypot(w[0] - self.path[-1][0],
                                       w[1] - self.path[-1][1]) > self.MIN_MOVE_MM):
                        self.path.append(w)
