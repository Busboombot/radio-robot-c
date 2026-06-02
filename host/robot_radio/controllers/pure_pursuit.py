"""Pure Pursuit path tracker for differential-drive robots.

Reference implementation: vendor/PurePursuit/RobotSimulator.py
(FRC differential drive pure pursuit by team 254 / open-source community)

Algorithm summary
-----------------
1. Lookahead point — arc-length lookahead from the nearest point on the
   path, delegated to :class:`~robot_radio.path_helper.Path`.  Falls back
   to the final waypoint when the remaining path length is less than the
   lookahead distance.

2. Curvature — signed lateral offset of the lookahead point in the robot
   frame divided by the squared lookahead distance::

       κ = 2 * d_lateral / Lf²

   Sign convention (standard math yaw: 0 = east, π/2 = north):
   positive κ → left wheel faster → robot curves right.
   negative κ → right wheel faster → robot curves left.

3. Differential-drive wheel speeds::

       left  = base_speed * (2 + κ * trackwidth) / 2
       right = base_speed * (2 - κ * trackwidth) / 2

   Both values are clamped to [-100, 100].

Internal geometry now uses :class:`~robot_radio.path_helper.Path` for
O(1)-amortised nearest-point search and arc-length lookahead, replacing
the previous circle-line intersection scan.
"""

from __future__ import annotations

import math
from typing import Sequence

from robot_radio.path.path_helper import Path
from robot_radio.controllers.base import Controller


class PurePursuitTracker(Controller):
    """Pure-pursuit geometry for differential-drive path tracking.

    Inherits from :class:`~robot_radio.controllers.base.Controller`.

    Parameters
    ----------
    path:
        Ordered sequence of (x, y) world-coordinate waypoints in cm.
    lookahead:
        Lookahead distance in cm.  Default 15.0.
    trackwidth:
        Wheel-to-wheel spacing in cm.  Default 9.0 (QBot Pro).
    base_speed:
        Nominal forward motor command (0-100).  Default 40.0.
    stop_dist:
        Distance from the final waypoint at which the robot is considered
        arrived and ``compute()`` returns ``(0.0, 0.0)``.  Default 5.0 cm.
    """

    def __init__(
        self,
        path: Sequence[tuple[float, float]],
        lookahead: float = 15.0,
        trackwidth: float = 9.0,
        base_speed: float = 40.0,
        stop_dist: float = 5.0,
    ) -> None:
        if len(path) < 2:
            raise ValueError("path must contain at least two waypoints")
        self.lookahead = lookahead
        self.trackwidth = trackwidth
        self.base_speed = base_speed
        self.stop_dist = stop_dist
        # Build the Path object and reset its cache
        self.set_path(path)

    # ------------------------------------------------------------------
    # Controller ABC methods
    # ------------------------------------------------------------------

    def set_path(self, waypoints: Sequence[tuple[float, float]]) -> None:
        """Load a new path and reset internal state.

        Parameters
        ----------
        waypoints:
            Ordered sequence of (x, y) waypoints in centimetres.  Must
            contain at least two points.

        Notes
        -----
        Calls :meth:`reset` internally so that the nearest-point cache is
        cleared whenever a new path is loaded.
        """
        self._path = Path(waypoints)
        self.reset()

    def is_finished(self, pos: tuple[float, float] | None = None) -> bool:
        """Return True when *pos* is within ``stop_dist`` of the final waypoint.

        Parameters
        ----------
        pos:
            Current robot position ``(x, y)`` in centimetres.  When called
            with no argument (ABC interface), returns False (path state
            unknown without position).
        """
        if pos is None:
            return False
        fx, fy = self._path._points[-1]
        return math.hypot(pos[0] - fx, pos[1] - fy) <= self.stop_dist

    def reset(self) -> None:
        """Reset the nearest-point cache without loading a new path.

        Call this when the robot is teleported to a position that may be
        behind the current cached segment index.
        """
        self._path.reset()

    # ------------------------------------------------------------------
    # Public compute API
    # ------------------------------------------------------------------

    def compute(
        self,
        robot_pos: tuple[float, float],
        robot_yaw: float,
    ) -> tuple[float, float]:
        """Return ``(left_speed, right_speed)`` motor commands.

        Parameters
        ----------
        robot_pos:
            Current robot position ``(x, y)`` in world coordinates (cm).
        robot_yaw:
            Robot heading in radians, standard math convention
            (0 = east, π/2 = north).

        Returns
        -------
        tuple[float, float]
            ``(left, right)`` motor commands clamped to [-100, 100], or
            ``(0.0, 0.0)`` when within ``stop_dist`` of the final waypoint.
        """
        # Arrival check — Euclidean distance to final waypoint
        if self.is_finished(robot_pos):
            return (0.0, 0.0)

        lx, ly = self._find_lookahead(robot_pos)
        kappa = self._curvature(robot_pos, robot_yaw, lx, ly)

        left = self.base_speed * (2.0 + kappa * self.trackwidth) / 2.0
        right = self.base_speed * (2.0 - kappa * self.trackwidth) / 2.0

        left = max(-100.0, min(100.0, left))
        right = max(-100.0, min(100.0, right))
        return (left, right)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _find_lookahead(
        self, robot_pos: tuple[float, float]
    ) -> tuple[float, float]:
        """Return the lookahead point on the path.

        Delegates to :meth:`~robot_radio.path_helper.Path.point_at_lookahead`,
        which walks arc-length ahead from the nearest point.  Returns the
        final waypoint when the remaining path length is less than the
        lookahead distance.
        """
        return self._path.point_at_lookahead(
            robot_pos[0], robot_pos[1], self.lookahead
        )

    @staticmethod
    def _curvature(
        robot_pos: tuple[float, float],
        robot_yaw: float,
        lx: float,
        ly: float,
    ) -> float:
        """Compute signed curvature κ from robot pose to lookahead point.

        Derived from the FRC reference ``curvature()`` function, translated
        to standard math angle convention (yaw=0 east, yaw=π/2 north).

        The lateral (perpendicular) signed offset of the lookahead in the
        robot frame is::

            d_lateral = sin(yaw)*(lx-rx) - cos(yaw)*(ly-ry)

        Positive ``d_lateral`` means lookahead is to the RIGHT of the heading.
        Curvature::

            κ = 2 * d_lateral / Lf²

        Positive κ → left > right → robot curves right (toward a right target).
        Negative κ → right > left → robot curves left.
        """
        rx, ry = robot_pos
        to_lx = lx - rx
        to_ly = ly - ry
        lf_sq = to_lx * to_lx + to_ly * to_ly

        if lf_sq < 1e-9:
            return 0.0

        # Lateral component: positive = lookahead is to the RIGHT
        d_lateral = math.sin(robot_yaw) * to_lx - math.cos(robot_yaw) * to_ly
        return 2.0 * d_lateral / lf_sq
