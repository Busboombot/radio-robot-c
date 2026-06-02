"""Stanley path-following controller for differential-drive robots.

The Stanley control law combines two error terms:

1. **Heading error** (``theta_e``): the difference between the path tangent
   and the robot's current heading.  Drives the robot to align with the path.

2. **Cross-track error** (``e``): the signed lateral distance from the robot
   to the nearest point on the path.  The ``atan2`` term softly saturates at
   ±π/2 via the arc-tangent, preventing excessive steering from large offsets.

Steering formula::

    delta = theta_e + atan2(k * e, v_desired + v_soft)
    omega = omega_gain * delta

    left  = base_speed + omega * trackwidth / 2
    right = base_speed - omega * trackwidth / 2
    # clamp both to [-100, 100]

All distance values are in **centimetres** (project convention).

``base_speed`` and ``v_soft`` are **motor command units** (0–100 scale),
NOT metres per second.  ``v_desired`` defaults to ``base_speed`` at
construction time.

Reference point: cross-track error is computed against the robot tag pose
directly (not a front-bumper offset).  If oscillation is observed, a
forward offset can be added as a ``NavParam`` field (see architecture open
question 1).
"""

from __future__ import annotations

import math
from typing import Sequence

from robot_radio.path.path_helper import Path
from robot_radio.controllers.base import Controller


def _wrap_to_pi(a: float) -> float:
    """Wrap angle *a* (radians) to (-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


class StanleyController(Controller):
    """Stanley path-following controller for differential-drive robots.

    Inherits from :class:`~robot_radio.controllers.base.Controller`.

    Parameters
    ----------
    k:
        Cross-track error gain.  Higher values converge faster but may
        overshoot or oscillate.  Default 0.8.
    v_soft:
        Soft-minimum speed for the cross-track term denominator (motor
        command units).  Prevents division-by-zero when ``base_speed`` is
        zero.  Default 0.1.
    omega_gain:
        Gain applied to the total steering angle ``delta`` to produce
        angular velocity.  Default 2.0.
    goal_tolerance:
        Distance from the final waypoint at which the robot is considered
        arrived (cm).  ``compute`` returns ``(0.0, 0.0)`` inside this
        radius.  Default 9.0 cm.
    base_speed:
        Nominal forward motor command (0–100 scale).  Also used as
        ``v_desired`` in the Stanley formula.  Default 40.0.
    trackwidth:
        Wheel-to-wheel spacing in centimetres.  Default 9.0 cm.
    max_delta:
        Maximum absolute steering angle (radians) applied after combining
        heading error and cross-track error.  Prevents runaway steering when
        the initial heading error is large.  Default π/2 (90°).
    """

    def __init__(
        self,
        k: float = 0.8,
        v_soft: float = 0.1,
        omega_gain: float = 2.0,
        goal_tolerance: float = 9.0,
        base_speed: float = 40.0,
        trackwidth: float = 9.0,
        max_delta: float = math.pi / 2,
    ) -> None:
        self.k = float(k)
        self.v_soft = float(v_soft)
        self.omega_gain = float(omega_gain)
        self.goal_tolerance = float(goal_tolerance)
        self.base_speed = float(base_speed)
        self.trackwidth = float(trackwidth)
        self.max_delta = float(max_delta)

        # Placeholder path — caller must call set_path before compute
        placeholder: list[tuple[float, float]] = [(0.0, 0.0), (1.0, 0.0)]
        self._path: Path = Path(placeholder)

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
        """
        self._path = Path(list(waypoints))
        self.reset()

    def is_finished(self, pos: tuple[float, float] | None = None) -> bool:
        """Return True when *pos* is within ``goal_tolerance`` of the final waypoint.

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
        return math.hypot(pos[0] - fx, pos[1] - fy) <= self.goal_tolerance

    def reset(self) -> None:
        """Reset the nearest-point cache without loading a new path.

        Call this when the robot is teleported to a position that may be
        behind the current cached segment index.
        """
        self._path.reset()

    def compute(
        self,
        pos: tuple[float, float],
        yaw: float,
    ) -> tuple[float, float]:
        """Compute motor commands using the Stanley control law.

        Parameters
        ----------
        pos:
            Current robot position ``(x, y)`` in world coordinates (cm).
        yaw:
            Robot heading in radians, standard math convention
            (0 = east, π/2 = north).

        Returns
        -------
        (left, right):
            Motor command floats clamped to [-100, 100], or ``(0.0, 0.0)``
            when within ``goal_tolerance`` of the final waypoint.
        """
        if self.is_finished(pos):
            return (0.0, 0.0)

        idx, px, py = self._path.nearest_point(pos[0], pos[1])
        path_theta = self._path.tangent_at(idx)

        # Heading error: signed difference from robot heading to path tangent.
        # The sign convention matches the steering law below: positive heading
        # error means the robot is pointing CW of the path (too far clockwise)
        # and needs a CCW correction (delta > 0 → left faster → theta increases
        # in the CW-positive coordinate system used by the robot camera).
        # Using (yaw - path_theta) rather than (path_theta - yaw) aligns the
        # heading correction with the cross-track correction direction.
        theta_e = _wrap_to_pi(yaw - path_theta)

        # Cross-track error: signed lateral distance from robot to nearest path point
        # Positive e → robot is to the left of the path (needs to steer right,
        # i.e. delta > 0 → left faster → CW correction).
        # Sign convention: e = -sin(path_theta)*dx + cos(path_theta)*dy
        dx = pos[0] - px
        dy = pos[1] - py
        e = -math.sin(path_theta) * dx + math.cos(path_theta) * dy

        # Stanley steering angle — atan2 saturates at ±π/2 for large errors.
        # delta is clamped to [-max_delta, max_delta] to prevent runaway
        # steering when theta_e is large (e.g. large initial heading offset).
        delta = theta_e + math.atan2(self.k * e, self.base_speed + self.v_soft)
        delta = max(-self.max_delta, min(self.max_delta, delta))

        # Convert steering angle to angular velocity, then to wheel speeds
        omega = self.omega_gain * delta

        left = self.base_speed + omega * self.trackwidth / 2.0
        right = self.base_speed - omega * self.trackwidth / 2.0

        left = max(-100.0, min(100.0, left))
        right = max(-100.0, min(100.0, right))
        return (left, right)
