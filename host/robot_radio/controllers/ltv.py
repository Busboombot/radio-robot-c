"""LTV Unicycle controller wrapping wpimath.controller.LTVUnicycleController.

Coordinate conventions
----------------------
- Project uses **CW-positive** yaw (camera convention): 0 = east, positive
  angles rotate clockwise when viewed from above.
- WPILib uses **CCW-positive** yaw (standard math convention): 0 = east,
  positive angles rotate counter-clockwise.
- Conversion: ``Rotation2d(-our_yaw)`` — negate the yaw before passing to
  WPILib.

Output
------
Returns ``(v_left_mms, v_right_mms)`` as integers using differential-drive
kinematics::

    v_left  = vx*1000 - omega * trackwidth_mm / 2
    v_right = vx*1000 + omega * trackwidth_mm / 2

where ``vx`` is in m/s (from ChassisSpeeds) and ``omega`` is in rad/s.
"""

from __future__ import annotations

import math
from typing import Sequence

from wpimath.controller import LTVUnicycleController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from robot_radio.controllers.base import Controller


class LTVController(Controller):
    """LTV Unicycle path-following controller wrapping WPILib.

    Wraps :class:`wpimath.controller.LTVUnicycleController` to produce
    differential-drive wheel-speed commands in mm/s.

    Parameters
    ----------
    trackwidth_mm:
        Wheel-to-wheel spacing in millimetres.
    dt:
        Control loop timestep in seconds.  Default 0.040 (25 Hz).
    max_vel_ms:
        Maximum velocity for the controller gain lookup table (m/s).
        Default 0.5.
    q_pos:
        Position error tolerance (m) — used as the Q matrix diagonal for
        x and y states.  Smaller → more aggressive position correction.
        Default 0.05.
    q_theta:
        Heading error tolerance (rad) — Q matrix diagonal for heading.
        Smaller → more aggressive heading correction.  Default 0.8.
    r_vel:
        Linear velocity control effort limit (m/s) — R matrix diagonal.
        Larger → less aggressive.  Default 2.0.
    r_omega:
        Angular velocity control effort limit (rad/s) — R matrix diagonal.
        Larger → less aggressive.  Default 0.3.
    stop_dist_cm:
        Distance from the final waypoint at which arrival is declared (cm).
        Default 5.0.
    """

    def __init__(
        self,
        trackwidth_mm: float,
        dt: float = 0.040,
        max_vel_ms: float = 0.5,
        q_pos: float = 0.05,
        q_theta: float = 0.8,
        r_vel: float = 2.0,
        r_omega: float = 0.3,
        stop_dist_cm: float = 5.0,
    ) -> None:
        self._trackwidth_mm = float(trackwidth_mm)
        self._stop_dist_cm = float(stop_dist_cm)
        self._dt = float(dt)

        # Construct the underlying WPILib LTV controller.
        # Qelems: (x_tolerance_m, y_tolerance_m, heading_tolerance_rad)
        # Relems: (linear_vel_m/s, angular_vel_rad/s)
        self._ltv = LTVUnicycleController(
            (q_pos, q_pos, q_theta),
            (r_vel, r_omega),
            dt,
            max_vel_ms,
        )

        # Path state
        self._path: list[tuple[float, float]] = []
        self._path_headings: list[float] = []
        self._finished: bool = False

    # ------------------------------------------------------------------
    # Controller ABC methods
    # ------------------------------------------------------------------

    def set_path(self, path: Sequence[tuple[float, float]]) -> None:
        """Load a new path and reset internal state.

        Parameters
        ----------
        path:
            Ordered sequence of ``(x, y)`` waypoints in centimetres.
            Must contain at least two points.
        """
        pts = list(path)
        if len(pts) < 2:
            raise ValueError("path must contain at least two waypoints")
        self._path = pts
        self._path_headings = self._compute_headings(pts)
        self._finished = False

    def is_finished(self) -> bool:
        """Return True when the robot has been declared arrived."""
        return self._finished

    def compute(
        self,
        pos: tuple[float, float],
        yaw: float,
    ) -> tuple[int, int]:
        """Compute differential-drive wheel speeds using the LTV controller.

        Finds the nearest point on the stored path to use as the reference
        pose and calls :meth:`calculate`.

        Parameters
        ----------
        pos:
            Current robot position ``(x, y)`` in centimetres.
        yaw:
            Robot heading in radians, CW-positive convention
            (0 = east, positive = clockwise).

        Returns
        -------
        tuple[int, int]
            ``(v_left_mms, v_right_mms)`` as integers, or ``(0, 0)``
            when within ``stop_dist_cm`` of the final waypoint.
        """
        if not self._path:
            return (0, 0)

        # Arrival check against final waypoint
        fx, fy = self._path[-1]
        dist_to_end = math.hypot(pos[0] - fx, pos[1] - fy)
        if dist_to_end <= self._stop_dist_cm:
            self._finished = True
            return (0, 0)

        # Find nearest waypoint index
        nearest_idx = self._nearest_idx(pos)
        # Use the next waypoint as the target reference (look ahead one step)
        ref_idx = min(nearest_idx + 1, len(self._path) - 1)
        target_pos_cm = self._path[ref_idx]
        tangent_ccw = self._path_headings[ref_idx]

        # Estimate reference velocities from spacing and dt
        if ref_idx > 0:
            prev = self._path[ref_idx - 1]
            seg_len_cm = math.hypot(
                target_pos_cm[0] - prev[0], target_pos_cm[1] - prev[1]
            )
            linear_vel_ms = (seg_len_cm / 100.0) / self._dt
        else:
            linear_vel_ms = 0.0

        # Angular velocity reference from heading change
        if ref_idx < len(self._path_headings) - 1:
            dtheta = _wrap_to_pi(
                self._path_headings[ref_idx + 1] - self._path_headings[ref_idx]
            )
            angular_vel_rads = dtheta / self._dt
        else:
            angular_vel_rads = 0.0

        return self.calculate(
            pos_cm=pos,
            yaw_rad_cw=yaw,
            target_pos_cm=target_pos_cm,
            tangent_ccw_rad=tangent_ccw,
            linear_vel_ref_ms=linear_vel_ms,
            angular_vel_ref_rads=angular_vel_rads,
        )

    # ------------------------------------------------------------------
    # Public calculate API (lower-level, called by compute)
    # ------------------------------------------------------------------

    def calculate(
        self,
        pos_cm: tuple[float, float],
        yaw_rad_cw: float,
        target_pos_cm: tuple[float, float],
        tangent_ccw_rad: float,
        linear_vel_ref_ms: float,
        angular_vel_ref_rads: float,
    ) -> tuple[int, int]:
        """Compute wheel speeds from explicit pose and reference inputs.

        Parameters
        ----------
        pos_cm:
            Current robot position ``(x, y)`` in centimetres.
        yaw_rad_cw:
            Current robot heading in radians, CW-positive (camera convention).
        target_pos_cm:
            Reference position ``(x, y)`` in centimetres.
        tangent_ccw_rad:
            Reference heading at the target in radians, CCW-positive
            (standard math convention — same as the path tangent from
            :class:`~robot_radio.path_helper.Path`).
        linear_vel_ref_ms:
            Reference linear velocity in m/s.
        angular_vel_ref_rads:
            Reference angular velocity in rad/s.

        Returns
        -------
        tuple[int, int]
            ``(v_left_mms, v_right_mms)`` as integers.
        """
        # Convert project cm → WPILib metres; negate CW yaw → CCW yaw
        current_pose = Pose2d(
            Translation2d(pos_cm[0] / 100.0, pos_cm[1] / 100.0),
            Rotation2d(-yaw_rad_cw),
        )
        ref_pose = Pose2d(
            Translation2d(target_pos_cm[0] / 100.0, target_pos_cm[1] / 100.0),
            Rotation2d(tangent_ccw_rad),
        )

        chassis = self._ltv.calculate(
            current_pose,
            ref_pose,
            linear_vel_ref_ms,
            angular_vel_ref_rads,
        )

        vx_ms = chassis.vx        # m/s forward
        omega_rads = chassis.omega  # rad/s CCW

        # Differential drive kinematics → wheel speeds in mm/s
        trackwidth_m = self._trackwidth_mm / 1000.0
        v_left_ms = vx_ms - omega_rads * trackwidth_m / 2.0
        v_right_ms = vx_ms + omega_rads * trackwidth_m / 2.0

        v_left_mms = int(v_left_ms * 1000.0)
        v_right_mms = int(v_right_ms * 1000.0)

        return (v_left_mms, v_right_mms)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _nearest_idx(self, pos: tuple[float, float]) -> int:
        """Return the index of the nearest waypoint to *pos*."""
        best_idx = 0
        best_dist_sq = float("inf")
        for i, (wx, wy) in enumerate(self._path):
            d2 = (pos[0] - wx) ** 2 + (pos[1] - wy) ** 2
            if d2 < best_dist_sq:
                best_dist_sq = d2
                best_idx = i
        return best_idx

    @staticmethod
    def _compute_headings(
        pts: list[tuple[float, float]],
    ) -> list[float]:
        """Compute forward-difference tangent headings for each waypoint.

        Headings are in CCW-positive radians (standard math convention).
        The final heading copies the penultimate segment direction.
        """
        headings: list[float] = []
        for i in range(len(pts) - 1):
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
            headings.append(math.atan2(dy, dx))
        # Repeat last heading for the final waypoint
        headings.append(headings[-1] if headings else 0.0)
        return headings


def _wrap_to_pi(a: float) -> float:
    """Wrap angle *a* (radians) to (-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))
