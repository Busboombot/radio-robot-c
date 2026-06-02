"""Arc path geometry for differential-drive robots.

Given a starting pose (x, y, heading) and a target position (x, y), computes
the unique circular arc that starts tangent to the heading and passes through
the target.
"""

from __future__ import annotations

import math


def compute_arc(
    start_pose: tuple[float, float, float],
    target_pos: tuple[float, float],
    trackwidth: float,
) -> tuple[float, float, float, float]:
    """Compute the arc from start_pose to target_pos for a differential-drive robot.

    The arc is the unique circle through start_pos that is tangent to the robot's
    heading at start and also passes through target_pos.

    Args:
        start_pose: (x_cm, y_cm, heading_rad) — robot center position and heading.
        target_pos: (x_cm, y_cm) — destination (heading at target is ignored).
        trackwidth: Distance between wheel contact points, in cm.

    Returns:
        (left_dist_cm, right_dist_cm, radius_cm, inscribed_angle_rad)

        radius > 0  → center is to the left of the robot (left/CCW turn)
        radius < 0  → center is to the right (right/CW turn)
        inscribed_angle > 0 → CCW arc; < 0 → CW arc

        Returns (dist, dist, inf, 0.0) for the straight-line degenerate case
        (target lies on the heading ray).
    """
    rx, ry, theta = start_pose
    tx, ty = target_pos

    dx = tx - rx
    dy = ty - ry

    # Lateral component of the target relative to the heading vector.
    # Positive = target is to the right of heading; negative = left.
    cross = dx * math.sin(theta) - dy * math.cos(theta)

    if abs(cross) < 1e-6:
        dist = math.hypot(dx, dy)
        return (dist, dist, math.inf, 0.0)

    # Signed radius: positive = center to the left = left/CCW turn.
    R = -(dx * dx + dy * dy) / (2.0 * cross)

    # Turn center.
    cx = rx - R * math.sin(theta)
    cy = ry + R * math.cos(theta)

    # Angle of robot and target relative to turn center.
    theta_start = math.atan2(ry - cy, rx - cx)
    theta_end = math.atan2(ty - cy, tx - cx)

    # Inscribed angle: sweep in the direction of travel.
    # R > 0 → CCW (increasing angle); R < 0 → CW (decreasing angle).
    alpha = theta_end - theta_start
    if R >= 0:
        if alpha <= 0:
            alpha += 2 * math.pi
    else:
        if alpha >= 0:
            alpha -= 2 * math.pi

    left_dist = (R - trackwidth / 2.0) * alpha
    right_dist = (R + trackwidth / 2.0) * alpha

    return (left_dist, right_dist, R, alpha)
