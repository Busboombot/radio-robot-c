"""Geometric waypoint patterns for robot path demonstrations.

Extracted from ``test/follow_path.py`` (sprint 006, ticket 004).
"""

from __future__ import annotations

import math


def four_leaf_waypoints(center, tips, bulge=0.30):
    """4-petal cloverleaf: center → approach → tip → depart → center, ×4."""
    cx, cy = center
    out = [center]
    for _label, (tx, ty) in tips:
        dx, dy = tx - cx, ty - cy
        L = math.hypot(dx, dy)
        if L < 1e-3:
            continue
        ux, uy = dx / L, dy / L
        px, py = -uy, ux
        ax = cx + 0.5 * L * ux + bulge * L * px
        ay = cy + 0.5 * L * uy + bulge * L * py
        bx = cx + 0.5 * L * ux - bulge * L * px
        by = cy + 0.5 * L * uy - bulge * L * py
        out.append((ax, ay))
        out.append((tx, ty))
        out.append((bx, by))
        out.append(center)
    return out
