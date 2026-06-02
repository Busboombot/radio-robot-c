"""Centripetal Catmull-Rom spline and pure-pursuit lookahead geometry.

Extracted from ``test/follow_path.py`` (sprint 006, ticket 004).
"""

from __future__ import annotations

import math


def catmull_rom(points, samples_per_segment=24, alpha=0.5):
    """Centripetal Catmull–Rom spline through *points*."""
    if len(points) < 2:
        return list(points)
    p0p = (2*points[0][0] - points[1][0], 2*points[0][1] - points[1][1])
    pNp = (2*points[-1][0] - points[-2][0], 2*points[-1][1] - points[-2][1])
    pts = [p0p] + list(points) + [pNp]

    def tj(ti, pi, pj):
        d = math.hypot(pj[0] - pi[0], pj[1] - pi[1])
        return ti + max(d, 1e-6) ** alpha

    def lerp(a, b, ta, tb, t):
        w = (tb - t) / (tb - ta)
        return (w*a[0] + (1-w)*b[0], w*a[1] + (1-w)*b[1])

    out = []
    for i in range(len(pts) - 3):
        p0, p1, p2, p3 = pts[i], pts[i+1], pts[i+2], pts[i+3]
        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)
        for k in range(samples_per_segment):
            u = t1 + (t2 - t1) * (k / samples_per_segment)
            A1 = lerp(p0, p1, t0, t1, u)
            A2 = lerp(p1, p2, t1, t2, u)
            A3 = lerp(p2, p3, t2, t3, u)
            B1 = lerp(A1, A2, t0, t2, u)
            B2 = lerp(A2, A3, t1, t3, u)
            out.append(lerp(B1, B2, t1, t2, u))
    out.append(points[-1])
    return out


def circle_intersections(robot_xy, radius_cm, spline, start_idx):
    """Find all (x, y, segment_idx) where the spline crosses the lookahead circle."""
    rx, ry = robot_xy
    out = []
    for i in range(start_idx, len(spline) - 1):
        a = spline[i]; b = spline[i + 1]
        da = math.hypot(a[0] - rx, a[1] - ry) - radius_cm
        db = math.hypot(b[0] - rx, b[1] - ry) - radius_cm
        if da * db <= 0 and (da != 0 or db != 0):
            t = 0.5 if abs(da - db) < 1e-9 else da / (da - db)
            ix = a[0] + t * (b[0] - a[0])
            iy = a[1] + t * (b[1] - a[1])
            direction = "exit" if da < 0 else "enter"
            out.append({"x": ix, "y": iy, "seg_idx": i, "dir": direction})
    return out


def find_lookahead_target(robot_xy, radius_cm, spline, start_idx):
    """Walk forward from start_idx and return the first exit from the lookahead circle.

    This avoids multi-intersection ambiguity by following path order: starting at
    start_idx, the first segment where the spline crosses from inside to outside the
    circle is the target.  Falls back to the old intersection scan if nothing is found
    (e.g. robot is already outside the circle the whole way).
    """
    rx, ry = robot_xy
    for i in range(start_idx, len(spline) - 1):
        a = spline[i]; b = spline[i + 1]
        da = math.hypot(a[0] - rx, a[1] - ry) - radius_cm
        db = math.hypot(b[0] - rx, b[1] - ry) - radius_cm
        if da <= 0 < db:   # inside → outside: this is the exit
            t = da / (da - db)
            return {"x": a[0] + t * (b[0] - a[0]),
                    "y": a[1] + t * (b[1] - a[1]),
                    "seg_idx": i, "dir": "exit"}
    # Fallback: collect all crossings and return the first exit
    xs = circle_intersections(robot_xy, radius_cm, spline, start_idx)
    for x in xs:
        if x["dir"] == "exit":
            return x
    return xs[-1] if xs else None
