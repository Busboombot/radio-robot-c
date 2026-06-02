"""Shared geometric operations on a polyline path.

All coordinates are in centimetres (project convention).

``Path`` provides nearest-point search (O(1) amortised via monotone caching),
arc-length lookahead, segment tangent, and total length queries.  Both
``PurePursuitTracker`` and ``StanleyController`` use this class internally.
"""

from __future__ import annotations

import math
from typing import Sequence


class Path:
    """A polyline path defined by an ordered sequence of (x, y) waypoints.

    Parameters
    ----------
    points:
        At least two (x, y) waypoints in centimetres.  Order defines the
        direction of travel.

    Notes
    -----
    ``nearest_point`` maintains an internal ``_last_idx`` cache.  Once an
    index is returned it is never decreased on subsequent calls, so the
    search is O(n) in the worst case (first call) and O(1) amortised as the
    robot advances along the path.  Call ``reset()`` whenever a new path is
    loaded or the robot is teleported to a position behind the last cached
    index.
    """

    def __init__(self, points: Sequence[tuple[float, float]]) -> None:
        if len(points) < 2:
            raise ValueError("Path must contain at least two waypoints")
        self._points: list[tuple[float, float]] = list(points)
        self._last_idx: int = 0

    # ------------------------------------------------------------------
    # Cached nearest-point search
    # ------------------------------------------------------------------

    def nearest_point(self, x: float, y: float) -> tuple[int, float, float]:
        """Return the nearest point on the path to ``(x, y)``.

        The search starts from the cached segment index and scans forward
        only, so the returned index never decreases between calls (monotone
        advance property).

        Parameters
        ----------
        x, y:
            Query position in centimetres.

        Returns
        -------
        (segment_index, px, py):
            ``segment_index`` is the index into ``points`` of the *start*
            of the nearest segment.  ``(px, py)`` is the closest point on
            that segment to ``(x, y)``.
        """
        pts = self._points
        n = len(pts)
        best_dist_sq = math.inf
        best_idx = self._last_idx
        best_px, best_py = pts[best_idx]

        for i in range(self._last_idx, n - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            dx = bx - ax
            dy = by - ay
            seg_len_sq = dx * dx + dy * dy

            if seg_len_sq < 1e-12:
                # Degenerate zero-length segment — use the start point
                px, py = ax, ay
            else:
                # Project (x, y) onto segment, clamping t to [0, 1]
                t = ((x - ax) * dx + (y - ay) * dy) / seg_len_sq
                t = max(0.0, min(1.0, t))
                px = ax + t * dx
                py = ay + t * dy

            dist_sq = (x - px) ** 2 + (y - py) ** 2
            if dist_sq <= best_dist_sq:
                # Use <= so that when two segments are equidistant the later
                # (more advanced) segment wins.  This ensures the cache
                # advances past corner vertices when the robot is at the
                # corner point — both flanking segments project to the same
                # vertex, and we want to track the outgoing segment.
                best_dist_sq = dist_sq
                best_idx = i
                best_px, best_py = px, py

        # Advance the monotone cache
        if best_idx > self._last_idx:
            self._last_idx = best_idx

        return (best_idx, best_px, best_py)

    # ------------------------------------------------------------------
    # Lookahead
    # ------------------------------------------------------------------

    def point_at_lookahead(
        self, x: float, y: float, L: float
    ) -> tuple[float, float]:
        """Return the point arc-length *L* ahead of the nearest point.

        Finds the nearest point on the path, then walks forward segment by
        segment accumulating arc length until *L* centimetres are consumed
        or the path ends.

        Parameters
        ----------
        x, y:
            Query (robot) position in centimetres.
        L:
            Lookahead distance in centimetres.  Must be non-negative.

        Returns
        -------
        (px, py):
            The lookahead point in centimetres.  Returns the final waypoint
            when the remaining path length is less than *L*.
        """
        pts = self._points
        seg_idx, nx, ny = self.nearest_point(x, y)

        remaining = L
        cur_x, cur_y = nx, ny

        # Walk from the nearest point along segment seg_idx, then forward
        for i in range(seg_idx, len(pts) - 1):
            # End of the current segment
            end_x, end_y = pts[i + 1]
            seg_dx = end_x - cur_x
            seg_dy = end_y - cur_y
            seg_len = math.hypot(seg_dx, seg_dy)

            if seg_len < 1e-9:
                # Zero-length segment; skip without consuming lookahead
                cur_x, cur_y = end_x, end_y
                continue

            if remaining <= seg_len:
                # Lookahead point is on this segment
                frac = remaining / seg_len
                return (cur_x + frac * seg_dx, cur_y + frac * seg_dy)

            remaining -= seg_len
            cur_x, cur_y = end_x, end_y

        # L exceeds remaining path — return the final waypoint
        return pts[-1]

    # ------------------------------------------------------------------
    # Tangent
    # ------------------------------------------------------------------

    def tangent_at(self, idx: int) -> float:
        """Return the heading (radians) of segment *idx*.

        Convention: 0 = east (+x direction), π/2 = north (+y direction),
        matching the project's standard math yaw convention.

        If *idx* is beyond the last valid segment, the last segment's
        heading is returned.

        Parameters
        ----------
        idx:
            Segment index (0-based).  Segment *i* runs from ``points[i]``
            to ``points[i+1]``.
        """
        pts = self._points
        # Clamp to valid segment range
        i = min(idx, len(pts) - 2)
        i = max(0, i)
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        return math.atan2(by - ay, bx - ax)

    # ------------------------------------------------------------------
    # Length
    # ------------------------------------------------------------------

    def length(self) -> float:
        """Return the total arc length of the path in centimetres."""
        pts = self._points
        total = 0.0
        for i in range(len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            total += math.hypot(bx - ax, by - ay)
        return total

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Clear the cached segment index.

        Call this when a new path is loaded or when the robot is teleported
        to a position that may be behind the current cached segment.
        """
        self._last_idx = 0

    # ------------------------------------------------------------------
    # Dunder helpers
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return len(self._points)

    def __repr__(self) -> str:  # pragma: no cover
        return f"Path(n={len(self._points)}, length={self.length():.1f} cm)"


# ---------------------------------------------------------------------------
# Smoke test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Two-point horizontal path from (0, 0) to (10, 0)
    p = Path([(0.0, 0.0), (10.0, 0.0)])
    assert p.length() == 10.0, f"expected 10.0, got {p.length()}"
    print(f"length() = {p.length()} cm  [OK]")

    # nearest_point at origin should be (0, 0, 0)
    idx, px, py = p.nearest_point(0.0, 0.0)
    assert idx == 0 and px == 0.0 and py == 0.0, f"got ({idx}, {px}, {py})"
    print(f"nearest_point(0,0) = ({idx}, {px}, {py})  [OK]")

    # lookahead of 5 from origin should be (5, 0)
    lx, ly = p.point_at_lookahead(0.0, 0.0, 5.0)
    assert abs(lx - 5.0) < 1e-9 and abs(ly) < 1e-9, f"got ({lx}, {ly})"
    print(f"point_at_lookahead(0,0,5) = ({lx}, {ly})  [OK]")

    # tangent should be 0.0 (east)
    theta = p.tangent_at(0)
    assert abs(theta) < 1e-9, f"got {theta}"
    print(f"tangent_at(0) = {theta} rad  [OK]")

    print("All smoke tests passed.")
