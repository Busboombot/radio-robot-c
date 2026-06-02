"""Cubic Bezier path builder with C¹ continuity across waypoints.

Algorithm summary
-----------------
1. **Heading inference pre-pass**: Any waypoint with ``heading=None`` is
   assigned the chord-tangent heading: ``atan2(next.y − prev.y,
   next.x − prev.x)``.  The start and end poses always have explicit
   headings.

2. **Segment construction**: For each consecutive pair of poses
   ``(P0, P3)`` a cubic Bezier is built:

   - ``P1 = P0 + d * (cos θ0, sin θ0)``
   - ``P2 = P3 − d * (cos θ3, sin θ3)``

   where ``d = tangent_frac * chord_length(P0, P3)``.

   This guarantees C¹ continuity at shared waypoints: the outgoing
   tangent of segment *k* and the incoming tangent of segment *k+1* are
   both aligned to the waypoint heading.

3. **Dense sampling**: Each segment is sampled at ``n_raw`` evenly-spaced
   *t* values. ``n_raw`` is chosen so the raw step is at most
   ``spacing_cm / 4`` (oversampled to ensure accurate arc-length
   estimation).

4. **Arc-length resampling**: Cumulative chord lengths are computed;
   ``numpy.interp`` resamples positions and headings at uniform arc-length
   intervals of ``spacing_cm``.

5. The endpoints of the resampled path are forced to exactly match the
   start and end poses.

Registration
------------
Importing this module registers ``"bezier"`` in
``robot_radio.path.builder._REGISTRY``.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from robot_radio.path.builder import _REGISTRY
from robot_radio.path.sampled_path import SampledPath
from robot_radio.nav.pose import Pose, Waypoint

# Default fraction of chord length used for tangent control-point offset.
_DEFAULT_TANGENT_FRAC: float = 0.33

# Default arc-length spacing between output points in centimetres.
_DEFAULT_SPACING_CM: float = 1.0


def _chord(ax: float, ay: float, bx: float, by: float) -> float:
    """Euclidean distance between two points."""
    return math.hypot(bx - ax, by - ay)


def _infer_headings(
    all_poses: list[tuple[float, float, float | None]],
) -> list[float]:
    """Return headings for every pose, inferring None values from neighbours.

    Parameters
    ----------
    all_poses:
        List of ``(x, y, heading_or_None)``.  The first and last entries
        must have explicit headings (start/end Pose).

    Returns
    -------
    list[float]
        Headings in radians, same length as *all_poses*.
    """
    n = len(all_poses)
    headings: list[float] = []

    for i, (x, y, h) in enumerate(all_poses):
        if h is not None:
            headings.append(h)
        else:
            # Use the chord from previous to next (or nearest available).
            prev_x, prev_y = all_poses[i - 1][0], all_poses[i - 1][1]
            next_x, next_y = all_poses[i + 1][0], all_poses[i + 1][1]
            headings.append(math.atan2(next_y - prev_y, next_x - prev_x))

    return headings


def _cubic_bezier_segment(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    n_samples: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Evaluate a cubic Bezier segment at *n_samples* uniform t values.

    Returns
    -------
    xs, ys : np.ndarray
        Sampled x and y coordinates, shape ``(n_samples,)``.
    ts : np.ndarray
        The t values used, shape ``(n_samples,)``.
    """
    ts = np.linspace(0.0, 1.0, n_samples)
    one_m = 1.0 - ts

    xs = (
        one_m**3 * p0[0]
        + 3 * one_m**2 * ts * p1[0]
        + 3 * one_m * ts**2 * p2[0]
        + ts**3 * p3[0]
    )
    ys = (
        one_m**3 * p0[1]
        + 3 * one_m**2 * ts * p1[1]
        + 3 * one_m * ts**2 * p2[1]
        + ts**3 * p3[1]
    )
    return xs, ys, ts


def _bezier_tangent(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> float:
    """Heading of the Bezier tangent at parameter *t* in radians."""
    one_m = 1.0 - t
    dx = (
        3 * one_m**2 * (p1[0] - p0[0])
        + 6 * one_m * t * (p2[0] - p1[0])
        + 3 * t**2 * (p3[0] - p2[0])
    )
    dy = (
        3 * one_m**2 * (p1[1] - p0[1])
        + 6 * one_m * t * (p2[1] - p1[1])
        + 3 * t**2 * (p3[1] - p2[1])
    )
    return math.atan2(dy, dx)


class BezierPathBuilder:
    """Cubic Bezier path builder registered as ``"bezier"``.

    Parameters (passed as kwargs to ``__call__``)
    -----------------------------------------------
    spacing_cm : float
        Desired arc-length distance between output points (default 1.0 cm).
    tangent_frac : float
        Fraction of chord length used for control-point offset (default 0.33).
    """

    name: str = "bezier"

    def __call__(
        self,
        start: Pose,
        end: Pose,
        waypoints: list[Waypoint],
        spacing_cm: float = _DEFAULT_SPACING_CM,
        tangent_frac: float = _DEFAULT_TANGENT_FRAC,
        **kwargs: Any,
    ) -> SampledPath:
        """Build a C¹ cubic Bezier path from *start* to *end*.

        Parameters
        ----------
        start:
            Starting pose.
        end:
            Ending pose.
        waypoints:
            Intermediate waypoints (may be empty; headings may be None).
        spacing_cm:
            Arc-length step between output samples in centimetres.
        tangent_frac:
            Fraction of each segment's chord length used for control points.

        Returns
        -------
        SampledPath
        """
        # Build flat list of (x, y, heading_or_None).
        all_poses: list[tuple[float, float, float | None]] = [
            (start.x, start.y, start.heading)
        ]
        for wp in waypoints:
            all_poses.append((wp.x, wp.y, wp.heading))
        all_poses.append((end.x, end.y, end.heading))

        # Infer missing headings.
        headings = _infer_headings(all_poses)

        # Collect dense samples across all segments.
        raw_xs: list[float] = []
        raw_ys: list[float] = []
        raw_hs: list[float] = []

        for i in range(len(all_poses) - 1):
            x0, y0 = all_poses[i][0], all_poses[i][1]
            x3, y3 = all_poses[i + 1][0], all_poses[i + 1][1]
            h0 = headings[i]
            h3 = headings[i + 1]

            chord = _chord(x0, y0, x3, y3)
            d = tangent_frac * chord if chord > 1e-9 else 0.0

            p0 = (x0, y0)
            p1 = (x0 + d * math.cos(h0), y0 + d * math.sin(h0))
            p2 = (x3 - d * math.cos(h3), y3 - d * math.sin(h3))
            p3 = (x3, y3)

            # Oversample by 4× relative to desired spacing so arc-length
            # interpolation is accurate.
            n_raw = max(4, int(math.ceil(chord / spacing_cm * 4)) + 1)

            xs, ys, ts = _cubic_bezier_segment(p0, p1, p2, p3, n_raw)

            # Compute tangent headings at each sample.
            seg_hs = [_bezier_tangent(p0, p1, p2, p3, float(t)) for t in ts]

            # Skip the first point of segments after the first to avoid
            # duplicating waypoint positions.
            start_idx = 0 if i == 0 else 1
            raw_xs.extend(xs[start_idx:].tolist())
            raw_ys.extend(ys[start_idx:].tolist())
            raw_hs.extend(seg_hs[start_idx:])

        if len(raw_xs) < 2:
            # Degenerate path — start == end.
            return SampledPath(
                points=[(raw_xs[0], raw_ys[0])],
                headings=[raw_hs[0]],
                builder_name=self.name,
                total_length_cm=0.0,
            )

        # Compute cumulative arc length along the raw samples.
        raw_x_arr = np.array(raw_xs)
        raw_y_arr = np.array(raw_ys)
        raw_h_arr = np.array(raw_hs)

        diffs_x = np.diff(raw_x_arr)
        diffs_y = np.diff(raw_y_arr)
        chord_lens = np.hypot(diffs_x, diffs_y)
        cum_arc = np.concatenate([[0.0], np.cumsum(chord_lens)])
        total_len = float(cum_arc[-1])

        if total_len < 1e-9:
            return SampledPath(
                points=[(raw_xs[0], raw_ys[0])],
                headings=[raw_hs[0]],
                builder_name=self.name,
                total_length_cm=0.0,
            )

        # Resample at uniform arc-length intervals.
        n_out = max(2, int(round(total_len / spacing_cm)) + 1)
        target_arcs = np.linspace(0.0, total_len, n_out)

        out_x = np.interp(target_arcs, cum_arc, raw_x_arr)
        out_y = np.interp(target_arcs, cum_arc, raw_y_arr)
        # Unwrap headings before interpolation to avoid wrap-around jumps.
        unwrapped_h = np.unwrap(raw_h_arr)
        out_h_unwrapped = np.interp(target_arcs, cum_arc, unwrapped_h)
        # Re-wrap to [-π, π].
        out_h = (out_h_unwrapped + math.pi) % (2 * math.pi) - math.pi

        # Force exact endpoints.
        out_x[0], out_y[0] = all_poses[0][0], all_poses[0][1]
        out_h[0] = headings[0]
        out_x[-1], out_y[-1] = all_poses[-1][0], all_poses[-1][1]
        out_h[-1] = headings[-1]

        points = list(zip(out_x.tolist(), out_y.tolist()))
        return SampledPath(
            points=points,
            headings=out_h.tolist(),
            builder_name=self.name,
            total_length_cm=total_len,
        )


# Register the builder on module import.
_BUILDER_INSTANCE = BezierPathBuilder()
_REGISTRY["bezier"] = _BUILDER_INSTANCE
