"""SampledPath — the output of every PathBuilder."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SampledPath:
    """A polyline with per-point headings produced by a PathBuilder.

    Parameters
    ----------
    points:
        Ordered list of ``(x, y)`` positions in centimetres.  The first
        point is the start pose and the last point is the end pose.
    headings:
        Tangent heading (radians) at each point.  Parallel to *points*;
        same length.
    builder_name:
        Identifies which builder produced this path (e.g. ``"bezier"``).
    total_length_cm:
        Approximate arc length of the path in centimetres, computed from
        the sum of chord lengths between consecutive points.
    """

    points: list[tuple[float, float]]
    headings: list[float]
    builder_name: str
    total_length_cm: float

    def to_dict(self) -> dict:
        """Serialize to a JSON-compatible dict.

        Returns
        -------
        dict
            Keys: ``points`` (list of [x, y] pairs), ``headings`` (list of
            float), ``builder_name`` (str), ``total_length_cm`` (float).
        """
        return {
            "points": [list(p) for p in self.points],
            "headings": list(self.headings),
            "builder_name": self.builder_name,
            "total_length_cm": self.total_length_cm,
        }
