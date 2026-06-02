"""Pose and Waypoint dataclasses for path-planning.

Coordinate convention: x/y in centimetres (world frame), heading in
radians (East = 0, counter-clockwise positive — standard maths convention
matches the camera homography output).
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    """A fully-specified position and orientation in world frame.

    Parameters
    ----------
    x:
        Horizontal position in centimetres.
    y:
        Vertical position in centimetres.
    heading:
        Orientation in radians.  East = 0, counter-clockwise positive.
    """

    x: float
    y: float
    heading: float


@dataclass(frozen=True)
class Waypoint:
    """A position constraint, optionally with a desired heading.

    When *heading* is ``None`` the path builder infers a heading from the
    chord tangent between the preceding and following poses.

    Parameters
    ----------
    x:
        Horizontal position in centimetres.
    y:
        Vertical position in centimetres.
    heading:
        Optional orientation in radians.  ``None`` means "infer from context".
    """

    x: float
    y: float
    heading: float | None = None


def heading_error(a: float, b: float) -> float:
    """Return the signed angular difference *b − a* wrapped to [-π, π].

    Parameters
    ----------
    a:
        Reference heading in radians.
    b:
        Target heading in radians.

    Returns
    -------
    float
        Angle in [-π, π] representing the shortest rotation from *a* to *b*.
    """
    diff = b - a
    return (diff + math.pi) % (2 * math.pi) - math.pi
