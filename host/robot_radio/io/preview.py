"""Preview module — send a polyline to the AprilCam draw layer.

This is a stub.  The AprilCam draw tool does not exist yet.  When it
ships, this is the single place to wire the call: replace the log
statement in ``preview_polyline`` with the actual MCP/API call.
"""

from __future__ import annotations

import logging

_log = logging.getLogger(__name__)


def preview_polyline(points: list[tuple[float, float]]) -> dict:
    """Log a polyline and return a stub response.

    Parameters
    ----------
    points:
        Ordered list of ``(x, y)`` positions in centimetres.

    Returns
    -------
    dict
        ``{"preview": "stubbed", "points": N}`` where *N* is the point count.
    """
    n = len(points)
    if n >= 2:
        total_len = sum(
            ((points[i + 1][0] - points[i][0]) ** 2
             + (points[i + 1][1] - points[i][1]) ** 2) ** 0.5
            for i in range(n - 1)
        )
    else:
        total_len = 0.0

    _log.info(
        "preview_polyline: %d points, total length %.2f cm "
        "(AprilCam draw tool not yet available — stubbed)",
        n,
        total_len,
    )
    print(
        f"[preview] {n} points, total length {total_len:.2f} cm "
        "(stubbed — AprilCam draw tool not yet available)"
    )
    return {"preview": "stubbed", "points": n}
