"""Path generation and geometry subpackage.

Public API
----------
- ``build_path`` — registry lookup + dispatch function.
- ``SampledPath`` — frozen dataclass holding the output polyline.
- ``catmull_rom`` — centripetal Catmull-Rom spline.
- ``plan_path`` — visibility-graph obstacle-avoiding path planner.
- ``build_safe_spline`` — obstacle-avoiding Catmull-Rom spline builder.
- ``four_leaf_waypoints`` — 4-petal cloverleaf waypoint generator.
- ``BezierPathBuilder`` — lazy import (requires numpy).

Importing ``bezier`` triggers the ``BezierPathBuilder`` registration in the
builder registry.  The import is deferred so that the subpackage is importable
without numpy.
"""

from robot_radio.path.builder import build_path
from robot_radio.path.sampled_path import SampledPath
from robot_radio.path.catmull_rom import catmull_rom
from robot_radio.path.obstacle import plan_path, build_safe_spline
from robot_radio.path.patterns import four_leaf_waypoints


def __getattr__(name: str):
    """Lazy import for numpy-dependent submodules."""
    if name in ("bezier", "BezierPathBuilder"):
        from robot_radio.path import bezier as _bezier  # noqa: F401
        return _bezier
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "build_path",
    "SampledPath",
    "catmull_rom",
    "plan_path",
    "build_safe_spline",
    "four_leaf_waypoints",
    "BezierPathBuilder",  # lazy
]
