"""Path generation and geometry subpackage.

Public API
----------
- ``build_path`` — registry lookup + dispatch function.
- ``SampledPath`` — frozen dataclass holding the output polyline.
- ``catmull_rom`` — centripetal Catmull-Rom spline.
- ``plan_path`` — visibility-graph obstacle-avoiding path planner.
- ``build_safe_spline`` — obstacle-avoiding Catmull-Rom spline builder.
- ``four_leaf_waypoints`` — 4-petal cloverleaf waypoint generator.

Importing this package automatically imports ``bezier``, which registers
``BezierPathBuilder`` under the ``"bezier"`` key in the builder registry.
"""

from robot_radio.path.builder import build_path
from robot_radio.path.sampled_path import SampledPath
from robot_radio.path.catmull_rom import catmull_rom
from robot_radio.path.obstacle import plan_path, build_safe_spline
from robot_radio.path.patterns import four_leaf_waypoints
from robot_radio.path import bezier as _bezier  # registers "bezier" # noqa: F401

__all__ = [
    "build_path",
    "SampledPath",
    "catmull_rom",
    "plan_path",
    "build_safe_spline",
    "four_leaf_waypoints",
]
