"""Path-following controllers for differential-drive robots.

Usage
-----
    from robot_radio.controllers import Controller, CONTROLLERS
    cls = CONTROLLERS["pure_pursuit"]
    ctrl = cls(path=waypoints, trackwidth=9.0, base_speed=40.0)
    left, right = ctrl.compute(pos, yaw)

Optional controllers (require wpimath + numpy):
  - LTVController — available via lazy import or CONTROLLERS["ltv"].
"""

from robot_radio.controllers.base import Controller
from robot_radio.controllers.pure_pursuit import PurePursuitTracker
from robot_radio.controllers.stanley import StanleyController

CONTROLLERS: dict[str, type[Controller]] = {
    "pure_pursuit": PurePursuitTracker,
    "stanley": StanleyController,
}


def __getattr__(name: str):
    """Lazy import for wpimath/numpy-dependent controllers."""
    if name == "LTVController":
        from robot_radio.controllers.ltv import LTVController
        CONTROLLERS["ltv"] = LTVController
        return LTVController
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "Controller",
    "PurePursuitTracker",
    "StanleyController",
    "LTVController",  # lazy
    "CONTROLLERS",
]
