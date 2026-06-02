"""Path-following controllers for differential-drive robots.

Usage
-----
    from robot_radio.controllers import Controller, CONTROLLERS
    cls = CONTROLLERS["pure_pursuit"]
    ctrl = cls(path=waypoints, trackwidth=9.0, base_speed=40.0)
    left, right = ctrl.compute(pos, yaw)
"""

from robot_radio.controllers.base import Controller
from robot_radio.controllers.pure_pursuit import PurePursuitTracker
from robot_radio.controllers.stanley import StanleyController
from robot_radio.controllers.ltv import LTVController

CONTROLLERS: dict[str, type[Controller]] = {
    "pure_pursuit": PurePursuitTracker,
    "stanley": StanleyController,
    "ltv": LTVController,
}

__all__ = [
    "Controller",
    "PurePursuitTracker",
    "StanleyController",
    "LTVController",
    "CONTROLLERS",
]
