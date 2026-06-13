"""Path-following controllers for differential-drive robots.

Usage
-----
    from robot_radio.controllers import Controller
    from robot_radio.controllers.pid import PID

After ticket 035-002 (pose-authority sprint A1), the host-side steering
controllers (PurePursuitTracker, StanleyController, LTVController) have been
deleted.  Navigation is now delegated to the firmware G command.  Only the PID
helper (used by the speed loop) is retained.
"""

from robot_radio.controllers.base import Controller


__all__ = [
    "Controller",
]
