"""robot_radio.sensors — sensor abstraction subpackage.

Re-exports all public names so callers can use either:
    from robot_radio.sensors import Otos, Odometry
or the fully-qualified form:
    from robot_radio.sensors.otos import Otos
"""

from robot_radio.sensors.otos import Otos
from robot_radio.sensors.odometry import Odometry
from robot_radio.sensors.cam_tracker import CamTracker
from robot_radio.sensors.odom_tracker import OdomTracker, parse_so
from robot_radio.sensors.color import ColorClassifier, nezha_classifier, calibrate_white
from robot_radio.sensors.motion_monitor import ThrashMonitor
from robot_radio.sensors.calibration import (
    CalibrationError,
    load,
    to_wire_values,
    apply,
    load_and_apply,
)

__all__ = [
    "Otos",
    "Odometry",
    "CamTracker",
    "OdomTracker",
    "parse_so",
    "ColorClassifier",
    "nezha_classifier",
    "calibrate_white",
    "ThrashMonitor",
    "CalibrationError",
    "load",
    "to_wire_values",
    "apply",
    "load_and_apply",
]
