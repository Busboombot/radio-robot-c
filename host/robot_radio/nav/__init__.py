"""robot_radio.nav — navigation subpackage.

Re-exports the public navigation API so callers can use:

    from robot_radio.nav import Navigator, NavParams, Pose, Waypoint, PoseAlign
    from robot_radio.nav import camera_goto
"""

from robot_radio.nav.nav_params import NavParams
from robot_radio.nav.pose import Pose, Waypoint
from robot_radio.nav.pose_align import align_otos_to_camera as PoseAlign
from robot_radio.nav import camera_goto


def __getattr__(name: str):
    """Lazy import for aprilcam-dependent Navigator."""
    if name == "Navigator":
        from robot_radio.nav.navigator import Navigator
        return Navigator
    if name == "log_record":
        from robot_radio.nav.navigator import log_record
        return log_record
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
