"""robot_radio.sensors — sensor abstraction subpackage.

Re-exports all public names so callers can use either::

    from robot_radio.sensors import OdomTracker
    from robot_radio.sensors import CamTracker     # lazy — cv2/grpc not loaded until here

Modules that pull in large optional dependencies (``cam_tracker`` → cv2,
aprilcam, grpc; ``otos`` → nav.pose) are loaded lazily via ``__getattr__``
so that a bare ``import robot_radio.sensors`` does NOT import those trees.

Heavy-import cost (cv2 ≈ 132 MB, grpc) is deferred until the name is
actually accessed.  The fully-qualified import forms also work without
triggering this module at all::

    from robot_radio.sensors.cam_tracker import CamTracker  # direct, still lazy

Eagerly-available names (no heavy deps):
    OdomTracker, parse_so, parse_tlm,
    ColorClassifier, nezha_classifier, calibrate_white,
    ThrashMonitor,
    CalibrationError, load, to_wire_values, apply, load_and_apply,
    Odometry

Lazily-available names (loaded on first access):
    CamTracker    — requires aprilcam / grpc / cv2
    Otos          — requires robot_radio.nav.pose (acceptable; isolated)
"""

from __future__ import annotations

from robot_radio.sensors.odometry import Odometry
from robot_radio.sensors.odom_tracker import OdomTracker, parse_so, parse_tlm
from robot_radio.sensors.color import ColorClassifier, nezha_classifier, calibrate_white
from robot_radio.sensors.motion_monitor import ThrashMonitor
from robot_radio.sensors.calibration import (
    CalibrationError,
    load,
    to_wire_values,
    apply,
    load_and_apply,
)

# Names that are always available
__all__ = [
    "Odometry",
    "OdomTracker",
    "parse_so",
    "parse_tlm",
    "ColorClassifier",
    "nezha_classifier",
    "calibrate_white",
    "ThrashMonitor",
    "CalibrationError",
    "load",
    "to_wire_values",
    "apply",
    "load_and_apply",
    # Lazy names — included in __all__ for documentation; loaded via __getattr__
    "CamTracker",
    "Otos",
]

_LAZY = {
    "CamTracker": ("robot_radio.sensors.cam_tracker", "CamTracker"),
    "Otos": ("robot_radio.sensors.otos", "Otos"),
}


def __getattr__(name: str):
    """Lazy import for heavy-dependency submodules."""
    if name in _LAZY:
        module_path, attr = _LAZY[name]
        import importlib
        mod = importlib.import_module(module_path)
        obj = getattr(mod, attr)
        # Cache in module globals to avoid repeated __getattr__ calls
        globals()[name] = obj
        return obj
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
