"""robot_radio.config — per-robot configuration subpackage.

Re-exports all public names from robot_config so callers can use either:
    from robot_radio.config import get_robot_config, RobotConfig
or the fully-qualified form:
    from robot_radio.config.robot_config import get_robot_config, RobotConfig
"""

from robot_radio.config.robot_config import (
    CalibrationConfig,
    ConnectionConfig,
    DriveConfig,
    EncodersConfig,
    GeometryConfig,
    GripperConfig,
    IdentityConfig,
    OffsetXY,
    OffsetXYYaw,
    RobotConfig,
    VisionConfig,
    WheelsConfig,
    _reset_robot_config,
    get_robot_config,
    load_robot_config,
)

__all__ = [
    "CalibrationConfig",
    "ConnectionConfig",
    "DriveConfig",
    "EncodersConfig",
    "GeometryConfig",
    "GripperConfig",
    "IdentityConfig",
    "OffsetXY",
    "OffsetXYYaw",
    "RobotConfig",
    "VisionConfig",
    "WheelsConfig",
    "_reset_robot_config",
    "get_robot_config",
    "load_robot_config",
]
