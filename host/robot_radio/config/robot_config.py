"""Per-robot configuration loader and validator.

This is a pure-Python leaf module with no imports from other robot_radio
modules. It loads and validates per-robot JSON config files, computes
derived encoder fields, and exposes a cached singleton via get_robot_config().

Resolution order for get_robot_config():
1. ROBOT_CONFIG env var — full path to a JSON config file.
2. data/robots/active_robot.json — either a full config (has 'identity' key),
   a symlink to one, or a pointer file with a path key.
3. Returns None with a logged WARNING if neither is found.
"""

import logging
import math
import os
from pathlib import Path
from typing import Optional

from pydantic import BaseModel, model_validator

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).parent.parent.parent


# ---------------------------------------------------------------------------
# Nested config models
# ---------------------------------------------------------------------------

class OffsetXY(BaseModel):
    x: float = 0.0
    y: float = 0.0


class OffsetXYYaw(BaseModel):
    x: float = 0.0
    y: float = 0.0
    yaw_rad: float = 0.0


class IdentityConfig(BaseModel):
    robot_name: str
    uid: str
    hardware_model: str = ""
    common_name: str = ""


class ConnectionConfig(BaseModel):
    device_announcement_name: str = ""
    serial_last_6: str = ""
    i2c_addresses: dict[str, int] = {}


class VisionConfig(BaseModel):
    robot_tag_id: int = 1
    tag_offset_mm: OffsetXYYaw = OffsetXYYaw()


class GeometryConfig(BaseModel):
    drive_axle_offset_mm: OffsetXY = OffsetXY()
    odometry_offset_mm: OffsetXYYaw = OffsetXYYaw()
    # True when the OTOS chip is mounted upside-down (Z-axis flipped).
    # The chip's natural frame is X-forward, Y-left, Z-up; flipping it
    # over (so Z points to the floor) effectively inverts the X axis
    # in the chip's reported position relative to robot motion.  When
    # this is true, firmware negates chip_x before applying yaw rotation
    # and translation offset.
    odometry_chip_upside_down: bool = False
    trackwidth: Optional[float] = None
    wheelbase_mm: Optional[float] = None


class WheelsConfig(BaseModel):
    wheel_diameter_mm: Optional[float] = None
    ticks_per_rev: Optional[float] = None
    ticks_per_mm: Optional[float] = None


class EncodersConfig(BaseModel):
    has_encoders: bool = False
    encoder_count: int = 0


class DriveConfig(BaseModel):
    motor_deadband: Optional[float] = None
    max_cmd: Optional[float] = None
    cmd_to_mm_per_s: Optional[float] = None
    max_drive_mm_s: Optional[float] = None
    max_turn_deg_s: Optional[float] = None
    min_drive_mm_s: Optional[float] = None
    crawl_threshold_mm_s: Optional[float] = None
    crawl_cmd: Optional[float] = None


class GripperConfig(BaseModel):
    has_gripper: bool = False
    gripper_offset_mm: Optional[OffsetXY] = None


class CalibrationConfig(BaseModel):
    """Sensor and odometry calibration values measured on this specific robot.

    Scales are stored as float multipliers (e.g. 0.9910, 1.050).
    The int8 encoding for the OTOS firmware register is derived on demand:
      scalar = round((scale - 1.0) / 0.001), clamped to -128..127
    """
    otos_angular_scale: float = 1.0
    otos_linear_scale:  float = 1.0
    # Per-wheel encoder calibration overrides. When set, they take precedence
    # over the wheel_diameter-derived value in _sync_calibration. When None,
    # _sync_calibration derives from wheels.wheel_diameter_mm as before.
    mm_per_wheel_deg_left:  Optional[float] = None
    mm_per_wheel_deg_right: Optional[float] = None
    # Body-rotation efficiency: actual_body_rotation_rad / no_slip_estimate_rad
    # where no_slip_estimate = 2 * arc_per_wheel / trackwidth.
    # 1.0 = no slip. 0.75 = robot only rotates 75% of what wheel arc predicts
    # (= wheel slippage during in-place turns). Used by rogo turn open-loop math.
    rotational_slip: Optional[float] = None
    # Linear correction model for rogo turn: actual_deg = gain × commanded + offset.
    # _turn_command compensates by sending an effective command of
    #   (target_deg - offset) / gain
    # Allows capturing both startup loss (offset) and proportional under/overshoot
    # (gain) that a single slip factor can't model. Defaults: gain=1.0, offset=0.
    rotation_gain:       Optional[float] = None
    rotation_offset_deg: Optional[float] = None
    # Separate parameters for negative (CW) turns when present, since motor
    # response can be asymmetric. When None, the positive params are used.
    rotation_gain_neg:       Optional[float] = None
    rotation_offset_deg_neg: Optional[float] = None

    @property
    def otos_angular_scalar(self) -> int:
        return max(-128, min(127, round((self.otos_angular_scale - 1.0) / 0.001)))

    @property
    def otos_linear_scalar(self) -> int:
        return max(-128, min(127, round((self.otos_linear_scale - 1.0) / 0.001)))


# ---------------------------------------------------------------------------
# Root config model
# ---------------------------------------------------------------------------

class RobotConfig(BaseModel):
    schema_version: int = 1
    identity: IdentityConfig
    connection: ConnectionConfig = ConnectionConfig()
    vision: VisionConfig = VisionConfig()
    geometry: GeometryConfig = GeometryConfig()
    wheels: WheelsConfig = WheelsConfig()
    encoders: EncodersConfig = EncodersConfig()
    drive: DriveConfig = DriveConfig()
    gripper: GripperConfig = GripperConfig()
    calibration: CalibrationConfig = CalibrationConfig()

    # Derived field — not stored in JSON, computed after load
    mm_per_tick: Optional[float] = None

    @model_validator(mode="after")
    def _resolve_encoder_fields(self) -> "RobotConfig":
        if not self.encoders.has_encoders:
            return self
        w = self.wheels
        wd, tpr, tpm = w.wheel_diameter_mm, w.ticks_per_rev, w.ticks_per_mm
        present = sum(v is not None for v in (wd, tpr, tpm))

        if present == 3:
            assert wd and tpr and tpm
            expected = tpr / (math.pi * wd)
            if abs(expected - tpm) / tpm > 0.01:
                raise ValueError(
                    f"Encoder fields inconsistent: wheel_diameter_mm={wd}, "
                    f"ticks_per_rev={tpr}, ticks_per_mm={tpm} (given) vs "
                    f"{expected:.6f} (computed). Exceeds 1% tolerance."
                )
            self.mm_per_tick = 1.0 / tpm

        elif present == 2:
            if wd is None:
                assert tpr and tpm
                w.wheel_diameter_mm = tpr / (math.pi * tpm)
            elif tpr is None:
                assert tpm and wd
                w.ticks_per_rev = tpm * math.pi * wd
            else:
                assert wd and tpr
                w.ticks_per_mm = tpr / (math.pi * wd)
            if w.ticks_per_mm is not None:
                self.mm_per_tick = 1.0 / w.ticks_per_mm

        elif w.ticks_per_mm is not None:
            self.mm_per_tick = 1.0 / w.ticks_per_mm

        return self

    # ------------------------------------------------------------------
    # Convenience flat accessors (avoids updating all call sites)
    # ------------------------------------------------------------------

    @property
    def robot_name(self) -> str:
        return self.identity.robot_name

    @property
    def uid(self) -> str:
        return self.identity.uid

    @property
    def hardware_model(self) -> str:
        return self.identity.hardware_model

    @property
    def common_name(self) -> str:
        return self.identity.common_name

    @property
    def robot_tag_id(self) -> int:
        return self.vision.robot_tag_id

    @property
    def tag_offset_mm(self) -> OffsetXYYaw:
        return self.vision.tag_offset_mm

    @property
    def trackwidth(self) -> Optional[float]:
        return self.geometry.trackwidth

    @property
    def motor_deadband(self) -> Optional[float]:
        return self.drive.motor_deadband

    @property
    def has_gripper(self) -> bool:
        return self.gripper.has_gripper

    @property
    def gripper_offset_mm(self) -> Optional[OffsetXY]:
        return self.gripper.gripper_offset_mm

    @property
    def device_announcement_name(self) -> str:
        return self.connection.device_announcement_name

    @property
    def serial_last_6(self) -> str:
        return self.connection.serial_last_6

    @property
    def has_encoders(self) -> bool:
        return self.encoders.has_encoders

    @property
    def otos_angular_scale(self) -> float:
        return self.calibration.otos_angular_scale

    @property
    def otos_angular_scalar(self) -> int:
        return self.calibration.otos_angular_scalar

    @property
    def otos_linear_scale(self) -> float:
        return self.calibration.otos_linear_scale

    @property
    def otos_linear_scalar(self) -> int:
        return self.calibration.otos_linear_scalar


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def load_robot_config(path: "str | Path") -> RobotConfig:
    """Load and validate a robot config JSON file.

    Raises:
        FileNotFoundError: if the file does not exist.
        pydantic.ValidationError: if the file fails schema validation.
        json.JSONDecodeError: if the file is not valid JSON.
    """
    path = Path(path)
    cfg = RobotConfig.model_validate_json(path.read_text())
    logger.info("Loaded robot config: robot_name=%r path=%s", cfg.robot_name, path)
    return cfg


# ---------------------------------------------------------------------------
# Cached singleton
# ---------------------------------------------------------------------------

_config_cache: Optional[RobotConfig] = None
_cache_loaded: bool = False


def _reset_robot_config() -> None:
    """Clear the cached singleton. Intended for testing only."""
    global _config_cache, _cache_loaded
    _config_cache = None
    _cache_loaded = False


def get_robot_config() -> Optional[RobotConfig]:
    """Return the cached RobotConfig singleton.

    Resolution order:
    1. ROBOT_CONFIG env var — treated as a full path to the JSON file.
    2. data/robots/active_robot.json — full config, symlink, or pointer file.
    3. Returns None with a logged WARNING if neither is found.
    """
    global _config_cache, _cache_loaded
    if _cache_loaded:
        return _config_cache

    _cache_loaded = True

    # 1. ROBOT_CONFIG env var
    env_path = os.environ.get("ROBOT_CONFIG")
    if env_path:
        config_path = Path(env_path)
        if not config_path.is_absolute():
            config_path = _PROJECT_ROOT / config_path
        try:
            _config_cache = load_robot_config(config_path)
            return _config_cache
        except FileNotFoundError:
            logger.warning("ROBOT_CONFIG env var points to missing file: %s", config_path)
            return None

    # 2. data/robots/active_robot.json
    active_path = _PROJECT_ROOT / "data" / "robots" / "active_robot.json"
    if active_path.exists():
        try:
            import json
            pointer = json.loads(active_path.read_text())
        except Exception as e:
            logger.warning("Failed to read active_robot.json: %s", e)
            return None

        if "identity" in pointer or "schema_version" in pointer:
            # Full config (or symlink target)
            try:
                _config_cache = load_robot_config(active_path)
                return _config_cache
            except Exception as e:
                logger.warning("Failed to load active_robot.json as config: %s", e)
                return None

        if "path" in pointer:
            target = _PROJECT_ROOT / pointer["path"]
            try:
                _config_cache = load_robot_config(target)
                return _config_cache
            except FileNotFoundError:
                logger.warning(
                    "active_robot.json path pointer points to missing file: %s", target
                )
                return None

        logger.warning(
            "active_robot.json has neither 'identity' nor 'path' key — "
            "cannot resolve robot config"
        )
        return None

    logger.warning(
        "No robot config found. Set ROBOT_CONFIG env var or create "
        "data/robots/active_robot.json"
    )
    return None
