#!/usr/bin/env python3
"""Generate source/robot/DefaultConfig.cpp from the active robot JSON config.

Run:  python3 scripts/gen_default_config.py

Reads the active robot config (via data/robots/active_robot.json or
ROBOT_CONFIG env var) and writes source/robot/DefaultConfig.cpp with a
defaultRobotConfig() that has per-robot calibration values baked in.

When no robot config is found, falls back to hardcoded defaults so the
build always succeeds.
"""

import json
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_FILE  = REPO_ROOT / "source" / "robot" / "DefaultConfig.cpp"
SCHEMA_FILE = REPO_ROOT / "data" / "robots" / "robot_config.schema.json"


# ---------------------------------------------------------------------------
# Config resolution
# ---------------------------------------------------------------------------

def _load_json(path: Path):
    return json.loads(path.read_text())


def load_robot_config():
    """Return (config_dict, source_path_str) or ({}, '(hardcoded defaults)')."""
    env_path = os.environ.get("ROBOT_CONFIG")
    if env_path:
        p = Path(env_path)
        if not p.is_absolute():
            p = REPO_ROOT / p
        try:
            return _load_json(p), str(p)
        except Exception as e:
            print(f"gen_default_config: ROBOT_CONFIG={p} unreadable: {e}", file=sys.stderr)

    active = REPO_ROOT / "data" / "robots" / "active_robot.json"
    if active.exists():
        try:
            data = _load_json(active)
        except Exception as e:
            print(f"gen_default_config: {active} unreadable: {e}", file=sys.stderr)
            return {}, "(hardcoded defaults)"

        if "identity" in data or "schema_version" in data:
            return data, str(active)

        if "path" in data:
            target = REPO_ROOT / data["path"]
            try:
                return _load_json(target), str(target)
            except Exception as e:
                print(f"gen_default_config: {target} unreadable: {e}", file=sys.stderr)

    print("gen_default_config: no robot config found — using hardcoded defaults", file=sys.stderr)
    return {}, "(hardcoded defaults)"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get(d, *keys, default=None):
    """Safely traverse a chain of dict keys; return default if any is missing."""
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return default if cur is None else cur


def _f(v) -> str:
    """Format a Python float as a C++ float literal."""
    s = f"{float(v):.7g}"
    # Ensure there is a decimal point so it reads as float (not int).
    if "." not in s and "e" not in s:
        s += ".0"
    return s + "f"


def _b(v) -> str:
    return "true" if v else "false"


def _emit_literal(kind: str, value) -> str:
    """Render a JSON value as a C++ literal per the schema `firmware.kind`."""
    if kind == "bool":
        return _b(value)
    if kind == "int":
        return str(int(round(float(value))))
    if kind == "float_as_int":
        return _f(int(round(float(value))))   # integer magnitude, float-typed field
    return _f(value)                           # "float"


def fw_overrides(cfg: dict) -> dict:
    """Schema-driven JSON->C-field map: {cpp_field: c_literal}.

    Reads the custom ``firmware`` keyword from robot_config.schema.json (the
    single source of truth for the mapping) and, for every property that has a
    ``firmware.field`` AND a non-null value in the robot config, renders the C++
    literal. The generator overrides its hardcoded default with these so adding a
    value to the robot JSON flows into DefaultConfig.cpp with no generator edit.
    """
    try:
        schema = json.loads(SCHEMA_FILE.read_text())
    except Exception as e:  # pragma: no cover - build robustness
        print(f"gen_default_config: schema unreadable ({e}); no overrides", file=sys.stderr)
        return {}
    out: dict = {}
    for section, sec in (schema.get("properties") or {}).items():
        for prop, ps in (sec.get("properties") or {}).items():
            fw = ps.get("firmware") if isinstance(ps, dict) else None
            if not fw or "field" not in fw:
                continue
            val = _get(cfg, section, prop)
            if val is None:
                continue
            out[fw["field"]] = _emit_literal(fw.get("kind", "float"), val)
    return out


# ---------------------------------------------------------------------------
# Code generation
# ---------------------------------------------------------------------------

def generate(cfg: dict, source_path: str) -> str:
    cal   = cfg.get("calibration", {}) or {}
    geom  = cfg.get("geometry",    {}) or {}
    ctrl  = cfg.get("control",     {}) or {}
    wheels = cfg.get("wheels",     {}) or {}

    # Derive default mm/deg from wheel diameter when explicit override is absent.
    wd = _get(wheels, "wheel_diameter_mm")
    default_mmpd = (math.pi * float(wd) / 360.0) if wd is not None else 0.487

    mm_per_deg_l = _get(cal, "mm_per_wheel_deg_left",  default=default_mmpd)
    mm_per_deg_r = _get(cal, "mm_per_wheel_deg_right", default=default_mmpd)

    otos_lin  = _get(cal, "otos_linear_scale",    default=1.05)
    otos_ang  = _get(cal, "otos_angular_scale",   default=0.987)
    rot_gp    = _get(cal, "rotation_gain",        default=1.0)
    rot_gn    = _get(cal, "rotation_gain_neg",    default=1.17)
    rot_op    = _get(cal, "rotation_offset_deg",  default=0.0)
    rot_on    = _get(cal, "rotation_offset_deg_neg", default=0.0)
    rot_slip  = _get(cal, "rotational_slip",      default=0.74)

    trackwidth    = _get(geom, "trackwidth",            default=126.0)
    odom_x        = _get(geom, "odometry_offset_mm", "x",       default=0.0)
    odom_y        = _get(geom, "odometry_offset_mm", "y",       default=0.0)
    odom_yaw_rad  = _get(geom, "odometry_offset_mm", "yaw_rad", default=0.0)
    odom_yaw_deg  = math.degrees(float(odom_yaw_rad))
    odom_upside   = _get(geom, "odometry_chip_upside_down", default=False)

    vel_kp    = _get(ctrl, "vel_kp",        default=0.3)
    vel_ki    = _get(ctrl, "vel_ki",        default=0.05)
    vel_kff   = _get(ctrl, "vel_kff",       default=0.15)
    vel_imax  = _get(ctrl, "vel_imax",      default=20.0)
    vel_kaw   = _get(ctrl, "vel_kaw",       default=3.0)
    vel_filt  = _get(ctrl, "vel_filt",      default=0.15)
    sync      = _get(ctrl, "sync",          default=1.0)
    min_wheel = _get(ctrl, "min_wheel_mms", default=20.0)

    # Schema-driven overrides: any robot-JSON value mapped via the schema's
    # `firmware` keyword wins over the hardcoded default below. This is how a
    # value placed in the robot JSON reaches DefaultConfig.cpp with no edit here
    # (e.g. control.turn_gate -> turnInPlaceGate).
    fw = fw_overrides(cfg)

    def ov(field, default):
        return fw.get(field, default)

    return f"""\
// AUTO-GENERATED — do not edit by hand.
// Regenerated by scripts/gen_default_config.py before each firmware build.
// Source: {source_path}

#include "../types/Config.h"

RobotConfig defaultRobotConfig() {{
    RobotConfig p{{}};

    // Motor forward-direction signs
    p.fwdSignL        = +1;
    p.fwdSignR        = -1;

    // Encoder calibration — baked from robot config
    p.mmPerDegL       = {_f(mm_per_deg_l)};
    p.mmPerDegR       = {_f(mm_per_deg_r)};

    // Feed-forward and motor scale factors
    p.kFF             = 0.15f;
    p.kScaleLF        = 1.0f;
    p.kScaleLB        = 1.0f;
    p.kScaleRF        = 1.0f;
    p.kScaleRB        = 1.0f;

    // Slower-wheel adjustment
    p.kAdjThreshold   = 0.5f;
    p.kAdjGain        = 0.05f;

    // Geometry — baked from robot config
    p.trackwidthMm    = {_f(trackwidth)};

    // Ratio PID gains
    p.ratioPidKp      = 300.0f;
    p.ratioPidKi      = 0.0f;
    p.ratioPidKd      = 0.0f;
    p.ratioPidMax     = 30.0f;

    // Wheel saturation ceiling and steering headroom
    p.vWheelMax       = 400.0f;
    p.steerHeadroom   = 20.0f;

    // OTOS complementary fusion
    p.alphaPos        = 0.15f;
    p.alphaYaw        = 0.10f;
    p.otosGate        = 50.0f;

    // EKF sensor fusion
    // N15 fix (030-009): Q values are now per-second spectral densities.
    // EKF::predict() multiplies Q by dt_s before adding to P.  At the default
    // controlPeriodMs = 10 ms, Q*dt = Q/100 matches the previous per-call values.
    // Q_per_second = Q_old / 0.010 s.
    p.ekfQxy         = 200.0f;    // was 2.0 per-call; 2.0/0.010 = 200/s
    p.ekfQtheta      = 0.5f;      // was 0.005 per-call; 0.005/0.010 = 0.5/s
    p.ekfROtosXy     = 50.0f;

    // EKF velocity fusion (Sprint 023)
    p.ekfQv          = 5000.0f;   // was 50.0 per-call; 50.0/0.010 = 5000/s
    p.ekfQomega      = 1.0f;      // was 0.01 per-call; 0.01/0.010 = 1.0/s
    p.ekfROtosV      = 200.0f;
    p.ekfREncV       = 100.0f;

    // EKF heading fusion (Sprint 024-004)
    p.ekfROtosTheta  = {ov('ekfROtosTheta', '0.01f')};  // ~(5.7 deg)^2

    // OTOS calibration scalars — baked from robot config.
    // OtosSensor::begin() programs the hardware registers from these
    // values at firmware boot; no host-side OL/OA push required.
    p.otosLinearScale      = {_f(otos_lin)};
    p.otosAngularScale     = {_f(otos_ang)};
    p.rotationGainPos      = {_f(rot_gp)};
    p.rotationGainNeg      = {_f(rot_gn)};
    p.rotationOffsetDeg    = {_f(rot_op)};
    p.rotationOffsetDegNeg = {_f(rot_on)};
    p.rotationalSlip       = {_f(rot_slip)};
    p.odomOffX             = {_f(odom_x)};
    p.odomOffY             = {_f(odom_y)};
    p.odomYawDeg           = {_f(odom_yaw_deg)};
    p.odomUpsideDown       = {_b(odom_upside)};

    // Velocity-loop gains — baked from robot config
    p.velKp           = {_f(vel_kp)};
    p.velKi           = {_f(vel_ki)};
    p.velKff          = {_f(vel_kff)};
    p.minWheelMms     = {_f(min_wheel)};
    p.velIMax         = {_f(vel_imax)};
    p.velKaw          = {_f(vel_kaw)};
    p.velFiltAlpha    = {_f(vel_filt)};
    p.syncGain        = {_f(sync)};

    // Legacy go-to tolerances
    p.turnThresholdMm = 50.0f;
    p.doneTolMm       = 5.0f;

    // Pose-control tunables
    p.aMax            = 300.0f;
    p.aDecel          = 250.0f;
    p.turnInPlaceGate = {ov('turnInPlaceGate', '45.0f')};
    p.arriveTolMm     = {ov('arriveTolMm', '5.0f')};

    // Body motion limits
    p.vBodyMax        = 400.0f;
    p.yawRateMax      = 180.0f;
    p.yawAccMax       = 720.0f;
    p.jMax            = 0.0f;
    p.yawJerkMax      = 0.0f;

    // Timing
    p.minSpeedMms     = 50;
    p.tickMs          = 20;
    p.sTimeoutMs      = 500;
    p.safetyEnabled   = {ov('safetyEnabled', 'true')};
    p.controlPeriodMs = 10;
    p.tlmPeriodMs     = 0;
    p.tlmFields       = 0xFF;
    p.tlmSnapPending  = false;

    // Sensor lag budgets
    p.lagOtosMs       = 100;
    p.lagLineMs       = 50;
    p.lagColorMs      = 100;
    p.lagPortsMs      = 50;

    return p;
}}
"""


def main():
    cfg, source_path = load_robot_config()
    content = generate(cfg, source_path)
    OUT_FILE.write_text(content)
    print(f"gen_default_config: wrote {OUT_FILE.relative_to(REPO_ROOT)}", file=sys.stderr)


if __name__ == "__main__":
    main()
