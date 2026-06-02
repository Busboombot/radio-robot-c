"""
Pure helper functions for the two-phase approach controller.

These functions have no hardware dependencies so they can be unit-tested
without a connected robot or camera.

Phases:
  far   — r > far_threshold_mm  (default 100 mm)
  near  — tolerance_mm < r <= far_threshold_mm
  done  — r <= tolerance_mm     (default 5 mm)
"""

from __future__ import annotations

import json
import logging
import math
from typing import Any

logger = logging.getLogger(__name__)

# ── Hard-coded sane defaults ─────────────────────────────────────────────────

_DEFAULT_LINEAR: dict[str, Any] = {
    "slope": 1.02,
    "intercept_mm": -17.0,
    "startup_stop_loss_mm": 17.0,
    "clamp_floor_mms_measured": 200.0,
}

_DEFAULT_CRAWL: dict[str, Any] = {
    "speed_mms": 300,
    "pulse_ms": 80,
    "delay_ms": 20,
    "mm_per_pulse_cam_mean": 6.5,
}

# Undershoot margin applied to far-phase commands (mm).
# We command D - MARGIN so the near phase closes the gap.
_FAR_MARGIN_MM = 20.0


# ── Calibration loading ──────────────────────────────────────────────────────

def load_approach_calibration(
    linear_path: str | None = None,
    crawl_path: str | None = None,
) -> dict[str, Any]:
    """Load approach calibration from JSON files.

    Falls back to hard-coded defaults if a file is missing or malformed.

    Returns a dict with keys ``"linear"`` and ``"crawl"``.
    """
    linear = _load_linear(linear_path)
    crawl = _load_crawl(crawl_path)
    return {"linear": linear, "crawl": crawl}


def _load_linear(path: str | None) -> dict[str, Any]:
    defaults = _DEFAULT_LINEAR.copy()
    if path is None:
        return defaults
    try:
        with open(path) as f:
            data = json.load(f)
        fit = data["fit_cmd_ge_200"]
        return {
            "slope": float(fit["slope"]),
            "intercept_mm": float(fit.get("intercept_mm", defaults["intercept_mm"])),
            "startup_stop_loss_mm": float(data.get("startup_stop_loss_mm",
                                                     defaults["startup_stop_loss_mm"])),
            "clamp_floor_mms_measured": float(data.get("clamp_floor_mms_measured",
                                                        defaults["clamp_floor_mms_measured"])),
        }
    except (FileNotFoundError, OSError):
        logger.warning("linear_calibration.json not found at %s — using defaults", path)
        return defaults
    except (KeyError, ValueError, json.JSONDecodeError) as exc:
        logger.warning("linear_calibration.json malformed (%s) — using defaults", exc)
        return defaults


def _load_crawl(path: str | None) -> dict[str, Any]:
    defaults = _DEFAULT_CRAWL.copy()
    if path is None:
        return defaults
    try:
        with open(path) as f:
            data = json.load(f)
        gb = data["global_best"]
        return {
            "speed_mms": int(gb["speed_mms"]),
            "pulse_ms": int(gb["pulse_ms"]),
            "delay_ms": int(gb["delay_ms"]),
            "mm_per_pulse_cam_mean": float(gb.get("mm_per_pulse_cam_mean",
                                                   defaults["mm_per_pulse_cam_mean"])),
        }
    except (FileNotFoundError, OSError):
        logger.warning("crawl_calibration.json not found at %s — using defaults", path)
        return defaults
    except (KeyError, ValueError, json.JSONDecodeError) as exc:
        logger.warning("crawl_calibration.json malformed (%s) — using defaults", exc)
        return defaults


# ── Pure controller functions ────────────────────────────────────────────────

def choose_phase(
    r_mm: float,
    far_threshold_mm: float = 100.0,
    tolerance_mm: float = 5.0,
) -> str:
    """Return the control phase for the given remaining distance.

    Args:
        r_mm: Distance to target in millimetres (>= 0).
        far_threshold_mm: Threshold separating far and near phases.
        tolerance_mm: Distance at which we declare arrival.

    Returns:
        ``"done"``  — r_mm <= tolerance_mm
        ``"near"``  — tolerance_mm < r_mm <= far_threshold_mm
        ``"far"``   — r_mm > far_threshold_mm
    """
    if r_mm <= tolerance_mm:
        return "done"
    if r_mm <= far_threshold_mm:
        return "near"
    return "far"


def compute_far_command(
    r_mm: float,
    calibration: dict[str, Any],
    margin_mm: float = _FAR_MARGIN_MM,
) -> tuple[int, int]:
    """Compute speed (mm/s) and duration (ms) for a single far-phase drive command.

    Physics model::

        D_actual ≈ v·t − v²/(2a)

    Rearranged to hit effective distance ``D_eff = r_mm - margin_mm``::

        v   = clamp(sqrt(FIT_A · D_eff), v_min, v_max)
        t_s = (D_eff + v² / (2a)) / v
            = (D_eff / v) + v / (2a)

    where ``FIT_A = 0.2 · 2a`` (the "10% rule" gives the usable fraction of
    the deceleration budget as a proportional factor).

    In practice the calibration constant ``startup_stop_loss_mm`` is the
    measured ``v²/(2a)`` at the clamped floor speed, so ``2·a`` can be
    estimated as ``v_floor² / startup_stop_loss_mm``.

    Args:
        r_mm: Remaining distance to target in millimetres.
        calibration: Dict with ``"linear"`` sub-dict from
            :func:`load_approach_calibration`.
        margin_mm: Safety margin subtracted from ``r_mm`` before computing.
            We intentionally undershoot by this amount so the near phase
            closes the gap.

    Returns:
        ``(speed_mms, duration_ms)`` — both positive integers. Returns
        ``(v_floor, 0)`` if the effective distance is <= 0 (caller should skip).
    """
    lin = calibration["linear"]
    v_floor = float(lin["clamp_floor_mms_measured"])  # ~206 mm/s
    loss_mm = float(lin["startup_stop_loss_mm"])       # ~17 mm

    # 2·a  from  loss = v_floor² / (2a)  →  2a = v_floor² / loss
    two_a = (v_floor * v_floor) / max(loss_mm, 1.0)   # ~2050 mm/s²
    fit_a = 0.2 * two_a                                # "10% rule" ≈ 410

    D_eff = r_mm - margin_mm
    if D_eff <= 0:
        return (int(v_floor), 0)

    v_max = 400.0
    v_raw = math.sqrt(fit_a * D_eff)
    v = max(v_floor, min(v_max, v_raw))

    # t = (D_eff + v²/(2a)) / v  = D_eff/v + v/(2a)
    t_s = D_eff / v + v / two_a
    t_ms = max(1, round(t_s * 1000))

    return (int(round(v)), t_ms)
