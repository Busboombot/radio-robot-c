"""Shared calibration math helpers — no hardware dependencies.

All functions here are pure (no I/O, no serial) and fully unit-testable.

Canonical implementations; do not duplicate these elsewhere.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Optional


# ---------------------------------------------------------------------------
# Scale encoding (OTOS firmware int8 representation)
# ---------------------------------------------------------------------------

def scale_to_int8(scale: float) -> int:
    """Convert an OTOS scale float to the firmware int8 encoding.

    Firmware stores OL/OA as a signed offset from 1.0 in units of 0.001
    (0.1% per step), clamped to int8 range.

    ``scale = 1.027`` → ``int8 = 27``.  Clamped to [-128, 127].
    """
    return max(-128, min(127, round((scale - 1.0) / 0.001)))


def int8_to_scale(val: int) -> float:
    """Decode a firmware int8 back to a float scale.

    ``int8 = 27`` → ``scale = 1.027``.
    """
    return 1.0 + val * 0.001


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def mean_stdev(values: list[float]) -> tuple[float, float]:
    """Return (mean, stdev) for a list of floats.

    Uses Bessel-corrected sample standard deviation.
    Returns (mean, 0.0) for a single-element list; (0.0, 0.0) for empty.
    """
    if not values:
        return 0.0, 0.0
    n = len(values)
    m = sum(values) / n
    if n < 2:
        return m, 0.0
    var = sum((x - m) ** 2 for x in values) / (n - 1)
    return m, math.sqrt(var)


# ---------------------------------------------------------------------------
# Config file helpers
# ---------------------------------------------------------------------------

def deep_merge(dst: dict, src: dict) -> None:
    """Recursively merge *src* into *dst* in-place.

    Nested dicts are merged recursively; all other values are overwritten.
    """
    for k, v in src.items():
        if k in dst and isinstance(dst[k], dict) and isinstance(v, dict):
            deep_merge(dst[k], v)
        else:
            dst[k] = v


def save_config(path: Path, updates: dict) -> None:
    """Read existing JSON at *path*, deep-merge *updates*, write back with indent=2."""
    data = json.loads(path.read_text())
    deep_merge(data, updates)
    path.write_text(json.dumps(data, indent=2) + "\n")


def resolve_save_path(project_root: Optional[Path] = None) -> Optional[Path]:
    """Resolve the path to the active robot config for saving.

    Resolution order:
      1. ``ROBOT_CONFIG`` environment variable — full or repo-relative path.
      2. ``data/robots/active_robot.json`` — pointer (``"path"`` key) or full config.

    *project_root* defaults to the repository root inferred from this file's
    location (``host/robot_radio/calibration/helpers.py`` → four parents up).
    Returns the resolved :class:`~pathlib.Path`, or ``None`` if not found.
    """
    if project_root is None:
        project_root = Path(__file__).resolve().parents[4]

    env_path = os.environ.get("ROBOT_CONFIG")
    if env_path:
        p = Path(env_path)
        if not p.is_absolute():
            p = project_root / p
        return p if p.exists() else None

    active = project_root / "data" / "robots" / "active_robot.json"
    if not active.exists():
        return None
    try:
        pointer = json.loads(active.read_text())
    except Exception:
        return None
    if "path" in pointer:
        return project_root / pointer["path"]
    # active_robot.json is itself the full config
    return active
