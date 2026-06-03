"""Robot calibration loader and applier.

The firmware exposes four runtime-tunable calibration values through
the ``SET ml`` / ``SET mr`` commands (v2 protocol).  The legacy K-command
family (KML, KMR, KSD, KST) has been retired; all calibration is now
pushed via ``SET`` and verified with ``GET`` / ``CFG`` responses.

Ground truth for calibration is provided by the v2 TLM ``pose=x,y,h``
field (emitted on each ``TLM`` line, or on demand via ``SNAP``).
Heading ``h`` is in centi-degrees (integer).  Position x/y are in mm.

Use ``calibrate_linear.py`` / ``calibrate_angular.py`` scripts in the
project root to collect measurements and update ``data/robots/<robot>.json``.

A ``robot_calibration.json`` file stores these as floats in human-
readable form plus provenance notes.  This module reads the file,
sends the corresponding K commands to the robot, and (optionally)
verifies via a ``K`` query round-trip that the firmware picked them
up correctly.

Schema (``robot_calibration/v1``)::

    {
      "schema": "robot_calibration/v1",
      "robot_id": "guvov",
      "linear": {
        "mm_per_wheel_deg_left":  0.487,
        "mm_per_wheel_deg_right": 0.481,
        "dist_scale":             0.94
      },
      "rotational": {
        "turn_scale": 1.07
      }
    }

Missing keys in ``linear`` or ``rotational`` are left at whatever the
firmware defaults to — only the keys that are present get sent.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from robot_radio.io.serial_conn import SerialConnection


SCHEMA = "robot_calibration/v1"
DEFAULT_PATH = (
    Path(__file__).resolve().parent.parent / "data" / "robot_calibration.json"
)

# Map of (section, json_key) → (K key, wire scale, human unit label)
_MAPPING: dict[tuple[str, str], tuple[str, int, str]] = {
    ("linear",     "mm_per_wheel_deg_left"):  ("ML", 1000, "mm/deg"),
    ("linear",     "mm_per_wheel_deg_right"): ("MR", 1000, "mm/deg"),
    ("linear",     "dist_scale"):             ("SD",  100, ""),
    ("rotational", "turn_scale"):             ("ST",  100, ""),
}


class CalibrationError(Exception):
    """Raised when a calibration file is malformed or firmware verification fails."""


def load(path: str | Path | None = None) -> dict[str, Any]:
    """Load and validate a robot calibration JSON file."""
    p = Path(path) if path else DEFAULT_PATH
    if not p.exists():
        raise CalibrationError(f"Calibration file not found: {p}")
    try:
        data = json.loads(p.read_text())
    except json.JSONDecodeError as e:
        raise CalibrationError(f"Invalid JSON in {p}: {e}") from e
    if data.get("schema") != SCHEMA:
        raise CalibrationError(
            f"Unexpected schema {data.get('schema')!r} in {p} "
            f"(expected {SCHEMA!r})"
        )
    return data


def to_wire_values(cal: dict[str, Any]) -> list[tuple[str, int]]:
    """Return the (K-key, wire-value) pairs to send, in deterministic order."""
    out: list[tuple[str, int]] = []
    for (section, key), (k_key, scale, _unit) in _MAPPING.items():
        val = cal.get(section, {}).get(key)
        if val is None:
            continue
        if not isinstance(val, (int, float)):
            raise CalibrationError(
                f"{section}.{key} must be a number, got {type(val).__name__}"
            )
        out.append((k_key, round(val * scale)))
    return out


def apply(conn: SerialConnection, cal: dict[str, Any],
          verify: bool = True, settle_ms: int = 120,
          log: bool = False) -> dict[str, int]:
    """Send K commands based on ``cal``.  Queries the robot first and
    skips any setters whose firmware value already matches — reduces
    stress on the relay TX→RX turnaround when nothing needs to change.
    Verifies via a follow-up K query when ``verify`` is True.

    Returns the dict of values reported by the robot in K-query wire
    units (e.g. ``{"ML": 487, "MR": 481, "SD": 94, "ST": 107}``).
    """
    expected = to_wire_values(cal)

    # Check current firmware state so we can skip redundant setters.
    current: dict[str, int] = {}
    for attempt in range(3):
        resp = conn.send("K", read_ms=600)
        current = _parse_cal_response(resp.get("responses", []))
        if current:
            break
        conn.read_lines(duration_ms=250)
    if log:
        print(f"calibration: current firmware state = {current}")

    sends = [(k, v) for k, v in expected if current.get(k) != v]
    if log:
        if sends:
            print(f"calibration: sending {len(sends)}/{len(expected)} K command(s)")
        else:
            print("calibration: firmware already matches — no setters needed")

    for k_key, val in sends:
        sign = "+" if val >= 0 else "-"
        wire = f"K{k_key}{sign}{abs(val):03d}"
        if log:
            print(f"  → {wire}")
        conn.send_fast(wire)
        conn.read_lines(duration_ms=settle_ms)

    if not verify:
        return {}

    # If no setters ran, we already have a fresh `current` readout.
    if not sends and current:
        if log:
            print(f"calibration: verified {current}")
        return dict(current)

    # The relay occasionally drops the K query's responses when it arrives
    # right after a burst of setter commands.  Retry a few times before
    # giving up — the firmware state change definitely went through, we
    # just couldn't read the verification back.
    expected_map = {k: v for k, v in expected}
    reported: dict[str, int] = {}
    last_mismatches: list[str] = []
    for attempt in range(3):
        resp = conn.send("K", read_ms=600)
        reported = _parse_cal_response(resp.get("responses", []))
        mismatches = []
        for k, v in expected_map.items():
            if k not in reported:
                mismatches.append(f"{k}: not reported")
            elif reported[k] != v:
                mismatches.append(f"{k}: sent {v}, robot reports {reported[k]}")
        if not mismatches:
            break
        last_mismatches = mismatches
        if log:
            print(f"calibration: verify attempt {attempt+1} incomplete, retrying...")
        # Give the relay a moment before the next query.
        conn.read_lines(duration_ms=250)
    else:
        raise CalibrationError(
            "Robot did not accept calibration cleanly: "
            + "; ".join(last_mismatches)
            + f" (reported={reported})"
        )

    if log:
        print(f"calibration: verified {reported}")
    return reported


def _parse_cal_response(lines: list[str]) -> dict[str, int]:
    """Parse ``CAL:XX <n>`` lines into a dict.  Tolerates relay
    prefixes like ``<CAL:ML 487``."""
    out: dict[str, int] = {}
    for line in lines:
        text = line.lstrip("<>").strip()
        if not text.startswith("CAL:"):
            continue
        parts = text.split()
        if len(parts) != 2:
            continue
        key = parts[0][4:]
        try:
            out[key] = int(parts[1])
        except ValueError:
            continue
    return out


def load_and_apply(conn: SerialConnection, path: str | Path | None = None,
                   *, verify: bool = True, log: bool = False) -> dict[str, Any]:
    """Convenience: load a calibration file and apply it.  Returns the
    parsed calibration dict (for callers that want to log provenance).
    """
    cal = load(path)
    apply(conn, cal, verify=verify, log=log)
    return cal
