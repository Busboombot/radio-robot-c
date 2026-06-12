"""calibrate_distance — interactive OTOS linear scale calibration.

Core logic extracted from ``host/calibrate_linear.py``.  No argparse or
sys.exit; the caller handles CLI setup and calls ``calibrate_distance()``.
"""

from __future__ import annotations

import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Optional

from robot_radio.calibration.helpers import (
    int8_to_scale,
    resolve_save_path,
    save_config,
    scale_to_int8,
)
from robot_radio.robot.protocol import parse_tlm

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DRIVE_SPEED_MMS = 200        # mm/s forward
DEFAULT_DISTANCE_CM = 50.0   # default target distance for a single trial
OTOS_FW_MIN_SCALE = 0.872    # int8 = -128
OTOS_FW_MAX_SCALE = 1.127    # int8 = +127
BAUD = 115200
WATCHDOG_MS = 3000


# ---------------------------------------------------------------------------
# Linear-specific math helpers
# ---------------------------------------------------------------------------

def compute_new_linear_scale(
    actual_mm: float,
    otos_mm: float,
    current_scale: float,
) -> tuple[float, int]:
    """Compute recommended new OTOS linear scale.

    Formula: new_scale = (actual_mm / otos_mm) * current_scale.
    Returns (new_scale_float, new_scale_int8).
    Clamps to firmware representable range.
    """
    ratio = actual_mm / otos_mm
    raw = ratio * current_scale
    clamped = max(OTOS_FW_MIN_SCALE, min(OTOS_FW_MAX_SCALE, raw))
    int8_val = scale_to_int8(clamped)
    return clamped, int8_val


def mean_ratio_stats(
    samples: list[tuple[float, float]],
) -> tuple[float, float, float]:
    """Compute (mean_ratio, stdev_ratio, sem) from (otos_mm, actual_mm) pairs."""
    ratios = [a / o for (o, a) in samples if o > 0]
    if not ratios:
        return 0.0, 0.0, 0.0
    mean = statistics.fmean(ratios)
    stdev = statistics.stdev(ratios) if len(ratios) >= 2 else 0.0
    sem = stdev / math.sqrt(len(ratios)) if len(ratios) >= 2 else 0.0
    return mean, stdev, sem


# ---------------------------------------------------------------------------
# Config helpers specific to linear calibration
# ---------------------------------------------------------------------------

def load_current_linear_scale(config_path: Path) -> float:
    """Read otos_linear_scale from robot config JSON, default 1.0."""
    try:
        data = json.loads(config_path.read_text())
        return float(data.get("calibration", {}).get("otos_linear_scale", 1.0))
    except Exception:
        return 1.0


def save_linear_scale_to_config(path: Path, new_scale: float) -> None:
    """Write otos_linear_scale into calibration section of robot JSON."""
    save_config(path, {"calibration": {"otos_linear_scale": round(new_scale, 6)}})


# ---------------------------------------------------------------------------
# Serial wire helpers
# ---------------------------------------------------------------------------

def _send_and_wait(ser, cmd: str, want_prefix: str, timeout: float = 5.0) -> list[str]:
    ser.write_line(cmd)
    collected: list[str] = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        lines = ser.read_available(timeout=0.1)
        for line in lines:
            collected.append(line)
            clean = line.strip().lstrip("<# ").strip()
            if clean.startswith(want_prefix):
                return collected
    return collected


def _snap_pose(ser, timeout: float = 3.0) -> Optional[tuple[int, int, int]]:
    lines = _send_and_wait(ser, "SNAP", "TLM", timeout=timeout)
    for line in lines:
        frame = parse_tlm(line)
        if frame is not None and frame.pose is not None:
            return frame.pose
    return None


def _snap_enc(ser, timeout: float = 3.0) -> Optional[tuple[int, int]]:
    lines = _send_and_wait(ser, "SNAP", "TLM", timeout=timeout)
    for line in lines:
        frame = parse_tlm(line)
        if frame is not None and frame.enc is not None:
            return frame.enc
    return None


def _wait_evt_done(ser, verb: str, timeout: float = 30.0) -> bool:
    target = f"EVT done {verb}"
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        lines = ser.read_available(timeout=0.2)
        for line in lines:
            clean = line.strip().lstrip("<# ").strip()
            if clean.startswith(target) or clean.startswith("EVT safety_stop"):
                return clean.startswith(target)
    return False


# ---------------------------------------------------------------------------
# Core interactive calibration logic
# ---------------------------------------------------------------------------

def calibrate_distance(
    ser,
    config_path: Optional[Path],
    target_cm: float = DEFAULT_DISTANCE_CM,
    speed_mms: int = DRIVE_SPEED_MMS,
    dry_run: bool = False,
) -> None:
    """Run interactive linear scale calibration.

    *ser* must expose ``write_line(text)``, ``read_available(timeout)`` and
    ``close()`` — satisfied by ``_RelaySerial`` or ``_DirectSerial`` from the
    entry-point script, or any compatible mock.

    *config_path* is the Path to the active robot JSON (may be None).
    """
    target_mm = round(target_cm * 10)

    current_scale = 1.0
    if config_path and config_path.exists():
        current_scale = load_current_linear_scale(config_path)
        print(f"  Config: {config_path}")
    else:
        print("  WARNING: No robot config found — using otos_linear_scale = 1.0")
    current_int8 = scale_to_int8(current_scale)
    print(f"  Current otos_linear_scale = {current_scale:.4f}  (int8={current_int8:+d})")

    # Ping & zero
    print("\n  Checking link (PING)...")
    lines = _send_and_wait(ser, "PING", "OK pong", timeout=3.0)
    if not any("pong" in ln for ln in lines):
        print("  WARNING: no PING reply — robot may not be reachable.")
    else:
        print("  Robot responding.")

    print("  Zeroing pose and encoders...")
    _send_and_wait(ser, "ZERO enc pose", "OK", timeout=2.0)
    time.sleep(0.2)

    print(f"  Setting OL {current_int8:+d} (scale={current_scale:.4f}) on hardware...")
    _send_and_wait(ser, f"OL {current_int8}", "OK", timeout=2.0)

    print(f"\n  Target distance: {target_cm:.1f} cm  Speed: {speed_mms} mm/s")
    print("  Mark the robot's starting position on the floor.")
    print("  Press Enter to drive each trial, 'q' to finish and see results.\n")

    samples: list[tuple[float, float]] = []  # (otos_mm, actual_mm)

    try:
        while True:
            n = len(samples)
            print(f"[Trial {n + 1}]  ({n} samples so far)  "
                  "— Enter to drive, 'q' to finish")
            try:
                raw = input().strip()
            except EOFError:
                break
            if raw.lower() in ("q", "quit", "exit"):
                break

            _send_and_wait(ser, "ZERO enc pose", "OK", timeout=2.0)
            time.sleep(0.15)

            print(f"  Driving {target_cm:.1f} cm ...")
            timeout_s = (target_mm / max(speed_mms, 1)) * 2.5 + 5.0
            _send_and_wait(ser, f"D {speed_mms} {speed_mms} {target_mm}",
                           "OK", timeout=2.0)
            done = _wait_evt_done(ser, "D", timeout=timeout_s)
            if not done:
                print("  WARNING: Did not receive EVT done D — robot may have stopped early.")
            time.sleep(0.3)

            pose = _snap_pose(ser, timeout=3.0)
            if pose is None:
                print("  WARNING: Could not read OTOS pose — skipping trial.")
                continue

            otos_x_mm = pose[0]
            enc = _snap_enc(ser, timeout=2.0)
            enc_mm_str = f"L={enc[0]}mm R={enc[1]}mm" if enc else "N/A"

            print(f"  OTOS pose: x={otos_x_mm}mm  y={pose[1]}mm  "
                  f"h={pose[2]/100:.1f}")
            print(f"  Encoders:  {enc_mm_str}")
            print("  Measure the actual distance traveled with a tape measure.")
            print("  Press Enter with no value to discard this trial.")

            try:
                raw = input("  Actual distance (cm): ").strip()
            except EOFError:
                break
            if not raw:
                print("  Discarded.")
                continue

            try:
                actual_cm = float(raw)
            except ValueError:
                print(f"  Invalid input '{raw}' — discarded.")
                continue
            if actual_cm <= 0:
                print("  Actual distance must be > 0 — discarded.")
                continue

            actual_mm = actual_cm * 10.0
            if abs(otos_x_mm) < 1:
                print("  OTOS x ~ 0 — sensor may not be responding. Discarded.")
                continue

            ratio = actual_mm / abs(otos_x_mm)
            if not (0.4 <= ratio <= 2.5):
                print(f"  WARNING: ratio {ratio:.3f} is out of range [0.4, 2.5] — "
                      f"check units (enter cm, not mm). Discarded.")
                continue

            samples.append((abs(otos_x_mm), actual_mm))
            err = actual_mm - abs(otos_x_mm)
            print(f"  Sample {len(samples)}: otos={otos_x_mm}mm  "
                  f"actual={actual_mm:.1f}mm  "
                  f"err={err:+.1f}mm ({err / abs(otos_x_mm) * 100:+.1f}%)  "
                  f"ratio={ratio:.4f}")

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write_line("STOP")
            time.sleep(0.2)
        except Exception:
            pass

    # Statistics
    print("\n" + "=" * 60)
    print(f"Samples collected: {len(samples)}")
    if len(samples) == 0:
        print("No usable samples — nothing to compute.")
        return

    print(f"\n{'#':>3}  {'otos_mm':>8}  {'actual_mm':>10}  {'ratio':>7}")
    for i, (o, a) in enumerate(samples, 1):
        print(f"{i:>3}  {o:>8.1f}  {a:>10.1f}  {a / o:>7.4f}")

    mean_r, stdev_r, sem_r = mean_ratio_stats(samples)
    new_scale_raw = mean_r * current_scale
    new_scale_clamped = max(OTOS_FW_MIN_SCALE, min(OTOS_FW_MAX_SCALE, new_scale_raw))
    new_int8 = scale_to_int8(new_scale_clamped)
    rounded_scale = int8_to_scale(new_int8)

    print(f"\n  ratio  mean={mean_r:.4f}  stdev={stdev_r:.4f}  "
          f"sem={sem_r:.4f}  (n={len(samples)})")
    print(f"  new scale = current ({current_scale:.4f}) x "
          f"mean_ratio ({mean_r:.4f}) = {new_scale_raw:.4f}")
    print(f"  clamped   = {new_scale_clamped:.4f}  ->  "
          f"int8={new_int8:+d}  -> rounded_scale={rounded_scale:.4f}")

    print(f"\n  To set manually:")
    print(f'    "otos_linear_scale": {rounded_scale:.4f}  # int8={new_int8:+d},'
          f' n={len(samples)}, stdev={stdev_r:.4f}')

    if dry_run:
        print("\n  --dry-run: config NOT updated.")
        return

    if config_path is None:
        print("\n  No config path found — cannot save. Update manually.")
        return

    print(f"\n  Save to {config_path}? [Y/n] ", end="", flush=True)
    try:
        ans = input().strip()
    except EOFError:
        ans = ""
    if ans.lower() in ("", "y", "yes"):
        save_linear_scale_to_config(config_path, rounded_scale)
        print(f"  Saved otos_linear_scale = {rounded_scale:.4f} to {config_path}")
    else:
        print("  Not saved.")
