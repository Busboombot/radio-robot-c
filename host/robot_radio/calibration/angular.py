"""calibrate_turns — interactive OTOS angular scale calibration.

Core logic extracted from ``host/calibrate_angular.py``.  No argparse or
sys.exit; the caller handles CLI setup and calls ``calibrate_turns()``.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional

from robot_radio.calibration.helpers import (
    deep_merge,
    int8_to_scale,
    mean_stdev,
    resolve_save_path,
    save_config,
    scale_to_int8,
)
from robot_radio.robot.protocol import parse_tlm

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

TURN_SPEED_MMS = 100         # mm/s wheel speed for spin
DEFAULT_ANGLE_DEG = 360.0    # target spin angle per trial
OTOS_FW_MIN_SCALE = 0.872    # int8 = -128
OTOS_FW_MAX_SCALE = 1.127    # int8 = +127
BAUD = 115200


# ---------------------------------------------------------------------------
# Angular-specific math helpers
# ---------------------------------------------------------------------------

def compute_new_angular_scale(
    target_deg: float,
    otos_deg: float,
    current_scale: float,
) -> tuple[float, int]:
    """Compute recommended new OTOS angular scale.

    Formula: new_scale = (target_deg / otos_deg) * current_scale.
    Returns (new_scale_float, new_scale_int8).
    Clamps to firmware representable range.
    """
    if abs(otos_deg) < 1.0:
        return current_scale, scale_to_int8(current_scale)
    ratio = target_deg / otos_deg
    raw = ratio * current_scale
    clamped = max(OTOS_FW_MIN_SCALE, min(OTOS_FW_MAX_SCALE, raw))
    return clamped, scale_to_int8(clamped)


def heading_delta_cdeg(before_cdeg: int, after_cdeg: int) -> float:
    """Compute signed heading change in degrees from two centi-degree readings.

    Handles wrap-around at ±18000 cdeg (±180°).
    Returns positive for CCW (firmware convention: CCW positive).
    """
    delta_cdeg = after_cdeg - before_cdeg
    while delta_cdeg > 18000:
        delta_cdeg -= 36000
    while delta_cdeg <= -18000:
        delta_cdeg += 36000
    return delta_cdeg / 100.0


# ---------------------------------------------------------------------------
# Config helpers specific to angular calibration
# ---------------------------------------------------------------------------

def load_current_angular_scale(config_path: Path) -> float:
    """Read otos_angular_scale from robot config JSON, default 1.0."""
    try:
        data = json.loads(config_path.read_text())
        return float(data.get("calibration", {}).get("otos_angular_scale", 1.0))
    except Exception:
        return 1.0


def save_angular_calibration_to_config(
    path: Path,
    angular_scale: float,
    rotation_gain: Optional[float] = None,
    rotation_gain_neg: Optional[float] = None,
) -> None:
    """Write angular calibration fields into robot config JSON."""
    cal: dict = {"otos_angular_scale": round(angular_scale, 6)}
    if rotation_gain is not None:
        cal["rotation_gain"] = round(rotation_gain, 6)
    if rotation_gain_neg is not None:
        cal["rotation_gain_neg"] = round(rotation_gain_neg, 6)
    save_config(path, {"calibration": cal})


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


def _wait_evt_done(ser, verb: str, timeout: float = 60.0) -> bool:
    target = f"EVT done {verb}"
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        lines = ser.read_available(timeout=0.2)
        for line in lines:
            clean = line.strip().lstrip("<# ").strip()
            if clean.startswith(target):
                return True
            if clean.startswith("EVT safety_stop"):
                return False
    return False


def _stream_tlm_until_evt(
    ser, verb: str, timeout: float
) -> tuple[list[tuple[int, int, int]], bool]:
    target = f"EVT done {verb}"
    poses: list[tuple[int, int, int]] = []
    deadline = time.monotonic() + timeout
    done = False
    while time.monotonic() < deadline:
        lines = ser.read_available(timeout=0.1)
        for line in lines:
            frame = parse_tlm(line)
            if frame is not None and frame.pose is not None:
                poses.append(frame.pose)
            clean = line.strip().lstrip("<# ").strip()
            if clean.startswith(target):
                done = True
                break
            if clean.startswith("EVT safety_stop"):
                break
        if done:
            break
    return poses, done


# ---------------------------------------------------------------------------
# Interactive arrow-key adjustment
# ---------------------------------------------------------------------------

def _interactive_adjust(ser, current_h_cdeg: int, target_deg: float,
                        nudge_speed: int = 80, nudge_ms: int = 60) -> int:
    """Interactive arrow-key heading adjustment. Returns final h_cdeg."""
    if not sys.stdin.isatty():
        print("  (stdin is not a TTY — skipping interactive adjustment)")
        return current_h_cdeg

    import select
    import termios
    import tty
    import os as _os

    h_cdeg = current_h_cdeg

    def _read_key(fd: int) -> str:
        ch = _os.read(fd, 1).decode("utf-8", errors="replace")
        if ch == "\x1b":
            if select.select([fd], [], [], 0.1)[0]:
                rest = _os.read(fd, 2).decode("utf-8", errors="replace")
                if rest == "[D":
                    return "left"
                if rest == "[C":
                    return "right"
            return "esc"
        if ch == "\x03":
            return "ctrl-c"
        if ch in ("\r", "\n"):
            return "enter"
        return ch

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    def _show():
        err_deg = h_cdeg / 100.0 - target_deg
        hint = " <- " if err_deg > 0 else " -> "
        print(f"\r  heading={h_cdeg / 100:+.2f}  "
              f"err={err_deg:+.2f}  [<- CW  -> CCW  Enter=done]{hint}   ",
              end="", flush=True)

    try:
        tty.setraw(fd)
        _show()
        while True:
            if select.select([fd], [], [], 0.02)[0]:
                k = _read_key(fd)
                if k == "enter":
                    break
                if k in ("esc", "ctrl-c"):
                    break
                if k == "left":
                    ser.write_line(f"T -{nudge_speed} {nudge_speed} {nudge_ms}")
                elif k == "right":
                    ser.write_line(f"T {nudge_speed} -{nudge_speed} {nudge_ms}")
                time.sleep(nudge_ms / 1000.0 + 0.1)
            lines = ser.read_available(timeout=0.05)
            for line in lines:
                frame = parse_tlm(line)
                if frame is not None and frame.pose is not None:
                    h_cdeg = frame.pose[2]
                    _show()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()

    return h_cdeg


# ---------------------------------------------------------------------------
# Core interactive calibration logic
# ---------------------------------------------------------------------------

def calibrate_turns(
    ser,
    config_path: Optional[Path],
    target_deg: float = DEFAULT_ANGLE_DEG,
    speed_mms: int = TURN_SPEED_MMS,
    dry_run: bool = False,
) -> None:
    """Run interactive angular scale calibration.

    *ser* must expose ``write_line(text)``, ``read_available(timeout)`` and
    ``close()`` — satisfied by ``_RelaySerial`` or ``_DirectSerial`` from the
    entry-point script, or any compatible mock.

    *config_path* is the Path to the active robot JSON (may be None).

    All output goes to stdout/stderr.  Raises ``SystemExit`` only on
    unrecoverable errors (already handled by the entry point).
    """
    current_scale = 1.0
    if config_path and config_path.exists():
        current_scale = load_current_angular_scale(config_path)
        print(f"  Config: {config_path}")
    else:
        print("  WARNING: No robot config found — using otos_angular_scale = 1.0")
    current_int8 = scale_to_int8(current_scale)
    print(f"  Current otos_angular_scale = {current_scale:.4f}  (int8={current_int8:+d})")

    # Ping & set angular scalar
    print("\n  Checking link (PING)...")
    lines = _send_and_wait(ser, "PING", "OK pong", timeout=3.0)
    if not any("pong" in ln for ln in lines):
        print("  WARNING: no PING reply — robot may not be reachable.")
    else:
        print("  Robot responding.")

    print(f"  Setting OA {current_int8:+d} (scale={current_scale:.4f}) on hardware...")
    _send_and_wait(ser, f"OA {current_int8}", "OK", timeout=2.0)

    print(f"\n  Target spin angle: {target_deg:.1f}  Speed: {speed_mms} mm/s")
    print("  Aim the robot's marker at a reference point on the wall/floor.")
    print("  Trials alternate CCW / CW.  Use <- -> to nudge back to the mark.")
    print("  Press Enter to spin each trial, 'q' to finish.\n")

    samples: list[tuple[int, float]] = []  # (direction_sign, otos_deg)
    direction = +1  # start CCW

    try:
        while True:
            n = len(samples)
            label = "CCW" if direction > 0 else "CW"
            print(f"[Trial {n + 1}]  ({n} samples)  next: {label}  "
                  "— Enter to spin, 'q' to finish")
            try:
                raw = input().strip()
            except EOFError:
                break
            if raw.lower() in ("q", "quit", "exit"):
                break

            _send_and_wait(ser, "ZERO enc pose", "OK", timeout=2.0)
            time.sleep(0.15)

            pose_before = _snap_pose(ser, timeout=2.0)
            h_before_cdeg = pose_before[2] if pose_before else 0

            _send_and_wait(ser, "STREAM 20", "OK", timeout=2.0)
            time.sleep(0.1)

            trackwidth_mm = 126.0
            t_ms = int((target_deg / 360.0) * math.pi * trackwidth_mm / speed_mms * 1000)
            t_ms = max(500, min(15000, t_ms))

            l_speed = direction * speed_mms
            r_speed = -direction * speed_mms
            print(f"  Spinning {label} for ~{t_ms}ms ({target_deg:.0f})...")

            spin_cmd = f"T {l_speed} {r_speed} {t_ms}"
            _send_and_wait(ser, spin_cmd, "OK", timeout=2.0)
            timeout_s = t_ms / 1000.0 + 5.0
            poses_during, done_flag = _stream_tlm_until_evt(ser, "T", timeout=timeout_s)

            if not done_flag:
                print("  WARNING: Did not receive EVT done T — spin may have been incomplete.")

            _send_and_wait(ser, "STREAM 0", "OK", timeout=2.0)
            time.sleep(0.2)

            pose_after = _snap_pose(ser, timeout=3.0)
            h_after_cdeg = pose_after[2] if pose_after else (
                poses_during[-1][2] if poses_during else h_before_cdeg
            )

            otos_deg_raw = heading_delta_cdeg(h_before_cdeg, h_after_cdeg)
            print(f"  OTOS heading: before={h_before_cdeg / 100:.2f}  "
                  f"after={h_after_cdeg / 100:.2f}  delta={otos_deg_raw:+.2f}")
            print(f"  Target: {direction * target_deg:+.1f}  "
                  f"Error: {otos_deg_raw - direction * target_deg:+.2f}")

            print(f"\n  Nudge the robot back to the starting mark with <- -> keys.")
            print(f"  Press Enter when aligned.")
            h_adjusted_cdeg = _interactive_adjust(
                ser, h_after_cdeg,
                target_deg=direction * target_deg,
                nudge_speed=80, nudge_ms=50,
            )
            ser.write_line("STOP")
            time.sleep(0.1)

            adjusted_deg = heading_delta_cdeg(h_before_cdeg, h_adjusted_cdeg)
            err_deg = adjusted_deg - direction * target_deg
            print(f"  Adjusted: otos={adjusted_deg:+.2f}  "
                  f"target={direction * target_deg:+.1f}  "
                  f"err={err_deg:+.2f}")

            if abs(adjusted_deg) < 10.0:
                print("  WARNING: adjusted heading < 10 — possible misread. Discarded.")
            else:
                samples.append((direction, abs(adjusted_deg)))
                print(f"  Sample {len(samples)} recorded: "
                      f"dir={label}  adjusted={abs(adjusted_deg):.2f}")

            direction = -direction

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write_line("STOP")
            ser.write_line("STREAM 0")
            time.sleep(0.2)
        except Exception:
            pass

    # Statistics
    print("\n" + "=" * 60)
    print(f"Samples collected: {len(samples)}")
    if len(samples) < 2:
        print("Need >= 2 samples — not enough data.")
        return

    ccw_degs = [d for (sign, d) in samples if sign > 0]
    cw_degs  = [d for (sign, d) in samples if sign < 0]

    print(f"\n{'#':>3}  {'dir':>4}  {'otos_deg':>9}  {'ratio':>7}  {'err':>7}")
    ratios: list[float] = []
    for i, (sign, d) in enumerate(samples, 1):
        label = "CCW" if sign > 0 else "CW"
        ratio = target_deg / d if d > 0 else 0.0
        ratios.append(ratio)
        print(f"{i:>3}  {label:>4}  {d:>9.2f}  {ratio:>7.4f}  "
              f"{d - target_deg:>+7.2f}")

    mean_all, std_all = mean_stdev(ratios)
    mean_ccw_d, std_ccw_d = mean_stdev(ccw_degs)
    mean_cw_d,  std_cw_d  = mean_stdev(cw_degs)

    new_scale_raw = mean_all * current_scale
    new_scale_clamped = max(OTOS_FW_MIN_SCALE, min(OTOS_FW_MAX_SCALE, new_scale_raw))
    new_int8 = scale_to_int8(new_scale_clamped)
    rounded_scale = int8_to_scale(new_int8)

    rot_gain = target_deg / mean_ccw_d if mean_ccw_d > 0 else None
    rot_gain_neg = target_deg / mean_cw_d if mean_cw_d > 0 else None

    print(f"\nRatio statistics (target_deg / otos_deg):")
    print(f"  Overall: mean={mean_all:.4f}  stdev={std_all:.4f}  (n={len(ratios)})")
    if ccw_degs:
        print(f"  CCW otos_deg: mean={mean_ccw_d:.2f}  stdev={std_ccw_d:.2f}  (n={len(ccw_degs)})")
    if cw_degs:
        print(f"  CW  otos_deg: mean={mean_cw_d:.2f}  stdev={std_cw_d:.2f}  (n={len(cw_degs)})")

    print(f"\nRecommended values:")
    print(f"  otos_angular_scale = {current_scale:.4f} x {mean_all:.4f} = {new_scale_raw:.4f}")
    print(f"  clamped = {new_scale_clamped:.4f}  ->  int8={new_int8:+d}  -> {rounded_scale:.4f}")
    if rot_gain is not None:
        print(f"  rotation_gain     (CCW) = {rot_gain:.4f}")
    if rot_gain_neg is not None:
        print(f"  rotation_gain_neg (CW)  = {rot_gain_neg:.4f}")

    print(f"\n  To set manually in data/robots/<robot>.json:")
    print(f'    "otos_angular_scale": {rounded_scale:.4f}')
    if rot_gain is not None:
        print(f'    "rotation_gain": {round(rot_gain, 4)}')
    if rot_gain_neg is not None:
        print(f'    "rotation_gain_neg": {round(rot_gain_neg, 4)}')

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
        save_angular_calibration_to_config(
            config_path, rounded_scale,
            rotation_gain=rot_gain,
            rotation_gain_neg=rot_gain_neg,
        )
        print(f"  Saved to {config_path}")
    else:
        print("  Not saved.")
