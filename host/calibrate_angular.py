"""Calibrate the OTOS angular scale over the v2 radio-relay data plane.

Spins the robot by a known angle (default 360°) via the ``T`` command
(timed spin), accumulates OTOS heading from ``TLM pose=`` frames (h field
is in centi-degrees), and prompts the operator to nudge the robot back to
the starting orientation with arrow keys or a ground-truth measurement.

The scale formula is:

    new_angular_scale = (target_deg / otos_deg) * current_scale

encoded as int8 via: ``int8 = round((scale - 1.0) / 0.001)`` clamped to
[-128, 127].

Per-direction gains are also computed:

    rotation_gain     = target_deg / mean(ccw_otos_deg)   (CCW turns)
    rotation_gain_neg = target_deg / mean(cw_otos_deg)    (CW turns)

Connection: relay first, direct robot USB if relay is not found.

Procedure:
  1. Mark the robot's laser/heading on the wall or floor.
  2. Press Enter — the robot spins 360° (CCW first).
  3. After the spin, the OTOS heading is displayed.
  4. Nudge the robot back to the mark with ← / → arrow keys (or by hand).
  5. Press Enter when aligned — this records the adjusted heading as ground truth.
  6. Repeat for ≥ 4 samples (alternating CCW/CW).
  7. Ctrl-C or 'q' to finish and see results.

Usage:
    uv run python calibrate_angular.py [--speed MMS] [--angle DEG] [--dry-run]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

_HOST_ROOT = Path(__file__).resolve().parent
_PROJECT_ROOT = _HOST_ROOT.parent
sys.path.insert(0, str(_HOST_ROOT))

from robot_radio.robot.protocol import parse_tlm, TLMFrame

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

TURN_SPEED_MMS = 100         # mm/s wheel speed for spin
DEFAULT_ANGLE_DEG = 360.0    # target spin angle per trial
OTOS_FW_MIN_SCALE = 0.872    # int8 = -128
OTOS_FW_MAX_SCALE = 1.127    # int8 = +127
BAUD = 115200


# ---------------------------------------------------------------------------
# Scale math (unit-testable; no hardware)
# ---------------------------------------------------------------------------

def scale_to_int8(scale: float) -> int:
    """Convert OTOS scale float to firmware int8 encoding.

    ``scale = 1.027`` → ``int8 = 27``.  Clamped to [-128, 127].
    """
    return max(-128, min(127, round((scale - 1.0) / 0.001)))


def int8_to_scale(val: int) -> float:
    """Decode firmware int8 back to float scale."""
    return 1.0 + val * 0.001


def compute_new_angular_scale(
    target_deg: float,
    otos_deg: float,
    current_scale: float,
) -> tuple[float, int]:
    """Compute recommended new OTOS angular scale.

    Formula: new_scale = (target_deg / otos_deg) * current_scale
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
    Returns positive for CCW (convention: firmware accumulates CCW positive).
    """
    delta_cdeg = after_cdeg - before_cdeg
    # Normalise to (-18000, 18000] — full 360° wrap
    while delta_cdeg > 18000:
        delta_cdeg -= 36000
    while delta_cdeg <= -18000:
        delta_cdeg += 36000
    return delta_cdeg / 100.0   # convert to degrees


def mean_stdev(values: list[float]) -> tuple[float, float]:
    """Return (mean, stdev) for a list of floats. Stdev = 0 for <2 elements."""
    if not values:
        return 0.0, 0.0
    n = len(values)
    m = sum(values) / n
    if n < 2:
        return m, 0.0
    var = sum((x - m) ** 2 for x in values) / (n - 1)
    return m, math.sqrt(var)


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

def _deep_merge(dst: dict, src: dict) -> None:
    for k, v in src.items():
        if k in dst and isinstance(dst[k], dict) and isinstance(v, dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v


def save_angular_calibration_to_config(
    path: Path,
    angular_scale: float,
    rotation_gain: float | None = None,
    rotation_gain_neg: float | None = None,
) -> None:
    """Write angular calibration fields into robot config JSON."""
    data = json.loads(path.read_text())
    cal: dict = {"otos_angular_scale": round(angular_scale, 6)}
    if rotation_gain is not None:
        cal["rotation_gain"] = round(rotation_gain, 6)
    if rotation_gain_neg is not None:
        cal["rotation_gain_neg"] = round(rotation_gain_neg, 6)
    updates: dict = {"calibration": cal}
    _deep_merge(data, updates)
    path.write_text(json.dumps(data, indent=2) + "\n")


def resolve_robot_config_path() -> Path | None:
    env_path = os.environ.get("ROBOT_CONFIG")
    if env_path:
        p = Path(env_path)
        if not p.is_absolute():
            p = _PROJECT_ROOT / p
        return p if p.exists() else None
    active = _PROJECT_ROOT / "data" / "robots" / "active_robot.json"
    if not active.exists():
        return None
    try:
        pointer = json.loads(active.read_text())
    except Exception:
        return None
    if "path" in pointer:
        return _PROJECT_ROOT / pointer["path"]
    return active


def load_current_angular_scale(config_path: Path) -> float:
    try:
        data = json.loads(config_path.read_text())
        return float(data.get("calibration", {}).get("otos_angular_scale", 1.0))
    except Exception:
        return 1.0


# ---------------------------------------------------------------------------
# Relay / direct serial helpers (same as calibrate_linear.py)
# ---------------------------------------------------------------------------

def _find_relay_port() -> str | None:
    reg = _PROJECT_ROOT / "config" / "devices.json"
    if reg.exists():
        for entry in json.loads(reg.read_text()).values():
            if (entry.get("role") or "").upper() == "RADIOBRIDGE" and entry.get("port"):
                return entry["port"]
    return None


def _find_robot_port() -> str | None:
    reg = _PROJECT_ROOT / "config" / "devices.json"
    if reg.exists():
        for entry in json.loads(reg.read_text()).values():
            role = (entry.get("role") or "").upper()
            if role in ("NEZHA2", "ROBOT") and entry.get("port"):
                return entry["port"]
    return None


class _RelaySerial:
    def __init__(self, port: str):
        import serial
        print(f"  Opening relay port {port} …")
        self._s = serial.Serial(port, BAUD, timeout=0.3)
        time.sleep(2.0)
        self._s.reset_input_buffer()

    def _line(self, text: str, wait: float = 0.4) -> str:
        self._s.write((text + "\n").encode())
        self._s.flush()
        time.sleep(wait)
        return self._s.read(8192).decode(errors="replace")

    def configure(self):
        banner = self._line("HELLO")
        print(f"  Relay: {banner.strip()}")
        self._line("!MODE RAW250", wait=0.3)
        self._line("!CG 0 10", wait=0.3)
        self._line("!P 7", wait=0.3)

    def go(self):
        resp = self._line("!GO", wait=0.8)
        print(f"  Relay data plane: {resp.strip()}")
        self._s.reset_input_buffer()

    def write_line(self, text: str):
        self._s.write((text + "\n").encode())
        self._s.flush()

    def read_available(self, timeout: float = 0.5) -> list[str]:
        lines: list[str] = []
        deadline = time.monotonic() + timeout
        buf = b""
        while time.monotonic() < deadline:
            chunk = self._s.read(4096)
            if chunk:
                buf += chunk
            parts = buf.replace(b"\r", b"").split(b"\n")
            buf = parts[-1]
            for p in parts[:-1]:
                s = p.decode(errors="replace").strip()
                if s:
                    lines.append(s)
            if not chunk:
                time.sleep(0.02)
        return lines

    def close(self):
        try:
            self._s.close()
        except Exception:
            pass


class _DirectSerial:
    def __init__(self, port: str):
        import serial
        print(f"  Opening direct robot port {port} …")
        self._s = serial.Serial(port, BAUD, timeout=0.3)
        time.sleep(1.5)
        self._s.reset_input_buffer()

    def write_line(self, text: str):
        self._s.write((text + "\n").encode())
        self._s.flush()

    def read_available(self, timeout: float = 0.5) -> list[str]:
        lines: list[str] = []
        deadline = time.monotonic() + timeout
        buf = b""
        while time.monotonic() < deadline:
            chunk = self._s.read(4096)
            if chunk:
                buf += chunk
            parts = buf.replace(b"\r", b"").split(b"\n")
            buf = parts[-1]
            for p in parts[:-1]:
                s = p.decode(errors="replace").strip()
                if s:
                    lines.append(s)
            if not chunk:
                time.sleep(0.02)
        return lines

    def close(self):
        try:
            self._s.close()
        except Exception:
            pass


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


def _snap_pose(ser, timeout: float = 3.0) -> tuple[int, int, int] | None:
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
    """Stream TLM frames until EVT done <verb> or timeout.

    Returns (pose_list, done_flag) where pose_list contains all
    (x_mm, y_mm, h_cdeg) tuples received while waiting.
    done_flag is True if EVT done was received.
    """
    target = f"EVT done {verb}"
    poses: list[tuple[int, int, int]] = []
    deadline = time.monotonic() + timeout
    done = False
    buf = b""
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
    """Interactive arrow-key heading adjustment. Returns final h_cdeg.

    The operator presses ← (CW nudge) or → (CCW nudge) to align the robot
    with the starting mark. Press Enter when aligned.

    Works only when stdin is a TTY; in non-TTY mode, returns current_h_cdeg
    unchanged (for scripted / piped usage).
    """
    if not sys.stdin.isatty():
        print("  (stdin is not a TTY — skipping interactive adjustment)")
        return current_h_cdeg

    import select
    import termios
    import tty

    h_cdeg = current_h_cdeg

    def _read_key(fd: int) -> str:
        ch = os.read(fd, 1).decode("utf-8", errors="replace")
        if ch == "\x1b":
            if select.select([fd], [], [], 0.1)[0]:
                rest = os.read(fd, 2).decode("utf-8", errors="replace")
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
        hint = " ← " if err_deg > 0 else " → "
        print(f"\r  heading={h_cdeg / 100:+.2f}°  "
              f"err={err_deg:+.2f}°  [← CW  → CCW  Enter=done]{hint}   ",
              end="", flush=True)

    try:
        import os as _os
        tty.setraw(fd)
        _show()
        while True:
            if select.select([fd], [], [], 0.02)[0]:
                k = _read_key(fd)
                if k == "enter":
                    break
                if k in ("esc", "ctrl-c"):
                    break
                if k == "left":   # CW nudge (decreases heading)
                    ser.write_line(f"T -{nudge_speed} {nudge_speed} {nudge_ms}")
                elif k == "right":   # CCW nudge (increases heading)
                    ser.write_line(f"T {nudge_speed} -{nudge_speed} {nudge_ms}")
                time.sleep(nudge_ms / 1000.0 + 0.1)
            # Read any pending TLM frames to update heading
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
# Main
# ---------------------------------------------------------------------------

import os   # noqa: E402  (after sys.path set)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=int, default=TURN_SPEED_MMS,
                        help=f"Wheel speed for spin (default {TURN_SPEED_MMS} mm/s)")
    parser.add_argument("--angle", type=float, default=DEFAULT_ANGLE_DEG,
                        help=f"Target spin angle per trial in degrees (default {DEFAULT_ANGLE_DEG})")
    parser.add_argument("--port", default=None,
                        help="Serial port override (relay or robot)")
    parser.add_argument("--direct", action="store_true",
                        help="Connect directly to robot USB (skip relay)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Compute and print recommended scale but do not write config")
    args = parser.parse_args()

    target_deg = args.angle
    speed_mms = args.speed

    # ── Find config ──────────────────────────────────────────────────────────
    config_path = resolve_robot_config_path()
    current_scale = 1.0
    if config_path and config_path.exists():
        current_scale = load_current_angular_scale(config_path)
        print(f"  Config: {config_path}")
    else:
        print("  WARNING: No robot config found — using otos_angular_scale = 1.0")
    current_int8 = scale_to_int8(current_scale)
    print(f"  Current otos_angular_scale = {current_scale:.4f}  (int8={current_int8:+d})")

    # ── Connect ──────────────────────────────────────────────────────────────
    ser = None
    try:
        if args.port:
            if args.direct:
                ser = _DirectSerial(args.port)
            else:
                ser = _RelaySerial(args.port)
                ser.configure()
                ser.go()
        elif args.direct:
            port = _find_robot_port()
            if port is None:
                print("ERROR: No direct robot port found.", file=sys.stderr)
                sys.exit(1)
            ser = _DirectSerial(port)
        else:
            port = _find_relay_port()
            if port is not None:
                ser = _RelaySerial(port)
                ser.configure()
                ser.go()
                print("  Connected via relay.")
            else:
                port = _find_robot_port()
                if port is None:
                    print("ERROR: No relay or robot port found.", file=sys.stderr)
                    sys.exit(1)
                ser = _DirectSerial(port)
                print("  Connected directly to robot.")
    except Exception as e:
        print(f"ERROR: Could not connect: {e}", file=sys.stderr)
        sys.exit(1)

    # ── Ping & set angular scalar ─────────────────────────────────────────────
    print("\n  Checking link (PING)…")
    lines = _send_and_wait(ser, "PING", "OK pong", timeout=3.0)
    if not any("pong" in ln for ln in lines):
        print("  WARNING: no PING reply — robot may not be reachable.")
    else:
        print("  Robot responding.")

    print(f"  Setting OA {current_int8:+d} (scale={current_scale:.4f}) on hardware…")
    _send_and_wait(ser, f"OA {current_int8}", "OK", timeout=2.0)

    print(f"\n  Target spin angle: {target_deg:.1f}°  Speed: {speed_mms} mm/s")
    print("  Aim the robot's marker at a reference point on the wall/floor.")
    print("  Trials alternate CCW / CW.  Use ← → to nudge back to the mark.")
    print("  Press Enter to spin each trial, 'q' to finish.\n")

    # Samples: list of (direction_sign, otos_deg)
    # direction_sign: +1 = CCW, -1 = CW
    samples: list[tuple[int, float]] = []
    direction = +1   # start CCW

    try:
        while True:
            n = len(samples)
            label = "CCW ↺" if direction > 0 else "CW ↻"
            print(f"[Trial {n + 1}]  ({n} samples)  next: {label}  — Enter to spin, 'q' to finish")
            try:
                raw = input().strip()
            except EOFError:
                break
            if raw.lower() in ("q", "quit", "exit"):
                break

            # Zero pose before spin
            _send_and_wait(ser, "ZERO enc pose", "OK", timeout=2.0)
            time.sleep(0.15)

            # Read initial pose (should be 0 after zero)
            pose_before = _snap_pose(ser, timeout=2.0)
            h_before_cdeg = pose_before[2] if pose_before else 0

            # Enable TLM streaming at 20 ms so we accumulate heading
            _send_and_wait(ser, "STREAM 20", "OK", timeout=2.0)
            time.sleep(0.1)

            # Spin: use timed T command for approximately target_deg
            # Trackwidth = ~126mm; spin (mm/s) * t_s = π * trackwidth * deg/360
            # t_ms = (target_deg / 360) * π * trackwidth_mm / speed_mms * 1000
            # Estimate 126mm trackwidth if unknown
            trackwidth_mm = 126.0
            t_ms = int((target_deg / 360.0) * math.pi * trackwidth_mm / speed_mms * 1000)
            t_ms = max(500, min(15000, t_ms))  # safety clamp

            l_speed = direction * speed_mms
            r_speed = -direction * speed_mms
            print(f"  Spinning {label} for ~{t_ms}ms ({target_deg:.0f}°)…")

            # Send T command and collect TLM while waiting for EVT done T
            spin_cmd = f"T {l_speed} {r_speed} {t_ms}"
            _send_and_wait(ser, spin_cmd, "OK", timeout=2.0)
            timeout_s = t_ms / 1000.0 + 5.0
            poses_during, done_flag = _stream_tlm_until_evt(ser, "T", timeout=timeout_s)

            if not done_flag:
                print("  WARNING: Did not receive EVT done T — spin may have been incomplete.")

            # Disable streaming
            _send_and_wait(ser, "STREAM 0", "OK", timeout=2.0)
            time.sleep(0.2)

            # Read final OTOS heading
            pose_after = _snap_pose(ser, timeout=3.0)
            h_after_cdeg = pose_after[2] if pose_after else (
                poses_during[-1][2] if poses_during else h_before_cdeg
            )

            otos_deg_raw = heading_delta_cdeg(h_before_cdeg, h_after_cdeg)
            print(f"  OTOS heading: before={h_before_cdeg / 100:.2f}°  "
                  f"after={h_after_cdeg / 100:.2f}°  delta={otos_deg_raw:+.2f}°")
            print(f"  Target: {direction * target_deg:+.1f}°  "
                  f"Error: {otos_deg_raw - direction * target_deg:+.2f}°")

            # Interactive adjustment — operator nudges robot to exact mark
            print(f"\n  Nudge the robot back to the starting mark with ← → keys.")
            print(f"  Press Enter when aligned.")
            h_adjusted_cdeg = _interactive_adjust(
                ser, h_after_cdeg,
                target_deg=direction * target_deg,
                nudge_speed=80, nudge_ms=50,
            )
            ser.write_line("STOP")
            time.sleep(0.1)

            # Final heading after adjustment = physical rotation (ground truth)
            adjusted_deg = heading_delta_cdeg(h_before_cdeg, h_adjusted_cdeg)
            err_deg = adjusted_deg - direction * target_deg
            print(f"  Adjusted: otos={adjusted_deg:+.2f}°  "
                  f"target={direction * target_deg:+.1f}°  "
                  f"err={err_deg:+.2f}°")

            if abs(adjusted_deg) < 10.0:
                print("  WARNING: adjusted heading < 10° — possible misread. Discarded.")
            else:
                samples.append((direction, abs(adjusted_deg)))
                print(f"  Sample {len(samples)} recorded: "
                      f"dir={label}  adjusted={abs(adjusted_deg):.2f}°")

            direction = -direction   # alternate CCW/CW

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write_line("STOP")
            ser.write_line("STREAM 0")
            time.sleep(0.2)
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass

    # ── Statistics ───────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print(f"Samples collected: {len(samples)}")
    if len(samples) < 2:
        print("Need ≥ 2 samples — not enough data.")
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
              f"{d - target_deg:>+7.2f}°")

    mean_all, std_all = mean_stdev(ratios)
    mean_ccw_d, std_ccw_d = mean_stdev(ccw_degs)
    mean_cw_d,  std_cw_d  = mean_stdev(cw_degs)

    # Angular scale: apply mean ratio to current scale
    new_scale_raw = mean_all * current_scale
    new_scale_clamped = max(OTOS_FW_MIN_SCALE, min(OTOS_FW_MAX_SCALE, new_scale_raw))
    new_int8 = scale_to_int8(new_scale_clamped)
    rounded_scale = int8_to_scale(new_int8)

    # Per-direction gains (relative to target_deg)
    rot_gain = target_deg / mean_ccw_d if mean_ccw_d > 0 else None
    rot_gain_neg = target_deg / mean_cw_d if mean_cw_d > 0 else None

    print(f"\nRatio statistics (target_deg / otos_deg):")
    print(f"  Overall: mean={mean_all:.4f}  stdev={std_all:.4f}  (n={len(ratios)})")
    if ccw_degs:
        print(f"  CCW otos_deg: mean={mean_ccw_d:.2f}°  stdev={std_ccw_d:.2f}°  (n={len(ccw_degs)})")
    if cw_degs:
        print(f"  CW  otos_deg: mean={mean_cw_d:.2f}°  stdev={std_cw_d:.2f}°  (n={len(cw_degs)})")

    print(f"\nRecommended values:")
    print(f"  otos_angular_scale = {current_scale:.4f} × {mean_all:.4f} = {new_scale_raw:.4f}")
    print(f"  clamped = {new_scale_clamped:.4f}  →  int8={new_int8:+d}  → {rounded_scale:.4f}")
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

    if args.dry_run:
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


if __name__ == "__main__":
    main()
