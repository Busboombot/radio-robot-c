"""Calibrate the OTOS linear scale over the v2 radio-relay data plane.

Drives the robot a measured distance via the ``D`` command (blocking, waits
for ``EVT done D``), then reads OTOS position from a ``SNAP``/``TLM pose=``
frame.  The operator measures the actual distance with a tape measure and
enters it.  Multiple samples are collected; the script computes the mean
ratio and recommended new ``otos_linear_scale`` encoded as an int8.

OTOS pose= units after Sprint 012/T03: x and y are in mm (integer).
Heading is in centi-degrees (integer).  The scale formula is:

    new_scale = actual_mm / otos_x_mm * current_scale

encoded as int8 via: ``int8 = round((scale - 1.0) / 0.001)`` clamped to
[-128, 127].

After ≥ 2 samples the script offers to write the result back to the active
robot config (``data/robots/<robot>.json``).

Connection: relay first, direct robot USB if relay is not found.

Procedure:
  1. Mark the robot starting position on the floor.
  2. Press Enter — the robot drives the target distance via ``D <l> <r> <mm>``.
  3. After ``EVT done D``, measure the actual distance with a tape measure.
  4. Enter the actual distance in cm.
  5. Repeat for ≥ 3 samples.  Ctrl-C or 'q' to finish and see results.

Usage:
    uv run python calibrate_linear.py [--distance CM] [--speed MMS] [--dry-run]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path

_HOST_ROOT = Path(__file__).resolve().parent
_PROJECT_ROOT = _HOST_ROOT.parent
sys.path.insert(0, str(_HOST_ROOT))

from robot_radio.robot.protocol import NezhaProtocol, parse_tlm, TLMFrame

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DRIVE_SPEED_MMS = 200        # mm/s forward
DEFAULT_DISTANCE_CM = 50.0   # default target distance for a single trial
OTOS_FW_MIN_SCALE = 0.872    # int8 = -128
OTOS_FW_MAX_SCALE = 1.127    # int8 = +127
BAUD = 115200
WATCHDOG_MS = 3000           # give D command generous watchdog


# ---------------------------------------------------------------------------
# Scale math (unit-testable; no hardware)
# ---------------------------------------------------------------------------

def scale_to_int8(scale: float) -> int:
    """Convert OTOS scale float to firmware int8 encoding.

    Firmware stores OL/OA as signed offset from 1.0 in units of 0.001.
    ``scale = 1.027`` → ``int8 = 27``.  Clamped to [-128, 127].
    """
    return max(-128, min(127, round((scale - 1.0) / 0.001)))


def int8_to_scale(val: int) -> float:
    """Decode firmware int8 back to float scale."""
    return 1.0 + val * 0.001


def compute_new_linear_scale(
    actual_mm: float,
    otos_mm: float,
    current_scale: float,
) -> tuple[float, int]:
    """Compute recommended new OTOS linear scale.

    Formula: new_scale = (actual_mm / otos_mm) * current_scale
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
# Config helpers (unit-testable)
# ---------------------------------------------------------------------------

def _deep_merge(dst: dict, src: dict) -> None:
    """Recursively merge src into dst in-place."""
    for k, v in src.items():
        if k in dst and isinstance(dst[k], dict) and isinstance(v, dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v


def save_linear_scale_to_config(path: Path, new_scale: float) -> None:
    """Write otos_linear_scale into calibration section of robot JSON."""
    data = json.loads(path.read_text())
    updates = {"calibration": {"otos_linear_scale": round(new_scale, 6)}}
    _deep_merge(data, updates)
    path.write_text(json.dumps(data, indent=2) + "\n")


def resolve_robot_config_path() -> Path | None:
    """Resolve active robot config path from env var or active_robot.json pointer."""
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


def load_current_linear_scale(config_path: Path) -> float:
    """Read otos_linear_scale from robot config JSON, default 1.0."""
    try:
        data = json.loads(config_path.read_text())
        return float(data.get("calibration", {}).get("otos_linear_scale", 1.0))
    except Exception:
        return 1.0


# ---------------------------------------------------------------------------
# Relay connection helper (mirrors radio_drive_test.py pattern)
# ---------------------------------------------------------------------------

def _find_relay_port() -> str | None:
    """Find RADIOBRIDGE relay port from config/devices.json."""
    reg = _PROJECT_ROOT / "config" / "devices.json"
    if reg.exists():
        for entry in json.loads(reg.read_text()).values():
            if (entry.get("role") or "").upper() == "RADIOBRIDGE" and entry.get("port"):
                return entry["port"]
    return None


def _find_robot_port() -> str | None:
    """Find direct NEZHA2 robot port from config/devices.json."""
    reg = _PROJECT_ROOT / "config" / "devices.json"
    if reg.exists():
        for entry in json.loads(reg.read_text()).values():
            role = (entry.get("role") or "").upper()
            if role in ("NEZHA2", "ROBOT") and entry.get("port"):
                return entry["port"]
    return None


class _RelaySerial:
    """Thin wrapper around a pyserial port for relay + transparent data plane."""

    def __init__(self, port: str):
        import serial
        print(f"  Opening relay port {port} …")
        self._s = serial.Serial(port, BAUD, timeout=0.3)
        time.sleep(2.0)   # DTR reset + boot
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
    """Thin wrapper for direct robot serial (no relay handshake needed)."""

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


# ---------------------------------------------------------------------------
# Wire helpers over the low-level serial (no SerialConnection abstraction)
# ---------------------------------------------------------------------------

def _send_and_wait(ser, cmd: str, want_prefix: str, timeout: float = 5.0) -> list[str]:
    """Send cmd; collect lines until one starts with want_prefix or timeout."""
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
    """Send SNAP; parse the TLM pose= field. Returns (x_mm, y_mm, h_cdeg) or None."""
    lines = _send_and_wait(ser, "SNAP", "TLM", timeout=timeout)
    for line in lines:
        frame = parse_tlm(line)
        if frame is not None and frame.pose is not None:
            return frame.pose
    return None


def _snap_enc(ser, timeout: float = 3.0) -> tuple[int, int] | None:
    """Send SNAP; parse TLM enc= field. Returns (left_mm, right_mm) or None."""
    lines = _send_and_wait(ser, "SNAP", "TLM", timeout=timeout)
    for line in lines:
        frame = parse_tlm(line)
        if frame is not None and frame.enc is not None:
            return frame.enc
    return None


def _wait_evt_done(ser, verb: str, timeout: float = 30.0) -> bool:
    """Block until EVT done <verb> arrives. Returns True if seen, False if timeout."""
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
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--distance", type=float, default=DEFAULT_DISTANCE_CM,
                        help=f"Drive distance per trial in cm (default {DEFAULT_DISTANCE_CM})")
    parser.add_argument("--speed", type=int, default=DRIVE_SPEED_MMS,
                        help=f"Drive speed mm/s (default {DRIVE_SPEED_MMS})")
    parser.add_argument("--port", default=None,
                        help="Serial port override (relay or robot, auto-detected otherwise)")
    parser.add_argument("--direct", action="store_true",
                        help="Connect directly to the robot USB (skip relay)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Compute and print recommended scale but do not write config")
    args = parser.parse_args()

    target_cm = args.distance
    target_mm = round(target_cm * 10)
    speed_mms = args.speed

    # ── Find config ──────────────────────────────────────────────────────────
    config_path = resolve_robot_config_path()
    current_scale = 1.0
    if config_path and config_path.exists():
        current_scale = load_current_linear_scale(config_path)
        print(f"  Config: {config_path}")
    else:
        print("  WARNING: No robot config found — using otos_linear_scale = 1.0")
    current_int8 = scale_to_int8(current_scale)
    print(f"  Current otos_linear_scale = {current_scale:.4f}  (int8={current_int8:+d})")

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
                print("ERROR: No direct robot port found in config/devices.json.", file=sys.stderr)
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
                    print("ERROR: No relay or robot port found. Pass --port or --direct.", file=sys.stderr)
                    sys.exit(1)
                ser = _DirectSerial(port)
                print("  Connected directly to robot.")
    except Exception as e:
        print(f"ERROR: Could not connect: {e}", file=sys.stderr)
        sys.exit(1)

    # ── Ping & zero ──────────────────────────────────────────────────────────
    print("\n  Checking link (PING)…")
    lines = _send_and_wait(ser, "PING", "OK pong", timeout=3.0)
    if not any("pong" in ln for ln in lines):
        print("  WARNING: no PING reply — robot may not be reachable.")
    else:
        print("  Robot responding.")

    print("  Zeroing pose and encoders…")
    _send_and_wait(ser, "ZERO enc pose", "OK", timeout=2.0)
    time.sleep(0.2)

    # ── Set current linear scalar on OTOS hardware ───────────────────────────
    print(f"  Setting OL {current_int8:+d} (scale={current_scale:.4f}) on hardware…")
    _send_and_wait(ser, f"OL {current_int8}", "OK", timeout=2.0)

    print(f"\n  Target distance: {target_cm:.1f} cm  Speed: {speed_mms} mm/s")
    print("  Mark the robot's starting position on the floor.")
    print("  Press Enter to drive each trial, 'q' to finish and see results.\n")

    samples: list[tuple[float, float]] = []   # (otos_mm, actual_mm)

    try:
        while True:
            n = len(samples)
            print(f"[Trial {n + 1}]  ({n} samples so far)  — Enter to drive, 'q' to finish")
            try:
                raw = input().strip()
            except EOFError:
                break
            if raw.lower() in ("q", "quit", "exit"):
                break

            # Zero before each drive
            _send_and_wait(ser, "ZERO enc pose", "OK", timeout=2.0)
            time.sleep(0.15)

            # Drive the target distance
            print(f"  Driving {target_cm:.1f} cm …")
            timeout_s = (target_mm / max(speed_mms, 1)) * 2.5 + 5.0
            _send_and_wait(ser, f"D {speed_mms} {speed_mms} {target_mm}", "OK", timeout=2.0)
            done = _wait_evt_done(ser, "D", timeout=timeout_s)
            if not done:
                print("  WARNING: Did not receive EVT done D — robot may have stopped early.")
            time.sleep(0.3)

            # Read OTOS pose after drive
            pose = _snap_pose(ser, timeout=3.0)
            if pose is None:
                print("  WARNING: Could not read OTOS pose — skipping trial.")
                continue

            otos_x_mm = pose[0]   # forward displacement (x axis after ZERO)
            enc = _snap_enc(ser, timeout=2.0)
            enc_mm_str = f"L={enc[0]}mm R={enc[1]}mm" if enc else "N/A"

            print(f"  OTOS pose: x={otos_x_mm}mm  y={pose[1]}mm  h={pose[2]/100:.1f}°")
            print(f"  Encoders:  {enc_mm_str}")
            print(f"  Measure the actual distance traveled with a tape measure.")
            print(f"  Press Enter with no value to discard this trial.")

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
                print("  OTOS x ≈ 0 — sensor may not be responding. Discarded.")
                continue

            ratio = actual_mm / abs(otos_x_mm)
            if not (0.4 <= ratio <= 2.5):
                print(f"  WARNING: ratio {ratio:.3f} is out of range [0.4, 2.5] — "
                      f"check units (enter cm, not mm). Discarded.")
                continue

            samples.append((abs(otos_x_mm), actual_mm))
            err = actual_mm - abs(otos_x_mm)
            print(f"  Sample {len(samples)}: otos={otos_x_mm}mm  actual={actual_mm:.1f}mm  "
                  f"err={err:+.1f}mm ({err / abs(otos_x_mm) * 100:+.1f}%)  ratio={ratio:.4f}")

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write_line("STOP")
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

    print(f"\n  ratio  mean={mean_r:.4f}  stdev={stdev_r:.4f}  sem={sem_r:.4f}  (n={len(samples)})")
    print(f"  new scale = current ({current_scale:.4f}) × mean_ratio ({mean_r:.4f}) = {new_scale_raw:.4f}")
    print(f"  clamped   = {new_scale_clamped:.4f}  →  int8={new_int8:+d}  → rounded_scale={rounded_scale:.4f}")

    print(f"\n  To set manually:")
    print(f'    "otos_linear_scale": {rounded_scale:.4f}  # int8={new_int8:+d},'
          f' n={len(samples)}, stdev={stdev_r:.4f}')

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
        save_linear_scale_to_config(config_path, rounded_scale)
        print(f"  Saved otos_linear_scale = {rounded_scale:.4f} to {config_path}")
    else:
        print("  Not saved.")


if __name__ == "__main__":
    main()
