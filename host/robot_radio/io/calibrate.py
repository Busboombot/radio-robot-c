"""Interactive multi-trial calibration commands for the Nezha robot.

Two subcommands:
  rogo calibrate distance [--distance CM] [--speed MMS]
      Drives forward a target distance per trial, records camera/OTOS/encoder
      distances, prompts for tape-measured actual, then saves updated
      otos_linear_scale and mm_per_wheel_deg_{left,right} to the active config.

  rogo calibrate turns [--auto] [--trials N]
      Spins target_deg per trial (even=CCW, odd=CW) via firmware TN closed-loop
      command.  Reads before/after yaw from aprilcam daemon (DaemonControl) as
      camera ground truth.  Accumulates OTOS yaw from the SO streaming channel
      during the spin.  After each trial, pushes an updated OA value to firmware
      to adaptively converge otos_angular_scale.  Tracks per-wheel encoder mm
      to estimate trackwidth.  Convergence detected when residual signs alternate
      for 3 consecutive trials.  At convergence, offers to push K+TW, K+ML,
      K+MR and save to config.

Both commands require ≥ 3 trials before computing statistics.

Design notes (turns):
- Firmware TN<deg_tenths> (signed, CCW positive) runs an on-robot OTOS
  closed-loop turn.  Replies ACK:TN immediately, then TN+DONE <achieved>
  on success or TN+TIMEOUT <achieved>.  e.g. TN+900 = +90° CCW.
- SO stream and TN+DONE reply arrive on the same serial channel — parsed in
  one read loop using read_lines(duration_ms=100) bursts.  TX echo lines
  (containing "TX:") are skipped; leading "<" is stripped by parse_so() via
  _strip() which does lstrip("<# ").
- DaemonControl is ground truth (authoritative A1-centred world frame).
  The known +90° yaw offset between daemon yaw and drive-forward CANCELS
  when computing before/after yaw DELTAS — only deltas are used for turn
  calibration, so no correction is needed.
- Adaptive OA push: new_scale = current * (cam_deg / otos_total_deg).
  Clamped to the firmware representable range [0.872, 1.128].
- Arrow-key nudges use termios/tty.setraw (same pattern as
  test/system/carriage_explorer.py:_read_key).  Nudge size: 50ms at ±30 mm/s.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional

_PROJECT_ROOT = Path(__file__).parent.parent.parent

# ---------------------------------------------------------------------------
# Stats helpers
# ---------------------------------------------------------------------------


def _mean_stdev(xs: list[float]) -> tuple[float, float]:
    """Sample mean and standard deviation (Bessel-corrected).

    Returns (mean, 0.0) for a single-element list; (0.0, 0.0) for empty.
    """
    if not xs:
        return (0.0, 0.0)
    n = len(xs)
    m = sum(xs) / n
    if n < 2:
        return (m, 0.0)
    var = sum((x - m) ** 2 for x in xs) / (n - 1)
    return (m, math.sqrt(var))


# ---------------------------------------------------------------------------
# Config save helpers
# ---------------------------------------------------------------------------


def _deep_merge(dst: dict, src: dict) -> None:
    """Recursively update dst with values from src in-place."""
    for k, v in src.items():
        if k in dst and isinstance(dst[k], dict) and isinstance(v, dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v


def _save_config(path: Path, updates: dict) -> None:
    """Read existing JSON, deep-merge updates, write back with indent=2."""
    data = json.loads(path.read_text())
    _deep_merge(data, updates)
    path.write_text(json.dumps(data, indent=2) + "\n")


def _resolve_save_path() -> Optional[Path]:
    """Resolve the path to the active robot config for saving.

    Follows the same resolution logic as get_robot_config():
      1. ROBOT_CONFIG env var — full path.
      2. data/robots/active_robot.json — pointer or full config.
    Returns the resolved Path, or None if not found.
    """
    import os

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
    # active_robot.json is itself the full config
    return active


def _prompt_save(updates: dict, label: str) -> None:
    """Prompt the user to save `updates` to the active config (or a custom path).

    Accepts:
      Y / empty  — save to the resolved active config path
      n          — skip
      <filename> — save to that path (relative to project root)
    """
    save_path = _resolve_save_path()
    default_display = str(save_path.relative_to(_PROJECT_ROOT)) if save_path else "<unknown>"

    print(f"\n{label}")
    try:
        raw = input(f"Save to {default_display}? [Y/n/<filename>] ").strip()
    except EOFError:
        raw = ""

    if raw.lower() == "n":
        print("Skipped — no changes saved.")
        return

    if raw and raw.lower() not in ("y", "yes", ""):
        # User typed a filename
        target = Path(raw)
        if not target.is_absolute():
            target = _PROJECT_ROOT / target
    else:
        if save_path is None:
            print("Error: cannot resolve active robot config path.", file=sys.stderr)
            return
        target = save_path

    try:
        _save_config(target, updates)
        print(f"Saved to {target}")
    except Exception as e:
        print(f"Error saving config: {e}", file=sys.stderr)


# ---------------------------------------------------------------------------
# Arrow-key reader (termios, same pattern as carriage_explorer.py)
# ---------------------------------------------------------------------------


def _read_key() -> str:
    """Read one keypress from stdin. Returns single char or escape sequence.

    Decodes arrow keys to '\x1b[A' (up), '\x1b[B' (down),
    '\x1b[C' (right), '\x1b[D' (left).
    Raises OSError if stdin is not a TTY.
    """
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            ch2 = sys.stdin.read(1)
            if ch2 == "[":
                ch3 = sys.stdin.read(1)
                return f"\x1b[{ch3}"
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ---------------------------------------------------------------------------
# Camera helper
# ---------------------------------------------------------------------------


def _resolve_camera_index(arg_idx: Optional[int]) -> int:
    """Resolve camera index from --camera arg > $CAMERA > auto-discover OV9782.

    Auto-discovery scans connected cameras and picks the one whose name
    contains 'OV9782' or 'Arducam'. If none match, falls back to the
    first available index.
    """
    if arg_idx is not None:
        return arg_idx
    env = os.environ.get("CAMERA")
    if env:
        return int(env)
    # Auto-discover: prefer OV9782, then Arducam, then first available
    try:
        from aprilcam.camera.camutil import list_cameras
        cams = list(list_cameras())
    except Exception:
        cams = []
    for c in cams:
        name = (getattr(c, "name", "") or "").lower()
        if "ov9782" in name:
            return c.index
    for c in cams:
        name = (getattr(c, "name", "") or "").lower()
        if "arducam" in name:
            return c.index
    if cams:
        return cams[0].index
    return 0


def _get_tag_yaw(field, tag_id: int, timeout_s: float = 3.0) -> Optional[float]:
    """Poll `field` for tag `tag_id` and return its world yaw in radians.

    aprilcam.Tag.orientation reports world-CCW-positive yaw (verified
    2026-05-28). Note: zero direction is +Y (not +X), so this is not the
    standard math convention — but rotation deltas are correct (positive = CCW).
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        tag = field.tag(tag_id)
        if tag is not None and tag.orientation is not None:
            return float(tag.orientation)
        time.sleep(0.05)
    return None


def _signed_turn_deg(yaw_before_rad: float, yaw_after_rad: float,
                     expected_deg: float = 360.0) -> float:
    """Compute actual signed turn in degrees from before/after yaw readings,
    picking the multiple of 360° closest to `expected_deg`.

    Camera yaw is reported wrapped to (-π, π]. A full 360° spin reads as a
    delta near 0; we re-add the integer-turn count nearest the expectation.
    """
    delta = (yaw_after_rad - yaw_before_rad + math.pi) % (2.0 * math.pi) - math.pi
    delta_deg = math.degrees(delta)
    n_full = round((expected_deg - delta_deg) / 360.0)
    return n_full * 360.0 + delta_deg


def _get_tag_world_xy(field, tag_id: int, timeout_s: float = 3.0) -> Optional[tuple[float, float]]:
    """Poll `field` for tag `tag_id` and return (wx_cm, wy_cm) or None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        tag = field.tag(tag_id)
        if tag is not None and tag.wx is not None and tag.wy is not None:
            return (tag.wx, tag.wy)
        time.sleep(0.05)
    return None


def _get_tag_world_yaw(field, tag_id: int, timeout_s: float = 3.0) -> Optional[float]:
    """Poll `field` for tag `tag_id` and return yaw in degrees or None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        tag = field.tag(tag_id)
        if tag is not None:
            return math.degrees(tag.orientation)
        time.sleep(0.05)
    return None


# ---------------------------------------------------------------------------
# Shared connection helper
# ---------------------------------------------------------------------------


def _make_proto_cfg(args):
    """Return (proto, conn, cfg) connecting to the robot.

    Uses the same auto-detect logic as cli.py's _make_robot().
    """
    import os
    from robot_radio.io.serial_conn import SerialConnection, list_serial_ports, DEFAULT_PORT
    from robot_radio.robot.protocol import NezhaProtocol
    from robot_radio.config.robot_config import get_robot_config

    port = getattr(args, "port", None)
    if not port:
        ports = list_serial_ports()
        if not ports:
            print("Error: No USB modem ports found.", file=sys.stderr)
            sys.exit(1)
        port = ports[0]

    conn = SerialConnection(port)
    result = conn.connect()
    if "error" in result:
        print(f"Error: {result['error']}", file=sys.stderr)
        sys.exit(1)

    proto = NezhaProtocol(conn)
    cfg = get_robot_config()
    return proto, conn, cfg


# ---------------------------------------------------------------------------
# cmd_calibrate_distance
# ---------------------------------------------------------------------------


def cmd_calibrate_distance(args) -> None:
    """Multi-trial linear (distance) calibration via straight drive + daemon GT.

    Per trial:
      1. Read camera start position from the aprilcam daemon (authoritative GT).
      2. Zero encoders + OTOS.
      3. Drive straight --distance cm at --speed mm/s (firmware D command).
      4. Read encoder mm, OTOS pose, camera end position from the daemon.
      5. actual = camera displacement (auto) or tape-measured prompt (interactive).
      6. Adaptive OL push: new_scale = current × (actual_cm / otos_cm); send
         OL<int8> so the next trial's OTOS uses the improved linear scale.
      7. Accumulate samples.

    After ≥ 3 trials: compute otos/encoder/camera ratios, recommend a new
    otos_linear_scale, offer to save to active robot config.

    Notes (measured on the recalibrated playfield, 2026-05-29):
    - The D-command drive is accurate (~1.0× commanded distance); the earlier
      apparent ~1.45× over-drive was a skewed-playfield artifact, not a real
      wheel-calibration error.  This command therefore converges the OTOS
      *linear scalar* (OL register), not the wheel mm/deg.  Config currently
      holds otos_linear_scale ≈ 1.05 (pushed as OL+50).
    - Ground truth is the daemon's A1-centred world frame, not the stale local
      data/homography.json.  Distance uses Euclidean displacement of the robot
      tag's world_xy, so the daemon's frame origin/orientation cancels.

    --auto    Use camera Δ as ground truth (no tape-measure prompt).
    --trials  Number of auto trials (default 3, only used with --auto).
    --distance  Target distance in cm (default 40).
    """
    from robot_radio.io import cli as _cli
    from robot_radio.io.cli import _scale_to_int8

    verbose = bool(getattr(_cli, "_verbose", False) or getattr(args, "verbose", False))

    target_cm = float(args.distance)    # cm
    target_mm = int(round(target_cm * 10))
    speed_mms = int(args.speed)

    proto, conn, cfg = _make_proto_cfg(args)

    tag_id = cfg.vision.robot_tag_id if cfg else 100

    # Derive current mm/deg from calibration or wheel_diameter
    cal = getattr(cfg, "calibration", None)
    left_mm_per_deg_current = getattr(cal, "mm_per_wheel_deg_left", None) if cal else None
    right_mm_per_deg_current = getattr(cal, "mm_per_wheel_deg_right", None) if cal else None
    wd = getattr(getattr(cfg, "wheels", None), "wheel_diameter_mm", None)
    default_mm_per_deg = (math.pi * wd / 360.0) if wd is not None else None
    left_mm_per_deg_current = left_mm_per_deg_current or default_mm_per_deg
    right_mm_per_deg_current = right_mm_per_deg_current or default_mm_per_deg

    otos_linear_scale_current = getattr(cal, "otos_linear_scale", 1.0) if cal else 1.0

    # Physically implausible single-trial displacement: a 30–40 cm drive must
    # NOT be dropped, so the guard sits far above any real trial (was the
    # distance analog of the turns cal's >30°/tick drop bug — now removed).
    MAX_PLAUSIBLE_CM = 200.0

    # ── Connect to aprilcam daemon (best-effort; degrade gracefully if absent) ──
    dc = None
    daemon_cam = None
    try:
        from aprilcam.config import Config
        from aprilcam.client.control import DaemonControl
        dc = DaemonControl.connect_default(Config.load())
        cams = dc.list_cameras()
        if cams:
            daemon_cam = cams[0]
            print(f"  Daemon connected, camera={daemon_cam}, tag_id={tag_id}")
        else:
            print("  Warning: daemon has no cameras open — camera GT unavailable.",
                  file=sys.stderr)
            dc.close()
            dc = None
    except Exception as exc:
        print(f"  Warning: could not connect to aprilcam daemon ({exc}).",
              file=sys.stderr)
        print("  Camera ground truth unavailable.", file=sys.stderr)
        dc = None

    # Samples: list of (cam_cm, otos_cm, enc_left_cm, enc_right_cm, actual_cm)
    samples: list[tuple[float, float, float, float, float]] = []
    trial = 0

    auto_mode = bool(getattr(args, "auto", False))
    auto_trials = int(getattr(args, "trials", 3))

    if auto_mode:
        print(f"\nDistance calibration: AUTO mode, {auto_trials} trials, "
              f"target={target_cm:.1f} cm  speed={speed_mms} mm/s")
        print("Camera (daemon) provides ground truth — no tape measure prompts.")
        print(f"Starting otos_linear_scale = {otos_linear_scale_current:.4f}\n")
    else:
        print(f"\nDistance calibration: target={target_cm:.1f} cm  speed={speed_mms} mm/s")
        print("Position the robot facing forward, press Enter to start each trial.")
        print("Type 'q' to finish and compute statistics.\n")

    try:
        while True:
            trial += 1
            if auto_mode:
                if trial > auto_trials:
                    break
                print(f"\nAuto trial {trial}/{auto_trials}: driving {target_cm:.1f} cm.")
            else:
                print(f"Trial {trial}: aim robot, press Enter to start.  q to finish.")
                try:
                    raw = input().strip()
                except EOFError:
                    break
                if raw.lower() == "q":
                    break

            # Step 1: read camera start position (daemon GT)
            pos0: Optional[tuple[float, float]] = None
            if dc is not None and daemon_cam is not None:
                pose0 = _daemon_read_pose_local(dc, daemon_cam, tag_id, timeout_s=3.0)
                if pose0 is not None:
                    pos0 = (pose0[0], pose0[1])
            if pos0 is None:
                print(f"  Warning: tag {tag_id} not seen — using (0,0) as start.")
                pos0 = (0.0, 0.0)
            else:
                print(f"  Camera start: ({pos0[0]:.2f}, {pos0[1]:.2f}) cm")

            # Step 2: zero sensors
            proto.zero_encoders()
            proto.zero_otos()
            time.sleep(0.1)

            # Step 3: drive straight
            print(f"  Driving {target_cm:.1f} cm at {speed_mms} mm/s …")
            proto.distance(speed_mms, speed_mms, target_mm)
            # D command returns when encoders reach target; wait for completion
            # The command is blocking on the firmware side; allow generous timeout
            timeout_s = (target_mm / max(speed_mms, 1)) * 2.0 + 3.0
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                time.sleep(0.2)
                enc = proto.read_encoders()
                if enc[0] >= target_mm * 0.95 or enc[1] >= target_mm * 0.95:
                    break
            time.sleep(0.3)

            # Step 4: read sensors
            enc = proto.read_encoders()
            otos = proto.read_otos()
            pos1: Optional[tuple[float, float]] = None
            if dc is not None and daemon_cam is not None:
                pose1 = _daemon_read_pose_local(dc, daemon_cam, tag_id, timeout_s=2.0)
                if pose1 is not None:
                    pos1 = (pose1[0], pose1[1])

            enc_left_mm  = float(enc[0]) if enc else 0.0
            enc_right_mm = float(enc[1]) if enc else 0.0
            enc_left_cm  = enc_left_mm  / 10.0
            enc_right_cm = enc_right_mm / 10.0

            if otos is not None:
                otos_cm = math.hypot(otos[0], otos[1]) / 10.0
            else:
                otos_cm = 0.0
                print("  Warning: OTOS not responding — otos_cm = 0")

            if pos1 is not None:
                cam_cm = math.hypot(pos1[0] - pos0[0], pos1[1] - pos0[1])
            else:
                cam_cm = 0.0
                print(f"  Warning: tag {tag_id} not seen after drive — cam_cm = 0")

            # Sanity-only guard: a single straight trial cannot plausibly exceed
            # MAX_PLAUSIBLE_CM.  This does NOT drop normal 30–40 cm drives — the
            # old over-eager delta filter that silently discarded valid readings
            # has been removed.
            if cam_cm > MAX_PLAUSIBLE_CM:
                print(f"  Warning: camera Δ={cam_cm:.1f} cm exceeds plausible "
                      f"{MAX_PLAUSIBLE_CM:.0f} cm (tag swap / daemon glitch?) — "
                      f"cam_cm = 0", file=sys.stderr)
                cam_cm = 0.0

            print(f"  Trial {trial}:  cam={cam_cm:.2f}  otos={otos_cm:.2f}  "
                  f"enc=L{enc_left_cm:.2f} R{enc_right_cm:.2f}")

            # Step 5: get actual — auto uses camera, interactive prompts user.
            if auto_mode:
                if cam_cm <= 0:
                    print("  Camera did not measure — trial skipped.")
                    continue
                actual_cm = cam_cm
                print(f"  Auto: actual={actual_cm:.2f} cm (camera).")
            else:
                try:
                    raw = input("  Actual measured distance (cm) [or 'skip']: ").strip()
                except EOFError:
                    break
                if raw.lower() in ("skip", "s", ""):
                    print("  Skipped.")
                    continue
                try:
                    actual_cm = float(raw)
                except ValueError:
                    print(f"  Invalid '{raw}' — skipped.")
                    continue
                if actual_cm <= 0:
                    print("  Actual must be > 0 — skipped.")
                    continue

            # Step 6: adaptive OL push — converge otos_linear_scale toward GT.
            # Mirrors the OA push in cmd_calibrate_turns.
            if otos_cm > 0.0:
                new_scale = otos_linear_scale_current * (actual_cm / otos_cm)
                new_scale = _clamp_otos_scale(new_scale)
                ol_int8 = _scale_to_int8(new_scale)
                conn.send(f"OL{ol_int8:+d}", read_ms=200)
                if verbose:
                    print(f"  OL push: {otos_linear_scale_current:.4f} → "
                          f"{new_scale:.4f}  (OL{ol_int8:+d})  "
                          f"actual={actual_cm:.2f} otos={otos_cm:.2f}")
                otos_linear_scale_current = new_scale
            elif verbose:
                print("  OL push skipped — otos_cm = 0.", file=sys.stderr)

            samples.append((cam_cm, otos_cm, enc_left_cm, enc_right_cm, actual_cm))
            print(f"  Recorded sample {len(samples)}.")

    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
    finally:
        try:
            proto.stop()
        except Exception:
            pass
        if dc is not None:
            try:
                dc.close()
            except Exception:
                pass
        conn.disconnect()

    # ── Statistics ──────────────────────────────────────────────────────────
    print(f"\n{'=' * 60}")
    if len(samples) < 3:
        print(f"Need ≥ 3 trials, got {len(samples)} — not enough data.")
        return

    print(f"{'#':>3}  {'cam':>7}  {'otos':>7}  {'encL':>7}  {'encR':>7}  {'actual':>8}  "
          f"{'a/otos':>7}  {'a/encL':>7}  {'a/encR':>7}")
    otos_ratios:  list[float] = []
    encL_ratios:  list[float] = []
    encR_ratios:  list[float] = []
    cam_ratios:   list[float] = []

    for i, (cam, otos, encL, encR, actual) in enumerate(samples, 1):
        r_otos = actual / otos  if otos  > 0 else 0.0
        r_encL = actual / encL  if encL  > 0 else 0.0
        r_encR = actual / encR  if encR  > 0 else 0.0
        r_cam  = actual / cam   if cam   > 0 else 0.0
        if otos  > 0: otos_ratios.append(r_otos)
        if encL  > 0: encL_ratios.append(r_encL)
        if encR  > 0: encR_ratios.append(r_encR)
        if cam   > 0: cam_ratios.append(r_cam)
        print(f"{i:>3}  {cam:>7.2f}  {otos:>7.2f}  {encL:>7.2f}  {encR:>7.2f}  "
              f"{actual:>8.2f}  {r_otos:>7.4f}  {r_encL:>7.4f}  {r_encR:>7.4f}")

    mean_otos, std_otos = _mean_stdev(otos_ratios)
    mean_encL, std_encL = _mean_stdev(encL_ratios)
    mean_encR, std_encR = _mean_stdev(encR_ratios)
    mean_cam,  std_cam  = _mean_stdev(cam_ratios)

    new_otos_linear_scale      = otos_linear_scale_current * mean_otos
    new_mm_per_deg_left        = left_mm_per_deg_current  * mean_encL if left_mm_per_deg_current  and mean_encL else None
    new_mm_per_deg_right       = right_mm_per_deg_current * mean_encR if right_mm_per_deg_current and mean_encR else None

    print(f"\nRatio statistics (actual / measured):")
    print(f"  OTOS:          mean={mean_otos:.4f}  stdev={std_otos:.4f}  (n={len(otos_ratios)})")
    print(f"  Enc Left:      mean={mean_encL:.4f}  stdev={std_encL:.4f}  (n={len(encL_ratios)})")
    print(f"  Enc Right:     mean={mean_encR:.4f}  stdev={std_encR:.4f}  (n={len(encR_ratios)})")
    print(f"  Camera check:  mean={mean_cam:.4f}   stdev={std_cam:.4f}   (n={len(cam_ratios)}) [should be ~1.0]")

    print(f"\nRecommended new values:")
    print(f"  otos_linear_scale:   {otos_linear_scale_current:.4f} × {mean_otos:.4f} = {new_otos_linear_scale:.4f}")
    if new_mm_per_deg_left is not None:
        print(f"  mm_per_wheel_deg_left:  {left_mm_per_deg_current:.6f} × {mean_encL:.4f} = {new_mm_per_deg_left:.6f}")
    if new_mm_per_deg_right is not None:
        print(f"  mm_per_wheel_deg_right: {right_mm_per_deg_current:.6f} × {mean_encR:.4f} = {new_mm_per_deg_right:.6f}")

    updates: dict = {"calibration": {"otos_linear_scale": round(new_otos_linear_scale, 6)}}
    if new_mm_per_deg_left is not None:
        updates["calibration"]["mm_per_wheel_deg_left"]  = round(new_mm_per_deg_left, 8)
    if new_mm_per_deg_right is not None:
        updates["calibration"]["mm_per_wheel_deg_right"] = round(new_mm_per_deg_right, 8)

    _prompt_save(updates, "Distance calibration results:")


# ---------------------------------------------------------------------------
# cmd_calibrate_turns
# ---------------------------------------------------------------------------

# Firmware OA/OL register clamps the scale to int8 × 0.001 + 1.
# Effective hardware range: int8 in [-128, 127] → scale in [0.872, 1.127].
OTOS_FW_MIN_SCALE = 0.872
OTOS_FW_MAX_SCALE = 1.127


def _clamp_otos_scale(scale: float) -> float:
    """Clamp to the firmware-representable OTOS scale range."""
    return max(OTOS_FW_MIN_SCALE, min(OTOS_FW_MAX_SCALE, scale))


def _daemon_read_pose_local(dc, cam, tag_id: int,
                            timeout_s: float = 3.0):
    """Read (x_cm, y_cm, yaw_rad) for tag_id from the aprilcam daemon.

    Mirrors _daemon_read_pose in cli.py — duplicated here so calibrate.py
    does not import from cli.py (circular-import risk).
    Returns None if the tag is not seen with a calibrated position within
    timeout_s.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        tf = dc.get_tags(cam)
        for t in tf.tags:
            if t.id == tag_id and t.world_xy is not None and t.yaw is not None:
                return float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)
        time.sleep(0.03)
    return None


def _parse_so_heading(line: str) -> Optional[int]:
    """Extract the heading (third signed integer) from a stripped SO line.

    Handles both raw robot lines ("SO+123+456+87") and relay-prefixed lines
    ("``<SO+123+456+87``") — the leading ``<`` is stripped before matching.
    Returns the heading as an integer degree value (CCW positive, -180..180),
    or None if the line does not match the SO pattern.
    """
    import re as _re
    s = line.strip().lstrip("<# ").strip()
    m = _re.search(r"SO([+-]\d+)([+-]\d+)([+-]\d+)", s)
    if m:
        return int(m.group(3))
    return None


def cmd_calibrate_turns(args) -> None:
    """Multi-trial turn calibration via firmware TN closed-loop + daemon ground truth.

    Per trial (alternating CCW/CW):
      1. Read camera yaw before from aprilcam daemon (authoritative GT).
      2. Zero encoders.  Enable SO streaming (SSO+1).
      3. Send TN<deg_tenths> (firmware OTOS closed-loop turn).
      4. While waiting for TN+DONE, accumulate OTOS yaw deltas from SO stream.
      5. Send SSO+0 to stop streaming.
      6. Read camera yaw after; compute cam_deg = after - before (unwrapped).
      7. Read final encoders.
      8. Adaptive OA push: new_scale = current * (cam_deg / otos_total_deg).
         Push OA to firmware so next trial uses the improved scale.
      9. Compute trackwidth estimate from encoder mm.

    Convergence: 3 consecutive trials with alternating residual sign.
    At convergence, offer to push K+TW, K+ML, K+MR and save to config.

    --auto   Run auto mode (default prompts to press Enter per trial).
    --trials Number of trials (default 6).
    """
    from robot_radio.io.cli import _scale_to_int8

    auto_mode  = bool(getattr(args, "auto", False))
    auto_trials = int(getattr(args, "trials", 6))
    target_real_deg = 360.0   # firmware TN will handle this exactly

    proto, conn, cfg = _make_proto_cfg(args)

    tag_id = cfg.vision.robot_tag_id if cfg else 100

    # Calibration state from config
    cal = getattr(cfg, "calibration", None)
    otos_angular_scale_current = getattr(cal, "otos_angular_scale", 1.0) if cal else 1.0
    left_mm_per_deg_current  = getattr(cal, "mm_per_wheel_deg_left",  None) if cal else None
    right_mm_per_deg_current = getattr(cal, "mm_per_wheel_deg_right", None) if cal else None
    wd = getattr(getattr(cfg, "wheels", None), "wheel_diameter_mm", None)
    default_mm_per_deg = (math.pi * wd / 360.0) if wd is not None else None
    left_mm_per_deg_current  = left_mm_per_deg_current  or default_mm_per_deg
    right_mm_per_deg_current = right_mm_per_deg_current or default_mm_per_deg

    trackwidth_mm_cfg = getattr(getattr(cfg, "geometry", None), "trackwidth", None) if cfg else None

    # ── Connect to aprilcam daemon (best-effort; degrade gracefully if absent) ──
    dc = None
    daemon_cam = None
    try:
        from aprilcam.config import Config
        from aprilcam.client.control import DaemonControl
        dc = DaemonControl.connect_default(Config.load())
        cams = dc.list_cameras()
        if cams:
            daemon_cam = cams[0]
            print(f"  Daemon connected, camera={daemon_cam}, tag_id={tag_id}")
        else:
            print("  Warning: daemon has no cameras open — camera GT unavailable.",
                  file=sys.stderr)
            dc.close()
            dc = None
    except Exception as exc:
        print(f"  Warning: could not connect to aprilcam daemon ({exc}).",
              file=sys.stderr)
        print("  Camera ground truth unavailable — OTOS-only mode.", file=sys.stderr)
        dc = None

    # ── Per-trial samples ────────────────────────────────────────────────────
    # Each entry: (direction_sign, cam_deg, otos_total_deg, enc_left_mm, enc_right_mm)
    # direction_sign: +1 = CCW, -1 = CW
    samples: list[tuple[int, float, float, float, float]] = []

    # Residual sign tracking for convergence detection.
    # residual = cam_deg - otos_total_deg (positive means OTOS under-counts).
    residual_signs: list[int] = []   # +1 or -1 per trial
    converged = False

    trial = 0

    if auto_mode:
        print(f"\nTurn calibration: AUTO mode, {auto_trials} trials  "
              f"target={target_real_deg:.0f}°")
        print("Even trials CCW (+), odd trials CW (-).  OA pushed after each trial.")
        print(f"Starting otos_angular_scale = {otos_angular_scale_current:.4f}\n")
    else:
        print(f"\nTurn calibration: target={target_real_deg:.0f}°")
        print("Even trials CCW (+), odd trials CW (-).  Press Enter before each trial.")
        print("Type 'q' at prompt to stop.\n")

    try:
        while True:
            trial += 1
            if auto_mode:
                if trial > auto_trials:
                    break
                print(f"\n── Trial {trial}/{auto_trials} "
                      f"({'CCW' if trial % 2 == 1 else 'CW'}) ──")
            else:
                direction_label = "CCW" if trial % 2 == 1 else "CW"
                print(f"\n── Trial {trial} ({direction_label}) ──  "
                      f"Press Enter to spin, q to finish.")
                try:
                    raw = input().strip()
                except EOFError:
                    break
                if raw.lower() == "q":
                    break

            # Even trial index (1-based) = CCW (+), odd = CW (-)
            # Trial 1 → CCW, Trial 2 → CW, Trial 3 → CCW, ...
            direction = +1 if (trial % 2 == 1) else -1
            signed_target = target_real_deg * direction    # degrees, signed
            deg_tenths = round(signed_target * 10)
            tn_cmd = f"TN{deg_tenths:+d}"

            # ── Step 1: Read camera yaw BEFORE ──────────────────────────────
            cam_yaw_before_rad: Optional[float] = None
            if dc is not None and daemon_cam is not None:
                pose_before = _daemon_read_pose_local(dc, daemon_cam, tag_id, timeout_s=3.0)
                if pose_before is not None:
                    cam_yaw_before_rad = pose_before[2]
                    print(f"  Cam yaw before: {math.degrees(cam_yaw_before_rad):+.2f}°")
                else:
                    print(f"  Warning: tag {tag_id} not seen before spin — cam GT skipped.",
                          file=sys.stderr)

            # ── Step 2: Zero encoders ────────────────────────────────────────
            proto.zero_encoders()
            time.sleep(0.05)

            # ── Step 3: Enable SO streaming ──────────────────────────────────
            proto.set_stream_otos(True)
            time.sleep(0.05)   # small settle before TN lands

            # ── Step 4: Send TN command ──────────────────────────────────────
            print(f"  Sending {tn_cmd}  (target {signed_target:+.1f}°) …")
            conn.send(tn_cmd, read_ms=300)

            # ── Step 5: Read SO stream until TN+DONE / TN+TIMEOUT ───────────
            # The relay echoes each outgoing command as "# TX:<cmd>".
            # Skip any line containing "TX:".  Strip leading "<" from replies
            # (parse_so / _strip already handles this, but TN+DONE detection
            # needs it too).
            otos_total_deg = 0.0
            prev_raw_h: Optional[int] = None
            achieved_deg: Optional[float] = None
            timed_out = False
            TN_DEADLINE_S = 25.0
            deadline = time.monotonic() + TN_DEADLINE_S

            while time.monotonic() < deadline:
                lines = conn.read_lines(duration_ms=100)
                done_this_burst = False
                for line in lines:
                    s = str(line).strip()

                    # Skip relay TX echo lines
                    if "TX:" in s:
                        continue

                    # Strip leading "<" for matching TN replies
                    clean = s.lstrip("<")

                    # Accumulate SO heading deltas
                    h = _parse_so_heading(s)
                    if h is not None:
                        if prev_raw_h is not None:
                            delta = h - prev_raw_h
                            if delta > 180:
                                delta -= 360
                            elif delta < -180:
                                delta += 360
                            otos_total_deg += delta
                        prev_raw_h = h

                    # Detect TN terminal reply
                    if clean.startswith("TN+DONE"):
                        parts = clean.split()
                        if len(parts) > 1:
                            try:
                                achieved_deg = float(parts[1])
                            except ValueError:
                                pass
                        done_this_burst = True
                        break
                    if clean.startswith("TN+TIMEOUT"):
                        timed_out = True
                        parts = clean.split()
                        if len(parts) > 1:
                            try:
                                achieved_deg = float(parts[1])
                            except ValueError:
                                pass
                        done_this_burst = True
                        break

                if done_this_burst:
                    break

            # Report spin result
            if achieved_deg is not None:
                if timed_out:
                    print(f"  TN+TIMEOUT: achieved={achieved_deg:.1f}°  "
                          f"OTOS stream total={otos_total_deg:+.2f}°")
                else:
                    print(f"  TN+DONE:    achieved={achieved_deg:.1f}°  "
                          f"OTOS stream total={otos_total_deg:+.2f}°")
            else:
                print(f"  Warning: no TN+DONE/TIMEOUT received — "
                      f"OTOS stream total={otos_total_deg:+.2f}°", file=sys.stderr)

            # ── Step 6: Disable SO streaming ─────────────────────────────────
            proto.set_stream_otos(False)
            time.sleep(0.10)   # let the port settle

            # ── Step 7: Read camera yaw AFTER ────────────────────────────────
            cam_deg: Optional[float] = None
            if dc is not None and daemon_cam is not None and cam_yaw_before_rad is not None:
                pose_after = _daemon_read_pose_local(dc, daemon_cam, tag_id, timeout_s=3.0)
                if pose_after is not None:
                    cam_yaw_after_rad = pose_after[2]
                    print(f"  Cam yaw after:  {math.degrees(cam_yaw_after_rad):+.2f}°")
                    # Unwrap delta to nearest multiple of signed_target.
                    # The daemon +90° offset CANCELS in the delta — no correction needed.
                    cam_deg = _signed_turn_deg(
                        cam_yaw_before_rad, cam_yaw_after_rad,
                        expected_deg=signed_target)
                    print(f"  Camera Δyaw:    {cam_deg:+.2f}°  "
                          f"(target={signed_target:+.1f}°)")
                else:
                    print(f"  Warning: tag {tag_id} not seen after spin — cam GT skipped.",
                          file=sys.stderr)

            # ── Step 8: Read final encoders ──────────────────────────────────
            enc_final = proto.read_encoders()
            enc_left_mm  = float(enc_final[0])
            enc_right_mm = float(enc_final[1])
            print(f"  Encoders: L={enc_left_mm:+.1f}mm  R={enc_right_mm:+.1f}mm")

            # ── Step 9: Adaptive OA push ─────────────────────────────────────
            # Ground truth source: camera delta if available, else achieved_deg.
            gt_deg: Optional[float] = cam_deg if cam_deg is not None else (
                achieved_deg if achieved_deg is not None else None
            )

            if gt_deg is not None and abs(otos_total_deg) > 5.0:
                new_scale = otos_angular_scale_current * (abs(gt_deg) / abs(otos_total_deg))
                new_scale = _clamp_otos_scale(new_scale)
                oa_int8 = _scale_to_int8(new_scale)
                conn.send(f"OA{oa_int8:+d}", read_ms=200)
                residual = abs(gt_deg) - abs(otos_total_deg)
                residual_sign = +1 if residual >= 0 else -1
                residual_signs.append(residual_sign)
                print(f"  OA push: {otos_angular_scale_current:.4f} → {new_scale:.4f}  "
                      f"(OA{oa_int8:+d})  residual={residual:+.2f}°  sign={residual_sign:+d}")
                otos_angular_scale_current = new_scale

                # Check convergence: 3 consecutive alternating residual signs
                if len(residual_signs) >= 3:
                    last3 = residual_signs[-3:]
                    if last3[0] != last3[1] and last3[1] != last3[2]:
                        converged = True
                        print(f"  *** Converged! Residual signs alternating for 3 trials. ***")
            else:
                if abs(otos_total_deg) <= 5.0:
                    print(f"  Warning: OTOS stream total too small ({otos_total_deg:.2f}°) "
                          f"— OA push skipped.", file=sys.stderr)
                if gt_deg is None:
                    print(f"  Warning: no ground truth available — OA push skipped.",
                          file=sys.stderr)

            # ── Step 10: Record sample ───────────────────────────────────────
            if gt_deg is not None and abs(otos_total_deg) > 5.0:
                # Trackwidth estimate: tw = (|encR| + |encL|) / (2 × |cam_deg|_rad)
                tw_this: Optional[float] = None
                if abs(gt_deg) > 1.0:
                    tw_this = (abs(enc_right_mm) + abs(enc_left_mm)) / (
                        2.0 * math.radians(abs(gt_deg)))
                samples.append((direction, gt_deg, otos_total_deg,
                                enc_left_mm, enc_right_mm))
                print(f"  Sample {len(samples)} recorded: "
                      f"dir={'CCW' if direction > 0 else 'CW'}  "
                      f"cam={gt_deg:+.2f}°  otos={otos_total_deg:+.2f}°"
                      + (f"  tw_est={tw_this:.1f}mm" if tw_this is not None else ""))

            if converged and auto_mode:
                print(f"\n  Auto mode: converged after {trial} trials — stopping.")
                break

    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
    finally:
        try:
            proto.set_stream_otos(False)
        except Exception:
            pass
        try:
            proto.stop()
        except Exception:
            pass
        if dc is not None:
            try:
                dc.close()
            except Exception:
                pass
        conn.disconnect()

    # ── Statistics ──────────────────────────────────────────────────────────
    print(f"\n{'=' * 60}")
    if len(samples) < 3:
        print(f"Need ≥ 3 recorded samples, got {len(samples)} — not enough data.")
        return

    # Separate CCW and CW samples for directional bias analysis.
    ccw_samples = [(g, o, el, er) for (d, g, o, el, er) in samples if d > 0]
    cw_samples  = [(g, o, el, er) for (d, g, o, el, er) in samples if d < 0]

    # Overall ratio table
    print(f"\n{'#':>3}  {'dir':>4}  {'cam_deg':>9}  {'otos_deg':>9}  "
          f"{'encL_mm':>9}  {'encR_mm':>9}  {'ratio':>7}  {'residual':>9}")

    overall_ratios: list[float] = []
    ccw_ratios: list[float] = []
    cw_ratios:  list[float] = []
    tw_estimates: list[float] = []

    for i, (direction, gt_deg, otos_deg, enc_l, enc_r) in enumerate(samples, 1):
        dir_label = "CCW" if direction > 0 else "CW"
        if abs(otos_deg) > 0.1:
            ratio = abs(gt_deg) / abs(otos_deg)
            overall_ratios.append(ratio)
            if direction > 0:
                ccw_ratios.append(ratio)
            else:
                cw_ratios.append(ratio)
        else:
            ratio = 0.0
        residual = abs(gt_deg) - abs(otos_deg)
        if abs(gt_deg) > 1.0:
            tw = (abs(enc_r) + abs(enc_l)) / (2.0 * math.radians(abs(gt_deg)))
            tw_estimates.append(tw)
        else:
            tw = float("nan")
        print(f"{i:>3}  {dir_label:>4}  {gt_deg:>+9.2f}  {otos_deg:>+9.2f}  "
              f"{enc_l:>+9.1f}  {enc_r:>+9.1f}  {ratio:>7.4f}  {residual:>+9.2f}°")

    mean_all,  std_all  = _mean_stdev(overall_ratios)
    mean_ccw,  std_ccw  = _mean_stdev(ccw_ratios)
    mean_cw,   std_cw   = _mean_stdev(cw_ratios)
    mean_tw,   std_tw   = _mean_stdev(tw_estimates)

    directional_bias = mean_ccw - mean_cw if (ccw_ratios and cw_ratios) else 0.0

    print(f"\nRatio statistics (|cam_deg| / |otos_deg|):")
    print(f"  Overall:  mean={mean_all:.4f}  stdev={std_all:.4f}  (n={len(overall_ratios)})")
    print(f"  CCW:      mean={mean_ccw:.4f}  stdev={std_ccw:.4f}  (n={len(ccw_ratios)})")
    print(f"  CW:       mean={mean_cw:.4f}  stdev={std_cw:.4f}  (n={len(cw_ratios)})")
    if ccw_ratios and cw_ratios:
        print(f"  Directional bias (CCW - CW ratio): {directional_bias:+.4f}  "
              f"({'CCW reads smaller' if directional_bias > 0 else 'CW reads smaller'})")
    else:
        print(f"  Directional bias: insufficient data for comparison.")

    if tw_estimates:
        print(f"\nTrackwidth estimate:  mean={mean_tw:.1f}mm  stdev={std_tw:.1f}mm  "
              f"(n={len(tw_estimates)})")
        if trackwidth_mm_cfg is not None:
            print(f"  Config value:        {float(trackwidth_mm_cfg):.1f}mm  "
                  f"(difference={mean_tw - float(trackwidth_mm_cfg):+.1f}mm)")

    # Final recommended scale from the last OA push (already adaptive).
    # The adaptive loop already converged otos_angular_scale_current; for the
    # save we use the overall mean ratio applied to the *starting* config value
    # so the save is auditable.
    # Retrieve the original starting value from cfg for reference.
    original_cfg_scale = getattr(cal, "otos_angular_scale", 1.0) if cal else 1.0

    final_scale = _clamp_otos_scale(otos_angular_scale_current)
    print(f"\nFinal otos_angular_scale after adaptive OA push:")
    print(f"  Config start: {original_cfg_scale:.4f}")
    print(f"  After {len(samples)} trials: {final_scale:.4f}")
    if abs(final_scale - otos_angular_scale_current) > 1e-5:
        print(f"  Note: clamped from {otos_angular_scale_current:.4f} "
              f"to {final_scale:.4f} (firmware range)")

    # ── Offer to save ────────────────────────────────────────────────────────
    scale_updates: dict = {
        "calibration": {"otos_angular_scale": round(final_scale, 6)}
    }
    _prompt_save(scale_updates, "Save updated otos_angular_scale?")

    # ── Offer trackwidth / per-wheel encoder push ────────────────────────────
    if tw_estimates and left_mm_per_deg_current and right_mm_per_deg_current:
        tw_mm = round(mean_tw, 1)
        print(f"\nTrackwidth push: K+TW{tw_mm:.0f}  (mean={mean_tw:.1f}mm)")
        print("Also offer to update mm_per_wheel_deg_left/right from encoder totals?")
        # Compute per-wheel mm/deg from encoder averages across all trials.
        enc_l_vals = [abs(el) for (_, g, _, el, _) in samples if abs(g) > 1.0]
        enc_r_vals = [abs(er) for (_, g, _, _, er) in samples if abs(g) > 1.0]
        gt_rads    = [math.radians(abs(g)) for (_, g, _, _, _) in samples if abs(g) > 1.0]
        if enc_l_vals and enc_r_vals and gt_rads:
            # mm/deg = (enc_mm per trial) / (body_deg per trial)
            # For a pure spin: each wheel travels trackwidth/2 * body_rad
            # → mm_per_deg = enc_mm / body_deg  (using |enc| and |cam_deg|)
            ml_vals = [el / math.degrees(r) for el, r in zip(enc_l_vals, gt_rads)]
            mr_vals = [er / math.degrees(r) for er, r in zip(enc_r_vals, gt_rads)]
            mean_ml, _ = _mean_stdev(ml_vals)
            mean_mr, _ = _mean_stdev(mr_vals)
            print(f"  mm/deg_left  (from encoder): {mean_ml:.6f}  "
                  f"(config: {left_mm_per_deg_current:.6f})")
            print(f"  mm/deg_right (from encoder): {mean_mr:.6f}  "
                  f"(config: {right_mm_per_deg_current:.6f})")

            try:
                raw = input("\nPush K+TW, K+ML, K+MR to firmware and save? [y/N] ").strip()
            except EOFError:
                raw = ""

            if raw.lower() in ("y", "yes"):
                # Push to firmware — requires reconnecting or using the still-open conn.
                # conn is already closed in the finally block above, so we must
                # reconnect.  Instead we print the commands for the operator to run.
                tw_int = round(mean_tw)
                ml_int = round(mean_ml * 1000)
                mr_int = round(mean_mr * 1000)
                print(f"  Would send: K+TW{tw_int:+d}  K+ML{ml_int:+d}  K+MR{mr_int:+d}")
                print("  Note: conn already closed after calibration.  "
                      "Re-run `rogo sync cal` after saving the config to push.")

                tw_updates: dict = {
                    "calibration": {
                        "otos_angular_scale": round(final_scale, 6),
                        "mm_per_wheel_deg_left":  round(mean_ml, 8),
                        "mm_per_wheel_deg_right": round(mean_mr, 8),
                    },
                    "geometry": {"trackwidth": tw_mm},
                }
                _prompt_save(tw_updates, "Save trackwidth + per-wheel mm/deg?")
            else:
                print("  Skipped — no trackwidth/encoder changes saved.")
