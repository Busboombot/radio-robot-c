"""rogo — CLI for direct QBot Pro control via serial relay.

Commands use mm/s for speeds and mm for distances.
"""

import argparse
import json
import math
import os
import sys
import time

from robot_radio.io.serial_conn import SerialConnection, list_serial_ports, DEFAULT_PORT
from robot_radio.robot import QBotPro, Nezha, NezhaProtocol, Cutebot
from robot_radio.robot.protocol import parse_tlm
from robot_radio.sensors.color import nezha_classifier
from robot_radio.config.robot_config import get_robot_config, match_robot_by_id

_verbose = False

# ---------------------------------------------------------------------------
# Session cache — connection state between rogo invocations
# ---------------------------------------------------------------------------

# Path to the ephemeral connection cache file.  Computed from __file__ so it
# works regardless of the current working directory when rogo is invoked.
_SESSION_CACHE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "..", "data", ".rogo_session.json"
)


def _read_session_cache() -> dict | None:
    """Read the session cache from data/.rogo_session.json.

    Returns a dict with at least ``port``, ``mode``, and ``device_name`` keys
    on success.  Returns ``None`` if the file does not exist, cannot be read,
    or is malformed.  Never raises.
    """
    try:
        with open(_SESSION_CACHE_PATH) as f:
            data = json.load(f)
        if isinstance(data, dict) and "port" in data and "mode" in data:
            return data
        return None
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


def _write_session_cache(port: str, mode: str, device_name: str) -> None:
    """Write the connection cache to data/.rogo_session.json atomically.

    Only called after a successful HELLO with a confidently detected mode
    (i.e. when a DEVICE: announcement was parsed, not a fallback guess).
    Writes to a .tmp file then renames to be crash-safe.  Swallows all
    exceptions so a cache write failure never breaks a command.
    """
    tmp = _SESSION_CACHE_PATH + ".tmp"
    try:
        payload = json.dumps({"port": port, "mode": mode, "device_name": device_name})
        with open(tmp, "w") as f:
            f.write(payload)
        os.replace(tmp, _SESSION_CACHE_PATH)
    except Exception:
        pass


# Default wheel-speed clamp below which `rogo drive --mm` falls back
# to crawl mode.  Override per-invocation with `--min-speed N` or
# globally with the `ROGO_MIN_SPEED` env var.  Varies per robot (200
# was the Cutebot floor; the Nezha runs much slower reliably).
DEFAULT_MIN_SPEED_MMS = 50


def _resolve_min_speed(args) -> int:
    """Min-speed precedence: --min-speed flag > $ROGO_MIN_SPEED > default."""
    if getattr(args, "min_speed", None) is not None:
        return int(args.min_speed)
    env = os.environ.get("ROGO_MIN_SPEED")
    if env:
        try:
            return int(env)
        except ValueError:
            pass
    return DEFAULT_MIN_SPEED_MMS

# Crawl-mode (pulse-train) parameters from data/crawl_calibration.json.
# Each pulse is a short T command at CRAWL_PULSE_SPEED for CRAWL_PULSE_MS,
# followed by CRAWL_DELAY_MS_BASE of coast.  The robot moves
# CRAWL_MM_PER_PULSE millimetres per pulse (camera-measured average).
# Effective speed at the calibration delay = 6.53 mm / 100 ms ≈ 65 mm/s.
# To get a slower effective speed, lengthen the per-pulse delay.
CRAWL_PULSE_SPEED  = 300   # mm/s commanded during the pulse
CRAWL_PULSE_MS     = 80
CRAWL_DELAY_MS_MIN = 20
CRAWL_MM_PER_PULSE = 6.53
CRAWL_MAX_EFF_SPEED = (CRAWL_MM_PER_PULSE * 1000.0
                       / (CRAWL_PULSE_MS + CRAWL_DELAY_MS_MIN))  # ≈ 65 mm/s


def _scale_to_int8(scale: float) -> int:
    """Convert an OTOS scale factor to the firmware int8 encoding.

    Firmware stores OL/OA as a signed offset from 1.0 in units of 0.001
    (0.1% per step), clamped to int8 range.  So ``otos_linear_scale=1.027``
    encodes as ``round((1.027 - 1.0) / 0.001) = 27``.

    Edge case: if the config has ``otos_linear_scale == 1.0`` exactly, the
    result is 0 — which collides with the firmware factory default (also 0).
    In that case the freshness check in ``_make_robot()`` cannot distinguish
    "never calibrated" from "calibrated to exactly 1.0".  For the current
    ``nezha-1`` config this is not an issue (scale is not 1.0).  If you edit
    the config to 1.0, the fast-path fires on fresh boot; the values pushed
    would be the correct defaults anyway, so this is safe but means the warmup
    skip still works correctly — no push is needed because the firmware default
    IS the right value.
    """
    return max(-128, min(127, round((scale - 1.0) / 0.001)))


def _calibration_path() -> str:
    return os.path.join(os.path.dirname(__file__), "..", "..",
                        "data", "robot_calibration.json")


def _load_robot_calibration() -> dict:
    """Read robot_calibration.json once.  Return {} if missing/unreadable."""
    p = _calibration_path()
    try:
        with open(p) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


def _log(msg: str):
    if _verbose:
        print(f"  [{msg}]", file=sys.stderr)


def _parse_device_line(lines: list[str]) -> dict | None:
    """Find and parse a DEVICE: announcement from serial lines.
    Tolerant of garbled serial — looks for 'DEVICE:' anywhere in the line.
    """
    for line in lines:
        # Find DEVICE: even if there's garbage before it
        idx = line.find("DEVICE:")
        if idx < 0:
            continue
        parts = line[idx:].split(":")
        if len(parts) >= 5:
            return {
                "role": parts[1],
                "common_name": parts[2],
                "device_name": parts[3],
                "serial_field": ":".join(parts[4:]),
            }
    return None


def _get_port(args) -> str:
    """Resolve port from args, session cache, or auto-detect.

    Precedence:
      1. ``--port`` flag — explicit port always wins; cache is bypassed.
      2. Session cache (data/.rogo_session.json) — if the cached port is
         still present in the current port list, return it.  This skips the
         auto-detection scan and lets ``_make_robot()`` take the fast path.
      3. Auto-detect: return the first port from ``list_serial_ports()``.
    """
    if args.port:
        return args.port
    ports = list_serial_ports()
    if not ports:
        print("Error: No USB modem ports found.", file=sys.stderr)
        sys.exit(1)
    # Try the session cache before falling back to index-0 auto-detect.
    cache = _read_session_cache()
    if cache and cache.get("port") in ports:
        _log(f"using cached port: {cache['port']} (mode={cache.get('mode')})")
        return cache["port"]
    _log(f"auto-detected port: {ports[0]}")
    return ports[0]


def _make_robot(args) -> tuple[QBotPro, SerialConnection, dict]:
    """Connect and return (robot, connection, connect_result).

    Auto-detects mode: 'relay' if a RELAY/BRIDGE device, 'direct' if a ROBOT.

    Connection cache (warm-path speedup):
    Before the full HELLO handshake, checks data/.rogo_session.json for a
    cached port+mode pair.  If the cached port matches the resolved port AND
    it is still present in the current port list, the connection is opened
    directly (skip_hello=True) without the 300 ms sleep or announcement read.
    On cache miss (stale port, missing file, or malformed JSON), falls back
    to the full HELLO handshake and writes the cache on success.

    After a successful HELLO that produced a confidently detected mode (a
    DEVICE: announcement was parsed, not a fallback guess), the cache is
    written with the port, mode, and device_name.

    Calibration freshness check (warm-path speedup):
    After connecting, reads the firmware's current OL register via
    ``proto.otos_get_linear_scalar()`` (one fast round-trip, ~tens of ms) and compares it
    against the config-derived expected value.  If they match, calibration
    was already pushed this session and the push is skipped.  If they don't
    match (including ``None`` on timeout, which happens when the robot was
    just power-cycled and the firmware returns the factory default), the full
    ``_push_calibration()`` is run automatically and a warning is emitted.

    This replaces the previous unconditional ``_push_calibration()`` call,
    which added ~2.1 s to every command invocation.
    """
    port = _get_port(args)
    on_send = (lambda cmd: _log(f"TX: {cmd}")) if _verbose else None

    # Check if we can use the fast-path (cache hit).
    # --port overrides the cache: if the user explicitly specified a port, do
    # a full HELLO regardless (they may have swapped devices).
    cache = _read_session_cache()
    ports = list_serial_ports()
    use_cache = (
        not args.port  # --port flag disables cache
        and cache is not None
        and cache.get("port") == port
        and port in ports
    )

    if use_cache:
        cached_mode = cache["mode"]
        _log(f"cache hit: port={port} mode={cached_mode} — skipping HELLO")
        conn = SerialConnection(port, mode=cached_mode, on_send=on_send)
        result = conn.connect(skip_ping=True)
        if "error" in result:
            # Cache path failed — fall through to full HELLO below.
            _log(f"cache-hit connect failed ({result['error']}), falling back to full HELLO")
            use_cache = False
        else:
            # Validate the cache against the poll-time announcement (if any).
            # The readiness poll sends HELLO and may receive a DEVICE: line.
            # If it did, we can detect device/mode changes without a full HELLO.
            ann = result.get("announcement")
            if ann:
                role = ann.get("role", "").upper()
                detected_mode = "relay" if ("RELAY" in role or "BRIDGE" in role) else "direct"
                detected_device = ann.get("device_name", "")
                cached_device = cache.get("device_name", "")
                mode_mismatch = (detected_mode != cached_mode)
                device_mismatch = (detected_device and detected_device != cached_device)
                if mode_mismatch or device_mismatch:
                    # Cache is stale: the device on the port has changed.
                    print(
                        f"Warning: session cache stale "
                        f"(mode={cached_mode!r}→{detected_mode!r}, "
                        f"device={cached_device!r}→{detected_device!r}) "
                        f"— re-detected mode={detected_mode}",
                        file=sys.stderr,
                    )
                    conn._mode = detected_mode
                    _write_session_cache(port, detected_mode, detected_device)
                # else: cache is valid — fast path proceeds unchanged
            # If ann is None (device did not announce during the short poll),
            # keep the cached mode — no regression from previous behaviour.

    if not use_cache:
        conn = SerialConnection(port, on_send=on_send)
        _log(f"connecting to {port}...")
        result = conn.connect()
        if "error" in result:
            print(f"Error: {result['error']}", file=sys.stderr)
            sys.exit(1)

        ann = result.get("announcement")
        # Also try to parse from raw lines in case announcement was garbled
        if not ann:
            ann = _parse_device_line(result.get("lines", []))
            if ann:
                result["announcement"] = ann
        _log(f"HELLO response: announcement={ann}, lines={result.get('lines', [])}")

        # Retry HELLO if still no announcement.
        # Send raw HELLO (no relay prefix) so the relay itself responds.
        if not ann:
            for attempt in range(3):
                _log(f"retry HELLO ({attempt + 1}/3)...")
                time.sleep(0.3)
                conn.handshake(b"HELLO\n")
                lines = conn.read_lines(duration_ms=1200, stop_token="DEVICE:")
                _log(f"  lines: {lines}")
                ann = _parse_device_line(lines)
                if ann:
                    result["announcement"] = ann
                    break

        if not ann:
            conn.disconnect()
            print(f"Error: No device found on {port}. Is it powered on?", file=sys.stderr)
            sys.exit(1)

        # Set mode based on what we found
        role = ann.get("role", "").upper()
        if "RELAY" in role or "BRIDGE" in role:
            conn._mode = "relay"
        else:
            conn._mode = "direct"

        _log(f"connected to {ann.get('role', '?')} '{ann.get('common_name', '?')}' on {port} (mode={conn.mode})")

        # Write the session cache.  Only cache when we have a confidently
        # detected mode (a DEVICE: announcement was parsed) — do NOT cache a
        # fallback/guessed mode, or a later invocation could use a wrong mode.
        device_name = ann.get("device_name", "")
        _write_session_cache(port, conn._mode, device_name)

    cfg = get_robot_config()
    model = getattr(cfg, "hardware_model", "cutebot").lower() if cfg else "cutebot"
    if "nezha" in model:
        robot = Nezha(NezhaProtocol(conn))
    else:
        robot = Cutebot(conn)

    # Calibration values (ml, mr, tw, vel.*, OTOS scalars) are baked into the
    # firmware at compile time by scripts/gen_default_config.py — no startup
    # push needed.  Use `rogo cal` to force a re-push after editing robot JSON
    # without reflashing.

    return robot, conn, result


def _push_calibration(conn: SerialConnection) -> None:
    """Push every runtime-configurable calibration value to firmware using v2 verbs.

    Uses blocking send() (waits for ACK before next send) so the radio queue
    is drained — over the relay path, back-to-back fire-and-forget writes
    overflow the radio and drop subsequent packets.

    Sequence:
      1. Send ``ID`` to identify the robot; call ``match_robot_by_id()`` to
         load the matching per-robot config.  Falls back to ``get_robot_config()``
         if the ID command fails or no config matches.
      2. ``SET ml <float>``  ← cfg.calibration.mm_per_wheel_deg_left
      3. ``SET mr <float>``  ← cfg.calibration.mm_per_wheel_deg_right
      4. ``SET tw <int>``    ← cfg.geometry.trackwidth (integer mm)
      5. ``OI``              ← OTOS init (must precede OL/OA scalar writes)
      6. ``OL <int8>``       ← cfg.calibration.otos_linear_scale encoded as
                               round((scale-1)/0.001), clamped to -128..127
      7. ``OA <int8>``       ← cfg.calibration.otos_angular_scale (same encoding)
      8. ``SET odomOffX/odomOffY/odomYaw`` — only when geometry offsets are
         nonzero in the config (tovez offsets are all 0 → skipped).

    Removed v1 verbs: KML, KMR, OO, OK — none recognized by v2 firmware.
    Source of truth: data/robots/<robot>.json.
    """
    # ── Step 1: identify robot via v2 ID command ──────────────────────────
    id_resp = conn.send("ID", read_ms=500)
    id_line: str | None = None
    for raw in id_resp.get("responses", []):
        stripped = raw.strip()
        if stripped.startswith("ID ") or stripped == "ID":
            id_line = stripped
            break

    if id_line:
        _log(f"push calibration: robot ID response: {id_line!r}")
        cfg = match_robot_by_id(id_line)
    else:
        _log("push calibration: no ID response — falling back to get_robot_config()")
        cfg = get_robot_config()

    if cfg is None:
        import sys as _sys
        print("Warning: no robot config — calibration push skipped.", file=_sys.stderr)
        return

    # ── Step 2–4: wheel encoder calibration and trackwidth (SET keys) ────
    cal = getattr(cfg, "calibration", None)

    # Derive mm/deg from per-wheel overrides, then from wheel_diameter_mm.
    wd = getattr(getattr(cfg, "wheels", None), "wheel_diameter_mm", None)
    default_mm_per_deg = (math.pi * wd / 360.0) if wd is not None else None

    left_mm_per_deg  = getattr(cal, "mm_per_wheel_deg_left",  None) if cal else None
    right_mm_per_deg = getattr(cal, "mm_per_wheel_deg_right", None) if cal else None
    left_mm_per_deg  = left_mm_per_deg  if left_mm_per_deg  is not None else default_mm_per_deg
    right_mm_per_deg = right_mm_per_deg if right_mm_per_deg is not None else default_mm_per_deg

    if left_mm_per_deg is not None:
        _log(f"push calibration: SET ml {left_mm_per_deg:.6f}")
        conn.send(f"SET ml={left_mm_per_deg:.6f}", read_ms=200)
    if right_mm_per_deg is not None:
        _log(f"push calibration: SET mr {right_mm_per_deg:.6f}")
        conn.send(f"SET mr={right_mm_per_deg:.6f}", read_ms=200)

    geom = getattr(cfg, "geometry", None)
    tw = getattr(geom, "trackwidth", None) if geom else None
    if tw is not None:
        tw_int = int(round(float(tw)))
        _log(f"push calibration: SET tw {tw_int}")
        conn.send(f"SET tw={tw_int}", read_ms=200)

    # ── Step 5: OTOS init (must precede scalar writes) ────────────────────
    _log("push calibration: OI (OTOS init)")
    conn.send("OI", read_ms=500)

    # ── Steps 6–7: OTOS distance and heading scalars ──────────────────────
    lin_int8 = _scale_to_int8(getattr(cfg, "otos_linear_scale", 1.0) or 1.0)
    ang_int8 = _scale_to_int8(getattr(cfg, "otos_angular_scale", 1.0) or 1.0)
    _log(f"push calibration: OL {lin_int8:+d} "
         f"(linear_scale={1.0 + lin_int8 * 0.001:.4f})")
    conn.send(f"OL {lin_int8}", read_ms=200)
    _log(f"push calibration: OA {ang_int8:+d} "
         f"(angular_scale={1.0 + ang_int8 * 0.001:.4f})")
    conn.send(f"OA {ang_int8}", read_ms=200)

    # ── Step 8: OTOS mounting offset via SET keys (skip if all zero) ──────
    off = getattr(geom, "odometry_offset_mm", None) if geom else None
    if off is not None:
        ox = float(off.x)
        oy = float(off.y)
        oyaw_deg = math.degrees(float(off.yaw_rad))
        if ox != 0.0 or oy != 0.0 or oyaw_deg != 0.0:
            _log(f"push calibration: SET odomOffX={ox:.3f} odomOffY={oy:.3f} "
                 f"odomYaw={oyaw_deg:.3f}")
            conn.send(f"SET odomOffX={ox:.3f}", read_ms=200)
            conn.send(f"SET odomOffY={oy:.3f}", read_ms=200)
            conn.send(f"SET odomYaw={oyaw_deg:.3f}", read_ms=200)
        else:
            _log("push calibration: odom offsets all zero — skipping SET odomOff*")

    # ── Step 9: velocity-loop PID + cross-wheel coupling (SET keys) ───────
    # PID/tuning params live in the robot config (control section), not
    # hard-coded. Each non-None value is pushed; None keeps the firmware
    # default. Persisted in firmware RAM until power-cycle, then re-pushed by
    # the open-robot freshness check (same as the OTOS/encoder calibration).
    ctrl = getattr(cfg, "control", None)
    if ctrl is not None:
        ctrl_sets = [
            ("vel.kP",      getattr(ctrl, "vel_kp",        None)),
            ("vel.kI",      getattr(ctrl, "vel_ki",        None)),
            ("vel.kFF",     getattr(ctrl, "vel_kff",       None)),
            ("vel.iMax",    getattr(ctrl, "vel_imax",      None)),
            ("vel.kAw",     getattr(ctrl, "vel_kaw",       None)),
            ("vel.filt",    getattr(ctrl, "vel_filt",      None)),
            ("sync",        getattr(ctrl, "sync",          None)),
            ("minWheelMms", getattr(ctrl, "min_wheel_mms", None)),
        ]
        for key, val in ctrl_sets:
            if val is not None:
                _log(f"push calibration: SET {key}={val:g}")
                conn.send(f"SET {key}={val:g}", read_ms=200)

    # ── Step 10: schema-driven push of the remaining firmware-mapped values ──
    # Bucket B fix: rotation calibration (rotGain*/rotOff*/rotSlip) and the turn
    # gate live in the robot JSON + schema but were never pushed at runtime, so a
    # running robot ignored JSON edits to them until reflash. Push every schema
    # `firmware.set_key` not already sent above, straight from the loaded config.
    _already_pushed = {
        "ml", "mr", "tw", "vel.kP", "vel.kI", "vel.kFF", "vel.iMax",
        "vel.kAw", "vel.filt", "sync", "minWheelMms",
    }
    try:
        import json as _json
        from pathlib import Path as _Path
        _schema = _json.loads(
            (_Path(__file__).resolve().parents[3] / "data" / "robots"
             / "robot_config.schema.json").read_text()
        )
    except Exception as exc:
        _log(f"push calibration: schema unreadable ({exc}); skipping schema-mapped SETs")
        _schema = None
    if _schema is not None:
        for section, sec in (_schema.get("properties") or {}).items():
            sec_obj = getattr(cfg, section, None)
            if sec_obj is None:
                continue
            for prop, ps in (sec.get("properties") or {}).items():
                fw = ps.get("firmware") if isinstance(ps, dict) else None
                if not fw or "set_key" not in fw or fw["set_key"] in _already_pushed:
                    continue
                val = getattr(sec_obj, prop, None)
                if val is None:
                    continue
                literal = (str(int(round(float(val))))
                           if fw.get("kind") in ("int", "float_as_int")
                           else f"{float(val):g}")
                _log(f"push calibration: SET {fw['set_key']}={literal}")
                conn.send(f"SET {fw['set_key']}={literal}", read_ms=200)


def cmd_sync_pose(args):
    """Seed the robot's OTOS odometer with its current daemon world pose.

    Connects to the aprilcam daemon via DaemonControl gRPC (the authoritative
    A1-centred world frame, cm) and reads the robot tag's world_xy and yaw.
    Converts to mm and firmware heading, then sends SI<x><y><h> to firmware.

    Unit conversions:
      - world_xy (cm) → mm: x_mm = round(x_cm * 10), y_mm = round(y_cm * 10)
      - heading: firmware heading (drive-forward direction) = degrees(tag.yaw) + 90°
        The +90° offset is verified on hardware (test/probe_heading.py): the
        robot's forward direction in the daemon frame is (-sin θ, cos θ), so
        drive-forward in world frame = tag.yaw + 90°. The daemon's yaw is
        CCW-positive; the firmware SI command uses the same CCW-positive
        convention (verified sprint-008). Result: h_deg = round(degrees(yaw) + 90).

    Does NOT open data/homography.json or construct a local Playfield.
    The local homography was deleted on 2026-05-29 as stale (~30% scale error).
    See data/CLAUDE.md and docs/knowledge/2026-05-29-daemon-pose-frame-vs-cli-homography.md.

    Errors:
      - Daemon not reachable → non-zero exit with clear message.
      - world_xy is None (no calibrated playfield) or robot tag not seen
        within 3 s → non-zero exit with clear message.
    """
    import math as _math
    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    from robot_radio.robot.protocol import NezhaProtocol

    cfg = get_robot_config()
    tag_id = cfg.vision.robot_tag_id if cfg else 100

    try:
        dc = DaemonControl.connect_default(Config.load())
    except Exception as exc:
        sys.exit(f"Error: could not connect to aprilcam daemon: {exc}\n"
                 "Is the aprilcam daemon running from the AprilTags project directory?")

    try:
        cams = dc.list_cameras()
        if not cams:
            sys.exit("Error: aprilcam daemon reports no cameras — is a camera open?")
        cam = cams[0]

        pose = _daemon_read_pose(dc, cam, tag_id, timeout_s=3.0)
    except Exception as exc:
        try:
            dc.close()
        except Exception:
            pass
        sys.exit(f"Error: reading pose from daemon: {exc}")

    dc.close()

    if pose is None:
        sys.exit(
            f"Error: robot tag {tag_id} not seen by daemon within 3 s, or "
            "world_xy is None (playfield not calibrated). "
            "Is the robot on the field and visible to the camera? "
            "Is the aprilcam daemon running from the AprilTags project directory?"
        )

    x_cm, y_cm, yaw_rad = pose

    # Convert daemon world pose (cm, rad) to firmware SI units (mm, deg).
    # world_xy is in cm → firmware SI wants mm: multiply by 10.
    x_mm = round(x_cm * 10)
    y_mm = round(y_cm * 10)
    # Heading: firmware forward direction = tag.yaw + 90° (verified hardware,
    # see test/probe_heading.py and the knowledge file above).
    h_deg = round(_math.degrees(yaw_rad) + 90.0)

    robot, conn, _ = _make_robot(args)
    if not isinstance(robot, Nezha):
        conn.disconnect()
        sys.exit("Error: rogo sync pose requires a Nezha robot.")

    # v2: use OV command via Nezha.set_world_pose(x_mm, y_mm, h_cdeg).
    # Heading is in degrees here; OV expects centi-degrees.
    h_cdeg = round(h_deg * 100)
    robot.set_world_pose(x_mm, y_mm, h_cdeg)
    print(
        f"sync pose: daemon=({x_cm:.1f}cm, {y_cm:.1f}cm, "
        f"{_math.degrees(yaw_rad):.1f}°)  "
        f"sent OV {x_mm} {y_mm} {h_cdeg}  (v2, OV command)"
    )
    conn.disconnect()


def cmd_sync_cal(args):
    """Connect to the robot, push the full calibration set, and disconnect.

    This is a one-time setup step after power-up or after editing the robot
    config.  Pushes per-robot calibration using v2 verbs: SET ml, SET mr,
    SET tw, OI, OL, OA, and optionally SET odomOffX/odomOffY/odomYaw.
    """
    port = _get_port(args)
    on_send = (lambda cmd: _log(f"TX: {cmd}")) if _verbose else None
    conn = SerialConnection(port, on_send=on_send)

    _log(f"connecting to {port}...")
    result = conn.connect()
    if "error" in result:
        print(f"Error: {result['error']}", file=sys.stderr)
        sys.exit(1)

    ann = result.get("announcement")
    if not ann:
        ann = _parse_device_line(result.get("lines", []))
    if ann:
        role = ann.get("role", "").upper()
        if "RELAY" in role or "BRIDGE" in role:
            conn._mode = "relay"
        else:
            conn._mode = "direct"
    _log(f"connected (mode={conn.mode}); pushing calibration...")

    # Gather the values we are about to push so we can print a summary.
    cfg = get_robot_config()
    if cfg is None:
        print("Error: no robot config found — cannot push calibration.", file=sys.stderr)
        conn.disconnect()
        sys.exit(1)

    _push_calibration(conn)

    # Print a human-readable summary of what was pushed (v2 verbs).
    # _push_calibration() resolved the config via ID+match_robot_by_id;
    # fall back to get_robot_config() for the summary display.
    cfg_display = get_robot_config()
    if cfg_display is None:
        print("sync cal: calibration pushed (no config for display summary)")
        conn.disconnect()
        return

    wd = getattr(getattr(cfg_display, "wheels", None), "wheel_diameter_mm", None)
    default_mm_per_deg = (math.pi * wd / 360.0) if wd is not None else None
    cal = getattr(cfg_display, "calibration", None)
    left_mm_per_deg  = getattr(cal, "mm_per_wheel_deg_left",  None) if cal else None
    right_mm_per_deg = getattr(cal, "mm_per_wheel_deg_right", None) if cal else None
    left_mm_per_deg  = left_mm_per_deg  if left_mm_per_deg  is not None else default_mm_per_deg
    right_mm_per_deg = right_mm_per_deg if right_mm_per_deg is not None else default_mm_per_deg

    lin_int8 = _scale_to_int8(getattr(cfg_display, "otos_linear_scale", 1.0) or 1.0)
    ang_int8 = _scale_to_int8(getattr(cfg_display, "otos_angular_scale", 1.0) or 1.0)
    geom = getattr(cfg_display, "geometry", None)
    tw = getattr(geom, "trackwidth", None) if geom else None
    off = getattr(geom, "odometry_offset_mm", None) if geom else None

    print("sync cal: calibration pushed successfully (v2 verbs)")
    if left_mm_per_deg is not None:
        print(f"  SET ml={left_mm_per_deg:.6f}  (mm/deg={left_mm_per_deg:.4f})")
    if right_mm_per_deg is not None:
        print(f"  SET mr={right_mm_per_deg:.6f}  (mm/deg={right_mm_per_deg:.4f})")
    if tw is not None:
        print(f"  SET tw={int(round(float(tw)))}  (trackwidth mm)")
    print("  OI  (OTOS init)")
    print(f"  OL  {lin_int8:+d}  "
          f"(linear_scale={1.0 + lin_int8 * 0.001:.4f},"
          f" source={'config' if getattr(cfg_display, 'otos_linear_scale', None) is not None else 'default'})")
    print(f"  OA  {ang_int8:+d}  "
          f"(angular_scale={1.0 + ang_int8 * 0.001:.4f},"
          f" source={'config' if getattr(cfg_display, 'otos_angular_scale', None) is not None else 'default'})")
    if off is not None:
        ox = float(off.x)
        oy = float(off.y)
        oyaw_deg = math.degrees(float(off.yaw_rad))
        if ox != 0.0 or oy != 0.0 or oyaw_deg != 0.0:
            print(f"  SET odomOffX={ox:.3f} odomOffY={oy:.3f} odomYaw={oyaw_deg:.3f}")
        else:
            print("  (odom offsets all zero — SET odomOff* skipped)")

    conn.disconnect()


# ── Rotation model (mirror of test/rotation_calibrate/RotationModel) ──────


def _turn_command(angle_deg: float, speed_mms: int,
                  cal: dict) -> tuple[int, int, int]:
    """Compute (cmd_left, cmd_right, duration_ms) for an in-place rotation
    of `angle_deg` degrees (positive = CCW / left).

    Resolution order for wheelbase / slip / motor model:
      1. data/robot_calibration.json (full firmware-calibrated rotation
         model, including a bivariate-polynomial motor model).
      2. data/robots/<active>.json via get_robot_config() — uses
         geometry.trackwidth as wheelbase and falls back to a no-slip
         linear model (t_ms = 1000 · target_arc / speed_mms).  Accuracy
         depends on motors; expect ~10-20% error on first try until a
         proper rotation calibration is run.
    """
    rot = cal.get("rotational", {})
    if "wheelbase_mm" not in rot or "rotational_slip" not in rot:
        # Fallback: use trackwidth + rotational_slip from data/robots/<active>.json
        cfg = get_robot_config()
        tw = getattr(getattr(cfg, "geometry", None), "trackwidth", None) if cfg else None
        if tw is None:
            raise SystemExit(
                "Error: data/robot_calibration.json missing rotational fields "
                "and data/robots/<active>.json has no geometry.trackwidth — "
                "can't compute turn."
            )
        # Optional rotational_slip in the active robot's calibration section.
        # 1.0 = no slip (over-shoots if slip is real). For Nezha with grippy
        # tires on the playfield, observed slip ≈ 0.75 (270° actual for 360°
        # commanded). Set in nezha-1.json via:
        #   "calibration": { "rotational_slip": 0.75, ... }
        cfg_cal = getattr(cfg, "calibration", None)
        cfg_slip = getattr(cfg_cal, "rotational_slip", None) if cfg_cal else None
        slip_val = float(cfg_slip) if cfg_slip is not None else 1.0
        rot = {"wheelbase_mm": float(tw), "rotational_slip": slip_val}
        _log(f"turn: trackwidth={tw}mm slip={slip_val} from active robot config")
    W = float(rot["wheelbase_mm"])
    slip = float(rot["rotational_slip"])

    # Optional linear correction: compensated_deg = (target_deg - offset) / gain.
    # When configured (in nezha-1.json calibration section), this captures
    # angle-dependent error that a single slip factor can't — e.g. fixed
    # startup loss makes small turns under-rotate more proportionally than
    # large turns.
    cfg_for_corr = get_robot_config()
    cal_corr = getattr(cfg_for_corr, "calibration", None) if cfg_for_corr else None
    if angle_deg >= 0:
        gain   = getattr(cal_corr, "rotation_gain",       None) if cal_corr else None
        offset = getattr(cal_corr, "rotation_offset_deg", None) if cal_corr else None
    else:
        gain   = getattr(cal_corr, "rotation_gain_neg",       None) if cal_corr else None
        offset = getattr(cal_corr, "rotation_offset_deg_neg", None) if cal_corr else None
        if gain is None:
            gain = getattr(cal_corr, "rotation_gain", None) if cal_corr else None
        if offset is None:
            offset = getattr(cal_corr, "rotation_offset_deg", None) if cal_corr else None
    gain   = float(gain)   if gain   is not None else 1.0
    offset = float(offset) if offset is not None else 0.0
    compensated_deg = (angle_deg - offset) / gain
    if gain != 1.0 or offset != 0.0:
        _log(f"turn: linear correction (target {angle_deg:+.1f}° → command {compensated_deg:+.2f}°) "
             f"with gain={gain:.4f}, offset={offset:.2f}°")

    theta = math.radians(compensated_deg)
    sign = 1 if theta >= 0 else -1
    target_arc = abs(theta) * W / (2.0 * slip)   # mm of arc per wheel

    mm = rot.get("motor_model")
    if mm and mm.get("type") == "bivariate_polynomial":
        # Polynomial model: arc_obs(v, t) = Σ c_{p,q}·v^p·t^q.
        # At fixed v, this is a polynomial in t; root-find for t such that
        # arc_obs(v, t) = target_arc.
        try:
            import numpy as np
        except ImportError:
            raise SystemExit("numpy is required for the polynomial turn model")
        terms = [(int(t["p"]), int(t["q"]), float(t["coef"]))
                 for t in mm["terms"]]
        # Coefficients of the polynomial in t at this v, indexed by power.
        by_q: dict[int, float] = {}
        for (p, q, c) in terms:
            by_q[q] = by_q.get(q, 0.0) + c * (float(speed_mms) ** p)
        max_q = max(by_q) if by_q else 0
        coeffs = [0.0] * (max_q + 1)
        for q, c in by_q.items():
            coeffs[q] = c
        # numpy.roots wants highest power first.
        coeffs_np = list(reversed(coeffs))
        coeffs_np[-1] -= target_arc
        roots = np.roots(coeffs_np)
        positive = [float(r.real) for r in roots
                    if abs(r.imag) < 1e-6 and r.real > 0.0]
        if not positive:
            raise SystemExit(
                f"Error: no positive real duration for {angle_deg:+.1f}° "
                f"at v={speed_mms} mm/s (target outside calibrated range?)"
            )
        t_ms = int(round(min(positive)))
    elif "arc_efficiency" in rot and "startup_loss_mm" in rot:
        # Legacy linear inverse:  t_ms = 1000·(target_arc + β) / (α·v)
        alpha = float(rot["arc_efficiency"])
        beta = float(rot["startup_loss_mm"])
        t_ms = int(round(1000.0 * (target_arc + beta) / (alpha * speed_mms)))
    else:
        # No motor model — use plain kinematic estimate.  At commanded
        # wheel speed v (mm/s) each wheel travels v*t mm of arc.
        # Pick t such that arc = target_arc:  t_ms = 1000 · target_arc / v.
        t_ms = int(round(1000.0 * target_arc / float(speed_mms)))

    # World-frame CCW = drive(-L, +R) on this Nezha. aprilcam.Tag.orientation
    # reports image-space (Y-down) yaw which is CW-positive in world frame, so
    # drive(+L, -R) produces image-positive but world-CW motion. To match the
    # user-facing convention (positive degrees = world CCW), we flip the sign.
    return (-sign * speed_mms, sign * speed_mms, t_ms)


# ── Crawl-mode pulse-train drive ─────────────────────────────────────────────


def _crawl_drive_distance(robot: QBotPro, speed_mms: int,
                          target_mm: int) -> tuple[int, int]:
    """Pulse-train slow drive when |speed| is below the firmware MIN clamp.

    Each pulse is a short T command at CRAWL_PULSE_SPEED for CRAWL_PULSE_MS,
    followed by `delay_ms` of coast.  We choose `delay_ms` so the average
    speed across pulse+delay matches `speed_mms`, clamped at the calibration
    floor (delay ≥ CRAWL_DELAY_MS_MIN).

    Stops after the number of pulses needed to cover `target_mm` at the
    calibrated mm-per-pulse.  Returns the firmware's final encoder reading.
    """
    eff_v = abs(speed_mms)
    if eff_v < 1:
        raise SystemExit("Error: crawl speed must be > 0")
    target_mm = abs(target_mm)
    if target_mm < 1:
        raise SystemExit("Error: crawl distance must be > 0")

    # Cycle period to hit the requested effective speed.
    cycle_ms = max(CRAWL_PULSE_MS + CRAWL_DELAY_MS_MIN,
                   int(round(CRAWL_MM_PER_PULSE * 1000.0 / eff_v)))
    delay_ms = cycle_ms - CRAWL_PULSE_MS
    pulses = max(1, int(round(target_mm / CRAWL_MM_PER_PULSE)))
    sign = 1 if speed_mms >= 0 else -1
    pulse_v = sign * CRAWL_PULSE_SPEED

    # Cap the actual effective speed reported back if we hit the floor.
    eff_actual = CRAWL_MM_PER_PULSE * 1000.0 / cycle_ms

    _log(f"crawl mode: {pulses} pulses × T+{pulse_v}+{pulse_v}+{CRAWL_PULSE_MS} "
         f"(delay {delay_ms} ms, eff ≈ {eff_actual:.1f} mm/s, "
         f"target {target_mm} mm ≈ {pulses * CRAWL_MM_PER_PULSE:.1f} mm)")

    enc_l, enc_r = 0, 0
    for i in range(pulses):
        enc_l, enc_r = robot.speed_for_time(pulse_v, pulse_v, CRAWL_PULSE_MS)
        if delay_ms > 0:
            time.sleep(delay_ms / 1000.0)
    return enc_l, enc_r


# ── Commands ──────────────────────────────────────────────────────────


def cmd_ports(args):
    """List available serial ports."""
    ports = list_serial_ports()
    if not ports:
        print("No USB modem ports found.")
    else:
        for p in ports:
            print(p)


def cmd_hello(args):
    """Probe device — send HELLO, print announcement."""
    port = _get_port(args)
    conn = SerialConnection(port, mode="relay")
    _log(f"connecting to {port}...")
    info = conn.connect()
    if "error" in info:
        print(f"Error: {info['error']}", file=sys.stderr)
        sys.exit(1)
    print(f"Port:    {info.get('port', '?')}")
    print(f"Mode:    {info.get('mode', '?')}")
    if info.get("announcement"):
        ann = info["announcement"]
        print(f"Role:    {ann.get('role', '?')}")
        print(f"Name:    {ann.get('common_name', '?')}")
        print(f"Device:  {ann.get('device_name', '?')}")
        print(f"Serial:  {ann.get('serial_field', '?')}")
    else:
        print("No device announcement received.")
        if info.get("lines"):
            for line in info["lines"]:
                print(f"  {line}")
    conn.disconnect()


def _maybe_zero(robot, args):
    if getattr(args, 'ez', False):
        robot.zero_encoders()


def _print_enc_dist(initial: tuple[int, int] | None,
                    final: tuple[int, int]) -> None:
    """Print final ENC, plus DIST (mm) if we can derive it from the active config."""
    print(f"ENC {final[0]} {final[1]}")
    if initial is None:
        return
    cfg = get_robot_config()
    mm_per_tick = getattr(cfg, "mm_per_tick", None) if cfg else None
    if mm_per_tick is None:
        return
    dl = (final[0] - initial[0]) * mm_per_tick
    dr = (final[1] - initial[1]) * mm_per_tick
    print(f"DIST {dl:.1f} {dr:.1f}")


def cmd_drive(args):
    """Drive at speed. --ms for time, --mm for distance, 'stream' for keepalive streaming.

    When |left| or |right| is below the configured crawl threshold,
    drive falls back to crawl mode (pulse-train).  Threshold comes from
    --min-speed, then $ROGO_MIN_SPEED, else DEFAULT_MIN_SPEED_MMS.
    Crawl requires --mm and symmetric wheel speeds (left == right).

    Stream mode (rogo drive <L> <R> stream [--resend MS]) sends S keepalives
    at the specified cadence and streams encoder readings until Ctrl-C.
    """
    # Validate and resolve the optional 'stream' keyword positional.
    stream_kw = getattr(args, "stream_kw", None)
    if stream_kw is not None and stream_kw != "stream":
        print(
            f"Error: unexpected positional argument '{stream_kw}'. "
            "Did you mean 'stream'?",
            file=sys.stderr,
        )
        sys.exit(1)
    args.stream_mode = (stream_kw == "stream")

    # Validate --resend.
    # Note: do not use `or 150` here — that would silently convert 0 to 150.
    resend_ms = getattr(args, "resend", None)
    if resend_ms is None:
        resend_ms = 150
    if args.stream_mode and resend_ms <= 0:
        print(f"Error: --resend must be > 0, got {resend_ms}", file=sys.stderr)
        sys.exit(1)

    if args.stream_mode and (args.ms is not None or args.mm is not None):
        print(
            "Error: 'stream' is mutually exclusive with --ms and --mm.",
            file=sys.stderr,
        )
        sys.exit(1)

    robot, conn, _ = _make_robot(args)
    _maybe_zero(robot, args)
    initial = robot.read_encoders()

    min_speed = _resolve_min_speed(args)
    asymmetric = (args.left != args.right)
    below_min  = (abs(args.left) < min_speed) or (abs(args.right) < min_speed)
    nonzero    = (args.left != 0) and (args.right != 0)

    if below_min and nonzero:
        # Crawl-mode dispatch.
        if asymmetric:
            print(
                f"Error: crawl mode requires equal wheel speeds; got "
                f"left={args.left}, right={args.right}.  "
                f"For asymmetric drive, both wheels must be ≥ {min_speed} mm/s.",
                file=sys.stderr,
            )
            conn.disconnect()
            sys.exit(1)
        if args.mm is None:
            print(
                f"Error: crawl mode (|speed| < {min_speed}) requires --mm "
                f"to specify the distance.",
                file=sys.stderr,
            )
            conn.disconnect()
            sys.exit(1)
        left_enc, right_enc = _crawl_drive_distance(robot, args.left, args.mm)
        _print_enc_dist(initial, (left_enc, right_enc))
        conn.disconnect()
        return

    if args.mm is not None:
        left_enc, right_enc = robot.speed_for_distance(args.left, args.right, args.mm)
        _print_enc_dist(initial, (left_enc, right_enc))
    elif args.ms is not None:
        left_enc, right_enc = robot.speed_for_time(args.left, args.right, args.ms)
        _print_enc_dist(initial, (left_enc, right_enc))
    elif args.stream_mode:
        # Stream mode: rogo drive <L> <R> stream [--resend MS] [--secs N]
        # --resend controls the S keepalive cadence sent to the firmware.
        # stream_drive(watchdog_ms=...) uses keepalive_s = watchdog_ms * 0.30 / 1000.
        # To achieve a resend cadence of resend_ms, set watchdog_ms = resend_ms / 0.30.
        # Example: resend_ms=150, watchdog_ms=500 → keepalive = 500*0.30 = 150 ms.
        # --secs N: auto-stop after N seconds (None = run until Ctrl-C).
        watchdog_ms = int(resend_ms / 0.30)  # keepalive = watchdog_ms * 0.30
        _log(f"stream mode: resend={resend_ms}ms → watchdog_ms={watchdog_ms}ms")
        secs = getattr(args, "secs", None)
        deadline = (time.monotonic() + secs) if secs is not None else None
        left_enc, right_enc = initial
        speeds = [args.left, args.right]
        try:
            for resp in robot.stream_drive(speeds, period_ms=40, watchdog_ms=watchdog_ms):
                if deadline is not None and time.monotonic() >= deadline:
                    break
                tlm = parse_tlm(resp.raw) if resp.tag == "TLM" else None
                if tlm and tlm.enc:
                    left_enc, right_enc = tlm.enc
                    vl = tlm.vel[0] if tlm.vel else 0
                    vr = tlm.vel[1] if tlm.vel else 0
                    print(f"ENC {left_enc} {right_enc}  VEL {vl} {vr}")
        except KeyboardInterrupt:
            print("\nCtrl-C caught, stopping...", file=sys.stderr)
        _log("sending STOP")
        robot.stop()
        _log("waiting for motors to stop")
        lines = conn.read_lines(duration_ms=500)
        for line in lines:
            _log(f"RX: {line}")
        _log("disconnecting")
        _print_enc_dist(initial, (left_enc, right_enc))
    else:
        left_enc, right_enc = initial
        try:
            for left_enc, right_enc in robot.speed(args.left, args.right):
                print(f"ENC {left_enc} {right_enc}")
        except KeyboardInterrupt:
            print("\nCtrl-C caught, stopping...", file=sys.stderr)
        _log("sending STOP")
        robot.stop()
        _log("waiting for motors to stop")
        # Give firmware time to process STOP and confirm
        lines = conn.read_lines(duration_ms=500)
        for line in lines:
            _log(f"RX: {line}")
        _log("disconnecting")
        _print_enc_dist(initial, (left_enc, right_enc))
    conn.disconnect()


def cmd_drive_stream(args):
    """Drive at speed (non-blocking), stream encoder positions until Ctrl-C."""
    robot, conn, _ = _make_robot(args)
    _maybe_zero(robot, args)
    try:
        for left_enc, right_enc in robot.speed(args.left, args.right):
            print(f"ENC {left_enc} {right_enc}")
    except KeyboardInterrupt:
        pass
    robot.stop()
    conn.disconnect()


def cmd_stop(args):
    """Stop motors (v2 STOP command)."""
    robot, conn, _ = _make_robot(args)
    robot.stop()
    print("STOP")
    conn.disconnect()


def cmd_turn(args):
    """Turn in place by N degrees (positive = CCW/left, negative = CW/right).

    Sends the firmware RT command — a RELATIVE spin computed on the robot from
    the encoder arc (arc = |deg|·π/180·trackwidth/2), stopped on the encoder
    differential. Pure dead reckoning: no OTOS, no heading odometry, no host
    loop. A firmware time bound guarantees it can't run away. rogo just sends
    the angle and waits for EVT done RT.

    The legacy open-loop T-command path is still available with --open-loop.
    """
    if getattr(args, "open_loop", False):
        cal = _load_robot_calibration()
        cmd_l, cmd_r, t_ms = _turn_command(args.degrees, args.speed, cal)
        _log(f"turn {args.degrees:+.1f}° → T{cmd_l:+d}{cmd_r:+d}{t_ms:+d} (open-loop)")
        robot, conn, _ = _make_robot(args)
        _maybe_zero(robot, args)
        left_enc, right_enc = robot.speed_for_time(cmd_l, cmd_r, t_ms)
        print(f"ENC {left_enc} {right_enc}")
        conn.disconnect()
        return

    # ── Default: firmware RT (relative encoder-arc turn, computed on-robot) ───
    from robot_radio.robot.protocol import NezhaProtocol

    robot, conn, _ = _make_robot(args)
    proto = getattr(robot, "_proto", None)
    if not isinstance(proto, NezhaProtocol):
        conn.disconnect()
        sys.exit("Error: rogo turn requires a Nezha robot with NezhaProtocol.")

    rel_cdeg = int(round(args.degrees * 100))
    corr = "1"
    _log(f"turn {args.degrees:+.1f}° → RT {rel_cdeg} (encoder-arc, on-robot)")
    proto.send(f"RT {rel_cdeg} #{corr}", 400)
    outcome = proto.wait_for_evt_done("RT", timeout_ms=20000, corr_id=corr)
    conn.disconnect()

    if outcome == "timeout":
        print("WARNING: no EVT done RT received within 20 s "
              "(is the firmware new enough to have RT?)")
    elif outcome == "safety_stop":
        print("WARNING: RT ended in safety_stop")
    else:
        print(f"done: turned {args.degrees:+.0f}° (RT, encoder dead-reckoning)")


def _spin_to_world_yaw(proto, field, tag_id, target_deg, speed, tol_deg):
    """Closed-loop streaming spin to an absolute world yaw (deg).

    Reads the robot tag yaw from `field` (the camera is ground truth, so this
    is slip-immune), computes the shortest signed delta to `target_deg`, then
    drives a streaming-S spin with velocity-projected stop. Caller owns the
    open camera/field and serial connection; this only spins.

    Returns the final signed yaw error in degrees, or None if the camera
    never saw the robot tag.
    """
    import math as _math
    from robot_radio.io.calibrate import _get_tag_yaw

    # Control parameters (mirror calibrate.py)
    TICK_S = 0.03
    WATCHDOG_MS = 500
    DELTA_MAX_DEG = 30.0
    MAX_SECS = 15.0
    COAST_S = 0.10   # empirical coast time after proto.stop() (tuned 2026-05-28)

    # Read current yaw → compute shortest signed delta to target.
    cur_yaw_rad = _get_tag_yaw(field, tag_id, timeout_s=3.0)
    if cur_yaw_rad is None:
        return None
    cur_deg = _math.degrees(cur_yaw_rad)
    raw_diff = target_deg - cur_deg
    diff = ((raw_diff + 180.0) % 360.0) - 180.0
    print(f"current={cur_deg:+.1f}°  target={target_deg:+.1f}°  "
          f"need={diff:+.1f}° ({'CCW' if diff > 0 else 'CW'})")

    # v2: no set_watchdog verb; use SET sTimeout=<ms> to configure firmware watchdog.
    proto.set_config(sTimeout=WATCHDOG_MS)
    prev_cam = cur_yaw_rad
    prev_time = time.monotonic()
    total_cam_deg = 0.0
    ang_vel = 0.0
    t_start = prev_time

    while True:
        cur_cam = _get_tag_yaw(field, tag_id, timeout_s=0.10)
        now = time.monotonic()
        if cur_cam is not None:
            raw = cur_cam - prev_cam
            d = ((raw + _math.pi) % (2.0 * _math.pi)) - _math.pi
            d_deg = _math.degrees(d)
            dt = now - prev_time
            if abs(d_deg) > DELTA_MAX_DEG:
                prev_cam = cur_cam
                prev_time = now
            elif dt > 0:
                ang_vel = 0.6 * ang_vel + 0.4 * (d_deg / dt)
                total_cam_deg += d_deg
                prev_cam = cur_cam
                prev_time = now

        remaining = diff - total_cam_deg
        projected = total_cam_deg + ang_vel * COAST_S
        projected_err = diff - projected

        if abs(projected_err) <= tol_deg and abs(ang_vel) > 5.0:
            proto.stop()
            time.sleep(max(COAST_S * 1.5, 0.4))
            break
        if time.monotonic() - t_start > MAX_SECS:
            proto.stop()
            print(f"WARNING: hit {MAX_SECS}s timeout; not at target")
            break

        direction = 1 if remaining > 0 else -1
        proto.drive(-direction * speed, direction * speed)
        time.sleep(TICK_S)

    final_cam = _get_tag_yaw(field, tag_id, timeout_s=2.0)
    if final_cam is None:
        return None
    final_deg = _math.degrees(final_cam)
    return ((target_deg - final_deg + 180.0) % 360.0) - 180.0


def cmd_turnto(args):
    """Closed-loop turn to an absolute world yaw using the aprilcam daemon.

    Reads the robot's yaw in real-time from the aprilcam daemon (authoritative
    A1-centred world frame), computes the shortest signed delta to the target,
    then drives a streaming-S spin with velocity-projected stop. Slip-immune.

    Yaw convention: daemon tag.yaw is CCW-positive radians (same as the local
    Playfield tag.orientation). No conversion needed — both sources share the
    same world-frame convention; only the firmware SI heading uses +90° offset.

        rogo turnto 0     # face world-CCW zero (which is +Y per aprilcam)
        rogo turnto 90    # face +X
        rogo turnto -90   # face -X
    """
    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    from robot_radio.robot.protocol import NezhaProtocol

    target_deg = float(args.degrees)
    speed = int(args.speed)
    tol_deg = float(args.tol)
    cfg = get_robot_config()
    tag_id = cfg.vision.robot_tag_id if cfg else 100

    try:
        dc = DaemonControl.connect_default(Config.load())
    except Exception as exc:
        sys.exit(f"Error: could not connect to aprilcam daemon: {exc}\n"
                 "Is the aprilcam daemon running from the AprilTags project directory?")

    cams = dc.list_cameras()
    if not cams:
        dc.close()
        sys.exit("Error: aprilcam daemon reports no cameras — is a camera open?")
    cam = cams[0]

    robot, conn, _ = _make_robot(args)
    proto = getattr(robot, "_proto", None)
    if not isinstance(proto, NezhaProtocol):
        conn.disconnect()
        try:
            dc.close()
        except Exception:
            pass
        sys.exit("Error: rogo turnto requires a Nezha robot with NezhaProtocol.")

    _log(f"turnto: target={target_deg:+.1f}°  speed={speed}mm/s  cam={cam}")

    def read_pose(timeout_s=1.0):
        return _daemon_read_pose(dc, cam, tag_id, timeout_s=timeout_s)

    try:
        err = _daemon_spin_to_yaw(proto, read_pose, target_deg, speed, tol_deg)
        if err is None:
            sys.exit(f"Error: daemon could not see robot tag {tag_id}.")
        print(f"final error={err:+.1f}°  (target={target_deg:+.1f}°)")
    finally:
        try:
            proto.stop()
        except Exception:
            pass
        try:
            dc.close()
        except Exception:
            pass
        conn.disconnect()


def _daemon_read_pose(dc, cam, tag_id, timeout_s=2.0):
    """Read (x_cm, y_cm, yaw_rad) for `tag_id` from the aprilcam daemon.

    Uses the daemon's gRPC API (DaemonControl), which reports calibrated
    world_xy in the A1-centred frame (origin at tag 1, x-right, y-up) — the
    authoritative frame, not the CLI's own stale homography. Returns None if
    the tag is not seen with a calibrated position within `timeout_s`.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        tf = dc.get_tags(cam)
        for t in tf.tags:
            if t.id == tag_id and t.world_xy is not None and t.yaw is not None:
                return float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)
        time.sleep(0.03)
    return None


def _daemon_spin_to_yaw(proto, read_pose, target_deg, speed, tol_deg,
                        max_secs=8.0):
    """Velocity-projected closed-loop spin to an absolute world yaw (deg).

    Same convergent control law as `turnto` (`_spin_to_world_yaw`), but reads
    yaw from the aprilcam daemon via `read_pose()` (a callable returning
    (x, y, yaw_rad) or None). Returns the final signed yaw error in degrees,
    or None if the daemon never reported the robot.
    """
    import math as _math
    COAST_S = 0.10
    p = read_pose(3.0)
    if p is None:
        return None
    cur_deg = _math.degrees(p[2])
    diff = ((target_deg - cur_deg + 180.0) % 360.0) - 180.0
    # v2: no set_watchdog verb; use SET sTimeout=<ms> to configure firmware watchdog.
    proto.set_config(sTimeout=500)
    prev_cam = p[2]
    prev_t = time.monotonic()
    total = 0.0
    ang_vel = 0.0
    t0 = prev_t
    while True:
        p = read_pose(0.2)
        now = time.monotonic()
        if p is not None:
            d = ((p[2] - prev_cam + _math.pi) % (2.0 * _math.pi)) - _math.pi
            d_deg = _math.degrees(d)
            dt = now - prev_t
            if abs(d_deg) <= 30.0 and dt > 0:
                ang_vel = 0.6 * ang_vel + 0.4 * (d_deg / dt)
                total += d_deg
            prev_cam = p[2]
            prev_t = now
        remaining = diff - total
        projected_err = diff - (total + ang_vel * COAST_S)
        if abs(projected_err) <= tol_deg and abs(ang_vel) > 5.0:
            proto.stop()
            time.sleep(max(COAST_S * 1.5, 0.4))
            break
        if now - t0 > max_secs:
            proto.stop()
            break
        direction = 1 if remaining > 0 else -1
        proto.drive(-direction * speed, direction * speed)
        time.sleep(0.03)
    p = read_pose(1.5)
    if p is None:
        return None
    return ((target_deg - _math.degrees(p[2]) + 180.0) % 360.0) - 180.0


def cmd_goto(args):
    """Turn to face an absolute world point, then drive there (closed-loop).

    The position analog of `turnto`. Reads the robot tag's world pose from the
    aprilcam daemon (authoritative A1-centred frame, origin at tag 1, cm), then
    runs a closed-loop pure-pursuit controller: it turns in place toward the
    target when badly mis-aimed, otherwise drives forward with mild steering
    correction, continuously re-reading the camera until it is within the
    arrival tolerance. Closed-loop on camera feedback — robust to the Nezha's
    stale open-loop distance/turn calibration.

        rogo goto 30 10            # drive to world (30, 10) cm
        rogo goto 30 10 --speed 120 --arrive 4

    Heading convention (verified on hardware 2026-05-28, see test/probe_heading.py):
        the robot's forward direction in the daemon frame is (-sinθ, cosθ),
        i.e. world motion_dir = tag_yaw + 90°. So to head toward a point at
        bearing φ = atan2(dy, dx), the required tag yaw is φ - 90°.
    """
    import math as _math
    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    from robot_radio.robot.protocol import NezhaProtocol

    target_x = float(args.x)
    target_y = float(args.y)
    cruise = int(args.speed)
    turn_speed = int(args.turn_speed)
    gate_deg = float(args.tol)          # turn-in-place if heading error exceeds this
    arrive_cm = float(args.arrive)
    max_secs = float(args.timeout)
    cfg = get_robot_config()
    tag_id = cfg.vision.robot_tag_id if cfg else 100

    robot, conn, _ = _make_robot(args)
    proto = getattr(robot, "_proto", None)
    if not isinstance(proto, NezhaProtocol):
        conn.disconnect()
        sys.exit("Error: rogo goto requires a Nezha robot with NezhaProtocol.")

    _log(f"goto: target=({target_x:+.1f}, {target_y:+.1f})cm  cruise={cruise}mm/s")
    dc = DaemonControl.connect_default(Config.load())
    cams = dc.list_cameras()
    if not cams:
        dc.close()
        conn.disconnect()
        sys.exit("Error: aprilcam daemon reports no cameras.")
    cam = cams[0]

    def _wrap(a):
        return (a + _math.pi) % (2.0 * _math.pi) - _math.pi

    def read_pose(timeout_s=1.0):
        return _daemon_read_pose(dc, cam, tag_id, timeout_s=timeout_s)

    # Control parameters.
    TICK_S = 0.05
    WATCHDOG_MS = 800
    AIM_GATE_DEG = gate_deg       # turn in place (convergent spin) above this
    REAIM_GATE_DEG = gate_deg * 1.8  # abort a forward burst if drift exceeds this
    SPIN_TOL_DEG = 4.0
    STEER_KP = 1.0                # gentle in-burst steering (frac per rad)
    SLOW_RADIUS_CM = 18.0         # ramp cruise down within this of target
    MIN_DRIVE = 70               # mm/s floor (above motor deadband)
    BURST_MAX_S = 1.2

    try:
        p = read_pose(3.0)
        if p is None:
            sys.exit(f"Error: daemon could not see robot tag {tag_id} "
                     "(calibrated). Is the playfield calibrated and the robot "
                     "in view?")
        rx, ry, yaw = p
        d0 = _math.hypot(target_x - rx, target_y - ry)
        print(f"start: robot=({rx:.1f}, {ry:.1f}) yaw={_math.degrees(yaw):+.0f}°  "
              f"target=({target_x:.1f}, {target_y:.1f})  dist={d0:.1f}cm")
        if d0 <= arrive_cm:
            print(f"Already within {arrive_cm:.1f}cm (dist={d0:.1f}cm); done.")
            return

        # v2: no set_watchdog verb; use SET sTimeout=<ms> to configure firmware watchdog.
        proto.set_config(sTimeout=WATCHDOG_MS)
        t_start = time.monotonic()

        while True:
            if time.monotonic() - t_start > max_secs:
                proto.stop()
                print(f"WARNING: hit {max_secs:.0f}s timeout; not at target")
                break

            p = read_pose(1.0)
            if p is None:
                proto.stop()
                continue
            rx, ry, yaw = p
            dx, dy = target_x - rx, target_y - ry
            dist = _math.hypot(dx, dy)
            if dist <= arrive_cm:
                proto.stop()
                break

            motion_dir = _math.atan2(dy, dx)
            req_yaw = _wrap(motion_dir - _math.pi / 2)   # forward = (-sinθ, cosθ)
            head_err = _wrap(req_yaw - yaw)

            # Aim with the proven velocity-projected spin if badly off heading.
            if abs(head_err) > _math.radians(AIM_GATE_DEG):
                _log(f"aim: head_err={_math.degrees(head_err):+.0f}° → "
                     f"spin to {_math.degrees(req_yaw):+.0f}°")
                _daemon_spin_to_yaw(proto, read_pose, _math.degrees(req_yaw),
                                    turn_speed, SPIN_TOL_DEG)
                continue

            # Forward burst toward the target, monitored — break out to re-aim
            # on drift, on arrival, or when distance stops decreasing.
            _log(f"drive: dist={dist:.1f}cm head_err={_math.degrees(head_err):+.0f}°")
            b_start = time.monotonic()
            best = dist
            while time.monotonic() - b_start < BURST_MAX_S:
                q = read_pose(0.25)
                if q is None:
                    break
                rx, ry, yaw = q
                dx, dy = target_x - rx, target_y - ry
                dist = _math.hypot(dx, dy)
                if dist <= arrive_cm:
                    break
                he = _wrap(_wrap(_math.atan2(dy, dx) - _math.pi / 2) - yaw)
                if abs(he) > _math.radians(REAIM_GATE_DEG):
                    break
                if dist > best + 2.0:   # overshot / moving away → re-aim
                    break
                best = min(best, dist)
                v = MIN_DRIVE + (cruise - MIN_DRIVE) * min(1.0, dist / SLOW_RADIUS_CM)
                steer = max(-0.5, min(0.5, STEER_KP * he))
                proto.drive(int(round(v * (1.0 - steer))),
                            int(round(v * (1.0 + steer))))
                time.sleep(TICK_S)
            proto.stop()
            time.sleep(0.15)

        # Final report.
        time.sleep(0.3)
        end = read_pose(2.0)
        if end is not None:
            ex, ey, eyaw = end
            err = _math.hypot(target_x - ex, target_y - ey)
            elapsed = time.monotonic() - t_start
            print(f"final=({ex:.1f}, {ey:.1f})cm yaw={_math.degrees(eyaw):+.0f}°  "
                  f"error={err:.1f}cm  ({elapsed:.1f}s)")
        else:
            print("done (lost robot tag for final readout)")
    finally:
        try:
            proto.stop()
        except Exception:
            pass
        try:
            dc.close()
        except Exception:
            pass
        conn.disconnect()


def cmd_go(args):
    """Drive to relative (X, Y) target via pure-pursuit arc (G command).

    Robot is at (0, 0) heading 0 in its own frame.  If |bearing(X, Y)| > 45°
    the firmware pre-rotates in place, then drives straight by sqrt(X²+Y²).
    Otherwise it follows the unique arc tangent to the heading.

    Prints "ENC <left> <right> <outcome>" where outcome is DONE,
    TIMEOUT (firmware deadline), or HOST_TIMEOUT (CLI wait expired).
    """
    robot, conn, _ = _make_robot(args)
    if args.ez:
        # Verify the zero actually took (relay path can lose the EZ packet).
        for attempt in range(3):
            robot.zero_encoders()
            time.sleep(0.15)
            enc = robot.read_encoders()
            if enc == (0, 0):
                break
            _log(f"EZ attempt {attempt+1}: encoders still at {enc}, retrying")
        else:
            print(f"Error: failed to zero encoders after 3 attempts (last={enc})",
                  file=sys.stderr)
            conn.disconnect()
            sys.exit(1)
    _log(f"go ({args.x:+d}, {args.y:+d}) mm at {args.speed} mm/s")
    left, right, outcome = robot.go_to(args.x, args.y, args.speed,
                                       timeout_s=args.timeout)
    print(f"ENC {left} {right} {outcome}")
    conn.disconnect()


def _wheel_arg(s: str):
    """Argparse type for a per-wheel angle.

    Accepts a float, or ``x`` / ``-`` / ``.`` to mean "skip this wheel".
    Returns None for skip, else float.
    """
    if s.lower() in ("x", "-", "."):
        return None
    return float(s)


def cmd_rotate(args):
    """Rotate each wheel by a relative angle.

    NOTE: This command is not supported in v2 firmware.  The v1 PR (relative
    rotate) servo verb has been removed.  Use 'rogo drive --ms' or 'rogo turn'
    for equivalent motion.

    The command accepts its arguments for backward compatibility with scripts
    but exits immediately with a clear message rather than crashing.
    """
    # v2 decision: the PR servo verb was Cutebot-only and is not implemented in
    # the v2 Nezha firmware.  The Nezha driver has no rotate() method.
    # Route through rogo turn <deg> for closed-loop rotation, or
    # rogo drive --ms for open-loop timed spin.
    print(
        "Error: 'rogo rotate' is not supported on v2 firmware (no PR verb).\n"
        "Use 'rogo turn <deg>' for closed-loop rotation, or\n"
        "'rogo drive -<speed> <speed> --ms <ms>' for open-loop timed spin.",
        file=sys.stderr,
    )
    sys.exit(1)


def cmd_angle(args):
    """Drive each wheel to an absolute angle.

    NOTE: This command is not supported in v2 firmware.  The v1 PA (absolute
    angle) servo verb has been removed.  Use 'rogo turn <deg>' for equivalent
    rotation, or 'rogo drive --ms' for timed motion.

    The command accepts its arguments for backward compatibility with scripts
    but exits immediately with a clear message rather than crashing.
    """
    # v2 decision: the PA servo verb was Cutebot-only and is not implemented in
    # the v2 Nezha firmware.  The Nezha driver has no angle() method.
    # Route through rogo turn <deg> for closed-loop rotation.
    print(
        "Error: 'rogo angle' is not supported on v2 firmware (no PA verb).\n"
        "Use 'rogo turn <deg>' for closed-loop rotation, or\n"
        "'rogo drive -<speed> <speed> --ms <ms>' for open-loop timed spin.",
        file=sys.stderr,
    )
    sys.exit(1)


def cmd_port(args):
    """Digital port I/O on J1..J4.

        rogo port 4 1       # set J4 HIGH (e.g. laser ON)
        rogo port 4 0       # set J4 LOW  (laser OFF)
        rogo port 4         # read J4 input (returns 0 or 1)

    Pin mapping: J1→P8, J2→P12, J3→P14, J4→P16.
    """
    if args.jack not in (1, 2, 3, 4):
        print(f"Error: jack must be 1..4, got {args.jack}", file=sys.stderr)
        sys.exit(1)
    robot, conn, _ = _make_robot(args)
    if args.value is None:
        wire = f"P {args.jack}"
    else:
        if args.value not in (0, 1):
            print(f"Error: digital value must be 0 or 1, got {args.value}", file=sys.stderr)
            conn.disconnect()
            sys.exit(1)
        wire = f"P {args.jack} {args.value}"
    result = robot.send(wire, read_ms=400)
    for line in result.get("responses", []):
        s = str(line).lstrip("<# ")
        if s.startswith(("OK port", "ACK:P ", "P ", "ERR")):
            print(s)
            break
    conn.disconnect()


def cmd_pwm(args):
    """Analog (PWM) port I/O on J1..J4.

        rogo pwm 4 512      # write PWM 512 (~50% duty) to J4
        rogo pwm 4 0        # write PWM 0 (off)
        rogo pwm 4          # read analog input from J4 (0..1023)

    Pin mapping: J1→P1, J2→P2, J3→P13, J4→P15.
    ADC read is reliable only on J1 (P1) and J2 (P2).
    """
    if args.jack not in (1, 2, 3, 4):
        print(f"Error: jack must be 1..4, got {args.jack}", file=sys.stderr)
        sys.exit(1)
    robot, conn, _ = _make_robot(args)
    if args.value is None:
        wire = f"PA {args.jack}"
    else:
        if not (0 <= args.value <= 1023):
            print(f"Error: PWM value must be 0..1023, got {args.value}", file=sys.stderr)
            conn.disconnect()
            sys.exit(1)
        wire = f"PA {args.jack} {args.value}"
    result = robot.send(wire, read_ms=400)
    for line in result.get("responses", []):
        s = str(line).lstrip("<# ")
        if s.startswith(("OK port", "OK analog", "ACK:PA ", "PA ", "ERR")):
            print(s)
            break
    conn.disconnect()


def cmd_grip(args):
    """Control gripper."""
    value = args.value
    if value == "open":
        angle = 0
    elif value == "close":
        angle = 180
    else:
        try:
            angle = int(value)
        except ValueError:
            print(
                f"Error: Invalid grip value '{value}'. "
                "Use 'open', 'close', or an angle.",
                file=sys.stderr,
            )
            sys.exit(1)
    robot, conn, _ = _make_robot(args)
    robot.grip(angle)
    print(f"G {angle}")
    conn.disconnect()


def cmd_enc(args):
    """Read encoder positions (mm) via v2 SNAP → TLM."""
    robot, conn, _ = _make_robot(args)
    # v2: read fresh encoder values via SNAP → TLM (not cached state).
    # The Nezha driver's read_encoders() returns cached state; SNAP requests
    # a live TLM frame from the firmware with current enc= values.
    frame = _snap_tlm(conn)
    if frame is not None and frame.enc is not None:
        left_enc, right_enc = frame.enc
    else:
        # Fallback: cached state from any prior streaming (may be stale on fresh connect).
        left_enc, right_enc = robot.read_encoders()
    print(f"ENC {left_enc} {right_enc}")
    conn.disconnect()


def cmd_opos(args):
    """Read robot OTOS fused pose via v2 SNAP → TLM.

    Outputs: POSE <x_mm> <y_mm> <h_deg> where h_deg is the heading in degrees
    (converted from the firmware's centi-degrees).

    v2 firmware does not support the v1 SO (Sensor Output) verb.  Fused pose
    is read via SNAP → TLM pose= field (x_mm, y_mm, h_cdeg).
    """
    robot, conn, _ = _make_robot(args)
    # v2: use SNAP → TLM to read OTOS fused pose; v1 SO verb is not supported.
    frame = _snap_tlm(conn)
    if frame is None or frame.pose is None:
        print("Error: no pose data in TLM frame (SNAP returned no pose= field)",
              file=sys.stderr)
        conn.disconnect()
        sys.exit(1)
    x_mm, y_mm, h_cdeg = frame.pose
    h_deg = h_cdeg / 100.0
    print(f"POSE {x_mm} {y_mm} {h_deg:.1f}")
    conn.disconnect()


def cmd_ez(args):
    """Zero encoders."""
    robot, conn, _ = _make_robot(args)
    robot.zero_encoders()
    print("EZ")
    conn.disconnect()


def cmd_send(args):
    """Send arbitrary command."""
    robot, conn, _ = _make_robot(args)
    result = robot.send(args.message, read_ms=args.read_ms)
    if "error" in result:
        print(f"Error: {result['error']}", file=sys.stderr)
    else:
        print(f"Sent: {result.get('sent', '')}")
        for line in result.get("responses", []):
            print(f"  {line}")
    conn.disconnect()


def _find_response(responses: list[str], prefix: str) -> str | None:
    """Return the first response line starting with `prefix`, or None."""
    for line in responses:
        if line.startswith(prefix):
            return line
    return None


def _snap_tlm(conn: SerialConnection):
    """Send SNAP and return the parsed TLMFrame, or None if not received.

    v2 firmware responds to SNAP with an immediate TLM frame.  This helper
    sends SNAP and reads the TLM frame from the response window.
    """
    from robot_radio.robot.protocol import parse_tlm as _parse_tlm
    # SNAP makes the firmware emit one TLM frame on its NEXT tick; over the lossy
    # radio relay that frame can take >0.5 s to arrive (the "OK snap" ack comes
    # first, the TLM after) and is sometimes dropped entirely. Read a generous
    # window and RETRY a few times until a frame with data arrives.
    for _attempt in range(4):
        result = conn.send("SNAP", read_ms=700)
        for raw in result.get("responses", []):
            frame = _parse_tlm(raw)
            if frame is None and "TLM" in raw:
                # The RAW250 relay can concatenate replies WITHOUT newline
                # separators ("OK snapTLM t=..."); re-parse from "TLM".
                frame = _parse_tlm(raw[raw.index("TLM"):])
            if frame is not None:
                return frame
    return None


def cmd_line(args):
    """Read the 4-channel line sensor via v2 SNAP → TLM (grayscale 0..255 each).

    v2 firmware does not support the v1 LS verb.  SNAP requests an immediate
    TLM frame; the line= field contains the four grayscale channels.
    """
    robot, conn, _ = _make_robot(args)
    # v2: use SNAP → TLM to read line sensor; v1 LS verb is not supported.
    frame = _snap_tlm(conn)
    if frame is None or frame.line is None:
        print("Error: no line sensor data in TLM frame (SNAP returned no line= field)",
              file=sys.stderr)
        conn.disconnect()
        sys.exit(1)
    g1, g2, g3, g4 = frame.line
    print(f"LS {g1} {g2} {g3} {g4}")
    conn.disconnect()


def cmd_color(args):
    """Read the color sensor via v2 SNAP → TLM.

    Default output: HSL (hue 0..360, sat 0..100, light 0..100), computed
    from the *white-balanced* RGB so it matches what the classifier sees.
    Add ``--name`` for a single label, ``--rgb`` / ``--hsv`` for those
    formats, or ``--raw`` to dump raw RGBC counts.  Pass
    ``--calibrate-white`` to install a fresh white reference (otherwise
    the Nezha factory default is used).

    v2 firmware does not support the v1 CS verb.  Color data is read via
    SNAP → TLM color= field (r,g,b,c).
    """
    import colorsys

    robot, conn, _ = _make_robot(args)
    clf = nezha_classifier()

    if args.calibrate_white:
        from robot_radio.sensors.color import calibrate_white as _do_calibrate
        def _read():
            # v2: use SNAP → TLM to read color sensor; v1 CS verb not supported.
            frame = _snap_tlm(conn)
            if frame is None or frame.color is None:
                return None
            return frame.color
        try:
            wr = _do_calibrate(clf, _read)
            print(f"# white ref: R={wr[0]} G={wr[1]} B={wr[2]} C={wr[3]}",
                  file=sys.stderr)
        except RuntimeError as e:
            print(f"Error: {e}", file=sys.stderr)
            conn.disconnect()
            sys.exit(1)

    # v2: use SNAP → TLM to read color sensor; v1 CS verb is not supported.
    frame = _snap_tlm(conn)
    if frame is None or frame.color is None:
        print("Error: no color sensor data in TLM frame (SNAP returned no color= field)",
              file=sys.stderr)
        conn.disconnect()
        sys.exit(1)
    conn.disconnect()

    r_raw, g_raw, b_raw, c_raw = frame.color

    if args.raw:
        print(f"CS {r_raw} {g_raw} {b_raw} {c_raw}")
        return
    if args.name:
        name, _h, _s, _l = clf.classify(r_raw, g_raw, b_raw, c_raw)
        print(name)
        return

    # HSL / HSV / RGB output — all computed on the white-balanced RGB.
    r_b = r_raw / max(clf.white_r, 1)
    g_b = g_raw / max(clf.white_g, 1)
    b_b = b_raw / max(clf.white_b, 1)
    m = max(r_b, g_b, b_b, 1e-6)
    rr_f, gg_f, bb_f = min(1.0, r_b / m), min(1.0, g_b / m), min(1.0, b_b / m)
    if args.rgb:
        print(f"RGB {round(rr_f * 255)} {round(gg_f * 255)} {round(bb_f * 255)}")
        return
    h, l, s_hls = colorsys.rgb_to_hls(rr_f, gg_f, bb_f)
    h_deg = h * 360.0
    if args.hsv:
        _, s_hsv, v = colorsys.rgb_to_hsv(rr_f, gg_f, bb_f)
        print(f"HSV {round(h_deg)} {round(s_hsv * 100)} {round(v * 100)}")
        return
    print(f"HSL {round(h_deg)} {round(s_hls * 100)} {round(l * 100)}")


def cmd_pose(args):
    """Report x, y, and angle for a tag from the camera."""
    import os
    from aprilcam import Camera, Playfield

    want_world = not args.pixels
    calibration_path = os.path.join(
        os.path.dirname(__file__), "..", "..", "data", "calibration.json")

    cam = Camera(args.camera)
    field = Playfield(
        cam, width_cm=101.0, height_cm=89.0,
        calibration=calibration_path if os.path.exists(calibration_path) else None,
    )
    try:
        if want_world and not field.is_calibrated:
            print(
                "Error: no homography loaded for camera "
                f"{args.camera}; cannot report world coordinates. "
                "Pass -p/--pixels to get pixel coordinates instead.",
                file=sys.stderr,
            )
            sys.exit(2)

        field.start()

        # field.tag() polls the ring buffer non-blockingly; it returns
        # the Tag only when visible in the latest frame.
        deadline = time.monotonic() + args.timeout
        tag = None
        while time.monotonic() < deadline:
            t = field.tag(args.tag)
            if t is not None:
                tag = t
                break
            time.sleep(0.05)

        if tag is None:
            print(
                f"Error: tag {args.tag} not seen within "
                f"{args.timeout:.1f}s on camera {args.camera}.",
                file=sys.stderr,
            )
            sys.exit(3)

        angle_rad = tag.orientation
        angle_deg = math.degrees(angle_rad)
        print(f"tag {args.tag}")
        if want_world:
            if tag.wx is None:
                print(
                    f"Error: tag {args.tag} is visible but has no world "
                    "coordinates (outside the calibrated playfield?).",
                    file=sys.stderr,
                )
                sys.exit(4)
            print(f"  x: {tag.wx:.2f} cm")
            print(f"  y: {tag.wy:.2f} cm")
        else:
            print(f"  x: {tag.cx:.1f} px")
            print(f"  y: {tag.cy:.1f} px")
        print(f"  angle: {angle_rad:.4f} rad ({angle_deg:.1f}°)")
    finally:
        try:
            field.stop()
        finally:
            cam.close()


def main():
    global _verbose

    parser = argparse.ArgumentParser(
        prog="rogo",
        description="Direct serial control for QBot Pro via relay. "
                    "Speeds in mm/s, distances in mm.",
    )
    parser.add_argument(
        "--port", default=None,
        help=f"Serial port (auto-detect if omitted, default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="Print connection and serial debug info",
    )

    sub = parser.add_subparsers(dest="command", help="Available commands")

    sub.add_parser("ports", help="List available serial ports")
    sub.add_parser("hello", help="Probe device (send HELLO, print announcement)")

    # drive: blocking speed control (--ms for time, --mm for distance)
    p_drive = sub.add_parser(
        "drive",
        help="Drive: rogo drive <L> <R> [--ms N | --mm N | stream]. "
             f"|speed| below the crawl threshold (default "
             f"{DEFAULT_MIN_SPEED_MMS} mm/s, override with --min-speed "
             "or $ROGO_MIN_SPEED) with --mm uses crawl mode (pulse-train, "
             "requires equal wheel speeds).",
    )
    p_drive.add_argument("left", type=int, help="Left speed (mm/s)")
    p_drive.add_argument("right", type=int, help="Right speed (mm/s)")
    p_drive.add_argument(
        "stream_kw", nargs="?", default=None, metavar="stream",
        help="Optional literal 'stream' to enable keepalive streaming mode "
             "(sends S command at the --resend cadence; Ctrl-C to stop).",
    )
    p_drive.add_argument("--ms", type=int, default=None, help="Duration in ms (blocking)")
    p_drive.add_argument("--mm", type=int, default=None, help="Distance in mm (blocking)")
    p_drive.add_argument(
        "--resend", type=int, default=150,
        help="Keepalive S resend interval in ms for stream mode (default 150; "
             "must be > 0). Lower values reduce motor throbbing risk; "
             "30%% of the firmware sTimeout (500 ms) = 150 ms is the recommended default.",
    )
    p_drive.add_argument("--ez", action="store_true", help="Zero encoders before driving")
    p_drive.add_argument(
        "--min-speed", type=int, default=None,
        help=f"Crawl-mode threshold in mm/s (default: {DEFAULT_MIN_SPEED_MMS}, "
             "or $ROGO_MIN_SPEED if set).",
    )
    p_drive.add_argument(
        "--secs", type=float, default=None,
        help="Stream mode only: auto-stop after N seconds (STOP + exit). "
             "Without this flag, stream runs until Ctrl-C.",
    )

    # turn: in-place rotation by N degrees using the calibrated rotation model
    p_turn = sub.add_parser(
        "turn",
        help="Turn in place by N degrees (positive = CCW/left). "
             "Uses data/robot_calibration.json.",
    )
    p_turn.add_argument("degrees", type=float, help="Angle in degrees, signed (CCW = +)")
    p_turn.add_argument(
        "--speed", type=int, default=200,
        help="Wheel speed magnitude during the turn (mm/s, default: 200; "
             "slip calibration is most consistent at this speed)",
    )
    p_turn.add_argument("--ez", action="store_true", help="Zero encoders before turning")
    p_turn.add_argument("--camera", type=int, default=None,
                        help="Camera index (overrides $CAMERA, default auto-discover OV9782)")
    p_turn.add_argument("--tol", type=float, default=3.0,
                        help="Convergence tolerance for closed-loop (default 3°)")
    p_turn.add_argument("--open-loop", action="store_true",
                        help="Use legacy timed-T command (no camera). "
                             "Less accurate due to motor slip variability.")

    p_turnto = sub.add_parser(
        "turnto",
        help="Closed-loop turn to an ABSOLUTE world yaw using camera feedback. "
             "Camera is the ground truth — slip-immune.",
    )
    p_turnto.add_argument("degrees", type=float,
                          help="Target absolute world yaw in degrees")
    p_turnto.add_argument("--speed", type=int, default=80,
                          help="Wheel speed during spin (mm/s, default 80; "
                               "stay below ~120 to keep camera tracking)")
    p_turnto.add_argument("--camera", type=int, default=None,
                          help="Camera index (overrides $CAMERA, default auto-discover)")
    p_turnto.add_argument("--tol", type=float, default=3.0,
                          help="Convergence tolerance in degrees (default 3°)")

    p_go = sub.add_parser(
        "go",
        help="Go to relative (X, Y) mm via pure-pursuit arc. "
             "Pre-rotates if |bearing| > 45°, else direct arc.",
    )
    p_go.add_argument("x", type=int, help="X target (mm forward of current heading)")
    p_go.add_argument("y", type=int, help="Y target (mm left of current heading)")
    p_go.add_argument(
        "--speed", type=int, default=200,
        help="Wheel speed magnitude (mm/s, default 200)",
    )
    p_go.add_argument(
        "--timeout", type=float, default=15.0,
        help="Max wait for G+DONE/G+TIMEOUT (seconds, default 15)",
    )
    p_go.add_argument("--ez", action="store_true", help="Zero encoders before going")

    p_goto = sub.add_parser(
        "goto",
        help="Turn to face an ABSOLUTE world (X, Y) point (daemon A1-centred "
             "frame, cm), then drive there. Closed-loop on camera feedback; "
             "position analog of turnto.",
    )
    p_goto.add_argument("x", type=float, help="Target world X (cm, daemon frame)")
    p_goto.add_argument("y", type=float, help="Target world Y (cm, daemon frame)")
    p_goto.add_argument("--speed", type=int, default=150,
                        help="Cruise drive speed (mm/s, default 150; ramps down "
                             "near the target)")
    p_goto.add_argument("--turn-speed", type=int, default=80, dest="turn_speed",
                        help="Wheel speed while turning in place (mm/s, default 80)")
    p_goto.add_argument("--tol", type=float, default=12.0,
                        help="Heading gate in degrees: turn in place when the "
                             "bearing error exceeds this, else drive with "
                             "steering correction (default 12°)")
    p_goto.add_argument("--arrive", type=float, default=4.0,
                        help="Stop when within this distance of the target "
                             "(cm, default 4)")
    p_goto.add_argument("--timeout", type=float, default=30.0,
                        help="Max seconds for the whole move (default 30)")
    p_goto.add_argument("--camera", type=int, default=None,
                        help="(unused; daemon selects the camera)")

    p_rot = sub.add_parser(
        "rotate",
        help="Rotate each wheel by N degrees (PR, relative). "
             "Two args (L, R), 'x' to skip a wheel.  Returns immediately.",
    )
    p_rot.add_argument("left", type=_wheel_arg,
                       help="Left wheel rotation degrees (signed), or 'x' to skip")
    p_rot.add_argument("right", type=_wheel_arg,
                       help="Right wheel rotation degrees (signed), or 'x' to skip")
    p_rot.add_argument("--speed", type=int, default=30,
                       help="Servo speed percent 1..100 (default 30)")
    p_rot.add_argument("--ez", action="store_true", help="Zero encoders before rotating")

    p_ang = sub.add_parser(
        "angle",
        help="Drive each wheel to absolute angle 0..360° (PA). "
             "Two args (L, R), 'x' to skip a wheel.  Returns immediately.",
    )
    p_ang.add_argument("left", type=_wheel_arg,
                       help="Left wheel target angle (0..360), or 'x' to skip")
    p_ang.add_argument("right", type=_wheel_arg,
                       help="Right wheel target angle (0..360), or 'x' to skip")
    p_ang.add_argument("--cw", action="store_true",
                       help="Force clockwise rotation")
    p_ang.add_argument("--ccw", action="store_true",
                       help="Force counter-clockwise rotation")
    p_ang.add_argument("--speed", type=int, default=30,
                       help="Servo speed percent 1..100 (default 30)")
    p_ang.add_argument("--ez", action="store_true", help="Zero encoders before moving")

    p_port = sub.add_parser(
        "port",
        help="Digital port I/O on J1..J4 (laser, LED, switch). "
             "Omit value to read; 0/1 to write.",
    )
    p_port.add_argument("jack", type=int, help="Port number 1..4 (J1..J4)")
    p_port.add_argument("value", type=int, nargs="?", default=None,
                        help="0 = LOW, 1 = HIGH.  Omit to read.")

    p_pwm = sub.add_parser(
        "pwm",
        help="Analog (PWM) port I/O on J1..J4. "
             "Omit value to read; 0..1023 to write.",
    )
    p_pwm.add_argument("jack", type=int, help="Port number 1..4 (J1..J4)")
    p_pwm.add_argument("value", type=int, nargs="?", default=None,
                       help="PWM 0..1023.  Omit to read.")

    sub.add_parser("stop", help="Stop motors")

    p_grip = sub.add_parser("grip", help="Control gripper: rogo grip open|close|<angle>")
    p_grip.add_argument("value", help="'open', 'close', or servo angle (0-180)")

    sub.add_parser("enc", help="Read encoder positions (mm) via v2 SNAP → TLM")
    sub.add_parser("opos", help="Read robot OTOS fused pose (x_mm, y_mm, h_deg) via v2 SNAP → TLM")
    sub.add_parser("ez", help="Zero encoders")
    sub.add_parser("line", help="Read 4-channel line sensor (grayscale 0..255) via v2 SNAP → TLM")
    p_color = sub.add_parser(
        "color",
        help="Read color sensor. Default: HSL. --rgb, --hsv, or --name for other forms.",
    )
    p_color_fmt = p_color.add_mutually_exclusive_group()
    p_color_fmt.add_argument("--rgb", action="store_true", help="Output display RGB (0..255)")
    p_color_fmt.add_argument("--hsv", action="store_true", help="Output HSV (hue 0..360, S/V 0..100)")
    p_color_fmt.add_argument("--name", action="store_true", help="Output a single colour label")
    p_color_fmt.add_argument("--raw", action="store_true", help="Output raw 'CS r g b c' (RGBC counts from TLM frame)")
    p_color.add_argument("--calibrate-white", action="store_true",
                         help="Sample over white first to install a fresh "
                              "white reference for this call.")

    p_send = sub.add_parser("send", help="Send arbitrary command")
    p_send.add_argument("message", help="Command string to send")
    p_send.add_argument("--read-ms", type=int, default=500, help="Response read timeout (ms)")

    p_pose = sub.add_parser(
        "pose",
        help="Report x, y, angle for a tag (world cm by default, --pixels for px)",
    )
    p_pose.add_argument("tag", type=int, help="AprilTag ID to read")
    p_pose.add_argument(
        "-p", "--pixels", action="store_true",
        help="Report pixel coordinates instead of world (cm)",
    )
    p_pose.add_argument(
        "--camera", type=int, default=3,
        help="Camera index (default: 3, the Arducam B&W)",
    )
    p_pose.add_argument(
        "--timeout", type=float, default=2.0,
        help="Seconds to wait for the tag to be seen (default: 2.0)",
    )

    # sync: one-time setup commands (calibration push, pose seed, …)
    p_sync = sub.add_parser(
        "sync",
        help="One-time setup commands (run after power-up or config edit).",
    )
    sync_sub = p_sync.add_subparsers(dest="sync_cmd", required=True)

    sync_sub.add_parser(
        "cal",
        help="Push full calibration to firmware using v2 verbs "
             "(SET ml/mr/tw, OI, OL, OA, SET odomOff*). "
             "Run once after power-up or after editing the robot config. "
             "Robot must be stationary during the ~700 ms IMU bias window.",
    )

    sync_sub.add_parser(
        "pose",
        help="Seed robot OTOS odometry from daemon world pose (run once at start "
             "of session or after repositioning). Requires the aprilcam daemon "
             "running from the AprilTags project directory with a calibrated "
             "playfield. Reads tag.world_xy (cm, A1-centred) and tag.yaw (rad) "
             "from the daemon, converts to mm/centi-degrees, and sends OV "
             "to firmware (v2 OV command). The firmware heading = degrees(tag.yaw) + 90° "
             "(the robot's drive-forward direction in world frame).",
    )

    # calibrate: interactive multi-trial calibration subcommands
    p_cal = sub.add_parser("calibrate", help="Interactive multi-trial calibration")
    cal_sub = p_cal.add_subparsers(dest="cal_command", required=True)

    p_dist = cal_sub.add_parser(
        "distance",
        help="Calibrate OTOS linear + per-wheel encoder mm/deg via straight drive",
    )
    p_dist.add_argument(
        "--distance", type=float, default=40.0,
        help="Target distance in cm (default 40)",
    )
    p_dist.add_argument(
        "--speed", type=int, default=200,
        help="Drive speed in mm/s (default 200)",
    )
    p_dist.add_argument(
        "--camera", type=int, default=None,
        help="Camera index (overrides $CAMERA, default auto-discover OV9782)",
    )
    p_dist.add_argument(
        "--auto", action="store_true",
        help="Auto mode: use camera Δ as ground truth (no tape measure prompt).",
    )
    p_dist.add_argument(
        "--trials", type=int, default=3,
        help="Number of auto trials (default 3, only used with --auto).",
    )

    p_turn_cal = cal_sub.add_parser(
        "turns",
        help="Calibrate OTOS angular scale + per-wheel mm/deg via 360° spin",
    )
    p_turn_cal.add_argument(
        "--speed", type=int, default=200,
        help="Wheel speed in mm/s (default 200)",
    )
    p_turn_cal.add_argument(
        "--camera", type=int, default=None,
        help="Camera index (overrides $CAMERA, default auto-discover OV9782)",
    )
    p_turn_cal.add_argument(
        "--auto", action="store_true",
        help="Auto mode: run trials unattended, push OA after each, "
             "stop when residual signs alternate (converged).",
    )
    p_turn_cal.add_argument(
        "--trials", type=int, default=6,
        help="Max number of auto trials (default 6).",
    )

    args = parser.parse_args()
    _verbose = args.verbose

    commands = {
        "ports": cmd_ports,
        "hello": cmd_hello,
        "drive": cmd_drive,
        "turn": cmd_turn,
        "turnto": cmd_turnto,
        "go": cmd_go,
        "goto": cmd_goto,
        "rotate": cmd_rotate,
        "angle": cmd_angle,
        "port": cmd_port,
        "pwm": cmd_pwm,
        "stop": cmd_stop,
        "grip": cmd_grip,
        "enc": cmd_enc,
        "opos": cmd_opos,
        "ez": cmd_ez,
        "send": cmd_send,
        "line": cmd_line,
        "color": cmd_color,
        "pose": cmd_pose,
    }

    if args.command is None:
        parser.print_help()
        sys.exit(1)

    if args.command == "sync":
        sync_commands = {
            "cal": cmd_sync_cal,
            "pose": cmd_sync_pose,
        }
        try:
            sync_commands[args.sync_cmd](args)
        except ConnectionError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            sys.exit(1)
        return

    if args.command == "calibrate":
        from robot_radio.io.calibrate import cmd_calibrate_distance, cmd_calibrate_turns
        cal_commands = {
            "distance": cmd_calibrate_distance,
            "turns":    cmd_calibrate_turns,
        }
        try:
            cal_commands[args.cal_command](args)
        except ConnectionError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            sys.exit(1)
        return

    try:
        commands[args.command](args)
    except ConnectionError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
