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
from robot_radio.sensors.color import nezha_classifier
from robot_radio.config.robot_config import get_robot_config

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
    ``proto.query_ol()`` (one fast round-trip, ~tens of ms) and compares it
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
        result = conn.connect(skip_hello=True)
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
                conn._ser.reset_input_buffer()
                conn._ser.write(b"HELLO\n")
                conn._ser.flush()
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

    # Calibration freshness check — avoids the ~2.1 s push on warm starts.
    # Compute the expected OL int8 value from the active config.
    expected_ol: int | None = None
    if cfg is not None:
        lin_scale = getattr(cfg, "otos_linear_scale", 1.0) or 1.0
        expected_ol = _scale_to_int8(lin_scale)

    # Query firmware's current OL value via proto (fast round-trip).
    proto = getattr(robot, "_proto", None)
    if proto is not None and expected_ol is not None:
        actual_ol = proto.query_ol()
        _log(f"freshness check: firmware OL={actual_ol}, expected OL={expected_ol}")
        if actual_ol is None:
            # First query returned None — transient miss (port not fully settled
            # after a cache-hit open, or relay garbling).  Retry once before
            # concluding the firmware is uncalibrated and re-pushing.
            actual_ol = proto.query_ol()
            _log(f"freshness check retry: firmware OL={actual_ol}, expected OL={expected_ol}")
        if actual_ol != expected_ol:
            # Mismatch (including None after retry): firmware was power-cycled
            # or never calibrated this session.
            print(
                "Warning: firmware not calibrated — running sync cal automatically.",
                file=sys.stderr,
            )
            _push_calibration(conn)
        # else: fast path — calibration already pushed, skip.
    else:
        # Can't query (no proto, or no config to derive expected value) — push
        # unconditionally to be safe.
        _push_calibration(conn)

    return robot, conn, result


def _push_calibration(conn: SerialConnection) -> None:
    """Push every runtime-configurable calibration value from the active
    robot config to firmware.  Uses blocking send() (waits for ACK) so the
    radio queue is drained before the next command is sent — over the relay
    path, back-to-back fire-and-forget writes overflow the radio and drop
    subsequent packets (observed: G command silently dropped if K commands
    precede it without ack-gating).

    Pushes:
      * KML / KMR     ← cfg.wheels.wheel_diameter_mm
      * OL / OA       ← cfg.otos_linear_scale / otos_angular_scale
      * OI            ← init signal processing (must come before OO)
      * OO            ← cfg.geometry.odometry_offset_mm
                        (x = sensor offset along robot-forward, mm;
                         y = sensor offset along robot-left, mm;
                         yaw_rad = chip X-axis rotation from
                         robot-forward, CCW positive)
      * OK            ← IMU bias calibration (~700ms, robot must be still)

    Source of truth: data/robots/<robot>.json.  Firmware defaults are
    factory values and will be wrong for any robot whose hardware does not
    match the defaults.
    """
    cfg = get_robot_config()
    if cfg is None:
        return

    # Wheel mm/deg (KML/KMR)
    # Per-wheel calibration overrides take precedence over wheel_diameter derived value.
    wd = getattr(getattr(cfg, "wheels", None), "wheel_diameter_mm", None)
    default_mm_per_deg = (math.pi * wd / 360.0) if wd is not None else None

    cal = getattr(cfg, "calibration", None)
    left_mm_per_deg  = getattr(cal, "mm_per_wheel_deg_left",  None) if cal else None
    right_mm_per_deg = getattr(cal, "mm_per_wheel_deg_right", None) if cal else None
    left_mm_per_deg  = left_mm_per_deg  if left_mm_per_deg  is not None else default_mm_per_deg
    right_mm_per_deg = right_mm_per_deg if right_mm_per_deg is not None else default_mm_per_deg

    if left_mm_per_deg is not None:
        val_l = round(left_mm_per_deg * 1000)
        _log(f"sync calibration: K+ML+{val_l} (mm/deg={left_mm_per_deg:.4f})")
        conn.send(f"K+ML+{val_l}", read_ms=200)
    if right_mm_per_deg is not None:
        val_r = round(right_mm_per_deg * 1000)
        _log(f"sync calibration: K+MR+{val_r} (mm/deg={right_mm_per_deg:.4f})")
        conn.send(f"K+MR+{val_r}", read_ms=200)

    # OTOS distance and heading scalars (OL/OA, int8 with 0.1%/step)
    lin_int8 = _scale_to_int8(getattr(cfg, "otos_linear_scale", 1.0) or 1.0)
    ang_int8 = _scale_to_int8(getattr(cfg, "otos_angular_scale", 1.0) or 1.0)
    _log(f"sync calibration: OL{lin_int8:+d} OA{ang_int8:+d} "
         f"(linear={1.0 + lin_int8 * 0.001:.4f}, "
         f"angular={1.0 + ang_int8 * 0.001:.4f})")
    conn.send(f"OL{lin_int8:+d}", read_ms=200)
    conn.send(f"OA{ang_int8:+d}", read_ms=200)

    # OTOS init must come before OO so the chip is ready to accept the
    # offset register write.  The IMU gyro bias is volatile (lost on
    # power-cycle); OK calibrates it (~600 ms, robot must be still).
    _log("sync calibration: OI (init signal processing)")
    conn.send("OI", read_ms=200)

    # OTOS mounting offset (OO).  Yaw is stored as radians in config,
    # converted to degrees for the wire protocol.  The flip flag is
    # sent as the 4th arg when present.
    geom = getattr(cfg, "geometry", None)
    off = getattr(geom, "odometry_offset_mm", None) if geom else None
    flip = bool(getattr(geom, "odometry_chip_upside_down", False)) if geom else False
    if off is not None:
        ox = int(round(float(off.x)))
        oy = int(round(float(off.y)))
        oh = int(round(math.degrees(float(off.yaw_rad))))
        of = 1 if flip else 0
        _log(f"sync calibration: OO{ox:+d}{oy:+d}{oh:+d}{of:+d} "
             f"(x={off.x:.1f}mm fwd, y={off.y:.1f}mm left, "
             f"yaw={math.degrees(off.yaw_rad):.1f}°, "
             f"flip={'upside-down' if flip else 'normal'})")
        conn.send(f"OO{ox:+d}{oy:+d}{oh:+d}{of:+d}", read_ms=200)

    # IMU bias calibration — must be after OI.  255 samples ≈ 612 ms;
    # robot MUST be stationary during this window.
    _log("sync calibration: OK (IMU calibration, ~700ms still)")
    conn.send("OK", read_ms=200)
    time.sleep(0.75)


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
    proto = getattr(robot, "_proto", None)
    if not isinstance(proto, NezhaProtocol):
        conn.disconnect()
        sys.exit("Error: rogo sync pose requires a Nezha robot with NezhaProtocol.")

    result = proto.set_world_pose(x_mm, y_mm, h_deg)
    print(
        f"sync pose: daemon=({x_cm:.1f}cm, {y_cm:.1f}cm, "
        f"{_math.degrees(yaw_rad):.1f}°)  "
        f"sent SI{x_mm:+d}{y_mm:+d}{h_deg:+d}"
    )
    for line in result.get("responses", []):
        s = str(line).lstrip("<# ")
        if "ACK" in s or "ERR" in s:
            print(f"  firmware: {s}")
            break
    conn.disconnect()


def cmd_sync_cal(args):
    """Connect to the robot, push the full calibration set, and disconnect.

    This is a one-time setup step after power-up or after editing the robot
    config.  The robot MUST be stationary during the ~700 ms IMU bias window.

    Pushes: KML, KMR, OL, OA, OI, OO, OK — same set as the automatic push
    in _make_robot(), but invoked explicitly so you can confirm the values
    before running motion commands.
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

    # Print a human-readable summary of everything pushed.
    wd = getattr(getattr(cfg, "wheels", None), "wheel_diameter_mm", None)
    default_mm_per_deg = (math.pi * wd / 360.0) if wd is not None else None
    cal = getattr(cfg, "calibration", None)
    left_mm_per_deg  = getattr(cal, "mm_per_wheel_deg_left",  None) if cal else None
    right_mm_per_deg = getattr(cal, "mm_per_wheel_deg_right", None) if cal else None
    left_mm_per_deg  = left_mm_per_deg  if left_mm_per_deg  is not None else default_mm_per_deg
    right_mm_per_deg = right_mm_per_deg if right_mm_per_deg is not None else default_mm_per_deg

    lin_int8 = _scale_to_int8(getattr(cfg, "otos_linear_scale", 1.0) or 1.0)
    ang_int8 = _scale_to_int8(getattr(cfg, "otos_angular_scale", 1.0) or 1.0)
    geom = getattr(cfg, "geometry", None)
    off = getattr(geom, "odometry_offset_mm", None) if geom else None
    flip = bool(getattr(geom, "odometry_chip_upside_down", False)) if geom else False

    print("sync cal: calibration pushed successfully")
    if left_mm_per_deg is not None:
        print(f"  KML {round(left_mm_per_deg * 1000):+d}  "
              f"(mm/deg={left_mm_per_deg:.4f})")
    if right_mm_per_deg is not None:
        print(f"  KMR {round(right_mm_per_deg * 1000):+d}  "
              f"(mm/deg={right_mm_per_deg:.4f})")
    print(f"  OL  {lin_int8:+d}  "
          f"(linear_scale={1.0 + lin_int8 * 0.001:.4f},"
          f" source={'config' if getattr(cfg, 'otos_linear_scale', None) is not None else 'default'})")
    print(f"  OA  {ang_int8:+d}  "
          f"(angular_scale={1.0 + ang_int8 * 0.001:.4f},"
          f" source={'config' if getattr(cfg, 'otos_angular_scale', None) is not None else 'default'})")
    print("  OI  (init signal processing)")
    if off is not None:
        ox = int(round(float(off.x)))
        oy = int(round(float(off.y)))
        oh = int(round(math.degrees(float(off.yaw_rad))))
        of = 1 if flip else 0
        print(f"  OO  {ox:+d}{oy:+d}{oh:+d}{of:+d}  "
              f"(x={off.x:.1f}mm fwd, y={off.y:.1f}mm left, "
              f"yaw={math.degrees(float(off.yaw_rad)):.1f}°, "
              f"flip={'upside-down' if flip else 'normal'})")
    else:
        print("  OO  (no geometry.odometry_offset_mm in config; skipped)")
    print("  OK  (IMU bias calibration; ~700ms, robot must be still)")

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
    """Drive at speed. --ms for time, --mm for distance, neither for streaming.

    When |left| or |right| is below the configured crawl threshold,
    drive falls back to crawl mode (pulse-train).  Threshold comes from
    --min-speed, then $ROGO_MIN_SPEED, else DEFAULT_MIN_SPEED_MMS.
    Crawl requires --mm and symmetric wheel speeds (left == right).
    """
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
    else:
        left_enc, right_enc = initial
        try:
            for left_enc, right_enc in robot.speed(args.left, args.right):
                print(f"ENC {left_enc} {right_enc}")
        except KeyboardInterrupt:
            print("\nCtrl-C caught, stopping...", file=sys.stderr)
        _log("sending X")
        robot.stop()
        _log("waiting for motors to stop")
        # Give firmware time to process X and confirm
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
    """Stop motors."""
    robot, conn, _ = _make_robot(args)
    robot.stop()
    print("X")
    conn.disconnect()


def cmd_turn(args):
    """Turn in place by N degrees (positive = CCW/left, negative = CW/right).

    Default (closed-loop): sends the firmware TN command and waits for
    TN+DONE or TN+TIMEOUT — no daemon or camera required.  The firmware
    runs a closed-loop OTOS turn on the robot itself (sprint-010 firmware).

    The legacy open-loop T-command path (using rotational_slip from
    nezha-1.json) is still available with --open-loop for callers that need
    the old behaviour or are running pre-010 firmware.
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

    # Closed-loop via firmware TN command (OTOS turn on-robot, no camera).
    from robot_radio.robot.protocol import NezhaProtocol

    deg_tenths = round(args.degrees * 10)
    sign = "+" if deg_tenths >= 0 else ""
    tn_cmd = f"TN{sign}{deg_tenths}"

    robot, conn, _ = _make_robot(args)
    proto = getattr(robot, "_proto", None)
    if not isinstance(proto, NezhaProtocol):
        conn.disconnect()
        sys.exit("Error: rogo turn requires a Nezha robot with NezhaProtocol.")

    _log(f"turn {args.degrees:+.1f} deg -> {tn_cmd} (firmware closed-loop OTOS)")

    # Send the TN command; ACK arrives quickly via the normal send() window.
    conn.send(tn_cmd, read_ms=500)

    # Poll for TN+DONE or TN+TIMEOUT.  We cannot use stop_token="TN+" because
    # the relay echoes the outgoing command as "# TX:TN+900" which contains
    # "TN+" — that would cause an early exit before the real reply arrives.
    # Instead we read in short bursts and scan each line ourselves, filtering
    # out relay echo lines (starting with "# TX:") and stripping a leading "<"
    # that the relay prepends to robot replies (e.g. "<TN+DONE 89").
    TN_TIMEOUT_S = 20.0
    deadline = time.time() + TN_TIMEOUT_S
    achieved: float | None = None
    timed_out = False
    no_reply = True

    while time.time() < deadline:
        burst = conn.read_lines(duration_ms=300)
        for line in burst:
            s = line.strip()
            # Filter relay echo lines (e.g. "# TX:TN+900").
            if "TX:" in s:
                _log(f"relay echo (ignored): {s}")
                continue
            # Strip leading "<" added by relay to robot-originated lines.
            clean = s.lstrip("<")
            if clean.startswith("TN+DONE"):
                no_reply = False
                parts = clean.split()
                if len(parts) > 1:
                    try:
                        achieved = float(parts[1])
                    except ValueError:
                        pass
                break
            if clean.startswith("TN+TIMEOUT"):
                no_reply = False
                timed_out = True
                parts = clean.split()
                if len(parts) > 1:
                    try:
                        achieved = float(parts[1])
                    except ValueError:
                        pass
                break
        else:
            # Inner loop completed without a break — keep polling.
            continue
        # Inner loop broke — we found the terminal reply.
        break

    conn.disconnect()

    if no_reply:
        print("WARNING: no TN+DONE/TIMEOUT received within 20 s")
    elif timed_out:
        if achieved is not None:
            print(f"WARNING: TN+TIMEOUT (achieved={achieved:.1f} deg)")
        else:
            print("WARNING: TN+TIMEOUT")
    else:
        if achieved is not None:
            err = args.degrees - achieved
            print(f"done: achieved={achieved:.1f} deg  error={err:+.1f} deg")
        else:
            print("done: TN+DONE received (no achieved value in reply)")


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

    proto.set_watchdog(WATCHDOG_MS)
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
    proto.set_watchdog(500)
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

        proto.set_watchdog(WATCHDOG_MS)
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
    """Rotate each wheel by a relative angle (PR command).

    Two positional args (L, R) — signed degrees from current position.
    Use 'x' (or '-' or '.') for either wheel to skip it.  Returns
    immediately after the firmware ACKs; the motor controller runs the
    move autonomously.

        rogo rotate 360 360 --speed 30      # both wheels one revolution
        rogo rotate 360 -360 --speed 30     # in-place spin
        rogo rotate 360 x --speed 30        # only left wheel rotates
    """
    robot, conn, _ = _make_robot(args)
    _maybe_zero(robot, args)
    l_str = "skip" if args.left is None else f"{args.left:+.1f}°"
    r_str = "skip" if args.right is None else f"{args.right:+.1f}°"

    _log(f"rotate L={l_str} R={r_str} at {args.speed}% speed")
    result = robot.rotate(args.left, args.right, args.speed)
    for line in result.get("responses", []):
        s = str(line).lstrip("<# ")
        if s.startswith("ACK:") or s.startswith("ERR"):
            print(s)
            break


    conn.disconnect()


def cmd_angle(args):
    """Drive each wheel to an absolute angle (PA command).

    Two positional args (L, R) — 0..360°.  Use 'x' (or '-' or '.') for
    either wheel to skip it.  --cw / --ccw force a direction; default
    is the shortest path.  Returns immediately after the firmware ACKs.

        rogo angle 0 0 --speed 20           # park both at 0° (shortest)
        rogo angle 0 x --speed 20           # only left wheel moves
        rogo angle 90 270 --cw --speed 20   # asymmetric, CW only
    """
    if args.cw and args.ccw:
        print("Error: --cw and --ccw are mutually exclusive", file=sys.stderr)
        sys.exit(1)
    mode = 1 if args.cw else 2 if args.ccw else 3
    robot, conn, _ = _make_robot(args)
    _maybe_zero(robot, args)
    l_str = "skip" if args.left is None else f"{args.left:.1f}°"
    r_str = "skip" if args.right is None else f"{args.right:.1f}°"
    mode_name = "CW" if mode == 1 else "CCW" if mode == 2 else "SHORTEST"
    _log(f"angle L={l_str} R={r_str} {mode_name} at {args.speed}% speed")
    result = robot.angle(args.left, args.right, mode, args.speed)
    for line in result.get("responses", []):
        s = str(line).lstrip("<# ")
        if s.startswith("ACK:") or s.startswith("ERR"):
            print(s)
            break
    conn.disconnect()


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
        wire = f"P+{args.jack}"
    else:
        if args.value not in (0, 1):
            print(f"Error: digital value must be 0 or 1, got {args.value}", file=sys.stderr)
            conn.disconnect()
            sys.exit(1)
        wire = f"P+{args.jack}+{args.value}"
    result = robot.send(wire, read_ms=400)
    for line in result.get("responses", []):
        s = str(line).lstrip("<# ")
        if s.startswith(("ACK:P ", "P ", "ERR")):
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
        wire = f"PA+{args.jack}"
    else:
        if not (0 <= args.value <= 1023):
            print(f"Error: PWM value must be 0..1023, got {args.value}", file=sys.stderr)
            conn.disconnect()
            sys.exit(1)
        wire = f"PA+{args.jack}+{args.value}"
    result = robot.send(wire, read_ms=400)
    for line in result.get("responses", []):
        s = str(line).lstrip("<# ")
        if s.startswith(("ACK:PA ", "PA ", "ERR")):
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
    """Read encoder positions (mm)."""
    robot, conn, _ = _make_robot(args)
    left_enc, right_enc = robot.read_encoders()
    print(f"ENC {left_enc} {right_enc}")
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


def cmd_line(args):
    """Read the 4-channel line sensor: 'LS g1 g2 g3 g4' (0..255 each)."""
    robot, conn, _ = _make_robot(args)
    result = robot.send("LS", read_ms=300)
    line = _find_response(result.get("responses", []), "LS ")
    if line is None:
        print("Error: no LS response from robot", file=sys.stderr)
        conn.disconnect()
        sys.exit(1)
    print(line)
    conn.disconnect()


def cmd_color(args):
    """Read the color sensor.

    Default output: HSL (hue 0..360, sat 0..100, light 0..100), computed
    from the *white-balanced* RGB so it matches what the classifier sees.
    Add ``--name`` for a single label, ``--rgb`` / ``--hsv`` for those
    formats, or ``--raw`` to dump raw RGBC counts.  Pass
    ``--calibrate-white`` to install a fresh white reference (otherwise
    the Nezha factory default is used).
    """
    import colorsys

    robot, conn, _ = _make_robot(args)
    clf = nezha_classifier()

    if args.calibrate_white:
        from robot_radio.sensors.color import calibrate_white as _do_calibrate
        def _read():
            res = robot.send("CS", read_ms=400)
            ln = _find_response(res.get("responses", []), "CS ")
            if not ln:
                return None
            p = ln.split()
            return (int(p[1]), int(p[2]), int(p[3]), int(p[4]))
        try:
            wr = _do_calibrate(clf, _read)
            print(f"# white ref: R={wr[0]} G={wr[1]} B={wr[2]} C={wr[3]}",
                  file=sys.stderr)
        except RuntimeError as e:
            print(f"Error: {e}", file=sys.stderr)
            conn.disconnect()
            sys.exit(1)

    result = robot.send("CS", read_ms=600)
    line = _find_response(result.get("responses", []), "CS ")
    if line is None:
        print("Error: no CS response from robot", file=sys.stderr)
        conn.disconnect()
        sys.exit(1)
    conn.disconnect()

    parts = line.split()
    if len(parts) < 5:
        print(f"Error: malformed CS response: {line}", file=sys.stderr)
        sys.exit(1)
    r_raw, g_raw, b_raw, c_raw = (int(parts[1]), int(parts[2]),
                                  int(parts[3]), int(parts[4]))

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
    p_drive.add_argument("--ms", type=int, default=None, help="Duration in ms (blocking)")
    p_drive.add_argument("--mm", type=int, default=None, help="Distance in mm (blocking)")
    p_drive.add_argument("--ez", action="store_true", help="Zero encoders before driving")
    p_drive.add_argument(
        "--min-speed", type=int, default=None,
        help=f"Crawl-mode threshold in mm/s (default: {DEFAULT_MIN_SPEED_MMS}, "
             "or $ROGO_MIN_SPEED if set).",
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

    sub.add_parser("enc", help="Read encoder positions (mm)")
    sub.add_parser("ez", help="Zero encoders")
    sub.add_parser("line", help="Read 4-channel line sensor (grayscale 0..255)")
    p_color = sub.add_parser(
        "color",
        help="Read color sensor. Default: HSL. --rgb, --hsv, or --name for other forms.",
    )
    p_color_fmt = p_color.add_mutually_exclusive_group()
    p_color_fmt.add_argument("--rgb", action="store_true", help="Output display RGB (0..255)")
    p_color_fmt.add_argument("--hsv", action="store_true", help="Output HSV (hue 0..360, S/V 0..100)")
    p_color_fmt.add_argument("--name", action="store_true", help="Output a single colour label")
    p_color_fmt.add_argument("--raw", action="store_true", help="Output raw 'CS r g b c' line from firmware")
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
        help="Push full calibration to firmware (KML, KMR, OL, OA, OI, OO, OK). "
             "Run once after power-up or after editing the robot config. "
             "Robot must be stationary during the ~700 ms IMU bias window.",
    )

    sync_sub.add_parser(
        "pose",
        help="Seed robot OTOS odometry from daemon world pose (run once at start "
             "of session or after repositioning). Requires the aprilcam daemon "
             "running from the AprilTags project directory with a calibrated "
             "playfield. Reads tag.world_xy (cm, A1-centred) and tag.yaw (rad) "
             "from the daemon, converts to mm/degrees, and sends SI<x><y><h> "
             "to firmware. The firmware heading = degrees(tag.yaw) + 90° "
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
