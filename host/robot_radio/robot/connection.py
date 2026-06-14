"""robot_radio.robot.connection — shared robot construction and port resolution.

Both the CLI (``robot_radio.io.cli``) and the MCP server
(``robot_radio.io.robot_mcp``) need to:

  1. Resolve which serial port to open.
  2. Perform the HELLO handshake to identify the device.
  3. Construct the right Robot/Protocol pair.
  4. Manage a session cache (``data/.rogo_session.json``) so repeated
     invocations skip the ~300 ms HELLO scan.

This module is the single authoritative implementation of all four concerns so
that the two front-ends cannot diverge.

Port resolution precedence
--------------------------
1. ``args.port`` — explicit ``--port`` flag always wins; cache is bypassed.
2. Session cache (``data/.rogo_session.json``) — if the cached port is still
   present in the current port list, return it immediately.
3. Auto-detect — return the first port from ``list_serial_ports()``.

Session cache fast-path
-----------------------
After a successful HELLO that produced a confidently detected mode (a
``DEVICE:`` announcement was parsed, not a fallback guess), the cache is
written with the port, mode, and device_name.  On the next invocation the
cache is checked first; if the port is still present the connection is opened
with ``skip_ping=True``, skipping the 300 ms sleep and announcement read.

Both the CLI and MCP use the SAME cache file (``_SESSION_CACHE_PATH``) and the
SAME read/write logic, so a ``rogo hello`` and an MCP ``connect`` share the
fast path.
"""

from __future__ import annotations

import json
import os
import sys
import time

from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
from robot_radio.robot.nezha import Nezha
from robot_radio.robot.protocol import NezhaProtocol
from robot_radio.config.robot_config import get_robot_config

# ---------------------------------------------------------------------------
# Session cache location
# ---------------------------------------------------------------------------

# Path to the ephemeral connection cache file.  Computed from __file__ so it
# works regardless of the current working directory when rogo is invoked.
_SESSION_CACHE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "..", "data", ".rogo_session.json"
)


def read_session_cache() -> dict | None:
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


def write_session_cache(port: str, mode: str, device_name: str) -> None:
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


# ---------------------------------------------------------------------------
# Device-line parse helper
# ---------------------------------------------------------------------------

def _parse_device_line(lines: list[str]) -> dict | None:
    """Find and parse a DEVICE: announcement from serial lines.

    Tolerant of garbled serial — looks for 'DEVICE:' anywhere in the line.
    """
    for line in lines:
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


# ---------------------------------------------------------------------------
# Port resolution
# ---------------------------------------------------------------------------

def get_port(args) -> str:
    """Resolve port from args, session cache, or auto-detect.

    Precedence:
      1. ``--port`` flag — explicit port always wins; cache is bypassed.
      2. Session cache (data/.rogo_session.json) — if the cached port is
         still present in the current port list, return it.  This skips the
         auto-detection scan and lets ``make_robot()`` take the fast path.
      3. Auto-detect: return the first port from ``list_serial_ports()``.
    """
    if args.port:
        return args.port
    ports = list_serial_ports()
    if not ports:
        print("Error: No USB modem ports found.", file=sys.stderr)
        sys.exit(1)
    cache = read_session_cache()
    if cache and cache.get("port") in ports:
        return cache["port"]
    return ports[0]


# ---------------------------------------------------------------------------
# Calibration path helper
# ---------------------------------------------------------------------------

def calibration_path() -> str:
    """Return the absolute path to data/robot_calibration.json."""
    return os.path.join(os.path.dirname(__file__), "..", "..",
                        "data", "robot_calibration.json")


# ---------------------------------------------------------------------------
# Robot construction
# ---------------------------------------------------------------------------

def make_robot(
    port: str | None,
    mode: str | None,
    verbose: bool,
    args,
) -> tuple:
    """Connect and return ``(robot, connection, connect_result)``.

    Parameters
    ----------
    port:
        Explicit port string, or ``None`` to resolve via ``get_port(args)``.
    mode:
        Explicit mode string (``'relay'`` or ``'direct'``), or ``None`` to
        auto-detect from the DEVICE: announcement.
    verbose:
        When ``True``, a log callback is attached to the connection so every
        TX command is printed to stderr.
    args:
        An object with a ``port`` attribute (argparse Namespace or a simple
        namespace created by ``_mock_args``).  Used by ``get_port`` for the
        ``--port`` override.

    Returns
    -------
    tuple
        ``(robot, conn, result)`` where:

        - ``robot`` is a :class:`~robot_radio.robot.nezha.Nezha` (or Cutebot
          for legacy hardware) instance.
        - ``conn`` is the open :class:`~robot_radio.io.serial_conn.SerialConnection`.
        - ``result`` is the connect result dict from
          :meth:`~robot_radio.io.serial_conn.SerialConnection.connect`.

    Connection cache (warm-path speedup)
    ------------------------------------
    Before the full HELLO handshake, checks data/.rogo_session.json for a
    cached port+mode pair.  If the cached port matches the resolved port AND
    it is still present in the current port list, the connection is opened
    directly (``skip_ping=True``) without the 300 ms sleep or announcement
    read.  On cache miss (stale port, missing file, or malformed JSON), falls
    back to the full HELLO handshake and writes the cache on success.

    After a successful HELLO that produced a confidently detected mode (a
    DEVICE: announcement was parsed, not a fallback guess), the cache is
    written with the port, mode, and device_name.
    """
    def _log(msg: str):
        if verbose:
            print(f"  [{msg}]", file=sys.stderr)

    resolved_port = port if port is not None else get_port(args)
    on_send = (lambda cmd: _log(f"TX: {cmd}")) if verbose else None

    # Check if we can use the fast-path (cache hit).
    # Explicit port overrides the cache: if the caller specified a port, do a
    # full HELLO regardless (they may have swapped devices).
    # For the args-based path, args.port being set also disables cache.
    explicit_port = bool(port) or bool(getattr(args, "port", None))
    cache = read_session_cache()
    ports = list_serial_ports()
    use_cache = (
        not explicit_port
        and cache is not None
        and cache.get("port") == resolved_port
        and resolved_port in ports
    )

    conn: SerialConnection

    if use_cache:
        cached_mode = cache["mode"]
        _log(f"cache hit: port={resolved_port} mode={cached_mode} — skipping HELLO")
        conn = SerialConnection(resolved_port, mode=cached_mode, on_send=on_send)
        result = conn.connect(skip_ping=True)
        if "error" in result:
            # Cache path failed — fall through to full HELLO below.
            _log(f"cache-hit connect failed ({result['error']}), falling back to full HELLO")
            use_cache = False
        else:
            # Validate the cache against the poll-time announcement (if any).
            ann = result.get("announcement")
            if ann:
                role = ann.get("role", "").upper()
                detected_mode = "relay" if ("RELAY" in role or "BRIDGE" in role) else "direct"
                detected_device = ann.get("device_name", "")
                cached_device = cache.get("device_name", "")
                mode_mismatch = (detected_mode != cached_mode)
                device_mismatch = (detected_device and detected_device != cached_device)
                if mode_mismatch or device_mismatch:
                    print(
                        f"Warning: session cache stale "
                        f"(mode={cached_mode!r}→{detected_mode!r}, "
                        f"device={cached_device!r}→{detected_device!r}) "
                        f"— re-detected mode={detected_mode}",
                        file=sys.stderr,
                    )
                    conn._mode = detected_mode
                    write_session_cache(resolved_port, detected_mode, detected_device)

    if not use_cache:
        # Full HELLO path: auto-detect mode or use caller-supplied mode.
        # connect() now runs the HELLO-classify handshake internally:
        # it sends HELLO, reads the DEVICE: banner before the reader thread
        # starts, runs !ECHO OFF / !MODE RAW250 / !GO for relay devices, then
        # sets conn._mode = "direct" (the post-!GO relay is a transparent pipe).
        if mode is not None:
            conn = SerialConnection(resolved_port, mode=mode, on_send=on_send)
        else:
            conn = SerialConnection(resolved_port, on_send=on_send)
        _log(f"connecting to {resolved_port}...")
        result = conn.connect()
        if "error" in result:
            print(f"Error: {result['error']}", file=sys.stderr)
            sys.exit(1)

        # connect() populates result["announcement"] when HELLO-classify
        # succeeded.  Also check result["lines"] for legacy / fallback cases.
        ann = result.get("announcement")
        if not ann:
            ann = _parse_device_line(result.get("lines", []))
            if ann:
                result["announcement"] = ann
        _log(f"HELLO response: announcement={ann}, lines={result.get('lines', [])}")

        # If classify timed out (no banner), try one more HELLO-classify
        # cycle by reconnecting.  The reader thread is already running so we
        # cannot call _banner_classify again, but we can check cached lines.
        # In practice the 2.5 s classify budget in connect() is generous enough
        # that this path should not be needed.
        if not ann:
            conn.disconnect()
            print(f"Error: No device found on {resolved_port}. Is it powered on?",
                  file=sys.stderr)
            sys.exit(1)

        # Determine the logical connection mode for logging and session cache.
        # conn._mode is always "direct" after the new handshake (the relay is
        # transparent post-!GO), but the session cache records the underlying
        # device type so we can log it meaningfully.
        role = ann.get("role", "").upper()
        is_relay = "RELAY" in role or "BRIDGE" in role
        # conn._mode is already "direct" from connect(); leave it alone.
        # Use "relay" as the cache key only if the caller did not override.
        cache_mode = "relay" if (mode is None and is_relay) else conn._mode

        _log(f"connected to {ann.get('role', '?')} '{ann.get('common_name', '?')}' "
             f"on {resolved_port} (mode={conn.mode}, relay={is_relay})")

        # Write the session cache (only when mode was auto-detected from
        # a DEVICE: announcement, not from a caller-supplied mode override).
        if mode is None:
            device_name = ann.get("device_name", "")
            write_session_cache(resolved_port, cache_mode, device_name)

    # Construct the robot driver.
    cfg = get_robot_config()
    hw_model = getattr(cfg, "hardware_model", "cutebot").lower() if cfg else "cutebot"
    if "nezha" in hw_model:
        robot = Nezha(NezhaProtocol(conn))
    else:
        from robot_radio.robot import Cutebot
        robot = Cutebot(conn)

    return robot, conn, result
