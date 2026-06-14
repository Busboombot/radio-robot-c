"""Serial connection management for micro:bit relay/direct devices.

Architecture (sprint 025, ticket 001):
---------------------------------------
SerialConnection owns the physical serial port and all I/O on it.  A single
background reader thread holds the only ``_ser.readline()`` call and
demultiplexes every incoming line into one of three queues:

- ``_reply_queues`` — keyed by corr-id; populated by the firmware's
  ``OK``/``ERR``/``CFG`` replies when the host appended ``#<id>`` to the
  command.  ``send()`` blocks on the corr-id-keyed queue entry it created
  before writing.
- ``_tlm_queue``   — bounded (256 frames); receives every ``TLM ...`` line.
  ``read_lines()`` and ``read_pending_lines()`` drain it.
- ``_evt_queue``   — unbounded; receives every ``EVT ...`` line.
  ``read_lines()`` and ``read_pending_lines()`` drain it; so does any caller
  of ``wait_for_evt_done()``.

Nothing outside ``SerialConnection`` reads from ``_ser`` directly.  The only
intentional internal ``_ser`` access points are:

- ``_banner_classify`` — ``write()`` / ``readline()`` during the pre-reader
                         HELLO classify handshake (before ``connect()`` returns).
- ``_relay_handshake``  — ``write()`` / ``readline()`` during the relay
                         ``!ECHO OFF`` / ``!MODE RAW250`` / ``!GO`` sequence,
                         also before the reader thread starts.
- ``_poll_ready``    — ``reset_input_buffer()``, ``write()``, ``readline()``
                       before the reader thread starts.
- ``_keepalive_loop`` — ``write()`` / ``flush()`` under ``_write_lock``.
- ``handshake()``   — ``write()`` / ``flush()`` under ``_write_lock``, before
                       the reader thread starts (device-detection phase only).
- ``_reader_loop``  — sole owner of ``readline()`` after ``connect()`` returns.

Connection handshake (sprint 036, ticket 007):
----------------------------------------------
``connect()`` now performs a HELLO-classify step before the reader thread starts:

1. The port is opened **with DTR asserted** (pyserial default).  DTR pulses on
   open-time close/reopen, resetting any micro:bit on the port and causing it to
   emit a ``DEVICE:`` announcement banner.  There is no ``dtr = False`` override.

2. ``_banner_classify()`` sends ``HELLO`` repeatedly (up to ~10 times, ~200 ms
   apart) and reads each response until it captures a ``DEVICE:<ROLE>:...`` line.
   Parsed ROLE determines the connection mode:

   - ``RADIOBRIDGE`` → relay; proceed to ``_relay_handshake()``.
   - ``NEZHA2``      → direct USB robot; skip to readiness poll.

3. ``_relay_handshake()`` sends ``!ECHO OFF``, ``!MODE RAW250``, then ``!GO``
   and waits for ``# entering data plane``.  After ``!GO`` the relay is a
   transparent byte pipe.  All subsequent traffic is **plain** (no ``>`` prefix).

4. After the handshake, ``connect()`` proceeds with the PING readiness poll, then
   starts the keepalive daemon and reader thread as before.  From the reader
   thread's perspective the relay connection is indistinguishable from direct: the
   same plain send, same ``+`` keepalive, same ``#<id>`` corr-id.

Radio channel note:
   The relay's channel, group, and mode persist in its flash.  Matching those
   values between the relay and the robot is a bench-setup concern, not managed
   here.  ``_banner_classify()`` queries ``?`` and logs the relay's reported
   channel/group/mode so mismatches are visible in verbose output.

Reader loop:
   Lines beginning with ``#`` are relay status/comment lines.  The reader loop
   drops them silently; they do not generate protocol errors.
"""

import glob
import queue
import re
import threading
import time
from typing import Any

import serial

BAUD_RATE = 115200
DEFAULT_PORT = "/dev/cu.usbmodem21431202"
READ_TIMEOUT_S = 0.12

# System safety-stop watchdog keepalive. The firmware safety-stops ANY motion
# after sTimeoutMs (default 500) of host silence, so the host must continuously
# send "+" keepalives while connected. We send them from a background daemon
# thread well inside that window, so if this process dies the keepalives stop
# and the robot safety-stops on its own. See LoopScheduler.cpp watchdog.
_KEEPALIVE_PERIOD_S = 0.15

# Active readiness-poll constants.
# After opening the serial port, the device is not immediately ready — the
# first command's reply is reliably lost if we simply sleep.  Instead we
# actively poll: send PING (v2), wait a short per-attempt window, retry until
# we see a valid response or hit the total timeout.
#
# Per-attempt read window: long enough to catch a single readline() from a
# responsive device, short enough that the poll loop is tight.
_POLL_ATTEMPT_MS = 130  # ms per PING attempt
# Total readiness budget for the normal (full PING) path.
_POLL_TOTAL_NORMAL_S = 1.5
# Total readiness budget for the fast (skip_ping / cache-hit) path.
# Shorter to preserve the cache speedup; device should already be running.
_POLL_TOTAL_FAST_S = 0.6

# HELLO-classify constants (sprint 036, ticket 007).
# Per-attempt delay between HELLO sends in the banner-classify loop.
_HELLO_ATTEMPT_DELAY_S = 0.20
# Total timeout budget for the HELLO-classify step.
_HELLO_CLASSIFY_TIMEOUT_S = 2.5
# Timeout for each relay command during the !GO handshake sequence.
_RELAY_CMD_TIMEOUT_S = 1.0

# Bounded TLM queue depth: if the consumer is slow, oldest frames are dropped.
_TLM_QUEUE_DEPTH = 256

# Corr-id pattern: ``#<digits>`` at the end of a reply line.
_CORR_ID_RE = re.compile(r"#(\d+)$")


def _disable_hupcl(ser) -> None:
    """Clear the HUPCL termios flag so close() does NOT pulse DTR.

    On macOS/Linux the default tty behaviour asserts DTR when the last handle
    closes (HUPCL = "hang up on close"), which the micro:bit DAPLink interprets
    as a target reset. Clearing it lets a CLI command open/close the port
    without rebooting the robot. No-op on platforms without termios.
    """
    try:
        import termios
        fd = ser.fileno()
        attrs = termios.tcgetattr(fd)
        attrs[2] &= ~termios.HUPCL      # c_cflag
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
    except Exception:
        pass


def _parse_device_banner(line: str) -> dict[str, Any] | None:
    """Parse a ``DEVICE:<ROLE>:<common>:<name>:<serial>`` announcement line.

    Tolerant of garbled prefix — locates ``DEVICE:`` anywhere in the line.
    Returns a dict with ``role``, ``common_name``, ``device_name``,
    ``serial_field`` keys, or ``None`` if no ``DEVICE:`` segment is found.
    """
    idx = line.find("DEVICE:")
    if idx < 0:
        return None
    parts = line[idx:].split(":")
    if len(parts) < 5:
        return None
    return {
        "role": parts[1],
        "common_name": parts[2],
        "device_name": parts[3],
        "serial_field": ":".join(parts[4:]),
    }


class SerialConnection:
    """Manages a serial connection to a micro:bit relay or direct device.

    After ``connect()`` returns, a background reader thread is the sole owner
    of ``_ser.readline()``.  It demultiplexes incoming lines into:

    - ``_reply_queues[corr_id]`` for ``OK``/``ERR``/``CFG`` replies.
    - ``_tlm_queue`` for ``TLM`` frames.
    - ``_evt_queue`` for ``EVT`` lines.

    ``send()`` appends ``#<corr_id>`` to every command and blocks on the
    corr-id-keyed reply queue.  ``read_lines()`` drains ``_tlm_queue`` and
    ``_evt_queue`` without ever calling ``_ser.readline()``.
    """

    def __init__(self, port: str = DEFAULT_PORT, baud: int = BAUD_RATE,
                 mode: str | None = None, on_send=None):
        self._port = port
        self._baud = baud
        self._mode = mode  # None = auto-detect from announcement
        self._ser: serial.Serial | None = None
        self.on_send = on_send  # callback(cmd_str) for verbose logging
        # Serial-write lock: serializes the keepalive thread's writes with the
        # main thread's command writes so their bytes never interleave.
        self._write_lock = threading.RLock()
        self._ka_thread: threading.Thread | None = None
        self._ka_stop = threading.Event()

        # ── Reader thread infrastructure (sprint 025, ticket 001) ────────────
        # One queue per in-flight corr-id; created before write, deleted after
        # reply.  Keyed by str(corr_id); "" is the catch-all for un-correlated
        # OK/ERR/CFG replies.
        self._reply_queues: dict[str, queue.Queue] = {}
        self._reply_lock = threading.Lock()

        # Bounded TLM queue: drop oldest frame on overflow rather than blocking
        # the reader thread.
        self._tlm_queue: queue.Queue = queue.Queue(maxsize=_TLM_QUEUE_DEPTH)

        # EVT queue: unbounded — EVT lines must not be dropped.
        self._evt_queue: queue.Queue = queue.Queue()

        # Reader thread state.
        self._reader_thread: threading.Thread | None = None
        self._reader_stop = threading.Event()

        # Monotonically incrementing corr-id source for send().
        self._corr_counter: int = 0

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def port(self) -> str | None:
        return self._port if self.is_open else None

    @property
    def mode(self) -> str | None:
        return self._mode

    def connect(self, skip_ping: bool = False, reset: bool = False) -> dict[str, Any]:
        """Open port, classify device, run handshake, start reader thread.

        Handshake algorithm (sprint 036, ticket 007):

        1. Open the port with **DTR asserted** (pyserial default — no
           ``dtr = False`` override).  Opening the port toggles DTR, which
           resets any micro:bit (via the DAPLink) into a clean command plane.
           The relay then emits a ``DEVICE:`` boot announcement.  Without a
           reset (no DTR pulse) there is no boot banner; the classify step sends
           ``HELLO`` to request one explicitly.

        2. ``_banner_classify()`` sends ``HELLO`` repeatedly (up to the
           ``_HELLO_CLASSIFY_TIMEOUT_S`` budget) until a ``DEVICE:<ROLE>:...``
           line is captured.  The ``DEVICE:`` line is read RAW (before the
           reader thread starts) so it is never silently dropped by the reader.
           Returns ``(role, banner_line)``.

        3a. If ROLE is ``RADIOBRIDGE`` (relay): ``_relay_handshake()`` sends
            ``?`` (log channel/group/mode), ``!ECHO OFF``, ``!MODE RAW250``,
            then ``!GO``.  After ``# entering data plane`` the relay is a
            transparent byte pipe.  ``self._mode`` is set to ``"direct"``
            (indistinguishable from a direct robot connection from the reader's
            perspective).

        3b. If ROLE is ``NEZHA2`` (direct robot): no ``!GO`` needed.
            ``self._mode`` is set to ``"direct"``.

        After the handshake, ``connect()`` proceeds with the PING readiness
        poll, starts the keepalive daemon, then the reader thread.

        Notes on HUPCL and DTR:
            On macOS/Linux close() pulses DTR via the HUPCL termios flag.
            For the relay this is desirable (next open will reset it again).
            For a direct robot connection it is undesirable (SET state lost).
            The ``reset`` parameter and ``_disable_hupcl`` control this.

        Radio channel note:
            Channel/group matching between relay and robot is a bench-setup
            concern.  ``_relay_handshake()`` queries and logs relay config via
            ``?`` so mismatches are visible to the operator.

        Args:
            skip_ping: When True (cache-hit fast path), skip the HELLO
                classify and readiness poll; use the cached ``self._mode``.
                The return dict will have ``lines=[]`` and ``pinged=False``.
            reset: When True, do NOT disable HUPCL — let close() pulse DTR and
                reset the device on exit.  Default False (preserve device state).
        """
        if self.is_open:
            if self._ser.port == self._port:
                return {"status": "already_connected", "port": self._port, "mode": self._mode}
            self._ser.close()

        try:
            # Open the port with DTR asserted (pyserial default).
            #
            # Historical note: an earlier version of this code forced
            # ``dtr=False`` to avoid resetting the device on open.  That
            # worked for direct robot connections where the device was already
            # running, but prevented the relay from emitting its DEVICE: banner
            # (no DTR pulse → no reset → no boot announcement).  The banner is
            # required for HELLO-classify.  DTR assertion is the correct default
            # for the relay path; for direct connections it merely resets the
            # robot into a clean state, which is benign.
            self._ser = serial.Serial(baudrate=self._baud, timeout=READ_TIMEOUT_S,
                                      dsrdtr=False, rtscts=False)
            self._ser.port = self._port
            # Do NOT force dtr=False here.  Let DTR stay asserted (the
            # dsrdtr=False kwarg above disables *hardware* flow-control, not
            # the DTR signal itself; pyserial defaults DTR to True when opening).
            self._ser.open()
            if not reset:
                # On macOS/Linux, close() pulses DTR via the HUPCL termios
                # flag, which the DAPLink reads as a target reset.  Clear HUPCL
                # so that close() leaves the line alone and does not reboot the
                # device on CLI exit.  Pass reset=True to deliberately reboot.
                _disable_hupcl(self._ser)

            if skip_ping:
                # Fast cache-hit path: skip HELLO classify and PING poll.
                # Mode was set by the caller from the session cache.
                if self._mode is None:
                    self._mode = "direct"
                self.start_keepalive()
                self._start_reader()
                return {
                    "status": "connected",
                    "port": self._port,
                    "mode": self._mode,
                    "lines": [],
                    "pinged": False,
                }

            # HELLO-classify: identify device role BEFORE starting the reader.
            # All I/O here is raw (_ser direct); the reader thread is not yet
            # running so DEVICE: lines cannot be silently dropped by it.
            announce: dict[str, Any] | None = None
            if self._mode is None:
                role, banner_line = self._banner_classify(
                    timeout_s=_HELLO_CLASSIFY_TIMEOUT_S)
                if banner_line:
                    announce = _parse_device_banner(banner_line)
                if role == "relay":
                    relay_info = self._relay_handshake(timeout_s=_RELAY_CMD_TIMEOUT_S)
                    self._mode = "direct"  # post-!GO: transparent plain pipe
                else:
                    # NEZHA2 or unknown role → treat as direct robot.
                    self._mode = "direct"
            else:
                # Caller supplied an explicit mode; skip classify.
                role = "relay" if self._mode == "relay" else "direct"

            # Normal path: active readiness poll via PING.
            # _poll_ready uses _ser directly (reader not running yet).
            lines = self._poll_ready(total_timeout_s=_POLL_TOTAL_NORMAL_S)

            self.start_keepalive()
            self._start_reader()

            result: dict[str, Any] = {
                "status": "connected",
                "port": self._port,
                "mode": self._mode,
                "lines": lines,
                "pinged": bool(lines),
            }
            if announce:
                result["announcement"] = announce
            return result

        except Exception as exc:
            self._ser = None
            return {"error": str(exc), "port": self._port}

    # ── HELLO-classify and relay handshake (sprint 036, ticket 007) ─────────

    def _banner_classify(
        self, timeout_s: float = _HELLO_CLASSIFY_TIMEOUT_S
    ) -> tuple[str, str]:
        """Send HELLO until a DEVICE: banner arrives; return (role, banner_line).

        Operates on ``_ser`` directly (before the reader thread starts).
        Sends ``HELLO`` up to once per ``_HELLO_ATTEMPT_DELAY_S`` and reads
        until ``timeout_s`` is exhausted.

        Returns:
            (role, banner_line) where role is ``"relay"`` or ``"direct"``.
            If no banner is captured within the timeout, returns
            ``("direct", "")``.
        """
        deadline = time.time() + timeout_s
        next_hello = 0.0  # send immediately on the first iteration

        while time.time() < deadline:
            now = time.time()
            if now >= next_hello:
                try:
                    self._ser.write(b"HELLO\n")
                    self._ser.flush()
                except Exception:
                    break
                next_hello = now + _HELLO_ATTEMPT_DELAY_S

            try:
                raw = self._ser.readline()
            except Exception:
                break
            if not raw:
                continue

            try:
                text = raw.decode("utf-8", "ignore").strip()
            except Exception:
                continue

            if not text:
                continue

            # Look for the DEVICE: announcement.
            idx = text.find("DEVICE:")
            if idx >= 0:
                parts = text[idx:].split(":")
                # DEVICE:<ROLE>:<common_name>:<device_name>:<serial>
                role_field = parts[1].upper() if len(parts) >= 2 else ""
                if "RADIOBRIDGE" in role_field or "RADIORELAY" in role_field:
                    return "relay", text
                # NEZHA2 or any other robot type → direct
                return "direct", text

        # Timeout reached without a banner.
        return "direct", ""

    def _relay_handshake(self, timeout_s: float = _RELAY_CMD_TIMEOUT_S) -> dict[str, Any]:
        """Run the relay command-plane setup and enter the data plane.

        Sequence (must be done before the reader thread starts):
          1. ``?``         — query and log channel/group/mode/power.
          2. ``!ECHO OFF`` — disable transponder echo.
          3. ``!MODE RAW250`` — select headerless 250-byte framing.
          4. ``!GO``       — enter the transparent data plane.

        Returns a dict with the relay's reported config (from ``?``) and
        whether ``# entering data plane`` was seen.

        Operates on ``_ser`` directly.  All relay responses are ``#``-prefixed
        comment lines which the reader loop would silently drop; we consume them
        here before the reader starts.
        """
        info: dict[str, Any] = {}

        def _send_relay_cmd(cmd_bytes: bytes, ack_fragment: str) -> str:
            """Send a relay command and wait for a line containing ack_fragment."""
            try:
                self._ser.write(cmd_bytes)
                self._ser.flush()
            except Exception:
                return ""
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                try:
                    raw = self._ser.readline()
                except Exception:
                    break
                if not raw:
                    continue
                try:
                    text = raw.decode("utf-8", "ignore").strip()
                except Exception:
                    continue
                if not text:
                    continue
                if ack_fragment in text:
                    return text
            return ""

        # Query relay config for logging; result is informational.
        query_resp = _send_relay_cmd(b"?\n", "channel:")
        if query_resp:
            info["relay_config"] = query_resp

        # !ECHO OFF — disable echo (transponder mode off).
        _send_relay_cmd(b"!ECHO OFF\n", "echo:")

        # !MODE RAW250 — headerless framing (must match robot firmware).
        _send_relay_cmd(b"!MODE RAW250\n", "mode:")

        # !GO — enter data plane.  Relay replies with "# entering data plane".
        go_resp = _send_relay_cmd(b"!GO\n", "entering data plane")
        info["entered_data_plane"] = "entering data plane" in go_resp

        return info

    def _start_reader(self) -> None:
        """Start the background reader thread.  Idempotent."""
        if self._reader_thread is not None and self._reader_thread.is_alive():
            return
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="serial-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def _stop_reader(self) -> None:
        """Signal the reader thread to stop and wait for it."""
        self._reader_stop.set()
        t = self._reader_thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=1.0)
        self._reader_thread = None

    def _reader_loop(self) -> None:
        """Background reader: sole owner of ``_ser.readline()``.

        Classifies each decoded, stripped line and routes it to the
        appropriate queue:

        - ``TLM ...``  → ``_tlm_queue``  (drop oldest if full)
        - ``EVT ...``  → ``_evt_queue``
        - ``OK``/``ERR``/``CFG`` with ``#<id>`` suffix → ``_reply_queues[id]``
        - ``OK``/``ERR``/``CFG`` with no corr-id → ``_reply_queues[""]``
        - ``OK keepalive`` / lines containing ``keepalive`` → dropped silently
        - Lines beginning with ``#`` → relay status/comment lines, dropped
        - Anything else → dropped silently
        """
        while not self._reader_stop.is_set():
            try:
                if self._ser is None or not self._ser.is_open:
                    break
                raw = self._ser.readline()
            except Exception:
                break  # port closed or gone — exit silently

            if not raw:
                continue

            try:
                text = raw.decode("utf-8", "ignore").strip()
            except Exception:
                continue

            if not text:
                continue

            # Drop keepalive acks.
            if "keepalive" in text:
                continue

            # Drop relay comment/status lines (# channel:, # entering data
            # plane, # echo:, # mode:, # DBG ..., etc.).  These are relay
            # command-plane responses that should have been consumed during the
            # pre-reader handshake; any that leak through post-!GO are benign.
            if text.startswith("#"):
                continue

            if text.startswith("TLM"):
                # Bounded TLM queue: drop oldest on overflow.
                if self._tlm_queue.full():
                    try:
                        self._tlm_queue.get_nowait()
                    except queue.Empty:
                        pass
                try:
                    self._tlm_queue.put_nowait(text)
                except queue.Full:
                    pass  # extremely unlikely race; drop
                continue

            if text.startswith("EVT"):
                self._evt_queue.put(text)
                continue

            # Route OK/ERR/CFG replies by corr-id.
            if text.startswith(("OK", "ERR", "CFG")):
                m = _CORR_ID_RE.search(text)
                if m:
                    corr_id = m.group(1)
                else:
                    corr_id = ""
                with self._reply_lock:
                    q = self._reply_queues.get(corr_id)
                if q is not None:
                    q.put(text)
                # If no queue is registered for this id, drop silently.
                continue

            # All other lines: drop silently (diagnostics, unknown, etc.)

    def _poll_ready(self, total_timeout_s: float = _POLL_TOTAL_NORMAL_S) -> list[str]:
        """Poll PING until the device responds or total_timeout_s is exceeded.

        Sends PING (always plain — after the !GO handshake the relay is a
        transparent pipe), reads for _POLL_ATTEMPT_MS, and returns immediately
        if any non-empty response is received. Retries until total_timeout_s
        expires.  Returns the response lines from the first successful attempt
        (or []).

        This method uses ``_ser`` directly and must only be called before the
        reader thread starts.
        """
        deadline = time.time() + total_timeout_s
        cmd = b"PING\n"
        while time.time() < deadline:
            self._ser.reset_input_buffer()
            self._ser.write(cmd)
            self._ser.flush()
            lines = self._poll_read_lines(_POLL_ATTEMPT_MS, stop_token="OK pong")
            if lines:
                return lines
        return []

    def _poll_read_lines(self, duration_ms: int, stop_token: str | None = None) -> list[str]:
        """Read lines directly from ``_ser`` for up to ``duration_ms``.

        Used exclusively by ``_poll_ready`` (before the reader thread starts).
        """
        lines: list[str] = []
        deadline = time.time() + (duration_ms / 1000.0)
        while time.time() < deadline:
            try:
                raw = self._ser.readline()
            except Exception:
                break
            if not raw:
                continue
            text = raw.decode("utf-8", "ignore").strip()
            if not text:
                continue
            if "keepalive" in text:
                continue
            lines.append(text)
            if stop_token and stop_token in text:
                break
        return lines

    def disconnect(self) -> dict[str, Any]:
        """Stop keepalive and reader threads, then close the serial port."""
        if not self.is_open:
            return {"status": "not_connected"}
        self.stop_keepalive()
        self._stop_reader()
        port = self._port
        self._ser.close()
        self._ser = None
        return {"status": "disconnected", "port": port}

    # ── safety-stop keepalive ────────────────────────────────────────────────
    def start_keepalive(self, period_s: float = _KEEPALIVE_PERIOD_S) -> None:
        """Start a background daemon thread that streams "+" keepalives so the
        firmware safety-stop watchdog never trips during normal operation. If
        this process dies the daemon thread dies with it, keepalives stop, and
        the robot safety-stops on its own. Idempotent."""
        if self._ka_thread is not None and self._ka_thread.is_alive():
            return
        self._ka_stop.clear()
        self._ka_thread = threading.Thread(
            target=self._keepalive_loop, args=(period_s,),
            name="serial-keepalive", daemon=True)
        self._ka_thread.start()

    def stop_keepalive(self) -> None:
        self._ka_stop.set()
        t = self._ka_thread
        if t is not None and t.is_alive() and t is not threading.current_thread():
            t.join(timeout=1.0)
        self._ka_thread = None

    def _keepalive_loop(self, period_s: float) -> None:
        # Always plain "+": after !GO the relay is a transparent pipe, and
        # direct connections were already plain.  The old ">+" relay prefix is
        # no longer used on any code path.
        msg = b"+\n"
        while not self._ka_stop.wait(period_s):
            try:
                if not self.is_open:
                    break
                with self._write_lock:
                    if self._ser is not None:
                        self._ser.write(msg)
                        self._ser.flush()
            except Exception:
                break   # port closed / gone — let the robot safety-stop

    def send(self, message: str, read_ms: int = 500, stop_token: str | None = "OK") -> dict[str, Any]:
        """Send a plain command, read and return responses.

        Appends a ``#<corr_id>`` suffix to the command so the reader thread
        can route the reply to this call's private queue.  Blocks on that
        queue until a reply arrives or ``read_ms + 500 ms`` timeout elapses.

        All commands are sent **plain** (no ``>`` prefix).  After the
        HELLO-classify / !GO handshake the relay is a transparent byte pipe,
        so no prefix is needed.  Direct robot connections were always plain.

        No ``reset_input_buffer()`` is called — the reader thread is the sole
        owner of the input side of the port.

        Args:
            message: Command string to send (without newline).
            read_ms: Maximum time to wait for the primary reply, in
                milliseconds.  An extra 500 ms grace is added for queue
                blocking to account for in-flight bytes.
            stop_token: If set, return as soon as a line containing this
                substring is received.  Defaults to ``"OK"`` so blocking
                sends return early on the v2 OK response.  Pass ``None`` to
                always drain for the full ``read_ms`` window.
        """
        if not self.is_open:
            return {"error": "Not connected. Call connect first."}

        # Assign a unique corr-id for this send().
        with self._reply_lock:
            self._corr_counter += 1
            corr_id = str(self._corr_counter)
            reply_q: queue.Queue = queue.Queue()
            self._reply_queues[corr_id] = reply_q

        # Build plain command with corr-id suffix.
        corr_suffix = f" #{corr_id}"
        cmd = f"{message}{corr_suffix}\n"

        if self.on_send:
            self.on_send(cmd.rstrip())

        try:
            with self._write_lock:
                self._ser.write(cmd.encode("utf-8"))
                self._ser.flush()
        except Exception as exc:
            with self._reply_lock:
                self._reply_queues.pop(corr_id, None)
            return {"error": str(exc), "sent": message}

        # Drain reply queue until stop_token matched or deadline.
        timeout_s = (read_ms / 1000.0) + 0.5
        lines: list[str] = []
        deadline = time.time() + timeout_s
        try:
            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                try:
                    line = reply_q.get(timeout=min(remaining, 0.05))
                except queue.Empty:
                    continue
                lines.append(line)
                if stop_token and stop_token in line:
                    break
        finally:
            with self._reply_lock:
                self._reply_queues.pop(corr_id, None)

        return {"sent": message, "mode": self._mode, "responses": lines}

    def send_fast(self, message: str) -> None:
        """Fire-and-forget: send plain command, no response reading.

        Always plain (no ``>`` prefix) — after !GO the relay is transparent.
        """
        if not self.is_open:
            raise ConnectionError("Not connected. Call connect first.")
        cmd = f"{message}\n"

        if self.on_send:
            self.on_send(cmd.rstrip())
        with self._write_lock:
            self._ser.write(cmd.encode("utf-8"))
            self._ser.flush()

    def read_lines(self, duration_ms: int = 500, stop_token: str | None = None) -> list[str]:
        """Read lines from the TLM and EVT queues within the given duration.

        Drains ``_tlm_queue`` and ``_evt_queue`` — does NOT call
        ``_ser.readline()``.  The reader thread feeds these queues.

        Falls back to a direct ``_ser.readline()`` path if the reader thread
        is not running (e.g. during ``_poll_ready``).

        Args:
            duration_ms: Maximum time to read for, in milliseconds (ceiling).
            stop_token: If set, return immediately after the first line that
                contains this substring is received.  Uses a plain substring
                check (``token in line``) so relay-prefix noise does not
                prevent matching.  When ``None`` (default), the loop always
                runs until the deadline.

        Returns:
            List of decoded, stripped response lines.
        """
        if not self.is_open:
            return []

        # If the reader thread is not running, fall back to direct _ser reads
        # (used by _poll_ready via _poll_read_lines; this branch is a safety
        # net for callers that connect without ping, etc.).
        if self._reader_thread is None or not self._reader_thread.is_alive():
            return self._poll_read_lines(duration_ms, stop_token=stop_token)

        lines: list[str] = []
        deadline = time.time() + (duration_ms / 1000.0)
        _sleep = 0.005  # 5 ms between drain attempts

        while time.time() < deadline:
            # Drain both queues in one pass.
            drained_this_pass = False
            for q in (self._tlm_queue, self._evt_queue):
                while True:
                    try:
                        line = q.get_nowait()
                    except queue.Empty:
                        break
                    lines.append(line)
                    drained_this_pass = True
                    if stop_token and stop_token in line:
                        return lines

            if not drained_this_pass:
                time.sleep(_sleep)

        return lines

    def read_pending_lines(self) -> list[str]:
        """Non-blocking drain of the TLM and EVT queues.

        Returns immediately with whatever is currently queued (may be empty).
        Does not block, does not touch ``_ser``.

        This is a named replacement for the ``_conn._ser.in_waiting`` peek
        pattern used in ``protocol.py`` before this sprint.  It has identical
        semantics (non-blocking drain) without exposing the internal serial
        object.

        Returns:
            List of all currently-queued TLM and EVT lines.
        """
        lines: list[str] = []
        for q in (self._tlm_queue, self._evt_queue):
            while True:
                try:
                    lines.append(q.get_nowait())
                except queue.Empty:
                    break
        return lines

    def handshake(self, line: bytes) -> None:
        """Write a raw line to the serial port, no relay prefix, no corr-id.

        Intended for the device-detection phase in ``cli.py`` — specifically
        the HELLO probe that identifies the relay/robot before ``connect()``
        is called.  This method is valid **only before the reader thread
        starts** (i.e. before ``connect()`` returns).  Calling it after the
        reader thread is running bypasses the demux layer and may cause the
        reader to discard the reply.

        Args:
            line: Raw bytes to write, including the trailing newline (e.g.
                ``b"HELLO\\n"``).
        """
        if not self.is_open:
            raise ConnectionError("Not connected. Call connect first.")
        with self._write_lock:
            self._ser.write(line)
            self._ser.flush()


def list_serial_ports() -> list[str]:
    """List USB modem serial ports."""
    return sorted(glob.glob("/dev/cu.usbmodem*"))


def probe_devices(read_ms: int = 1200) -> list[dict[str, Any]]:
    """Probe each USB modem port by sending PING (v2 protocol).

    Returns a list of dicts with port, lines, and a 'responsive' flag.
    """
    results = []
    for port in list_serial_ports():
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=READ_TIMEOUT_S)
            time.sleep(0.25)
            ser.reset_input_buffer()
            # Try relay mode first (most common deployment).
            ser.write(b">PING\n")
            ser.flush()
            lines: list[str] = []
            deadline = time.time() + (read_ms / 1000.0)
            while time.time() < deadline:
                raw = ser.readline()
                if not raw:
                    continue
                text = raw.decode("utf-8", "ignore").strip()
                if text:
                    lines.append(text)
            ser.close()
            responsive = any("OK pong" in ln or "OK " in ln for ln in lines)
            results.append({"port": port, "lines": lines, "responsive": responsive})
        except Exception as exc:
            results.append({"port": port, "error": str(exc)})
    return results
