"""Serial connection management for micro:bit relay/direct devices."""

import glob
import time
from typing import Any

import serial

BAUD_RATE = 115200
DEFAULT_PORT = "/dev/cu.usbmodem21431202"
READ_TIMEOUT_S = 0.12

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


class SerialConnection:
    """Manages a serial connection to a micro:bit relay or direct device."""

    def __init__(self, port: str = DEFAULT_PORT, baud: int = BAUD_RATE,
                 mode: str | None = None, on_send=None):
        self._port = port
        self._baud = baud
        self._mode = mode  # None = auto-detect from announcement
        self._ser: serial.Serial | None = None
        self.on_send = on_send  # callback(cmd_str) for verbose logging

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def port(self) -> str | None:
        return self._port if self.is_open else None

    @property
    def mode(self) -> str | None:
        return self._mode

    def connect(self, skip_ping: bool = False) -> dict[str, Any]:
        """Open port, send PING (v2), confirm readiness.

        After opening the serial port the device is not immediately ready —
        the first command's reply is reliably lost if we simply sleep.  Both
        paths use an active readiness poll: repeatedly send PING with a short
        per-attempt read window until a valid response arrives or the total
        timeout expires.

        In relay mode (self._mode == "relay"), the relay is transparent and
        the PING is forwarded to the robot.  The relay self-identifies via its
        own messages (e.g. "RX:" or "TX:" prefixes); we do not parse those here.

        If ``self._mode`` is None on entry, it defaults to "relay" (the normal
        deployment: host -> relay -> robot).  Set ``mode="direct"`` on
        construction for a direct USB connection.

        Args:
            skip_ping: When True (cache-hit fast path), skip the readiness poll
                and use the cached ``self._mode``.  The return dict will have
                ``lines=[]`` and ``pinged=False``.
        """
        if self.is_open:
            if self._ser.port == self._port:
                return {"status": "already_connected", "port": self._port, "mode": self._mode}
            self._ser.close()

        try:
            # Open the port WITHOUT toggling DTR.  On macOS the default
            # pyserial behaviour pulses DTR low on open() and again on close(),
            # which the micro:bit's DAPLink interface interprets as a target
            # reset request.  Opening with dsrdtr=False and explicitly holding
            # DTR/RTS at their current level avoids resetting the chip every
            # time a CLI invocation connects or exits.
            self._ser = serial.Serial(baudrate=self._baud, timeout=READ_TIMEOUT_S,
                                      dsrdtr=False, rtscts=False)
            self._ser.port = self._port
            self._ser.dtr = False
            self._ser.rts = False
            self._ser.open()

            # Default mode to relay if not set.
            if self._mode is None:
                self._mode = "relay"

            if skip_ping:
                return {
                    "status": "connected",
                    "port": self._port,
                    "mode": self._mode,
                    "lines": [],
                    "pinged": False,
                }

            # Normal path: active readiness poll via PING.
            lines = self._poll_ready(total_timeout_s=_POLL_TOTAL_NORMAL_S)

            return {
                "status": "connected",
                "port": self._port,
                "mode": self._mode,
                "lines": lines,
                "pinged": bool(lines),
            }
        except Exception as exc:
            self._ser = None
            return {"error": str(exc), "port": self._port}

    def _poll_ready(self, total_timeout_s: float = _POLL_TOTAL_NORMAL_S) -> list[str]:
        """Poll PING until the device responds or total_timeout_s is exceeded.

        Sends PING (with relay prefix if in relay mode), reads for
        _POLL_ATTEMPT_MS, and returns immediately if any non-empty response
        is received. Retries until total_timeout_s expires.
        Returns the response lines from the first successful attempt (or []).
        """
        deadline = time.time() + total_timeout_s
        cmd = b">PING\n" if self._mode == "relay" else b"PING\n"
        while time.time() < deadline:
            self._ser.reset_input_buffer()
            self._ser.write(cmd)
            self._ser.flush()
            lines = self.read_lines(_POLL_ATTEMPT_MS, stop_token="OK pong")
            if lines:
                return lines
        return []

    def disconnect(self) -> dict[str, Any]:
        if not self.is_open:
            return {"status": "not_connected"}
        port = self._port
        self._ser.close()
        self._ser = None
        return {"status": "disconnected", "port": port}

    def send(self, message: str, read_ms: int = 500, stop_token: str | None = "OK") -> dict[str, Any]:
        """Send command with mode prefix, read and return responses.

        Args:
            message: Command string to send (without mode prefix or newline).
            read_ms: Maximum time to wait for responses, in milliseconds.
            stop_token: If set, return as soon as a line containing this
                substring is received (deadline is still the ceiling).
                Defaults to ``"OK"`` so blocking sends return early on the v2
                OK response. Pass ``None`` to always drain for the full
                ``read_ms`` window.
        """
        if not self.is_open:
            return {"error": "Not connected. Call connect first."}
        cmd = f">{message}\n" if self._mode == "relay" else f"{message}\n"
        if self.on_send:
            self.on_send(cmd.rstrip())
        self._ser.reset_input_buffer()
        self._ser.write(cmd.encode("utf-8"))
        self._ser.flush()
        lines = self.read_lines(read_ms, stop_token=stop_token)
        return {"sent": message, "mode": self._mode, "responses": lines}

    def send_fast(self, message: str) -> None:
        """Fire-and-forget: send with mode prefix, no response reading."""
        if not self.is_open:
            raise ConnectionError("Not connected. Call connect first.")
        cmd = f">{message}\n" if self._mode == "relay" else f"{message}\n"
        
        if self.on_send:
            self.on_send(cmd.rstrip())
        self._ser.write(cmd.encode("utf-8"))
        self._ser.flush()

    def read_lines(self, duration_ms: int = 500, stop_token: str | None = None) -> list[str]:
        """Read lines from the serial port within the given duration.

        Args:
            duration_ms: Maximum time to read for, in milliseconds (ceiling).
            stop_token: If set, return immediately after the first line that
                contains this substring is received.  Uses a plain substring
                check (``token in line``) so relay-prefix noise (e.g. ``<``)
                does not prevent matching.  When ``None`` (default), the loop
                always runs until the deadline.

        Returns:
            List of decoded, stripped response lines.
        """
        if not self.is_open:
            return []
        lines: list[str] = []
        deadline = time.time() + (duration_ms / 1000.0)
        while time.time() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", "ignore").strip()
            if text:
                lines.append(text)
                if stop_token and stop_token in text:
                    break
        return lines


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
