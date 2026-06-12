"""Low-level serial connection helpers for the standalone calibration scripts.

These classes wrap pyserial directly (relay + transparent data plane or direct
robot USB) for use by ``host/calibrate_angular.py`` and
``host/calibrate_linear.py``.  They do NOT use the higher-level
``SerialConnection`` abstraction because the calibration scripts need very
fine-grained timing control over relay handshake and DTR reset.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

BAUD = 115200
_PROJECT_ROOT = Path(__file__).resolve().parents[4]


def find_relay_port() -> str | None:
    """Find RADIOBRIDGE relay port from config/devices.json."""
    reg = _PROJECT_ROOT / "config" / "devices.json"
    if reg.exists():
        for entry in json.loads(reg.read_text()).values():
            if (entry.get("role") or "").upper() == "RADIOBRIDGE" and entry.get("port"):
                return entry["port"]
    return None


def find_robot_port() -> str | None:
    """Find direct NEZHA2 robot port from config/devices.json."""
    reg = _PROJECT_ROOT / "config" / "devices.json"
    if reg.exists():
        for entry in json.loads(reg.read_text()).values():
            role = (entry.get("role") or "").upper()
            if role in ("NEZHA2", "ROBOT") and entry.get("port"):
                return entry["port"]
    return None


class RelaySerial:
    """Thin wrapper around a pyserial port for relay + transparent data plane."""

    def __init__(self, port: str):
        import serial
        print(f"  Opening relay port {port} ...")
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


class DirectSerial:
    """Thin wrapper for direct robot serial (no relay handshake needed)."""

    def __init__(self, port: str):
        import serial
        print(f"  Opening direct robot port {port} ...")
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


def make_serial_conn(port: str | None, direct: bool):
    """Create and return a connected serial object (RelaySerial or DirectSerial).

    Handles auto-detection via config/devices.json when *port* is None.
    Exits the process on connection failure.
    """
    import sys

    ser = None
    try:
        if port:
            if direct:
                ser = DirectSerial(port)
            else:
                ser = RelaySerial(port)
                ser.configure(); ser.go()
        elif direct:
            p = find_robot_port()
            if p is None:
                print("ERROR: No direct robot port found in config/devices.json.",
                      file=sys.stderr)
                sys.exit(1)
            ser = DirectSerial(p)
        else:
            p = find_relay_port()
            if p is not None:
                ser = RelaySerial(p); ser.configure(); ser.go()
                print("  Connected via relay.")
            else:
                p = find_robot_port()
                if p is None:
                    print("ERROR: No relay or robot port found. "
                          "Pass --port or --direct.", file=sys.stderr)
                    sys.exit(1)
                ser = DirectSerial(p)
                print("  Connected directly to robot.")
    except Exception as e:
        print(f"ERROR: Could not connect: {e}", file=sys.stderr)
        sys.exit(1)

    return ser
