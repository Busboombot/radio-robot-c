#!/usr/bin/env python3
"""device_link — join a micro:bit's MSD volume to its CDC type announcement.

This is the Python mirror of the join described in ``docs/DEVICE_LINKING.md``
(whose reference implementation, ``scripts/lib/device-link.js``, lives in the
TypeScript repo). It exists so the deploy tooling can prove that the volume it
is about to flash belongs to the robot — and refuse to overwrite the radio
relay.

The join key is the **USB serial number**, which is shared across every USB
interface a micro:bit exposes:

    MSD volume   --(DETAILS.TXT "Unique ID")-->  USB serial
    USB serial   --(ioreg IOCalloutDevice)----->  /dev/cu.* port
    /dev/cu.*    --(HELLO -> "DEVICE:" line)----->  device type

Match all three and the type you read and the volume you flash are guaranteed
to be the same physical board.

macOS only for the port<->serial step (it uses ``ioreg``). Off macOS,
``port_serial_map`` returns ``{}`` so volumes degrade to "type unknown" rather
than being mis-attributed.
"""

from __future__ import annotations

import glob
import re
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

try:  # serial is only needed for the (optional) type probe
    import serial  # type: ignore
except Exception:  # pragma: no cover - serial may be absent in some envs
    serial = None  # type: ignore

BAUD_RATE = 115200

# This firmware announces ``DEVICE:Nezha2:<name>:microbit:<serial>`` (see
# source/app/Announcer.cpp). Auto-detect targets this role positively; any
# other non-relay device (e.g. a SPEEDTEST rig) is treated as ambiguous rather
# than auto-flashed.
ROBOT_ROLES = {"nezha2"}

# A micro:bit DAPLink USB serial is the long form (== DETAILS.TXT "Unique ID").
_MICROBIT_SERIAL_RE = re.compile(r"^[0-9a-fA-F]{40,52}$")
_IOREG_SERIAL_RE = re.compile(r'"USB Serial Number"\s*=\s*"([^"]+)"')
_IOREG_CALLOUT_RE = re.compile(r'"IOCalloutDevice"\s*=\s*"([^"]+)"')
_DETAILS_UNIQUE_ID_RE = re.compile(r"Unique ID:\s*([0-9a-fA-F]+)", re.IGNORECASE)


@dataclass
class Device:
    """One micro:bit, with whatever faces we could join on its USB serial."""

    serial: str | None = None          # USB serial (the join key)
    volume: Path | None = None         # /Volumes/MICROBIT*
    port: str | None = None            # /dev/cu.usbmodem*
    role: str | None = None            # DEVICE: type field, e.g. Nezha2 / RADIOBRIDGE
    common_name: str | None = None     # DEVICE: name field
    announcement: str | None = None    # raw DEVICE: line

    @property
    def is_relay(self) -> bool:
        return is_relay(self.role)

    @property
    def is_robot(self) -> bool:
        return is_robot(self.role)

    @property
    def type_known(self) -> bool:
        return self.role is not None


def is_relay(role: str | None) -> bool:
    """True if a DEVICE: role/type names a radio relay/bridge.

    Matches both the documented ``RADIORELAY`` and the firmware's actual
    ``RADIOBRIDGE`` (announce line ``DEVICE:RADIOBRIDGE:relay:zavaz:...``) by
    looking for either token, case-insensitively.
    """
    if not role:
        return False
    r = role.upper()
    return "RELAY" in r or "BRIDGE" in r


def is_robot(role: str | None) -> bool:
    """True if a DEVICE: role/type names this project's robot firmware."""
    if not role:
        return False
    return role.lower() in ROBOT_ROLES


# ---------------------------------------------------------------------------
# Individual enumeration sources
# ---------------------------------------------------------------------------

def microbit_volumes() -> dict[str, Path]:
    """Map USB serial -> mounted MICROBIT volume via DETAILS.TXT 'Unique ID'."""
    out: dict[str, Path] = {}
    for vol in glob.glob("/Volumes/MICROBIT*"):
        details = Path(vol) / "DETAILS.TXT"
        try:
            text = details.read_text(errors="ignore")
        except OSError:
            continue
        m = _DETAILS_UNIQUE_ID_RE.search(text)
        if m:
            out[m.group(1)] = Path(vol)
    return out


def port_serial_map(known_serials: set[str] | None = None) -> dict[str, str]:
    """Map USB serial -> /dev/cu.* port by parsing ``ioreg`` (macOS only).

    When ``known_serials`` is given, only those serials are recorded, so a
    non-micro:bit serial port can never be mis-attributed to a micro:bit
    (the safeguard called out in DEVICE_LINKING.md). Returns ``{}`` off macOS
    or if ``ioreg`` is unavailable.
    """
    try:
        proc = subprocess.run(
            ["ioreg", "-r", "-c", "IOUSBHostDevice", "-l"],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return {}
    if proc.returncode != 0:
        return {}

    out: dict[str, str] = {}
    current_serial: str | None = None
    for line in proc.stdout.splitlines():
        sm = _IOREG_SERIAL_RE.search(line)
        if sm:
            current_serial = sm.group(1)
            continue
        cm = _IOREG_CALLOUT_RE.search(line)
        if cm and current_serial:
            if known_serials is not None and current_serial not in known_serials:
                continue
            # IOCalloutDevice nests under its USB device, so the most recent
            # serial above it is this port's serial. First callout wins.
            out.setdefault(current_serial, cm.group(1))
    return out


def probe_type(port: str, timeout_s: float = 1.6) -> dict[str, str] | None:
    """Open ``port``, send HELLO, and parse the ``DEVICE:`` announcement.

    Returns ``{role, common_name, device_name, serial, raw}`` or ``None`` if
    no announcement arrived (port busy, no firmware, or timed out). Best-effort
    by design — see the "TYPE column is best-effort" note in DEVICE_LINKING.md.
    """
    if serial is None:
        return None
    ser = None
    try:
        ser = serial.Serial(baudrate=BAUD_RATE, timeout=0.12, dsrdtr=False, rtscts=False)
        ser.port = port
        ser.dtr = False
        ser.rts = False
        ser.open()
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(b"HELLO\n")
        ser.flush()
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", "ignore").strip()
            if text.startswith("DEVICE:"):
                parts = text.split(":")
                if len(parts) >= 5:
                    return {
                        "role": parts[1],
                        "common_name": parts[2],
                        "device_name": parts[3],
                        "serial": ":".join(parts[4:]),
                        "raw": text,
                    }
        return None
    except Exception:
        return None
    finally:
        if ser is not None and ser.is_open:
            ser.close()


# ---------------------------------------------------------------------------
# The join
# ---------------------------------------------------------------------------

def enumerate_devices(announce: bool = True) -> list[Device]:
    """Every mounted micro:bit, with volume/port/type joined on USB serial."""
    volumes = microbit_volumes()
    ports = port_serial_map(set(volumes))

    devices: list[Device] = []
    for ser_no, vol in volumes.items():
        dev = Device(serial=ser_no, volume=vol, port=ports.get(ser_no))
        if announce and dev.port:
            info = probe_type(dev.port)
            if info:
                dev.role = info["role"]
                dev.common_name = info["common_name"]
                dev.announcement = info["raw"]
        devices.append(dev)
    return devices


def classify_volume(volume: Path, announce: bool = True) -> Device:
    """Resolve a single MICROBIT volume into a Device (serial/port/type)."""
    volume = Path(volume)
    for dev in enumerate_devices(announce=announce):
        if dev.volume == volume:
            return dev
    # Volume not found among MICROBIT mounts (e.g. a custom --usb-mount path):
    # return a bare Device so callers can still decide what to do.
    return Device(volume=volume)
