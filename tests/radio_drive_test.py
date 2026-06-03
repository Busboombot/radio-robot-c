#!/usr/bin/env python3
"""radio_drive_test.py — exercise the robot over the RADIORELAY (RAW250).

Opens the relay's USB serial port, configures it (RAW250, channel 0, group 10),
enters the transparent data plane (`!GO`), then drives the robot with protocol-v2
commands over the radio and checks the encoders before/after each move.

The robot drives on the floor — keep the area clear. Moves are modest
(≤300 mm legs). The script always sends STOP on exit (including Ctrl-C / error).

Run:
    uv run tests/radio_drive_test.py
    uv run tests/radio_drive_test.py --port /dev/cu.usbmodem21421302
    uv run tests/radio_drive_test.py --dry        # config + PING/ID only, no motion

The relay is found automatically from config/devices.json (role RADIOBRIDGE);
override with --port. One serial session only: the relay's data plane has no
escape except closing the port, which this script does at the end.
"""

import argparse
import json
import re
import sys
import time
from pathlib import Path

import serial

BAUD = 115200
TLM_ENC_RE = re.compile(r"enc=(-?\d+),(-?\d+)")
TLM_POSE_RE = re.compile(r"pose=(-?\d+),(-?\d+),(-?\d+)")


# --------------------------------------------------------------------------- #
# Relay link
# --------------------------------------------------------------------------- #
def find_relay_port() -> str:
    reg = Path(__file__).resolve().parent.parent / "config" / "devices.json"
    if reg.exists():
        for entry in json.loads(reg.read_text()).values():
            if (entry.get("role") or "").upper() == "RADIOBRIDGE" and entry.get("port"):
                return entry["port"]
    sys.exit("Could not find a RADIOBRIDGE relay in config/devices.json — pass --port.")


class Relay:
    """Drives the relay's command plane, then the transparent data plane."""

    def __init__(self, port: str):
        print(f"[relay] opening {port} (resets relay → command plane)…")
        self._s = serial.Serial(port, BAUD, timeout=0.2)
        time.sleep(2.0)  # DTR reset + boot into command plane
        self._s.reset_input_buffer()

    def _line(self, text: str, wait: float = 0.4) -> str:
        self._s.write((text + "\n").encode())
        self._s.flush()
        time.sleep(wait)
        return self._s.read(8192).decode(errors="replace")

    def configure(self):
        banner = self._line("HELLO")
        print(f"[relay] {banner.strip()}")
        if "RADIOBRIDGE" not in banner:
            print("[relay] WARNING: no RADIOBRIDGE banner — is this the relay port?")
        self._line("!MODE RAW250")
        self._line("!CG 0 10")          # channel 0, group 10 — matches robot Radio.cpp
        self._line("!P 7")
        cfg = self._line("?")
        print(f"[relay] {cfg.strip()}")

    def go(self):
        resp = self._line("!GO", wait=0.8)
        print(f"[relay] {resp.strip()}  (data plane — transparent)")
        self._s.reset_input_buffer()

    # ---- data plane (transparent v2 to the robot over radio) ----
    def send(self, text: str):
        self._s.write((text + "\n").encode())
        self._s.flush()

    def request(self, text: str, want: str, timeout: float = 3.0) -> str | None:
        """Send a v2 command; return the first reply line starting with `want`."""
        self._s.reset_input_buffer()
        self.send(text)
        return self._read_until(want, timeout)

    def _read_until(self, want: str, timeout: float) -> str | None:
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline:
            buf += self._s.read(4096)
            for line in buf.replace(b"\r", b"").split(b"\n"):
                if line.decode(errors="replace").startswith(want):
                    return line.decode(errors="replace")
            time.sleep(0.05)
        return None

    def stop(self):
        try:
            self.send("STOP")
            time.sleep(0.3)
        except Exception:
            pass

    def close(self):
        try:
            self._s.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# Telemetry helpers
# --------------------------------------------------------------------------- #
def snap_enc(relay: Relay) -> tuple[int, int] | None:
    """One-shot SNAP → parse enc=L,R from the TLM frame."""
    line = relay.request("SNAP", "TLM", timeout=2.5)
    if not line:
        return None
    m = TLM_ENC_RE.search(line)
    return (int(m.group(1)), int(m.group(2))) if m else None


def wait_done(relay: Relay, verb: str, timeout: float) -> bool:
    """Wait for `EVT done <verb>` (T/D/G self-complete)."""
    line = relay._read_until(f"EVT done {verb}", timeout)
    return line is not None


# --------------------------------------------------------------------------- #
# Test sequence
# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="relay serial port (default: from devices.json)")
    ap.add_argument("--dry", action="store_true", help="link check only, no motion")
    ap.add_argument("--speed", type=int, default=180, help="drive speed mm/s")
    args = ap.parse_args()

    port = args.port or find_relay_port()
    relay = Relay(port)
    results: list[tuple[str, str]] = []

    try:
        relay.configure()
        relay.go()

        # --- link check ---
        pong = relay.request("PING", "OK pong", timeout=3.0)
        print(f"[robot] PING -> {pong}")
        if not pong:
            print("!! No PING reply over the radio. Check relay config / robot power / range.")
            return 1
        ident = relay.request("ID", "ID ", timeout=3.0)
        print(f"[robot] ID   -> {ident}")

        if args.dry:
            print("\n[dry] link OK — skipping motion.")
            return 0

        # --- zero + baseline ---
        relay.request("ZERO enc", "OK", timeout=2.0)
        relay.request("ZERO pose", "OK", timeout=2.0)
        base = snap_enc(relay)
        print(f"\n[enc] baseline: {base}")

        def leg(label, command, verb, expect, timeout=8.0):
            print(f"\n>>> {label}: {command}")
            before = snap_enc(relay)
            relay.request(command, "OK", timeout=2.0)
            done = wait_done(relay, verb, timeout) if verb else True
            time.sleep(0.3)
            after = snap_enc(relay)
            dl = dr = None
            if before and after:
                dl, dr = after[0] - before[0], after[1] - before[1]
            print(f"    done={done}  enc {before} -> {after}   ΔL={dl} ΔR={dr}   (expect: {expect})")
            results.append((label, f"ΔL={dl} ΔR={dr} done={done}"))

        sp = args.speed
        # Forward ~300 mm
        leg(f"forward 300mm @ {sp}", f"D {sp} {sp} 300", "D", "both Δ ≈ +300")
        # Spin in place ~0.7 s (left fwd, right back)
        leg("spin-left ~0.7s", f"T {sp} {-sp} 700", "T", "ΔL>0, ΔR<0")
        # Forward ~200 mm
        leg(f"forward 200mm @ {sp}", f"D {sp} {sp} 200", "D", "both Δ ≈ +200")
        # Spin back the other way
        leg("spin-right ~0.7s", f"T {-sp} {sp} 700", "T", "ΔL<0, ΔR>0")

        final = snap_enc(relay)
        print(f"\n[enc] final cumulative: {final}")

        print("\n==================== SUMMARY ====================")
        for label, r in results:
            print(f"  {label:24} {r}")
        print("=================================================")
        print("Eyeball it: forward legs → both encoders climb together;")
        print("spins → encoders split (opposite signs). If a wheel's Δ is ~0,")
        print("that motor/encoder isn't responding.")
        return 0

    except KeyboardInterrupt:
        print("\n[interrupted] stopping robot.")
        return 130
    finally:
        relay.stop()
        relay.close()


if __name__ == "__main__":
    sys.exit(main())
