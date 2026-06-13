#!/usr/bin/env python3
"""enc_balance_test.py — measure LEFT vs RIGHT encoder counts under EQUAL wheel commands.

Talks to the robot through its USB serial port DIRECTLY (not via the relay).
DBG replies (ForceReply::SERIAL) and TLM replies are routed to the robot's own
USB serial port — over the relay they are invisible. This script uses
SerialConnection(port, mode="direct") so all replies are correctly received.

Connect the robot's USB port (NEZHA2 device, e.g. /dev/cu.usbmodem2121102) to
the host — NOT the relay (RADIOBRIDGE, e.g. /dev/cu.usbmodem2121402).

The firmware D command is:  D <leftSpeed> <rightSpeed> <distanceMm>
(confirmed in source/app/MotionCommandHandlers.cpp parseD: tokens[0]=left, tokens[1]=right).
So an HONEST encoder-balance test must command EQUAL speeds, e.g. `D 200 200 300`
(both wheels 200 mm/s, drive 300 mm). On a healthy robot encL ~= encR.

This drives several equal-wheel moves and prints the L/R encoder split, flagging
any right-side under-count or `EVT enc_wedged`. It tells you plainly whether the
right encoder genuinely under-counts, or whether an earlier "under-count" was just
an artifact of commanding UNEQUAL wheel speeds.

Setup: robot on a stand, robot USB port (NEZHA2) plugged in directly.
Run:   uv run python tests/bench/enc_balance_test.py
       (override port with: --port /dev/cu.usbmodemXXXX)

Comms: uses SerialConnection(port, mode="direct") — plain commands with corr-id
suffix, no `>` relay prefix, no `!GO`. Replies are under the "responses" key.
TLM frames from SNAP are delivered via conn.read_lines() (not conn.send() responses),
because the firmware emits a raw TLM line (not OK-wrapped) for SNAP.
"""
from __future__ import annotations
import argparse
import pathlib
import re
import sys
import time

_REPO = pathlib.Path(__file__).resolve().parents[2]

# Robot USB serial port (NEZHA2 device) — not the relay (RADIOBRIDGE).
_DEFAULT_PORT = "/dev/cu.usbmodem2121102"


def _robot_port_from_config() -> str | None:
    """Auto-detect robot USB port from config/devices.json."""
    import json
    reg = _REPO / "config" / "devices.json"
    if reg.exists():
        for entry in json.loads(reg.read_text()).values():
            role = (entry.get("role") or "").upper()
            if role in ("NEZHA2", "ROBOT") and entry.get("port"):
                return entry["port"]
    return None


def _make_connection(port: str):
    """Open a SerialConnection in direct mode to the robot's USB serial port."""
    sys.path.insert(0, str(_REPO / "host"))
    from robot_radio.io.serial_conn import SerialConnection
    conn = SerialConnection(port, mode="direct")
    result = conn.connect()
    if "error" in result:
        raise RuntimeError(f"Could not connect to {port}: {result['error']}")
    return conn


def _tx(conn, cmd: str, read_ms: int = 400) -> str:
    """Send a command and return the concatenated response text."""
    result = conn.send(cmd, read_ms=read_ms, stop_token="OK")
    return " ".join(result.get("responses", []))


def _read_enc(conn) -> tuple[int, int] | None:
    """Poll SNAP up to 6 times and return the first enc=(L,R) tuple found."""
    for _ in range(6):
        conn.send_fast("SNAP")
        lines = conn.read_lines(350, stop_token="TLM")
        for ln in lines:
            m = re.search(r"enc=(-?\d+),(-?\d+)", ln)
            if m:
                return int(m.group(1)), int(m.group(2))
    return None


def main():
    ap = argparse.ArgumentParser(
        description="Equal-wheel encoder balance test. "
                    "Requires the robot's USB serial port (NEZHA2 device), NOT the relay.")
    ap.add_argument("--port", default=None,
                    help="Robot USB serial port (NEZHA2 device, e.g. /dev/cu.usbmodem2121102). "
                         "Defaults to auto-detect from config/devices.json, "
                         f"then falls back to {_DEFAULT_PORT}.")
    ap.add_argument("--speed", type=int, default=200, help="equal wheel speed mm/s")
    ap.add_argument("--dist", type=int, default=300, help="drive distance mm")
    ap.add_argument("--trials", type=int, default=6)
    args = ap.parse_args()

    port = args.port or _robot_port_from_config() or _DEFAULT_PORT
    print(f"Connecting to robot USB serial: {port}  (mode=direct, NOT relay)")

    conn = _make_connection(port)

    # Confirm robot is alive.
    pong = _tx(conn, "PING", read_ms=600)
    if "pong" not in pong.lower():
        print("Robot not responding. Is it powered on and the robot USB plugged in?")
        conn.disconnect()
        return 2

    print(f"EQUAL-wheel drives: D {args.speed} {args.speed} {args.dist}  "
          f"(both wheels {args.speed} mm/s, {args.dist} mm). Healthy: encL ~= encR.\n")
    print(f"{'trial':>5} {'encL':>7} {'encR':>7} {'R/L':>6}  note")
    rows = []
    for i in range(args.trials):
        _tx(conn, "ZERO enc", read_ms=300)
        evt = _tx(conn, f"D {args.speed} {args.speed} {args.dist}", read_ms=300)
        time.sleep(args.dist / args.speed + 1.6)     # wait for the move to finish
        e = _read_enc(conn)
        note = "EVT enc_wedged" if "enc_wedged" in evt.lower() else ""
        if e:
            el, er = e
            ratio = (er / el) if el else 0.0
            rows.append((el, er, ratio))
            # The finding-under-test is asymmetry: encR much less than encL.
            imbalanced = el != 0 and not (0.80 <= ratio <= 1.25)
            if imbalanced:
                flag = "  <-- R UNDER-COUNTS (asymmetric)"
            elif abs(el) < 50:
                flag = "  (low travel — see note below)"
            else:
                flag = ""
            print(f"{i+1:>5} {el:>7} {er:>7} {ratio:>6.2f}  {note}{flag}")
        else:
            print(f"{i+1:>5} {'?':>7} {'?':>7}        no telemetry")

    _tx(conn, "X", read_ms=300)
    conn.disconnect()

    print()
    moved = [r for r in rows if abs(r[0]) >= 8]
    bad = [r for r in moved if not (0.80 <= r[2] <= 1.25)]
    if moved and len(bad) >= max(1, len(moved) // 2):
        print(f"RESULT: right-encoder under-count DEMONSTRATED — encR << encL on "
              f"{len(bad)}/{len(moved)} EQUAL-wheel drives.")
        return 0
    print(f"RESULT: encoders BALANCED — encR ~= encL (ratio in 0.80-1.25) on "
          f"{len(moved) - len(bad)}/{len(moved)} equal-wheel drives. The right encoder does NOT "
          f"under-count under EQUAL commands; the earlier 'under-count' was an artifact of\n"
          f"        commanding UNEQUAL wheel speeds (D is `D <left> <right> <dist>`, so e.g. "
          f"`D 250 150 150` drives left 250 / right 150 — left SHOULD count more).")
    if moved and all(abs(r[0]) < 50 for r in moved):
        print("NOTE: absolute travel is low (~20 mm for a much larger commanded distance) on every "
              "drive — likely a drained motor battery after a long session (drove ~500 mm when "
              "fresh). Recharge and re-run to rule out a drive-distance issue.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
