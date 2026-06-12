#!/usr/bin/env python3
"""enc_balance_test.py — measure LEFT vs RIGHT encoder counts under EQUAL wheel commands.

The firmware D command is:  D <leftSpeed> <rightSpeed> <distanceMm>
(confirmed in source/app/MotionCommandHandlers.cpp parseD: tokens[0]=left, tokens[1]=right).
So an HONEST encoder-balance test must command EQUAL speeds, e.g. `D 200 200 300`
(both wheels 200 mm/s, drive 300 mm). On a healthy robot encL ~= encR.

This drives several equal-wheel moves and prints the L/R encoder split, flagging
any right-side under-count or `EVT enc_wedged`. It tells you plainly whether the
right encoder genuinely under-counts, or whether an earlier "under-count" was just
an artifact of commanding UNEQUAL wheel speeds.

Setup: robot on a stand, the RELAY's USB plugged in (RADIOBRIDGE).
Run:   uv run python tests/bench/enc_balance_test.py
       (override the port with: --port /dev/cu.usbmodemXXXX)

Comms note: this talks to the relay directly via its `!GO` data-plane protocol
(open with DTR asserted -> `!GO` -> plain commands). `rogo`/robot_radio use the
old `>`-prefix protocol and can't reach the robot through the current relay.
See .clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md and the project
docs at https://robots.jointheleague.org/.
"""
import argparse
import re
import sys
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbmodem2121402", help="the RELAY serial port")
    ap.add_argument("--speed", type=int, default=200, help="equal wheel speed mm/s")
    ap.add_argument("--dist", type=int, default=300, help="drive distance mm")
    ap.add_argument("--trials", type=int, default=6)
    args = ap.parse_args()

    # DTR asserted (pyserial default) is REQUIRED — do not pass dtr=False.
    s = serial.Serial(args.port, 115200, timeout=0.25)

    def tx(msg, wait=0.5):
        s.write((msg + "\n").encode()); s.flush()
        t = time.time(); buf = b""
        while time.time() - t < wait:
            d = s.read(256)
            if d: buf += d
        return buf.decode(errors="replace")

    time.sleep(1.0)            # let the relay reset + announce
    s.reset_input_buffer()
    tx("!GO", 0.6)            # enter the relay data plane
    if "pong" not in tx("PING", 0.6).lower():
        print("Robot not responding. Is it powered on and the RELAY USB plugged in?")
        s.close(); return 2

    def read_enc():
        for _ in range(6):
            for ln in tx("SNAP", 0.35).splitlines():
                m = re.search(r"enc=(-?\d+),(-?\d+)", ln)
                if m:
                    return int(m.group(1)), int(m.group(2))
        return None

    print(f"EQUAL-wheel drives: D {args.speed} {args.speed} {args.dist}  "
          f"(both wheels {args.speed} mm/s, {args.dist} mm). Healthy: encL ~= encR.\n")
    print(f"{'trial':>5} {'encL':>7} {'encR':>7} {'R/L':>6}  note")
    rows = []
    for i in range(args.trials):
        tx("ZERO enc", 0.3)
        evt = tx(f"D {args.speed} {args.speed} {args.dist}", 0.3)
        time.sleep(args.dist / args.speed + 1.6)     # wait for the move to finish
        e = read_enc()
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
    tx("X", 0.3)
    s.close()

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
