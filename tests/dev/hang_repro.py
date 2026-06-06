#!/usr/bin/env python3
"""hang_repro.py — try to deterministically reproduce the firmware hard-hang.

Hypothesis (ticket 015-005): with TLM streaming enabled, if the host stops
reading the serial port, the firmware's 255-byte TX buffer fills and the
cooperative loop blocks on the serial write — hanging the firmware until a
power cycle. The ASYNC-TX fix (SerialPort::send) should prevent this by
dropping frames instead of blocking.

This test forces the condition and reports whether the robot survives.

Modes:
  stall      (default) — enable STREAM, drive, then STOP READING the port for
             --stall seconds (simulating a frozen GUI / dead reader) while the
             firmware keeps streaming. Then resume and PING. If the robot does
             not answer PING afterward, the firmware HUNG -> bug reproduced.
  endurance  — drive + stream continuously for --secs, reading normally, and
             report frame flow + final liveness (tests normal long-run use).

Usage:
  uv run python tests/dev/hang_repro.py [--mode stall|endurance] [--stall S]
                                        [--secs S] [--speed MMPS] [--stream-ms MS]
                                        [--drive]
"""

import argparse
import sys
import time
import serial


def parse_args():
    p = argparse.ArgumentParser(description="Reproduce / rule out the firmware hard-hang")
    p.add_argument("--port", default="/dev/cu.usbmodem2121102")
    p.add_argument("--mode", choices=["stall", "endurance"], default="stall")
    p.add_argument("--secs", type=float, default=60.0, help="endurance duration")
    p.add_argument("--stall", type=float, default=10.0,
                   help="stall mode: seconds to STOP reading while STREAM is on")
    p.add_argument("--speed", type=int, default=200)
    p.add_argument("--stream-ms", type=int, default=40, help="TLM period (lower = heavier)")
    p.add_argument("--drive", action="store_true",
                   help="also drive the wheels (S keepalive) during the test")
    return p.parse_args()


def open_port(port):
    s = serial.Serial(port, 115200, timeout=0.05, dsrdtr=False, rtscts=False)
    s.dtr = False
    s.rts = False
    return s


def wait_alive(s, secs=14):
    t = time.time()
    while time.time() - t < secs:
        s.write(b"PING\n"); s.flush()
        d = time.time()
        while time.time() - d < 0.5:
            if "pong" in s.readline().decode(errors="replace"):
                return True
    return False


def ping_alive(s, secs=3):
    s.reset_input_buffer()
    t = time.time()
    while time.time() - t < secs:
        s.write(b"PING\n"); s.flush()
        d = time.time()
        while time.time() - d < 0.5:
            if "pong" in s.readline().decode(errors="replace"):
                return True
    return False


def main():
    a = parse_args()
    s = open_port(a.port)
    print(f"mode={a.mode} stream={a.stream_ms}ms drive={a.drive} speed={a.speed}")
    print("waiting for robot...")
    if not wait_alive(s):
        print("ERROR: robot not responding at start (already hung?)"); s.close(); return 1
    print("robot alive. starting.")
    s.reset_input_buffer()

    def w(c): s.write((c + "\n").encode()); s.flush()

    w("SET sTimeout=10000"); time.sleep(0.2)
    w(f"STREAM {a.stream_ms}"); time.sleep(0.1)
    if a.drive:
        w(f"S {a.speed} {a.speed}")
    last_ka = time.time()

    if a.mode == "stall":
        # Read normally for 2 s to confirm flow.
        t0 = time.time(); n = 0
        while time.time() - t0 < 2.0:
            if s.readline().strip(): n += 1
            if a.drive and time.time() - last_ka >= 0.15:
                w(f"S {a.speed} {a.speed}"); last_ka = time.time()
        print(f"pre-stall: {n} lines in 2 s (flowing)")

        # THE STALL: stop reading entirely for --stall seconds while STREAM is on.
        # The firmware keeps emitting TLM into a TX buffer nobody drains.
        print(f"STALL: not reading for {a.stall} s (firmware streaming into a full buffer)...")
        time.sleep(a.stall)   # host does nothing — no reads, no keepalive

        # Did the firmware survive? Drain whatever is buffered, then PING.
        s.reset_input_buffer()
        alive = ping_alive(s, secs=4)
        print()
        print("=" * 56)
        if alive:
            print("RESULT: robot ALIVE after stall — hang NOT reproduced (ASYNC TX ok)")
        else:
            print("RESULT: robot HUNG after stall — BUG REPRODUCED (needs power cycle)")
        print("=" * 56)

    else:  # endurance
        t0 = time.time(); n = 0; last = time.time(); maxgap = 0.0; gaps = []
        while time.time() - t0 < a.secs:
            ln = s.readline().decode(errors="replace").strip()
            if ln:
                now = time.time(); g = now - last; last = now
                if n > 5 and g > maxgap: maxgap = g
                if g > 2.0: gaps.append(round(time.time() - t0, 1))
                n += 1
            if a.drive and time.time() - last_ka >= 0.15:
                w(f"S {a.speed} {a.speed}"); last_ka = time.time()
        alive = ping_alive(s, secs=4)
        print()
        print("=" * 56)
        print(f"endurance {a.secs:.0f}s: {n} lines, max gap {maxgap:.2f}s, "
              f">2s gaps at {gaps}")
        print(f"RESULT: robot {'ALIVE' if alive else 'HUNG'} after {a.secs:.0f}s "
              f"of {'drive+' if a.drive else ''}stream")
        print("=" * 56)

    for _ in range(3):
        w("STOP"); time.sleep(0.05)
    w("STREAM 0")
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
