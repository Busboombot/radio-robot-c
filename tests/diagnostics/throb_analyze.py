#!/usr/bin/env python3
"""throb_analyze.py — objective motor-throb metric (no human eyeballing).

Drives the robot at a steady speed on the stand and samples encoder POSITIONS
with the firmware's own t= timestamp (stamped at sensor-read time, so Δpos/Δt is
a valid velocity regardless of radio/polling jitter). Computes per-interval wheel
velocity host-side and reports:

  - mean / stdev / coefficient-of-variation (CV) of velocity   (throb magnitude)
  - % of NON-MONOTONIC position reads on a forward drive        (read reliability)

A steady, well-controlled wheel has low CV (< ~0.15) and ~0% non-monotonic reads.
High CV and/or non-monotonic reads == throbbing / unreliable feedback.

Run (robot on the stand, relay connected):
    uv run python tests/diagnostics/throb_analyze.py [--speed 200] [--ms 3500] [--port DEV]

This is a low-level bench diagnostic: it talks to the RADIORELAY RAW250 data
plane directly (proven path) rather than the library, so it keeps working even
when the higher-level stack is being changed.
"""
import argparse
import re
import statistics
import sys
import time

import serial

DEFAULT_PORT = "/dev/cu.usbmodem21421302"


class Relay:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.2)
        time.sleep(2.0)
        self.s.reset_input_buffer()

    def _cmd(self, line, w=0.4):
        self.s.write((line + "\n").encode()); self.s.flush(); time.sleep(w)
        return self.s.read(8192).decode(errors="replace")

    def go(self):
        self._cmd("HELLO"); self._cmd("!MODE RAW250"); self._cmd("!CG 0 10")
        self._cmd("!P 7"); self._cmd("!GO", 0.8); self.s.reset_input_buffer()

    def send(self, line):
        self.s.write((line + "\n").encode()); self.s.flush()

    def snap(self, to=0.5):
        """SNAP -> (t_ms, encL, encR) from the TLM frame, or None."""
        self.s.reset_input_buffer(); self.send("SNAP")
        deadline = time.time() + to; buf = ""
        while time.time() < deadline:
            buf += self.s.read(4096).decode(errors="replace")
            if "TLM" in buf and "enc=" in buf:
                seg = buf[buf.index("TLM"):]
                mt = re.search(r"t=(\d+)", seg)
                me = re.search(r"enc=(-?\d+),(-?\d+)", seg)
                if mt and me:
                    return (int(mt.group(1)), int(me.group(1)), int(me.group(2)))
            time.sleep(0.01)
        return None

    def close(self):
        try:
            self.send("STOP"); time.sleep(0.3); self.s.close()
        except Exception:
            pass


def _stats(samples, idx, name):
    vs, nonmono = [], 0
    for a, b in zip(samples, samples[1:]):
        dt = (b[0] - a[0]) / 1000.0
        if dt <= 0:
            continue
        dpos = b[idx] - a[idx]
        if dpos < 0:
            nonmono += 1
        vs.append(dpos / dt)
    if len(vs) < 3:
        print(f"  {name}: too few usable intervals ({len(vs)})")
        return None
    mean = statistics.fmean(vs); sd = statistics.pstdev(vs)
    cv = sd / abs(mean) if mean else float("inf")
    pct_nm = 100.0 * nonmono / len(vs)
    print(f"  {name}: n={len(vs)} mean={mean:.0f} mm/s  stdev={sd:.0f}  CV={cv:.2f}  "
          f"band[{min(vs):.0f}..{max(vs):.0f}]  non-monotonic={pct_nm:.0f}%")
    return cv


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=int, default=200, help="drive speed mm/s")
    ap.add_argument("--ms", type=int, default=3500, help="drive duration ms")
    ap.add_argument("--port", default=DEFAULT_PORT, help="relay serial port")
    args = ap.parse_args()

    r = Relay(args.port)
    try:
        r.go()
        if not r.snap():
            print("No TLM from robot — is it powered on / on the relay channel?")
            return 2
        r.send("ZERO enc"); time.sleep(0.3); r.snap()
        print(f"driving T {args.speed} {args.speed} {args.ms} and sampling positions…")
        r.send(f"T {args.speed} {args.speed} {args.ms}")
        samples = []
        end = time.time() + args.ms / 1000.0
        while time.time() < end:
            x = r.snap(0.35)
            if x:
                samples.append(x)
        r.send("STOP"); time.sleep(0.3)
        final = r.snap()
        print(f"final enc: {final}   samples: {len(samples)}  (cmd speed={args.speed})")
        cvL = _stats(samples, 1, "L")
        cvR = _stats(samples, 2, "R")
        worst = max([c for c in (cvL, cvR) if c is not None], default=None)
        print("\nVERDICT:",
              "no data" if worst is None else
              (f"THROBBING (CV={worst:.2f} ≥ 0.15)" if worst >= 0.15
               else f"SMOOTH (CV={worst:.2f} < 0.15)"))
        return 0
    finally:
        r.close()


if __name__ == "__main__":
    sys.exit(main())
