#!/usr/bin/env python3
"""enc_selftest.py — definitive encoder liveness probe.

Answers ONE question without guessing: are the wheel encoders actually stuck?

It does NOT trust SNAP (observed to report enc=0) or stale buffered frames. It
reads ENC off the live TLM stream (the path enc_watch uses) *while the wheels are
moving*, after flushing stale frames, and measures the change DURING each pulse:

    forward nudge (control: proven to move)  -> ENC must change
    rotate one way (wheels opposite)         -> ENC must change
    rotate back                              -> ENC must change back

If ENC changes during the pulses, encoders are ALIVE. If ENC is constant while the
wheels are commanded to move (and the forward control DID change), they are stuck
on rotation. If even the forward control doesn't change, watch the wheels: if they
spin it's the encoders/telemetry; if they sit still it's the motors/command.

    uv run python tests/dev/enc_selftest.py [--speed 50] [--pulse 0.8] [--thresh 4]

Net motion ~zero (nudge fwd, rotate, unrotate). Safe on stand or playfield.
"""

from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None)
    p.add_argument("--speed", type=int, default=50, help="wheel speed mm/s (default 50)")
    p.add_argument("--pulse", type=float, default=0.8, help="seconds per pulse (default 0.8)")
    p.add_argument("--thresh", type=int, default=4, help="mm change that counts as moved")
    args = p.parse_args()

    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.nezha import Nezha
    from robot_radio.robot.protocol import parse_tlm

    class _A:
        port = args.port
        verbose = False

    print("Connecting (relay, pushes calibration) …")
    robot, conn, _ = _make_robot(_A())
    if not isinstance(robot, Nezha):
        print("ERROR: need a Nezha"); return 2
    proto = robot._proto

    def flush(secs: float = 0.4) -> None:
        """Drop any stale buffered TLM frames while idle."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            proto.read_lines(duration_ms=40)

    def pulse(label: str, left: int, right: int, secs: float):
        """Drive (left,right) for secs, collecting the ENC sequence DURING motion.
        Returns (delta_L, delta_R, n_samples, moved)."""
        flush(0.35)
        seq = []
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs:
            proto.drive(left, right)
            for line in proto.read_lines(duration_ms=40):
                tlm = parse_tlm(line)
                if tlm is not None and tlm.enc is not None:
                    seq.append(tlm.enc)
        for _ in range(4):
            proto.stop(); time.sleep(0.04)
        if not seq:
            print(f"  {label:<12} samples=0  (NO telemetry while driving)")
            return 0, 0, 0, False
        dL = seq[-1][0] - seq[0][0]
        dR = seq[-1][1] - seq[0][1]
        show = seq if len(seq) <= 6 else [seq[0], seq[len(seq)//2], seq[-1]]
        print(f"  {label:<12} samples={len(seq):<3} seq~{show}  ->  dL{dL:+d} dR{dR:+d} mm")
        moved = abs(dL) >= args.thresh or abs(dR) >= args.thresh
        return dL, dR, len(seq), moved

    try:
        proto.send("STOP", 200)
        proto.send("STREAM 0", 200)
        proto.send("SET sTimeout=10000", 300)
        proto.zero_encoders()
        proto.stream(50)
        time.sleep(0.2)
        print("  setup done (zeroed, streaming). Reading ENC live during each pulse:\n")

        fwd = pulse("forward",   +args.speed, +args.speed, args.pulse)   # control
        time.sleep(0.15)
        out = pulse("rotate-CCW", -args.speed, +args.speed, args.pulse)
        time.sleep(0.15)
        back = pulse("rotate-CW",  +args.speed, -args.speed, args.pulse)

        print("\n========================================")
        any_tlm = (fwd[2] + out[2] + back[2]) > 0
        if not any_tlm:
            print("  NO TELEMETRY in any pulse — stream problem, NOT proof of stuck encoders.")
            return 1
        # Per-wheel: did this encoder change in ANY pulse?
        leftMoved  = any(abs(r[0]) >= args.thresh for r in (fwd, out, back))
        rightMoved = any(abs(r[1]) >= args.thresh for r in (fwd, out, back))
        print(f"  LEFT  encoder: {'COUNTS ✓' if leftMoved else 'no change ✗'}")
        print(f"  RIGHT encoder: {'COUNTS ✓' if rightMoved else 'no change ✗'}")
        if leftMoved and rightMoved:
            print("  -> both encoders alive.")
            return 0
        if not leftMoved and not rightMoved:
            print("  -> neither changed. Encoders stuck, OR the wheels didn't move")
            print("     (jammed / against a wall). Re-run with wheels FREE (the stand).")
            return 1
        stuck = "LEFT" if not leftMoved else "RIGHT"
        print(f"  -> only one side moved. The {stuck} encoder didn't change — but that")
        print(f"     also happens if the {stuck} wheel didn't physically turn (e.g. the")
        print("     robot pivoted against the boards). RE-RUN WITH WHEELS FREE (the")
        print(f"     stand) before concluding the {stuck} encoder is stuck.")
        return 1
    finally:
        try:
            for _ in range(3):
                proto.stop(); time.sleep(0.04)
            proto.stream(0)
            conn.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
