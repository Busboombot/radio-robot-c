#!/usr/bin/env python3
"""eturn.py — encoder-distance dead-reckoning turn (streaming TLM).

Pure dead reckoning, the way the stakeholder asked for it:
  1. Compute the per-wheel arc length for N degrees from the trackwidth.
       arc = |deg| * (pi/180) * (trackwidth / 2)
     A full 360 deg spin is ~1.6 wheel revolutions; 90 deg is ~0.4.
  2. Spin in place, watching the encoders via the STREAMING TLM path
     (proto.stream + parse_tlm) — the reliable read, not the SNAP path
     that returns zeros.
  3. STOP the instant the wheels have rotated that arc.

Three independent ways it stops (it can NOT run away):
  - reaches the encoder target  -> stop_hard()
  - a wall-clock time cap        -> stop_hard()
  - firmware watchdog: sTimeout=400ms, so if this script dies mid-spin the
    robot halts on its own within 400 ms.

Connection is via _make_robot (relay-aware, pushes calibration), same as
enc_watch.py.

Usage: uv run python tests/dev/eturn.py 90 [--speed 60] [--coast 3] [--port ...]
"""
import argparse
import math
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("degrees", type=float, help="signed turn angle (+ = CCW)")
    ap.add_argument("--port", default=None)
    ap.add_argument("--speed", type=int, default=60, help="wheel speed mm/s")
    ap.add_argument("--coast", type=float, default=3.0, help="stop N mm early for coast")
    ap.add_argument("--tw", type=float, default=126.0, help="trackwidth mm")
    args = ap.parse_args()

    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.nezha import Nezha
    from robot_radio.robot.protocol import parse_tlm

    class _A:
        port = args.port
        verbose = False

    mm_per_deg = (args.tw / 2.0) * (math.pi / 180.0)
    target = abs(args.degrees) * mm_per_deg
    sign = 1 if args.degrees >= 0 else -1            # +deg = CCW = drive(-L,+R)
    wheel_circ = math.pi * 80.77
    cap_s = (target / max(args.speed, 1)) * 2.5 + 1.5

    print(f"turn {args.degrees:+.0f}° -> {target:.1f}mm/wheel "
          f"({target / wheel_circ:.2f} wheel-rev)  speed={args.speed} "
          f"coast={args.coast}  cap={cap_s:.1f}s")

    robot, conn, _ = _make_robot(_A())
    if not isinstance(robot, Nezha):
        print("ERROR: need a Nezha")
        return 2
    proto = robot._proto

    def stop_hard():
        for _ in range(4):
            proto.stop()
            time.sleep(0.04)
        proto.stream(0)

    reached = False
    moved = False
    enc = (0, 0)
    try:
        proto.send("STOP", 200)
        proto.send("STREAM 0", 200)
        proto.send("SET sTimeout=400", 300)         # auto-stop if keepalives lapse
        proto.zero_encoders()
        proto.stream(50)

        t0 = time.monotonic()
        last_send = last_print = 0.0
        while True:
            now = time.monotonic()
            if now - t0 > cap_s:
                print(f"  CAP {cap_s:.1f}s reached — stopping.")
                break
            if now - last_send >= 0.12:
                proto.drive(-sign * args.speed, sign * args.speed)
                last_send = now
            for line in proto.read_lines(duration_ms=25):
                if "EVT safety_stop" in line:
                    proto.drive(-sign * args.speed, sign * args.speed)
                tlm = parse_tlm(line)
                if tlm is not None and tlm.enc is not None:
                    enc = tlm.enc
            prog = (abs(enc[0]) + abs(enc[1])) / 2.0
            if prog >= 5:
                moved = True
            if now - last_print >= 0.2:
                last_print = now
                print(f"  t={now - t0:4.1f}s enc=L{enc[0]:>6} R{enc[1]:>6}  "
                      f"prog={prog:6.1f}mm ~{prog / mm_per_deg:6.1f}°")
            if prog >= target - args.coast:
                reached = True
                break
        stop_hard()

        time.sleep(0.3)
        proto.stream(50)                            # short burst to read final
        fin = enc
        t1 = time.monotonic()
        while time.monotonic() - t1 < 0.6:
            for line in proto.read_lines(duration_ms=25):
                tlm = parse_tlm(line)
                if tlm is not None and tlm.enc is not None:
                    fin = tlm.enc
        proto.stream(0)
        prog = (abs(fin[0]) + abs(fin[1])) / 2.0
        status = ("REACHED" if reached else
                  "MOVED-BUT-CAPPED" if moved else "NO-ENCODER-MOTION")
        print(f"DONE [{status}]: final enc=L{fin[0]} R{fin[1]} -> "
              f"{prog / mm_per_deg:.1f}°  (target {abs(args.degrees):.0f}°)")
    finally:
        try:
            stop_hard()
            conn.disconnect()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
