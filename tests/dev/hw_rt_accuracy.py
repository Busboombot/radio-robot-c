#!/usr/bin/env python3
"""hw_rt_accuracy.py — clean encoder measurement of the firmware RT turn.

For each angle: zero encoders, send RT, wait for EVT done RT, then read the
final encoder differential (streaming, post-done so it's clean) and convert to
degrees via the trackwidth. Reports encoder-implied angle vs commanded.

Encoder-implied angle == physical angle when there is no wheel slip (the sim
verified the geometry; only slip would diverge the two). A safety X brackets
each turn.

  uv run python tests/dev/hw_rt_accuracy.py [--port ...] [--tw 126]
"""
import argparse
import math
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--tw", type=float, default=126.0)
    ap.add_argument("--angles", default="90,-90,180,360,45")
    args = ap.parse_args()

    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.nezha import Nezha
    from robot_radio.robot.protocol import parse_tlm

    class _A:
        port = args.port
        verbose = False

    robot, conn, _ = _make_robot(_A())
    if not isinstance(robot, Nezha):
        print("ERROR: need a Nezha")
        return 2
    proto = robot._proto
    proto.send("SET sTimeout=60000", 300)

    def safety_stop():
        for _ in range(5):
            proto.send("X", 100)
            time.sleep(0.03)
        proto.stream(0)

    print(f"  TW={args.tw}  (encoder-implied angle = (encR-encL)/TW)")
    print(f"  {'cmd°':>6} {'enc°':>8} {'err°':>7} {'encL':>6} {'encR':>6} {'done':>5}")
    for a in [float(x) for x in args.angles.split(",")]:
        safety_stop()
        proto.zero_encoders()
        proto.stream(50)
        enc = (0, 0)
        done = False
        t0 = time.monotonic()
        proto.send(f"RT {int(round(a * 100))}", 80)
        # watch for EVT done RT, then 0.5 s settle
        t_done = None
        while time.monotonic() - t0 < 10.0:
            for line in proto.read_lines(duration_ms=40):
                tlm = parse_tlm(line)
                if tlm is not None and tlm.enc is not None:
                    enc = tlm.enc
                if "done" in line and "RT" in line:
                    done = True
                    t_done = time.monotonic()
            if t_done is not None and time.monotonic() - t_done > 0.5:
                break
        proto.stream(0)
        safety_stop()

        diff = enc[1] - enc[0]                      # encR - encL
        enc_deg = diff / args.tw * 180.0 / math.pi
        print(f"  {a:6.0f} {enc_deg:8.1f} {enc_deg - a:7.1f} "
              f"{enc[0]:>6} {enc[1]:>6} {'yes' if done else 'NO':>5}")

    safety_stop()
    try:
        conn.disconnect()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
