#!/usr/bin/env python3
"""rotation_sweep.py — collect rotation error vs (angle, speed, direction).

Spins RAW (geometric, gain=1: arc = angle × π/180 × trackwidth/2) at a grid of
commanded angles × speeds × both directions, measures the ACTUAL rotation with
the overhead camera (AprilTag), and appends the raw data to
data/calibration/rotation_sweep.csv. No curve fit here — that's done offline in
data/calibration/rotation_fit.ipynb so the model can be explored.

Robot must be in camera view (playfield, via the relay).

Usage:
  uv run python tests/calibrate/rotation_sweep.py
      [--speeds 60,100,150] [--angles 30,60,120,200,300,400] [--reps 1]
"""
import argparse
import csv
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rotation_cal import _Cam, wrap180   # reuse camera + helper

_REPO_ROOT = Path(__file__).resolve().parents[2]
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"
_OUT_CSV = _REPO_ROOT / "data" / "calibration" / "rotation_sweep.csv"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--speeds", default="60,100,150", help="comma list of mm/s")
    ap.add_argument("--angles", default="30,60,120,200,300,400", help="comma list of deg")
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--out", default=str(_OUT_CSV))
    args = ap.parse_args()

    from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
    from robot_radio.robot.protocol import NezhaProtocol
    from robot_radio.config.robot_config import load_robot_config

    cfg = load_robot_config(_TOVEZ_JSON)
    tw = float(getattr(cfg.geometry, "trackwidth", None) or 126.0)
    DEG2ARC = (math.pi / 180.0) * (tw / 2.0)
    tag = getattr(cfg.vision, "robot_tag_id", None) or 100
    speeds = [int(x) for x in args.speeds.split(",")]
    angles = [int(x) for x in args.angles.split(",")]
    total = args.reps * len(speeds) * len(angles) * 2
    print(f"  trackwidth={tw:.0f}  tag={tag}  speeds={speeds}  angles={angles}  → {total} spins")

    port = args.port or (list_serial_ports() or [None])[0]
    if port is None:
        print("ERROR: no serial port"); return 2
    conn = SerialConnection(port=port)
    conn.connect()
    proto = NezhaProtocol(conn)
    time.sleep(0.5)
    try:
        cal = cfg.calibration
        if cal.mm_per_wheel_deg_left and cal.mm_per_wheel_deg_right:
            proto.set_config(ml=cal.mm_per_wheel_deg_left, mr=cal.mm_per_wheel_deg_right)
        ctrl = getattr(cfg, "control", None)
        if ctrl is not None:
            for k, v in (("vel.kP", ctrl.vel_kp), ("vel.kI", ctrl.vel_ki),
                         ("vel.kFF", ctrl.vel_kff), ("vel.iMax", ctrl.vel_imax),
                         ("vel.kAw", ctrl.vel_kaw), ("vel.filt", ctrl.vel_filt),
                         ("sync", ctrl.sync)):
                if v is not None:
                    proto.send(f"SET {k}={v:g}", 200)
    except Exception as exc:
        print(f"  WARN config push: {exc}")

    cam = _Cam(tag_id=tag)

    def heading():
        return cam.heading_deg()

    def raw_turn(cmd_deg, speed):
        arc = max(1, int(round(abs(cmd_deg) * DEG2ARC)))
        left, right = (speed, -speed) if (cmd_deg > 0) == lpos_ccw else (-speed, speed)
        proto.distance(left, right, arc)
        time.sleep(arc / max(speed, 1) * 1.3 + 1.2)
        proto.stop()
        time.sleep(0.5)

    # Probe the turn sign (which wheel pattern gives +yaw / CCW).
    ha = heading()
    proto.distance(speeds[0], -speeds[0], max(1, int(30 * DEG2ARC)))
    time.sleep(30 * DEG2ARC / speeds[0] * 1.3 + 1.2)
    proto.stop(); time.sleep(0.5)
    hb = heading()
    lpos_ccw = wrap180((hb or 0.0) - (ha or 0.0)) > 0.0
    print(f"  probe: (L+,R−) is {'CCW' if lpos_ccw else 'CW'}")

    outp = Path(args.out)
    outp.parent.mkdir(parents=True, exist_ok=True)
    new_file = not outp.exists()
    n = 0
    with open(outp, "a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(["cmd_deg", "speed_mms", "dir", "actual_deg",
                        "error_deg", "h0_deg", "h1_deg"])
        for _rep in range(args.reps):
            for speed in speeds:
                for ang in angles:
                    for d in (+1, -1):
                        cmd = d * ang
                        h0 = heading()
                        raw_turn(cmd, speed)
                        h1 = heading()
                        if h0 is None or h1 is None:
                            print(f"  cmd={cmd:+4d} spd={speed}: (no camera) — skip")
                            continue
                        dh = wrap180(h1 - h0)
                        actual = round((cmd - dh) / 360.0) * 360.0 + dh
                        err = actual - cmd
                        w.writerow([cmd, speed, d, round(actual, 2),
                                    round(err, 2), round(h0, 2), round(h1, 2)])
                        f.flush()
                        n += 1
                        print(f"  [{n:2d}/{total}] cmd={cmd:+4d} spd={speed:3d}: "
                              f"actual={actual:+7.1f}°  err={err:+6.1f}°")

    proto.stop()
    try:
        conn.disconnect()
    except Exception:
        pass
    print(f"\n  wrote {n} rows → {outp}")
    print("  fit the model in data/calibration/rotation_fit.ipynb")
    return 0


if __name__ == "__main__":
    sys.exit(main())
