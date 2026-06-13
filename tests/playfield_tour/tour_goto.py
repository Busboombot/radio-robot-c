#!/usr/bin/env python3
"""tour_goto.py — test the firmware G go-to: the robot drives itself on the OTOS.

This is a TEST of the onboard `G` arc go-to. The robot localizes with its own
OTOS odometer and drives one smooth arc to each goal — the camera does NOT steer.
Per hop:
  1. read the camera once (ground-truth world pose),
  2. **OV** — establish the OTOS's world position + heading from that fix,
  3. **G <fwd> <lft> <speed>** — the robot drives there itself on the OTOS,
  4. stop when `EVT done G` fires (or the camera shows arrival / fence), read the
     true error from the camera, then re-establish at the next stop.

Targets: 16 colored sites (8 squares + 8 dots). Each iteration ranks them by
distance, drops the closest 4, and picks one at random. Background keepalive +
firmware safety-stop are the backstop.

  uv run python tests/playfield_tour/tour_goto.py            # 10 hops
  uv run python tests/playfield_tour/tour_goto.py --iters 20 --seed 3
"""
import argparse
import math
import pathlib
import random
import sys
import time

_BENCH = pathlib.Path(__file__).resolve().parent.parent / "bench"  # shared bench helpers
if str(_BENCH) not in sys.path:
    sys.path.insert(0, str(_BENCH))
from bench_safety import BenchRun  # noqa: E402

ROBOT = 100
FX, FY = 67 - 8, 44.65 - 8     # A1-centred safety fence (inset from the ArUco corners)

SITES = {
    "purple-NW": (-35.0, 24.0), "black-N": (0.0, 24.0), "orange-NE": (35.0, 24.0),
    "red-E": (35.0, 0.0), "green-SE": (35.0, -24.0), "magenta-S": (0.0, -24.0),
    "blue-SW": (-35.0, -24.0), "red-W": (-35.0, 0.0),
    "dotO-NW": (-50.0, 30.0), "dotG-N": (0.0, 30.0), "dotO-NE": (50.0, 30.0),
    "dotR-E": (50.0, 0.0), "dotO-SE": (50.0, -30.0), "dotY-S": (0.0, -30.0),
    "dotO-SW": (-50.0, -30.0), "dotB-W": (-50.0, 0.0),
}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def in_fence(x, y):
    return -FX <= x <= FX and -FY <= y <= FY


def open_daemon():
    from aprilcam.config import Config
    from aprilcam.client.control import DaemonControl
    dc = DaemonControl.connect_default(Config.load())
    return dc, dc.list_cameras()[0]


def get_tag(dc, cam, tid, timeout=0.6):
    dl = time.monotonic() + timeout
    while time.monotonic() < dl:
        for t in dc.get_tags(cam).tags:
            if t.id == tid and t.world_xy is not None and t.yaw is not None:
                return float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)
        time.sleep(0.02)
    return None


def robot_pose(dc, cam, n=4):
    xs = ys = mx = my = 0.0
    got = 0
    for _ in range(n):
        r = get_tag(dc, cam, ROBOT, 0.35)
        if r:
            xs += r[0]; ys += r[1]
            mx += math.cos(r[2]); my += math.sin(r[2])
            got += 1
    return (xs / got, ys / got, math.atan2(my, mx)) if got else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=10)
    ap.add_argument("--seed", type=int, default=None, help="RNG seed (omit = fresh random)")
    ap.add_argument("--arrive", type=float, default=6.0, help="arrival tolerance cm")
    ap.add_argument("--speed", type=int, default=200, help="G drive speed mm/s")
    ap.add_argument("--drop", type=int, default=4, help="exclude the N closest targets")
    ap.add_argument("--gtries", type=int, default=3, help="max G drives per goal")
    ap.add_argument("--port", default="/dev/cu.usbmodem2121302")
    args = ap.parse_args()
    rng = random.Random(args.seed)

    dc, cam = open_daemon()
    if get_tag(dc, cam, ROBOT, 2.0) is None:
        dc.close()
        raise SystemExit("robot tag (100) not visible — check the tag is on and in the camera view")

    from robot_radio.io.cli import _make_robot

    class A:
        port = args.port
        verbose = False

    robot, conn, _ = _make_robot(A())
    proto = robot._proto
    proto.send("SET sTimeout=800", 200)
    proto.send("SET turnGate=35", 200)

    def stop():
        for _ in range(2):
            proto.stop(); time.sleep(0.03)

    def reacquire():
        r = robot_pose(dc, cam)
        if r is not None:
            return r
        stop()
        for _ in range(8):
            time.sleep(0.2)
            r = robot_pose(dc, cam)
            if r is not None:
                return r
        return None

    def establish_and_go(gx, gy, rx, ry, yaw):
        """SI: establish the odometry world pose from the camera fix; then G to goal."""
        # SI — set the odometry pose G reads (firmware heading = yaw + 90°).
        h_cdeg = int(round((math.degrees(yaw) + 90.0) * 100.0))
        proto.send(f"SI {int(round(rx * 10))} {int(round(ry * 10))} {h_cdeg}", 150)
        time.sleep(0.12)
        # robot-relative target: forward=(-sin,cos), left=(-cos,-sin) of yaw (cm→mm)
        dx, dy = gx - rx, gy - ry
        fwd = (-dx * math.sin(yaw) + dy * math.cos(yaw)) * 10.0
        lft = (-dx * math.cos(yaw) - dy * math.sin(yaw)) * 10.0
        proto.send(f"G {int(round(fwd))} {int(round(lft))} {args.speed}", 150)

    def drive_to(gx, gy):
        """ONE G: from where the robot is, arc all the way to the goal on the OTOS.
        OV establishes the OTOS world pose first; then a single G drives the full
        arc. The camera only watches the fence (safety) and reads the true error
        at the end — it never steers and there are no hops."""
        r = reacquire()
        if r is None:
            return "LOST", (0.0, 0.0), 0.0
        rx, ry, yaw = r
        if math.hypot(gx - rx, gy - ry) <= args.arrive:
            return "OK", (rx, ry), math.hypot(gx - rx, gy - ry)
        establish_and_go(gx, gy, rx, ry, yaw)        # OV, then ONE G (full arc)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 16.0:
            rr = get_tag(dc, cam, ROBOT, 0.12)
            if rr and not in_fence(rr[0], rr[1]):     # safety only — fence breach
                stop()
                return "FENCE", (rr[0], rr[1]), math.hypot(gx - rr[0], gy - rr[1])
            if any("done G" in ln for ln in conn.read_lines(90)):
                break
        stop()
        r = robot_pose(dc, cam)
        if r is None:
            return "LOST", (rx, ry), 0.0
        return "GDONE", (r[0], r[1]), math.hypot(gx - r[0], gy - r[1])

    errs = []
    t_start = time.monotonic()
    try:
        with BenchRun(proto, max_seconds=args.iters * 20 + 30):
            for it in range(args.iters):
                p = robot_pose(dc, cam)
                if p is None:
                    print(f"[{it + 1}] lost the robot tag — stopping")
                    break
                rx, ry, _ = p
                ranked = sorted(SITES.items(),
                                key=lambda kv: math.hypot(kv[1][0] - rx, kv[1][1] - ry))
                name, (tx, ty) = rng.choice(ranked[args.drop:] or ranked)
                dist = math.hypot(tx - rx, ty - ry)
                print(f"[{it + 1:2d}/{args.iters}] from ({rx:+5.1f},{ry:+5.1f}) → "
                      f"{name:10s} ({tx:+.0f},{ty:+.0f})  {dist:4.0f}cm out")
                st, xy, dd = drive_to(tx, ty)
                if st in ("OK", "GDONE"):
                    errs.append(dd)
                print(f"            reached ({xy[0]:+5.1f},{xy[1]:+5.1f})  {dd:4.1f}cm off  [{st}]")
        dt = time.monotonic() - t_start
        if errs:
            print(f"\n{len(errs)} hops in {dt:.0f}s — G+OTOS arrival: "
                  f"mean {sum(errs) / len(errs):.1f}cm  max {max(errs):.1f}cm")
        else:
            print(f"\ntour ended in {dt:.0f}s")
    finally:
        stop()
        try:
            conn.disconnect()
        except Exception:
            pass
        try:
            dc.close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
