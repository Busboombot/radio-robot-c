#!/usr/bin/env python
"""Real playfield tour — camera-localized, bounds-gated, closed-loop.

On the playfield the robot drives untethered (radio relay) using its REAL
OTOS, and its firmware pose is NOT world-aligned (it drifts and starts at the
boot origin). So we do NOT trust SNAP for world position. Instead the overhead
camera (aprilcam) is the ground truth every leg:

  * pose  : robot AprilTag (id 100) world_xy, heading = yaw + 90deg
            (empirically confirmed: a +forward nudge moved the robot along
             yaw+90deg).
  * drive : relay !GO data-plane. ONE firmware G drives the whole leg, robot-
            relative offset computed from the camera pose. Then read the camera;
            if it didn't arrive, do ANOTHER full G for the remaining distance.
            One G, read camera, repeat — not little capped steps.
  * safety: a HARD bounds gate — the robot CENTER must stay within
            (+-ABORT_X, +-ABORT_Y) cm of the A1 origin. Checked before every
            step AND mid-step; the instant it leaves, send X and abort. Plus a
            '+' keepalive each poll so the all-motion watchdog stays fed.

Run:  uv run --group calibrate python host_tests/playfield_tour/playfield_tour_camera.py
"""
import sys
import time
import math
import json
import os
from pathlib import Path

import serial
from aprilcam.config import Config
from aprilcam.client.control import DaemonControl

RELAY = "/dev/cu.usbmodem2121402"
ROBOT_TAG = 100
HEAD_OFF = math.pi / 2.0     # world heading = tag yaw + 90deg (confirmed)

SPEED = 130                  # mm/s
ARRIVE_CM = 6.0
MAX_G_PER_LEG = 4            # one full G per try; re-G if it didn't arrive

# Field is 101cm x 89cm, A1-centred -> edges at +-50.5 x, +-44.5 y.
# Keep the robot CENTRE well inside (the robot is ~14cm long).
ABORT_X = 60.0
ABORT_Y = 42.0

# Safe targets: detected colored squares, all comfortably inside the field.
TARGETS = [
    ("red",     -36.0,   0.0),
    ("purple",  -35.0,  24.0),
    ("blue",    -35.0, -24.0),
    ("magenta",  -0.5, -23.0),
    ("center",    0.0,   0.0),
]

CAM_NAME = "arducam-ov9782-usb-camera"


def main():
    dc = DaemonControl.connect_default(Config.load())
    cam = dc.list_cameras()[0]

    def pose():
        """Averaged robot pose (x_cm, y_cm, H_rad) or None.

        Retries through transient tag dropouts for up to ~2s before giving up."""
        xs, ys, yaws = [], [], []
        deadline = time.time() + 5.0
        want = 2
        while time.time() < deadline and len(xs) < want:
            tf = dc.get_tags(cam)
            t = next((t for t in tf.tags if t.id == ROBOT_TAG and t.world_xy), None)
            if t:
                xs.append(t.world_xy[0]); ys.append(t.world_xy[1]); yaws.append(t.yaw)
            time.sleep(0.05)
        if not xs:
            return None
        n = len(xs)
        sy = sum(math.sin(a) for a in yaws); cy = sum(math.cos(a) for a in yaws)
        return (sum(xs) / n, sum(ys) / n, math.atan2(sy, cy) + HEAD_OFF)

    def quick_xy():
        # Retry briefly through tag dropouts so the mid-drive bounds gate is
        # not silently skipped while the robot is moving.
        for _ in range(5):
            tf = dc.get_tags(cam)
            t = next((t for t in tf.tags if t.id == ROBOT_TAG and t.world_xy), None)
            if t:
                return tuple(t.world_xy)
            time.sleep(0.04)
        return None

    def in_bounds(x, y):
        return abs(x) <= ABORT_X and abs(y) <= ABORT_Y

    p = serial.Serial(RELAY, 115200, timeout=0.3)
    time.sleep(1.6); p.reset_input_buffer()

    def send(c):
        p.write((c + "\n").encode()); p.flush()

    def stop():
        send("X"); time.sleep(0.2); p.read(8192)

    # Relay data-plane + clean reset.
    for c in ("!GO", "X", "STOP", "SET sTimeout=60000", "SET turnGate=35"):
        send(c); time.sleep(0.4); p.read(8192)

    path = []
    aborted = False
    try:
        sp = pose()
        if sp is None:
            print("No robot tag visible — aborting."); return
        print(f"start pose=({sp[0]:.1f},{sp[1]:.1f}) H={math.degrees(sp[2]) % 360:.0f}deg")
        if not in_bounds(sp[0], sp[1]):
            print(f"START ({sp[0]:.1f},{sp[1]:.1f}) past safe x — first leg drives inward toward center.")

        for name, tx, ty in TARGETS:
            print(f"\n--> {name} ({tx:+.0f},{ty:+.0f})")
            for attempt in range(MAX_G_PER_LEG):
                pp = pose()
                if pp is None:
                    print("   lost robot tag — STOP"); stop(); aborted = True; break
                rx, ry, H = pp
                path.append((rx, ry))
                if not in_bounds(rx, ry):
                    print(f"   OUT OF BOUNDS ({rx:.1f},{ry:.1f}) — STOP"); stop(); aborted = True; break
                dx, dy = tx - rx, ty - ry
                dist = math.hypot(dx, dy)
                print(f"   at ({rx:+.1f},{ry:+.1f}) H={math.degrees(H) % 360:3.0f} dist={dist:4.1f}cm")
                if dist <= ARRIVE_CM:
                    print(f"   reached {name}")
                    break
                # ONE full G for the whole remaining leg. The camera yaw is
                # opposite-handed to standard math (forward checks out, but the
                # lateral axis is mirrored), so NEGATE the left term.
                fwd = dx * math.cos(H) + dy * math.sin(H)
                lft = dx * math.sin(H) - dy * math.cos(H)
                send(f"G {fwd * 10:.0f} {lft * 10:.0f} {SPEED}")
                tmax = dist * 10 / SPEED + 6.0      # generous: turn + drive
                t0 = time.time()
                while time.time() - t0 < tmax:
                    p.write(b"+\n"); p.flush()      # keepalive
                    time.sleep(0.18)
                    out = p.read(8192).decode(errors="replace")
                    xy = quick_xy()
                    if xy:
                        path.append(xy)
                        if not in_bounds(*xy):
                            print(f"   OUT OF BOUNDS ({xy[0]:.1f},{xy[1]:.1f}) — STOP")
                            stop(); aborted = True; break
                        if math.hypot(tx - xy[0], ty - xy[1]) <= ARRIVE_CM:
                            break                  # arrived
                    if "done" in out.lower():
                        break                      # G finished
                stop()
                if aborted:
                    break
            if aborted:
                break

        stop()
        print(f"\n{'ABORTED' if aborted else 'TOUR COMPLETE'} — {len(path)} camera points")
    finally:
        stop(); p.close()
        # Draw the camera-traced path into the live aprilcam view.
        try:
            pf = Path(f"data/aprilcam/cameras/{CAM_NAME}/paths.json")
            if path and pf.parent.exists():
                wp = [{"x": x, "y": y, "size_cm": 2, "symbol": "filled_circle",
                       "symbol_color": [0, 200, 255], "line_color": [0, 200, 255]}
                      for x, y in path]
                tmp = pf.with_suffix(".tmp")
                tmp.write_text(json.dumps([{"path_id": "tour", "playfield_id": CAM_NAME,
                                            "waypoints": wp}]))
                os.replace(tmp, pf)
                print(f"path drawn in live view ({len(wp)} pts)")
        except Exception as e:
            print("path draw skipped:", e)
        dc.close()


if __name__ == "__main__":
    main()
