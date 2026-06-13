#!/usr/bin/env python
"""Random playfield tour — drive the robot from slot to slot with single G commands,
and draw two paths on the AprilCam view: the robot's own odometry (telemetry) and
the camera's tag-100 track.

Each hop:
  1. select_location() — ask the camera where the robot is (x, y, heading).
  2. select_target()   — rank all playfield slots (squares + dots) by distance,
                         randomly pick one of the 5 farthest.
  3. compute_g()       — from (position, target) return the G command parameters.
  4. issue that ONE G, then keepalive_until_done() drives the watchdog while
     COLLECTING (a) every line the robot sends back (telemetry response) and
     (b) the camera tag-100 poses sampled during the move.
  5. extract_telemetry() pulls the odometry track out of the response, and
     draw_paths() draws odometry + camera tracks onto the view via the daemon
     overlay API (gRPC socket).

Run 5 hops in a loop.

  uv run --group calibrate python host_tests/playfield_tour/playfield_random_tour.py
"""
import json
import math
import re
import time
import random

import serial
from aprilcam.config import Config
from aprilcam.client.control import DaemonControl

RELAY = "/dev/cu.usbmodem2121402"          # radio relay serial port
ROBOT_TAG = 100                            # robot's AprilTag id
SPEED = 140                                # mm/s
STREAM_MS = 100                            # robot telemetry stream period
PLAYFIELD = "/Volumes/Proj/proj/RobotProjects/AprilTags/data/aprilcam/playfield.json"

ODO_COLOR = [255, 140, 0]                  # odometry track  (orange)
CAM_COLOR = [0, 200, 255]                  # camera track    (cyan)

# Target slots: the defined colored RECTANGLES only (the dots sit at the field
# edges where the robot can't reach and the camera read is parallax-corrupt).
_pf = json.load(open(PLAYFIELD))
SLOTS = [(s["slug"], float(s["x"]), float(s["y"]))
         for s in _pf["rectangles"]]

# --- camera connection -----------------------------------------------------
dc = DaemonControl.connect_default(Config.load())
cam = dc.list_cameras()[0]

# --- robot connection (relay !GO data-plane) -------------------------------
p = serial.Serial(RELAY, 115200, timeout=0.3)
time.sleep(1.6); p.reset_input_buffer()


def send(c):
    p.write((c + "\n").encode()); p.flush()


def keepalive():
    """Feed the robot's all-motion safety watchdog over the radio link."""
    p.write(b"+\n"); p.flush()


def robot_camera_xy():
    """Latest camera (x_cm, y_cm) for the robot tag, or None."""
    tf = dc.get_tags(cam)
    t = next((t for t in tf.tags if t.id == ROBOT_TAG and t.world_xy), None)
    return tuple(t.world_xy) if t else None


def keepalive_until_done(tmax):
    """Drive the watchdog until the robot reports the move done (or tmax elapses).

    While waiting it does two things and returns both:
      * collects EVERY line the robot sends back -> one block of response text;
      * samples the camera tag-100 position each tick -> a list of (x_cm, y_cm).
    Returns (response_text, camera_poses)."""
    chunks = []
    camera_poses = []
    t0 = time.time()
    while time.time() - t0 < tmax:
        keepalive()
        send("SNAP")                       # reliable telemetry (request/reply)
        time.sleep(0.15)
        chunk = p.read(8192).decode(errors="replace")
        if chunk:
            chunks.append(chunk)
        xy = robot_camera_xy()
        if xy:
            camera_poses.append(xy)
        if chunk and "done" in chunk.lower():
            break
    return ("".join(chunks), camera_poses)


# TLM frames arrive back-to-back with no newlines, so scan the whole blob for
# every "pose=x_mm,y_mm" field rather than splitting into lines.
_POSE_RE = re.compile(r"pose=(-?\d+),(-?\d+)")


def extract_telemetry(response_text):
    """Pull the odometry track out of a response block: (x_cm, y_cm) per TLM frame."""
    return [(int(x) / 10.0, int(y) / 10.0)
            for x, y in _POSE_RE.findall(response_text)]


def draw_paths(odometry, camera_track):
    """Draw odometry + camera tracks on the AprilCam view via the daemon overlay API."""
    elems = []
    if len(camera_track) >= 2:
        flat = [v for xy in camera_track for v in xy]
        elems.append({"type": "polyline", "params": flat,
                      "color": CAM_COLOR, "thickness": 3})
    if len(odometry) >= 2:
        flat = [v for xy in odometry for v in xy]
        elems.append({"type": "polyline", "params": flat,
                      "color": ODO_COLOR, "thickness": 2})
    if elems:
        dc.publish_overlay(cam, elems, ttl=120.0)


def distance(target, location):
    """Planar distance in cm between a slot (slug, x, y) and a location (x, y, ...)."""
    return math.hypot(target[1] - location[0], target[2] - location[1])


def select_location():
    """Use the camera to figure out where we are: (x_cm, y_cm, heading_rad).

    Heading = tag yaw + 90deg (the robot's forward axis)."""
    for _ in range(60):
        xy = robot_camera_xy()
        if xy:
            tf = dc.get_tags(cam)
            t = next((t for t in tf.tags if t.id == ROBOT_TAG and t.world_xy), None)
            if t:
                return (t.world_xy[0], t.world_xy[1], t.yaw + math.pi / 2.0)
        time.sleep(0.05)
    raise RuntimeError("robot tag 100 not visible to camera")


def select_target(location):
    """Given our location, return a slot randomly chosen from the 5 farthest away."""
    ranked = sorted(SLOTS, key=lambda s: distance(s, location), reverse=True)
    return random.choice(ranked[:5])


def compute_g(location, target):
    """From our position+heading and the target position, return G params (fwd_mm, left_mm).

    Robot-relative, standard right-handed: forward along the heading, left = +90deg
    CCW. The firmware beginGoTo converts this back to a world target with the SAME
    standard rotation (world = pose + R(h)*(fwd,left)), and hop() SIs the firmware
    heading to this H, so the two rotations cancel and the robot drives to (tx,ty)
    exactly. (Do NOT negate the lateral — that reflects the target across H.)"""
    x, y, H = location
    tx, ty = target[1], target[2]
    dx, dy = tx - x, ty - y
    fwd = dx * math.cos(H) + dy * math.sin(H)
    lft = -dx * math.sin(H) + dy * math.cos(H)
    return (fwd * 10.0, lft * 10.0)


# Accumulated tracks across the whole tour, redrawn each hop.
all_odo = []
all_cam = []


def hop():
    """One place-to-place move: locate, pick a far target, drive, collect + draw tracks."""
    loc = select_location()
    target = select_target(loc)
    fwd, lft = compute_g(loc, target)
    dist = distance(target, loc)
    print(f"at ({loc[0]:+5.0f},{loc[1]:+5.0f}) H={math.degrees(loc[2]) % 360:3.0f}  "
          f"-> {target[0]:22s} ({target[1]:+5.0f},{target[2]:+5.0f})  "
          f"G {fwd:.0f} {lft:.0f} {SPEED}")

    # Anchor the firmware odometry to the camera world pose so the telemetry
    # track is in the same frame as the camera track.
    send(f"SI {loc[0] * 10:.0f} {loc[1] * 10:.0f} {math.degrees(loc[2]) * 100:.0f}")
    time.sleep(0.2); p.read(8192)

    send(f"G {fwd:.0f} {lft:.0f} {SPEED}")
    response, camera_poses = keepalive_until_done(dist * 10 / SPEED + 6.0)
    send("X"); time.sleep(0.3); p.read(8192)

    odometry = extract_telemetry(response)
    all_odo.extend(odometry)
    all_cam.extend(camera_poses)
    draw_paths(all_odo, all_cam)

    final = select_location()
    err = distance(target, final)
    print(f"   arrived ({final[0]:+5.0f},{final[1]:+5.0f})  err={err:.1f}cm  "
          f"odo={len(odometry)}pts cam={len(camera_poses)}pts")


try:
    for c in ("!GO", "X", "STOP", "SET sTimeout=60000", "SET turnGate=35",
              f"STREAM {STREAM_MS}"):
        send(c); time.sleep(0.4); p.read(8192)
    for i in range(5):
        print(f"=== hop {i + 1}/5 ===")
        hop()
        time.sleep(0.4)
finally:
    send("STREAM 0"); send("X"); p.close(); dc.close()
print("tour done")
