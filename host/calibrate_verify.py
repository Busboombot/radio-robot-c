#!/usr/bin/env python3
"""calibrate_verify.py — autonomous bench verification using the overhead camera.

Drives the robot over the RADIORELAY (v2 / RAW250 data plane) and compares the
robot's ONBOARD telemetry (encoders, fused pose) against GROUND TRUTH from the
aprilcam daemon (the robot's AprilTag, world frame, cm). Frame-invariant deltas
(distance magnitude, heading change) are used so the two frames need no alignment.

Needs both `aprilcam` (the AprilTags project venv) and `pyserial`. Run as:
    cd /Volumes/Proj/proj/RobotProjects/AprilTags
    uv run --with pyserial python \
        /Volumes/Proj/proj/RobotProjects/radio-robot-c/host/calibrate_verify.py

Always sends STOP on exit. Robot is on the playfield (real floor motion).
"""
import math
import re
import sys
import time
import statistics

import serial
from aprilcam.client.control import DaemonControl
from aprilcam.config import Config

RELAY_PORT = "/dev/cu.usbmodem21421302"
ROBOT_TAG = 100   # tovez wears AprilTag 100 (tag 1 is a static field marker)
BAUD = 115200


# ----------------------------- camera ground truth ------------------------- #
class Cam:
    def __init__(self):
        self.dc = DaemonControl.connect_default(Config.load())
        cams = self.dc.list_cameras()
        if not cams:
            raise SystemExit("aprilcam: no cameras open")
        self.cam = cams[0] if isinstance(cams[0], str) else getattr(cams[0], "id", cams[0])

    def pose(self, samples=5):
        """Median (x_mm, y_mm, yaw_rad) of the robot tag over a few frames."""
        xs, ys, yaws = [], [], []
        for _ in range(samples):
            tf = self.dc.get_tags(self.cam)
            for t in tf.tags:
                if t.id == ROBOT_TAG and getattr(t, "world_xy", None) is not None:
                    xs.append(float(t.world_xy[0]) * 10.0)   # cm -> mm
                    ys.append(float(t.world_xy[1]) * 10.0)
                    yaws.append(float(t.yaw))
            time.sleep(0.05)
        if not xs:
            return None
        # circular median for yaw via atan2 of mean unit vector
        cy = math.atan2(statistics.fmean(math.sin(y) for y in yaws),
                        statistics.fmean(math.cos(y) for y in yaws))
        return (statistics.median(xs), statistics.median(ys), cy)

    def close(self):
        try: self.dc.close()
        except Exception: pass


def yaw_delta(a, b):
    """Smallest signed b-a in radians, wrapped to (-pi, pi]."""
    return (b - a + math.pi) % (2 * math.pi) - math.pi


# ----------------------------- relay / robot ------------------------------- #
class Relay:
    def __init__(self, port):
        self.s = serial.Serial(port, BAUD, timeout=0.2)
        time.sleep(2.0)
        self.s.reset_input_buffer()

    def _cmd(self, line, w=0.4):
        self.s.write((line + "\n").encode()); self.s.flush(); time.sleep(w)
        return self.s.read(8192).decode(errors="replace")

    def configure_go(self):
        b = self._cmd("HELLO")
        self._cmd("!MODE RAW250"); self._cmd("!CG 0 10"); self._cmd("!P 7")
        self._cmd("!GO", 0.8); self.s.reset_input_buffer()
        return b.strip()

    def send(self, line): self.s.write((line + "\n").encode()); self.s.flush()

    def read_until(self, want, timeout):
        t = time.time() + timeout; buf = b""
        while time.time() < t:
            buf += self.s.read(4096)
            if want in buf.decode(errors="replace"): return buf.decode(errors="replace")
            time.sleep(0.04)
        return buf.decode(errors="replace")

    def snap(self):
        """Return dict with enc=(l,r) and pose=(x,y,h_cdeg) from a TLM frame."""
        self.s.reset_input_buffer(); self.send("SNAP")
        b = self.read_until("TLM", 2.5)
        enc = re.findall(r"enc=(-?\d+),(-?\d+)", b)
        pose = re.findall(r"pose=(-?\d+),(-?\d+),(-?\d+)", b)
        d = {}
        if enc: d["enc"] = (int(enc[-1][0]), int(enc[-1][1]))
        if pose: d["pose"] = (int(pose[-1][0]), int(pose[-1][1]), int(pose[-1][2]))
        return d

    def query(self, cmd, want, timeout=2.0):
        self.s.reset_input_buffer(); self.send(cmd)
        return self.read_until(want, timeout).strip()

    def stop(self):
        try: self.send("STOP"); time.sleep(0.3)
        except Exception: pass

    def close(self):
        try: self.s.close()
        except Exception: pass


def main():
    cam = Cam()
    relay = Relay(RELAY_PORT)
    try:
        print("relay:", relay.configure_go())
        print("ID   :", relay.query(b"ID".decode() if False else "ID", "ID ", 3.0))
        print("connect-apply: ", relay.query("GET tw ml mr", "CG", 2.0))
        print("  OL:", relay.query("OL", "OK", 1.5), " OA:", relay.query("OA", "OK", 1.5))

        # ---- baseline ----
        relay.query("ZERO enc", "OK"); relay.query("ZERO pose", "OK"); time.sleep(0.3)
        c0 = cam.pose(); s0 = relay.snap()
        print(f"\nbaseline  cam={fmt(c0)}  robot_snap={s0}")

        # ---- LEG 1: forward 300 mm ----
        print("\n>>> FORWARD: D 150 150 300")
        relay.send("D 150 150 300"); done = "EVT done D" in relay.read_until("EVT done D", 8.0)
        time.sleep(0.4); c1 = cam.pose(); s1 = relay.snap()
        cam_d = dist(c0, c1); cam_dh = math.degrees(yaw_delta(c0[2], c1[2]))
        enc_d = ((s1["enc"][0]-s0["enc"][0]) + (s1["enc"][1]-s0["enc"][1]))/2 if "enc" in s0 and "enc" in s1 else None
        rp_d = dist(s0.get("pose"), s1.get("pose")) if s0.get("pose") and s1.get("pose") else None
        rp_dh = (s1["pose"][2]-s0["pose"][2])/100.0 if s0.get("pose") and s1.get("pose") else None
        print(f"  done={done}")
        print(f"  CAMERA actual: dist={cam_d:.0f}mm  headingΔ={cam_dh:.1f}°")
        print(f"  ENCODER mean : {enc_d}mm   (cmd 300)   enc/cam={enc_d/cam_d if cam_d else 0:.3f}")
        print(f"  ROBOT pose   : dist={rp_d}mm headingΔ={rp_dh}°   pose/cam={ (rp_d/cam_d) if (rp_d and cam_d) else 0:.3f}")
        print(f"  STRAIGHTNESS : camera heading drift over the leg = {cam_dh:.1f}°")

        # ---- LEG 2: spin in place ----
        print("\n>>> SPIN: T 140 -140 1100")
        relay.query("ZERO enc", "OK"); time.sleep(0.2)
        c2 = cam.pose()
        relay.send("T 140 -140 1100"); relay.read_until("EVT done T", 4.0)
        time.sleep(0.4); c3 = cam.pose(); s3 = relay.snap()
        cam_spin = math.degrees(yaw_delta(c2[2], c3[2]))
        print(f"  CAMERA actual rotation = {cam_spin:.1f}°   (translation drift={dist(c2,c3):.0f}mm — want ~0 for in-place)")
        print(f"  robot snap after spin: {s3}")

        print("\n==================== VERDICT ====================")
        if cam_d:
            print(f"  Distance:  encoder reads {enc_d/cam_d*100:.0f}% of camera-actual" if enc_d else "  (no encoder)")
            print(f"  Pose:      robot fused pose reads {rp_d/cam_d*100:.0f}% of camera-actual" if rp_d else "  (no pose)")
        print("  (want ~100%; <90 or >110 means a scale/calibration gap)")
    finally:
        relay.stop(); relay.close(); cam.close()


def fmt(p): return None if p is None else f"({p[0]:.0f},{p[1]:.0f}mm,{math.degrees(p[2]):.1f}°)"
def dist(a, b):
    if not a or not b: return None
    return math.hypot(b[0]-a[0], b[1]-a[1])


if __name__ == "__main__":
    sys.exit(main())
