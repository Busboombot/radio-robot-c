"""square_run.py — real-hardware EKF characterisation on the playfield.

Drives the robot around the ring of colored playfield boxes (a "square"), using
the on-board EKF-fused pose to navigate (firmware `G` arc go-to), while logging
the raw encoder, raw OTOS, and fused-pose telemetry every frame plus the overhead
camera (AprilTag) pose as GROUND TRUTH. Produces a CSV that plot_square.py turns
into the same truth-vs-encoder-vs-OTOS-vs-fused graphs as the sim notebook —
except this time from real hardware.

Pipeline:
  1. PING the robot (hard-fail if silent).
  2. Read the robot's AprilTag-100 world pose from the camera; SI-set the
     firmware world pose to it (firmware frame := playfield A1-centred frame).
  3. Stream TLM fields enc,pose,otos,twist.
  4. For each colored box: `G <x_mm> <y_mm> <speed>`; while driving, log every
     TLM frame and poll the camera for ground truth; stream "+" keepalives.
  5. Save the log CSV.

Frames/units: playfield A1-centred, +X east, +Y north, CCW heading. Camera gives
cm; firmware gives mm; firmware heading = camera_yaw + 90 deg.

Usage:
    uv run python tests/bench/square_run.py --verify          # bench: stream-only, prove otos= telemetry
    uv run python tests/bench/square_run.py --no-camera       # bench: drive on stand, no camera truth
    uv run python tests/bench/square_run.py                   # playfield: full run, default ring of 8 boxes
    uv run python tests/bench/square_run.py --boxes purple-NW,orange-NE,green-SE,blue-SW   # 4-corner square
    uv run python tests/bench/square_run.py --correct         # SI-correct from camera at each box (stopped-fix)
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import sys
import time

_REPO = pathlib.Path(__file__).resolve().parents[2]
_HOST = _REPO / "host"
if str(_HOST) not in sys.path:
    sys.path.insert(0, str(_HOST))

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, parse_tlm

# Colored playfield boxes (cm, A1-centred frame) — from tests/bench/tour_goto.py.
SITES = {
    "purple-NW": (-35, 24), "black-N": (0, 24), "orange-NE": (35, 24),
    "red-E": (35, 0), "green-SE": (35, -24), "magenta-S": (0, -24),
    "blue-SW": (-35, -24), "red-W": (-35, 0),
}
# Default "square": all 8 colored boxes in ring order (a rounded rectangle loop).
DEFAULT_RING = ["purple-NW", "black-N", "orange-NE", "red-E",
                "green-SE", "magenta-S", "blue-SW", "red-W"]

ROBOT_TAG = 100
RAD = math.pi / 180.0


# --------------------------------------------------------------------------- #
# Camera (aprilcam daemon)                                                     #
# --------------------------------------------------------------------------- #
def open_camera():
    """Connect to the aprilcam daemon; return (dc, cam) or (None, None)."""
    try:
        from aprilcam.config import Config
        from aprilcam.client.control import DaemonControl
        dc = DaemonControl.connect_default(Config.load())
        cam = dc.list_cameras()[0]
        return dc, cam
    except Exception as e:  # noqa: BLE001
        print(f"[camera] unavailable: {e}")
        return None, None


def read_tag(dc, cam, tid=ROBOT_TAG, timeout_s=0.5):
    """One camera read of tag `tid`: (x_cm, y_cm, yaw_rad) or None."""
    if dc is None:
        return None
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            tf = dc.get_tags(cam)
        except Exception:  # noqa: BLE001
            return None
        for t in tf.tags:
            if t.id == tid and t.world_xy is not None and t.yaw is not None:
                return float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)
        time.sleep(0.02)
    return None


def robot_pose(dc, cam, n=5):
    """Median-filtered camera pose of the robot tag: (x_cm, y_cm, yaw_rad) or None."""
    xs, ys, yaws = [], [], []
    for _ in range(n):
        p = read_tag(dc, cam)
        if p:
            xs.append(p[0]); ys.append(p[1]); yaws.append(p[2])
        time.sleep(0.02)
    if not xs:
        return None
    xs.sort(); ys.sort(); yaws.sort()
    m = len(xs) // 2
    return xs[m], ys[m], yaws[m]


# --------------------------------------------------------------------------- #
# Main run                                                                     #
# --------------------------------------------------------------------------- #
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="robot serial port (default: auto)")
    ap.add_argument("--speed", type=int, default=160, help="drive speed mm/s")
    ap.add_argument("--boxes", default=None, help="comma list of box names (default: ring of 8)")
    ap.add_argument("--correct", action="store_true", help="SI-correct from camera at each box stop")
    ap.add_argument("--no-camera", action="store_true", help="skip camera (telemetry only)")
    ap.add_argument("--verify", action="store_true", help="stream-only smoke test; no driving")
    ap.add_argument("--settle-s", type=float, default=0.6, help="pause at each box (s)")
    ap.add_argument("--timeout-s", type=float, default=12.0, help="per-leg drive timeout (s)")
    ap.add_argument("--out", default=str(_REPO / "host_tests" / "square_run_log.csv"))
    args = ap.parse_args()

    # ---- connect + liveness ------------------------------------------------ #
    conn = SerialConnection(args.port) if args.port else SerialConnection()
    res = conn.connect()
    if res.get("error"):
        sys.exit(f"connect failed: {res['error']}")
    proto = NezhaProtocol(conn)
    png = proto.ping()
    if not png:
        sys.exit("PING failed — robot silent. Check power/port.")
    print(f"PING ok (robot_t={png[0]} ms, rtt={png[1]:.0f} ms)")
    proto.set_config(sTimeout=60000)  # keep watchdog out of the way

    trackwidth_mm = _get_trackwidth(proto)
    print(f"trackwidth = {trackwidth_mm:.1f} mm")

    # ---- camera + SI-set start pose --------------------------------------- #
    dc, cam = (None, None) if (args.no_camera or args.verify) else open_camera()
    x0_cm = y0_cm = yaw0 = None
    if dc is not None:
        p = robot_pose(dc, cam)
        if p is None:
            print("[camera] robot tag not seen — continuing without start fix")
        else:
            x0_cm, y0_cm, yaw0 = p
            h_cdeg = int(round((math.degrees(yaw0) + 90.0) * 100.0))
            proto.send(f"SI {int(round(x0_cm*10))} {int(round(y0_cm*10))} {h_cdeg}", 200)
            print(f"start camera pose: x={x0_cm:.1f}cm y={y0_cm:.1f}cm yaw={math.degrees(yaw0):.1f}deg "
                  f"-> SI set (h={h_cdeg}cdeg)")

    # ---- enable telemetry stream ------------------------------------------ #
    proto.send("STREAM fields=enc,pose,otos,twist", 200)
    proto.send("STREAM 40", 200)  # 40 ms period

    tlm_rows: list[dict] = []
    cam_rows: list[dict] = []
    t_start = time.monotonic()

    def pump(duration_s: float, *, keepalive: bool):
        """Read TLM for duration_s, logging frames + camera; optional keepalive."""
        end = time.monotonic() + duration_s
        got_done = False
        while time.monotonic() < end:
            if keepalive:
                conn.send_fast("+")
            for ln in conn.read_lines(duration_ms=60):
                if "done G" in ln or "EVT done" in ln:
                    got_done = True
                tlm = parse_tlm(ln)
                if tlm is None:
                    continue
                row = {"host_t": time.monotonic() - t_start, "robot_t": tlm.t, "mode": tlm.mode}
                if tlm.enc:   row["enc_l"], row["enc_r"] = tlm.enc
                if tlm.pose:  row["pose_x"], row["pose_y"], row["pose_h"] = tlm.pose
                if tlm.otos:  row["otos_x"], row["otos_y"], row["otos_h"] = tlm.otos
                if tlm.twist: row["v"], row["omega"] = tlm.twist
                tlm_rows.append(row)
            cp = read_tag(dc, cam, timeout_s=0.0) if dc is not None else None
            if cp:
                cam_rows.append({"host_t": time.monotonic() - t_start,
                                 "cam_x": cp[0], "cam_y": cp[1], "cam_yaw": cp[2]})
        return got_done

    # ---- verify mode: just stream a couple seconds and report -------------- #
    if args.verify:
        print("VERIFY: streaming 2s ...")
        pump(2.0, keepalive=False)
        _report_verify(tlm_rows)
        conn.send("STREAM 0", 100)
        conn.disconnect()
        return

    # ---- drive the square -------------------------------------------------- #
    route = (args.boxes.split(",") if args.boxes else DEFAULT_RING)
    print(f"route: {route}")
    pump(0.4, keepalive=False)  # capture a few resting frames at start
    for name in route:
        if name not in SITES:
            print(f"  skip unknown box {name}")
            continue
        bx_cm, by_cm = SITES[name]
        print(f"  -> {name} ({bx_cm},{by_cm} cm)")
        proto.send(f"G {int(round(bx_cm*10))} {int(round(by_cm*10))} {args.speed}", 200)
        done = pump(args.timeout_s, keepalive=True)
        print(f"     {'arrived' if done else 'timeout'}")
        conn.send_fast("X")            # ensure stop
        pump(args.settle_s, keepalive=False)
        if args.correct and dc is not None:
            p = robot_pose(dc, cam, n=4)
            if p:
                cx, cy, cyaw = p
                h_cdeg = int(round((math.degrees(cyaw) + 90.0) * 100.0))
                proto.send(f"SI {int(round(cx*10))} {int(round(cy*10))} {h_cdeg}", 150)
                print(f"     camera correct -> SI ({cx:.1f},{cy:.1f})")

    conn.send("STREAM 0", 100)
    conn.send_fast("X")
    conn.disconnect()

    # ---- save log ---------------------------------------------------------- #
    meta = {"trackwidth_mm": trackwidth_mm,
            "start_x_cm": x0_cm, "start_y_cm": y0_cm, "start_yaw_rad": yaw0,
            "speed": args.speed, "route": route}
    _save(args.out, tlm_rows, cam_rows, meta)
    print(f"\nlogged {len(tlm_rows)} TLM frames, {len(cam_rows)} camera frames -> {args.out}")
    print("plot with: uv run python tests/bench/plot_square.py")


def _get_trackwidth(proto, default=143.0) -> float:
    try:
        resp = proto.send("GET trackwidth", 200)
        for ln in resp.get("responses", []):
            if "trackwidth" in ln:
                for tok in ln.replace("=", " ").split():
                    try:
                        v = float(tok)
                        if 50 < v < 400:
                            return v
                    except ValueError:
                        continue
    except Exception:  # noqa: BLE001
        pass
    return default


def _report_verify(rows: list[dict]) -> None:
    if not rows:
        print("  NO TLM frames received — telemetry not streaming!")
        return
    have_otos = sum(1 for r in rows if "otos_x" in r)
    have_pose = sum(1 for r in rows if "pose_x" in r)
    have_enc = sum(1 for r in rows if "enc_l" in r)
    print(f"  frames={len(rows)}  with enc={have_enc} pose={have_pose} otos={have_otos}")
    last = rows[-1]
    print(f"  last frame: enc=({last.get('enc_l')},{last.get('enc_r')}) "
          f"pose=({last.get('pose_x')},{last.get('pose_y')},{last.get('pose_h')}) "
          f"otos=({last.get('otos_x')},{last.get('otos_y')},{last.get('otos_h')})")
    if have_otos:
        print("  OK: otos= field is present in the TLM stream.")
    else:
        print("  WARNING: no otos= field — check firmware flash / STREAM fields.")


def _save(path: str, tlm_rows, cam_rows, meta) -> None:
    p = pathlib.Path(path)
    cols = ["host_t", "robot_t", "mode", "enc_l", "enc_r",
            "pose_x", "pose_y", "pose_h", "otos_x", "otos_y", "otos_h", "v", "omega"]
    with open(p, "w", newline="") as f:
        f.write("# " + str(meta) + "\n")
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in tlm_rows:
            w.writerow({c: r.get(c, "") for c in cols})
    cam_path = p.with_name(p.stem + "_camera.csv")
    with open(cam_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["host_t", "cam_x", "cam_y", "cam_yaw"])
        w.writeheader()
        for r in cam_rows:
            w.writerow(r)


if __name__ == "__main__":
    main()
