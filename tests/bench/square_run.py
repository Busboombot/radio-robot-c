"""square_run.py — real-hardware EKF characterisation on the playfield.

Drives the robot around the ring of colored playfield boxes (a "square"), using
the on-board EKF-fused pose to navigate (firmware `G` arc go-to), while logging
the raw encoder, raw OTOS, and fused-pose telemetry plus the overhead camera
(AprilTag) pose as GROUND TRUTH. Produces a CSV that plot_square.py turns into
the same truth-vs-encoder-vs-OTOS-vs-fused graphs as the sim notebook — except
from real hardware.

Telemetry is gathered by polling `SNAP` (synchronous one-shot TLM, request/reply
— reliable over the radio relay; async STREAM frames get dropped by the bridge).

Pipeline:
  1. PING the robot (hard-fail if silent).
  2. Read the robot's AprilTag-100 world pose from the camera; SI-set the
     firmware world pose to it (firmware frame := playfield A1-centred frame).
  3. For each colored box: `G <x_mm> <y_mm> <speed>`; poll SNAP + camera until
     the fused pose reaches the box (or timeout); stream "+" keepalives.
  4. Save the log CSV.

Frames/units: playfield A1-centred, +X east, +Y north, CCW heading. Camera gives
cm; firmware gives mm; firmware heading = camera_yaw + 90 deg.

Usage:
    uv run python tests/bench/square_run.py --verify --port /dev/cu.usbmodem2121302
    uv run python tests/bench/square_run.py --no-camera --port /dev/cu.usbmodem2121302
    uv run python tests/bench/square_run.py --port /dev/cu.usbmodem2121302
    uv run python tests/bench/square_run.py --boxes purple-NW,orange-NE,green-SE,blue-SW --port ...
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

SITES = {
    "purple-NW": (-35, 24), "black-N": (0, 24), "orange-NE": (35, 24),
    "red-E": (35, 0), "green-SE": (35, -24), "magenta-S": (0, -24),
    "blue-SW": (-35, -24), "red-W": (-35, 0),
}
DEFAULT_RING = ["purple-NW", "black-N", "orange-NE", "red-E",
                "green-SE", "magenta-S", "blue-SW", "red-W"]
ROBOT_TAG = 100


# --------------------------------------------------------------------------- #
# Camera (aprilcam daemon)                                                     #
# --------------------------------------------------------------------------- #
def open_camera():
    try:
        from aprilcam.config import Config
        from aprilcam.client.control import DaemonControl
        dc = DaemonControl.connect_default(Config.load())
        cam = dc.list_cameras()[0]
        return dc, cam
    except Exception as e:  # noqa: BLE001
        print(f"[camera] unavailable: {e}")
        return None, None


def read_tag(dc, cam, tid=ROBOT_TAG, timeout_s=0.3):
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
        if timeout_s <= 0:
            return None
        time.sleep(0.02)
    return None


def robot_pose(dc, cam, n=5):
    xs, ys, yaws = [], [], []
    for _ in range(n):
        p = read_tag(dc, cam, timeout_s=0.4)
        if p:
            xs.append(p[0]); ys.append(p[1]); yaws.append(p[2])
        time.sleep(0.02)
    if not xs:
        return None
    xs.sort(); ys.sort(); yaws.sort()
    m = len(xs) // 2
    return xs[m], ys[m], yaws[m]


# --------------------------------------------------------------------------- #
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="relay serial port")
    ap.add_argument("--speed", type=int, default=150, help="drive speed mm/s")
    ap.add_argument("--boxes", default=None, help="comma list of box names")
    ap.add_argument("--correct", action="store_true", help="SI-correct from camera at each box")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--verify", action="store_true", help="SNAP-poll smoke test; no driving")
    ap.add_argument("--arrive-cm", type=float, default=7.0, help="arrival radius (cm)")
    ap.add_argument("--settle-s", type=float, default=0.8)
    ap.add_argument("--timeout-s", type=float, default=14.0, help="per-leg drive timeout")
    ap.add_argument("--out", default=str(_REPO / "host_tests" / "square_run_log.csv"))
    args = ap.parse_args()

    conn = SerialConnection(args.port) if args.port else SerialConnection()
    res = conn.connect()
    if res.get("error"):
        sys.exit(f"connect failed: {res['error']}")
    proto = NezhaProtocol(conn)
    png = proto.ping()
    if not png:
        sys.exit("PING failed — robot silent. Power-cycle and retry.")
    print(f"PING ok (robot_t={png[0]} ms, rtt={png[1]:.0f} ms)")
    # sTimeout uses the firmware default (500 ms); TIME-stopped commands
    # (G, TURN, D, T, RT) are exempt from the keepalive requirement after
    # sprint 024-003, so the 60 s override is no longer needed here.
    trackwidth_mm = _get_trackwidth(proto)
    print(f"trackwidth = {trackwidth_mm:.1f} mm")

    dc, cam = (None, None) if (args.no_camera or args.verify) else open_camera()
    x0_cm = y0_cm = yaw0 = None
    if dc is not None:
        p = robot_pose(dc, cam)
        if p is None:
            print("[camera] robot tag not seen — continuing without start fix")
        else:
            x0_cm, y0_cm, yaw0 = p
            h_cdeg = int(round((math.degrees(yaw0) + 90.0) * 100.0))
            proto.send(f"SI {int(round(x0_cm*10))} {int(round(y0_cm*10))} {h_cdeg}", 250)
            print(f"start pose: x={x0_cm:.1f}cm y={y0_cm:.1f}cm yaw={math.degrees(yaw0):.0f}deg -> SI")

    tlm_rows: list[dict] = []
    cam_rows: list[dict] = []
    t_start = time.monotonic()

    def snap_once():
        """One SNAP request/reply -> (TLMFrame|None, saw_done_G)."""
        r = conn.send("SNAP", read_ms=120, stop_token="TLM")
        tlm, done = None, False
        for ln in r.get("responses", []):
            if "done G" in ln:
                done = True
            t = parse_tlm(ln)
            if t is not None:
                tlm = t
        return tlm, done

    def log_tlm(tlm):
        row = {"host_t": time.monotonic() - t_start, "robot_t": tlm.t, "mode": tlm.mode}
        if tlm.enc:   row["enc_l"], row["enc_r"] = tlm.enc
        if tlm.pose:  row["pose_x"], row["pose_y"], row["pose_h"] = tlm.pose
        if tlm.otos:  row["otos_x"], row["otos_y"], row["otos_h"] = tlm.otos
        if tlm.twist: row["v"], row["omega"] = tlm.twist
        tlm_rows.append(row)
        return row

    def poll(duration_s, *, keepalive, target_mm=None, arrive_mm=70.0):
        """Poll SNAP + camera for duration_s; early-exit on arrival/done-G."""
        end = time.monotonic() + duration_s
        arrived = False
        while time.monotonic() < end:
            if keepalive:
                conn.send_fast("+")
            tlm, done = snap_once()
            if tlm is not None:
                row = log_tlm(tlm)
                if target_mm and "pose_x" in row:
                    if math.hypot(row["pose_x"] - target_mm[0], row["pose_y"] - target_mm[1]) <= arrive_mm:
                        arrived = True
            if dc is not None:
                cp = read_tag(dc, cam, timeout_s=0.0)
                if cp:
                    cam_rows.append({"host_t": time.monotonic() - t_start,
                                     "cam_x": cp[0], "cam_y": cp[1], "cam_yaw": cp[2]})
            if done or arrived:
                return True
        return False

    # ---- verify ----------------------------------------------------------- #
    if args.verify:
        print("VERIFY: polling SNAP 2.5s ...")
        poll(2.5, keepalive=False)
        _report_verify(tlm_rows)
        conn.disconnect()
        return

    # ---- drive the square ------------------------------------------------- #
    route = (args.boxes.split(",") if args.boxes else DEFAULT_RING)
    print(f"route: {route}")
    poll(0.5, keepalive=False)
    arrive_mm = args.arrive_cm * 10.0
    for name in route:
        if name not in SITES:
            print(f"  skip unknown box {name}"); continue
        bx_cm, by_cm = SITES[name]
        tgt = (bx_cm * 10.0, by_cm * 10.0)
        print(f"  -> {name} ({bx_cm},{by_cm} cm)")
        proto.send(f"G {int(tgt[0])} {int(tgt[1])} {args.speed}", 250)
        ok = poll(args.timeout_s, keepalive=True, target_mm=tgt, arrive_mm=arrive_mm)
        print(f"     {'reached' if ok else 'timeout'}")
        conn.send_fast("X")
        poll(args.settle_s, keepalive=False)
        if args.correct and dc is not None:
            p = robot_pose(dc, cam, n=4)
            if p:
                cx, cy, cyaw = p
                h_cdeg = int(round((math.degrees(cyaw) + 90.0) * 100.0))
                proto.send(f"SI {int(round(cx*10))} {int(round(cy*10))} {h_cdeg}", 150)
                print(f"     camera correct -> SI ({cx:.1f},{cy:.1f})")

    conn.send_fast("X")
    conn.disconnect()

    meta = {"trackwidth_mm": trackwidth_mm,
            "start_x_cm": x0_cm, "start_y_cm": y0_cm, "start_yaw_rad": yaw0,
            "speed": args.speed, "route": route}
    _save(args.out, tlm_rows, cam_rows, meta)
    print(f"\nlogged {len(tlm_rows)} TLM frames, {len(cam_rows)} camera frames -> {args.out}")
    print("plot:  uv run python tests/bench/plot_square.py")


def _get_trackwidth(proto, default=143.0) -> float:
    try:
        resp = proto.send("GET trackwidth", 250)
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
        print("  NO TLM frames received — telemetry not responding!")
        return
    have_otos = sum(1 for r in rows if "otos_x" in r)
    print(f"  SNAP frames={len(rows)}  with otos={have_otos}")
    last = rows[-1]
    print(f"  last: enc=({last.get('enc_l')},{last.get('enc_r')}) "
          f"pose=({last.get('pose_x')},{last.get('pose_y')},{last.get('pose_h')}) "
          f"otos=({last.get('otos_x')},{last.get('otos_y')},{last.get('otos_h')})")
    print("  OK: otos= field present." if have_otos else "  WARNING: no otos= field!")


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
    with open(p.with_name(p.stem + "_camera.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["host_t", "cam_x", "cam_y", "cam_yaw"])
        w.writeheader()
        for r in cam_rows:
            w.writerow(r)


if __name__ == "__main__":
    main()
