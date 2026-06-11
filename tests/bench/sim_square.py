"""sim_square.py — drive the playfield square in the SIMULATOR and emit the
same CSV schema as square_run.py, so the plotting pipeline can be validated
end-to-end without the physical robot.

Uses the firmware `G` arc go-to (same command the hardware run uses), with the
EKF fusion enabled, so this is a faithful dry-run of the real square_run.py.
Writes host_tests/square_run_log.csv + _camera.csv (exact pose = camera truth).
"""
from __future__ import annotations

import math
import pathlib
import sys

_REPO = pathlib.Path(__file__).resolve().parents[2]
for p in (str(_REPO), str(_REPO / "host")):
    if p not in sys.path:
        sys.path.insert(0, p)

from robot_radio.io.sim_conn import SimConnection

# 4-corner colored-box square (cm, A1-centred), plus return to start corner.
CORNERS = [("purple-NW", -35, 24), ("orange-NE", 35, 24),
           ("green-SE", 35, -24), ("blue-SW", -35, -24), ("purple-NW", -35, 24)]
SPEED = 160
TRACKWIDTH_MM = 143.0


def main() -> None:
    c = SimConnection()
    r = c.connect()
    if r.get("error"):
        sys.exit(r["error"])
    c.send("SET sTimeout=60000")
    c.set_slip(0.010, 0.05)
    c.set_encoder_noise(0.10)
    c.enable_otos_model()
    c.set_otos_noise(linear=0.02, yaw=0.04)
    c.enable_otos_fusion(True)

    # The sim robot physically starts at the world origin (0,0) heading 0.
    # SI/OTOS MUST be set to that actual pose (on hardware this is the camera
    # reading); setting it to anything else makes the firmware drive on a lie.
    s0 = c.get_state()
    x0_mm, y0_mm, h0 = s0["exact_pose_x"], s0["exact_pose_y"], s0["exact_pose_h"]
    c.send(f"SI {int(round(x0_mm))} {int(round(y0_mm))} {int(round(h0 * 5729.578))}")
    c.set_otos_pose(x0_mm, y0_mm, h0)
    start_x_cm = x0_mm / 10.0
    start_y_cm = y0_mm / 10.0
    start_yaw = h0 - math.pi / 2

    rows, cam = [], []
    t0 = 0.0

    def sample():
        st = c.get_state()
        rows.append({
            "host_t": st["time_ms"] / 1000.0, "robot_t": int(st["time_ms"]), "mode": "G",
            "enc_l": st["enc_l"], "enc_r": st["enc_r"],
            "pose_x": int(st["pose_x"]), "pose_y": int(st["pose_y"]),
            "pose_h": int(st["pose_h"] * 5729.578),
            "otos_x": int(st["otos_x"]), "otos_y": int(st["otos_y"]),
            "otos_h": int(st["otos_h"] * 5729.578),
            "v": int(st.get("vel_l", 0)), "omega": 0,
        })
        cam.append({"host_t": st["time_ms"] / 1000.0,
                    "cam_x": st["exact_pose_x"] / 10.0,
                    "cam_y": st["exact_pose_y"] / 10.0,
                    "cam_yaw": st["exact_pose_h"] - math.pi / 2})

    for name, bx, by in CORNERS[1:]:
        c.send(f"G {int(bx*10)} {int(by*10)} {SPEED}")
        done = False
        for _ in range(400):  # up to ~10 s/leg at 24 ms tick
            c.send_fast("+")
            evts = c.tick(24)
            sample()
            if any("done G" in e for e in evts):
                done = True
                break
        c.send_fast("X")
        for _ in range(15):
            c.tick(24); sample()
        print(f"  {name}: {'arrived' if done else 'timeout'} at "
              f"pose=({rows[-1]['pose_x']},{rows[-1]['pose_y']}) "
              f"truth=({cam[-1]['cam_x']:.0f},{cam[-1]['cam_y']:.0f}) cm")

    c.disconnect()

    # write CSVs in square_run.py schema
    import csv
    meta = {"trackwidth_mm": TRACKWIDTH_MM, "start_x_cm": start_x_cm,
            "start_y_cm": start_y_cm, "start_yaw_rad": start_yaw,
            "speed": SPEED, "route": [n for n, _, _ in CORNERS]}
    out = _REPO / "host_tests" / "square_run_log.csv"
    cols = ["host_t", "robot_t", "mode", "enc_l", "enc_r", "pose_x", "pose_y",
            "pose_h", "otos_x", "otos_y", "otos_h", "v", "omega"]
    with open(out, "w", newline="") as f:
        f.write("# " + str(meta) + "\n")
        w = csv.DictWriter(f, fieldnames=cols); w.writeheader()
        for r in rows:
            w.writerow(r)
    with open(out.with_name("square_run_log_camera.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["host_t", "cam_x", "cam_y", "cam_yaw"]); w.writeheader()
        for r in cam:
            w.writerow(r)
    print(f"\nwrote {len(rows)} rows -> {out}")


if __name__ == "__main__":
    main()
