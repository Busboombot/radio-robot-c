"""plot_square.py — plot a real-hardware (or sim) square run.

Reads the CSV produced by square_run.py and draws the same four lines as the
EKF sim notebook, but from logged data:

  * camera ground truth  (overhead AprilTag)        — black
  * encoder-only          (host-integrated from enc) — blue
  * raw OTOS              (otos= telemetry)           — green
  * firmware EKF fused    (pose= telemetry)           — red

plus localisation error vs the camera truth over time.

Usage:
    uv run python tests/bench/plot_square.py
    uv run python tests/bench/plot_square.py --log host_tests/square_run_log.csv
"""

from __future__ import annotations

import argparse
import ast
import csv
import math
import pathlib

# colored box sites (cm, A1-centred) for plot markers
SITES = {
    "purple-NW": (-35, 24), "black-N": (0, 24), "orange-NE": (35, 24),
    "red-E": (35, 0), "green-SE": (35, -24), "magenta-S": (0, -24),
    "blue-SW": (-35, -24), "red-W": (-35, 0),
}
_BOX_COLOR = {"purple-NW": "purple", "black-N": "black", "orange-NE": "orange",
              "red-E": "red", "green-SE": "green", "magenta-S": "magenta",
              "blue-SW": "blue", "red-W": "darkred"}


def _read_meta_and_rows(log_path: pathlib.Path):
    meta = {}
    rows = []
    with open(log_path) as f:
        first = f.readline()
        if first.startswith("#"):
            try:
                meta = ast.literal_eval(first[1:].strip())
            except Exception:  # noqa: BLE001
                meta = {}
        else:
            f.seek(0)
        for r in csv.DictReader(f):
            rows.append(r)
    return meta, rows


def _f(row, key):
    v = row.get(key, "")
    return float(v) if v not in ("", None) else None


def _encoder_only(rows, meta):
    """Integrate the streamed encoders host-side (cm), from the start pose."""
    tw = float(meta.get("trackwidth_mm") or 143.0)
    sx = meta.get("start_x_cm"); sy = meta.get("start_y_cm"); syaw = meta.get("start_yaw_rad")
    x = (float(sx) * 10.0) if sx is not None else None
    y = (float(sy) * 10.0) if sy is not None else None
    th = (float(syaw) + math.pi / 2) if syaw is not None else math.pi / 2
    prevL = prevR = None
    xs, ys, ts = [], [], []
    for r in rows:
        eL, eR = _f(r, "enc_l"), _f(r, "enc_r")
        if eL is None or eR is None:
            continue
        if prevL is None:
            prevL, prevR = eL, eR
            # seed start from first fused pose if no camera start
            if x is None:
                x = _f(r, "pose_x") or 0.0
                y = _f(r, "pose_y") or 0.0
            xs.append(x / 10.0); ys.append(y / 10.0); ts.append(_f(r, "host_t"))
            continue
        dL, dR = eL - prevL, eR - prevR
        prevL, prevR = eL, eR
        dC = (dL + dR) * 0.5
        dTh = (dR - dL) / tw
        x += dC * math.cos(th + dTh / 2)
        y += dC * math.sin(th + dTh / 2)
        th += dTh
        xs.append(x / 10.0); ys.append(y / 10.0); ts.append(_f(r, "host_t"))
    return ts, xs, ys


def _interp(ts_src, vs_src, ts_dst):
    """Linear-interp vs_src(ts_src) onto ts_dst."""
    out = []
    j = 0
    for t in ts_dst:
        while j + 1 < len(ts_src) and ts_src[j + 1] < t:
            j += 1
        if j + 1 >= len(ts_src):
            out.append(vs_src[-1]); continue
        t0, t1 = ts_src[j], ts_src[j + 1]
        if t1 == t0:
            out.append(vs_src[j]); continue
        a = (t - t0) / (t1 - t0)
        out.append(vs_src[j] * (1 - a) + vs_src[j + 1] * a)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    repo = pathlib.Path(__file__).resolve().parents[2]
    ap.add_argument("--log", default=str(repo / "host_tests" / "square_run_log.csv"))
    ap.add_argument("--out", default=str(repo / "host_tests" / "square_run.png"))
    args = ap.parse_args()

    log_path = pathlib.Path(args.log)
    meta, rows = _read_meta_and_rows(log_path)
    cam_path = log_path.with_name(log_path.stem + "_camera.csv")

    # telemetry trajectories (cm)
    t_tlm = [_f(r, "host_t") for r in rows if _f(r, "pose_x") is not None]
    pose_x = [(_f(r, "pose_x")) / 10.0 for r in rows if _f(r, "pose_x") is not None]
    pose_y = [(_f(r, "pose_y")) / 10.0 for r in rows if _f(r, "pose_x") is not None]
    otos_x = [(_f(r, "otos_x")) / 10.0 for r in rows if _f(r, "otos_x") is not None]
    otos_y = [(_f(r, "otos_y")) / 10.0 for r in rows if _f(r, "otos_x") is not None]
    enc_t, enc_x, enc_y = _encoder_only(rows, meta)

    # camera truth (cm)
    cam_t = cam_x = cam_y = None
    if cam_path.exists():
        ct, cx, cy = [], [], []
        with open(cam_path) as f:
            for r in csv.DictReader(f):
                try:
                    ct.append(float(r["host_t"])); cx.append(float(r["cam_x"])); cy.append(float(r["cam_y"]))
                except (ValueError, KeyError):
                    pass
        if ct:
            cam_t, cam_x, cam_y = ct, cx, cy

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 6.5), gridspec_kw={"width_ratios": [3, 2]})

    # --- trajectory ---
    for name, (bx, by) in SITES.items():
        ax.plot(bx, by, "s", color=_BOX_COLOR[name], ms=11, alpha=0.5, mec="k")
    if cam_x:
        ax.plot(cam_x, cam_y, "k-", lw=2.5, label="Camera truth", zorder=5)
    if enc_x:
        ax.plot(enc_x, enc_y, color="tab:blue", lw=1.4, label="Encoder-only (integrated)")
    if otos_x:
        ax.plot(otos_x, otos_y, color="tab:green", lw=1.1, alpha=0.8, label="Raw OTOS")
    if pose_x:
        ax.plot(pose_x, pose_y, color="tab:red", lw=1.7, label="Firmware EKF fused")
    if cam_x:
        ax.plot(cam_x[0], cam_y[0], "ko", ms=9)
    ax.set_aspect("equal"); ax.set_xlabel("x (cm, A1-centred)"); ax.set_ylabel("y (cm)")
    ax.set_title("Playfield square — real hardware\n(camera vs encoder vs OTOS vs EKF-fused)")
    ax.legend(loc="best", fontsize=8); ax.grid(True, alpha=0.3)

    # --- error vs time (needs camera truth) ---
    if cam_x and t_tlm:
        cxi = _interp(cam_t, cam_x, t_tlm)
        cyi = _interp(cam_t, cam_y, t_tlm)
        fused_err = [math.hypot(cxi[i] - pose_x[i], cyi[i] - pose_y[i]) * 10 for i in range(len(t_tlm))]
        ax2.plot(t_tlm, fused_err, color="tab:red", lw=1.8, label="EKF fused")
        if otos_x and len(otos_x) == len(t_tlm):
            otos_err = [math.hypot(cxi[i] - otos_x[i], cyi[i] - otos_y[i]) * 10 for i in range(len(t_tlm))]
            ax2.plot(t_tlm, otos_err, color="tab:green", label="Raw OTOS")
        if enc_x:
            cxe = _interp(cam_t, cam_x, enc_t); cye = _interp(cam_t, cam_y, enc_t)
            enc_err = [math.hypot(cxe[i] - enc_x[i], cye[i] - enc_y[i]) * 10 for i in range(len(enc_t))]
            ax2.plot(enc_t, enc_err, color="tab:blue", label="Encoder-only")
        ax2.set_xlabel("time (s)"); ax2.set_ylabel("error vs camera truth (mm)")
        ax2.set_title("Localisation error vs time"); ax2.legend(); ax2.grid(True, alpha=0.3)

        def _stat(e):
            return sum(e) / len(e), max(e)
        print("Error vs camera truth (mm):")
        print(f"  EKF fused    mean {_stat(fused_err)[0]:6.1f}  max {_stat(fused_err)[1]:6.1f}")
        if enc_x:
            print(f"  encoder-only mean {_stat(enc_err)[0]:6.1f}  max {_stat(enc_err)[1]:6.1f}")
    else:
        ax2.text(0.5, 0.5, "no camera truth in log\n(error-vs-time needs camera)",
                 ha="center", va="center", transform=ax2.transAxes)

    plt.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
