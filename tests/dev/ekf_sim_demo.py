"""ekf_sim_demo.py — drive the REAL firmware EKF in the host simulator.

This is NOT a hand-authored fixture. It loads ``libfirmware_host`` (the actual
firmware compiled for the host via MockHAL) and drives the robot through a
physical S-curve-plus-circle trajectory using ``VW`` commands. The simulator
provides:

  * ground truth      — ExactPoseTracker oracle (where the robot physically is)
  * noisy encoders    — MockMotor with slip + Gaussian encoder noise
  * noisy OTOS         — MockOtosSensor integrating true velocity + noise
  * firmware pose      — the on-board EKF estimate (sim_get_pose_*)

It runs the SAME drive twice on the SAME (fixed-seed) noise realization:

  * pass 1 — OTOS fusion OFF  → firmware pose is encoder-only dead reckoning
  * pass 2 — OTOS fusion ON   → firmware pose is the fused EKF estimate

so the encoder-only and fused trajectories are directly comparable.

Run standalone to produce a plot + error table:
    uv run python tests/dev/ekf_sim_demo.py
"""

from __future__ import annotations

import math
import pathlib
import sys

# --- make the host package importable -------------------------------------
_REPO = pathlib.Path(__file__).resolve().parents[2]
_HOST = _REPO / "host"
if str(_HOST) not in sys.path:
    sys.path.insert(0, str(_HOST))

from robot_radio.io.sim_conn import SimConnection  # noqa: E402

# --- trajectory: (omega_mrad_s, duration_ms) segments at constant v --------
V_MMPS = 180
STEP_MS = 48  # control step (2 sim ticks of 24 ms)

#  warm-up straight, then S (left arc, right arc, left, right), then a full circle
SEGMENTS = [
    (0,     600),   # straight warm-up
    (+1300, 1900),  # arc left   ┐
    (-1300, 1900),  # arc right  ┘ first S
    (+1300, 1900),  # arc left   ┐
    (-1300, 1900),  # arc right  ┘ second S
    (+1500, 4400),  # full circle (~one loop)
]


def drive(fusion: bool) -> list[dict]:
    """Run the trajectory once; return the per-tick state log."""
    conn = SimConnection()
    res = conn.connect()
    if "error" in res:
        raise RuntimeError(res["error"])
    conn.send("SET sTimeout=60000")          # keep the watchdog out of the way
    conn.set_slip(0.010, 0.05)               # 1% straight, +5% on turns
    conn.set_encoder_noise(0.10)             # 0.10 mm/tick Gaussian
    conn.enable_otos_model()                 # OTOS integrates true vel + noise
    conn.set_otos_noise(linear=0.02, yaw=0.04)
    if fusion:
        conn.enable_otos_fusion(True)        # run firmware Robot::otosCorrect()

    conn.clear_state_log()
    for omega_mrad, dur_ms in SEGMENTS:
        elapsed = 0
        while elapsed < dur_ms:
            conn.send_fast(f"VW {V_MMPS} {omega_mrad}")
            conn.send_fast("+")              # keepalive
            conn.tick(STEP_MS)
            elapsed += STEP_MS
    log = list(conn.state_log)
    conn.disconnect()
    return log


def _err(ex, ey, x, y):
    return math.hypot(ex - x, ey - y)


def run_demo() -> dict:
    """Run both passes and return aligned trajectory arrays + error stats."""
    log_dr = drive(fusion=False)   # encoder-only firmware pose
    log_ekf = drive(fusion=True)   # fused firmware pose
    n = min(len(log_dr), len(log_ekf))

    out = {
        "t":        [log_ekf[i]["time_ms"] for i in range(n)],
        "truth_x":  [log_ekf[i]["exact_pose_x"] for i in range(n)],
        "truth_y":  [log_ekf[i]["exact_pose_y"] for i in range(n)],
        "enc_x":    [log_dr[i]["pose_x"] for i in range(n)],   # fusion OFF pose
        "enc_y":    [log_dr[i]["pose_y"] for i in range(n)],
        "otos_x":   [log_ekf[i]["otos_x"] for i in range(n)],
        "otos_y":   [log_ekf[i]["otos_y"] for i in range(n)],
        "fused_x":  [log_ekf[i]["pose_x"] for i in range(n)],  # fusion ON pose
        "fused_y":  [log_ekf[i]["pose_y"] for i in range(n)],
    }
    # error series vs ground truth
    for tag in ("enc", "otos", "fused"):
        out[f"{tag}_err"] = [
            _err(out["truth_x"][i], out["truth_y"][i], out[f"{tag}_x"][i], out[f"{tag}_y"][i])
            for i in range(n)
        ]
    return out


def _summary(d: dict) -> str:
    def stats(tag):
        e = d[f"{tag}_err"]
        return sum(e) / len(e), max(e), e[-1]
    rows = []
    rows.append(f"{'source':<22}{'mean err':>10}{'max err':>10}{'final err':>11}")
    rows.append("-" * 53)
    for tag, name in (("enc", "encoder-only (DR)"),
                      ("otos", "raw OTOS"),
                      ("fused", "firmware EKF fused")):
        m, mx, f = stats(tag)
        rows.append(f"{name:<22}{m:>9.1f} {mx:>9.1f} {f:>10.1f}   (mm)")
    return "\n".join(rows)


def main() -> None:
    d = run_demo()
    print(f"Ticks: {len(d['t'])}   sim time: {d['t'][-1]/1000:.1f} s")
    print(_summary(d))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 6), gridspec_kw={"width_ratios": [3, 2]})

    ax.plot(d["truth_x"], d["truth_y"], "k-", lw=2.5, label="Ground truth", zorder=5)
    ax.plot(d["enc_x"], d["enc_y"], color="tab:blue", lw=1.3, alpha=0.9, label="Encoder-only (firmware DR)")
    ax.plot(d["otos_x"], d["otos_y"], color="tab:green", lw=1.0, alpha=0.7, label="Raw OTOS")
    ax.plot(d["fused_x"], d["fused_y"], color="tab:red", lw=1.6, label="Firmware EKF fused")
    ax.plot(d["truth_x"][0], d["truth_y"][0], "ko", ms=9)
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
    ax.set_title("S-curve + circle — real firmware in simulator\n(truth vs encoder vs OTOS vs EKF-fused)")
    ax.legend(loc="best"); ax.grid(True, alpha=0.3)

    ax2.plot([t/1000 for t in d["t"]], d["enc_err"], color="tab:blue", label="Encoder-only")
    ax2.plot([t/1000 for t in d["t"]], d["otos_err"], color="tab:green", label="Raw OTOS")
    ax2.plot([t/1000 for t in d["t"]], d["fused_err"], color="tab:red", lw=1.8, label="EKF fused")
    ax2.set_xlabel("time (s)"); ax2.set_ylabel("position error vs truth (mm)")
    ax2.set_title("Localisation error vs time")
    ax2.legend(); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out_png = _REPO / "host_tests" / "ekf_sim_scurve.png"
    fig.savefig(out_png, dpi=110)
    print(f"\nPlot saved: {out_png}")


if __name__ == "__main__":
    main()
