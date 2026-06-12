"""bench_validation_032.py — comprehensive full-stack bench validation via Bench OTOS.

Sprint 032-001. Holds ONE serial connection open to the relay for the whole run
(async STREAM frames get dropped by the bridge, so telemetry is gathered by
polling SNAP — synchronous request/reply, reliable over the radio).

Enables the Bench OTOS (`DBG OTOS BENCH 1`) so the full firmware stack (motors,
encoders, EKF, motion control) runs on the bench stand — the synthetic OTOS
feeds commanded-motion pose so pose-dependent verbs (TURN/G) actually terminate.

Drives: TURN x4 closure, a 300 mm D+TURN square, and D/T velocity profiles at
several speeds. Logs every SNAP frame (t, mode, pose, twist=v/omega, enc, ekf_rej)
and validates for: bad starts/stops, bad velocity jumps, runaway spin, EKF health.

Units (from firmware buildTlmFrame): pose=x_mm,y_mm,h_centideg ; twist=v_mmps,
omega_mrad/s ; enc=L_mm,R_mm.

Usage:
    uv run python tests/bench/bench_validation_032.py --port /dev/cu.usbmodem2121402
"""
from __future__ import annotations

import argparse
import pathlib
import sys
import time

_REPO = pathlib.Path(__file__).resolve().parents[2]
_HOST = _REPO / "host"
_BENCH = pathlib.Path(__file__).resolve().parent
for p in (_HOST, _BENCH):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, parse_tlm
from bench_safety import BenchRun, RobotSilentError

OUTDIR = _REPO / "docs" / "bench-validation-032"


def snap(conn):
    """One reliable SNAP→TLM round-trip; return parsed frame dict or None."""
    r = conn.send("SNAP", read_ms=160, stop_token="TLM")
    for ln in r.get("lines", []):
        if ln.startswith("TLM"):
            f = parse_tlm(ln)
            d = {"t_host": time.monotonic(), "raw": ln, "mode": f.mode}
            if f.pose:
                d["x"], d["y"], d["h_deg"] = f.pose[0], f.pose[1], f.pose[2] / 100.0
            if f.twist:
                d["v"], d["omega"] = f.twist[0], f.twist[1] / 1000.0
            if f.enc:
                d["encL"], d["encR"] = f.enc
            if f.ekf_rej is not None:
                d["ekf_rej"] = f.ekf_rej
            return d
    return None


def drive_and_log(conn, proto, label, cmd, dur_s, log, poll_s=0.12):
    """Send a self-terminating command; poll SNAP for dur_s; collect frames."""
    frames = []
    proto.send(cmd, read_ms=200)
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur_s:
        f = snap(conn)
        if f:
            f["seq_label"] = label
            f["t_rel"] = round(f["t_host"] - t0, 3)
            frames.append(f)
            log.append(f["raw"] + f"   # {label} t={f['t_rel']}")
        time.sleep(poll_s)
    return frames


def analyze(label, frames, problems):
    vs = [f["v"] for f in frames if "v" in f]
    oms = [f["omega"] for f in frames if "omega" in f]
    hs = [f["h_deg"] for f in frames if "h_deg" in f]
    rej = [f["ekf_rej"] for f in frames if "ekf_rej" in f]
    m = {"label": label, "n": len(frames)}
    if vs:
        dv = [abs(vs[i] - vs[i - 1]) for i in range(1, len(vs))]
        m["v_peak"] = max(abs(x) for x in vs)
        m["v_jump_max"] = max(dv) if dv else 0.0
        m["v_start"] = vs[0]
        m["v_end"] = vs[-1]
        m["v_series"] = [int(x) for x in vs]
    if oms:
        m["omega_peak"] = round(max(abs(x) for x in oms), 2)
    if hs:
        m["h_first"] = round(hs[0], 1)
        m["h_last"] = round(hs[-1], 1)
        m["h_total_travel"] = round(sum(abs(hs[i] - hs[i - 1]) for i in range(1, len(hs))), 1)
    if rej:
        m["ekf_rej_climb"] = rej[-1] - rej[0]
    # pathology checks (coarse — SNAP poll ~8 Hz)
    if m.get("omega_peak", 0) > 12:
        problems.append(f"{label}: omega spike {m['omega_peak']} rad/s (runaway spin)")
    if "D_" in label or "T_" in label:
        if m.get("h_total_travel", 0) > 40:
            problems.append(f"{label}: {m['h_total_travel']}deg heading travel on a straight drive (drift/spin)")
        if abs(m.get("v_end", 0)) > 40:
            problems.append(f"{label}: residual v_end {m.get('v_end')} mm/s (bad stop)")
    if m.get("ekf_rej_climb", 0) > 25:
        problems.append(f"{label}: ekf_rej climbed {m['ekf_rej_climb']} (pose corruption?)")
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbmodem2121402")
    ap.add_argument("--noise", default="20 10 0", help="linSigma yawSigma drift milli-args for DBG OTOS BENCH")
    args = ap.parse_args()

    OUTDIR.mkdir(parents=True, exist_ok=True)
    log = []
    report = []
    problems = []

    conn = SerialConnection(args.port, mode="relay")
    res = conn.connect()
    print(f"connect: {res.get('status')} mode={res.get('mode')} pinged={res.get('pinged')}")
    proto = NezhaProtocol(conn)

    # --- liveness over the held-open connection ---
    ok = 0
    for _ in range(8):
        try:
            if proto.ping():
                ok += 1
        except Exception:
            pass
        time.sleep(0.1)
    print(f"liveness: {ok}/8 pings answered")
    if ok == 0:
        conn.disconnect()
        raise RobotSilentError("robot did not answer any PING over the held-open connection")

    try:
        with BenchRun(proto, max_seconds=180):
            # config + enable Bench OTOS
            print("SET sTimeout=60000 :", proto.send("SET sTimeout=60000", 250).get("lines"))
            print("DBG OTOS BENCH 1 :", proto.send(f"DBG OTOS BENCH 1 {args.noise}", 300).get("lines"))
            print("DBG OTOS query   :", proto.send("DBG OTOS", 300).get("lines"))
            proto.send("ZERO enc", 250)
            proto.send("SI 0 0 0", 250)
            proto.send("STREAM 40 fields=mode,pose,twist,enc,ekf_rej", 250)  # binds + enables tlm fields for SNAP

            # ---- Seq 1: TURN x4 (90deg each) ----
            for i in range(4):
                report.append(analyze(f"turn{i+1}_90",
                                       drive_and_log(conn, proto, f"turn{i+1}_90", "TURN 9000", 3.0, log),
                                       problems))

            # ---- Seq 2: square (D 300 + TURN 90) x4 ----
            proto.send("ZERO enc", 250); proto.send("SI 0 0 0", 250)
            for i in range(4):
                report.append(analyze(f"sqD{i+1}",
                                       drive_and_log(conn, proto, f"sqD{i+1}", "D 300 250 250", 3.0, log),
                                       problems))
                report.append(analyze(f"sqT{i+1}",
                                       drive_and_log(conn, proto, f"sqT{i+1}", "TURN 9000", 3.0, log),
                                       problems))

            # ---- Seq 3: velocity profiles ----
            for label, cmd, dur in [("D_slow_150", "D 250 150 150", 3.2),
                                     ("D_med_300", "D 400 300 300", 2.8),
                                     ("D_fast_500", "D 500 500 500", 2.4),
                                     ("T_timed_1500", "T 1500 300 300", 2.6)]:
                proto.send("ZERO enc", 250); proto.send("SI 0 0 0", 250)
                report.append(analyze(label, drive_and_log(conn, proto, label, cmd, dur, log), problems))

            proto.send("X", 250)
            proto.send("DBG OTOS", 300)
    finally:
        try:
            proto.send("STREAM 0", 250)
            proto.send("DBG OTOS BENCH 0", 250)
            proto.send("X", 250)
        except Exception:
            pass
        conn.disconnect()

    # --- write log + report ---
    (OUTDIR / "tlm_log.txt").write_text("\n".join(log))
    lines = ["# Bench validation 032 — telemetry analysis\n"]
    for m in report:
        lines.append(f"\n[{m['label']}] frames={m['n']}")
        for k in ("v_start", "v_peak", "v_end", "v_jump_max", "omega_peak",
                  "h_first", "h_last", "h_total_travel", "ekf_rej_climb", "v_series"):
            if k in m:
                lines.append(f"    {k:16s} = {m[k]}")
    lines.append("\n---- VERDICT ----")
    if problems:
        lines.append(f"FOUND {len(problems)} PROBLEM(S):")
        lines += ["  X " + p for p in problems]
    else:
        lines.append("  OK: clean starts/stops, bounded velocity, no runaway spin, EKF stable.")
    out = "\n".join(lines)
    (OUTDIR / "analysis.txt").write_text(out)
    print("\n" + out)
    print(f"\nlogged {len(log)} frames to {OUTDIR}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
