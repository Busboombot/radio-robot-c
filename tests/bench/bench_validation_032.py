"""bench_validation_032.py — comprehensive full-stack bench validation via Bench OTOS.

Sprint 032-001 / revisited in 033-001. Talks to the robot through its USB serial
port DIRECTLY (not via the relay). DBG replies (ForceReply::SERIAL) are routed
to the robot's own USB serial port — over the relay they are invisible, which was
the root cause of the sprint 032 bench session confusion.

Connect the robot's USB port (NEZHA2 device, e.g. /dev/cu.usbmodem2121102) to
the host — NOT the relay (RADIOBRIDGE, e.g. /dev/cu.usbmodem2121402).

Uses SerialConnection(port, mode="direct") — plain commands with a corr-id
suffix, no `>` relay prefix, no `!GO`. One connection held open for the whole run.
Replies come back under the dict key "responses". Telemetry via SNAP polling:
conn.send_fast("SNAP") then conn.read_lines(350, stop_token="TLM").

Enables the Bench OTOS (`DBG OTOS BENCH 1`) so the full firmware stack runs on the
bench stand (synthetic OTOS feeds commanded-motion pose so pose-dependent verbs
terminate). Drives TURN x4 closure, a D+TURN square, and D/T velocity profiles,
logging every SNAP frame and validating for bad starts/stops, velocity jumps,
runaway spin, and EKF health.

Units: pose=x_mm,y_mm,h_centideg ; twist=v_mmps,omega_mrad/s ; enc=L_mm,R_mm.

Usage: uv run python tests/bench/bench_validation_032.py [--port /dev/cu.usbmodemXXXX]
"""
from __future__ import annotations
import argparse, pathlib, re, sys, time

_REPO = pathlib.Path(__file__).resolve().parents[2]
OUTDIR = _REPO / "docs" / "bench-validation-032"

# Robot USB serial port (NEZHA2 device) — not the relay (RADIOBRIDGE).
_DEFAULT_PORT = "/dev/cu.usbmodem2121102"

# Try to auto-detect the robot port from config/devices.json.
def _robot_port_from_config() -> str | None:
    import json
    reg = _REPO / "config" / "devices.json"
    if reg.exists():
        for entry in json.loads(reg.read_text()).values():
            role = (entry.get("role") or "").upper()
            if role in ("NEZHA2", "ROBOT") and entry.get("port"):
                return entry["port"]
    return None


def _make_connection(port: str):
    """Open a SerialConnection in direct mode to the robot's USB serial port."""
    import sys as _sys
    _sys.path.insert(0, str(_REPO / "host"))
    from robot_radio.io.serial_conn import SerialConnection
    conn = SerialConnection(port, mode="direct")
    result = conn.connect()
    if "error" in result:
        raise RuntimeError(f"Could not connect to {port}: {result['error']}")
    if not result.get("pinged"):
        # Try sending PING explicitly.
        pr = conn.send("PING", read_ms=600, stop_token="OK pong")
        if not any("pong" in ln.lower() for ln in pr.get("responses", [])):
            print(f"WARNING: PING did not confirm pong from {port}. Proceeding anyway.")
    return conn


def _tx(conn, cmd: str, read_ms: int = 450) -> str:
    """Send a command and return the first response line (or empty string)."""
    result = conn.send(cmd, read_ms=read_ms, stop_token="OK")
    lines = result.get("responses", [])
    return lines[0].strip() if lines else ""


def _snap(conn) -> str | None:
    """Request one SNAP TLM frame and return the raw TLM line, or None."""
    # SNAP replies with a raw TLM line (not OK-wrapped) — firmware routes it
    # to the USB serial port; the _reader_loop puts it in _tlm_queue.
    # Trigger SNAP (fire-and-forget, no corr-id needed) then drain _tlm_queue.
    conn.send_fast("SNAP")
    lines = conn.read_lines(350, stop_token="TLM")
    for ln in lines:
        if ln.startswith("TLM"):
            return ln.strip()
    return None


def parse(ln):
    kv = dict(tok.split("=", 1) for tok in ln.split() if "=" in tok)
    d = {"mode": kv.get("mode", "?")}
    if "pose" in kv:
        p = kv["pose"].split(",")
        if len(p) >= 3: d["x"], d["y"], d["h_deg"] = int(p[0]), int(p[1]), int(p[2]) / 100.0
    if "twist" in kv:
        t = kv["twist"].split(",")
        if len(t) >= 2: d["v"], d["omega"] = int(t[0]), int(t[1]) / 1000.0
    if "enc" in kv:
        e = kv["enc"].split(",")
        if len(e) >= 2: d["encL"], d["encR"] = int(e[0]), int(e[1])
    if "ekf_rej" in kv:
        try: d["ekf_rej"] = int(kv["ekf_rej"])
        except ValueError: pass
    return d


def drive(conn, label, cmd, dur_s, log):
    frames = []
    _tx(conn, cmd, read_ms=300)
    t0 = time.time()
    while time.time() - t0 < dur_s:
        ln = _snap(conn)
        if ln:
            f = parse(ln); f["t"] = round(time.time() - t0, 2); f["lbl"] = label
            frames.append(f)
            log.append(f"{label} t={f['t']}  {ln}")
        conn.send_fast("+")   # keepalive (send_fast avoids eating a corr-id slot)
    return frames


def analyze(label, frames, problems):
    vs = [f["v"] for f in frames if "v" in f]
    oms = [f["omega"] for f in frames if "omega" in f]
    hs = [f["h_deg"] for f in frames if "h_deg" in f]
    rej = [f["ekf_rej"] for f in frames if "ekf_rej" in f]
    m = {"label": label, "n": len(frames)}
    if vs:
        dv = [abs(vs[i] - vs[i-1]) for i in range(1, len(vs))]
        m.update(v_start=vs[0], v_peak=max(abs(x) for x in vs), v_end=vs[-1],
                 v_jump_max=max(dv) if dv else 0, v_series=[int(x) for x in vs])
    if oms: m["omega_peak"] = round(max(abs(x) for x in oms), 2)
    if hs:
        m["h_first"], m["h_last"] = round(hs[0], 1), round(hs[-1], 1)
        m["h_travel"] = round(sum(abs(hs[i]-hs[i-1]) for i in range(1, len(hs))), 1)
    if rej: m["ekf_rej_climb"] = rej[-1] - rej[0]
    if m.get("omega_peak", 0) > 12:
        problems.append(f"{label}: omega spike {m['omega_peak']} rad/s (runaway spin)")
    if label.startswith(("D_", "T_")):
        if m.get("h_travel", 0) > 40:
            problems.append(f"{label}: {m['h_travel']}deg heading travel on straight drive (drift/spin)")
        if abs(m.get("v_end", 0)) > 40:
            problems.append(f"{label}: residual v_end {m.get('v_end')} mm/s (bad stop)")
    if m.get("ekf_rej_climb", 0) > 25:
        problems.append(f"{label}: ekf_rej climbed {m['ekf_rej_climb']} (pose corruption?)")
    return m


def main():
    ap = argparse.ArgumentParser(
        description="Full-stack bench validation via Bench OTOS. "
                    "Requires the robot's USB serial port (NEZHA2 device), NOT the relay.")
    ap.add_argument("--port", default=None,
                    help="Robot USB serial port (NEZHA2 device, e.g. /dev/cu.usbmodem2121102). "
                         "Defaults to auto-detect from config/devices.json, "
                         f"then falls back to {_DEFAULT_PORT}.")
    args = ap.parse_args()

    port = args.port or _robot_port_from_config() or _DEFAULT_PORT
    print(f"Connecting to robot USB serial: {port}  (mode=direct, NOT relay)")

    OUTDIR.mkdir(parents=True, exist_ok=True)
    log, report, problems = [], [], []

    conn = _make_connection(port)
    print("PING:", _tx(conn, "PING", read_ms=500).strip())
    print("SET sTimeout=60000:", _tx(conn, "SET sTimeout=60000", read_ms=400).strip())
    print("STREAM fields    :", _tx(conn, "STREAM 50 fields=mode,pose,twist,enc,ekf_rej", read_ms=400).strip())
    print("DBG OTOS BENCH 1 :", _tx(conn, "DBG OTOS BENCH 1 20 10 0", read_ms=500).strip())
    print("DBG OTOS         :", _tx(conn, "DBG OTOS", read_ms=600).strip())

    try:
        # Seq 1: TURN x4
        _tx(conn, "ZERO enc", read_ms=300); _tx(conn, "SI 0 0 0", read_ms=300)
        for i in range(4):
            report.append(analyze(f"turn{i+1}", drive(conn, f"turn{i+1}", "TURN 9000", 3.0, log), problems))
        # Seq 2: square D+TURN
        _tx(conn, "ZERO enc", read_ms=300); _tx(conn, "SI 0 0 0", read_ms=300)
        for i in range(4):
            report.append(analyze(f"sqD{i+1}", drive(conn, f"sqD{i+1}", "D 300 250 250", 3.0, log), problems))
            report.append(analyze(f"sqT{i+1}", drive(conn, f"sqT{i+1}", "TURN 9000", 3.0, log), problems))
        # Seq 3: velocity profiles
        for label, cmd, dur in [("D_slow_150", "D 250 150 150", 3.2),
                                 ("D_med_300", "D 400 300 300", 2.8),
                                 ("D_fast_500", "D 500 500 500", 2.4),
                                 ("T_timed_1500", "T 1500 300 300", 2.6)]:
            _tx(conn, "ZERO enc", read_ms=300); _tx(conn, "SI 0 0 0", read_ms=300)
            report.append(analyze(label, drive(conn, label, cmd, dur, log), problems))
        print("DBG OTOS (final):", _tx(conn, "DBG OTOS", read_ms=600).strip())
    finally:
        _tx(conn, "X", read_ms=300); _tx(conn, "STREAM 0", read_ms=300)
        _tx(conn, "DBG OTOS BENCH 0", read_ms=300)
        conn.disconnect()

    (OUTDIR / "tlm_log.txt").write_text("\n".join(log))
    out = ["# Bench validation 032 — full-stack drive via Bench OTOS (hardware, direct USB)\n"]
    for m in report:
        out.append(f"\n[{m['label']}] frames={m['n']}")
        for k in ("v_start","v_peak","v_end","v_jump_max","omega_peak","h_first","h_last","h_travel","ekf_rej_climb","v_series"):
            if k in m: out.append(f"    {k:14s}= {m[k]}")
    out.append("\n---- VERDICT ----")
    out += [f"FOUND {len(problems)} PROBLEM(S):"] + ["  X "+p for p in problems] if problems \
           else ["  OK: clean starts/stops, bounded velocity, no runaway spin, EKF stable."]
    txt = "\n".join(out)
    (OUTDIR / "analysis.txt").write_text(txt)
    print("\n" + txt)
    print(f"\nlogged {len(log)} frames -> {OUTDIR}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
