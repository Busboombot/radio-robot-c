#!/usr/bin/env python3
"""
bench_validation_033.py — comprehensive on-robot bench validation for the
sprint-033 firmware fixes, using the Bench OTOS synthetic sensor.

Talks to the robot's OWN USB serial port directly with RAW pyserial (DTR
asserted on open).  We do NOT use robot_radio.SerialConnection here: its
direct mode only surfaces OK/ERR reply lines and silently drops the TLM frames
that SNAP returns, so it cannot read telemetry.  Raw read captures everything.

Validates:
  - 033-002: DBG OTOS BENCH 1 engages bench mode (bench=1).
  - 033-003: twist (v, omega) is non-zero while driving / turning (encoder
             velocity fuses into the EKF even with the real OTOS off-surface).
  - 033-004: a D following a TURN (no ZERO enc) travels its full distance
             instead of instant-completing.
  - General health: clean starts/stops, no wild velocity jumps, no
             out-of-control spinning (bounded heading + omega).

Run on the bench (robot on a stand, wheels free to spin):
    uv run python tests/bench/bench_validation_033.py
Optional port override:
    uv run python tests/bench/bench_validation_033.py /dev/cu.usbmodem2121102
"""
import sys
import time

import serial  # pyserial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2121102"
BAUD = 115200


# ---------------------------------------------------------------------------
# Raw serial transport
# ---------------------------------------------------------------------------

class Bench:
    def __init__(self, port=PORT, baud=BAUD):
        # DTR asserted by default (do NOT pass dtr=False — that yields silence).
        self.p = serial.Serial(port, baud, timeout=0.2)
        time.sleep(1.5)            # let the board boot / announce
        self.p.reset_input_buffer()

    def send(self, cmd, read_ms=500):
        """Write a command, read for read_ms, return list of reply lines."""
        self.p.reset_input_buffer()
        self.p.write((cmd + "\n").encode())
        self.p.flush()
        deadline = time.time() + read_ms / 1000.0
        buf = b""
        while time.time() < deadline:
            chunk = self.p.read(4096)
            if chunk:
                buf += chunk
            else:
                time.sleep(0.01)
        return [ln for ln in buf.decode(errors="replace").splitlines() if ln.strip()]

    def snap(self):
        """Return the SNAP TLM frame as a dict of parsed fields, or None."""
        for ln in self.send("SNAP", read_ms=300):
            if ln.startswith("TLM"):
                return _parse_tlm(ln)
        return None

    def close(self):
        self.p.close()


def _parse_tlm(line):
    """Parse 'TLM t=.. mode=I enc=L,R pose=x,y,h vel=.. twist=v,w ..' → dict."""
    out = {"raw": line}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def _pair(s):
    """'12,-3' → (12.0, -3.0); tolerant of empties."""
    try:
        a, b = s.split(",")[:2]
        return float(a or 0), float(b or 0)
    except Exception:
        return (0.0, 0.0)


def _triple(s):
    try:
        a, b, c = s.split(",")[:3]
        return float(a or 0), float(b or 0), float(c or 0)
    except Exception:
        return (0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Telemetry capture during a motion command
# ---------------------------------------------------------------------------

def capture(b, label, duration_s, period_s=0.1):
    """Poll SNAP for duration_s; return list of parsed frames."""
    frames = []
    t_end = time.time() + duration_s
    while time.time() < t_end:
        f = b.snap()
        if f:
            f["_t"] = time.time()
            frames.append(f)
        time.sleep(period_s)
    return frames


def summarize(frames):
    """Extract trajectories of interest from a frame list."""
    v, omega, encL, encR, hx = [], [], [], [], []
    modes = []
    for f in frames:
        vv, ww = _pair(f.get("twist", "0,0"))
        el, er = _pair(f.get("enc", "0,0"))
        px, py, ph = _triple(f.get("pose", "0,0,0"))
        v.append(vv); omega.append(ww)
        encL.append(el); encR.append(er); hx.append(ph)
        modes.append(f.get("mode", "?"))
    return dict(v=v, omega=omega, encL=encL, encR=encR, h=hx, modes=modes)


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))


def main():
    b = Bench()
    print(f"== Bench validation 033 on {PORT} ==\n")

    # ---- Preflight: liveness + VER gate ----
    ping = b.send("PING")
    ver = b.send("VER")
    idr = b.send("ID")
    print("PING:", ping)
    print("VER :", ver)
    print("ID  :", idr)
    alive = any("pong" in x for x in ping)
    check("robot alive (PING)", alive)
    if not alive:
        print("\nABORT: robot not responding on", PORT)
        b.close()
        return

    # ---- 033-002: bench OTOS engages ----
    print("\n-- 033-002: DBG OTOS BENCH --")
    r = b.send("DBG OTOS BENCH 1")
    print("BENCH 1:", r)
    check("bench OTOS engages (bench=1)", any("bench=1" in x for x in r), str(r))

    # ---- Baseline ----
    b.send("SET sTimeout=60000")
    b.send("ZERO enc")
    base = b.snap()
    print("baseline SNAP:", base.get("enc"), "twist=", base.get("twist") if base else None)

    # ---- 033-003: twist non-zero while driving straight ----
    print("\n-- 033-003: twist while driving (T 150 150 2500) --")
    b.send("ZERO enc")
    b.send("T 150 150 2500")
    fr = capture(b, "straight", 2.6, 0.08)
    s = summarize(fr)
    vmax = max((abs(x) for x in s["v"]), default=0.0)
    encmax = max((abs(x) for x in s["encL"] + s["encR"]), default=0.0)
    print(f"   v samples: {[round(x) for x in s['v']]}")
    print(f"   encL last={s['encL'][-1] if s['encL'] else 0:.0f} encR last={s['encR'][-1] if s['encR'] else 0:.0f}")
    check("033-003 twist.v non-zero while driving", vmax > 30.0, f"max|v|={vmax:.0f} mm/s")
    check("encoders advanced while driving", encmax > 30.0, f"max|enc|={encmax:.0f} mm")
    b.send("X"); time.sleep(0.5)

    # ---- 033-003: omega non-zero while turning ----
    print("\n-- 033-003: omega while turning (T 150 -150 2000) --")
    b.send("T 150 -150 2000")
    fr = capture(b, "spin", 2.1, 0.08)
    s = summarize(fr)
    wmax = max((abs(x) for x in s["omega"]), default=0.0)
    print(f"   omega samples (mrad/s): {[round(x) for x in s['omega']]}")
    check("033-003 twist.omega non-zero while turning", wmax > 50.0, f"max|omega|={wmax:.0f} mrad/s")
    b.send("X"); time.sleep(0.5)

    # ---- 033-004: D after TURN travels full distance (no instant-complete) ----
    print("\n-- 033-004: D after TURN (no ZERO enc between) --")
    b.send("ZERO enc")
    b.send("D 150 150 250"); time.sleep(2.5)     # D1: drive 250
    b.send("TURN 9000"); time.sleep(2.5)         # turn ~90 deg (no ZERO enc)
    # capture enc just before D2
    pre = b.snap()
    encL_pre, encR_pre = _pair(pre.get("enc", "0,0")) if pre else (0, 0)
    b.send("D 150 150 200")                       # D2: should travel ~200, not instant-stop
    fr = capture(b, "D2", 3.0, 0.08)
    s = summarize(fr)
    # D2 resets enc at start; the travel during D2 is the final enc magnitude.
    d2_travel = max((abs(x) for x in s["encL"]), default=0.0)
    last = b.snap()
    print(f"   pre-D2 enc={encL_pre:.0f},{encR_pre:.0f}  D2 max encL travel={d2_travel:.0f}  final mode={last.get('mode') if last else '?'}")
    check("033-004 D-after-TURN travels (not instant-stop)", d2_travel > 120.0,
          f"D2 traveled {d2_travel:.0f} mm of 200 commanded")
    b.send("X"); time.sleep(0.5)

    # ---- Health: starts/stops clean, no wild jumps, no runaway spin ----
    print("\n-- health: velocity smoothness + bounded turn --")
    b.send("ZERO enc")
    b.send("T 180 180 2500")
    fr = capture(b, "vsmooth", 2.6, 0.06)
    s = summarize(fr)
    vs = [x for x in s["v"] if x is not None]
    jumps = [abs(vs[i + 1] - vs[i]) for i in range(len(vs) - 1)] if len(vs) > 1 else [0]
    maxjump = max(jumps, default=0)
    print(f"   v trace: {[round(x) for x in vs]}")
    # On a free-spinning stand, big single-sample jumps (>300 mm/s tick-to-tick)
    # would indicate the velocity-loop pathologies the stakeholder flagged.
    check("no wild velocity jumps", maxjump < 300.0, f"max tick-to-tick dv={maxjump:.0f} mm/s")
    b.send("X"); time.sleep(0.5)

    # bounded turn: TURN to 90 deg, heading should settle near 9000 cdeg, not spin past
    b.send("ZERO enc")
    b.send("OZ")  # zero OTOS/bench heading baseline if supported
    b.send("TURN 9000")
    fr = capture(b, "turn90", 3.0, 0.08)
    s = summarize(fr)
    hmax = max((abs(x) for x in s["h"]), default=0.0)
    print(f"   heading trace (cdeg): {[round(x) for x in s['h']]}")
    # Heading should not blow past ~ +/- 18000 (would indicate runaway spin).
    check("no out-of-control spin (heading bounded)", hmax < 20000.0, f"max|h|={hmax:.0f} cdeg")
    b.send("X"); time.sleep(0.3)

    # ---- restore ----
    b.send("DBG OTOS BENCH 0")
    b.send("X")

    # ---- summary ----
    print("\n== SUMMARY ==")
    npass = sum(1 for _, ok, _ in RESULTS if ok)
    for name, ok, detail in RESULTS:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))
    print(f"\n{npass}/{len(RESULTS)} checks passed")
    b.close()


if __name__ == "__main__":
    main()
