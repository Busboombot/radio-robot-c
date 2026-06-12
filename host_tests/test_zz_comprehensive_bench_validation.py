"""
Comprehensive full-stack drive validation (Bench-OTOS-equivalent, sim).

Drives the firmware through the same sequences the stakeholder asked to validate
on the bench — turns, a square, and velocity profiles — using the field profile
(OTOS fusion ON + turn slip), which exercises the identical pose-synthesis +
EKF-fusion path that the on-hardware Bench OTOS uses. (The Bench OTOS firmware
class was ported from the same MockOtosSensor sim-model this harness drives.)

For each sequence it collects per-tick TLM and checks for the failure signatures
the stakeholder named:
  - bad starts (instant velocity jump at command start instead of a ramp)
  - bad stops (residual velocity after the command's terminal event)
  - bad velocity jumps (large tick-to-tick |dv| mid-motion)
  - out-of-control spinning (unbounded heading growth / omega over the rate cap)
  - EKF health (ekf_rej not climbing)

Run with:
  uv run --with pytest python -m pytest host_tests/test_zz_comprehensive_bench_validation.py -s -v
"""
import math
import re

import pytest

STEP_MS = 24  # realistic loop period under load (matches field-profile tests)


def _kv(line):
    d = {}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            d[k] = v
    return d


def _frames(sim, ms):
    """Tick for `ms`, return parsed TLM dicts with t,mode,pose,twist,enc,ekf_rej."""
    raw = sim.tick_collect_tlm(ms, STEP_MS)
    out = []
    for ln in raw:
        if not ln.startswith("TLM"):
            continue
        kv = _kv(ln)
        f = {"raw": ln, "mode": kv.get("mode", "?")}
        if "pose" in kv:
            p = kv["pose"].split(",")
            if len(p) >= 3:
                f["x"], f["y"], f["h"] = float(p[0]), float(p[1]), float(p[2])
        if "twist" in kv:
            t = kv["twist"].split(",")
            if len(t) >= 2:
                f["v"], f["omega"] = float(t[0]), float(t[1])
        if "vel" in kv and "v" not in f:
            t = kv["vel"].split(",")
            try:
                f["v"] = float(t[0])
            except (ValueError, IndexError):
                pass
        if "enc" in kv:
            e = kv["enc"].split(",")
            if len(e) >= 2:
                f["encL"], f["encR"] = float(e[0]), float(e[1])
        if "ekf_rej" in kv:
            try:
                f["ekf_rej"] = int(kv["ekf_rej"])
            except ValueError:
                pass
        out.append(f)
    return out


def _analyze(name, frames, report):
    """Compute drive-health metrics over a list of TLM frames; append to report."""
    vs = [f["v"] for f in frames if "v" in f]
    oms = [f["omega"] for f in frames if "omega" in f]
    hs = [f["h"] for f in frames if "h" in f]
    rej = [f["ekf_rej"] for f in frames if "ekf_rej" in f]

    metrics = {"name": name, "n_frames": len(frames)}
    if vs:
        dv = [abs(vs[i] - vs[i - 1]) for i in range(1, len(vs))]
        metrics["v_max"] = max(abs(x) for x in vs)
        metrics["v_jump_max"] = max(dv) if dv else 0.0
        metrics["v_start"] = vs[0]
        metrics["v_end"] = vs[-1]
        # start ramp: first non-trivial v should appear within a couple ticks, not instantly maxed
        metrics["v_first3"] = [round(x, 1) for x in vs[:3]]
    if oms:
        metrics["omega_max"] = max(abs(x) for x in oms)
    if hs:
        metrics["heading_total_change_deg"] = math.degrees(
            sum(abs(hs[i] - hs[i - 1]) for i in range(1, len(hs)))
        )
        metrics["heading_final_deg"] = math.degrees(hs[-1])
    if rej:
        metrics["ekf_rej_start"] = rej[0]
        metrics["ekf_rej_end"] = rej[-1]
        metrics["ekf_rej_climb"] = rej[-1] - rej[0]
    report.append(metrics)
    return metrics


@pytest.fixture
def fsim(sim):
    """Field-profile sim: OTOS fusion ON + turn slip (Bench-OTOS-equivalent)."""
    sim.send_command("SET sTimeout=60000")
    sim.set_field_profile(slip_turn_extra=0.26, fuse_otos=True)
    sim.send_command("STREAM 30 fields=mode,pose,twist,enc,ekf_rej")
    sim.drain_reply_store()
    return sim


def test_comprehensive_bench_validation(fsim):
    sim = fsim
    report = []
    problems = []

    # ---- Sequence 1: TURN closure (4 x 90deg CCW -> ~360, back to start) ----
    turn_frames = []
    sim.send_command("ZERO enc")
    sim.send_command("SI 0 0 0")  # zero pose
    sim.drain_reply_store()
    for _ in range(4):
        sim.send_command("TURN 9000")  # 90.00 deg (centi-deg)
        turn_frames += _frames(sim, 2500)
        sim.get_async_evts()
    m = _analyze("turns_4x90", turn_frames, report)
    # spin sanity: total heading change should be ~360 (4x90), not a runaway multi-rev
    if m.get("heading_total_change_deg", 0) > 360 * 2.0:
        problems.append(f"turns: runaway heading {m['heading_total_change_deg']:.0f}deg (>720)")
    if m.get("omega_max", 0) > 12.0:  # rad/s — far above any sane yaw rate cap
        problems.append(f"turns: omega spike {m['omega_max']:.1f} rad/s")
    if abs(m.get("v_end", 0)) > 30:
        problems.append(f"turns: nonzero residual v_end {m['v_end']:.1f} mm/s (bad stop)")

    # ---- Sequence 2: Square via D + TURN (4 sides of 300mm) ----
    sq_frames = []
    sim.send_command("ZERO enc")
    sim.send_command("SI 0 0 0")
    sim.drain_reply_store()
    for _ in range(4):
        sim.send_command("D 300 250 250")  # 300mm at 250mm/s
        sq_frames += _frames(sim, 3000)
        sim.get_async_evts()
        sim.send_command("TURN 9000")
        sq_frames += _frames(sim, 2500)
        sim.get_async_evts()
    m = _analyze("square_DxTURN", sq_frames, report)
    if m.get("v_jump_max", 0) > 120:  # mm/s per ~24ms tick; trapezoid ramp is ~tens
        problems.append(f"square: velocity jump {m['v_jump_max']:.0f} mm/s/tick (bad jump)")
    if m.get("ekf_rej_climb", 0) > 20:
        problems.append(f"square: ekf_rej climbed {m['ekf_rej_climb']} (pose corruption?)")
    if abs(m.get("v_end", 0)) > 30:
        problems.append(f"square: residual v_end {m['v_end']:.1f} mm/s (bad stop)")

    # ---- Sequence 3: Velocity profiles (D at 3 speeds + T) ----
    for label, cmd, dur in [
        ("D_slow_150", "D 250 150 150", 3000),
        ("D_med_300", "D 400 300 300", 2500),
        ("D_fast_500", "D 500 500 500", 2000),
        ("T_timed_1500", "T 1500 300 300", 2200),
    ]:
        sim.send_command("ZERO enc")
        sim.send_command("SI 0 0 0")
        sim.drain_reply_store()
        sim.send_command(cmd)
        fr = _frames(sim, dur)
        sim.get_async_evts()
        m = _analyze(label, fr, report)
        # start ramp: should not jump to >60% of peak in the very first frame
        v0 = abs(m.get("v_start", 0))
        vmax = m.get("v_max", 1) or 1
        if v0 > 0.6 * vmax and vmax > 50:
            problems.append(f"{label}: instant start jump v0={v0:.0f} of vmax={vmax:.0f} (bad start)")
        if m.get("v_jump_max", 0) > 120:
            problems.append(f"{label}: velocity jump {m['v_jump_max']:.0f} mm/s/tick")
        if abs(m.get("v_end", 0)) > 30:
            problems.append(f"{label}: residual v_end {m['v_end']:.1f} mm/s (bad stop)")
        # straight drive must not spin
        if m.get("heading_total_change_deg", 0) > 25:
            problems.append(f"{label}: spurious heading change {m['heading_total_change_deg']:.0f}deg on straight drive")

    # ---- Report ----
    print("\n\n================ COMPREHENSIVE BENCH VALIDATION (sim, field profile) ================")
    for m in report:
        print(f"\n[{m['name']}]  frames={m['n_frames']}")
        for k in ("v_first3", "v_max", "v_jump_max", "v_start", "v_end",
                  "omega_max", "heading_total_change_deg", "heading_final_deg",
                  "ekf_rej_start", "ekf_rej_end", "ekf_rej_climb"):
            if k in m:
                val = m[k]
                if isinstance(val, float):
                    val = round(val, 2)
                print(f"    {k:28s} = {val}")
    print("\n---------------- VERDICT ----------------")
    if problems:
        print(f"FOUND {len(problems)} PROBLEM(S):")
        for p in problems:
            print("  ✗ " + p)
    else:
        print("  ✓ No pathologies detected: clean starts/stops, bounded velocity,")
        print("    no runaway spin, EKF stable across turns/square/velocity profiles.")
    print("=========================================================================\n")

    assert not problems, f"Drive validation found pathologies: {problems}"
