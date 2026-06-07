#!/usr/bin/env python3
"""stand_soak.py — randomized maneuver soak that trips on the FIRST encoder glitch.

STAND ONLY (wheels spin free). Drives a wide variety of "normal" operations —
forward / arc / spin / one-wheel across many speeds, ratios, durations and stop
modes — while also toggling streaming features and writing the OTOS world pose
(the camera→robot "you are here" sync). It watches EVERY encoder reading and
stops immediately on the first anomaly, then dumps the recent raw reads, the
recent OV/stream events, and DBG I2C so the trigger can be assessed.

Anomalies (checked per individual encoder read, using the firmware TLM clock):
  * absurd velocity  — implied |Δmm/Δt| above a plausible ceiling
  * read jump        — a single read moves an implausible distance
  * encoder → 0      — drops to (0,0) while commanded and previously nonzero
  * frozen           — N identical reads while commanded to move
  * EVT enc_wedged   — firmware's own latch

    uv run python tests/dev/stand_soak.py [--minutes 30] [--seed 1]
"""

from __future__ import annotations

import argparse
import collections
import random
import sys
import time

# Plausibility ceilings (wheels max out ~250 mm/s commanded).
VMAX_MMPS = 1500     # implied wheel velocity above this = absurd
JUMP_MM   = 400      # single-read distance jump above this = corrupt
FROZEN_K  = 4        # identical reads while commanded = frozen (catch onset fast,
                     # before the firmware EVT enc_wedged latch, so the dumped
                     # ring buffer still holds the last *counting* reads)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None)
    p.add_argument("--minutes", type=float, default=30.0)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--stimeout", type=int, default=1500, help="firmware S-watchdog ms")
    p.add_argument("--set", dest="sets", action="append", default=[], metavar="K=V",
                   help="SET override applied on connect (repeatable), e.g. --set encAtomic=1")
    args = p.parse_args()

    rng = random.Random(args.seed)
    print(f"seed={args.seed}  budget={args.minutes:.0f} min  sTimeout={args.stimeout}ms")

    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.nezha import Nezha
    from robot_radio.robot.protocol import parse_tlm

    class _A:
        port = args.port
        verbose = False

    print("Connecting (relay, pushes calibration) …")
    robot, conn, _ = _make_robot(_A())
    if not isinstance(robot, Nezha):
        print("ERROR: need a Nezha"); return 2
    proto = robot._proto

    # --- shared monitor state ---
    enc = [0, 0]                         # latest encoder totals (mm)
    cur_cmd = [0, 0]                     # currently commanded wheel speeds
    prev = [None]                        # (t_ms, encL, encR) of previous reading
    frozen_run = [0]
    reads = collections.deque(maxlen=40)  # recent (t_ms, eL, eR, vL, vR, cmdL, cmdR)
    events = collections.deque(maxlen=16)  # recent (elapsed, "OV..."/"stream...")
    anomaly = [None]                     # dict on first glitch
    t_start = time.monotonic()

    def note(tag):
        events.append((time.monotonic() - t_start, tag))

    def check_read(t_ms, eL, eR):
        """Per-read anomaly check using the firmware TLM timestamp."""
        cmdL, cmdR = cur_cmd
        commanded = (abs(cmdL) + abs(cmdR)) > 0
        if prev[0] is not None:
            pt, pL, pR = prev[0]
            dt = (t_ms - pt) / 1000.0
            if dt <= 0:
                dt = 0.001
            dL, dR = eL - pL, eR - pR
            vL, vR = dL / dt, dR / dt
            reads.append((t_ms, eL, eR, round(vL), round(vR), cmdL, cmdR))
            reason = None
            if commanded and (abs(vL) > VMAX_MMPS or abs(vR) > VMAX_MMPS):
                reason = f"absurd velocity vL={vL:.0f} vR={vR:.0f} mm/s (dt={dt*1000:.0f}ms)"
            elif abs(dL) > JUMP_MM or abs(dR) > JUMP_MM:
                reason = f"read jump dL={dL} dR={dR} mm in one read"
            elif commanded and eL == 0 and eR == 0 and (pL != 0 or pR != 0):
                reason = "encoder dropped to 0 while driving"
            # frozen tracking
            if commanded and dL == 0 and dR == 0:
                frozen_run[0] += 1
                if frozen_run[0] >= FROZEN_K and reason is None:
                    reason = f"frozen — {frozen_run[0]} identical reads while driving"
            else:
                frozen_run[0] = 0
            if reason and anomaly[0] is None:
                anomaly[0] = {"reason": reason, "t_ms": t_ms,
                              "cmd": (cmdL, cmdR)}
        prev[0] = (t_ms, eL, eR)

    def pump(duration_ms=20):
        for line in proto.read_lines(duration_ms=duration_ms):
            if "enc_wedged" in line and anomaly[0] is None:
                anomaly[0] = {"reason": "EVT enc_wedged (firmware latch)",
                              "t_ms": -1, "cmd": tuple(cur_cmd)}
            tlm = parse_tlm(line)
            if tlm is None:
                continue
            if tlm.enc is not None:
                enc[0], enc[1] = tlm.enc
                t_ms = tlm.t if tlm.t is not None else int(time.monotonic() * 1000)
                check_read(t_ms, tlm.enc[0], tlm.enc[1])

    def dbg_i2c():
        r = proto.send("DBG I2C", 400)
        for line in r.get("responses", []):
            if line.strip().startswith("I2C "):
                return line.strip()
        return "(no I2C line)"

    def set_stream(period, fields):
        fl = ",".join(["enc"] + [f for f in fields if f != "enc"])
        proto.send(f"STREAM fields={fl}", 250)
        proto.stream(period)
        note(f"stream({period}ms,{fl})")

    def ov_write():
        x, y, h = rng.randint(-600, 600), rng.randint(-400, 400), rng.randint(0, 36000)
        robot.set_world_pose(x, y, h)
        note(f"OV({x},{y},{h})")

    SPEEDS = [40, 60, 90, 120, 150, 180, 220]
    DURS = [0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0]
    STOPS = ["clean", "clean", "coast", "watchdog"]
    FIELD_POOL = ["pose", "vel", "line", "color"]
    PERIODS = [20, 50, 100, 200]  # 0 removed: stream-off gap makes delta since last packet look like a jump

    def gen():
        kind = rng.choice(["fwd", "fwd", "arc", "spin", "one", "back"])
        s = rng.choice(SPEEDS)
        if kind == "fwd":   l, r = s, s
        elif kind == "back": l, r = -s, -s
        elif kind == "arc":
            l, r = s, int(s * rng.choice([0.5, 0.7, 0.85, 1.3]))
            if rng.random() < 0.5: l, r = r, l
        elif kind == "spin":
            l, r = (-s, s) if rng.random() < 0.5 else (s, -s)
        else:
            l, r = (s, 0) if rng.random() < 0.5 else (0, s)
        return kind, l, r, rng.choice(DURS), rng.choice(STOPS)

    def drive_window(seconds):
        """Keep alive at cur_cmd for `seconds`, pumping (and monitoring) throughout.

        Injects OV writes / stream toggles MID-drive (concurrent with the
        firmware's motor-writes + encoder-reads on 0x10) — the realistic worst
        case for a read/write collision."""
        t0 = time.monotonic()
        last = 0.0
        last_perturb = 0.0
        while time.monotonic() - t0 < seconds:
            now = time.monotonic()
            if now - last >= 0.15:
                proto.drive(cur_cmd[0], cur_cmd[1])
                last = now
            # ~every 0.4s of driving, perturb the bus while wheels are moving.
            if now - last_perturb >= 0.4:
                last_perturb = now
                if rng.random() < 0.6:
                    ov_write()
                else:
                    set_stream(rng.choice(PERIODS),
                               rng.sample(FIELD_POOL, rng.randint(0, 4)))
            pump(20)
            if anomaly[0]:
                return

    history = []
    try:
        proto.send("STOP", 200)
        proto.send("STREAM 0", 200)
        time.sleep(0.05)
        proto.send(f"SET sTimeout={args.stimeout}", 300)
        for kv in args.sets:
            r = proto.send(f"SET {kv}", 300)
            print(f"  SET {kv} -> {r.get('responses', ['?'])[-1]}")
        proto.send("OI", 400)
        proto.zero_encoders()
        set_stream(50, FIELD_POOL)
        time.sleep(0.2)
        pump(60)
        print(f"I2C start: {dbg_i2c()}\n")

        deadline = t_start + args.minutes * 60.0
        n = 0
        while time.monotonic() < deadline and anomaly[0] is None:
            n += 1
            extra = ""
            if n % 7 == 0:
                set_stream(rng.choice(PERIODS), rng.sample(FIELD_POOL, rng.randint(0, 4)))
                extra += " " + events[-1][1]
            if rng.random() < 0.35:
                ov_write(); extra += " " + events[-1][1]

            kind, l, r, dur, stop = gen()
            cur_cmd[0], cur_cmd[1] = l, r
            drive_window(dur)

            # If the glitch hit mid-drive, break NOW (before the stop phase floods
            # the read buffer with frozen cmd=0 reads) so the dump shows the
            # actual counting→jump→freeze transition.
            if anomaly[0]:
                cur_cmd[0] = cur_cmd[1] = 0
                history.append(f"#{n:04d} {kind:4s} L{l:+4d} R{r:+4d} {dur:.1f}s "
                               f"{stop:8s}{extra}  <<< glitch during drive")
                break

            # stop phase
            cur_cmd[0] = cur_cmd[1] = 0
            if stop == "clean":
                for _ in range(3):
                    proto.stop(); time.sleep(0.05); pump(15)
            elif stop == "coast":
                proto.drive(0, 0); time.sleep(0.3); pump(20)
            else:
                wd = time.monotonic() + args.stimeout / 1000.0 + 0.8
                while time.monotonic() < wd:
                    pump(30)
            time.sleep(0.2); pump(40)

            history.append(f"#{n:04d} {kind:4s} L{l:+4d} R{r:+4d} {dur:.1f}s {stop:8s}{extra}")
            if n % 10 == 0:
                el = time.monotonic() - t_start
                print(f"  [{el/60:4.1f}m] #{n} enc=L{enc[0]} R{enc[1]}  ok")
            if n % 25 == 0:
                print(f"         I2C: {dbg_i2c()}"); pump(20)

        for _ in range(3):
            proto.stop(); time.sleep(0.05)
        proto.stream(0)

        el = time.monotonic() - t_start
        print("\n" + "=" * 64)
        if anomaly[0]:
            a = anomaly[0]
            print(f"  ANOMALY after {n} maneuvers ({el/60:.1f} min): {a['reason']}")
            print(f"  commanded wheels at glitch: L{a['cmd'][0]} R{a['cmd'][1]}")
            print(f"  I2C now: {dbg_i2c()}")
            print("  --- recent OV / stream events (elapsed s) ---")
            for t, tag in events:
                print(f"     {t:6.1f}s  {tag}")
            print("  --- last raw encoder reads (t_ms, encL, encR, vL, vR, cmdL, cmdR) ---")
            for rd in list(reads)[-24:]:
                print(f"     {rd}")
            print("  --- last maneuvers ---")
            for h in history[-8:]:
                print("     " + h)
        else:
            print(f"  NO ANOMALY in {n} maneuvers over {el/60:.1f} min — looks clean.")
            print(f"  I2C now: {dbg_i2c()}")
        print("=" * 64)
    finally:
        try:
            for _ in range(3):
                proto.stop(); time.sleep(0.04)
            proto.stream(0)
            conn.disconnect()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
