#!/usr/bin/env python3
"""hw_stop_matrix.py — on REAL hardware: does every command actually STOP?

For each (start, stop) scenario it streams the encoders, sends the command,
waits for the EVT done / safety_stop, then watches the encoders for ~1.6 s
AFTER that. If the encoders keep growing once the command has finished, the
motors are still running (RUNAWAY).

A safety X is sent before AND after every scenario, so the robot can never
spin unattended.

  uv run python tests/dev/hw_stop_matrix.py [--port ...]
"""
import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.nezha import Nezha
    from robot_radio.robot.protocol import parse_tlm

    class _A:
        port = args.port
        verbose = False

    robot, conn, _ = _make_robot(_A())
    if not isinstance(robot, Nezha):
        print("ERROR: need a Nezha")
        return 2
    proto = robot._proto

    def safety_stop():
        for _ in range(5):
            proto.send("X", 120)
            time.sleep(0.03)
        proto.stream(0)

    # (label, start_cmd, stop_cmd_or_None, run_ms_before_stop)
    scenarios = [
        ("RT 90 natural",  "RT 9000",       None,   0),
        ("RT 360 natural", "RT 36000",      None,   0),
        ("RT -180 nat",    "RT -18000",     None,   0),
        ("TURN 90 nat",    "TURN 9000",     None,   0),
        ("T natural",      "T 150 150 700", None,   0),
        ("D natural",      "D 150 150 150", None,   0),
        ("S + X",          "S 150 150",     "X",    700),
        ("S + STOP",       "S 150 150",     "STOP", 700),
        ("VW spin + X",    "VW 0 1200",     "X",    700),
        ("RT mid + X",     "RT 18000",      "X",    500),
    ]

    proto.send("SET sTimeout=60000", 300)
    print(f"  {'scenario':<16} {'moved°':>7} {'done@s':>7} {'post-grow_mm':>13} {'verdict':>9}")
    fails = 0
    MM_PER_DEG = 63.0 * 3.14159265 / 180.0

    for label, start, stop, run_ms in scenarios:
        safety_stop()
        proto.zero_encoders()
        proto.stream(50)
        samples, events = [], []
        t0 = time.monotonic()

        def pump(dur):
            td = time.monotonic()
            while time.monotonic() - td < dur:
                for line in proto.read_lines(duration_ms=40):
                    tt = time.monotonic() - t0
                    tlm = parse_tlm(line)
                    if tlm is not None and tlm.enc is not None:
                        samples.append((tt, tlm.enc[0], tlm.enc[1]))
                    if "EVT" in line:
                        events.append((tt, line.strip()))

        pump(0.15)
        proto.send(start, 80)
        t_start = time.monotonic() - t0
        if run_ms > 0:
            pump(run_ms / 1000.0)
            if stop:
                proto.send(stop, 80)
            t_done = time.monotonic() - t0
        else:
            t_done = None

        # Watch until a done/safety event (for natural) + 1.6 s settle, max 9 s.
        while time.monotonic() - t0 < 9.0:
            pump(0.1)
            if t_done is None:
                for (tt, ln) in events:
                    if tt > t_start and ("done" in ln or "safety_stop" in ln):
                        t_done = tt
                        break
            if t_done is not None and (time.monotonic() - t0) > t_done + 1.6:
                break
        proto.stream(0)
        safety_stop()

        # total motion (deg) and post-stop growth (mm in the window after done)
        if samples:
            moved_mm = max(max(abs(s[1]) for s in samples),
                           max(abs(s[2]) for s in samples))
        else:
            moved_mm = 0.0
        grow = 0.0
        if t_done is not None:
            post = [s for s in samples if s[0] >= t_done + 0.3]
            if len(post) >= 2:
                grow = max(abs(post[-1][1] - post[0][1]),
                           abs(post[-1][2] - post[0][2]))
        ok = grow <= 4.0
        if not ok:
            fails += 1
        dshow = f"{t_done:.1f}" if t_done is not None else "—"
        print(f"  {label:<16} {moved_mm / MM_PER_DEG:>7.0f} {dshow:>7} "
              f"{grow:>13.1f} {'OK' if ok else 'RUNAWAY':>9}")

    safety_stop()
    try:
        conn.disconnect()
    except Exception:
        pass
    print(f"\n  {'ALL STOPPED ✓' if not fails else str(fails) + ' RUNAWAY ✗'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
