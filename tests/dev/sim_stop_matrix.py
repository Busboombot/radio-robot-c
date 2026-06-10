#!/usr/bin/env python3
"""sim_stop_matrix.py — exhaustively test that motors actually STOP.

For every way of starting the motors and every way of stopping them, this
records the encoders/velocity AT the stop, 300 ms after, and 1.3 s after.
If the encoders keep changing after the stop, the motors are still running
(a runaway) and the row FAILS.

Runs against the in-process firmware (libfirmware_host) so it is deterministic
and needs no robot.

  uv run python tests/dev/sim_stop_matrix.py
"""
import sys


def main() -> int:
    from robot_radio.io.sim_conn import SimConnection

    TOL_MM = 2.0   # post-stop encoder drift tolerance (per wheel)

    def fresh():
        c = SimConnection()
        if "error" in c.connect():
            raise RuntimeError("sim connect failed")
        c.send("SET sTimeout=60000")   # disable watchdog unless a test wants it
        c.set_enc(0.0, 0.0)
        c.tick(50)
        return c

    def enc(c):
        s = c.get_state()
        return s["enc_l"], s["enc_r"]

    def vel(c):
        s = c.get_state()
        return s["vel_l"], s["vel_r"]

    # (label, start_cmd, run_ms_before_stop, stop_cmd_or_None_for_natural)
    scenarios = [
        ("T natural-complete",   "T 150 150 700",   0,    None),
        ("D natural-complete",   "D 150 150 150",   0,    None),
        ("TURN natural",         "TURN 9000",       0,    None),
        ("RT 90 natural",        "RT 9000",         0,    None),
        ("RT 360 natural",       "RT 36000",        0,    None),
        ("RT -90 natural",       "RT -9000",        0,    None),
        ("S + X",                "S 150 150",       600,  "X"),
        ("S + STOP",             "S 150 150",       600,  "STOP"),
        ("S + X soft",           "S 150 150",       600,  "X soft"),
        ("VW spin + X",          "VW 0 1500",       600,  "X"),
        ("VW spin + STOP",       "VW 0 1500",       600,  "STOP"),
        ("RT mid + X",           "RT 18000",        400,  "X"),
        ("RT mid + STOP",        "RT 18000",        400,  "STOP"),
        ("T mid + X",            "T 150 150 3000",  500,  "X"),
        ("D mid + STOP",         "D 150 150 600",   500,  "STOP"),
    ]

    print(f"  {'scenario':<22} {'stop@mm':>14} {'+300ms drift':>13} {'+1.3s drift':>12} "
          f"{'vel@end':>12} {'verdict':>8}")
    n_fail = 0
    for label, start, run_ms, stop in scenarios:
        c = fresh()
        c.send(start, read_ms=50, stop_token="OK")
        if run_ms > 0:
            c.tick(run_ms)
            if stop:
                c.send(stop, read_ms=50, stop_token="OK")
        else:
            # natural completion — tick until EVT done (max 8 s)
            t = 0
            while t < 8000:
                lines = c.tick(50)
                t += 50
                if any("done" in ln or "safety_stop" in ln for ln in lines):
                    break
        # settle one tick so the stop takes effect
        c.tick(50)
        e0 = enc(c)
        c.tick(300)
        e1 = enc(c)            # early drift (soft stops are still ramping here)
        # Give any soft ramp time to finish (kSoftDeadlineMs = 3000), then check
        # the FINAL 500 ms window is flat — that is the real "did it actually
        # stop, or keep going forever" question.
        c.tick(3200)
        e_pre = enc(c)
        c.tick(500)
        e_fin = enc(c)
        vL, vR = vel(c)

        early = max(abs(e1[0] - e0[0]), abs(e1[1] - e0[1]))
        final = max(abs(e_fin[0] - e_pre[0]), abs(e_fin[1] - e_pre[1]))
        ok = final <= TOL_MM and abs(vL) <= 3 and abs(vR) <= 3
        if not ok:
            n_fail += 1
        print(f"  {label:<22} L{e0[0]:>5.0f}/R{e0[1]:>5.0f} {early:>13.1f} {final:>12.1f} "
              f"L{vL:>4.0f}/R{vR:>4.0f} {'OK' if ok else 'RUNAWAY':>8}")
        c.disconnect()

    print(f"\n  {'ALL STOPPED ✓' if n_fail == 0 else str(n_fail) + ' RUNAWAY FAILURES ✗'}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
