#!/usr/bin/env python3
"""velocity_chart.py — target-agnostic interactive velocity-chart dashboard.

Streams live wheel-velocity and OTOS telemetry via ``make_target`` and renders
a multi-panel matplotlib dashboard while the robot drives.  Works against sim,
bench, or production targets identically — no target-branching in this file.

Panels:
  - Wheel velocity vL, vR (mm/s) vs time
  - OTOS position x, y (mm) vs time

Usage::

    python3 tests/tools/velocity_chart.py --target sim --full-speed
    python3 tests/tools/velocity_chart.py --target bench --port /dev/cu.usbmodem...
    python3 tests/tools/velocity_chart.py --target production --real-time

CLI flags::

  --target {sim,bench,production}   Target mode (default: sim)
  --real-time                       Sim paces to wall-clock speed
  --full-speed                      Sim runs as fast as possible (default)
  --port PORT                       Serial port for bench/production
  --duration SECS                   How long to stream (default: 30)
  --speed MMPS                      Wheel speed mm/s (default: 200)
  --set K=V                         Apply SET override on connect (repeatable)
  --headless                        Skip the matplotlib figure (CSV only)
  --csv PATH                        CSV output path (default: /tmp/velocity_chart.csv)

Notes
-----
matplotlib is imported LAZILY inside ``main()`` so that
``import tests.tools.velocity_chart`` works in environments without a display.
"""

from __future__ import annotations

import argparse
import sys
import time


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Target-agnostic robot velocity chart dashboard"
    )
    p.add_argument(
        "--target", choices=["sim", "bench", "production"], default="sim",
        help="Target mode: sim, bench, or production (default: sim)",
    )

    rt_group = p.add_mutually_exclusive_group()
    rt_group.add_argument(
        "--real-time", dest="real_time", action="store_true",
        help="(sim) pace sim to wall-clock speed",
    )
    rt_group.add_argument(
        "--full-speed", dest="real_time", action="store_false",
        help="(sim) run as fast as possible (default)",
    )
    p.set_defaults(real_time=False)

    p.add_argument(
        "--port", default=None,
        help="Serial port for bench/production (auto-detect if omitted)",
    )
    p.add_argument(
        "--duration", type=float, default=30.0,
        help="Stream duration in seconds (default: 30)",
    )
    p.add_argument(
        "--speed", type=int, default=200,
        help="Wheel speed mm/s (default: 200)",
    )
    p.add_argument(
        "--set", dest="sets", action="append", default=[], metavar="K=V",
        help="Apply SET override on connect (repeatable)",
    )
    p.add_argument(
        "--headless", action="store_true",
        help="Skip matplotlib figure (CSV logging only)",
    )
    p.add_argument(
        "--csv", default="/tmp/velocity_chart.csv",
        help="CSV output path (default: /tmp/velocity_chart.csv)",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)

    # All heavy imports deferred — matplotlib, robot_radio.testkit.
    from robot_radio.testkit import make_target, SafeRun
    from robot_radio.robot.protocol import parse_tlm

    print(
        f"[velocity_chart] target={args.target}  speed={args.speed} mm/s  "
        f"duration={args.duration}s  real_time={args.real_time}"
    )

    tr = make_target(
        args.target,
        real_time=args.real_time,
        port=args.port,
    )

    # Apply any SET overrides before driving.
    for kv in args.sets:
        k, v = kv.split("=", 1)
        result = tr.robot.send(f"SET {k}={v}", 300)
        print(f"  SET {kv} -> {result}")

    # Clear any encoder I2C wedge so per-wheel velocity reads correctly on
    # bench/production: the encoders can boot frozen at 0 until a ZERO enc
    # resets them, which would make the velocity panel read flat 0 even while
    # the robot drives.  Harmless on sim (encoders start at 0 anyway).
    try:
        tr.robot.zero_encoders()
    except Exception:
        pass

    # Build the dashboard (lazy matplotlib — deferred until show()).
    panels = [
        ("Wheel velocity (mm/s)", "mm/s", ["vL", "vR"]),
        ("OTOS position (mm)", "mm", ["x", "y"]),
    ]
    from robot_radio.testkit.dash import Dashboard

    dash = Dashboard("Robot velocity chart", panels, window_s=args.duration)

    if not args.headless:
        # Select matplotlib backend (must happen before any pyplot call).
        import matplotlib
        import platform

        if platform.system() == "Darwin":
            matplotlib.use("MacOSX")
        else:
            matplotlib.use("TkAgg")
        dash.show()

    # Drive loop — streams wheel velocity and pose telemetry.
    t_end = time.monotonic() + args.duration

    # speeds[0]/[1] = left/right wheel mm/s; mutable so caller can adjust.
    speeds = [args.speed, args.speed]

    # Last-seen per-wheel velocity (carry-forward across partial frames).
    last_vL: float = 0.0
    last_vR: float = 0.0

    try:
        with SafeRun(tr, max_seconds=args.duration + 5):
            for resp in tr.robot.stream_drive(speeds, period_ms=40):
                if resp.tag == "TLM":
                    # Extract per-wheel velocity from the raw TLM frame.
                    tlm = parse_tlm(resp.raw)
                    if tlm is not None and tlm.vel is not None:
                        last_vL = float(tlm.vel[0])
                        last_vR = float(tlm.vel[1])

                # Read pose from the robot's current state (updated by
                # stream_drive via _apply_tlm before yielding).
                state = tr.robot.state
                data = {
                    "vL": last_vL,
                    "vR": last_vR,
                    "x": float(state.pose.x),
                    "y": float(state.pose.y),
                }
                dash.update(data)

                if not args.headless and dash.is_open():
                    dash.draw()

                if time.monotonic() >= t_end:
                    break

    except KeyboardInterrupt:
        print("\n[velocity_chart] interrupted by user")
    except Exception as exc:
        print(f"\n[velocity_chart] stopped: {exc}")
    finally:
        try:
            tr.conn.disconnect()
        except Exception:
            pass

    # Save CSV.
    dash.save_csv(args.csv)
    print(f"[velocity_chart] CSV saved to {args.csv}  ({len(dash._rows)} rows)")

    if not args.headless:
        try:
            import matplotlib.pyplot as plt

            plt.close("all")
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
