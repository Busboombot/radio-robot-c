#!/usr/bin/env python3
"""playfield_tour.py — target-agnostic playfield tour tool.

Drives the robot through a sequence of waypoints loaded from
``data/aprilcam/playfield.json`` rectangles.  Works against sim, bench, or
production targets — no target-branching here; all differences are expressed
through ``make_target`` and ``PoseSource``.

Control loop (per leg)::

  1. Read pose via ``tr.pose.read()`` → (x_cm, y_cm, yaw_rad).
  2. For camera/production runs: anchor firmware OTOS to camera fix via
     ``robot.update_world_pose(x_cm, y_cm, yaw_rad)``.
  3. Compute robot-relative (fwd_mm, left_mm) from the world delta.
     Lateral negation convention (from playfield_random_tour.py)::

         fwd = dx*cos(H) + dy*sin(H)   (world→robot forward projection)
         lft = dx*sin(H) - dy*cos(H)   (world→robot lateral; NEGATED vs standard math)

     The negation compensates for the camera/field handedness: a target to
     the robot's left in the world frame has lft < 0, which the firmware
     interprets as "turn left (CCW)".

  4. Call ``robot.go_to(fwd_mm, left_mm, speed, on_tick=on_tick_cb)``.
     ``on_tick_cb`` updates the camera track, checks bounds, and returns
     False to abort if the robot leaves the safe box.
  5. After the leg, check camera arrival within ``--arrive`` tolerance.

Usage::

    python3 tests/tools/playfield_tour.py --target sim --full-speed
    python3 tests/tools/playfield_tour.py --target bench --port /dev/cu.usbmodem...
    python3 tests/tools/playfield_tour.py --target production --pose camera \\
        --port /dev/cu.usbmodem... --camera arducam-ov9782-usb-camera

CLI flags::

  --target {sim,bench,production}   Target mode (default: sim)
  --pose {firmware,camera,auto}     Pose source; auto = firmware for sim/bench,
                                    camera for production (default: auto)
  --real-time                       Sim paces to wall-clock speed
  --full-speed                      Sim runs as fast as possible (default)
  --port PORT                       Serial port for bench/production
  --camera CAM                      Camera name for production/camera-pose runs
  --hops N                          Number of tour hops (default: 6)
  --speed MMPS                      G arc speed mm/s (default: 160)
  --arrive CM                       Arrival tolerance in cm (default: 8)
  --seed N                          RNG seed for waypoint selection
  --abort-x CM                      Safety box half-width in x (default: 60)
  --abort-y CM                      Safety box half-width in y (default: 42)
  --playfield-json PATH             Override playfield.json path

Notes
-----
matplotlib and aprilcam are NOT imported at module level — all imports are
deferred inside ``main()`` so that ``import tests.tools.playfield_tour``
works without a display or camera daemon.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import random
import sys
import time


# Default safety box (cm) — keeps robot centre well inside the field.
_DEFAULT_ABORT_X = 60.0
_DEFAULT_ABORT_Y = 42.0

# Default waypoint set (used when no playfield.json is available).
# These are the 8 named-square centres from the standard LEAG playfield
# (A1-centred world coordinates, in cm).
_FALLBACK_WAYPOINTS = [
    ("purple-NW", -35.0, 24.0),
    ("black-N",    0.0, 24.0),
    ("orange-NE",  35.0, 24.0),
    ("red-E",      35.0,  0.0),
    ("green-SE",   35.0, -24.0),
    ("magenta-S",   0.0, -24.0),
    ("blue-SW",   -35.0, -24.0),
    ("red-W",     -35.0,  0.0),
]


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Target-agnostic playfield tour tool"
    )
    p.add_argument(
        "--target", choices=["sim", "bench", "production"], default="sim",
        help="Target mode (default: sim)",
    )
    p.add_argument(
        "--pose", choices=["firmware", "camera", "auto"], default="auto",
        help="Pose source: firmware, camera, or auto (default: auto = "
             "firmware for sim/bench, camera for production)",
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
        "--camera", default=None,
        help="Camera name for production/camera-pose runs",
    )
    p.add_argument(
        "--hops", type=int, default=6,
        help="Number of tour hops (default: 6)",
    )
    p.add_argument(
        "--speed", type=int, default=160,
        help="G arc speed mm/s (default: 160)",
    )
    p.add_argument(
        "--arrive", type=float, default=8.0,
        help="Arrival tolerance in cm (default: 8)",
    )
    p.add_argument(
        "--seed", type=int, default=None,
        help="RNG seed for waypoint selection (default: random)",
    )
    p.add_argument(
        "--abort-x", type=float, default=_DEFAULT_ABORT_X,
        help=f"Safety box half-width in x cm (default: {_DEFAULT_ABORT_X})",
    )
    p.add_argument(
        "--abort-y", type=float, default=_DEFAULT_ABORT_Y,
        help=f"Safety box half-width in y cm (default: {_DEFAULT_ABORT_Y})",
    )
    p.add_argument(
        "--playfield-json", default=None, dest="playfield_json",
        help="Override playfield.json path",
    )
    return p.parse_args(argv)


def _load_waypoints(
    playfield_json: str | None,
) -> list[tuple[str, float, float]]:
    """Return [(slug, x_cm, y_cm), ...] from playfield.json rectangles.

    Falls back to ``_FALLBACK_WAYPOINTS`` when no file is specified or the
    file is absent / has no rectangles.
    """
    if playfield_json is not None:
        path = pathlib.Path(playfield_json)
    else:
        # Default location: data/aprilcam/playfield.json relative to the
        # repo root (three levels up from this file: tools/ → tests/ → repo/).
        repo = pathlib.Path(__file__).resolve().parent.parent.parent
        path = repo / "data" / "aprilcam" / "playfield.json"

    if path.exists():
        import json

        try:
            data = json.loads(path.read_text())
            rects = data.get("rectangles", [])
            if rects:
                return [
                    (rec["slug"], float(rec["x"]), float(rec["y"]))
                    for rec in rects
                    if "slug" in rec and "x" in rec and "y" in rec
                ]
        except Exception as exc:
            print(f"[playfield_tour] warning: failed to load {path}: {exc}",
                  file=sys.stderr)

    print(f"[playfield_tour] playfield.json not found at {path}; "
          "using fallback waypoints", file=sys.stderr)
    return list(_FALLBACK_WAYPOINTS)


def _compute_robot_relative(
    x_cm: float, y_cm: float, yaw_rad: float,
    tx_cm: float, ty_cm: float,
) -> tuple[float, float]:
    """Convert world-frame target to robot-relative (fwd_mm, left_mm).

    Applies the lateral negation convention documented in
    ``playfield_random_tour.py``: a target to the robot's physical left in
    world coordinates produces lft < 0, which the firmware interprets as
    "turn CCW (left)".  This compensates for the field's handedness.

        fwd = dx*cos(H) + dy*sin(H)
        lft = dx*sin(H) - dy*cos(H)    # NEGATED vs standard math left

    Arguments are in cm; return value is in mm.
    """
    dx = tx_cm - x_cm
    dy = ty_cm - y_cm
    fwd = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
    lft = dx * math.sin(yaw_rad) - dy * math.cos(yaw_rad)
    return (fwd * 10.0, lft * 10.0)   # cm → mm


def _select_target(
    waypoints: list[tuple[str, float, float]],
    x_cm: float, y_cm: float,
    rng: random.Random,
    drop: int = 4,
) -> tuple[str, float, float]:
    """Pick a random target from the ``drop`` farthest waypoints."""
    ranked = sorted(
        waypoints,
        key=lambda w: math.hypot(w[1] - x_cm, w[2] - y_cm),
        reverse=True,
    )
    pool = ranked[:drop] if len(ranked) >= drop else ranked
    return rng.choice(pool)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)

    # Resolve pose mode.
    if args.pose == "auto":
        use_camera = (args.target == "production")
    elif args.pose == "camera":
        use_camera = True
    else:
        use_camera = False

    # Load waypoints.
    waypoints = _load_waypoints(args.playfield_json)
    print(f"[playfield_tour] loaded {len(waypoints)} waypoints")

    # All heavy imports deferred — aprilcam, robot_radio.testkit.
    from robot_radio.testkit import make_target, SafeRun

    print(
        f"[playfield_tour] target={args.target}  pose={'camera' if use_camera else 'firmware'}  "
        f"hops={args.hops}  speed={args.speed} mm/s  real_time={args.real_time}"
    )

    # camera kwarg is only passed when production + camera is requested.
    camera_arg = args.camera if use_camera else None

    tr = make_target(
        args.target,
        real_time=args.real_time,
        port=args.port,
        camera=camera_arg,
    )

    # Safety geometry.
    abort_x = args.abort_x
    abort_y = args.abort_y

    def _in_bounds(x: float, y: float) -> bool:
        return abs(x) <= abort_x and abs(y) <= abort_y

    # Camera tracks for path overlay (camera pose only).
    cam_track: list[tuple[float, float]] = []

    def _make_on_tick(robot_ref: list) -> object:
        """Build an on_tick callback for go_to.

        - For camera pose: poll ``tr.pose.read()`` each tick, record the track,
          draw via ``tr.playfield.add_path()``, and return False on bounds breach.
        - For firmware pose: just record odometry and return None (continue).
        """
        def on_tick(robot) -> bool | None:
            if use_camera and tr.playfield is not None:
                try:
                    cx, cy, _ = tr.pose.read()
                    cam_track.append((cx, cy))
                    if not _in_bounds(cx, cy):
                        print(f"  [BOUNDS-ABORT] camera ({cx:.1f},{cy:.1f})")
                        return False
                    if len(cam_track) >= 2:
                        tr.playfield.add_path(
                            "tour", cam_track,
                            symbol="filled_circle",
                            color=(0, 200, 255),
                            size_cm=1.2,
                        )
                except Exception:
                    pass   # transient tag dropout — keep driving
            return None   # continue

        return on_tick

    robot_ref: list = [tr.robot]
    on_tick = _make_on_tick(robot_ref)

    rng = random.Random(args.seed)
    results: list[dict] = []

    max_per_tour = args.hops * 20 + 30   # generous wall-clock cap

    def _advance(ms: int) -> None:
        """Advance sim time or sleep on hardware — no target branching needed."""
        if hasattr(tr.conn, "tick"):
            tr.conn.tick(ms)
        else:
            time.sleep(ms / 1000.0)

    try:
        with SafeRun(tr, max_seconds=max_per_tour):
            # Warm-up: advance state so the first pose read is non-zero.
            _advance(60)

            for hop_i in range(1, args.hops + 1):
                # Read current pose.
                try:
                    x_cm, y_cm, yaw_rad = tr.pose.read()
                except Exception as exc:
                    print(f"[hop {hop_i}] pose read failed: {exc} — stopping tour")
                    break

                if use_camera and not _in_bounds(x_cm, y_cm):
                    print(f"[hop {hop_i}] start ({x_cm:.1f},{y_cm:.1f}) out of bounds — aborting")
                    break

                # Pick target.
                slug, tx, ty = _select_target(waypoints, x_cm, y_cm, rng)
                dist_cm = math.hypot(tx - x_cm, ty - y_cm)
                print(
                    f"[{hop_i:2d}/{args.hops}] from ({x_cm:+.1f},{y_cm:+.1f}) "
                    f"yaw={math.degrees(yaw_rad)%360:.0f}° → {slug} "
                    f"({tx:+.0f},{ty:+.0f})  {dist_cm:.1f} cm out"
                )

                if dist_cm < args.arrive:
                    print(f"  already within {args.arrive:.0f} cm — skipping")
                    continue

                # For camera runs: anchor firmware OTOS to camera fix.
                if use_camera:
                    tr.robot.update_world_pose(x_cm, y_cm, yaw_rad)
                    time.sleep(0.1)

                # Compute robot-relative target in mm.
                fwd_mm, lft_mm = _compute_robot_relative(x_cm, y_cm, yaw_rad, tx, ty)

                # Timeout: generous distance/speed + 6 s margin.
                timeout_s = dist_cm * 10.0 / max(args.speed, 1) + 6.0

                _el, _er, outcome = tr.robot.go_to(
                    int(round(fwd_mm)),
                    int(round(lft_mm)),
                    args.speed,
                    on_tick=on_tick,
                    timeout_s=timeout_s,
                )

                print(f"  outcome={outcome}")
                results.append(
                    {"hop": hop_i, "slug": slug, "tx": tx, "ty": ty,
                     "outcome": outcome}
                )

                if outcome == "aborted":
                    print("  [BOUNDS-ABORT] tour stopped")
                    break

                # Brief pause between legs.
                _advance(100)

    except KeyboardInterrupt:
        print("\n[playfield_tour] interrupted by user")
    except Exception as exc:
        print(f"\n[playfield_tour] error: {exc}")
    finally:
        try:
            tr.conn.disconnect()
        except Exception:
            pass
        if tr.playfield is not None:
            try:
                tr.playfield.close()
            except Exception:
                pass

    n_done = sum(1 for r in results if r["outcome"] in ("done", "timeout"))
    print(
        f"\n[playfield_tour] done — {len(results)} hops attempted, "
        f"{n_done} completed, {len(cam_track)} camera points"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
