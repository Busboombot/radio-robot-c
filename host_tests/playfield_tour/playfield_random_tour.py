#!/usr/bin/env python
"""Playfield tour — ONE smooth firmware G per leg, camera between legs.

Per leg:
  1. CHECK    — read the camera (averaged) for the robot's world pose.
  2. ANCHOR   — robot.update_world_pose() fixes the firmware OTOS to that pose.
  3. CALCULATE — robot-relative (fwd_mm, left_mm) to the next rectangle.
  4. DRIVE    — robot.go_to(fwd, lft, SPEED, on_tick=on_tick_cb) issues ONE
                smooth firmware G arc. The on_tick callback feeds the camera
                poll each telemetry tick, records both camera and odometry
                tracks, and returns False to abort if the robot leaves the
                safe box.

Turn sign: the firmware turns CW for a positive relative angle, so the
lateral component is NEGATED — a target on the robot's left gets a negative
`left`, which makes the firmware turn left (CCW). beginGoTo turns by
atan2(left, fwd) and drives sqrt(fwd^2+left^2), so the absolute SI heading
cancels out of the motion.

Draws two persistent paths on the live view: camera track (cyan dots) and
the robot's streamed odometry (small light-yellow crosses).

Usage::

    uv run --group calibrate python host_tests/playfield_tour/playfield_random_tour.py

Optional flags::

    --port  /dev/cu.usbmodem2121402   # relay USB port (auto-detected if omitted)
    --hops  6
    --speed 160
"""
from __future__ import annotations

import argparse
import math
import random
import sys
import time

ROBOT_TAG = 100
SPEED = 160                # mm/s — smooth single-G arc
HOPS = 6
ARRIVE_CM = 8.0            # within this of the rectangle = arrived
MAX_G_PER_LEG = 3          # full smooth G per try; recompute from camera if short
# Safety box (cm) — abort the leg if the camera shows the robot centre leaving it.
ABORT_X = 40.0
ABORT_Y = 33.0

# ---------------------------------------------------------------------------
# Module-level track lists — cleared per run
# ---------------------------------------------------------------------------

CAM_TRACK: list[tuple[float, float]] = []
ODO_TRACK: list[tuple[float, float]] = []


# ---------------------------------------------------------------------------
# Pure maths helpers (unchanged from original)
# ---------------------------------------------------------------------------

def distance(target_slug_xy: tuple, loc: tuple[float, float, float]) -> float:
    """Euclidean cm distance from loc (x, y, heading) to target (slug, x, y)."""
    return math.hypot(target_slug_xy[1] - loc[0], target_slug_xy[2] - loc[1])


def in_bounds(x: float, y: float) -> bool:
    return abs(x) <= ABORT_X and abs(y) <= ABORT_Y


def select_target(slots: list[tuple], loc: tuple[float, float, float]):
    """Pick a random target from the 5 farthest rectangles."""
    ranked = sorted(slots, key=lambda s: distance(s, loc), reverse=True)
    return random.choice(ranked[:5])


def compute_g(loc: tuple[float, float, float], target: tuple) -> tuple[float, float]:
    """Return robot-relative (fwd_mm, left_mm). Lateral NEGATED (see module docstring)."""
    x, y, H = loc
    dx, dy = target[1] - x, target[2] - y
    fwd = dx * math.cos(H) + dy * math.sin(H)
    lft = dx * math.sin(H) - dy * math.cos(H)
    return (fwd * 10.0, lft * 10.0)


# ---------------------------------------------------------------------------
# Camera helpers
# ---------------------------------------------------------------------------

def select_location(field, n: int = 5) -> tuple[float, float, float]:
    """Averaged camera pose (x_cm, y_cm, yaw_rad); retries through tag dropouts."""
    xs, ys, ss, cc = [], [], [], []
    deadline = time.time() + 4.0
    while time.time() < deadline and len(xs) < n:
        tag = field.get_tag(ROBOT_TAG)
        if tag is not None:
            xs.append(tag.x)
            ys.append(tag.y)
            ss.append(math.sin(tag.yaw))
            cc.append(math.cos(tag.yaw))
        time.sleep(0.04)
    if not xs:
        raise RuntimeError("robot tag 100 not visible to camera")
    return (
        sum(xs) / len(xs),
        sum(ys) / len(ys),
        math.atan2(sum(ss) / len(ss), sum(cc) / len(cc)),
    )


# ---------------------------------------------------------------------------
# Per-leg on_tick callback
# ---------------------------------------------------------------------------

def _make_on_tick(field, robot_ref_holder: list):
    """Return an on_tick(robot) callback that records tracks and checks bounds.

    robot_ref_holder is a one-element list so the closure can update
    the reference (Python closures bind names, not values).
    """
    def on_tick(robot) -> bool | None:
        # 1. Camera sample for track.
        tag = field.get_tag(ROBOT_TAG)
        if tag is not None:
            CAM_TRACK.append((tag.x, tag.y))
            # 2. Bounds check — return False to abort.
            if not in_bounds(tag.x, tag.y):
                return False

        # 3. Odometry track from robot state (x_mm / 10 → cm).
        pose = robot.state.pose
        ODO_TRACK.append((pose.x / 10.0, pose.y / 10.0))

        # 4. Draw both tracks.
        if len(CAM_TRACK) >= 2:
            field.add_path(
                "camera", CAM_TRACK,
                symbol="filled_circle",
                color=(0, 200, 255),
                size_cm=1.2,
            )
        if len(ODO_TRACK) >= 2:
            field.add_path(
                "odometry", ODO_TRACK,
                symbol="x",
                color=(255, 235, 130),
                size_cm=0.8,
            )
        return None  # continue

    return on_tick


# ---------------------------------------------------------------------------
# Hop
# ---------------------------------------------------------------------------

def hop(i: int, field, robot, slots: list[tuple]) -> None:
    """Execute one hop: pick a target, drive to it with camera-anchored G arcs."""
    target = select_target(slots, select_location(field))
    print(f"=== hop {i}/{HOPS}: {target[0]} ({target[1]:+.0f},{target[2]:+.0f}) ===")

    on_tick = _make_on_tick(field, [robot])

    for attempt in range(MAX_G_PER_LEG):
        loc = select_location(field)
        d = distance(target, loc)
        if d <= ARRIVE_CM:
            break
        if not in_bounds(loc[0], loc[1]):
            print("   start out of safe box — stop")
            break

        fwd, lft = compute_g(loc, target)
        print(
            f"   G{attempt + 1}: from ({loc[0]:+.0f},{loc[1]:+.0f}) "
            f"H={math.degrees(loc[2]) % 360:.0f}  d={d:.0f}cm  "
            f"G {fwd:.0f} {lft:.0f} {SPEED}"
        )

        # Anchor OTOS to camera fix.
        robot.update_world_pose(loc[0], loc[1], loc[2])
        time.sleep(0.1)

        # Compute timeout: distance at speed + 6 s margin.
        timeout_s = d * 10.0 / SPEED + 6.0

        _el, _er, outcome = robot.go_to(
            round(fwd), round(lft), SPEED,
            on_tick=on_tick,
            timeout_s=timeout_s,
        )

        # Flush remaining path data after the leg.
        if len(CAM_TRACK) >= 2:
            field.add_path(
                "camera", CAM_TRACK,
                symbol="filled_circle",
                color=(0, 200, 255),
                size_cm=1.2,
            )
        if len(ODO_TRACK) >= 2:
            field.add_path(
                "odometry", ODO_TRACK,
                symbol="x",
                color=(255, 235, 130),
                size_cm=0.8,
            )

        if outcome == "aborted":
            print("   [BOUNDS-ABORT]")
            break

    final = select_location(field)
    print(
        f"   reached ({final[0]:+.0f},{final[1]:+.0f})  "
        f"err={distance(target, final):.1f}cm"
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Playfield random tour demo")
    ap.add_argument("--port", default=None,
                    help="Serial port (relay USB, e.g. /dev/cu.usbmodem2121402). "
                         "Auto-detected if omitted.")
    ap.add_argument("--hops", type=int, default=HOPS)
    ap.add_argument("--speed", type=int, default=SPEED)
    args = ap.parse_args(argv)

    # Defer all hardware imports until inside main() so that
    # 'import playfield_random_tour' and ast.parse work without hardware.
    from robot_radio.field.playfield import Playfield
    from robot_radio.robot.connection import make_robot

    # Build a minimal args-like object for make_robot's port resolution.
    class _Args:
        port = args.port
        verbose = False

    robot, conn, _ = make_robot(
        port=args.port,
        mode=None,
        verbose=False,
        args=_Args(),
    )

    field = Playfield.open()

    # Load slot list from the playfield.
    slots: list[tuple] = []
    for rec in field._playfield_data.get("rectangles", []):
        slots.append((rec["slug"], float(rec["x"]), float(rec["y"])))
    if not slots:
        print("No rectangles found in playfield.json — check playfield config.",
              file=sys.stderr)
        conn.disconnect()
        field.close()
        return 1

    # Push config to firmware.
    robot.set_config(sTimeout=60000, turnGate=35, alphaYaw=0, yawRateMax=60)

    # Clear any stale overlay paths.
    field.clear_paths()

    try:
        for i in range(1, args.hops + 1):
            hop(i, field, robot, slots)
            time.sleep(0.3)
    finally:
        robot.stop()
        try:
            conn.disconnect()
        except Exception:
            pass
        field.close()

    print("tour done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
