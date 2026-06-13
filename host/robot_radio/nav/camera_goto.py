"""camera_goto — camera-feedback navigation helpers for rogo.

Extracted from host/robot_radio/io/cli.py as part of ticket 035-001
(pose-authority sprint A1).  These functions contain the inline
closed-loop control logic that was previously embedded directly in the
CLI command handlers.  Moving them here makes the logic reusable and
unit-testable without importing the full CLI module.

Public API:
    go_to_world_camera(proto, read_pose, target_x, target_y,
                       cruise, turn_speed, gate_deg, arrive_cm,
                       max_secs, log=None)
    spin_to_yaw_camera(proto, read_pose, target_deg, speed,
                       tol_deg, max_secs=8.0, log=None)
    crawl_drive_distance(robot, speed_mms, target_mm, log=None)

Import constraint: this module must NOT import from robot_radio.io.cli.
"""

from __future__ import annotations

import math
import time
from typing import Callable

# ---------------------------------------------------------------------------
# Crawl-mode (pulse-train) constants — mirror of cli.py module-level values.
# These are calibrated values; keep them in sync if cli.py changes.
# ---------------------------------------------------------------------------
CRAWL_PULSE_SPEED = 300   # mm/s commanded during the pulse
CRAWL_PULSE_MS = 80
CRAWL_DELAY_MS_MIN = 20
CRAWL_MM_PER_PULSE = 6.53


def _noop(msg: str) -> None:  # noqa: ARG001
    """Default no-op log callable."""


def crawl_drive_distance(
    robot,
    speed_mms: int,
    target_mm: int,
    log: Callable[[str], None] = _noop,
) -> tuple[int, int]:
    """Pulse-train slow drive when |speed| is below the firmware MIN clamp.

    Each pulse is a short T command at CRAWL_PULSE_SPEED for CRAWL_PULSE_MS,
    followed by ``delay_ms`` of coast.  We choose ``delay_ms`` so the average
    speed across pulse+delay matches ``speed_mms``, clamped at the calibration
    floor (delay >= CRAWL_DELAY_MS_MIN).

    Stops after the number of pulses needed to cover ``target_mm`` at the
    calibrated mm-per-pulse.  Returns the firmware's final encoder reading.

    Extracted from cli.py ``_crawl_drive_distance`` (ticket 035-001).
    """
    eff_v = abs(speed_mms)
    if eff_v < 1:
        raise SystemExit("Error: crawl speed must be > 0")
    target_mm = abs(target_mm)
    if target_mm < 1:
        raise SystemExit("Error: crawl distance must be > 0")

    # Cycle period to hit the requested effective speed.
    cycle_ms = max(
        CRAWL_PULSE_MS + CRAWL_DELAY_MS_MIN,
        int(round(CRAWL_MM_PER_PULSE * 1000.0 / eff_v)),
    )
    delay_ms = cycle_ms - CRAWL_PULSE_MS
    pulses = max(1, int(round(target_mm / CRAWL_MM_PER_PULSE)))
    sign = 1 if speed_mms >= 0 else -1
    pulse_v = sign * CRAWL_PULSE_SPEED

    # Cap the actual effective speed reported back if we hit the floor.
    eff_actual = CRAWL_MM_PER_PULSE * 1000.0 / cycle_ms

    log(
        f"crawl mode: {pulses} pulses × T+{pulse_v}+{pulse_v}+{CRAWL_PULSE_MS} "
        f"(delay {delay_ms} ms, eff ≈ {eff_actual:.1f} mm/s, "
        f"target {target_mm} mm ≈ {pulses * CRAWL_MM_PER_PULSE:.1f} mm)"
    )

    enc_l, enc_r = 0, 0
    for _i in range(pulses):
        enc_l, enc_r = robot.speed_for_time(pulse_v, pulse_v, CRAWL_PULSE_MS)
        if delay_ms > 0:
            time.sleep(delay_ms / 1000.0)
    return enc_l, enc_r


def spin_to_yaw_camera(
    proto,
    read_pose: Callable[..., tuple[float, float, float] | None],
    target_deg: float,
    speed: int,
    tol_deg: float,
    max_secs: float = 8.0,
    log: Callable[[str], None] = _noop,
) -> float | None:
    """Velocity-projected closed-loop spin to an absolute world yaw (deg).

    Reads yaw from ``read_pose()`` (a callable returning (x, y, yaw_rad) or
    None).  Computes the shortest signed delta to ``target_deg``, then drives
    a streaming-S spin with velocity-projected stop.  Returns the final signed
    yaw error in degrees, or None if the pose source never reported the robot.

    Extracted from cli.py ``_daemon_spin_to_yaw`` (ticket 035-001).
    """
    COAST_S = 0.10
    p = read_pose(3.0)
    if p is None:
        return None
    cur_deg = math.degrees(p[2])
    diff = ((target_deg - cur_deg + 180.0) % 360.0) - 180.0
    # v2: no set_watchdog verb; use SET sTimeout=<ms> to configure firmware watchdog.
    proto.set_config(sTimeout=500)
    prev_cam = p[2]
    prev_t = time.monotonic()
    total = 0.0
    ang_vel = 0.0
    t0 = prev_t
    while True:
        p = read_pose(0.2)
        now = time.monotonic()
        if p is not None:
            d = ((p[2] - prev_cam + math.pi) % (2.0 * math.pi)) - math.pi
            d_deg = math.degrees(d)
            dt = now - prev_t
            if abs(d_deg) <= 30.0 and dt > 0:
                ang_vel = 0.6 * ang_vel + 0.4 * (d_deg / dt)
                total += d_deg
            prev_cam = p[2]
            prev_t = now
        remaining = diff - total
        projected_err = diff - (total + ang_vel * COAST_S)
        if abs(projected_err) <= tol_deg and abs(ang_vel) > 5.0:
            proto.stop()
            time.sleep(max(COAST_S * 1.5, 0.4))
            break
        if now - t0 > max_secs:
            proto.stop()
            break
        direction = 1 if remaining > 0 else -1
        proto.drive(-direction * speed, direction * speed)
        time.sleep(0.03)
    p = read_pose(1.5)
    if p is None:
        return None
    return ((target_deg - math.degrees(p[2]) + 180.0) % 360.0) - 180.0


def go_to_world_camera(
    proto,
    read_pose: Callable[..., tuple[float, float, float] | None],
    target_x: float,
    target_y: float,
    cruise: int,
    turn_speed: int,
    gate_deg: float,
    arrive_cm: float,
    max_secs: float,
    log: Callable[[str], None] = _noop,
) -> None:
    """Turn to face an absolute world point, then drive there (closed-loop).

    Reads the robot tag's world pose from ``read_pose()`` (callable returning
    (x_cm, y_cm, yaw_rad) or None), then runs a closed-loop pure-pursuit
    controller: it turns in place toward the target when badly mis-aimed,
    otherwise drives forward with mild steering correction, continuously
    re-reading until within the arrival tolerance.

    Prints start/final position lines to stdout (same as the original
    ``cmd_goto`` did).  Raises SystemExit on fatal conditions.

    Parameters
    ----------
    proto:
        NezhaProtocol instance for sending drive commands.
    read_pose:
        Callable(timeout_s) -> (x_cm, y_cm, yaw_rad) | None.
    target_x, target_y:
        Target world position in cm (daemon A1-centred frame).
    cruise:
        Forward drive speed in mm/s.
    turn_speed:
        Wheel speed during in-place turns in mm/s.
    gate_deg:
        Turn-in-place threshold: if heading error exceeds this, spin to aim.
    arrive_cm:
        Arrival tolerance in cm.
    max_secs:
        Maximum duration for the whole move.
    log:
        Optional debug-log callable (receives a single str).

    Extracted from cli.py ``cmd_goto`` core loop (ticket 035-001).
    """

    def _wrap(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    # Control parameters (kept identical to original cmd_goto).
    TICK_S = 0.05
    WATCHDOG_MS = 800
    AIM_GATE_DEG = gate_deg
    REAIM_GATE_DEG = gate_deg * 1.8
    SPIN_TOL_DEG = 4.0
    STEER_KP = 1.0
    SLOW_RADIUS_CM = 18.0
    MIN_DRIVE = 70
    BURST_MAX_S = 1.2

    p = read_pose(3.0)
    if p is None:
        import sys
        sys.exit(
            "Error: daemon could not see robot tag "
            "(calibrated). Is the playfield calibrated and the robot "
            "in view?"
        )
    rx, ry, yaw = p
    d0 = math.hypot(target_x - rx, target_y - ry)
    print(
        f"start: robot=({rx:.1f}, {ry:.1f}) yaw={math.degrees(yaw):+.0f}°  "
        f"target=({target_x:.1f}, {target_y:.1f})  dist={d0:.1f}cm"
    )
    if d0 <= arrive_cm:
        print(f"Already within {arrive_cm:.1f}cm (dist={d0:.1f}cm); done.")
        return

    # v2: no set_watchdog verb; use SET sTimeout=<ms> to configure firmware watchdog.
    proto.set_config(sTimeout=WATCHDOG_MS)
    t_start = time.monotonic()

    while True:
        if time.monotonic() - t_start > max_secs:
            proto.stop()
            print(f"WARNING: hit {max_secs:.0f}s timeout; not at target")
            break

        p = read_pose(1.0)
        if p is None:
            proto.stop()
            continue
        rx, ry, yaw = p
        dx, dy = target_x - rx, target_y - ry
        dist = math.hypot(dx, dy)
        if dist <= arrive_cm:
            proto.stop()
            break

        motion_dir = math.atan2(dy, dx)
        req_yaw = _wrap(motion_dir - math.pi / 2)   # forward = (-sinθ, cosθ)
        head_err = _wrap(req_yaw - yaw)

        # Aim with the proven velocity-projected spin if badly off heading.
        if abs(head_err) > math.radians(AIM_GATE_DEG):
            log(
                f"aim: head_err={math.degrees(head_err):+.0f}° → "
                f"spin to {math.degrees(req_yaw):+.0f}°"
            )
            spin_to_yaw_camera(
                proto, read_pose, math.degrees(req_yaw),
                turn_speed, SPIN_TOL_DEG, log=log,
            )
            continue

        # Forward burst toward the target, monitored — break out to re-aim
        # on drift, on arrival, or when distance stops decreasing.
        log(f"drive: dist={dist:.1f}cm head_err={math.degrees(head_err):+.0f}°")
        b_start = time.monotonic()
        best = dist
        while time.monotonic() - b_start < BURST_MAX_S:
            q = read_pose(0.25)
            if q is None:
                break
            rx, ry, yaw = q
            dx, dy = target_x - rx, target_y - ry
            dist = math.hypot(dx, dy)
            if dist <= arrive_cm:
                break
            he = _wrap(_wrap(math.atan2(dy, dx) - math.pi / 2) - yaw)
            if abs(he) > math.radians(REAIM_GATE_DEG):
                break
            if dist > best + 2.0:   # overshot / moving away → re-aim
                break
            best = min(best, dist)
            v = MIN_DRIVE + (cruise - MIN_DRIVE) * min(1.0, dist / SLOW_RADIUS_CM)
            steer = max(-0.5, min(0.5, STEER_KP * he))
            proto.drive(
                int(round(v * (1.0 - steer))),
                int(round(v * (1.0 + steer))),
            )
            time.sleep(TICK_S)
        proto.stop()
        time.sleep(0.15)

    # Final report.
    time.sleep(0.3)
    end = read_pose(2.0)
    if end is not None:
        ex, ey, eyaw = end
        err = math.hypot(target_x - ex, target_y - ey)
        elapsed = time.monotonic() - t_start
        print(
            f"final=({ex:.1f}, {ey:.1f})cm yaw={math.degrees(eyaw):+.0f}°  "
            f"error={err:.1f}cm  ({elapsed:.1f}s)"
        )
    else:
        print("done (lost robot tag for final readout)")
