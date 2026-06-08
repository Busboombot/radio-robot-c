#!/usr/bin/env python3
"""turn_test.py — camera-verified turn accuracy with accel-limited profiles.

The turn primitive is a TRAPEZOIDAL angular-velocity profile: ramp 0→peak at
max_rot_accel (deg/s^2, from config), cruise at the requested speed (deg/s),
ramp peak→0 at max_rot_accel — executed by streaming wheel-velocity (S) commands.
The 2-param model (gain + offset, per direction) corrects the commanded angle.

  turn(deg, dps)   open-loop accel-limited spin of `deg` at peak `dps` deg/s
  turn2(target)    closed-loop: iterate turn() with camera feedback to a heading

Three checks: A. dead-reckoning accuracy, B. closed-loop turn2, C. accumulation.
Robot in camera view (playfield, via relay).
Usage: uv run python tests/calibrate/turn_test.py [--dps 90] [--accel 300] [--tol 2.5]
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rotation_cal import _Cam, wrap180   # reuse camera + helper

_REPO_ROOT = Path(__file__).resolve().parents[2]
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"
PROFILE_HZ = 20            # velocity-stream rate during a turn


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--dps", type=float, default=90.0, help="peak turn speed, deg/s")
    ap.add_argument("--accel", type=float, default=None, help="max rot accel deg/s^2 (override config)")
    ap.add_argument("--tol", type=float, default=2.5, help="closed-loop tolerance deg")
    args = ap.parse_args()

    from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
    from robot_radio.robot.protocol import NezhaProtocol
    from robot_radio.config.robot_config import load_robot_config

    cfg = load_robot_config(_TOVEZ_JSON)
    tw = float(getattr(cfg.geometry, "trackwidth", None) or 126.0)
    cal = cfg.calibration
    ctrl = getattr(cfg, "control", None)
    gain_ccw = float(getattr(cal, "rotation_gain", None) or 1.0)
    gain_cw = float(getattr(cal, "rotation_gain_neg", None) or gain_ccw)
    off_ccw = float(getattr(cal, "rotation_offset_deg", None) or 0.0)
    off_cw = float(getattr(cal, "rotation_offset_deg_neg", None) or 0.0)
    max_accel = args.accel or float(getattr(ctrl, "max_rot_accel_dps2", None) or 300.0)
    DEG2ARC = (math.pi / 180.0) * (tw / 2.0)   # wheel mm/s per deg/s (and mm per deg)
    DPS = args.dps
    tag = getattr(cfg.vision, "robot_tag_id", None) or 100
    print(f"  model: CCW gain={gain_ccw:.4f} off={off_ccw:+.2f}°  CW gain={gain_cw:.4f} "
          f"off={off_cw:+.2f}°   peak={DPS:.0f}°/s  accel={max_accel:.0f}°/s²  tag={tag}")

    port = args.port or (list_serial_ports() or [None])[0]
    conn = SerialConnection(port=port)
    conn.connect()
    proto = NezhaProtocol(conn)
    time.sleep(0.5)
    try:
        if cal.mm_per_wheel_deg_left and cal.mm_per_wheel_deg_right:
            proto.set_config(ml=cal.mm_per_wheel_deg_left, mr=cal.mm_per_wheel_deg_right)
        if ctrl is not None:
            for k, v in (("vel.kP", ctrl.vel_kp), ("vel.kI", ctrl.vel_ki),
                         ("vel.kFF", ctrl.vel_kff), ("vel.iMax", ctrl.vel_imax),
                         ("vel.kAw", ctrl.vel_kaw), ("vel.filt", ctrl.vel_filt),
                         ("sync", ctrl.sync)):
                if v is not None:
                    proto.send(f"SET {k}={v:g}", 200)
        proto.send("SET sTimeout=600", 200)   # keep the velocity stream alive between S frames
    except Exception as exc:
        print(f"  WARN config push: {exc}")

    cam = _Cam(tag_id=tag)

    def heading():
        return cam.heading_deg()

    def _profile_time(A):
        """(t_ramp, t_cruise, t_total, peak) for a trapezoid covering A deg."""
        a, vpk = max_accel, DPS
        ramp_ang = vpk * vpk / (2.0 * a)
        if 2.0 * ramp_ang >= A:            # triangular — never reaches DPS
            vpk = math.sqrt(A * a)
            return vpk / a, 0.0, 2.0 * (vpk / a), vpk
        t_ramp = vpk / a
        t_cruise = (A - 2.0 * ramp_ang) / vpk
        return t_ramp, t_cruise, 2.0 * t_ramp + t_cruise, vpk

    def _stream_turn(geom_deg):
        """Trapezoidal spin of |geom_deg| (already model-corrected), streamed."""
        A = abs(geom_deg)
        if A <= 0.5:
            return
        t_ramp, t_cruise, t_total, vpk = _profile_time(A)
        lpos = (geom_deg > 0) == lpos_ccw      # which wheel goes positive
        dt = 1.0 / PROFILE_HZ
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            if t >= t_total:
                break
            if t < t_ramp:
                w = max_accel * t
            elif t < t_ramp + t_cruise:
                w = vpk
            else:
                w = max(0.0, max_accel * (t_total - t))
            v = int(round(w * DEG2ARC))
            proto.drive(v, -v) if lpos else proto.drive(-v, v)
            time.sleep(dt)
        proto.stop()
        time.sleep(0.4)

    def turn(deg, dps=None):
        """Open-loop accel-limited turn, signed (+=CCW), via the 2-param model."""
        gain, offset = (gain_ccw, off_ccw) if deg >= 0 else (gain_cw, off_cw)
        A = (abs(deg) - offset) / max(gain, 1e-3)
        _stream_turn(math.copysign(A, deg))

    def turn2(target, iters=6):
        """Closed-loop: iterate accel-limited turns with camera feedback."""
        for _ in range(iters):
            h = heading()
            if h is None:
                time.sleep(0.3); continue
            err = wrap180(target - h)
            if abs(err) <= args.tol:
                return h
            turn(err)
        return heading()

    # Probe the turn sign with a short constant-velocity pulse.
    ha = heading()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.5:
        v = int(round(50.0 * DEG2ARC))
        proto.drive(v, -v)
        time.sleep(1.0 / PROFILE_HZ)
    proto.stop(); time.sleep(0.4)
    hb = heading()
    lpos_ccw = wrap180((hb or 0.0) - (ha or 0.0)) > 0.0
    _, _, tt90, pk90 = _profile_time(90.0)
    print(f"  probe: (L+,R−) is {'CCW' if lpos_ccw else 'CW'}   "
          f"(a 90° turn: {tt90:.2f}s, peaks {pk90:.0f}°/s)")

    # ── A. Dead-reckoning (single open-loop turn from 0) ─────────────────────
    print("\n  A. DEAD-RECKONING (open-loop, one accel-limited turn from 0):")
    a_errs = []
    for N in (90, -90, 135, -135, 45, -45):
        turn2(0.0)
        h0 = heading()
        turn(N)
        h1 = heading()
        if h0 is None or h1 is None:
            print(f"    turn {N:+4d}°: (no camera)"); continue
        actual = wrap180(h1 - h0)
        err = wrap180(actual - N)
        a_errs.append(abs(err))
        print(f"    turn {N:+4d}°: actual={actual:+6.1f}°  err={err:+5.1f}°")
    if a_errs:
        print(f"  → dead-reckoning mean |err| = {sum(a_errs)/len(a_errs):.1f}°")

    # ── B. Closed-loop turn2 (camera-verified to target heading) ─────────────
    print("\n  B. CLOSED-LOOP (turn2, camera-verified):")
    b_errs = []
    for target in (0, 90, -90, 135, -45):
        turn2(target)
        h = heading()
        if h is None:
            print(f"    turn2 → {target:+4d}°: (no camera)"); continue
        err = wrap180(h - target)
        b_errs.append(abs(err))
        ok = "OK" if abs(err) <= max(args.tol + 1.0, 3.0) else "FAIL"
        print(f"    turn2 → {target:+4d}°: final={h:+6.1f}°  err={err:+5.1f}°  [{ok}]")
    if b_errs:
        print(f"  → closed-loop mean |err| = {sum(b_errs)/len(b_errs):.1f}°")

    # ── C. Dead-reckoning accumulation (four +90° should close the circle) ────
    print("\n  C. ACCUMULATION (four open-loop +90° turns = full circle):")
    turn2(0.0)
    start = heading()
    for i in range(4):
        turn(90.0)
        h = heading()
        if start is not None and h is not None:
            print(f"    after {i+1}×90°: heading={h:+6.1f}°")
    end = heading()
    if start is not None and end is not None:
        print(f"  → closed the loop within {wrap180(end - start):+.1f}° "
              f"after 360° of open-loop turning")

    proto.stop()
    try:
        conn.disconnect()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
