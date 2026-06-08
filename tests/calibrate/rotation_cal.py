#!/usr/bin/env python3
"""rotation_cal.py — automated camera-based rotation (turn) calibration.

Fully automated using the overhead camera (AprilTag on the robot):

  Phase 1 (fit):   3 rounds. Orient near 90° (camera feedback), spin +360° then
                   −360° RAW (gain=1), measure the actual rotation each way →
                   fit the per-direction turn gain (CCW, CW).
  Phase 2 (verify):3 rounds. With that gain set, spin ±1080° (3 turns each way)
                   and report how close to ±1080° we land.

The turn is an in-place spin (D with opposite wheels). The camera yaw wraps at
±180°, so a spin's actual rotation = (commanded full turns)×360 + the wrapped
residual — valid as long as the turn lands within ±180° of the commanded full
turns (true for slip > ~0.5 on the raw 360°, and near-exact once the gain is on).

Writes rotation_gain / rotation_gain_neg to the robot config (unless --no-write).
Robot must be in camera view (i.e. on the playfield, reached via the relay).

Usage: uv run python tests/calibrate/rotation_cal.py [--port DEV] [--spd 100] [--no-write]
"""
import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"
ROBOT_TAG = 100


def wrap180(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


class _Cam:
    """Overhead camera ground truth from the aprilcam daemon, with retry."""

    def __init__(self, tag_id: int = ROBOT_TAG):
        self.tag_id = tag_id
        self._connect()

    def _connect(self) -> None:
        from aprilcam.client.control import DaemonControl
        from aprilcam.config import Config
        self.dc = DaemonControl.connect_default(Config.load())
        cams = self.dc.list_cameras()
        if not cams:
            raise SystemExit("aprilcam: no cameras open")
        c0 = cams[0]
        self.cam = c0 if isinstance(c0, str) else getattr(c0, "id", c0)

    def _reconnect(self) -> None:
        try:
            self.dc.close()
        except Exception:
            pass
        time.sleep(0.4)
        self._connect()

    def _read_once(self):
        out = []
        tf = self.dc.get_tags(self.cam)
        for t in tf.tags:
            if t.id == self.tag_id and getattr(t, "world_xy", None) is not None:
                out.append((float(t.world_xy[0]), float(t.world_xy[1]), float(t.yaw)))
        return out

    def heading_deg(self, samples: int = 6, settle: float = 0.05):
        """Median robot-tag yaw in degrees, or None if the tag isn't seen."""
        yaws = []
        attempts = 0
        while len(yaws) < samples and attempts < samples * 4:
            attempts += 1
            try:
                for (_x, _y, yaw) in self._read_once():
                    yaws.append(math.degrees(yaw))
            except Exception:
                self._reconnect()
            time.sleep(settle)
        if not yaws:
            return None
        # Circular median via the dominant value (headings are clustered between
        # turns); align to the first sample to avoid the ±180 wrap.
        base = yaws[0]
        aligned = [base + wrap180(y - base) for y in yaws]
        return wrap180(statistics.median(aligned))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--spd", type=int, default=100, help="wheel speed mm/s for turns")
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--no-write", action="store_true")
    args = ap.parse_args()

    from robot_radio.io.serial_conn import SerialConnection, list_serial_ports
    from robot_radio.robot.protocol import NezhaProtocol
    from robot_radio.config.robot_config import load_robot_config

    cfg = load_robot_config(_TOVEZ_JSON)
    trackwidth = float(getattr(cfg.geometry, "trackwidth", None) or 126.0)
    tag = getattr(cfg.vision, "robot_tag_id", None) or ROBOT_TAG
    DEG2ARC = (math.pi / 180.0) * (trackwidth / 2.0)   # wheel mm per degree of body turn
    SPD = args.spd

    port = args.port or (list_serial_ports() or [None])[0]
    if port is None:
        print("ERROR: no serial port"); return 2
    print(f"  port: {port}   trackwidth={trackwidth:.0f}mm  tag={tag}")
    conn = SerialConnection(port=port)
    conn.connect()
    proto = NezhaProtocol(conn)
    time.sleep(0.5)

    # Push calibration so the turn drives are tuned (kI etc.) and consistent.
    try:
        ml = cfg.calibration.mm_per_wheel_deg_left
        mr = cfg.calibration.mm_per_wheel_deg_right
        if ml and mr:
            proto.set_config(ml=ml, mr=mr)
        ctrl = getattr(cfg, "control", None)
        if ctrl is not None:
            for key, val in (("vel.kP", ctrl.vel_kp), ("vel.kI", ctrl.vel_ki),
                             ("vel.kFF", ctrl.vel_kff), ("vel.iMax", ctrl.vel_imax),
                             ("vel.kAw", ctrl.vel_kaw), ("vel.filt", ctrl.vel_filt),
                             ("sync", ctrl.sync)):
                if val is not None:
                    proto.send(f"SET {key}={val:g}", 200)
    except Exception as exc:
        print(f"  WARN config push: {exc}")

    cam = _Cam(tag_id=tag)

    def heading():
        return cam.heading_deg()

    def raw_turn(left, right, arc_mm):
        arc_mm = max(1, int(round(arc_mm)))
        proto.distance(left, right, arc_mm)
        time.sleep(arc_mm / max(SPD, 1) * 1.3 + 1.5)   # time-based (robust over radio)
        proto.stop()
        time.sleep(0.6)

    # Probe: which wheel sign gives +yaw (CCW)?
    h_a = heading()
    raw_turn(SPD, -SPD, 30 * DEG2ARC)
    h_b = heading()
    probe = wrap180((h_b or 0.0) - (h_a or 0.0))
    lpos_is_ccw = probe > 0.0   # (L+,R−) produced +yaw
    print(f"  probe turn (L+,R−): Δyaw={probe:+.1f}°  → (L+,R−) is "
          f"{'CCW' if lpos_is_ccw else 'CW'}")

    def turn(deg, gain):
        """Spin `deg` (signed, +=CCW), compensating slip with `gain`."""
        arc = abs(deg) / max(gain, 1e-3) * DEG2ARC
        want_ccw = deg > 0
        if want_ccw == lpos_is_ccw:
            raw_turn(SPD, -SPD, arc)
        else:
            raw_turn(-SPD, SPD, arc)

    def orient_to(target_deg, gain):
        for _ in range(6):
            h = heading()
            if h is None:
                time.sleep(0.3); continue
            err = wrap180(target_deg - h)
            if abs(err) < 6.0:
                return h
            turn(err, gain)
        return heading()

    def measure_spin(target_deg, gain):
        """Spin target_deg (using gain); return the actual rotation (camera)."""
        h0 = heading()
        turn(target_deg, gain)
        h1 = heading()
        if h0 is None or h1 is None:
            return None
        base = round(target_deg / 360.0) * 360.0
        return base + wrap180(h1 - h0)

    # ── Phase 1: fit the gain from ±360° raw (gain=1) ────────────────────────
    print("\n  PHASE 1 — fit gain (±360°, raw):")
    ccw, cw = [], []
    for r in range(args.rounds):
        orient_to(90.0, 1.0)
        a = measure_spin(+360.0, 1.0)
        b = measure_spin(-360.0, 1.0)
        print(f"    round {r+1}: CCW actual={a if a is None else round(a,1)}°   "
              f"CW actual={b if b is None else round(b,1)}°")
        if a is not None and abs(wrap180(a - 360.0)) < 150:
            ccw.append(a)
        if b is not None and abs(wrap180(b + 360.0)) < 150:
            cw.append(abs(b))
    gain_ccw = (sum(ccw) / len(ccw)) / 360.0 if ccw else 1.0
    gain_cw = (sum(cw) / len(cw)) / 360.0 if cw else 1.0
    print(f"  → GAIN  ccw={gain_ccw:.4f}  cw={gain_cw:.4f}   "
          f"(actual/commanded; <1 = slip)")

    # ── Phase 2: verify at ±1080° with the gain applied ──────────────────────
    print("\n  PHASE 2 — verify (±1080°, with gain):")
    e_ccw, e_cw = [], []
    for r in range(args.rounds):
        orient_to(90.0, gain_ccw)
        a = measure_spin(+1080.0, gain_ccw)
        b = measure_spin(-1080.0, gain_cw)
        ea = None if a is None else a - 1080.0
        eb = None if b is None else b + 1080.0
        print(f"    round {r+1}: CCW={a if a is None else round(a,1)}° "
              f"(err {ea if ea is None else round(ea,1)}°)   "
              f"CW={b if b is None else round(b,1)}° "
              f"(err {eb if eb is None else round(eb,1)}°)")
        if ea is not None:
            e_ccw.append(ea)
        if eb is not None:
            e_cw.append(eb)
    if e_ccw:
        print(f"  CCW @1080 mean err: {sum(e_ccw)/len(e_ccw):+.1f}°")
    if e_cw:
        print(f"  CW  @1080 mean err: {sum(e_cw)/len(e_cw):+.1f}°")

    proto.stop()
    try:
        conn.disconnect()
    except Exception:
        pass

    if args.no_write:
        print(f"\n  --no-write: NOT writing. rotation_gain={gain_ccw:.4f} "
              f"rotation_gain_neg={gain_cw:.4f}")
    else:
        d = json.loads(_TOVEZ_JSON.read_text())
        d.setdefault("calibration", {})["rotation_gain"] = round(gain_ccw, 4)
        d["calibration"]["rotation_gain_neg"] = round(gain_cw, 4)
        _TOVEZ_JSON.write_text(json.dumps(d, indent=2) + "\n")
        print(f"\n  wrote rotation_gain={gain_ccw:.4f} "
              f"rotation_gain_neg={gain_cw:.4f} to {_TOVEZ_JSON.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
