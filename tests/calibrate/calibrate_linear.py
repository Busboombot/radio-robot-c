#!/usr/bin/env python3
"""calibrate_linear.py — interactive linear-distance calibration for tovez.

Closed-loop calibration of BOTH onboard distance estimators against ground
truth, modeled on the prior repo's test/calibrate/calibrate_linear.py but
extended to (a) use the overhead camera as automatic ground truth and (b)
update the robot every trial so accuracy visibly improves run to run.

Per trial:
  1. Laser (port 4) is on so you can mark the start point on the floor.
  2. Press Enter — the robot drives forward a fixed distance (default 900 mm)
     with ONE blocking `D` command (deliberate; it self-stops).
  3. The camera (AprilTag 100) reports how far it actually moved (validation),
     and the encoders and OTOS report what THEY think it moved.
  4. You type the tape-measure distance (cm) — the DEFINITIVE ground truth.
     The camera is scored against it too, so its distance error is measured.
  5. Both onboard estimators are calibrated toward the truth and PUSHED TO THE ROBOT:
       encoders -> mm-per-wheel-degree (SET ml/mr)
       OTOS     -> linear scalar int8 (OL)
     so the next trial starts from the improved values.
  6. Repeat as many trials as you like. 'q' (or Ctrl-C) stops.
  7. Running per-estimator mean +/- stddev of the pre-correction error is shown
     so you can watch repeatability; the latest-trial error should shrink.
  8. On exit the final calibration is written to data/robots/tovez.json
     (unless --no-write).

Ground truth: the tape measure (definitive). The camera is a tracked estimator
— its distance error vs the tape is measured and a camera_distance_scale logged.

Run (aprilcam + pyserial come from this project's `calibrate` group):
    uv run python tests/calibrate/calibrate_linear.py
Options: --distance MM  --speed MMPS  --port DEV  --field "W H"  --no-write
"""

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Library imports — all hardware work is deferred to main().
# Import guard: this file lives under tests/calibrate/ which is in pytest
# testpaths. These top-level imports must NOT open serial, start cameras, or
# define any test_* functions. All hardware work is inside main().
# ---------------------------------------------------------------------------

from robot_radio.robot.nezha import Nezha, RobotNotFoundError
from robot_radio.robot.protocol import NezhaProtocol, parse_tlm
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.config.robot_config import load_robot_config, RobotConfig

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_DISTANCE_MM = 900     # 90 cm
DEFAULT_SPEED_MMPS = 200
ROBOT_TAG = 100               # tovez wears AprilTag 100
LASER_PORT_DEFAULT = 4        # J4 digital port — the line laser

_REPO_ROOT = Path(__file__).resolve().parents[2]
_TOVEZ_JSON = _REPO_ROOT / "data" / "robots" / "tovez.json"


# ---------------------------------------------------------------------------
# OTOS scalar conversions
# ---------------------------------------------------------------------------

def scale_to_int8(scale: float) -> int:
    """OTOS linear float scale -> int8 register value (clamped)."""
    return max(-128, min(127, round((scale - 1.0) / 0.001)))


def int8_to_scale(n: int) -> float:
    return 1.0 + n * 0.001


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def dist2d_mm(a_cm: tuple | None, b_cm: tuple | None) -> float | None:
    """Euclidean distance between two (x_cm, y_cm) camera points, in mm."""
    if not a_cm or not b_cm:
        return None
    return math.hypot(b_cm[0] - a_cm[0], b_cm[1] - a_cm[1]) * 10.0


def predict_end_mm(pos_cm: tuple, yaw_rad: float, distance_mm: float) -> tuple:
    """Predicted end (x_mm, y_mm) if robot drives distance_mm along its yaw."""
    x_mm = pos_cm[0] * 10.0
    y_mm = pos_cm[1] * 10.0
    return (x_mm + distance_mm * math.cos(yaw_rad),
            y_mm + distance_mm * math.sin(yaw_rad))


# ---------------------------------------------------------------------------
# Camera pose helper using aprilcam directly
# ---------------------------------------------------------------------------

class _Cam:
    """Overhead camera ground truth from the aprilcam daemon, with retry."""

    def __init__(self, tag_id: int = ROBOT_TAG):
        self.tag_id = tag_id
        self._connect()

    def _connect(self) -> None:
        from aprilcam.client.control import DaemonControl
        from aprilcam.config import Config
        self._DaemonControl = DaemonControl
        self._Config = Config
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

    def _read_once(self) -> list[tuple]:
        """One frame -> list of (x_cm, y_cm, yaw_rad) for the robot tag."""
        out = []
        tf = self.dc.get_tags(self.cam)
        for t in tf.tags:
            if t.id == self.tag_id and getattr(t, "world_xy", None) is not None:
                out.append((float(t.world_xy[0]),   # cm (aprilcam units)
                            float(t.world_xy[1]),
                            float(t.yaw)))
        return out

    def pose(self, samples: int = 6, settle: float = 0.05) -> tuple | None:
        """Median (x_cm, y_cm, yaw_rad) of the robot tag over several frames.

        Returns None if the tag is never seen. Retries/reconnects on gRPC error.
        Position is in cm (aprilcam world_xy units).
        """
        xs, ys, yaws = [], [], []
        attempts = 0
        while len(xs) < samples and attempts < samples * 4:
            attempts += 1
            try:
                for (x, y, yaw) in self._read_once():
                    xs.append(x)
                    ys.append(y)
                    yaws.append(yaw)
            except Exception:
                self._reconnect()
            time.sleep(settle)
        if not xs:
            return None
        cy = math.atan2(statistics.fmean(math.sin(v) for v in yaws),
                        statistics.fmean(math.cos(v) for v in yaws))
        return (statistics.median(xs), statistics.median(ys), cy)

    def close(self) -> None:
        try:
            self.dc.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Snap helper (send SNAP, parse TLM for enc)
# ---------------------------------------------------------------------------

def _snap_enc(proto: NezhaProtocol) -> tuple[int, int] | None:
    """Send SNAP and return enc (left_mm, right_mm) from the TLM response, or None."""
    resp = proto.send("SNAP", 500)
    for line in resp.get("responses", []):
        frame = parse_tlm(line)
        if frame and frame.enc is not None:
            return frame.enc
    return None


# ---------------------------------------------------------------------------
# Config write-back
# ---------------------------------------------------------------------------

def _save_calibration(ml: float, mr: float, otos_scale: float,
                      cam_scale: float | None) -> None:
    """Merge updated calibration into tovez.json and write it back."""
    text = _TOVEZ_JSON.read_text()
    cfg = json.loads(text)
    cal = cfg.setdefault("calibration", {})
    cal["mm_per_wheel_deg_left"] = round(ml, 5)
    cal["mm_per_wheel_deg_right"] = round(mr, 5)
    cal["otos_linear_scale"] = round(otos_scale, 4)
    if cam_scale is not None:
        cfg.setdefault("vision", {})["camera_distance_scale"] = round(cam_scale, 4)
    _TOVEZ_JSON.write_text(json.dumps(cfg, indent=2) + "\n")


# ---------------------------------------------------------------------------
# Reporting helpers
# ---------------------------------------------------------------------------

def banner(msg: str) -> None:
    print("\n" + "=" * 64 + f"\n  {msg}\n" + "=" * 64)


def _errs(samples: list[dict], key: str) -> list[float]:
    return [s[key] for s in samples if s.get(key) is not None]


def _print_running_stats(samples: list[dict], final: bool = False) -> None:
    for key, label in (("enc_err", "encoders"), ("otos_err", "OTOS"),
                       ("cam_err", "camera")):
        e = _errs(samples, key)
        if not e:
            continue
        mean = statistics.fmean(e)
        sd = statistics.stdev(e) if len(e) >= 2 else 0.0
        latest = e[-1]
        tail = f"  latest={latest:+.1f}%" if not final else ""
        print(f"    {label:9} error  n={len(e)}  "
              f"mean={mean:+.2f}%  stdev={sd:.2f}%{tail}")


def _print_table(samples: list[dict]) -> None:
    if not samples:
        print("  (no trials)")
        return
    print(f"  {'#':>2}  {'target':>6}  {'tape':>6}  {'cam':>7}  {'camE%':>6}  "
          f"{'enc':>7}  {'encE%':>6}  {'otos':>7}  {'otosE%':>6}")
    for i, s in enumerate(samples, 1):
        def f(v, w: int = 7, p: int = 1) -> str:
            return f"{v:>{w}.{p}f}" if isinstance(v, (int, float)) else f"{'—':>{w}}"
        print(f"  {i:>2}  {f(s['target'],6,0)}  {f(s['tape'],6,0)}  {f(s['cam'])}  "
              f"{f(s['cam_err'],6)}  {f(s['enc'])}  {f(s['enc_err'],6)}  "
              f"{f(s['otos'])}  {f(s['otos_err'],6)}")


# ---------------------------------------------------------------------------
# Calibration math helpers (importable for unit tests, no hardware)
# ---------------------------------------------------------------------------

def compute_encoder_correction(tape_mm: float, enc_mm: float,
                               ml: float, mr: float) -> tuple[float, float]:
    """Scale ml/mr proportionally so future enc readings match tape_mm."""
    k = tape_mm / enc_mm
    return ml * k, mr * k


def compute_otos_scale_correction(tape_mm: float, otos_mm: float,
                                  current_scale: float) -> float:
    """Return updated OTOS float scale that corrects toward tape_mm."""
    return current_scale * (tape_mm / otos_mm)


# ---------------------------------------------------------------------------
# Main entry point — ALL hardware work lives here.
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--distance", type=int, default=DEFAULT_DISTANCE_MM,
                    help=f"target drive distance mm (default {DEFAULT_DISTANCE_MM})")
    ap.add_argument("--speed", type=int, default=DEFAULT_SPEED_MMPS,
                    help=f"drive speed mm/s (default {DEFAULT_SPEED_MMPS})")
    ap.add_argument("--port", default=None, help="relay serial port")
    ap.add_argument("--field", default=None,
                    help='field bounds "W H" mm; refuse drives whose predicted '
                         'end is out of [0,W]x[0,H] (safety)')
    ap.add_argument("--no-write", action="store_true",
                    help="do not persist final calibration to tovez.json")
    args = ap.parse_args()

    field = None
    if args.field:
        try:
            w, h = (float(v) for v in args.field.split())
            field = (w, h)
        except ValueError:
            print(f"  ignoring bad --field {args.field!r} (want 'W H' mm)")

    # ---- load config -------------------------------------------------------
    cfg = load_robot_config(_TOVEZ_JSON)
    ml = cfg.calibration.mm_per_wheel_deg_left or 0.484
    mr = cfg.calibration.mm_per_wheel_deg_right or 0.484
    otos_scale = cfg.calibration.otos_linear_scale
    otos_int8 = scale_to_int8(otos_scale)
    laser_port = cfg.peripherals.laser_port or LASER_PORT_DEFAULT
    print(f"Loaded calibration from {_TOVEZ_JSON.name}:")
    print(f"  encoders  ml={ml:.5f}  mr={mr:.5f}  mm/deg")
    print(f"  OTOS      scale={otos_scale:.4f}  (int8={otos_int8:+d})")

    # ---- connect -----------------------------------------------------------
    port = args.port or None
    if port is None:
        from robot_radio.io.serial_conn import list_serial_ports
        ports = list_serial_ports()
        if not ports:
            print("  ERROR: no USB modem serial ports found.")
            return 2
        port = ports[0]
        print(f"  auto-selected port: {port}")

    conn = SerialConnection(port=port)
    conn.connect()

    proto = NezhaProtocol(conn)
    nezha = Nezha(proto)

    cam: _Cam | None = None
    laser_on = False
    samples: list[dict] = []

    try:
        print("\nConnecting…")
        try:
            identity = nezha.connect()
            print(f"  robot: ALIVE — {identity}")
        except RobotNotFoundError as exc:
            print(f"\n  FATAL: {exc}")
            print("    Check: is the robot powered on? in radio range? flashed?")
            return 2

        # Robot confirmed alive — now bring up the overhead camera.
        cam = _Cam()

        # Push current calibration to the robot and init the OTOS once.
        proto.set_config(ml=ml, mr=mr)
        rb = proto.otos_set_linear_scalar(otos_int8)
        proto.otos_init()
        if rb is not None:
            print(f"  pushed: ml={ml:.5f} mr={mr:.5f}  OL={otos_int8:+d} "
                  f"(readback {rb:+d})")

        # Laser on for floor marking.
        proto.port_write(laser_port, True)
        laser_on = True
        print(f"\n  Laser ON (port {laser_port}).  Mark the robot's start point.")
        print(f"  Target distance: {args.distance} mm "
              f"({args.distance/10:.0f} cm) @ {args.speed} mm/s")
        print("  Each round: Enter = drive,  q = quit.\n")

        while True:
            n = len(samples)
            try:
                key = input(f"[Round {n + 1}]  Enter to drive (q to quit): ").strip()
            except EOFError:
                break
            if key.lower() in ("q", "quit", "exit"):
                break

            # ---- safety: where are we, where will we end? ------------------
            c0 = cam.pose()
            if c0 is None:
                print("  camera can't see tag 100 — reposition into view. Skipping.")
                continue
            # c0 is (x_cm, y_cm, yaw_rad); convert for display
            x0_mm, y0_mm = c0[0] * 10.0, c0[1] * 10.0
            end = predict_end_mm(c0, c0[2], args.distance)
            print(f"  start (cam): x={x0_mm:.0f} y={y0_mm:.0f} mm  "
                  f"heading={math.degrees(c0[2]):.0f}°  → predicted end "
                  f"x={end[0]:.0f} y={end[1]:.0f} mm")
            if field and not (0 <= end[0] <= field[0] and 0 <= end[1] <= field[1]):
                print(f"  predicted end is outside the field {field} — "
                      f"reposition the robot. Not driving.")
                continue

            # ---- baseline reads --------------------------------------------
            proto.otos_zero()
            proto.zero_encoders()
            time.sleep(0.3)
            enc0 = _snap_enc(proto)
            op0 = proto.otos_get_position()   # (x_mm, y_mm, h_cdeg) or None

            # ---- drive (one deliberate blocking command) --------------------
            print(f"  driving {args.distance} mm…")
            timeout_ms = int(args.distance / max(args.speed, 1) * 1000) + 4000
            proto.distance(args.speed, args.speed, args.distance)
            outcome = proto.wait_for_evt_done("D", timeout_ms)
            proto.stop()
            time.sleep(0.4)

            # ---- after reads -----------------------------------------------
            enc1 = _snap_enc(proto)
            op1 = proto.otos_get_position()
            c1 = cam.pose()

            # ---- distances -------------------------------------------------
            enc_mm: float | None = None
            if enc0 is not None and enc1 is not None:
                # v2 TLM enc is already in mm (cumulative since last ZERO)
                dL = enc1[0] - enc0[0]
                dR = enc1[1] - enc0[1]
                enc_mm = (dL + dR) / 2.0

            otos_mm: float | None = None
            if op0 is not None and op1 is not None:
                # otos_get_position() returns (x_mm, y_mm, h_cdeg) — already mm
                otos_mm = math.hypot(op1[0] - op0[0], op1[1] - op0[1])

            # dist2d_mm takes (x_cm, y_cm) tuples and returns mm
            cam_mm = dist2d_mm(c0, c1)

            print(f"  done={outcome}")
            if cam_mm is not None:
                print(f"  VISION (camera) actual : {cam_mm:.1f} mm")
            else:
                print("  VISION (camera) actual : (tag lost — no validation)")
            if enc_mm is not None:
                print(f"  ENCODERS think         : {enc_mm:.1f} mm")
            else:
                print("  ENCODERS think         : (no enc)")
            if otos_mm is not None:
                print(f"  OTOS thinks            : {otos_mm:.1f} mm")
            else:
                print("  OTOS thinks            : (no OTOS)")

            # ---- ground truth: the TAPE MEASURE is definitive --------------
            try:
                raw = input("  Tape-measure distance cm (DEFINITIVE) "
                            "[Enter = skip/uncalibrated, q = quit]: ").strip()
            except EOFError:
                break
            if raw.lower() in ("q", "quit", "exit"):
                break
            tape_mm = None
            if raw:
                try:
                    tape_mm = float(raw) * 10.0
                except ValueError:
                    print(f"  invalid number {raw!r}.")
            if tape_mm is None or tape_mm <= 0:
                print("  no tape measurement — trial recorded, NOT calibrated.")
                samples.append(dict(target=args.distance, enc=enc_mm, otos=otos_mm,
                                    cam=cam_mm, tape=None,
                                    enc_err=None, otos_err=None, cam_err=None))
                continue
            truth = tape_mm

            # ---- error of EVERY estimator vs the tape (incl. the camera) --
            enc_err = (enc_mm - truth) / truth * 100 if enc_mm else None
            otos_err = (otos_mm - truth) / truth * 100 if otos_mm else None
            cam_err = (cam_mm - truth) / truth * 100 if cam_mm else None

            # ---- closed-loop correction of the ONBOARD estimators ----------
            if enc_mm and enc_mm > 0:
                ml, mr = compute_encoder_correction(truth, enc_mm, ml, mr)
                proto.set_config(ml=ml, mr=mr)
            if otos_mm and otos_mm > 0:
                otos_scale = compute_otos_scale_correction(truth, otos_mm, otos_scale)
                otos_int8 = scale_to_int8(otos_scale)
                rb = proto.otos_set_linear_scalar(otos_int8)
                otos_scale = int8_to_scale(rb if rb is not None else otos_int8)

            samples.append(dict(target=args.distance, enc=enc_mm, otos=otos_mm,
                                cam=cam_mm, tape=tape_mm,
                                enc_err=enc_err, otos_err=otos_err, cam_err=cam_err))

            # ---- report this trial + running stats -------------------------
            parts = []
            if enc_err is not None:
                parts.append(f"encoders={enc_err:+.1f}%")
            if otos_err is not None:
                parts.append(f"OTOS={otos_err:+.1f}%")
            if cam_err is not None:
                parts.append(f"camera={cam_err:+.1f}%")
            print("  error vs tape:  " + "   ".join(parts))
            print(f"  UPDATED on robot →  ml={ml:.5f} mr={mr:.5f}  "
                  f"OTOS scale={otos_scale:.4f} (int8={otos_int8:+d})")
            print("  (camera is observed only — its scale is logged, not pushed)")
            _print_running_stats(samples)

    except KeyboardInterrupt:
        print("\n  interrupted.")
    finally:
        proto.stop()
        if laser_on:
            proto.port_write(laser_port, False)
        conn.disconnect()
        if cam is not None:
            cam.close()

    # No usable session if the robot never came up — skip summary/persist.
    if cam is None:
        return 2

    # ---- final summary + persist -------------------------------------------
    banner("CALIBRATION SUMMARY")
    _print_table(samples)
    _print_running_stats(samples, final=True)
    print(f"\nFinal calibration:")
    print(f"  encoders  ml={ml:.5f}  mr={mr:.5f}")
    print(f"  OTOS      scale={otos_scale:.4f}  (int8={otos_int8:+d})")

    # Camera distance scale: multiply a camera-measured distance by this to get
    # the true (tape) distance. Observed only — never pushed to the robot.
    cam_ratios = [s["tape"] / s["cam"] for s in samples
                  if s.get("tape") and s.get("cam")]
    cam_scale = None
    if cam_ratios:
        cam_scale = statistics.fmean(cam_ratios)
        cam_sd = statistics.stdev(cam_ratios) if len(cam_ratios) >= 2 else 0.0
        print(f"  CAMERA    distance scale={cam_scale:.4f} "
              f"(±{cam_sd:.4f}, n={len(cam_ratios)})  "
              f"→ camera reads {(1/cam_scale - 1)*100:+.1f}% vs tape")

    calibrated = [s for s in samples if s.get("tape")]
    if not calibrated:
        print("\nNo calibrated trials — nothing written.")
        return 0
    if args.no_write:
        print(f"\n--no-write: NOT writing {_TOVEZ_JSON.name}. Values above are "
              f"live on the robot until reboot.")
        return 0

    _save_calibration(ml, mr, otos_scale, cam_scale)
    print(f"\nWrote calibration to {_TOVEZ_JSON}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
