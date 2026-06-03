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
    cd /Volumes/Proj/proj/RobotProjects/radio-robot-c
    uv run python tests/calibrate/calibrate_linear.py
Options: --distance MM  --speed MMPS  --port DEV  --field "W H"  --no-write
"""

import argparse
import math
import statistics
import sys
import time

from calib_common import (
    Cam, Relay, ROBOT_CONFIG, OTOS_MM_PER_LSB, LASER_PORT,
    load_config, save_updates, scale_to_int8, int8_to_scale, dist2d,
)

DEFAULT_DISTANCE_MM = 900     # 90 cm
DEFAULT_SPEED_MMPS = 200


def banner(msg):
    print("\n" + "=" * 64 + f"\n  {msg}\n" + "=" * 64)


def predict_end(pose, distance_mm):
    """Predicted end (x,y) mm if the robot drives `distance_mm` along its yaw."""
    x, y, yaw = pose
    return (x + distance_mm * math.cos(yaw), y + distance_mm * math.sin(yaw))


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

    # ---- current calibration from config -----------------------------------
    cfg = load_config()
    cal = cfg["calibration"]
    ml = float(cal["mm_per_wheel_deg_left"])
    mr = float(cal["mm_per_wheel_deg_right"])
    otos_scale = float(cal["otos_linear_scale"])
    otos_int8 = scale_to_int8(otos_scale)
    laser_port = (cfg.get("peripherals") or {}).get("laser_port") or LASER_PORT
    print(f"Loaded calibration from {ROBOT_CONFIG.name}:")
    print(f"  encoders  ml={ml:.5f}  mr={mr:.5f}  mm/deg")
    print(f"  OTOS      scale={otos_scale:.4f}  (int8={otos_int8:+d})")

    # ---- connect -----------------------------------------------------------
    # Open the relay and PROVE THE ROBOT IS ALIVE before touching the camera or
    # moving anything. The relay being present says nothing about the robot.
    relay = Relay(args.port) if args.port else Relay()
    cam = None
    laser_on = False
    samples = []   # dicts per trial

    try:
        print("\nConnecting…")
        print("  relay:", relay.configure_go())
        if not relay.preflight():
            return 2
        relay.stop()

        # Robot confirmed alive — now bring up the overhead camera.
        cam = Cam()

        # Push current calibration to the robot and init the OTOS once.
        relay.set_encoder_cal(ml, mr)
        rb = relay.set_otos_int8(otos_int8)
        relay.otos_init()
        print(f"  pushed: ml={ml:.5f} mr={mr:.5f}  OL={otos_int8:+d} "
              f"(readback {rb:+d})" if rb is not None else "")

        # Laser on for floor marking.
        relay.set_port(laser_port, True)
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

            # ---- safety: where are we, where will we end? ----
            c0 = cam.pose()
            if c0 is None:
                print("  ⚠ camera can't see tag 100 — reposition into view. Skipping.")
                continue
            end = predict_end(c0, args.distance)
            print(f"  start (cam): x={c0[0]:.0f} y={c0[1]:.0f} mm  "
                  f"heading={math.degrees(c0[2]):.0f}°  → predicted end "
                  f"x={end[0]:.0f} y={end[1]:.0f} mm")
            if field and not (0 <= end[0] <= field[0] and 0 <= end[1] <= field[1]):
                print(f"  ⛔ predicted end is outside the field {field} — "
                      f"reposition the robot. Not driving.")
                continue

            # ---- baseline reads ----
            relay.otos_zero()
            relay.zero_enc_pose()
            time.sleep(0.3)
            s0 = relay.snap()
            op0 = relay.otos_raw() or (0, 0, 0)

            # ---- drive (one deliberate blocking command) ----
            print(f"  driving {args.distance} mm…")
            done = relay.drive_distance(args.speed, args.distance)
            relay.stop()
            time.sleep(0.4)

            # ---- after reads ----
            s1 = relay.snap()
            op1 = relay.otos_raw() or op0
            c1 = cam.pose()

            # ---- distances ----
            enc_mm = otos_mm = None
            if "enc" in s0 and "enc" in s1:
                dL = (s1["enc"][0] - s0["enc"][0]) * ml
                dR = (s1["enc"][1] - s0["enc"][1]) * mr
                enc_mm = (dL + dR) / 2.0
            otos_mm = math.hypot(op1[0] - op0[0], op1[1] - op0[1]) * OTOS_MM_PER_LSB
            cam_mm = dist2d(c0, c1)

            print(f"  done={done}")
            print(f"  VISION (camera) actual : "
                  f"{cam_mm:.1f} mm" if cam_mm is not None else
                  "  VISION (camera) actual : (tag lost — no validation)")
            print(f"  ENCODERS think         : "
                  f"{enc_mm:.1f} mm" if enc_mm is not None else
                  "  ENCODERS think         : (no enc)")
            print(f"  OTOS thinks            : {otos_mm:.1f} mm")

            # ---- ground truth: the TAPE MEASURE is definitive ----
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

            # ---- error of EVERY estimator vs the tape (incl. the camera) ----
            enc_err = (enc_mm - truth) / truth * 100 if enc_mm else None
            otos_err = (otos_mm - truth) / truth * 100 if otos_mm else None
            cam_err = (cam_mm - truth) / truth * 100 if cam_mm else None

            # ---- closed-loop correction of the ONBOARD estimators ----
            if enc_mm and enc_mm > 0:
                k = truth / enc_mm
                ml, mr = ml * k, mr * k
                relay.set_encoder_cal(ml, mr)
            if otos_mm and otos_mm > 0:
                otos_scale *= truth / otos_mm
                otos_int8 = scale_to_int8(otos_scale)
                rb = relay.set_otos_int8(otos_int8)
                otos_scale = int8_to_scale(rb if rb is not None else otos_int8)

            samples.append(dict(target=args.distance, enc=enc_mm, otos=otos_mm,
                                cam=cam_mm, tape=tape_mm,
                                enc_err=enc_err, otos_err=otos_err, cam_err=cam_err))

            # ---- report this trial + running stats ----
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
        relay.stop()
        if laser_on:
            relay.set_port(laser_port, False)
        relay.close()
        if cam is not None:
            cam.close()

    # No usable session if the robot never came up — skip summary/persist.
    if cam is None:
        return 2

    # ---- final summary + persist ------------------------------------------
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
        print(f"\n--no-write: NOT writing {ROBOT_CONFIG.name}. Values above are "
              f"live on the robot until reboot.")
        return 0
    vision = {"camera_distance_scale": round(cam_scale, 4)} if cam_scale else None
    save_updates(
        calibration={
            "mm_per_wheel_deg_left": round(ml, 5),
            "mm_per_wheel_deg_right": round(mr, 5),
            "otos_linear_scale": round(otos_scale, 4),
        },
        vision=vision,
    )
    print(f"\nWrote calibration to {ROBOT_CONFIG}.")
    return 0


def _errs(samples, key):
    return [s[key] for s in samples if s.get(key) is not None]


def _print_running_stats(samples, final=False):
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


def _print_table(samples):
    if not samples:
        print("  (no trials)")
        return
    print(f"  {'#':>2}  {'target':>6}  {'tape':>6}  {'cam':>7}  {'camE%':>6}  "
          f"{'enc':>7}  {'encE%':>6}  {'otos':>7}  {'otosE%':>6}")
    for i, s in enumerate(samples, 1):
        def f(v, w=7, p=1):
            return f"{v:>{w}.{p}f}" if isinstance(v, (int, float)) else f"{'—':>{w}}"
        print(f"  {i:>2}  {f(s['target'],6,0)}  {f(s['tape'],6,0)}  {f(s['cam'])}  "
              f"{f(s['cam_err'],6)}  {f(s['enc'])}  {f(s['enc_err'],6)}  "
              f"{f(s['otos'])}  {f(s['otos_err'],6)}")


if __name__ == "__main__":
    sys.exit(main())
