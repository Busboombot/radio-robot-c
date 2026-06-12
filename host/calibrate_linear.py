"""Calibrate OTOS linear scale. All logic in robot_radio.calibration.linear."""
from __future__ import annotations
import argparse, sys
from pathlib import Path
_HOST_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(_HOST_ROOT))
from robot_radio.calibration.linear import DRIVE_SPEED_MMS, DEFAULT_DISTANCE_CM, calibrate_distance
from robot_radio.calibration.helpers import resolve_save_path
from robot_radio.calibration._conn_helpers import make_serial_conn

def main() -> None:
    p = argparse.ArgumentParser(
        description="Calibrate OTOS linear scale. "
                    "Usage: uv run python calibrate_linear.py [--distance CM] [--speed MMS]")
    p.add_argument("--distance", type=float, default=DEFAULT_DISTANCE_CM)
    p.add_argument("--speed", type=int, default=DRIVE_SPEED_MMS)
    p.add_argument("--port", default=None)
    p.add_argument("--direct", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()
    ser = make_serial_conn(args.port, args.direct)
    try:
        calibrate_distance(ser, resolve_save_path(),
                           target_cm=args.distance, speed_mms=args.speed,
                           dry_run=args.dry_run)
    finally:
        try: ser.close()
        except Exception: pass

if __name__ == "__main__":
    main()
