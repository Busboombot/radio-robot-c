#!/usr/bin/env python3
"""i2c_diag.py — does the encoder I2C read fail DURING a drive?

Bench-safe: spins in place (left back, right fwd). Streams TLM and watches the
live encoder counts climb, and brackets the drive with `DBG I2C` error counters.

  - encoders climb + I2C errors flat  -> reads are fine; "freeze" is only the
    idle one-shot read path.
  - encoders stall + I2C errors climb  -> reads are genuinely failing on the bus
    during the drive (the real wedge behind the 400° runaway).

  uv run python tests/dev/i2c_diag.py [--speed 60] [--secs 4]
"""
import argparse
import sys
import time


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None)
    p.add_argument("--speed", type=int, default=60)
    p.add_argument("--secs", type=float, default=4.0)
    args = p.parse_args()

    from robot_radio.io.cli import _make_robot
    from robot_radio.robot.nezha import Nezha
    from robot_radio.robot.protocol import parse_tlm

    class _A:
        port = args.port
        verbose = False

    robot, conn, _ = _make_robot(_A())
    if not isinstance(robot, Nezha):
        print("ERROR: need a Nezha")
        return 2
    proto = robot._proto

    def dbg_i2c() -> str:
        r = proto.send("DBG I2C", 400)
        for line in r.get("responses", []):
            if line.strip().startswith("I2C "):
                return line.strip()
        return "(no I2C line)"

    try:
        proto.send("STOP", 200)
        proto.send("STREAM 0", 200)
        proto.send("SET sTimeout=10000", 300)
        print(f"  one-shot ENC (idle): {proto.send('ENC', 300).get('responses', [])}")
        proto.send("DBG I2C RESET", 300)
        print(f"  I2C before: {dbg_i2c()}")
        proto.zero_encoders()
        proto.stream(50)

        print(f"\n  spinning in place {args.speed}mm/s for {args.secs:.0f}s "
              f"(streaming reads):")
        enc = (0, 0)
        moved = False
        t0 = time.monotonic()
        last_send = last_print = 0.0
        while time.monotonic() - t0 < args.secs:
            now = time.monotonic()
            if now - last_send >= 0.15:
                proto.drive(-args.speed, args.speed)
                last_send = now
            for line in proto.read_lines(duration_ms=25):
                if "EVT safety_stop" in line:
                    proto.drive(-args.speed, args.speed)
                tlm = parse_tlm(line)
                if tlm is not None and tlm.enc is not None:
                    enc = tlm.enc
                    if abs(enc[0]) >= 5 or abs(enc[1]) >= 5:
                        moved = True
            if now - last_print >= 0.3:
                last_print = now
                print(f"    t={now - t0:4.1f}s  enc=L{enc[0]:>6} R{enc[1]:>6} mm")

        for _ in range(4):
            proto.stop()
            time.sleep(0.05)
        proto.stream(0)

        print(f"\n  one-shot ENC (idle, post-spin): "
              f"{proto.send('ENC', 300).get('responses', [])}")
        print(f"  I2C after:  {dbg_i2c()}")
        print("\n========================================")
        print(f"  streaming encoders moved: {'YES ✓' if moved else 'NO ✗'}")
        print("  -> compare I2C before/after: error counts flat = bus is fine;")
        print("     climbing = reads failing during the drive (real wedge).")
        print("========================================")
    finally:
        try:
            for _ in range(3):
                proto.stop()
                time.sleep(0.04)
            proto.stream(0)
            conn.disconnect()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
