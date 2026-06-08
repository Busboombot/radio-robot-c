#!/usr/bin/env python3
"""laser_test.py — toggle each digital J-port so you can SEE which one lights the
laser. Direct USB (the relay drops/garbles; use the robot's own port).

Sends `P <port> 1` (on) for a few seconds per port, then `P <port> 0` (off).
Digital port map (firmware PortIO): J1→P8, J2→P12, J3→P14, J4→P16.

Usage: uv run python tests/dev/laser_test.py --port /dev/cu.usbmodem2121102
"""
import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbmodem2121102")
    ap.add_argument("--secs", type=float, default=3.0)
    ap.add_argument("--ports", default="4,1,2,3")
    args = ap.parse_args()

    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol

    conn = SerialConnection(port=args.port, mode="direct")
    conn.connect(skip_ping=True)
    proto = NezhaProtocol(conn)
    time.sleep(1.0)
    r = proto.send("VER", 400)
    print("  alive:", [l for l in r.get("responses", []) if "fw=" in l][:1])

    def p(port, val):
        resp = conn.send(f"P {port} {val}", read_ms=300)
        line = next((l for l in resp.get("responses", []) if "port" in l or "OK" in l), "")
        return line.strip()

    try:
        for port in [int(x) for x in args.ports.split(",")]:
            print(f"\n  >>> J{port} ON  — watch for the laser  ({p(port, 1)})")
            time.sleep(args.secs)
            print(f"      J{port} OFF                          ({p(port, 0)})")
            time.sleep(0.6)
    finally:
        for port in [int(x) for x in args.ports.split(",")]:
            conn.send(f"P {port} 0", read_ms=120)
        conn.disconnect()
    print("\n  done — tell me which Jn lit the laser.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
