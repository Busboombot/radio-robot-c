#!/usr/bin/env python3
"""run_wedge.py — drive the in-firmware DBG WEDGE harness and stream its output.

Talks raw serial to the robot's DIRECT USB port (the harness prints via
uBit.serial, not the radio relay). Sends `DBG WEDGE <rate>`, prints every line,
and stops when the harness reports a wedge or ends (or on --secs timeout).

    uv run python tests/dev/run_wedge.py [--rate 50] [--write-ms 40] [--bus 400]
                                         [--dither 3] [--secs 120]

The harness now MIRRORS the production motor path (PID-like per-tick dithered
writes, rate-limited like setSpeed, 400 kHz bus) so it reproduces the real
wedge. Sweep --write-ms / --bus / --dither to find a pattern that does NOT wedge.
"""
import argparse, sys, time
import serial


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/cu.usbmodem2121102")
    p.add_argument("--secs", type=float, default=120.0)
    p.add_argument("--rate", type=int, default=50, help="loop/read rate in Hz")
    p.add_argument("--write-ms", type=int, default=40,
                   help="min ms between motor writes (setSpeed-style; 0 = every tick)")
    p.add_argument("--bus", type=int, default=400, help="I2C bus speed in kHz (prod=400)")
    p.add_argument("--dither", type=int, default=3,
                   help="per-tick pwm dither +/- units (forces a write every tick)")
    p.add_argument("--reg", type=int, default=46, choices=(46, 47),
                   help="encoder read register: 46=angle/pos (firmware default), 47=speed")
    p.add_argument("--sensors", type=int, default=0, choices=(0, 1),
                   help="1 = also hammer OTOS/colour/line on the shared bus (prod load)")
    p.add_argument("--real", type=int, default=0, choices=(0, 1),
                   help="1 = drive via the REAL production PID/Motor path (phases are mm/s)")
    args = p.parse_args()

    cmd = (f"DBG WEDGE {args.rate} {args.write_ms} {args.bus} "
           f"{args.dither} {args.reg} {args.sensors} {args.real}")
    s = serial.Serial(args.port, 115200, timeout=0.2)
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(f"{cmd}\r\n".encode())
    print(f"-> {cmd} on {args.port}\n", flush=True)

    t0 = time.time()
    buf = b""
    try:
        while time.time() - t0 < args.secs:
            data = s.read(256)
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                txt = line.decode("ascii", "replace").rstrip("\r")
                if txt:
                    print(txt, flush=True)
                if "WEDGE-" in txt or "WEDGETEST end" in txt:
                    print(f"\n[done after {time.time()-t0:.1f}s]")
                    s.close()
                    return 0
        print(f"\n[timeout {args.secs:.0f}s — sending stop byte]")
        s.write(b"x\r\n")
        time.sleep(0.5)
        print(s.read(400).decode("ascii", "replace"))
    except KeyboardInterrupt:
        s.write(b"x\r\n")
        time.sleep(0.5)
        print(s.read(400).decode("ascii", "replace"))
    finally:
        s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
