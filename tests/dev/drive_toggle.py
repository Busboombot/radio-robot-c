#!/usr/bin/env python3
"""drive_toggle.py — SPACE toggles drive/stop; watch the wheels + encoder.

Each SPACE press flips between DRIVE (sends S keepalives) and STOP. While
driving, it prints the live encoder delta so you can see whether it's counting
while you watch whether the wheels are physically turning. Ctrl-C / 'q' quits
(and stops the motors).

Usage:
  uv run python tests/bench/drive_toggle.py [PORT] [SPEED]
"""

import sys
import time
import re
import select
import termios
import tty
import serial

PORT  = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2121102"
SPEED = int(sys.argv[2]) if len(sys.argv) > 2 else 200
KEEPALIVE = 0.15

s = serial.Serial(PORT, 115200, timeout=0.01, dsrdtr=False, rtscts=False)
s.dtr = False
s.rts = False

def send(cmd):
    s.write((cmd + "\n").encode())
    s.flush()

# Liveness.
send("PING")
t = time.time()
while time.time() - t < 1.5:
    if "pong" in s.readline().decode(errors="replace"):
        break
s.reset_input_buffer()
send("STREAM 60")
time.sleep(0.1)

print("=" * 60)
print(f"  SPACE = toggle DRIVE/STOP at {SPEED} mm/s   q = quit")
print("  Watch the WHEELS while it drives; the encoder delta prints here.")
print("=" * 60)

# Raw terminal so we can read single keypresses without Enter.
fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
tty.setcbreak(fd)

driving = False
last_send = 0.0
enc0 = None
enc_last = None
last_print = 0.0

def key_ready():
    return select.select([sys.stdin], [], [], 0)[0]

try:
    while True:
        # Handle keypress.
        if key_ready():
            c = sys.stdin.read(1)
            if c == "q":
                break
            if c == " ":
                driving = not driving
                if driving:
                    enc0 = None
                    send(f"S {SPEED} {SPEED}")
                    last_send = time.monotonic()
                    sys.stdout.write("\r\n>>> DRIVE\r\n")
                else:
                    send("STOP")
                    d = (enc_last[0]-enc0[0], enc_last[1]-enc0[1]) if (enc0 and enc_last) else (0,0)
                    sys.stdout.write(f"\r\n>>> STOP   (this run moved {d})\r\n")
                sys.stdout.flush()

        # Read a line of telemetry.
        ln = s.readline().decode(errors="replace").strip()
        if ln:
            m = re.search(r"enc=([\-0-9]+),([\-0-9]+)", ln)
            if m:
                e = (int(m.group(1)), int(m.group(2)))
                if driving and enc0 is None:
                    enc0 = e
                enc_last = e
                now = time.monotonic()
                if driving and now - last_print > 0.25:
                    d = (e[0]-enc0[0], e[1]-enc0[1]) if enc0 else (0,0)
                    sys.stdout.write(f"\r  enc={e}  moved={d}    ")
                    sys.stdout.flush()
                    last_print = now

        # Keepalive while driving.
        if driving:
            now = time.monotonic()
            if now - last_send >= KEEPALIVE:
                send(f"S {SPEED} {SPEED}")
                last_send = now
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    for _ in range(3):
        send("STOP"); time.sleep(0.05)
    send("STREAM 0")
    time.sleep(0.2)
    s.close()
    print("\nstopped.")
