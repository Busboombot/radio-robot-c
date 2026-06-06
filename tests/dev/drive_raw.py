#!/usr/bin/env python3
"""drive_raw.py — drive the wheels with raw serial commands for a fixed time.

Connects, drives both wheels at SPEED for SECS seconds (re-sending the S
keepalive), prints every line, then STOPS automatically and reports whether
the encoders ever counted.

Usage:
  uv run python tests/bench/drive_raw.py [PORT] [SPEED] [SECS]
  e.g.  uv run python tests/bench/drive_raw.py /dev/cu.usbmodem2121102 200 5
"""

import sys
import time
import re
import serial

PORT  = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2121102"
SPEED = int(sys.argv[2]) if len(sys.argv) > 2 else 200
SECS  = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
KEEPALIVE = 0.15

print(f"port={PORT} speed={SPEED} secs={SECS}")

s = serial.Serial(PORT, 115200, timeout=0.02, dsrdtr=False, rtscts=False)
s.dtr = False
s.rts = False

def send(cmd):
    print(f">>> {cmd}")
    s.write((cmd + "\n").encode()); s.flush()

# liveness check (robot may already be running, or just booted)
s.write(b"PING\n"); s.flush()
alive = False
t = time.time()
while time.time() - t < 1.5:
    ln = s.readline().decode(errors="replace").strip()
    if ln:
        print(f"    {ln}")
    if "pong" in ln:
        alive = True
        break
if not alive:
    print("waiting for boot...")
    t = time.time()
    while time.time() - t < 12:
        ln = s.readline().decode(errors="replace").strip()
        if ln:
            print(f"    {ln}")
        if "DEVICE:" in ln or "pong" in ln:
            alive = True
            break
if not alive:
    print("ERROR: no robot response"); s.close(); sys.exit(1)

send("SET sTimeout=2000")
time.sleep(0.1)
send("STREAM 40")
time.sleep(0.1)
send(f"S {SPEED} {SPEED}")

# Drive for SECS seconds, tracking encoder values.
enc_max = 0
enc_any_nonzero = False
last_send = time.time()
t0 = time.time()
while time.time() - t0 < SECS:
    ln = s.readline().decode(errors="replace").strip()
    if ln:
        print(f"    {ln}")
        m = re.search(r"enc=([\-0-9]+),([\-0-9]+)", ln)
        if m:
            el, er = abs(int(m.group(1))), abs(int(m.group(2)))
            enc_max = max(enc_max, el, er)
            if el != 0 or er != 0:
                enc_any_nonzero = True
    now = time.time()
    if now - last_send >= KEEPALIVE:
        send(f"S {SPEED} {SPEED}")
        last_send = now

# Stop.
for _ in range(3):
    send("STOP"); time.sleep(0.05)
send("STREAM 0")
time.sleep(0.2)
s.close()

print()
print("=" * 50)
print(f"RESULT: encoders {'COUNTED' if enc_any_nonzero else 'STAYED ZERO'}"
      f"  (max |enc| = {enc_max})")
print("=" * 50)
