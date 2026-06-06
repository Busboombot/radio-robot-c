#!/usr/bin/env python3
"""wedge_repro.py -- deterministic, headless encoder-wedge reproduction harness.

Drives the robot through N drive->stop->drive cycles and classifies each cycle
as WEDGED (commanded to move but encoder delta ~= 0) or CLEAN (encoder moved).
Produces a final wedge rate for use as a numeric diagnostic signal.

Stop-mode controls how each drive phase ends:

  --stop-mode clean
      Sends an explicit STOP command (3x with 50 ms gaps), then waits 200 ms
      before restarting. Tests the nominal stop path used by application code.

  --stop-mode watchdog
      Ceases the S keepalive for sTimeout_ms + 500 ms so the firmware
      S-watchdog fires safety_stop internally. Tests theory T2: whether the
      firmware watchdog path leaves internal state (e.g. I2C mutex) in a wedge
      condition that a clean STOP does not.

Wedge detection: after each stop->restart, streams encoder lines for a 1.5 s
observation window. A cycle is WEDGED if both enc=L,R deltas remain < 5 mm
over the entire window (absolute value). A cycle is CLEAN if either encoder
accumulates >= 5 mm of movement.

Usage:
  uv run python tests/bench/wedge_repro.py --help
  uv run python tests/bench/wedge_repro.py --stop-mode watchdog --cycles 20
  uv run python tests/bench/wedge_repro.py --stop-mode clean --cycles 10 --speed 200
"""

import argparse
import re
import sys
import time

import serial

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

BAUD          = 115200
READ_TIMEOUT  = 0.02   # seconds -- non-blocking readline
KEEPALIVE_S   = 0.15   # S command interval while driving
STIMEOUT_MS   = 2000   # firmware S-watchdog timeout (ms)
STREAM_PERIOD = 40     # STREAM period ms  (25 Hz)

WEDGE_THRESHOLD_MM = 5  # |delta| below this on both wheels => wedge
OBSERVE_WINDOW_S   = 1.5
DRIVE_SECS         = 1.5  # drive duration per cycle

DEFAULT_PORT   = "/dev/cu.usbmodem2121102"
DEFAULT_SPEED  = 200
DEFAULT_CYCLES = 50


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Headless encoder-wedge reproduction and detection harness.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Stop modes:\n"
            "  clean     -- explicit STOP command (3x) then 200 ms settle\n"
            "  watchdog  -- let S keepalive lapse; firmware watchdog fires stop\n"
        ),
    )
    p.add_argument("--port", default=DEFAULT_PORT,
                   help=f"Serial port (default {DEFAULT_PORT})")
    p.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                   help=f"Wheel speed mm/s passed to 'S V V' (default {DEFAULT_SPEED})")
    p.add_argument("--cycles", type=int, default=DEFAULT_CYCLES,
                   help=f"Number of drive->stop->drive cycles (default {DEFAULT_CYCLES})")
    p.add_argument("--secs", type=float, default=DRIVE_SECS,
                   help=f"Drive phase duration per cycle in seconds (default {DRIVE_SECS})")
    p.add_argument("--stop-mode", choices=["clean", "watchdog"], default="clean",
                   help="How to stop between cycles: clean (explicit STOP) "
                        "or watchdog (keepalive lapse, firmware fires stop). "
                        "Default: clean")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------

def connect(port: str) -> serial.Serial:
    """Open serial port without DTR reset; PING for liveness.

    Raises SystemExit with a clear message if the robot does not respond.
    """
    s = serial.Serial(port, BAUD, timeout=READ_TIMEOUT, dsrdtr=False, rtscts=False)
    s.dtr = False
    s.rts = False

    # PING -- robot may already be running, or may need a moment to boot.
    s.write(b"PING\n")
    s.flush()
    alive = False
    t = time.time()
    while time.time() - t < 1.5:
        ln = s.readline().decode(errors="replace").strip()
        if "pong" in ln.lower():
            alive = True
            break

    if not alive:
        # Give a boot window.
        print("waiting for robot boot (up to 12 s)...")
        s.write(b"PING\n")
        s.flush()
        t = time.time()
        while time.time() - t < 12.0:
            ln = s.readline().decode(errors="replace").strip()
            if "DEVICE:" in ln or "pong" in ln.lower():
                alive = True
                break

    if not alive:
        s.close()
        print("ERROR: no robot response -- check port and power.", file=sys.stderr)
        sys.exit(1)

    return s


def send(s: serial.Serial, cmd: str) -> None:
    """Send a newline-terminated command and flush."""
    s.write((cmd + "\n").encode())
    s.flush()


def setup_stream(s: serial.Serial) -> None:
    """Configure firmware watchdog and start telemetry stream."""
    send(s, f"SET sTimeout={STIMEOUT_MS}")
    time.sleep(0.1)
    send(s, f"STREAM {STREAM_PERIOD}")
    time.sleep(0.1)


# ---------------------------------------------------------------------------
# Drive phases
# ---------------------------------------------------------------------------

def drive_phase(s: serial.Serial, speed: int, duration_s: float) -> None:
    """Drive both wheels at `speed` for `duration_s` seconds with keepalives."""
    send(s, f"S {speed} {speed}")
    last_send = time.time()
    t0 = time.time()
    while time.time() - t0 < duration_s:
        # Drain telemetry so the serial buffer stays clear.
        s.readline()
        now = time.time()
        if now - last_send >= KEEPALIVE_S:
            send(s, f"S {speed} {speed}")
            last_send = now


def stop_clean(s: serial.Serial) -> None:
    """Stop motors with 3 explicit STOP commands, then settle 200 ms."""
    for _ in range(3):
        send(s, "STOP")
        time.sleep(0.05)
    time.sleep(0.2)


def stop_watchdog(s: serial.Serial) -> None:
    """Stop motors by letting the S keepalive lapse until firmware watchdog fires.

    Waits for sTimeout_ms + 500 ms to guarantee the firmware S-watchdog fires
    safety_stop, then resumes (no explicit STOP sent).
    """
    lapse_s = (STIMEOUT_MS + 500) / 1000.0
    t0 = time.time()
    while time.time() - t0 < lapse_s:
        # Keep draining the serial buffer; just don't send S.
        s.readline()


# ---------------------------------------------------------------------------
# Wedge detection
# ---------------------------------------------------------------------------

_ENC_RE = re.compile(r"enc=([\-0-9]+),([\-0-9]+)")


def observe_encoders(s: serial.Serial, window_s: float) -> tuple[int, int]:
    """Collect enc=L,R lines for `window_s` seconds; return (L_delta, R_delta).

    Returns the absolute delta across the observation window (last - first).
    If no encoder lines are seen, returns (0, 0).
    """
    first_l: int | None = None
    first_r: int | None = None
    last_l:  int | None = None
    last_r:  int | None = None

    t0 = time.time()
    while time.time() - t0 < window_s:
        ln = s.readline().decode(errors="replace").strip()
        if not ln:
            continue
        m = _ENC_RE.search(ln)
        if m:
            el, er = int(m.group(1)), int(m.group(2))
            if first_l is None:
                first_l, first_r = el, er
            last_l, last_r = el, er

    if first_l is None:
        return (0, 0)

    return (abs(last_l - first_l), abs(last_r - first_r))


def classify_wedge(l_delta: int, r_delta: int) -> bool:
    """Return True (WEDGED) if both deltas are below the wedge threshold."""
    return l_delta < WEDGE_THRESHOLD_MM and r_delta < WEDGE_THRESHOLD_MM


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()

    print(f"port={args.port}  speed={args.speed}  cycles={args.cycles}"
          f"  secs/cycle={args.secs}  stop-mode={args.stop_mode}")
    print(f"wedge threshold: |delta| < {WEDGE_THRESHOLD_MM} mm on both wheels")
    print()

    s = connect(args.port)

    try:
        setup_stream(s)

        wedge_count = 0
        total = args.cycles

        for cycle in range(1, total + 1):
            # Drive phase.
            drive_phase(s, args.speed, args.secs)

            # Stop phase.
            if args.stop_mode == "clean":
                stop_clean(s)
            else:
                stop_watchdog(s)

            # Restart drive and observe encoders to detect wedge.
            send(s, f"S {args.speed} {args.speed}")
            l_delta, r_delta = observe_encoders(s, OBSERVE_WINDOW_S)

            wedged = classify_wedge(l_delta, r_delta)
            if wedged:
                wedge_count += 1
                label = "WEDGED"
            else:
                label = "OK"

            print(f"  cycle {cycle:3d}/{total}  {label:6s}  "
                  f"enc_delta=L:{l_delta:4d} R:{r_delta:4d}")

        print()
        print(f"RESULT: {wedge_count}/{total} wedged  mode={args.stop_mode}")

    finally:
        # Always stop motors and end stream -- even on Ctrl-C or exception.
        try:
            for _ in range(3):
                send(s, "STOP")
                time.sleep(0.05)
            send(s, "STREAM 0")
            time.sleep(0.2)
        except Exception:
            pass
        s.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
