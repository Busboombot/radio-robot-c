#!/usr/bin/env python3
"""safe_cmd_bench.py — verify SAFE one-shot re-arm semantics on the bench robot.

Sprint 024-003 changed SAFE off from a permanent disable to a one-shot:

[1] SAFE off, no motion command → safety stays disabled (one-shot flag armed
    but not triggered).  A streaming S with no keepalives AFTER the re-arm
    fires should safety-stop after sTimeoutMs.

[2] SAFE off, then S (re-arms safety), keepalives cut → safety_stop fires
    ~500 ms later.  The EVT stream must include "EVT safety re-armed" before
    the motion finishes.

[3] SAFE on 600 → standard watchdog: keepalives cut → safety_stop ~600 ms.

Leaves the watchdog ON (safe default) at the end.
"""
import sys
import time

from robot_radio.io.serial_conn import SerialConnection

PORT = "/dev/cu.usbmodem2121102"


def saw_safety(c, window_s):
    t0 = time.time()
    while time.time() - t0 < window_s:
        for ln in c.read_lines(120):
            if "safety_stop" in ln:
                return (time.time() - t0) * 1000.0
    return None


def saw_rearm(c, window_s):
    """Return True if 'EVT safety re-armed' appears within window_s."""
    t0 = time.time()
    while time.time() - t0 < window_s:
        for ln in c.read_lines(120):
            if "safety re-armed" in ln:
                return True
    return False


def main():
    c = SerialConnection(port=PORT, mode="direct")
    c.connect()
    print("PING:", [x for x in c.send("PING")["responses"] if "pong" in x][:1])

    # [1] SAFE off, then motion → EVT safety re-armed emitted, safety restored.
    print("\n[1] SAFE off → start S → expect 'EVT safety re-armed'")
    print("    reply:", c.send("SAFE off")["responses"])
    c.send("S -45 45")           # triggers one-shot re-arm
    rearm_seen = saw_rearm(c, 0.5)
    print(f"    EVT safety re-armed seen = {rearm_seen}   (want True)")
    c.send("X")
    c.start_keepalive()
    time.sleep(0.3)

    # [2] SAFE off then S with keepalives cut → safety_stop fires (safety re-armed)
    print("\n[2] SAFE off → S → cut keepalive → robot must SAFETY-STOP (~500 ms)")
    print("    reply:", c.send("SAFE off")["responses"])
    c.send("S -45 45")           # one-shot re-arms safety
    c.stop_keepalive()
    ms2 = saw_safety(c, 2.0)
    print(f"    EVT safety_stop at {ms2:.0f} ms" if ms2 else
          "    NO safety_stop (FAIL — safety should have re-armed)")
    c.start_keepalive()
    c.send("X")
    time.sleep(0.3)

    # [3] SAFE on 600 — standard watchdog, keepalives cut → safety_stop ~600 ms
    print("\n[3] SAFE on 600 → cut keepalive → EVT safety_stop ~600 ms")
    print("    reply:", c.send("SAFE on 600")["responses"])
    c.send("S -45 45")
    c.stop_keepalive()
    ms3 = saw_safety(c, 2.0)
    print(f"    EVT safety_stop at {ms3:.0f} ms" if ms3 else "    NO safety_stop (FAIL)")
    c.start_keepalive()
    c.send("X")

    # restore safe default
    print("\n[restore]", c.send("SAFE on")["responses"])
    c.disconnect()

    ok = rearm_seen and (ms2 is not None) and (ms3 is not None)
    print("\nRESULT:", "PASS" if ok else "FAIL")
    print("  [1] re-arm EVT seen:", rearm_seen)
    print("  [2] safety_stop after re-arm:", ms2 is not None,
          f"({ms2:.0f} ms)" if ms2 else "")
    print("  [3] standard watchdog:", ms3 is not None,
          f"({ms3:.0f} ms)" if ms3 else "")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
