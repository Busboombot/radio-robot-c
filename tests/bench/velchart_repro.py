#!/usr/bin/env python3
"""velchart_repro.py — headless reproduction of the velocity_chart freeze.

velocity_chart.py runs a worker thread that drives via NezhaProtocol.stream_drive
while matplotlib renders in the main thread. stream_drive only re-sends the S
keepalive *between* yields; a yield suspends the generator until the consumer
asks for the next item. When the main thread holds the GIL for a CPU-bound
render, the worker can't resume the generator, the keepalive slips, and the
firmware S-watchdog eventually fires safety_stop — the chart freezes.

This reproduces that headlessly, with NO matplotlib, by simulating the GUI's
GIL-holding render as a CPU-busy burst in the main thread. `time.sleep()` would
NOT reproduce it (sleep releases the GIL); a busy-spin does.

It replicates stream_drive inline so it can LOG the keepalive gaps and the exact
stop reason (safety_stop vs exception vs timeout).

Usage:
  uv run python tests/bench/velchart_repro.py [--gui-load-ms N] [--secs S] ...
  e.g.  --gui-load-ms 0     (light main loop — control)
        --gui-load-ms 400   (simulate 400 ms GIL-holding renders)
"""

import sys
import time
import threading
import queue
import argparse


def parse_args():
    p = argparse.ArgumentParser(description="Headless velocity_chart freeze repro")
    p.add_argument("--port", default="/dev/cu.usbmodem2121102")
    p.add_argument("--speed", type=int, default=200)
    p.add_argument("--secs", type=float, default=30.0, help="max run seconds")
    p.add_argument("--gui-load-ms", type=float, default=0.0,
                   help="CPU-busy burst per main-loop frame (ms) — simulates "
                        "matplotlib holding the GIL. 0 = light loop.")
    p.add_argument("--frame-ms", type=float, default=33.0, help="main loop frame period")
    p.add_argument("--stimeout", type=int, default=10000, help="firmware SET sTimeout")
    p.add_argument("--watchdog-ms", type=int, default=1000,
                   help="stream_drive keepalive window (host sends S every 30%% of this)")
    p.add_argument("--period-ms", type=int, default=100, help="TLM stream period")
    return p.parse_args()


args = parse_args()

from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol, parse_response, parse_tlm
from robot_radio.robot.nezha import Nezha

stop_event = threading.Event()
data_q: "queue.Queue" = queue.Queue()
result: dict = {"reason": "?", "lines": 0, "runtime": 0.0, "max_ka_gap": 0.0}


def worker(conn):
    """Replicates NezhaProtocol.stream_drive with keepalive-gap instrumentation."""
    speeds = [args.speed, args.speed]
    keepalive_s = args.watchdog_ms * 0.30 / 1000.0
    conn.send_fast(f"S {speeds[0]} {speeds[1]}")
    last_send = time.monotonic()
    t0 = time.monotonic()
    lines = 0
    sent_s = 0
    ack_s = 0
    max_ka_gap = 0.0
    reason = "timeout"

    def resend(last):
        nonlocal max_ka_gap, sent_s
        now = time.monotonic()
        gap = now - last
        if gap >= keepalive_s:
            conn.send_fast(f"S {speeds[0]} {speeds[1]}")
            sent_s += 1
            if gap > max_ka_gap:
                max_ka_gap = gap
            return now
        return last

    try:
        # initial S to enter streaming drive
        conn.send_fast(f"S {speeds[0]} {speeds[1]}"); sent_s += 1
        while not stop_event.is_set():
            for raw in conn.read_lines(duration_ms=50):
                r = parse_response(raw)
                if r is None:
                    continue
                if r.tag == "EVT" and r.tokens and r.tokens[0] == "safety_stop":
                    reason = "safety_stop"
                    raise StopIteration
                if r.tag == "OK" and r.tokens and r.tokens[0] == "drive":
                    ack_s += 1                       # firmware acked our keepalive S
                if r.tag == "TLM":
                    lines += 1
                    tlm = parse_tlm(r.raw)
                    if tlm and tlm.vel is not None:
                        data_q.put((time.monotonic(), tlm.vel[0], tlm.vel[1]))
                last_send = resend(last_send)
            last_send = resend(last_send)
        reason = "stop_event"
    except StopIteration:
        pass
    except Exception as exc:
        reason = f"exception: {exc!r}"

    result.update(reason=reason, lines=lines, sent_s=sent_s, ack_s=ack_s,
                  runtime=time.monotonic() - t0, max_ka_gap=max_ka_gap)


def main():
    print(f"port={args.port} speed={args.speed} gui_load={args.gui_load_ms}ms "
          f"frame={args.frame_ms}ms sTimeout={args.stimeout} watchdog={args.watchdog_ms}")

    conn = SerialConnection(port=args.port, mode="direct")
    conn.connect(skip_ping=True)
    proto = NezhaProtocol(conn)
    nezha = Nezha(proto)

    print("connecting...")
    dl = time.monotonic() + 12
    ok = False
    while time.monotonic() < dl:
        try:
            nezha.connect(); ok = True; break
        except Exception:
            time.sleep(0.4)
    if not ok:
        print("ERROR: no robot"); conn.disconnect(); return 1

    proto.send(f"SET sTimeout={args.stimeout}", 300)
    gi = proto.send("GET sTimeout", 300)
    print("sTimeout now:", gi.get("responses"))
    proto.stream(args.period_ms)
    conn._ser.reset_input_buffer()   # drop any stale EVT safety_stop from a prior run

    th = threading.Thread(target=worker, args=(conn,), daemon=True)
    th.start()

    # Main loop: drain the queue (like _update) then optionally hold the GIL with
    # a CPU-busy burst (simulating a matplotlib render).
    t0 = time.monotonic()
    last_data = time.monotonic()
    max_data_gap = 0.0
    drained = 0
    busy = args.gui_load_ms / 1000.0
    while time.monotonic() - t0 < args.secs and th.is_alive():
        try:
            while True:
                data_q.get_nowait(); drained += 1
                g = time.monotonic() - last_data
                last_data = time.monotonic()
                if g > max_data_gap:
                    max_data_gap = g
        except queue.Empty:
            pass
        if busy > 0:
            spin = time.monotonic() + busy
            while time.monotonic() < spin:
                pass  # GIL-holding busy wait (matplotlib render stand-in)
        time.sleep(args.frame_ms / 1000.0)

    stop_event.set()
    th.join(timeout=3)

    for _ in range(3):
        conn.send_fast("STOP"); time.sleep(0.05)
    proto.stream(0)
    time.sleep(0.2)

    # Read firmware bus stats.
    conn._ser.reset_input_buffer()
    conn.send_fast("DBG I2C")
    i2c = "(none)"
    t = time.time()
    while time.time() - t < 1.5:
        ln = conn._ser.readline().decode(errors="replace").strip()
        if ln.startswith("I2C "):
            i2c = ln; break
    conn.disconnect()

    print()
    print("=" * 60)
    print(f"stop reason     : {result['reason']}")
    print(f"streamed        : {result['runtime']:.1f}s, {result['lines']} TLM lines")
    print(f"keepalive S sent: {result.get('sent_s')}   firmware OK-drive acks: {result.get('ack_s')}")
    print(f"drained (main)  : {drained}")
    print(f"max keepalive gap: {result['max_ka_gap']:.2f}s  (firmware watchdog "
          f"= {args.stimeout/1000:.1f}s)")
    print(f"max data gap    : {max_data_gap:.2f}s")
    print(f"DBG I2C         : {i2c}")
    print("=" * 60)
    if result["reason"] == "safety_stop":
        print(">>> REPRODUCED: watchdog fired — keepalive was starved by GIL load.")
    elif result["runtime"] >= args.secs - 1:
        print(">>> ran full duration without stopping.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
