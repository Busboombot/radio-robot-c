#!/usr/bin/env python3
"""sensors_chart.py — real-time sensor strip charts bench tool.

Streams live telemetry from the robot and renders three updating matplotlib
strip charts (no driving — the robot sits still; run the carriage to see the
sensors vary):
  - Color sensor: all four channels (R, G, B, C)
  - Line sensor:  all four channels (1..4)
  - OTOS optical-flow odometer: X and Y position (mm)

SPACE — connect/start streaming or disconnect. Each press opens a fresh serial
connection (resilient to robot reboots). The wheels are never commanded.

Usage:
    uv run python tests/bench/sensors_chart.py [--port DEV] [--window S]
"""

import argparse
import collections
import queue
import sys
import threading
import time


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Real-time robot sensor charts")
    p.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    p.add_argument("--window", type=float, default=8.0,
                   help="Rolling window seconds (default 8)")
    p.add_argument("--period", type=int, default=50,
                   help="TLM stream period ms (default 50 = 20 Hz)")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Streaming worker — STREAM only, never drives. One instance per SPACE-start.
# ---------------------------------------------------------------------------

def _stream_worker(
    port: str,
    period_ms: int,
    data_queue: "queue.Queue",
    stop_event: threading.Event,
    status_queue: "queue.Queue[str]",
) -> None:
    """Open a fresh connection, enable TLM streaming, push sensor tuples.

    Pushes (t, (r,g,b,c), (l1,l2,l3,l4), (x,y)) per TLM frame. Does NOT send any
    drive command — the motors stay off; only the carriage moves the sensors.
    """
    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol, parse_response, parse_tlm
    from robot_radio.robot.nezha import Nezha

    conn = None
    try:
        conn = SerialConnection(port=port, mode="direct")
        conn.connect(skip_ping=True)
        proto = NezhaProtocol(conn)
        nezha = Nezha(proto)

        status_queue.put("CONNECTING")
        deadline = time.monotonic() + 12.0
        identity = None
        while time.monotonic() < deadline and not stop_event.is_set():
            try:
                identity = nezha.connect()
                break
            except Exception:
                time.sleep(0.4)
        if identity is None or stop_event.is_set():
            status_queue.put("FAILED")
            return

        # Flush any stale buffered lines, then stream.
        try:
            conn.send_fast("STREAM 0")
            time.sleep(0.05)
            conn._ser.reset_input_buffer()
        except Exception:
            pass
        proto.stream(period_ms)
        status_queue.put(f"RUNNING — {identity.get('name', '?')}")

        while not stop_event.is_set():
            for raw_line in conn.read_lines(duration_ms=50):
                r = parse_response(raw_line)
                if r is None or r.tag != "TLM":
                    continue
                tlm = parse_tlm(r.raw)
                if tlm is None:
                    continue
                col  = tlm.color if tlm.color is not None else (0, 0, 0, 0)
                ln   = tlm.line  if tlm.line  is not None else (0, 0, 0, 0)
                pose = tlm.pose  if tlm.pose  is not None else (0, 0, 0)
                data_queue.put((time.monotonic(), col, ln, (pose[0], pose[1])))

    except OSError as exc:
        if exc.errno == 6:
            status_queue.put("DISCONNECTED — power cycle detected, press SPACE to reconnect")
        else:
            status_queue.put(f"ERROR: {exc}")
    except Exception as exc:
        status_queue.put(f"ERROR: {exc}")
    finally:
        status_queue.put("STOPPED")
        if conn is not None:
            try:
                from robot_radio.robot.protocol import NezhaProtocol
                NezhaProtocol(conn).stream(0)
            except Exception:
                pass
            try:
                conn.disconnect()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Main — matplotlib window + spacebar connection control
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse_args()

    if args.port is None:
        from robot_radio.io.serial_conn import list_serial_ports
        ports = list_serial_ports()
        if not ports:
            print("ERROR: no USB modem serial ports found.")
            return 2
        port = ports[0]
    else:
        port = args.port
    print(f"  port: {port}   window: {args.window} s   period: {args.period} ms")
    print("  Press SPACE in the plot window to connect/disconnect. Run the carriage.")

    import matplotlib
    import platform
    if platform.system() == "Darwin":
        matplotlib.use("MacOSX")
    else:
        matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import numpy as np

    plt.style.use("dark_background")

    window_s = args.window
    maxlen   = int(window_s * 60)

    times_buf: "collections.deque[float]" = collections.deque(maxlen=maxlen)
    # 4 color, 4 line, 2 OTOS channels
    col_buf = [collections.deque(maxlen=maxlen) for _ in range(4)]
    lin_buf = [collections.deque(maxlen=maxlen) for _ in range(4)]
    otos_buf = [collections.deque(maxlen=maxlen) for _ in range(2)]

    data_queue:   "queue.Queue" = queue.Queue()
    status_queue: "queue.Queue[str]" = queue.Queue()
    worker_state = {"thread": None, "stop": None}

    # ---- figure: 3 stacked strip charts ----
    fig, (axc, axl, axo) = plt.subplots(3, 1, figsize=(11, 9))
    title_text = fig.suptitle("Robot sensors  [SPACE = connect]  (run the carriage)",
                              color="white", fontsize=12)
    fig.subplots_adjust(hspace=0.5, top=0.92)

    for ax in (axc, axl, axo):
        ax.set_xlim(0, window_s)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.grid(True, alpha=0.3)

    axc.set_title("Color sensor (R,G,B,C)", fontsize=10)
    axc.set_ylabel("counts", fontsize=8)
    col_colors = ["red", "lime", "deepskyblue", "white"]
    col_labels = ["R", "G", "B", "C"]
    col_lines = [axc.plot([], [], color=c, linewidth=1.1, label=lab)[0]
                 for c, lab in zip(col_colors, col_labels)]
    axc.legend(fontsize=7, loc="upper right", ncol=4)

    axl.set_title("Line sensor (4 channels)", fontsize=10)
    axl.set_ylabel("counts", fontsize=8)
    lin_colors = ["orange", "cyan", "magenta", "yellow"]
    lin_lines = [axl.plot([], [], color=c, linewidth=1.1, label=f"L{i+1}")[0]
                 for i, c in enumerate(lin_colors)]
    axl.legend(fontsize=7, loc="upper right", ncol=4)

    axo.set_title("OTOS optical-flow position (mm)", fontsize=10)
    axo.set_ylabel("mm", fontsize=8)
    otos_lines = [axo.plot([], [], color="deepskyblue", linewidth=1.2, label="X")[0],
                  axo.plot([], [], color="tomato",      linewidth=1.2, label="Y")[0]]
    axo.legend(fontsize=7, loc="upper right", ncol=2)

    all_groups = [(axc, col_lines, col_buf),
                  (axl, lin_lines, lin_buf),
                  (axo, otos_lines, otos_buf)]

    # ---- spacebar handler ----
    def _on_key(event):
        if event.key != " ":
            return
        th = worker_state["thread"]
        if th is not None and th.is_alive():
            worker_state["stop"].set()
            title_text.set_text("Robot sensors  [SPACE = connect]  (run the carriage)")
        else:
            stop_ev = threading.Event()
            worker_state["stop"]   = stop_ev
            worker_state["thread"] = threading.Thread(
                target=_stream_worker,
                args=(port, args.period, data_queue, stop_ev, status_queue),
                daemon=True,
            )
            worker_state["thread"].start()
            title_text.set_text("Robot sensors  [connecting…]")
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("key_press_event", _on_key)

    # ---- update loop ----
    def _update():
        while not status_queue.empty():
            msg = status_queue.get_nowait()
            if msg.startswith("RUNNING"):
                title_text.set_text(f"Robot sensors  ▶ {msg}  [SPACE = disconnect]")
            elif msg.startswith("DISCONNECTED"):
                title_text.set_text(f"Robot sensors  ⚡ {msg}")
            elif msg.startswith("ERROR"):
                title_text.set_text(f"Robot sensors  ⚠ {msg}  [SPACE = retry]")

        try:
            while True:
                t, col, ln, pose = data_queue.get_nowait()
                times_buf.append(t)
                for i in range(4):
                    col_buf[i].append(col[i])
                    lin_buf[i].append(ln[i])
                otos_buf[0].append(pose[0])
                otos_buf[1].append(pose[1])
        except queue.Empty:
            pass

        if not times_buf:
            return

        t_arr = np.array(times_buf)
        now   = t_arr[-1]
        rel   = np.clip(t_arr - (now - window_s), 0, window_s)

        for ax, lines, bufs in all_groups:
            for ln_artist, buf in zip(lines, bufs):
                ln_artist.set_data(rel, np.array(buf))
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)   # auto y, fixed x window

    plt.ion()
    plt.show(block=False)

    try:
        while plt.fignum_exists(fig.number):
            _update()
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(0.033)
    except KeyboardInterrupt:
        pass
    finally:
        if worker_state["stop"] is not None:
            worker_state["stop"].set()
        th = worker_state["thread"]
        if th is not None:
            th.join(timeout=2.0)
        plt.close("all")

    return 0


if __name__ == "__main__":
    sys.exit(main())
