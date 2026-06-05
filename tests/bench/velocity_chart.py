#!/usr/bin/env python3
"""velocity_chart.py — real-time velocity strip charts + phase plot bench tool.

Streams live telemetry from the robot and renders three updating matplotlib
plots in a window:
  - Left wheel velocity (mm/s) strip chart
  - Right wheel velocity (mm/s) strip chart
  - Phase plot: vR vs vL with reference line and current-point dot

Usage:
    uv run python tests/bench/velocity_chart.py [--port DEV] [--speed MMPS] [--window S]

Options:
    --port PORT     Serial port (auto-detect if omitted)
    --speed MMPS    Wheel speed mm/s for both wheels (default: 200)
    --window S      Rolling window seconds (default: 8.0)
"""

import argparse
import collections
import queue
import sys
import threading
import time

# ---------------------------------------------------------------------------
# CLI — parse before any hardware or matplotlib imports
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Real-time robot velocity charts")
    p.add_argument("--port", default=None, help="Serial port (auto-detect if omitted)")
    p.add_argument("--speed", type=int, default=200,
                   help="Wheel speed mm/s (default 200)")
    p.add_argument("--window", type=float, default=8.0,
                   help="Rolling window seconds (default 8)")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Telemetry streaming thread
# ---------------------------------------------------------------------------

def _stream_worker(
    port: str,
    speed: int,
    data_queue: "queue.Queue[tuple[float, int, int]]",
    stop_event: threading.Event,
    proto_holder: list,  # [proto] written after connect so main can send STOP
) -> None:
    """Daemon thread: connect, stream_drive, push (t, vL, vR) tuples into queue."""
    from robot_radio.io.serial_conn import SerialConnection
    from robot_radio.robot.protocol import NezhaProtocol, parse_tlm
    from robot_radio.robot.nezha import Nezha, RobotNotFoundError

    conn = SerialConnection(port=port)
    conn.connect()

    proto = NezhaProtocol(conn)
    nezha = Nezha(proto)
    proto_holder.append(proto)

    try:
        identity = nezha.connect()
        print(f"  robot: ALIVE — {identity}")
    except RobotNotFoundError as exc:
        print(f"\n  FATAL: {exc}")
        stop_event.set()
        return

    speeds = [speed, speed]
    try:
        for resp in nezha.stream_drive(speeds, period_ms=40, watchdog_ms=500):
            if stop_event.is_set():
                break
            if resp.tag == "TLM":
                tlm = parse_tlm(resp.raw)
                if tlm and tlm.vel is not None:
                    vL, vR = tlm.vel
                    data_queue.put((time.monotonic(), vL, vR))
    except Exception as exc:
        print(f"\n  stream error: {exc}", file=sys.stderr)
    finally:
        try:
            proto.stop()
            proto.stream(0)
        except Exception:
            pass
        try:
            conn.disconnect()
        except Exception:
            pass
        stop_event.set()


# ---------------------------------------------------------------------------
# Main — matplotlib animation
# ---------------------------------------------------------------------------

def main() -> int:
    args = _parse_args()

    # Resolve port.
    if args.port is None:
        from robot_radio.io.serial_conn import list_serial_ports
        ports = list_serial_ports()
        if not ports:
            print("  ERROR: no USB modem serial ports found.")
            return 2
        port = ports[0]
    else:
        port = args.port
    print(f"  port: {port}")
    print(f"  speed: {args.speed} mm/s   window: {args.window} s")

    import matplotlib
    matplotlib.use("TkAgg")          # works headless-safe on macOS + Linux
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import numpy as np

    plt.style.use("dark_background")

    window_s = args.window
    maxlen = int(window_s * 50)      # 50 samples/s headroom
    cmd_speed = args.speed

    # Rolling buffers: absolute monotonic timestamps + velocities.
    times_buf:  "collections.deque[float]" = collections.deque(maxlen=maxlen)
    vL_buf:     "collections.deque[int]"   = collections.deque(maxlen=maxlen)
    vR_buf:     "collections.deque[int]"   = collections.deque(maxlen=maxlen)

    data_queue: "queue.Queue[tuple[float, int, int]]" = queue.Queue()
    stop_event = threading.Event()
    proto_holder: list = []

    worker = threading.Thread(
        target=_stream_worker,
        args=(port, args.speed, data_queue, stop_event, proto_holder),
        daemon=True,
    )
    worker.start()

    # ------------------------------------------------------------------
    # Figure layout: 3 stacked panels, portrait
    # ------------------------------------------------------------------
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(7, 9))
    fig.suptitle("Robot wheel velocity", color="white", fontsize=12)
    fig.tight_layout(pad=2.5)

    # Strip chart common styling
    for ax, label in ((ax1, "Left wheel velocity (mm/s)"),
                      (ax2, "Right wheel velocity (mm/s)")):
        ax.set_title(label, fontsize=10)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylabel("mm/s", fontsize=8)
        ax.set_xlim(0, window_s)
        ax.set_ylim(-350, 350)
        ax.grid(True, alpha=0.3)

    # Command speed reference lines (dashed)
    ax1.axhline(cmd_speed, color="yellow", linestyle="--", linewidth=1.0,
                alpha=0.7, label=f"cmd={cmd_speed}")
    ax2.axhline(cmd_speed, color="yellow", linestyle="--", linewidth=1.0,
                alpha=0.7, label=f"cmd={cmd_speed}")
    ax1.legend(fontsize=7, loc="upper right")
    ax2.legend(fontsize=7, loc="upper right")

    # Velocity line artists
    (line_vL,) = ax1.plot([], [], color="deepskyblue", linewidth=1.2)
    (line_vR,) = ax2.plot([], [], color="tomato", linewidth=1.2)

    # Phase plot (ax3)
    ax3.set_title("Phase plot: vR vs vL (mm/s)", fontsize=10)
    ax3.set_xlabel("vL (mm/s)", fontsize=8)
    ax3.set_ylabel("vR (mm/s)", fontsize=8)
    ax3.set_xlim(-350, 350)
    ax3.set_ylim(-350, 350)
    ax3.set_aspect("equal")
    ax3.grid(True, alpha=0.3)

    # Reference line: slope = cmd_vR / cmd_vL = 1.0 for equal speeds
    ref_x = np.array([-350, 350])
    ref_slope = 1.0  # cmd_vR / cmd_vL
    ax3.plot(ref_x, ref_slope * ref_x, color="dodgerblue", linestyle="--",
             linewidth=1.0, alpha=0.8, label="vR=vL (reference)")
    ax3.legend(fontsize=7, loc="upper left")

    # Phase trace and current-point artists
    (phase_trace,) = ax3.plot([], [], color="grey", linewidth=0.8, alpha=0.6)
    (phase_dot,)   = ax3.plot([], [], "o", color="red", markersize=8)

    # ------------------------------------------------------------------
    # Animation update
    # ------------------------------------------------------------------
    def _update(_frame):
        # Drain queue into rolling buffers.
        try:
            while True:
                t, vl, vr = data_queue.get_nowait()
                times_buf.append(t)
                vL_buf.append(vl)
                vR_buf.append(vr)
        except queue.Empty:
            pass

        if not times_buf:
            return line_vL, line_vR, phase_trace, phase_dot

        # Build relative time axis (0 = oldest in window, window_s = now).
        t_arr = np.array(times_buf)
        vl_arr = np.array(vL_buf)
        vr_arr = np.array(vR_buf)

        now = t_arr[-1]
        rel = t_arr - (now - window_s)  # oldest anchor at 0
        rel = np.clip(rel, 0, window_s)

        line_vL.set_data(rel, vl_arr)
        line_vR.set_data(rel, vr_arr)

        # Phase plot
        phase_trace.set_data(vl_arr, vr_arr)
        phase_dot.set_data([vl_arr[-1]], [vr_arr[-1]])

        return line_vL, line_vR, phase_trace, phase_dot

    anim = animation.FuncAnimation(
        fig, _update,
        interval=33,
        blit=True,
        cache_frame_data=False,
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        # Attempt graceful motor stop via proto if available.
        if proto_holder:
            try:
                proto_holder[0].stream(0)
                proto_holder[0].stop()
            except Exception:
                pass
        worker.join(timeout=2.0)
        plt.close("all")

    return 0


if __name__ == "__main__":
    sys.exit(main())
