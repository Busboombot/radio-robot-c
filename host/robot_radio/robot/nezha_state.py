"""NezhaState — hardware state manager for the Nezha robot (protocol v2).

Owns the update cycle: sends S keepalive commands via NezhaProtocol, reads
TLM frames, maintains timestamped sensor state, and supports both synchronous
and async (daemon thread) operation.

Usage (manual):
    proto = NezhaProtocol(conn)
    state = NezhaState(proto)
    state.update(left_mms=100, right_mms=100)
    print(state.encoders, state.otos_pose)

Usage (async):
    state.start_async()
    state.wheel_speeds = [100, 100]
    time.sleep(1.0)
    state.stop_async()
"""

from __future__ import annotations

import math
import threading
import time

from robot_radio.robot.protocol import NezhaProtocol, parse_tlm


class NezhaState:
    """Hardware state manager for the Nezha robot (protocol v2).

    Maintains the most recent TLM sensor values.  All public state attributes
    are protected by ``_lock``.  Callers needing a consistent snapshot should
    copy under the lock:

        with state._lock:
            enc = state.encoders
            pose = state.otos_pose

    ``update()`` is the unit of work: sends current wheel speeds and reads
    available TLM lines for ~40 ms.  Call it manually in a tight control loop,
    or use ``start_async()`` / ``stop_async()`` to run it on a daemon thread at
    ~25 Hz.
    """

    def __init__(self, proto: NezhaProtocol) -> None:
        self._proto = proto
        self._lock = threading.Lock()
        self._wheel_speeds: list[int] = [0, 0]
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        # Sensor state — always access under _lock
        self.encoders: tuple[int, int] = (0, 0)
        # otos_pose stores (x_mm, y_mm, h_cdeg) — centi-degrees for heading
        self.otos_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
        # heading_rad: CCW-positive heading in radians, derived from TLM pose cdeg
        self.heading_rad: float = 0.0
        self.line_sensor: tuple[int, int, int, int] = (255, 255, 255, 255)
        self.color: tuple[int, int, int, int] = (0, 0, 0, 0)
        self.last_tlm_t: int | None = None   # robot clock of last TLM frame (ms)
        self.last_update_s: float = 0.0
        self.dt_s: float = 0.0

    # ------------------------------------------------------------------
    # Wheel speed property
    # ------------------------------------------------------------------

    @property
    def wheel_speeds(self) -> list[int]:
        """Current commanded wheel speeds [left_mms, right_mms]."""
        with self._lock:
            return list(self._wheel_speeds)

    @wheel_speeds.setter
    def wheel_speeds(self, speeds: list[int]) -> None:
        with self._lock:
            self._wheel_speeds[0] = speeds[0]
            self._wheel_speeds[1] = speeds[1]

    # ------------------------------------------------------------------
    # Update cycle
    # ------------------------------------------------------------------

    def update(self, left_mms: int | None = None, right_mms: int | None = None) -> None:
        """Send current wheel speeds and read available TLM lines for 40 ms.

        If ``left_mms`` or ``right_mms`` are provided they are stored as the
        new commanded wheel speeds before sending.
        """
        if left_mms is not None or right_mms is not None:
            with self._lock:
                if left_mms is not None:
                    self._wheel_speeds[0] = left_mms
                if right_mms is not None:
                    self._wheel_speeds[1] = right_mms

        with self._lock:
            l, r = self._wheel_speeds[0], self._wheel_speeds[1]

        self._proto.drive(l, r)

        now = time.monotonic()
        for line in self._proto.read_lines(duration_ms=40):
            self._process_line(line)

        with self._lock:
            self.dt_s = now - self.last_update_s if self.last_update_s > 0 else 0.0
            self.last_update_s = now

    def _process_line(self, line: str) -> None:
        """Parse a single serial line and update matching state attributes."""
        tlm = parse_tlm(line)
        if tlm is not None:
            with self._lock:
                if tlm.enc is not None:
                    self.encoders = tlm.enc
                if tlm.pose is not None:
                    x_mm, y_mm, h_cdeg = tlm.pose
                    self.otos_pose = (float(x_mm), float(y_mm), float(h_cdeg))
                    # Convert centidegrees to radians: cdeg / 18000.0 * math.pi
                    self.heading_rad = h_cdeg / 18000.0 * math.pi
                if tlm.line is not None:
                    self.line_sensor = tlm.line
                if tlm.color is not None:
                    self.color = tlm.color
                if tlm.t is not None:
                    self.last_tlm_t = tlm.t

    # ------------------------------------------------------------------
    # One-shot commands
    # ------------------------------------------------------------------

    def zero_otos(self) -> None:
        """Zero the OTOS position at the current location (ZERO pose command)."""
        self._proto.zero_otos()
        with self._lock:
            x, y, h = self.otos_pose
            self.otos_pose = (0.0, 0.0, h)

    def zero_encoders(self) -> None:
        """Zero all encoder counters (ZERO enc command)."""
        self._proto.zero_encoders()
        with self._lock:
            self.encoders = (0, 0)

    def stop(self) -> None:
        """Stop motors immediately (STOP command)."""
        with self._lock:
            self._wheel_speeds = [0, 0]
        self._proto.stop()

    def set_world_pose(self, x_mm: float, y_mm: float, h_deg: float) -> None:
        """Set OTOS world-frame pose (OV command). Heading in degrees."""
        xi, yi = int(round(x_mm)), int(round(y_mm))
        # Firmware OV expects centi-degrees for heading
        h_cdeg = int(round(h_deg * 100))
        self._proto.otos_set_position(xi, yi, h_cdeg)
        with self._lock:
            self.otos_pose = (float(xi), float(yi), float(h_cdeg))
            self.heading_rad = h_cdeg / 18000.0 * math.pi

    def enable_stream(self, period_ms: int = 40) -> None:
        """Enable TLM streaming at the given period (STREAM <ms>)."""
        self._proto.stream(period_ms)

    def disable_stream(self) -> None:
        """Disable TLM streaming (STREAM 0)."""
        self._proto.stream(0)

    # ------------------------------------------------------------------
    # Async operation
    # ------------------------------------------------------------------

    def start_async(self, interval_s: float = 0.040) -> None:
        """Start a daemon thread calling ``update()`` in a loop at ~25 Hz."""
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            args=(interval_s,),
            daemon=True,
            name="NezhaState-async",
        )
        self._thread.start()

    def stop_async(self) -> None:
        """Signal the async thread to exit and wait for it to join (up to 2 s)."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self, interval_s: float) -> None:
        sleep_s = max(0.0, interval_s - 0.040)
        while not self._stop_event.is_set():
            try:
                self.update()
            except Exception:
                pass
            if sleep_s > 0:
                time.sleep(sleep_s)
