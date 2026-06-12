"""SimConnection — SerialConnection-compatible sim backend.

Drop-in replacement for SerialConnection that drives libfirmware_host
(MockHAL + Robot + CommandProcessor) via ctypes instead of a serial port.

Usage::

    from robot_radio.io.sim_conn import SimConnection
    from robot_radio.robot.protocol import NezhaProtocol

    conn = SimConnection()
    conn.connect()
    proto = NezhaProtocol(conn)
    proto.timed(200, 200, 2500)
    proto.wait_for_evt_done("T", timeout_ms=5000)
    df = conn.state_df()   # pandas DataFrame of time-series state

The sim backend advances wall-clock time explicitly: every call to
read_lines() (and wait_for_evt_done(), which calls read_lines() in a
loop) advances the simulation by the requested duration in small steps,
collecting async EVTs and recording per-step state (velocities, poses,
encoder positions).  This makes time-series analysis trivially easy.

Thread safety: not thread-safe.  Use from one thread.
"""

from __future__ import annotations

import ctypes
import pathlib
import sys
from typing import Any

_LIB_NAME = "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"
_HERE = pathlib.Path(__file__).parent
# Resolve the dylib relative to this file: host/robot_radio/io/ -> ../../.. = repo root
_DEFAULT_LIB = (_HERE / "../../../host_tests/build" / _LIB_NAME).resolve()

# Default tick step: 24 ms matches the conftest fixture and is fine for the
# 25 ms control period.  Smaller = smoother state log, more CPU.
_DEFAULT_TICK_MS = 24


class SimConnection:
    """SerialConnection-compatible backend using libfirmware_host.

    Supports the same interface as SerialConnection so NezhaProtocol
    works unchanged:
      - connect() / disconnect()
      - send(message, read_ms, stop_token) -> dict
      - send_fast(message) -> None
      - read_lines(duration_ms, stop_token) -> list[str]
      - is_open property
      - mode property

    Extra sim-only interface:
      - tick(ms) -> list[str]
          Advance sim time, return EVT lines, record state.
      - state_log  -> list[dict]
          All state snapshots recorded during ticking.
      - state_df() -> pd.DataFrame
          state_log as a DataFrame (requires pandas).
      - clear_state_log()
          Reset the log.
    """

    def __init__(self, lib_path: str | pathlib.Path | None = None,
                 tick_step_ms: int = _DEFAULT_TICK_MS) -> None:
        self._lib_path = pathlib.Path(lib_path) if lib_path else _DEFAULT_LIB
        self._tick_step_ms = tick_step_ms
        self._lib: ctypes.CDLL | None = None
        self._h: ctypes.c_void_p | None = None
        self._t: int = 0
        self._state_log: list[dict[str, float]] = []

    # ------------------------------------------------------------------
    # SerialConnection-compatible interface
    # ------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._h is not None

    @property
    def mode(self) -> str | None:
        return "sim" if self.is_open else None

    def connect(self, skip_ping: bool = False, **_: Any) -> dict[str, Any]:
        """Load the shared library, create a SimHandle, optionally PING.

        Parameters match SerialConnection.connect() so callers are portable.
        """
        if self.is_open:
            return {"status": "already_connected", "mode": "sim"}

        if not self._lib_path.exists():
            return {
                "error": f"Sim library not found at {self._lib_path}. "
                         f"Run: cmake -S host_tests -B host_tests/build && "
                         f"cmake --build host_tests/build",
                "lib_path": str(self._lib_path),
            }

        self._lib = ctypes.CDLL(str(self._lib_path))
        self._setup_types()
        self._h = self._lib.sim_create()
        self._t = 0
        self._state_log = []

        if not self._h:
            self._lib = None
            return {"error": "sim_create() returned NULL"}

        if not skip_ping:
            resp = self.send("PING", read_ms=200)
            if not any("pong" in l for l in resp.get("responses", [])):
                return {"error": "PING failed — sim may not have initialised"}

        return {"status": "connected", "mode": "sim",
                "lib": str(self._lib_path)}

    def disconnect(self) -> dict[str, Any]:
        if not self.is_open:
            return {"status": "not_connected"}
        self._lib.sim_destroy(self._h)
        self._h = None
        self._lib = None
        return {"status": "disconnected", "ticks": self._t}

    def send(self, message: str,
             read_ms: int = 500,
             stop_token: str | None = "OK") -> dict[str, Any]:
        """Send a command; advance sim for read_ms collecting EVTs.

        Mirrors SerialConnection.send() return shape:
            {"sent": ..., "mode": "sim", "responses": [line, ...]}

        For non-blocking commands (PING, GET, SET, T, D, TURN …) the
        firmware replies OK immediately.  The stop_token="OK" default
        means _advance() exits as soon as OK is in the response —
        before ticking — so no sim time is consumed for fast commands.
        For long-running reads (EVT-wait loops via read_lines()) callers
        pass stop_token explicitly.
        """
        if not self.is_open:
            return {"error": "Not connected. Call connect() first."}

        sync = self._raw_command(message)
        lines: list[str] = [l for l in sync.strip().split("\n") if l.strip()] if sync else []

        # Collect additional EVTs by advancing time; stop early on stop_token.
        evts = self._advance(read_ms, stop_token, existing_lines=lines)
        lines.extend(evts)

        return {"sent": message, "mode": "sim", "responses": lines}

    def send_fast(self, message: str) -> None:
        """Fire-and-forget: dispatch the command, consume no sim time."""
        if not self.is_open:
            raise ConnectionError("Not connected. Call connect() first.")
        self._raw_command(message)

    def read_lines(self, duration_ms: int = 500,
                   stop_token: str | None = None) -> list[str]:
        """Tick the sim for duration_ms, collecting and returning EVT lines.

        This is the hook that NezhaProtocol.wait_for_evt_done() calls
        repeatedly (with duration_ms=100) to poll for EVT done T/D/TURN.
        Each 100 ms block advances sim time and records state into state_log.
        """
        if not self.is_open:
            return []
        return self._advance(duration_ms, stop_token)

    def read_pending_lines(self) -> list[str]:
        """Non-blocking drain — always empty in sim (no buffered input concept).

        The sim has no equivalent of the serial TLM/EVT queues: all sim output
        is produced synchronously by _raw_command() or _advance().  Callers
        that expect a non-blocking poll (e.g. NezhaProtocol.read_pending_lines)
        get an empty list, which is correct — there is nothing waiting.
        """
        return []

    # ------------------------------------------------------------------
    # Sim-only interface
    # ------------------------------------------------------------------

    def tick(self, ms: int) -> list[str]:
        """Advance sim time by ms milliseconds; return EVT lines emitted."""
        if not self.is_open:
            return []
        return self._advance(ms, stop_token=None)

    # ------------------------------------------------------------------
    # Sim state injection helpers (no firmware command needed)
    # ------------------------------------------------------------------

    def set_motor_offset(self, side: int, factor: float) -> None:
        """Scale one (or both) wheels' effective speed.

        side: 0=left, 1=right, 2=both.
        factor: 1.0 = nominal; 0.8 = 80% efficiency.
        """
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_motor_offset(self._h, ctypes.c_int(side),
                                       ctypes.c_float(factor))

    def set_enc(self, l_mm: float, r_mm: float) -> None:
        """Directly inject encoder positions (zeroes physics history)."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_enc_l(self._h, ctypes.c_float(l_mm))
        self._lib.sim_set_enc_r(self._h, ctypes.c_float(r_mm))

    def set_otos_pose(self, x_mm: float, y_mm: float, h_rad: float) -> None:
        """Inject an OTOS pose reading for the next otosCorrect() call."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_otos_pose(self._h, ctypes.c_float(x_mm),
                                    ctypes.c_float(y_mm), ctypes.c_float(h_rad))

    def get_exact_pose(self) -> dict:
        """Return oracle ground truth pose from ExactPoseTracker.

        Returns {"x": mm, "y": mm, "h": rad}.
        """
        if not self.is_open:
            raise ConnectionError("Not connected")
        lib, h = self._lib, self._h
        return {"x": float(lib.sim_get_exact_pose_x(h)),
                "y": float(lib.sim_get_exact_pose_y(h)),
                "h": float(lib.sim_get_exact_pose_h(h))}

    def set_slip(self, straight: float = 0.005, turn_extra: float = 0.03) -> None:
        """Apply slip ratio to both wheels.

        straight: fractional slip on straight motion (e.g. 0.005 = 0.5%).
        turn_extra: additional slip fraction during turns.
        """
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_motor_slip(self._h, ctypes.c_int(2),
                                     ctypes.c_float(straight), ctypes.c_float(turn_extra))

    def set_encoder_noise(self, sigma_mm: float = 0.05) -> None:
        """Apply Gaussian encoder noise to both wheels.

        sigma_mm: standard deviation of per-tick encoder noise in mm.
        """
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_encoder_noise(self._h, ctypes.c_int(2), ctypes.c_float(sigma_mm))

    def enable_otos_model(self) -> None:
        """Enable the OTOS simulation model (integrates true velocities with noise)."""
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_enable_otos_model(self._h)

    def enable_otos_fusion(self, on: bool = True) -> None:
        """Run the firmware's OTOS EKF correction (Robot::otosCorrect) in sim_tick.

        When enabled, the firmware ``pose`` is the fused EKF estimate (encoder
        predict + OTOS update). When disabled (default), ``pose`` is encoder-only
        dead reckoning.
        """
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_otos_fusion(self._h, ctypes.c_int(1 if on else 0))

    def set_otos_noise(self, linear: float = 0.01, yaw: float = 0.025) -> None:
        """Set OTOS noise fractions.

        linear: fractional standard deviation for linear velocity noise.
        yaw: fractional standard deviation for yaw velocity noise.
        """
        if not self.is_open:
            raise ConnectionError("Not connected")
        self._lib.sim_set_otos_linear_noise(self._h, ctypes.c_float(linear))
        self._lib.sim_set_otos_yaw_noise(self._h, ctypes.c_float(yaw))

    def get_otos_pose(self) -> dict:
        """Return accumulated OTOS odometry pose (noisy, sim model must be enabled).

        Returns {"x": mm, "y": mm, "h": rad}.
        """
        if not self.is_open:
            raise ConnectionError("Not connected")
        lib, h = self._lib, self._h
        return {"x": float(lib.sim_get_otos_x(h)),
                "y": float(lib.sim_get_otos_y(h)),
                "h": float(lib.sim_get_otos_h(h))}

    def clear_state_log(self) -> None:
        """Clear the accumulated state log."""
        self._state_log.clear()

    @property
    def state_log(self) -> list[dict[str, float]]:
        """Time-series state recorded during ticking.

        Each entry: {time_ms, vel_l, vel_r, enc_l, enc_r, pose_x, pose_y, pose_h}
        pose_h is in radians (raw from dead-reckoning odometry).
        """
        return self._state_log

    def state_df(self):
        """Return state_log as a pandas DataFrame.

        Requires pandas to be installed.  Columns:
            time_ms, vel_l, vel_r, enc_l, enc_r, pose_x, pose_y, pose_h
        """
        import pandas as pd  # local import — not a hard dependency
        return pd.DataFrame(self._state_log)

    def get_state(self) -> dict[str, float]:
        """Return a single current-state snapshot (does not record to log)."""
        if not self.is_open:
            return {}
        return self._snapshot()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _raw_command(self, line: str) -> str:
        """Send one command to the C sim; return the synchronous reply."""
        buf = ctypes.create_string_buffer(512)
        n = self._lib.sim_command(self._h, line.encode(), buf, 512)
        return buf.raw[:n].decode(errors="replace") if n > 0 else ""

    def _get_evts(self) -> str:
        """Drain async EVT buffer from the C sim."""
        buf = ctypes.create_string_buffer(2048)
        n = self._lib.sim_get_async_evts(self._h, buf, 2048)
        return buf.raw[:n].decode(errors="replace") if n > 0 else ""

    def _advance(self, total_ms: int, stop_token: str | None = None,
                 existing_lines: list[str] | None = None) -> list[str]:
        """Tick the sim for total_ms ms, recording state each step.

        Returns EVT lines accumulated during the advance.  If stop_token
        is set, returns as soon as a line containing stop_token is seen.
        existing_lines is checked first; if stop_token already satisfied,
        returns immediately without ticking (fast path for OK commands).
        """
        lines: list[str] = []

        # Fast path: stop_token already satisfied by the sync reply.
        if stop_token and existing_lines and any(stop_token in l for l in existing_lines):
            return lines

        step = self._tick_step_ms
        end_t = self._t + total_ms

        while self._t < end_t:
            dt = min(step, end_t - self._t)
            self._lib.sim_tick(self._h, ctypes.c_uint32(self._t))
            self._t += dt
            self._state_log.append(self._snapshot())

            evts = self._get_evts()
            if evts:
                for ln in evts.strip().split("\n"):
                    ln = ln.strip()
                    if ln:
                        lines.append(ln)
                if stop_token and any(stop_token in l for l in lines):
                    break

        return lines

    def _snapshot(self) -> dict[str, float]:
        """Read all sim state getters into a single dict."""
        lib, h = self._lib, self._h
        return {
            "time_ms":      float(self._t),
            "vel_l":        float(lib.sim_get_vel_l(h)),
            "vel_r":        float(lib.sim_get_vel_r(h)),
            "enc_l":        float(lib.sim_get_enc_l(h)),
            "enc_r":        float(lib.sim_get_enc_r(h)),
            "pose_x":       float(lib.sim_get_pose_x(h)),
            "pose_y":       float(lib.sim_get_pose_y(h)),
            "pose_h":       float(lib.sim_get_pose_h(h)),
            "exact_pose_x": float(lib.sim_get_exact_pose_x(h)),
            "exact_pose_y": float(lib.sim_get_exact_pose_y(h)),
            "exact_pose_h": float(lib.sim_get_exact_pose_h(h)),
            "otos_x":       float(lib.sim_get_otos_x(h)),
            "otos_y":       float(lib.sim_get_otos_y(h)),
            "otos_h":       float(lib.sim_get_otos_h(h)),
        }

    def _setup_types(self) -> None:
        lib = self._lib

        lib.sim_create.argtypes = []
        lib.sim_create.restype = ctypes.c_void_p

        lib.sim_destroy.argtypes = [ctypes.c_void_p]
        lib.sim_destroy.restype = None

        lib.sim_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        lib.sim_tick.restype = None

        lib.sim_command.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
        lib.sim_command.restype = ctypes.c_int

        lib.sim_get_async_evts.argtypes = [
            ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        lib.sim_get_async_evts.restype = ctypes.c_int

        for name in ("sim_get_enc_l", "sim_get_enc_r",
                     "sim_get_vel_l", "sim_get_vel_r",
                     "sim_get_pwm_l", "sim_get_pwm_r",
                     "sim_get_pose_x", "sim_get_pose_y", "sim_get_pose_h"):
            fn = getattr(lib, name)
            fn.argtypes = [ctypes.c_void_p]
            fn.restype = ctypes.c_float

        lib.sim_set_enc_l.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_enc_l.restype = None
        lib.sim_set_enc_r.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_enc_r.restype = None

        lib.sim_set_otos_pose.argtypes = [
            ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        lib.sim_set_otos_pose.restype = None

        lib.sim_set_motor_offset.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
        lib.sim_set_motor_offset.restype = None

        for name in ("sim_get_exact_pose_x", "sim_get_exact_pose_y", "sim_get_exact_pose_h",
                     "sim_get_otos_x", "sim_get_otos_y", "sim_get_otos_h"):
            fn = getattr(lib, name)
            fn.argtypes = [ctypes.c_void_p]
            fn.restype = ctypes.c_float

        lib.sim_set_motor_slip.argtypes = [
            ctypes.c_void_p, ctypes.c_int, ctypes.c_float, ctypes.c_float]
        lib.sim_set_motor_slip.restype = None

        lib.sim_set_encoder_noise.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
        lib.sim_set_encoder_noise.restype = None

        lib.sim_enable_otos_model.argtypes = [ctypes.c_void_p]
        lib.sim_enable_otos_model.restype = None

        lib.sim_set_otos_fusion.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.sim_set_otos_fusion.restype = None

        lib.sim_set_otos_linear_noise.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_otos_linear_noise.restype = None

        lib.sim_set_otos_yaw_noise.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_otos_yaw_noise.restype = None
