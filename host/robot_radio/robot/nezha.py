"""Nezha — robot driver for the Nezha2 firmware, protocol v2.

Commands (full v2 reference in source/app/CommandProcessor.cpp):

Motion
------
  S <l> <r>         streaming speed burst (watchdog-based)
  T <l> <r> <ms>    timed speed burst; robot sends EVT done T when done
  D <l> <r> <mm>    drive by distance;  robot sends EVT done D when done
  G <x> <y> <speed> go-to XY arc;       robot sends EVT done G when done
  STOP              stop immediately

Gripper
-------
  GRIP <deg>        set gripper servo angle
  GRIP              query current angle (returns OK grip deg=<val>)

Encoders / pose
---------------
  ZERO enc          zero encoder counters
  ZERO pose         zero OTOS pose tracking
  ZERO enc pose     zero both

Telemetry
---------
  STREAM <ms>       set TLM period (0=off)
  STREAM fields=…   subscribe to a subset of TLM fields
  SNAP              request one immediate TLM frame

OTOS sensor
-----------
  OI    init signal processing
  OZ    zero position at current location
  OR    reset Kalman filters
  OP    query position -> OK pos x= y= h=
  OV    set world-frame position

Config
------
  GET [keys…]       read config (all or named subset) -> CFG key=val ...
  SET key=val …     write config keys               -> OK set key=val ...
"""

from __future__ import annotations

import math
import time
from typing import Any, Generator

from robot_radio.robot.robot import Robot
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame, ParsedResponse, parse_tlm


class RobotNotFoundError(ConnectionError):
    """Raised when the robot does not respond to liveness preflight (PING/ID)."""


class Nezha(Robot):
    """Driver for the DFRobot Nezha2 via serial running protocol v2 firmware.

    Streaming drive interface:
      - ``stream_drive(speeds, ...)`` — generator; yields ParsedResponse objects
        and updates robot state (encoders, otos_pose, line_sensor, color)
      - ``send_drive(left, right)``   — fire-and-forget S keepalive for manual loops

    Typical streaming loop::

        speeds = [200, 200]
        for resp in robot.stream_drive(speeds, period_ms=40):
            tlm = parse_tlm(resp.raw) if resp.tag == "TLM" else None
            if tlm and tlm.enc:
                print(tlm.enc)
            # mutate speeds to steer
            speeds[0] = new_left
            speeds[1] = new_right

    ``otos_pose`` stores ``(x_mm, y_mm, yaw_rad)`` with CCW-positive yaw in radians.
    ``encoders`` stores ``(left_mm, right_mm)`` cumulative totals.
    """

    # Below this, neither wheel reliably rotates (deadband).
    MIN_SPEED_MMS = 12

    def __init__(self, proto: NezhaProtocol) -> None:
        self._proto = proto

        # Live sensor state — updated by streaming generators.
        self.encoders:    tuple[int, int]             = (0, 0)
        self.otos_pose:   tuple[float, float, float]  = (0.0, 0.0, 0.0)  # x_mm, y_mm, yaw_rad
        self.line_sensor: tuple[int, int, int, int]   = (255, 255, 255, 255)
        self.color:       tuple[int, int, int, int]   = (0, 0, 0, 0)

    # ------------------------------------------------------------------
    # Robot interface — connection
    # ------------------------------------------------------------------

    def is_connected(self) -> bool:
        return self._proto.is_open

    def connect(self) -> dict[str, str]:
        """Run liveness preflight: PING then ID.

        Returns the robot identity kv dict on success.
        Raises RobotNotFoundError if either step times out or returns no response.
        """
        ping_result = self._proto.ping()
        if ping_result is None:
            raise RobotNotFoundError(
                "Robot did not respond to PING — check cable, relay, and power."
            )
        id_result = self._proto.get_id()
        if id_result is None:
            raise RobotNotFoundError(
                "Robot did not respond to ID — relay may be present but robot is silent."
            )
        return id_result

    # ------------------------------------------------------------------
    # Robot interface — motion
    # ------------------------------------------------------------------

    def stop(self) -> None:
        """Stop motors immediately (STOP command)."""
        self._proto.stop()

    def grip(self, angle: int) -> None:
        """Set gripper servo angle (GRIP <deg> command)."""
        self._proto.grip(angle)

    def speed(self, left_mms: int, right_mms: int) -> Generator[tuple[int, int], None, None]:
        """Non-blocking encoder streaming. Yields (left_mm, right_mm) encoder totals.

        Sends S keepalives to maintain streaming. Close the generator to stop.
        """
        def _clamp(v: int) -> int:
            return 0 if v == 0 else max(self.MIN_SPEED_MMS, abs(v)) * (1 if v > 0 else -1)

        l, r = _clamp(left_mms), _clamp(right_mms)
        speeds = [l, r]
        try:
            for resp in self._proto.stream_drive(speeds, period_ms=40, watchdog_ms=200):
                tlm = parse_tlm(resp.raw) if resp.tag == "TLM" else None
                if tlm and tlm.enc:
                    self.encoders = tlm.enc
                    yield tlm.enc
        except GeneratorExit:
            pass

    def speed_for_time(self, left_mms: int, right_mms: int, ms: int) -> tuple[int, int]:
        """Blocking timed drive (T command). Returns final encoder totals (mm).

        Waits for EVT done T or a conservative host-side timeout.
        """
        def _clamp(v: int) -> int:
            return 0 if v == 0 else max(self.MIN_SPEED_MMS, abs(v)) * (1 if v > 0 else -1)

        l, r = _clamp(left_mms), _clamp(right_mms)
        self._proto.timed(l, r, ms)
        self._proto.wait_for_evt_done("T", ms + 2000)
        return self.encoders

    def speed_for_distance(self, left_mms: int, right_mms: int, mm: int) -> tuple[int, int]:
        """Blocking distance drive (D command). Returns final encoder totals (mm)."""
        def _clamp(v: int) -> int:
            return 0 if v == 0 else max(self.MIN_SPEED_MMS, abs(v)) * (1 if v > 0 else -1)

        l, r = _clamp(left_mms), _clamp(right_mms)
        remaining_mm = abs(int(mm))
        if remaining_mm == 0:
            return self.encoders

        cruise_mms = max(abs(l), abs(r), 1)
        hop_mm_max = max(40, int(cruise_mms * 1.5))

        while remaining_mm > 0:
            hop_mm = min(remaining_mm, hop_mm_max)
            self._proto.read_pending_lines()
            self._proto.distance(l, r, hop_mm)

            outcome = self._proto.wait_for_evt_done("D", timeout_ms=6000)
            if outcome == "timeout":
                self._proto.stop()
                raise TimeoutError(
                    f"Distance hop timed out: target={hop_mm}mm at speeds {l},{r}"
                )
            remaining_mm -= hop_mm

        return self.encoders

    def go_to(self, x_mm: int, y_mm: int, speed_mms: int,
              timeout_s: float = 15.0) -> tuple[int, int, str]:
        """Blocking go-to (G command). Returns (left_enc_mm, right_enc_mm, outcome).

        outcome is one of "done", "safety_stop", or "timeout".
        """
        speed = max(abs(speed_mms), 1)
        self._proto.go_to(x_mm, y_mm, speed)
        timeout_ms = int(timeout_s * 1000)
        outcome = self._proto.wait_for_evt_done("G", timeout_ms)
        time.sleep(0.2)
        return self.encoders[0], self.encoders[1], outcome

    def read_encoders(self) -> tuple[int, int]:
        """Return cached encoder state (updated by streaming TLM frames)."""
        return self.encoders

    def zero_encoders(self) -> None:
        """Zero encoder counters (ZERO enc command)."""
        self._proto.zero_encoders()
        self.encoders = (0, 0)

    def send(self, message: str, read_ms: int = 500) -> dict[str, Any]:
        """Send arbitrary v2 command string, return raw response dict."""
        return self._proto.send(message, read_ms)

    # ------------------------------------------------------------------
    # Streaming drive
    # ------------------------------------------------------------------

    def send_drive(self, left_mms: int, right_mms: int) -> None:
        """Fire-and-forget S keepalive for manual control loops."""
        self._proto.drive(left_mms, right_mms)

    def stream_drive(
        self,
        speeds: list[int],
        *,
        period_ms: int = 40,
        watchdog_ms: int = 500,
    ) -> Generator[ParsedResponse, None, None]:
        """Streaming drive generator. Yields ParsedResponse for each incoming line.

        Updates self.encoders, self.otos_pose, self.line_sensor, self.color
        before yielding each parsed TLM frame. Mutate ``speeds`` in the caller
        loop to change velocity mid-stream. Ends naturally on EVT safety_stop.
        """
        for resp in self._proto.stream_drive(
            speeds, period_ms=period_ms, watchdog_ms=watchdog_ms
        ):
            if resp.tag == "TLM":
                tlm = parse_tlm(resp.raw)
                if tlm:
                    self._apply_tlm(tlm)
            yield resp

    def _apply_tlm(self, tlm: TLMFrame) -> None:
        """Apply a TLMFrame to live sensor state."""
        if tlm.enc is not None:
            self.encoders = tlm.enc
        if tlm.pose is not None:
            x_mm, y_mm, h_cdeg = tlm.pose
            # heading is centi-degrees (integer); convert to radians CCW-positive
            yaw_rad = math.radians(h_cdeg / 100.0)
            self.otos_pose = (float(x_mm), float(y_mm), yaw_rad)
        if tlm.line is not None:
            self.line_sensor = tlm.line
        if tlm.color is not None:
            self.color = tlm.color

    # ------------------------------------------------------------------
    # Ping / identity
    # ------------------------------------------------------------------

    def ping(self) -> tuple[int, float] | None:
        """Send PING, return (t_robot_ms, rtt_ms) or None."""
        return self._proto.ping()

    def get_id(self) -> dict[str, str] | None:
        """Send ID command, return identity kv dict or None."""
        return self._proto.get_id()

    # ------------------------------------------------------------------
    # Config
    # ------------------------------------------------------------------

    def get_config(self, *keys: str) -> dict[str, str] | None:
        """Read config (GET [keys...]). Returns key->value dict or None."""
        return self._proto.get_config(*keys)

    def set_config(self, **kwargs: Any) -> dict[str, str] | None:
        """Write config keys (SET key=val ...). Returns applied kv or None."""
        return self._proto.set_config(**kwargs)

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def stream_tlm(self, period_ms: int) -> None:
        """Enable TLM streaming at the given period (STREAM <ms>). 0 = off."""
        self._proto.stream(period_ms)

    def snap(self) -> None:
        """Request one immediate TLM frame (SNAP command)."""
        self._proto.snap()

    # ------------------------------------------------------------------
    # OTOS sensor management
    # ------------------------------------------------------------------

    def zero_otos(self) -> None:
        """Zero the OTOS position at the current location (ZERO pose command)."""
        self._proto.zero_otos()
        self.otos_pose = (0.0, 0.0, self.otos_pose[2])

    def init_otos(self) -> None:
        """Initialise the OTOS sensor (OI command). Robot must be still."""
        self._proto.otos_init()

    def reset_otos_tracking(self) -> None:
        """Reset OTOS Kalman filters (OR command)."""
        self._proto.otos_reset_tracking()

    def set_world_pose(self, x_mm: int, y_mm: int, h_cdeg: int) -> None:
        """Set OTOS world-frame pose (OV command). Heading in centi-degrees."""
        self._proto.otos_set_position(x_mm, y_mm, h_cdeg)

    def read_otos_pose(self) -> tuple[int, int, int] | None:
        """Query current OTOS pose (OP command). Returns (x_mm, y_mm, h_cdeg) or None."""
        return self._proto.otos_get_position()

    def set_otos_linear_scalar(self, val: int) -> int | None:
        """Set OTOS linear scalar (OL <val>). Returns confirmed value or None."""
        return self._proto.otos_set_linear_scalar(val)

    def get_otos_linear_scalar(self) -> int | None:
        """Read back OTOS linear scalar (OL). Returns value or None."""
        return self._proto.otos_get_linear_scalar()

    def set_otos_angular_scalar(self, val: int) -> int | None:
        """Set OTOS angular scalar (OA <val>). Returns confirmed value or None."""
        return self._proto.otos_set_angular_scalar(val)

    def get_otos_angular_scalar(self) -> int | None:
        """Read back OTOS angular scalar (OA). Returns value or None."""
        return self._proto.otos_get_angular_scalar()

    # ------------------------------------------------------------------
    # J-port I/O
    # ------------------------------------------------------------------

    def port_read(self, port: int) -> int | None:
        """Read digital J-port (P <port>). Returns 0/1 or None."""
        return self._proto.port_read(port)

    def port_write(self, port: int, value: bool) -> None:
        """Write digital J-port (P <port> <val>)."""
        self._proto.port_write(port, value)

    def port_read_analog(self, port: int) -> int | None:
        """Read analog J-port (PA <port>). Returns 0-1023 or None."""
        return self._proto.port_read_analog(port)

    def port_write_analog(self, port: int, value: int) -> None:
        """Write PWM (0-1023) to J-port (PA <port> <val>)."""
        self._proto.port_write_analog(port, value)

    # ------------------------------------------------------------------
    # Connection access
    # ------------------------------------------------------------------

    @property
    def connection(self) -> NezhaProtocol:
        return self._proto
