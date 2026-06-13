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

from robot_radio.nav.pose import Pose
from robot_radio.robot.robot import Robot
from robot_radio.robot.protocol import NezhaProtocol, TLMFrame, ParsedResponse, parse_tlm, parse_response
from robot_radio.robot.robot_state import RobotState


class RobotNotFoundError(ConnectionError):
    """Raised when the robot does not respond to liveness preflight (PING/ID)."""


class Nezha(Robot):
    """Driver for the DFRobot Nezha2 via serial running protocol v2 firmware.

    The canonical robot state snapshot is available as ``robot.state``
    (a frozen :class:`RobotState`), updated by ``_apply_tlm`` on every
    incoming TLM frame.  The legacy per-attribute names
    (``encoders``, ``otos_pose``, ``line_sensor``, ``color``) remain
    available as thin properties over ``state`` so all existing callers
    are unaffected.

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

        # Unified live sensor state — updated by _apply_tlm on every TLM frame.
        # Legacy per-attribute names (encoders, otos_pose, line_sensor, color)
        # are thin properties over this object so existing callers are unaffected.
        self.state: RobotState = RobotState(
            pose=Pose(x=0.0, y=0.0, heading=0.0),
            v=0.0,
            omega=0.0,
            accel=None,
            stamp=time.monotonic(),
            encoders=None,
            twist=None,
            line=None,
            color=None,
            world_pose=None,
        )

    # ------------------------------------------------------------------
    # Back-compat sensor properties (thin wrappers over self.state)
    # ------------------------------------------------------------------

    @property
    def encoders(self) -> tuple[int, int]:
        """Cached encoder totals (left_mm, right_mm). Returns (0, 0) until first TLM frame."""
        return self.state.encoders or (0, 0)

    @property
    def otos_pose(self) -> tuple[float, float, float]:
        """Cached OTOS pose (x_mm, y_mm, yaw_rad). Returns (0.0, 0.0, 0.0) until first TLM."""
        p = self.state.pose
        return (p.x, p.y, p.heading)

    @property
    def line_sensor(self) -> tuple[int, int, int, int]:
        """Cached line sensor values (g1, g2, g3, g4). Returns (255,255,255,255) until first TLM."""
        return self.state.line or (255, 255, 255, 255)

    @property
    def color(self) -> tuple[int, int, int, int]:
        """Cached colour sensor values (r, g, b, c). Returns (0, 0, 0, 0) until first TLM."""
        return self.state.color or (0, 0, 0, 0)

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
                    self._apply_tlm(tlm)
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

    def _run_until_done(
        self, verb: str, on_tick: Any, timeout_s: float
    ) -> str:
        """Private tick loop for callback-driven go_to / turn.

        Reads lines from ``self._proto._conn.read_lines(duration_ms=50)``,
        updates robot state from each TLM frame, and calls ``on_tick(self)``
        after each update.

        Returns one of: ``"done"``, ``"safety_stop"``, ``"aborted"``,
        ``"timeout"``.

        Termination rules (in evaluation order per iteration):
        1. ``on_tick`` returns ``False`` → send ``X``, disable stream,
           return ``"aborted"``.
        2. ``EVT done <verb>`` arrives → disable stream, return ``"done"``.
        3. ``EVT safety_stop`` arrives → disable stream, return
           ``"safety_stop"``.
        4. Wall-clock ``timeout_s`` exceeded → send ``X``, disable stream,
           return ``"timeout"``.
        5. No TLM arrived and keepalive interval elapsed → send ``+``
           keepalive (safety belt; the SerialConnection daemon also does
           this).
        """
        _KEEPALIVE_INTERVAL = 0.200  # seconds between keepalives
        deadline = time.monotonic() + timeout_s
        last_keepalive = time.monotonic()
        had_tlm = False

        while time.monotonic() < deadline:
            lines = self._proto._conn.read_lines(duration_ms=50)
            had_tlm = False

            for raw_line in lines:
                r = parse_response(raw_line)
                if r is None:
                    continue

                if r.tag == "TLM":
                    tlm = parse_tlm(raw_line)
                    if tlm is not None:
                        self._apply_tlm(tlm)
                        had_tlm = True
                        result = on_tick(self)
                        if result is False:
                            self._proto._conn.send_fast("X")
                            self._proto.stream(0)
                            return "aborted"

                elif r.tag == "EVT":
                    tokens = r.tokens
                    if tokens and tokens[0] == "done":
                        if len(tokens) < 2 or tokens[1] == verb:
                            self._proto.stream(0)
                            return "done"
                    elif tokens and tokens[0] == "safety_stop":
                        self._proto.stream(0)
                        return "safety_stop"

            # Keepalive safety belt (daemon normally covers this).
            now = time.monotonic()
            if not had_tlm and (now - last_keepalive) >= _KEEPALIVE_INTERVAL:
                self._proto._conn.send_fast("+")
                last_keepalive = now

        # Deadline exceeded.
        self._proto._conn.send_fast("X")
        self._proto.stream(0)
        return "timeout"

    def go_to(self, x_mm: int, y_mm: int, speed_mms: int,
              on_tick: Any = None,
              timeout_s: float = 15.0) -> tuple[int, int, str]:
        """Blocking or callback-driven go-to (G command).

        Returns ``(left_enc_mm, right_enc_mm, outcome)`` where ``outcome``
        is one of ``"done"``, ``"safety_stop"``, or ``"timeout"``.

        Parameters
        ----------
        x_mm, y_mm:
            Target position in robot-relative mm (forward, left).
        speed_mms:
            Cruise speed in mm/s (clamped to ≥ 1).
        on_tick:
            When ``None`` (default), the method blocks using
            ``wait_for_evt_done`` — identical to the pre-sprint behaviour.
            No STREAM is enabled.  This is the path used by Navigator.

            When a callable is provided, ``STREAM 80`` is enabled before
            issuing ``G``, and ``on_tick(robot)`` is called after each TLM
            tick.  Returning ``False`` from ``on_tick`` aborts the move
            (sends ``X``, outcome ``"aborted"``).
        timeout_s:
            Maximum wall-clock seconds to wait.
        """
        speed = max(abs(speed_mms), 1)
        if on_tick is None:
            # Back-compat blocking path — behaviour unchanged from pre-sprint.
            self._proto.go_to(x_mm, y_mm, speed)
            timeout_ms = int(timeout_s * 1000)
            outcome = self._proto.wait_for_evt_done("G", timeout_ms)
            time.sleep(0.2)
            return self.encoders[0], self.encoders[1], outcome
        else:
            self._proto.stream(80)
            self._proto.go_to(x_mm, y_mm, speed)
            outcome = self._run_until_done("G", on_tick, timeout_s)
            return self.encoders[0], self.encoders[1], outcome

    def turn(self, heading_cdeg: int, on_tick: Any = None,
             eps_cdeg: int | None = None,
             timeout_s: float = 10.0) -> str:
        """Rotate to an absolute heading (TURN command).

        Returns outcome string: ``"done"``, ``"safety_stop"``,
        ``"aborted"``, or ``"timeout"``.

        Parameters
        ----------
        heading_cdeg:
            Target heading in centi-degrees.  Positive = CCW (matches OTOS
            CCW convention).  Range −18000 … +18000.
        on_tick:
            When ``None`` (default), blocks using ``wait_for_evt_done``.
            When a callable, enables ``STREAM 80``, issues ``TURN``, and
            calls ``on_tick(robot)`` after each TLM tick.  Return ``False``
            from ``on_tick`` to abort.
        eps_cdeg:
            Optional heading tolerance in centi-degrees (default 300 = 3°).
        timeout_s:
            Maximum wall-clock seconds to wait.
        """
        if on_tick is None:
            self._proto.turn(heading_cdeg, eps_cdeg=eps_cdeg)
            timeout_ms = int(timeout_s * 1000)
            return self._proto.wait_for_evt_done("TURN", timeout_ms)
        else:
            self._proto.stream(80)
            self._proto.turn(heading_cdeg, eps_cdeg=eps_cdeg)
            return self._run_until_done("TURN", on_tick, timeout_s)

    def read_encoders(self) -> tuple[int, int]:
        """Return cached encoder state (updated by streaming TLM frames)."""
        return self.encoders

    def zero_encoders(self) -> None:
        """Zero encoder counters (ZERO enc command)."""
        self._proto.zero_encoders()
        self.state = RobotState(
            pose=self.state.pose,
            v=self.state.v,
            omega=self.state.omega,
            accel=self.state.accel,
            stamp=self.state.stamp,
            encoders=(0, 0),
            twist=self.state.twist,
            line=self.state.line,
            color=self.state.color,
            world_pose=self.state.world_pose,
        )

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
        """Construct a new frozen RobotState from the incoming TLMFrame.

        Fields absent from the frame retain their previous values from
        ``self.state`` (partial-frame handling).
        """
        prev = self.state

        # Pose: update if the frame carries a pose= field.
        if tlm.pose is not None:
            x_mm, y_mm, h_cdeg = tlm.pose
            # heading is centi-degrees (integer); convert to radians CCW-positive
            yaw_rad = math.radians(h_cdeg / 100.0)
            new_pose = Pose(x=float(x_mm), y=float(y_mm), heading=yaw_rad)
        else:
            new_pose = prev.pose

        # Fused body-frame velocity (optional twist= field).
        if tlm.twist is not None:
            v_mmps, omega_mradps = tlm.twist
            new_v = float(v_mmps)
            new_omega = float(omega_mradps) / 1000.0  # mrad/s → rad/s
        else:
            new_v = prev.v
            new_omega = prev.omega

        self.state = RobotState(
            pose=new_pose,
            v=new_v,
            omega=new_omega,
            accel=prev.accel,
            stamp=time.monotonic(),
            encoders=tlm.enc if tlm.enc is not None else prev.encoders,
            twist=tlm.twist if tlm.twist is not None else prev.twist,
            line=tlm.line if tlm.line is not None else prev.line,
            color=tlm.color if tlm.color is not None else prev.color,
            world_pose=prev.world_pose,
        )

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
        """Request one immediate TLM frame (SNAP command).

        Low-level fire-and-forget wrapper; the returned frame is discarded.
        Use ``refresh()`` to issue SNAP and capture the result into ``state``.
        """
        self._proto.snap()

    def refresh(self) -> "RobotState":
        """Issue a one-shot SNAP and return the updated robot state.

        Calls ``self._proto.snap()`` which synchronously returns a parsed
        ``TLMFrame`` (or ``None`` if the robot sent no TLM in its response).
        When a frame is received it is fed through ``_apply_tlm`` to update
        ``self.state``; when no frame is received the prior ``self.state`` is
        returned unchanged.

        This is the idle-state-query path — no streaming, no background thread.

        Returns
        -------
        RobotState
            The current (possibly just-refreshed) robot state.
        """
        tlm = self._proto.snap()
        if tlm is not None:
            self._apply_tlm(tlm)
        return self.state

    # ------------------------------------------------------------------
    # OTOS sensor management
    # ------------------------------------------------------------------

    def zero_otos(self) -> None:
        """Zero the OTOS position at the current location (ZERO pose command)."""
        self._proto.zero_otos()
        prev = self.state
        self.state = RobotState(
            pose=Pose(x=0.0, y=0.0, heading=prev.pose.heading),
            v=prev.v,
            omega=prev.omega,
            accel=prev.accel,
            stamp=prev.stamp,
            encoders=prev.encoders,
            twist=prev.twist,
            line=prev.line,
            color=prev.color,
            world_pose=prev.world_pose,
        )

    def init_otos(self) -> None:
        """Initialise the OTOS sensor (OI command). Robot must be still."""
        self._proto.otos_init()

    def reset_otos_tracking(self) -> None:
        """Reset OTOS Kalman filters (OR command)."""
        self._proto.otos_reset_tracking()

    def set_world_pose(self, x_mm: int, y_mm: int, h_cdeg: int) -> None:
        """Set OTOS world-frame pose (OV command). Heading in centi-degrees."""
        self._proto.otos_set_position(x_mm, y_mm, h_cdeg)

    def update_world_pose(self, x_cm: float, y_cm: float, yaw_rad: float) -> None:
        """Set the world-frame pose from camera-native units and record it in state.

        Converts camera-native units to firmware wire units and calls
        ``set_world_pose`` (OV command), then records the camera-native values
        in ``self.state.world_pose`` as ``(x_cm, y_cm, yaw_rad)`` so callers
        can read the last-set world pose back without unit conversion.

        Unit conventions
        ----------------
        Input  (camera-native): centimetres for x/y, radians for heading.
        Wire   (firmware):      ``x_mm = round(x_cm * 10)``,
                                ``y_mm = round(y_cm * 10)``,
                                ``h_cdeg = round(degrees(yaw_rad) * 100)``.
        Stored (state):         ``(x_cm, y_cm, yaw_rad)`` — camera units, unchanged.

        Parameters
        ----------
        x_cm:
            World x-position in centimetres.
        y_cm:
            World y-position in centimetres.
        yaw_rad:
            World heading in radians (CCW-positive, 0 = north/forward).
        """
        x_mm = round(x_cm * 10)
        y_mm = round(y_cm * 10)
        h_cdeg = round(math.degrees(yaw_rad) * 100)
        self._proto.otos_set_position(x_mm, y_mm, h_cdeg)
        prev = self.state
        self.state = RobotState(
            pose=prev.pose,
            v=prev.v,
            omega=prev.omega,
            accel=prev.accel,
            stamp=prev.stamp,
            encoders=prev.encoders,
            twist=prev.twist,
            line=prev.line,
            color=prev.color,
            world_pose=(x_cm, y_cm, yaw_rad),
        )

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
