"""NezhaProtocol — v2 wire-protocol adapter for the Nezha firmware.

Owns the SerialConnection and is the only code that touches the serial port.
All command encoding and response parsing lives here; higher-level objects
(NezhaState, Nezha) delegate every wire operation to this class.

Wire format — protocol v2
--------------------------
Requests:
  One '\n'-terminated line, whitespace-delimited tokens.
  Verb is upper-cased by the firmware; remaining tokens preserve case.
  Optional trailing '#<id>' for request correlation.
  Example: "S 200 150\n", "SET ml=0.487\n", "T 200 200 1000 #7\n"

Responses:
  OK   — command accepted:       "OK pong t=12345"
  ERR  — rejected:               "ERR badarg missing key"
  EVT  — async event:            "EVT done T", "EVT done T #12", "EVT safety_stop"
  TLM  — telemetry frame:        "TLM t=12345 enc=1024,1019 pose=350,-12,1780"
  CFG  — config dump:            "CFG ml=0.487 mr=0.481 ..."
  ID   — identity/capabilities:  "ID model=Nezha2 name=GUTOV ..."

EVT done T/D/G and EVT safety_stop carry a trailing '#<id>' when the
originating T/D/G command included one.  Bare events (no id) are unchanged.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Generator

from robot_radio.io.serial_conn import SerialConnection


# ---------------------------------------------------------------------------
# Public data types
# ---------------------------------------------------------------------------

@dataclass
class TLMFrame:
    """Parsed TLM telemetry frame from the firmware.

    All fields are optional — only sensors present in the frame are populated.
    ``t`` is the robot clock in milliseconds at sensor-sample time.
    ``pose`` heading is in centi-degrees (integer), positions in mm.
    ``vel`` is per-wheel measured speed in mm/s (chip-preferred, encoder fallback).
    """
    t: int | None = None
    mode: str | None = None
    enc: tuple[int, int] | None = None          # (left_mm, right_mm)
    pose: tuple[int, int, int] | None = None    # (x_mm, y_mm, heading_cdeg)
    vel: tuple[int, int] | None = None          # (vL_mmps, vR_mmps) — per-wheel mm/s
    line: tuple[int, int, int, int] | None = None   # (g1, g2, g3, g4)
    color: tuple[int, int, int, int] | None = None  # (r, g, b, c)


@dataclass
class ParsedResponse:
    """Structured representation of a single response line from the firmware."""
    tag: str          # "OK", "ERR", "EVT", "TLM", "CFG", "ID"
    tokens: list[str] = field(default_factory=list)  # plain tokens after tag
    kv: dict[str, str] = field(default_factory=dict) # key=value pairs
    corr_id: str | None = None                       # trailing #<id>, if any
    raw: str = ""                                    # original stripped line


# ---------------------------------------------------------------------------
# Module-level parse functions (can be used without a NezhaProtocol instance)
# ---------------------------------------------------------------------------

_RESPONSE_TAGS = frozenset(("OK", "ERR", "EVT", "TLM", "CFG", "ID"))


def _strip_relay(line: str) -> str:
    """Strip relay prefix characters and surrounding whitespace."""
    return line.strip().lstrip("<# ").strip()


def parse_response(line: str) -> ParsedResponse | None:
    """Parse one v2 response line into a ParsedResponse, or None if unrecognised.

    Handles relay prefix stripping, optional trailing '#<id>' correlation token,
    and key=value pair extraction.
    """
    s = _strip_relay(line)
    if not s:
        return None

    parts = s.split()
    if not parts:
        return None

    tag = parts[0].upper()
    if tag not in _RESPONSE_TAGS:
        return None

    rest = parts[1:]

    # Extract trailing corr_id: '#' followed by digits only.
    corr_id: str | None = None
    if rest and rest[-1].startswith("#") and rest[-1][1:].isdigit():
        corr_id = rest[-1][1:]
        rest = rest[:-1]

    # Parse key=value pairs; remainder are plain positional tokens.
    kv: dict[str, str] = {}
    plain: list[str] = []
    for tok in rest:
        if "=" in tok and not tok.startswith("="):
            k, _, v = tok.partition("=")
            kv[k] = v
        else:
            plain.append(tok)

    return ParsedResponse(
        tag=tag,
        tokens=plain,
        kv=kv,
        corr_id=corr_id,
        raw=s,
    )


def parse_tlm(line: str) -> TLMFrame | None:
    """Parse a TLM frame line into a TLMFrame dataclass, or None if not TLM."""
    resp = parse_response(line)
    if resp is None or resp.tag != "TLM":
        return None

    frame = TLMFrame()
    kv = resp.kv

    if "t" in kv:
        try:
            frame.t = int(kv["t"])
        except ValueError:
            pass

    if "mode" in kv:
        frame.mode = kv["mode"]

    if "enc" in kv:
        try:
            parts = kv["enc"].split(",")
            if len(parts) == 2:
                frame.enc = (int(parts[0]), int(parts[1]))
        except ValueError:
            pass

    if "pose" in kv:
        try:
            parts = kv["pose"].split(",")
            if len(parts) == 3:
                frame.pose = (int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError:
            pass

    if "vel" in kv:
        try:
            parts = kv["vel"].split(",")
            if len(parts) == 2:
                frame.vel = (int(parts[0]), int(parts[1]))
        except ValueError:
            pass

    if "line" in kv:
        try:
            parts = kv["line"].split(",")
            if len(parts) == 4:
                frame.line = (int(parts[0]), int(parts[1]),
                              int(parts[2]), int(parts[3]))
        except ValueError:
            pass

    if "color" in kv:
        try:
            parts = kv["color"].split(",")
            if len(parts) == 4:
                frame.color = (int(parts[0]), int(parts[1]),
                               int(parts[2]), int(parts[3]))
        except ValueError:
            pass

    return frame


def parse_cfg(line: str) -> dict[str, str] | None:
    """Parse a CFG response line into a key->value dict, or None if not CFG."""
    resp = parse_response(line)
    if resp is None or resp.tag != "CFG":
        return None
    return dict(resp.kv)


# ---------------------------------------------------------------------------
# NezhaProtocol
# ---------------------------------------------------------------------------

class NezhaProtocol:
    """Wire protocol v2 adapter for the Nezha firmware.

    Owns a SerialConnection and exposes one method per firmware command group.
    All response parsing delegates to module-level parse_* functions so callers
    can reuse them on lines received through other paths (streaming generators).

    v2 protocol rules:
    - Commands are whitespace-separated tokens, verb upper-cased only.
    - Integer values are literal mm (no implicit scaling, no sign prefix).
    - Optional trailing '#<id>' for request/response correlation.
    - Response tags: OK, ERR, EVT, TLM, CFG, ID.
    """

    def __init__(self, conn: SerialConnection) -> None:
        self._conn = conn

    # ------------------------------------------------------------------
    # Connection delegation
    # ------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._conn.is_open

    @property
    def mode(self) -> str | None:
        return self._conn.mode

    def send(self, cmd: str, read_ms: int = 500) -> dict:
        """Send a v2 command, return raw response dict (for ad-hoc / pass-through)."""
        return self._conn.send(cmd, read_ms)

    def send_fast(self, cmd: str) -> None:
        """Fire-and-forget send with no response reading."""
        self._conn.send_fast(cmd)

    def read_lines(self, duration_ms: int) -> list[str]:
        """Blocking read for up to duration_ms milliseconds."""
        return self._conn.read_lines(duration_ms)

    def read_pending_lines(self) -> list[str]:
        """Drain the serial input buffer without blocking."""
        ser = self._conn._ser
        if ser is None or not ser.in_waiting:
            return []
        raw = ser.read(ser.in_waiting).decode("utf-8", errors="replace")
        return [ln for ln in raw.split("\n") if ln.strip()]

    # ------------------------------------------------------------------
    # Static parse helpers (reusable on raw lines from streaming callers)
    # ------------------------------------------------------------------

    @staticmethod
    def parse_response(line: str) -> ParsedResponse | None:
        """Parse a v2 response line. Delegates to module-level parse_response()."""
        return parse_response(line)

    @staticmethod
    def parse_tlm(line: str) -> TLMFrame | None:
        """Parse a TLM line into a TLMFrame. Delegates to module-level parse_tlm()."""
        return parse_tlm(line)

    @staticmethod
    def parse_cfg(line: str) -> dict[str, str] | None:
        """Parse a CFG line into a key->value dict. Delegates to parse_cfg()."""
        return parse_cfg(line)

    # ------------------------------------------------------------------
    # Liveness / identity
    # ------------------------------------------------------------------

    def ping(self, corr_id: str | None = None) -> tuple[int, float] | None:
        """Send PING, parse OK pong t=<robot_ms>.

        Returns (t_robot_ms, rtt_ms) or None if no valid response.
        rtt_ms is the round-trip time measured by this call.
        """
        cmd = "PING" if corr_id is None else f"PING #{corr_id}"
        t0 = time.monotonic()
        resp_dict = self._conn.send(cmd, read_ms=500)
        t1 = time.monotonic()
        rtt_ms = (t1 - t0) * 1000.0

        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "pong":
                if "t" in r.kv:
                    try:
                        return (int(r.kv["t"]), rtt_ms)
                    except ValueError:
                        pass
        return None

    def echo(self, payload: str) -> str | None:
        """Send ECHO <payload>, return echoed payload string or None."""
        resp_dict = self._conn.send(f"ECHO {payload}", read_ms=500)
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "echo":
                # Payload follows "OK echo " in the stripped line.
                prefix = "OK echo "
                s = _strip_relay(raw_line)
                if s.startswith(prefix):
                    return s[len(prefix):].rstrip()
        return None

    def get_id(self) -> dict[str, str] | None:
        """Send ID command. Returns kv dict (model, name, serial, fw, proto, caps) or None."""
        resp_dict = self._conn.send("ID", read_ms=500)
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "ID":
                return dict(r.kv)
        return None

    def get_ver(self) -> dict[str, str] | None:
        """Send VER command. Returns kv dict (fw, proto) or None."""
        resp_dict = self._conn.send("VER", read_ms=500)
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "ver":
                return dict(r.kv)
        return None

    def get_help(self) -> str | None:
        """Send HELP. Returns the verb-list string or None."""
        resp_dict = self._conn.send("HELP", read_ms=500)
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "help":
                return " ".join(r.tokens[1:])
        return None

    # ------------------------------------------------------------------
    # Config: GET / SET
    # ------------------------------------------------------------------

    def get_config(self, *keys: str) -> dict[str, str] | None:
        """Send GET [keys...], parse CFG response into key->value dict.

        With no keys, returns the full config dump (all registered keys).
        Returns None if no CFG line was received.
        """
        cmd = ("GET " + " ".join(keys)) if keys else "GET"
        resp_dict = self._conn.send(cmd, read_ms=500)
        result: dict[str, str] = {}
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "CFG":
                result.update(r.kv)
        return result if result else None

    def set_config(self, **kwargs: Any) -> dict[str, str] | None:
        """Send SET key=value ..., parse OK set response.

        Returns dict of applied keys (from OK set response) or None.
        Floats are formatted with up to 6 significant digits.
        """
        if not kwargs:
            return None
        pairs = []
        for k, v in kwargs.items():
            if isinstance(v, float):
                pairs.append(f"{k}={v:.6g}")
            else:
                pairs.append(f"{k}={v}")
        cmd = "SET " + " ".join(pairs)
        resp_dict = self._conn.send(cmd, read_ms=500)
        result: dict[str, str] = {}
        for raw_line in resp_dict.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "set":
                result.update(r.kv)
        return result if result else None

    # ------------------------------------------------------------------
    # Drive commands
    # ------------------------------------------------------------------

    def stop(self) -> None:
        """Stop motors immediately (STOP command)."""
        self._conn.send_fast("STOP")

    def cancel(self) -> None:
        """Cancel the active motion command (hard stop). Sends X."""
        self._conn.send_fast("X")

    def arc(self, speed_mms: int, radius_mm: int,
            corr_id: str | None = None) -> None:
        """Send R arc command — sets body arc motion (open-ended, no built-in timeout).

        Format: R <speed_mms> <radius_mm> [#id]
        - ``speed_mms``: forward speed in mm/s (−1000 … +1000).
        - ``radius_mm``: arc radius in mm (−10000 … +10000; 0 = straight).
          **Sign convention: positive radius ⇒ CCW (left arc).**
          Matches BodyKinematics::inverse where CCW-positive ω gives vL < vR.
        - ``corr_id``: optional correlation id; echoed in EVT done R.

        Uses fire-and-forget (send_fast). The arc runs until the host sends X
        (hard cancel) or R 0 <r> (speed=0 triggers SOFT ramp-down + EVT done R).
        To use as a keepalive-driven command, re-send within the firmware sTimeout
        window; the firmware does NOT have a built-in keepalive watchdog for R.

        Robot replies ``OK arc speed=… radius=…`` synchronously. On soft-stop
        (speed=0), the firmware emits ``EVT done R`` asynchronously.
        """
        if corr_id is not None:
            self._conn.send_fast(f"R {speed_mms} {radius_mm} #{corr_id}")
        else:
            self._conn.send_fast(f"R {speed_mms} {radius_mm}")

    def vw(self, v_mms: int, omega_mrads: int,
           corr_id: str | None = None) -> None:
        """Send VW keepalive — sets body-twist velocity, resets watchdog.

        Format: VW <v> <omega_mrads> [#id]
        - ``v_mms``: forward speed in mm/s (−1000 … +1000).
        - ``omega_mrads``: yaw rate in milli-radians/s (−3142 … +3142).
          Positive = CCW (left turn).
        - ``corr_id``: optional correlation id; echoed in EVT safety_stop.

        Uses fire-and-forget (send_fast) so it can be called at streaming
        rate without blocking.  The firmware echoes ``OK vw v=… omega=…``
        synchronously, but callers driving at high frequency typically ignore
        the per-frame reply.
        """
        if corr_id is not None:
            self._conn.send_fast(f"VW {v_mms} {omega_mrads} #{corr_id}")
        else:
            self._conn.send_fast(f"VW {v_mms} {omega_mrads}")

    def drive(self, left_mms: int, right_mms: int) -> None:
        """Send S keepalive — sets streaming wheel speeds, resets watchdog.

        Format: S <l> <r>  (space-separated integers, literal mm/s)
        """
        self._conn.send_fast(f"S {left_mms} {right_mms}")

    def timed(self, left_mms: int, right_mms: int, ms: int,
             sensor: str | None = None) -> list[str]:
        """Send T command; return initial response lines.

        Format: T <l> <r> <ms> [sensor=<ch>:<op>:<thr>]
        Robot replies OK drive ...; later sends EVT done T.

        Optional ``sensor`` modifier stops the drive early when a sensor crosses
        a threshold.  Format: ``"<ch>:<op>:<thr>"`` where ch ∈ line0–line3,
        colorR/G/B/C; op ∈ ge|le; thr is an integer raw ADC count.
        Example: sensor="line0:ge:512"
        """
        cmd = f"T {left_mms} {right_mms} {ms}"
        if sensor is not None:
            cmd += f" sensor={sensor}"
        resp = self._conn.send(cmd, read_ms=300)
        return resp.get("responses", [])

    def distance(self, left_mms: int, right_mms: int, mm: int,
                sensor: str | None = None) -> list[str]:
        """Send D command; return initial response lines.

        Format: D <l> <r> <mm> [sensor=<ch>:<op>:<thr>]
        Robot replies OK drive ...; later sends EVT done D.

        Optional ``sensor`` modifier stops the drive early when a sensor crosses
        a threshold.  Format: ``"<ch>:<op>:<thr>"`` (same as timed()).
        Example: sensor="colorC:ge:800"
        """
        cmd = f"D {left_mms} {right_mms} {mm}"
        if sensor is not None:
            cmd += f" sensor={sensor}"
        resp = self._conn.send(cmd, read_ms=300)
        return resp.get("responses", [])

    def go_to(self, x_mm: int, y_mm: int, speed_mms: int) -> list[str]:
        """Send G go-to command; return initial response lines.

        Format: G <x> <y> <speed>
        Robot replies OK goto ...; later sends EVT done G.
        """
        resp = self._conn.send(f"G {x_mm} {y_mm} {speed_mms}", read_ms=300)
        return resp.get("responses", [])

    def turn(self, heading_cdeg: int, eps_cdeg: int | None = None,
             corr_id: str | None = None,
             sensor: str | None = None) -> list[str]:
        """Send TURN command — rotate to an absolute heading and stop within eps.

        Format: TURN <heading_cdeg> [eps=<cdeg>] [sensor=<ch>:<op>:<thr>] [#id]
        - ``heading_cdeg``: target heading in centidegrees (−18000 … +18000 = ±180°).
          Positive values are CCW (matches OTOS CCW convention).
        - ``eps_cdeg``: optional tolerance in centidegrees (default 300 = 3°;
          range 10–1800). Pass a tighter value for calibration use (e.g. 100 = 1°).
        - ``sensor``: optional early-stop modifier; format ``"<ch>:<op>:<thr>"``
          (same as timed() / distance()). Example: sensor="line0:ge:512"
        - ``corr_id``: optional correlation id; echoed in EVT done TURN.

        Robot replies ``OK turn heading=<cdeg> eps=<cdeg>`` synchronously.
        On arrival within eps (or sensor trip): ``EVT done TURN [#<id>]`` emitted async.

        To wait for completion, use ``wait_for_evt_done("TURN", timeout_ms)``.
        Example::

            proto.turn(9000, eps_cdeg=100, corr_id="1")  # turn to +90° (CCW), 1° eps
            result = proto.wait_for_evt_done("TURN", timeout_ms=10000, corr_id="1")
        """
        cmd = f"TURN {heading_cdeg}"
        if eps_cdeg is not None:
            cmd += f" eps={eps_cdeg}"
        if sensor is not None:
            cmd += f" sensor={sensor}"
        if corr_id is not None:
            cmd += f" #{corr_id}"
        resp = self._conn.send(cmd, read_ms=300)
        return resp.get("responses", [])

    def drive_until_sensor(self, left_mms: int, right_mms: int,
                           duration_ms: int,
                           channel: str, threshold: int,
                           op: str = "ge") -> list[str]:
        """Drive timed until a sensor crosses a threshold (or duration expires).

        Convenience wrapper around T with a ``sensor=`` modifier.  The drive stops
        at whichever comes first: the sensor condition or the time limit.

        Args:
            left_mms:   Left wheel speed in mm/s (−1000 … +1000).
            right_mms:  Right wheel speed in mm/s (−1000 … +1000).
            duration_ms: Maximum duration in ms (1 … 30000). Acts as a safety timeout.
            channel:    Sensor channel name: line0–line3, colorR, colorG, colorB, colorC.
            threshold:  Integer threshold in raw sensor units (uint16_t ADC counts).
            op:         Comparison operator: "ge" (≥, default) or "le" (≤).

        Returns:
            Initial response lines from the firmware (OK drive … or ERR …).
            EVT done T is emitted asynchronously; wait with wait_for_evt_done("T").

        Wire format: ``T <left_mms> <right_mms> <duration_ms> sensor=<channel>:<op>:<threshold>``

        Example::

            proto.drive_until_sensor(200, 200, 10000, "line0", 512)
            result = proto.wait_for_evt_done("T", timeout_ms=12000)
            # result is "done" (sensor tripped) or "timeout"
        """
        sensor_token = f"{channel}:{op}:{threshold}"
        return self.timed(left_mms, right_mms, duration_ms, sensor=sensor_token)

    def grip(self, deg: int | None = None) -> int | None:
        """Send GRIP [deg] command. Returns confirmed degree or None.

        Format: GRIP <deg>  or  GRIP (query only)
        Robot replies OK grip deg=<deg>.
        """
        cmd = f"GRIP {deg}" if deg is not None else "GRIP"
        resp = self._conn.send(cmd, read_ms=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "grip":
                try:
                    return int(r.kv["deg"])
                except (KeyError, ValueError):
                    pass
        return None

    def zero_encoders(self) -> None:
        """Zero encoders (ZERO enc command)."""
        self._conn.send("ZERO enc", read_ms=200)

    def zero_otos(self) -> None:
        """Zero OTOS pose tracking (ZERO pose command)."""
        self._conn.send("ZERO pose", read_ms=200)

    def zero_all(self) -> None:
        """Zero both encoders and OTOS pose (ZERO enc pose command)."""
        self._conn.send("ZERO enc pose", read_ms=200)

    # ------------------------------------------------------------------
    # Telemetry streaming
    # ------------------------------------------------------------------

    def stream(self, period_ms: int) -> None:
        """Set TLM streaming period in ms (0 = off).

        Format: STREAM <ms>
        """
        self._conn.send(f"STREAM {period_ms}", read_ms=300)

    def stream_fields(self, fields: str) -> None:
        """Set TLM streaming with a field subset.

        Format: STREAM fields=enc,pose,line
        ``fields`` is a comma-separated string of field names.
        """
        self._conn.send(f"STREAM fields={fields}", read_ms=300)

    def snap(self) -> "TLMFrame | None":
        """Request ONE telemetry frame synchronously and return it (parsed).

        SNAP returns the frame as its reply (request/response), not via the
        stream — so it works while stopped and survives the radio relay (an
        ordinary command-response, unlike a dropped async stream frame). Set the
        desired fields first with stream_fields(); no continuous stream needed.
        """
        resp = self._conn.send("SNAP", read_ms=400)
        for ln in resp.get("responses", []):
            f = parse_tlm(ln)
            if f is not None:
                return f
        return None

    # ------------------------------------------------------------------
    # OTOS sensor
    # ------------------------------------------------------------------

    def otos_init(self) -> None:
        """Enable OTOS signal processing (OI command)."""
        self._conn.send("OI", read_ms=500)

    def otos_zero(self) -> None:
        """Zero OTOS position to current location (OZ command)."""
        self._conn.send("OZ", read_ms=200)

    def otos_reset_tracking(self) -> None:
        """Reset OTOS Kalman filters (OR command)."""
        self._conn.send("OR", read_ms=200)

    def otos_get_position(self) -> tuple[int, int, int] | None:
        """Query OTOS position (OP command). Returns (x_mm, y_mm, h_cdeg) or None."""
        resp = self._conn.send("OP", read_ms=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "pos":
                try:
                    return (int(r.kv["x"]), int(r.kv["y"]), int(r.kv["h"]))
                except (KeyError, ValueError):
                    pass
        return None

    def otos_set_position(self, x_mm: int, y_mm: int, h_cdeg: int) -> None:
        """Set OTOS world-frame position (OV command)."""
        self._conn.send(f"OV {x_mm} {y_mm} {h_cdeg}", read_ms=300)

    def otos_set_linear_scalar(self, val: int) -> int | None:
        """Set OTOS linear scalar (OL <val> command). Returns confirmed value or None."""
        resp = self._conn.send(f"OL {val}", read_ms=500)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "linear":
                try:
                    return int(r.kv["scalar"])
                except (KeyError, ValueError):
                    pass
        return None

    def otos_get_linear_scalar(self) -> int | None:
        """Read back OTOS linear scalar (OL no-arg command). Returns value or None."""
        resp = self._conn.send("OL", read_ms=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "linear":
                try:
                    return int(r.kv["scalar"])
                except (KeyError, ValueError):
                    pass
        return None

    def otos_set_angular_scalar(self, val: int) -> int | None:
        """Set OTOS angular scalar (OA <val> command). Returns confirmed value or None."""
        resp = self._conn.send(f"OA {val}", read_ms=500)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "angular":
                try:
                    return int(r.kv["scalar"])
                except (KeyError, ValueError):
                    pass
        return None

    def otos_get_angular_scalar(self) -> int | None:
        """Read back OTOS angular scalar (OA no-arg command). Returns value or None."""
        resp = self._conn.send("OA", read_ms=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "angular":
                try:
                    return int(r.kv["scalar"])
                except (KeyError, ValueError):
                    pass
        return None

    # ------------------------------------------------------------------
    # J-port I/O
    # ------------------------------------------------------------------

    def port_read(self, port: int) -> int | None:
        """Read digital J-port (P <port> command). Returns 0/1 or None."""
        resp = self._conn.send(f"P {port}", read_ms=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "port":
                try:
                    return int(r.kv["v"])
                except (KeyError, ValueError):
                    pass
        return None

    def port_write(self, port: int, value: bool) -> None:
        """Write digital J-port (P <port> <val> command)."""
        self._conn.send(f"P {port} {1 if value else 0}", read_ms=200)

    def port_read_analog(self, port: int) -> int | None:
        """Read analog J-port (PA <port> command). Returns 0-1023 or None."""
        resp = self._conn.send(f"PA {port}", read_ms=300)
        for raw_line in resp.get("responses", []):
            r = parse_response(raw_line)
            if r and r.tag == "OK" and r.tokens and r.tokens[0] == "aport":
                try:
                    return int(r.kv["v"])
                except (KeyError, ValueError):
                    pass
        return None

    def port_write_analog(self, port: int, value: int) -> None:
        """Write PWM (0-1023) to J-port (PA <port> <val> command)."""
        self._conn.send(f"PA {port} {value}", read_ms=200)

    # ------------------------------------------------------------------
    # Blocking drive helpers (wait for EVT done or safety_stop)
    # ------------------------------------------------------------------

    def wait_for_evt_done(self, verb: str, timeout_ms: int,
                          corr_id: str | None = None) -> str:
        """Block until 'EVT done <verb>' or 'EVT safety_stop' arrives.

        Returns the outcome string: "done", "safety_stop", or "timeout".

        If ``corr_id`` is provided, only EVT lines carrying that id (or bare
        EVT lines without any id) are accepted.  This lets the host distinguish
        completions when multiple correlated drives are in flight.
        """
        deadline = time.time() + timeout_ms / 1000.0
        while time.time() < deadline:
            for raw_line in self._conn.read_lines(duration_ms=100):
                r = parse_response(raw_line)
                if r is None:
                    continue
                if r.tag == "EVT":
                    # When a corr_id filter is specified, skip EVT lines that
                    # carry a *different* id.  Bare EVT lines (r.corr_id None)
                    # are always accepted.
                    if corr_id is not None and r.corr_id is not None:
                        if r.corr_id != corr_id:
                            continue
                    if r.tokens and r.tokens[0] == "done":
                        # Accept if verb matches or no verb given in EVT.
                        if len(r.tokens) < 2 or r.tokens[1] == verb:
                            return "done"
                    elif r.tokens and r.tokens[0] == "safety_stop":
                        return "safety_stop"
        return "timeout"

    # ------------------------------------------------------------------
    # Streaming drive generator
    # ------------------------------------------------------------------

    def stream_drive(
        self,
        speeds: list[int],
        *,
        period_ms: int = 40,
        watchdog_ms: int = 500,
    ) -> Generator[ParsedResponse, None, None]:
        """Streaming drive generator. Yields ParsedResponse for each incoming line.

        Enables TLM streaming on entry, sends S keepalives, disables streaming
        on GeneratorExit. Mutate ``speeds`` in the caller loop to change velocity.
        Ends naturally on EVT safety_stop.

        Args:
            speeds: Mutable [left_mms, right_mms] list; mutate to steer.
            period_ms: TLM streaming period in ms.
            watchdog_ms: S keepalive deadline (ms); must re-send within firmware
                watchdog timeout or motors stop.
        """
        self.stream(period_ms)
        keepalive_s = watchdog_ms * 0.30 / 1000.0

        def _resend_if_due(last: float) -> float:
            now = time.monotonic()
            if now - last >= keepalive_s:
                self._conn.send_fast(f"S {speeds[0]} {speeds[1]}")
                return now
            return last

        try:
            self._conn.send_fast(f"S {speeds[0]} {speeds[1]}")
            last_send = time.monotonic()
            while True:
                for raw_line in self._conn.read_lines(duration_ms=50):
                    r = parse_response(raw_line)
                    if r is None:
                        continue
                    if r.tag == "EVT" and r.tokens and r.tokens[0] == "safety_stop":
                        return
                    yield r
                    last_send = _resend_if_due(last_send)
                last_send = _resend_if_due(last_send)
        except GeneratorExit:
            try:
                self._conn.send_fast("STOP")
                self.stream(0)
            except Exception:
                pass
