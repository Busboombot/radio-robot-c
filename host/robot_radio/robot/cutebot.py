"""Cutebot robot — concrete Robot implementation via serial relay.

Uses the compact sign-delimited command protocol:
  S+L-R        Non-blocking PID speed (mm/s), streams ENC reports
  T+L+R+ms     Blocking PID speed for time, returns final ENC
  D+L+R+mm     Blocking PID speed for distance, returns final ENC
  X            Stop
  G+angle      Grip servo
  ENC          Read encoders (mm)
  EZ           Zero encoders
"""

import time
from typing import Any, Generator

from robot_radio.robot.robot import Robot
from robot_radio.io.serial_conn import SerialConnection

GRIPPER_OFFSET_CM = 14.0


def _sign(v: int) -> str:
    """Format integer with explicit sign for the compact protocol."""
    return f"+{v}" if v >= 0 else str(v)


def _parse_enc(line: str) -> tuple[int, int] | None:
    """Parse 'ENC <left> <right>' line into (left_mm, right_mm).
    Tolerates relay prefix (e.g. '<ENC 100 200')."""
    # Strip leading non-alpha chars (relay prefix like '<' or '# ')
    stripped = line.lstrip("<# ")
    parts = stripped.split()
    if len(parts) >= 3 and parts[0] == "ENC":
        try:
            return int(parts[1]), int(parts[2])
        except ValueError:
            pass
    return None


class Cutebot(Robot):
    """Cutebot communicating via serial relay to a micro:bit."""

    def __init__(self, conn: SerialConnection):
        self._conn = conn

    def is_connected(self) -> bool:
        return self._conn.is_open

    def speed(self, left_mms: int, right_mms: int) -> Generator[tuple[int, int], None, None]:
        """Non-blocking PID speed. Yields encoder positions as they stream.

        The firmware streams ENC reports every ~50ms while motors run.
        The firmware stops motors if no S command is re-sent within 150ms,
        so this generator re-sends S periodically to keep motors alive.
        Close the generator to stop.
        """
        cmd = f"S{_sign(left_mms)}{_sign(right_mms)}"
        try:
            self._conn.send_fast(cmd)
            while True:
                lines = self._conn.read_lines(duration_ms=100)
                for line in lines:
                    enc = _parse_enc(line)
                    if enc:
                        yield enc
                    elif "SAFETY_STOP" in line or "LOG:X" in line:
                        return
                # Re-send S to keep motors alive (firmware timeout is 200ms)
                self._conn.send_fast(cmd)
        except GeneratorExit:
            # Stop re-sending S. Send explicit X, then wait for firmware
            # to confirm stop (SAFETY_STOP or LOG:X) before returning.
            # This avoids closing the port while motors are still running.
            try:
                self._conn.send_fast("X")
            except Exception:
                pass
            deadline = time.time() + 0.5
            while time.time() < deadline:
                lines = self._conn.read_lines(duration_ms=100)
                for line in lines:
                    if "SAFETY_STOP" in line or "LOG:X" in line:
                        return

    def _send_and_wait_enc(self, cmd: str, timeout_ms: int) -> tuple[int, int]:
        """Send command, read lines until ENC response arrives.
        If the ENC response is lost (radio is unreliable), fall back
        to sending an explicit ENC command to read encoders."""
        wire = f">{cmd}\n" if self._conn.mode == "relay" else f"{cmd}\n"
        if self._conn.on_send:
            self._conn.on_send(wire.rstrip())
        self._conn._ser.write(wire.encode("utf-8"))
        self._conn._ser.flush()
        deadline = time.time() + timeout_ms / 1000.0
        while time.time() < deadline:
            lines = self._conn.read_lines(duration_ms=200)
            for line in lines:
                enc = _parse_enc(line)
                if enc:
                    return enc
        # Response lost (radio is unreliable) — ask for encoders directly
        for _ in range(3):
            enc = self.read_encoders()
            if enc != (0, 0):
                return enc
            time.sleep(0.1)
        return self.read_encoders()

    def speed_for_time(self, left_mms: int, right_mms: int, ms: int) -> tuple[int, int]:
        """Blocking: drive at speed for ms milliseconds. Returns final encoder (mm)."""
        cmd = f"T{_sign(left_mms)}{_sign(right_mms)}{_sign(ms)}"
        return self._send_and_wait_enc(cmd, ms + 2000)

    def speed_for_distance(self, left_mms: int, right_mms: int, mm: int) -> tuple[int, int]:
        """Blocking: drive at speed until distance. Returns final encoder (mm)."""
        cmd = f"D{_sign(left_mms)}{_sign(right_mms)}{_sign(mm)}"
        min_speed = max(abs(left_mms), abs(right_mms), 1)
        timeout_ms = int(mm / min_speed * 1000) + 3000
        return self._send_and_wait_enc(cmd, min(timeout_ms, 8000))

    def go_to(self, x_mm: int, y_mm: int, speed_mms: int,
              timeout_s: float = 15.0) -> tuple[int, int, str]:
        """Blocking go-to (G command) — pure-pursuit arc to relative (X, Y) in mm.

        The firmware pre-rotates if |bearing| > 45°, then drives an arc (or
        straight line after rotation).  Completion is signalled by the
        firmware emitting G+DONE; G+TIMEOUT indicates the deadline fired.

        Returns (left_enc_mm, right_enc_mm, outcome) where outcome is
        "DONE" or "TIMEOUT" (or "HOST_TIMEOUT" if our own wait expired).
        """
        speed = max(abs(speed_mms), 1)
        cmd = f"G{_sign(x_mm)}{_sign(y_mm)}{_sign(speed)}"
        self._conn.send(cmd, read_ms=300)

        deadline = time.time() + timeout_s
        outcome = "HOST_TIMEOUT"
        while time.time() < deadline:
            lines = self._conn.read_lines(duration_ms=200)
            done = False
            for line in lines:
                # Relay responses are prefixed with '<'; strip it before matching.
                s = str(line).lstrip("<# ")
                if s.startswith("G+DONE"):
                    outcome = "DONE"
                    done = True
                    break
                if s.startswith("G+TIMEOUT"):
                    outcome = "TIMEOUT"
                    done = True
                    break
            if done:
                break

        time.sleep(0.2)
        left, right = self.read_encoders()
        return left, right, outcome

    # Position mode constants — match POS_MODE_* in src/nezha.ts.
    POS_MODE_CW       = 1
    POS_MODE_CCW      = 2
    POS_MODE_SHORTEST = 3

    def rotate(self, l_deg: float | None, r_deg: float | None,
               speed_pct: int) -> dict:
        """Send a PR (relative rotate) command and return immediately.

        ``l_deg`` / ``r_deg`` are signed degrees from current position.
        ``None`` for either argument means "skip that wheel" (the
        firmware just leaves it where it is).  ``speed_pct`` is 1..100
        of max servo speed.

        Does NOT wait for completion — the motor controller runs the
        move autonomously.  Returns the raw firmware response dict
        (with the ACK line).
        """
        speed = max(min(abs(speed_pct), 100), 1)
        # PR skip sentinel = 0 (no rotation to do).
        l_tenths = 0 if l_deg is None else int(round(l_deg * 10))
        r_tenths = 0 if r_deg is None else int(round(r_deg * 10))
        cmd = f"ROT{_sign(l_tenths)}{_sign(r_tenths)}{_sign(speed)}"
        left_enc, right_enc = robot.read_encoders()

        return self._conn.send(cmd, read_ms=300)

    def angle(self, l_deg: float | None, r_deg: float | None,
              mode: int, speed_pct: int) -> dict:
        """Send a PA (absolute angle) command and return immediately.

        ``l_deg`` / ``r_deg`` are 0..360°.  ``None`` for either means
        "skip that wheel".  ``mode`` is POS_MODE_CW, POS_MODE_CCW, or
        POS_MODE_SHORTEST.

        Does NOT wait for completion.
        """
        speed = max(min(abs(speed_pct), 100), 1)
        # PA skip sentinel = -1 (since 0 is a valid absolute angle).
        l_tenths = -1 if l_deg is None else int(round(l_deg * 10)) % 3600
        r_tenths = -1 if r_deg is None else int(round(r_deg * 10)) % 3600
        cmd = f"ANG{_sign(l_tenths)}{_sign(r_tenths)}{_sign(mode)}{_sign(speed)}"
        return self._conn.send(cmd, read_ms=300)

    def _legacy_await_position(self, cmd: str, timeout_s: float) -> tuple[int, int, str]:
        """Legacy: send a position command and wait for P+DONE/TIMEOUT.
        Kept for callers that actually want to block until the move
        finishes (typically only the calibration tests)."""
        self._conn.send(cmd, read_ms=300)
        deadline = time.time() + timeout_s
        outcome = "HOST_TIMEOUT"
        actual_l = 0
        actual_r = 0
        while time.time() < deadline:
            lines = self._conn.read_lines(duration_ms=200)
            done = False
            for line in lines:
                s = str(line).lstrip("<# ")
                if s.startswith("P+DONE") or s.startswith("P+TIMEOUT"):
                    parts = s.split()
                    outcome = "DONE" if parts[0] == "P+DONE" else "TIMEOUT"
                    if len(parts) >= 3:
                        try:
                            actual_l = int(parts[1])
                            actual_r = int(parts[2])
                        except ValueError:
                            pass
                    done = True
                    break
            if done:
                break
        return actual_l, actual_r, outcome

    def stop(self) -> None:
        self._conn.send_fast("X")

    def grip(self, angle: int) -> None:
        # Use blocking send (waits for ACK) — fire-and-forget can be
        # discarded by the OS USB driver before bytes drain if the CLI
        # exits immediately after.
        self._conn.send(f"G{_sign(angle)}", read_ms=300)

    def read_encoders(self) -> tuple[int, int]:
        """Read encoder positions in whole degrees.

        Returns (left_deg, right_deg).  To convert to mm, multiply by
        the wheel's mm/deg ratio.
        """
        resp = self._conn.send("ENC", read_ms=200)
        for line in resp.get("responses", []):
            enc = _parse_enc(line)
            if enc:
                return enc
        return (0, 0)

    def zero_encoders(self) -> None:
        self._conn.send("EZ", read_ms=200)

    def send(self, message: str, read_ms: int = 500) -> dict[str, Any]:
        return self._conn.send(message, read_ms)

    @property
    def gripper_offset(self) -> float:
        return GRIPPER_OFFSET_CM

    @property
    def connection(self) -> SerialConnection:
        return self._conn


# Backward-compatibility alias
QBotPro = Cutebot
