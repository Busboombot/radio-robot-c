"""Unit tests for the HELLO-classify / !GO relay handshake (sprint 036-007).

All tests mock the ``serial.Serial`` boundary — no hardware required.

Scenarios covered:
  - Relay path (RADIOBRIDGE): DTR asserted, HELLO-until-banner, exact
    !ECHO OFF → !MODE RAW250 → !GO sequence, post-!GO sends are plain,
    # lines ignored by reader, corr-id / keepalive work in plain mode.
  - Direct path (NEZHA2): banner → no !GO, plain commands, connect succeeds.
  - Regression path: banner present but reader would previously drop it;
    verify the new classify captures it before the reader starts.
  - Unknown/timeout: classify timeout falls back to direct mode gracefully.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Any
from unittest.mock import MagicMock, patch, call

import pytest

# ---------------------------------------------------------------------------
# Helpers — fake serial device
# ---------------------------------------------------------------------------

class _FakeSerial:
    """Fake pyserial Serial object for handshake tests.

    Simulates an input byte stream via a queue.  The ``readline()`` method
    returns lines injected via ``inject()``.  Written bytes are recorded in
    ``written`` for assertion.
    """

    def __init__(self):
        self._q: queue.Queue[bytes] = queue.Queue()
        self.written: list[bytes] = []
        self.is_open: bool = False
        self.port: str = "/dev/fake"
        # pyserial attributes the code touches:
        self.dtr: bool | None = None  # will be set if caller sets it
        self.rts: bool | None = None
        self.timeout: float = 0.12
        self._open_called: bool = False
        self._dtr_at_open: bool | None = None  # capture DTR value at open()

    # --- pyserial interface --------------------------------------------------

    def open(self):
        self._open_called = True
        self._dtr_at_open = self.dtr
        self.is_open = True

    def close(self):
        self.is_open = False

    def write(self, data: bytes) -> int:
        self.written.append(data)
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        while not self._q.empty():
            try:
                self._q.get_nowait()
            except queue.Empty:
                break

    def readline(self) -> bytes:
        try:
            return self._q.get(timeout=0.05)
        except queue.Empty:
            return b""

    def fileno(self):
        raise OSError("no fd")  # so _disable_hupcl no-ops

    # --- test helpers --------------------------------------------------------

    def inject(self, line: str) -> None:
        """Queue a response line (newline appended automatically)."""
        self._q.put((line + "\n").encode("utf-8"))

    def inject_bytes(self, data: bytes) -> None:
        """Queue raw bytes (no newline added)."""
        self._q.put(data)

    def written_text(self) -> list[str]:
        """Return written bytes decoded as stripped text lines."""
        out = []
        for b in self.written:
            for part in b.decode("utf-8", "ignore").splitlines():
                s = part.strip()
                if s:
                    out.append(s)
        return out


# ---------------------------------------------------------------------------
# Helpers — patch serial.Serial
# ---------------------------------------------------------------------------

def _patch_serial(fake: _FakeSerial):
    """Context-manager patch: ``serial.Serial(...)`` returns ``fake``."""
    return patch("robot_radio.io.serial_conn.serial.Serial", return_value=fake)


def _make_conn(port: str = "/dev/fake"):
    """Create a SerialConnection with the given port (mode=None → auto)."""
    from robot_radio.io.serial_conn import SerialConnection
    return SerialConnection(port=port)


# ---------------------------------------------------------------------------
# Relay handshake scenario
# ---------------------------------------------------------------------------

class TestRelayHandshake:
    """RADIOBRIDGE relay: DTR asserted, !ECHO OFF → !MODE RAW250 → !GO order."""

    def _build_fake_relay(self) -> _FakeSerial:
        """Return a fake serial that behaves like the RADIOBRIDGE relay."""
        fake = _FakeSerial()

        def _respond():
            # Give the code a moment to open and send HELLO.
            time.sleep(0.05)
            # Respond to HELLO with DEVICE: banner.
            fake.inject("DEVICE:RADIOBRIDGE:relay:gozop:00:11:22:33")
            # Respond to ? query.
            time.sleep(0.05)
            fake.inject("# channel: 0 group: 10 mode: RAW250 power: 7")
            # Respond to !ECHO OFF.
            time.sleep(0.02)
            fake.inject("# echo: OFF")
            # Respond to !MODE RAW250.
            time.sleep(0.02)
            fake.inject("# mode: RAW250")
            # Respond to !GO.
            time.sleep(0.02)
            fake.inject("# entering data plane")
            # Respond to PING readiness poll (plain).
            time.sleep(0.05)
            fake.inject("OK pong t=12345 #1")

        t = threading.Thread(target=_respond, daemon=True)
        t.start()
        return fake

    def test_dtr_not_forced_false(self):
        """DTR must NOT be forced False — the code must not set dtr=False."""
        fake = self._build_fake_relay()
        conn = _make_conn()

        with _patch_serial(fake):
            result = conn.connect()
            conn.disconnect()

        # The old bug: serial_conn.py explicitly set self._ser.dtr = False.
        # Verify the code did NOT write dtr=False onto the fake object BEFORE
        # open().  We check that dtr at open-time was not False (None = left
        # alone by pyserial = asserted by default, or True = explicitly set).
        assert fake._dtr_at_open is not False, (
            f"DTR was forced False before open() — must not happen; "
            f"dtr_at_open={fake._dtr_at_open!r}"
        )

    def test_hello_sent_before_banner(self):
        """HELLO is written to the port during classify."""
        fake = self._build_fake_relay()
        conn = _make_conn()

        with _patch_serial(fake):
            conn.connect()
            conn.disconnect()

        sent = fake.written_text()
        assert "HELLO" in sent, f"HELLO not found in writes: {sent}"

    def test_handshake_sequence_order(self):
        """Exact relay command sequence: ? → !ECHO OFF → !MODE RAW250 → !GO."""
        fake = self._build_fake_relay()
        conn = _make_conn()

        with _patch_serial(fake):
            conn.connect()
            conn.disconnect()

        sent = fake.written_text()
        # Find positions of each command.
        def _pos(cmd):
            for i, s in enumerate(sent):
                if cmd in s:
                    return i
            return -1

        pos_q    = _pos("?")
        pos_echo = _pos("!ECHO OFF")
        pos_mode = _pos("!MODE RAW250")
        pos_go   = _pos("!GO")

        assert pos_q    >= 0, f"? query not sent; writes={sent}"
        assert pos_echo >= 0, f"!ECHO OFF not sent; writes={sent}"
        assert pos_mode >= 0, f"!MODE RAW250 not sent; writes={sent}"
        assert pos_go   >= 0, f"!GO not sent; writes={sent}"

        assert pos_q < pos_echo, "? must come before !ECHO OFF"
        assert pos_echo < pos_mode, "!ECHO OFF must come before !MODE RAW250"
        assert pos_mode < pos_go, "!MODE RAW250 must come before !GO"

    def test_post_go_sends_are_plain(self):
        """send() after !GO must NOT use the > prefix."""
        fake = self._build_fake_relay()
        conn = _make_conn()

        with _patch_serial(fake):
            conn.connect()
            # Inject a reply for the subsequent send().
            fake.inject("OK pong #2")
            conn.send("PING", read_ms=200)
            conn.disconnect()

        sent = fake.written_text()
        # Find the PING after !GO (i.e. after connect() returns).
        # All the handshake commands are HELLO, ?, !ECHO OFF, !MODE RAW250,
        # !GO, PING (readiness poll) — then the explicit PING from send().
        # None of them should start with ">".
        relay_prefixed = [s for s in sent if s.startswith(">")]
        assert not relay_prefixed, (
            f"Found >-prefixed commands in sent traffic (relay prefix not expected): "
            f"{relay_prefixed}"
        )

    def test_mode_is_direct_after_handshake(self):
        """After !GO the relay is a transparent pipe; mode must be 'direct'."""
        fake = self._build_fake_relay()
        conn = _make_conn()

        with _patch_serial(fake):
            result = conn.connect()
            conn.disconnect()

        assert conn.mode == "direct", (
            f"Expected mode='direct' after !GO handshake, got {conn.mode!r}"
        )

    def test_hash_lines_ignored_by_reader(self):
        """Lines starting with # must be silently dropped by the reader loop."""
        fake = self._build_fake_relay()
        conn = _make_conn()

        with _patch_serial(fake):
            conn.connect()
            # Inject relay comment lines into the reader's stream.
            fake.inject("# channel: 0 group: 10 mode: RAW250 power: 7")
            fake.inject("# DBG some debug line")
            # Now inject a real reply.
            fake.inject("OK pong t=99 #2")

            # send() should get only the OK pong, not the # lines.
            fake.inject("OK pong t=99 #3")
            result = conn.send("PING", read_ms=300)
            conn.disconnect()

        responses = result.get("responses", [])
        for line in responses:
            assert not line.startswith("#"), (
                f"Reader delivered a # comment line to send(): {line!r}"
            )

    def test_keepalive_is_plain(self):
        """The keepalive '+' must be sent plain (no > prefix)."""
        fake = self._build_fake_relay()
        conn = _make_conn()

        with _patch_serial(fake):
            conn.connect()
            # Let the keepalive thread run for a bit.
            time.sleep(0.35)
            conn.disconnect()

        sent = fake.written_text()
        ka_sends = [s for s in sent if s == "+"]
        assert ka_sends, "No '+' keepalive seen in writes"
        relay_ka = [s for s in sent if s == ">+"]
        assert not relay_ka, f"Found relay-prefixed keepalive '>+': {relay_ka}"

    def test_announcement_in_result(self):
        """connect() result must include the parsed DEVICE: announcement."""
        fake = self._build_fake_relay()
        conn = _make_conn()

        with _patch_serial(fake):
            result = conn.connect()
            conn.disconnect()

        ann = result.get("announcement")
        assert ann is not None, f"No announcement in result: {result}"
        assert ann.get("role") == "RADIOBRIDGE", f"Role mismatch: {ann}"
        assert ann.get("device_name") == "gozop", f"device_name mismatch: {ann}"


# ---------------------------------------------------------------------------
# Direct robot scenario (NEZHA2)
# ---------------------------------------------------------------------------

class TestDirectRobotHandshake:
    """NEZHA2 direct USB: banner → no !GO, plain commands, connect succeeds."""

    def _build_fake_robot(self) -> _FakeSerial:
        """Return a fake serial that behaves like a direct NEZHA2 connection."""
        fake = _FakeSerial()

        def _respond():
            time.sleep(0.05)
            # Respond to HELLO with robot DEVICE: banner.
            fake.inject("DEVICE:NEZHA2:robot:tovez:AB:CD:EF:01")
            # Respond to PING readiness poll.
            time.sleep(0.05)
            fake.inject("OK pong t=11 #1")

        t = threading.Thread(target=_respond, daemon=True)
        t.start()
        return fake

    def test_no_go_sent_for_direct_robot(self):
        """!GO must NOT be sent when the device is NEZHA2 (direct)."""
        fake = self._build_fake_robot()
        conn = _make_conn()

        with _patch_serial(fake):
            conn.connect()
            conn.disconnect()

        sent = fake.written_text()
        assert "!GO" not in sent, (
            f"!GO was sent for a direct NEZHA2 connection; writes={sent}"
        )

    def test_no_echo_off_sent_for_direct_robot(self):
        """Relay config commands must NOT be sent to a direct NEZHA2."""
        fake = self._build_fake_robot()
        conn = _make_conn()

        with _patch_serial(fake):
            conn.connect()
            conn.disconnect()

        sent = fake.written_text()
        relay_cmds = [s for s in sent if s.startswith("!") and s != "HELLO"]
        assert not relay_cmds, (
            f"Relay commands sent to direct NEZHA2: {relay_cmds}"
        )

    def test_plain_commands_for_direct(self):
        """After direct connect, send() issues plain commands."""
        fake = self._build_fake_robot()
        conn = _make_conn()

        with _patch_serial(fake):
            conn.connect()
            fake.inject("OK id model=Nezha2 #2")
            conn.send("ID", read_ms=200)
            conn.disconnect()

        sent = fake.written_text()
        relay_prefixed = [s for s in sent if s.startswith(">")]
        assert not relay_prefixed, (
            f">-prefixed command sent to direct robot: {relay_prefixed}"
        )

    def test_mode_is_direct_for_nezha2(self):
        """Mode must be 'direct' for a direct NEZHA2 connection."""
        fake = self._build_fake_robot()
        conn = _make_conn()

        with _patch_serial(fake):
            result = conn.connect()
            conn.disconnect()

        assert conn.mode == "direct", f"Expected direct mode, got {conn.mode!r}"

    def test_announcement_in_result(self):
        """connect() result includes NEZHA2 DEVICE: announcement."""
        fake = self._build_fake_robot()
        conn = _make_conn()

        with _patch_serial(fake):
            result = conn.connect()
            conn.disconnect()

        ann = result.get("announcement")
        assert ann is not None, f"No announcement in result: {result}"
        assert ann.get("role") == "NEZHA2", f"Role mismatch: {ann}"


# ---------------------------------------------------------------------------
# Regression: banner previously dropped by reader loop
# ---------------------------------------------------------------------------

class TestBannerNotDroppedByReader:
    """Regression: DEVICE: lines were silently dropped by the reader loop.

    The old code let the reader thread start before any HELLO classify, so
    DEVICE: lines from the connect() poll hit the reader which dropped them
    (they don't start with TLM/EVT/OK/ERR/CFG).  This led to 'No device
    found' even when the relay was present and responding.

    The new code runs _banner_classify() BEFORE starting the reader thread,
    so DEVICE: lines are consumed by the classify step directly.
    """

    def test_banner_captured_before_reader_starts(self):
        """DEVICE: line captured during classify, not silently dropped."""
        # Build a fake that emits a DEVICE: banner but then nothing else
        # (simulates the relay's boot banner path with no PING response).
        fake = _FakeSerial()

        def _respond():
            time.sleep(0.05)
            fake.inject("DEVICE:RADIOBRIDGE:relay:test:00:00")
            # Provide a response to ? query.
            time.sleep(0.05)
            fake.inject("# channel: 0 group: 10 mode: RAW250 power: 7")
            # !ECHO OFF ack.
            time.sleep(0.02)
            fake.inject("# echo: OFF")
            # !MODE RAW250 ack.
            time.sleep(0.02)
            fake.inject("# mode: RAW250")
            # !GO ack.
            time.sleep(0.02)
            fake.inject("# entering data plane")
            # PING poll.
            time.sleep(0.05)
            fake.inject("OK pong t=1 #1")

        t = threading.Thread(target=_respond, daemon=True)
        t.start()

        conn = _make_conn()
        with _patch_serial(fake):
            result = conn.connect()
            conn.disconnect()

        assert "error" not in result, f"connect() failed: {result}"
        ann = result.get("announcement")
        assert ann is not None, (
            "Regression: DEVICE: banner was not captured. "
            "This was the 'No device found' bug. "
            f"connect() result={result}"
        )
        assert ann.get("role") == "RADIOBRIDGE"

    def test_hash_lines_not_delivered_to_send(self):
        """Regression: # comment lines (relay status) must not reach send()."""
        fake = _FakeSerial()

        def _respond():
            time.sleep(0.05)
            fake.inject("DEVICE:RADIOBRIDGE:relay:r:0")
            time.sleep(0.05)
            fake.inject("# channel: 0 group: 10 mode: RAW250 power: 7")
            time.sleep(0.02)
            fake.inject("# echo: OFF")
            time.sleep(0.02)
            fake.inject("# mode: RAW250")
            time.sleep(0.02)
            fake.inject("# entering data plane")
            # Readiness poll reply.
            time.sleep(0.05)
            fake.inject("OK pong t=1 #1")

        t = threading.Thread(target=_respond, daemon=True)
        t.start()

        conn = _make_conn()
        with _patch_serial(fake):
            conn.connect()
            # Now inject a # comment line AND a real reply.
            # The # line should be dropped by the reader.
            time.sleep(0.05)
            fake.inject("# some relay status")
            fake.inject("OK pong t=2 #2")

            result = conn.send("PING", read_ms=300)
            conn.disconnect()

        responses = result.get("responses", [])
        hash_lines = [r for r in responses if r.startswith("#")]
        assert not hash_lines, (
            f"send() received # comment lines from reader: {hash_lines}"
        )


# ---------------------------------------------------------------------------
# Timeout / unknown device fallback
# ---------------------------------------------------------------------------

class TestClassifyTimeout:
    """When no DEVICE: banner arrives, classify falls back to 'direct'."""

    def test_timeout_falls_back_to_direct(self):
        """No banner within timeout → mode='direct', no error raised."""
        fake = _FakeSerial()

        def _respond():
            # Simulate a very slow device: respond to PING only, no DEVICE:.
            time.sleep(0.3)
            fake.inject("OK pong t=5 #1")

        t = threading.Thread(target=_respond, daemon=True)
        t.start()

        from robot_radio.io.serial_conn import SerialConnection
        conn = SerialConnection(port="/dev/fake")

        with _patch_serial(fake):
            # Use a very short classify timeout so the test is fast.
            with patch("robot_radio.io.serial_conn._HELLO_CLASSIFY_TIMEOUT_S", 0.15):
                result = conn.connect()
            conn.disconnect()

        # Should not have errored; mode falls back to direct.
        assert "error" not in result, f"Unexpected error: {result}"
        assert conn.mode == "direct"

    def test_no_go_on_timeout(self):
        """When no banner → no !GO sent (no !ECHO OFF etc.)."""
        fake = _FakeSerial()

        def _respond():
            time.sleep(0.3)
            fake.inject("OK pong t=5 #1")

        t = threading.Thread(target=_respond, daemon=True)
        t.start()

        from robot_radio.io.serial_conn import SerialConnection
        conn = SerialConnection(port="/dev/fake")

        with _patch_serial(fake):
            with patch("robot_radio.io.serial_conn._HELLO_CLASSIFY_TIMEOUT_S", 0.15):
                conn.connect()
            conn.disconnect()

        sent = fake.written_text()
        assert "!GO" not in sent, f"!GO sent despite no banner: {sent}"


# ---------------------------------------------------------------------------
# Corr-id and keepalive in transparent mode
# ---------------------------------------------------------------------------

class TestCorrIdAndKeepaliveRelay:
    """Corr-id suffix and keepalive work correctly in post-!GO plain mode."""

    def _build_and_connect(self):
        fake = _FakeSerial()

        def _respond():
            time.sleep(0.05)
            fake.inject("DEVICE:RADIOBRIDGE:relay:r:0")
            time.sleep(0.05)
            fake.inject("# channel: 0 group: 10 mode: RAW250 power: 7")
            time.sleep(0.02)
            fake.inject("# echo: OFF")
            time.sleep(0.02)
            fake.inject("# mode: RAW250")
            time.sleep(0.02)
            fake.inject("# entering data plane")
            time.sleep(0.05)
            fake.inject("OK pong t=1 #1")

        t = threading.Thread(target=_respond, daemon=True)
        t.start()

        from robot_radio.io.serial_conn import SerialConnection
        conn = SerialConnection(port="/dev/fake")
        with _patch_serial(fake):
            conn.connect()
        return conn, fake

    def test_corr_id_suffix_appended(self):
        """send() appends #<n> corr-id to plain commands."""
        conn, fake = self._build_and_connect()

        fake.inject("OK snap #2")
        with _patch_serial(fake):
            conn.send("SNAP", read_ms=200)
            conn.disconnect()

        sent = fake.written_text()
        snap_sends = [s for s in sent if "SNAP" in s]
        assert snap_sends, f"SNAP not found in writes: {sent}"
        for s in snap_sends:
            assert "#" in s, f"corr-id suffix missing from SNAP command: {s!r}"

    def test_keepalive_plain_plus(self):
        """Keepalive thread sends plain '+', never '>+'."""
        conn, fake = self._build_and_connect()

        with _patch_serial(fake):
            time.sleep(0.4)  # let keepalive tick a few times
            conn.disconnect()

        sent = fake.written_text()
        assert "+" in sent, "No keepalive '+' seen after relay connect"
        assert ">+" not in sent, "Relay-prefixed keepalive '>+' sent (old bug)"
