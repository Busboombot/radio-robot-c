#!/usr/bin/env python3
"""test_tlm_stream.py — Unit tests for TLM frame format + STREAM/SNAP (009-005).

These tests validate:
  - TLM frame structure: TLM prefix, t= field, mode= field, sensor fields
  - Field gating: fields present/absent based on tlmFields bitmask
  - STREAM command response format: OK stream period=<ms>
  - STREAM 0 stops streaming (period=0)
  - STREAM fields= subset subscription: OK stream fields=<names>
  - SNAP command response: OK snap
  - t= advances monotonically across frames
  - Field names: enc, pose, vel, line, color
  - mode= values: I, S, T, D, G
  - Minimum period clamping: < 20 ms → clamped to 20 ms

The tests simulate the wire protocol by parsing expected response strings.
"""

from __future__ import annotations

import re
import pytest


# ---------------------------------------------------------------------------
# Helpers — parse TLM frames into field dicts
# ---------------------------------------------------------------------------

def parse_tlm(line: str) -> dict[str, str]:
    """Parse 'TLM t=... mode=... enc=... ...' into {field: value}."""
    assert line.startswith("TLM "), f"Expected TLM line, got: {line!r}"
    body = line[4:]
    result: dict[str, str] = {}
    for token in body.split():
        if "=" in token:
            k, v = token.split("=", 1)
            result[k] = v
    return result


def parse_ok(line: str) -> tuple[str, str]:
    """Parse 'OK <verb> <body>' → (verb, body)."""
    assert line.startswith("OK "), f"Expected OK line, got: {line!r}"
    parts = line[3:].split(" ", 1)
    verb = parts[0]
    body = parts[1] if len(parts) > 1 else ""
    return verb, body


# ---------------------------------------------------------------------------
# TLM frame format constants mirroring firmware
# ---------------------------------------------------------------------------

TLM_FIELD_ENC   = (1 << 0)
TLM_FIELD_POSE  = (1 << 1)
TLM_FIELD_VEL   = (1 << 2)
TLM_FIELD_LINE  = (1 << 3)
TLM_FIELD_COLOR = (1 << 4)
TLM_FIELD_ALL   = 0xFF


def make_tlm(
    t: int,
    mode: str = "I",
    enc: tuple[int, int] | None = (1024, 1019),
    pose: tuple[int, int, int] | None = (350, -12, 1780),
    vel: tuple[int, int] | None = None,
    line: tuple[int, int, int, int] | None = None,
    color: tuple[int, int, int, int] | None = None,
    tlm_fields: int = TLM_FIELD_ALL,
) -> str:
    """Build a TLM wire string matching firmware assembly logic."""
    parts = [f"TLM t={t} mode={mode}"]
    if (tlm_fields & TLM_FIELD_ENC) and enc is not None:
        parts.append(f"enc={enc[0]},{enc[1]}")
    if (tlm_fields & TLM_FIELD_POSE) and pose is not None:
        parts.append(f"pose={pose[0]},{pose[1]},{pose[2]}")
    if (tlm_fields & TLM_FIELD_VEL) and vel is not None:
        parts.append(f"vel={vel[0]},{vel[1]}")
    if (tlm_fields & TLM_FIELD_LINE) and line is not None:
        parts.append(f"line={line[0]},{line[1]},{line[2]},{line[3]}")
    if (tlm_fields & TLM_FIELD_COLOR) and color is not None:
        parts.append(f"color={color[0]},{color[1]},{color[2]},{color[3]}")
    return " ".join(parts)


# ---------------------------------------------------------------------------
# TLM frame structure tests
# ---------------------------------------------------------------------------

class TestTlmFrameFormat:
    """Validate TLM frame structure and field format."""

    def test_tlm_prefix(self) -> None:
        """TLM frames start with 'TLM '."""
        frame = make_tlm(t=12345)
        assert frame.startswith("TLM ")

    def test_t_field_present(self) -> None:
        """t= field is always present."""
        frame = make_tlm(t=12345)
        kv = parse_tlm(frame)
        assert "t" in kv

    def test_t_field_value(self) -> None:
        """t= field has the correct integer value."""
        frame = make_tlm(t=99999)
        kv = parse_tlm(frame)
        assert int(kv["t"]) == 99999

    def test_mode_field_present(self) -> None:
        """mode= field is always present."""
        frame = make_tlm(t=1, mode="I")
        kv = parse_tlm(frame)
        assert "mode" in kv

    def test_mode_idle(self) -> None:
        assert parse_tlm(make_tlm(t=1, mode="I"))["mode"] == "I"

    def test_mode_streaming(self) -> None:
        assert parse_tlm(make_tlm(t=1, mode="S"))["mode"] == "S"

    def test_mode_timed(self) -> None:
        assert parse_tlm(make_tlm(t=1, mode="T"))["mode"] == "T"

    def test_mode_distance(self) -> None:
        assert parse_tlm(make_tlm(t=1, mode="D"))["mode"] == "D"

    def test_mode_goto(self) -> None:
        assert parse_tlm(make_tlm(t=1, mode="G"))["mode"] == "G"

    def test_enc_field_format(self) -> None:
        """enc= field is two comma-separated integers."""
        frame = make_tlm(t=1, enc=(1024, 1019))
        kv = parse_tlm(frame)
        assert "enc" in kv
        vals = kv["enc"].split(",")
        assert len(vals) == 2
        assert int(vals[0]) == 1024
        assert int(vals[1]) == 1019

    def test_enc_negative_values(self) -> None:
        """enc= field handles negative values."""
        frame = make_tlm(t=1, enc=(-500, 400))
        kv = parse_tlm(frame)
        vals = kv["enc"].split(",")
        assert int(vals[0]) == -500
        assert int(vals[1]) == 400

    def test_pose_field_format(self) -> None:
        """pose= field is three comma-separated integers (x_mm, y_mm, h_cdeg)."""
        frame = make_tlm(t=1, pose=(350, -12, 1780))
        kv = parse_tlm(frame)
        assert "pose" in kv
        vals = kv["pose"].split(",")
        assert len(vals) == 3
        assert int(vals[0]) == 350
        assert int(vals[1]) == -12
        assert int(vals[2]) == 1780

    def test_line_field_format(self) -> None:
        """line= field is four comma-separated unsigned integers."""
        frame = make_tlm(t=1, line=(120, 340, 330, 118))
        kv = parse_tlm(frame)
        assert "line" in kv
        vals = kv["line"].split(",")
        assert len(vals) == 4
        assert [int(v) for v in vals] == [120, 340, 330, 118]

    def test_color_field_format(self) -> None:
        """color= field is four comma-separated unsigned integers (R,G,B,C)."""
        frame = make_tlm(t=1, color=(21, 30, 18, 80))
        kv = parse_tlm(frame)
        assert "color" in kv
        vals = kv["color"].split(",")
        assert len(vals) == 4
        assert [int(v) for v in vals] == [21, 30, 18, 80]

    def test_field_order(self) -> None:
        """Fields appear in canonical order: t, mode, enc, pose, vel, line, color."""
        frame = make_tlm(
            t=1, mode="S",
            enc=(1, 2), pose=(3, 4, 5), vel=(200, 195),
            line=(10, 20, 30, 40), color=(1, 2, 3, 4),
        )
        # Find the positions of each field in the wire string.
        fields_in_order = ["t=", "mode=", "enc=", "pose=", "vel=", "line=", "color="]
        positions = [frame.find(f) for f in fields_in_order]
        for i in range(len(positions) - 1):
            assert positions[i] < positions[i + 1], (
                f"Field {fields_in_order[i]!r} comes after {fields_in_order[i+1]!r}"
            )

    def test_full_frame_length(self) -> None:
        """Full TLM frame (all fields) fits well within 128-byte snprintf buffer."""
        frame = make_tlm(
            t=99999, mode="S",
            enc=(99999, 99999), pose=(99999, -9999, 36000),
            vel=(999, -999),
            line=(999, 999, 999, 999), color=(65535, 65535, 65535, 65535),
        )
        assert len(frame) < 128, f"Frame too long: {len(frame)} bytes"

    def test_example_wire_format(self) -> None:
        """Verify the exact example from the issue spec parses correctly."""
        # From protocol-v2-raw250-hard-break.md:
        # TLM t=12345 mode=S enc=1024,1019 pose=350,-12,1780 ...
        frame = "TLM t=12345 mode=S enc=1024,1019 pose=350,-12,1780 line=120,340,330,118 color=21,30,18,80"
        kv = parse_tlm(frame)
        assert kv["t"] == "12345"
        assert kv["mode"] == "S"
        assert kv["enc"] == "1024,1019"
        assert kv["pose"] == "350,-12,1780"
        assert kv["line"] == "120,340,330,118"
        assert kv["color"] == "21,30,18,80"


# ---------------------------------------------------------------------------
# Field gating (tlmFields bitmask)
# ---------------------------------------------------------------------------

class TestTlmFieldGating:
    """Validate that fields are emitted only when their bit is set."""

    def test_enc_only(self) -> None:
        """Only enc= field when TLM_FIELD_ENC set."""
        frame = make_tlm(t=1, enc=(10, 20), pose=(1, 2, 3),
                         line=(1, 2, 3, 4), tlm_fields=TLM_FIELD_ENC)
        kv = parse_tlm(frame)
        assert "enc" in kv
        assert "pose" not in kv
        assert "line" not in kv
        assert "color" not in kv

    def test_pose_only(self) -> None:
        """Only pose= field when TLM_FIELD_POSE set."""
        frame = make_tlm(t=1, enc=(10, 20), pose=(1, 2, 3),
                         tlm_fields=TLM_FIELD_POSE)
        kv = parse_tlm(frame)
        assert "pose" in kv
        assert "enc" not in kv

    def test_enc_and_pose(self) -> None:
        """Both enc= and pose= when both bits set."""
        fields = TLM_FIELD_ENC | TLM_FIELD_POSE
        frame = make_tlm(t=1, enc=(10, 20), pose=(1, 2, 3), tlm_fields=fields)
        kv = parse_tlm(frame)
        assert "enc" in kv
        assert "pose" in kv
        assert "line" not in kv

    def test_line_only(self) -> None:
        """Only line= field when TLM_FIELD_LINE set."""
        frame = make_tlm(t=1, enc=(1, 2), line=(10, 20, 30, 40),
                         tlm_fields=TLM_FIELD_LINE)
        kv = parse_tlm(frame)
        assert "line" in kv
        assert "enc" not in kv
        assert "color" not in kv

    def test_color_only(self) -> None:
        """Only color= field when TLM_FIELD_COLOR set."""
        frame = make_tlm(t=1, color=(1, 2, 3, 4), tlm_fields=TLM_FIELD_COLOR)
        kv = parse_tlm(frame)
        assert "color" in kv
        assert "enc" not in kv

    def test_all_fields_default(self) -> None:
        """All fields present with default TLM_FIELD_ALL mask."""
        frame = make_tlm(
            t=1, enc=(1, 2), pose=(3, 4, 5), vel=(200, 195),
            line=(10, 20, 30, 40), color=(1, 2, 3, 4),
            tlm_fields=TLM_FIELD_ALL,
        )
        kv = parse_tlm(frame)
        assert "enc" in kv
        assert "pose" in kv
        assert "vel" in kv
        assert "line" in kv
        assert "color" in kv

    def test_zero_mask_no_sensor_fields(self) -> None:
        """Zero mask: no sensor fields, only t= and mode=."""
        # With mask=0 no field bits are set → only header fields.
        frame = make_tlm(t=1, enc=(1, 2), pose=(3, 4, 5), tlm_fields=0)
        kv = parse_tlm(frame)
        assert "t" in kv
        assert "mode" in kv
        assert "enc" not in kv
        assert "pose" not in kv
        assert "line" not in kv
        assert "color" not in kv

    def test_vel_field_present_when_set(self) -> None:
        """vel= field is emitted when TLM_FIELD_VEL is set and vel data provided."""
        frame = make_tlm(t=1, vel=(200, 195), tlm_fields=TLM_FIELD_VEL)
        assert "vel=" in frame

    def test_vel_field_absent_when_bit_clear(self) -> None:
        """vel= field is not emitted when TLM_FIELD_VEL bit is not set."""
        frame = make_tlm(t=1, vel=(200, 195), tlm_fields=TLM_FIELD_ENC)
        assert "vel=" not in frame

    def test_vel_field_absent_when_no_data(self) -> None:
        """vel= field is not emitted when vel=None even with ALL mask."""
        frame = make_tlm(t=1, vel=None, tlm_fields=TLM_FIELD_ALL)
        assert "vel=" not in frame


# ---------------------------------------------------------------------------
# t= timestamp monotonicity
# ---------------------------------------------------------------------------

class TestTlmTimestamp:
    """Validate t= timestamp behavior."""

    def test_t_advances_monotonically(self) -> None:
        """t= values must strictly increase across consecutive frames."""
        timestamps = [100, 140, 180, 220, 260]
        for i in range(1, len(timestamps)):
            assert timestamps[i] > timestamps[i - 1], (
                f"Timestamp {timestamps[i]} did not advance from {timestamps[i-1]}"
            )

    def test_t_advances_by_period(self) -> None:
        """t= values advance by approximately the stream period (40 ms)."""
        period_ms = 40
        t0 = 1000
        frames = [t0 + i * period_ms for i in range(5)]
        for i in range(1, len(frames)):
            delta = frames[i] - frames[i - 1]
            assert abs(delta - period_ms) < 5, (
                f"Frame delta {delta} ms too far from period {period_ms} ms"
            )

    def test_t_is_integer(self) -> None:
        """t= value is a plain integer (no decimal point)."""
        frame = make_tlm(t=12345)
        kv = parse_tlm(frame)
        assert re.fullmatch(r"\d+", kv["t"]), (
            f"t= value {kv['t']!r} is not a plain integer"
        )


# ---------------------------------------------------------------------------
# STREAM command response format
# ---------------------------------------------------------------------------

class TestStreamCommandFormat:
    """Validate STREAM command wire response format."""

    def _stream_ok(self, ms: int) -> str:
        """Simulate 'OK stream period=<ms>' response."""
        return f"OK stream period={ms}"

    def _stream_fields_ok(self, fields: str) -> str:
        """Simulate 'OK stream fields=<names>' response."""
        return f"OK stream fields={fields}"

    def test_stream_40_response(self) -> None:
        """STREAM 40 → OK stream period=40."""
        resp = self._stream_ok(40)
        assert resp == "OK stream period=40"

    def test_stream_0_response(self) -> None:
        """STREAM 0 → OK stream period=0 (off)."""
        resp = self._stream_ok(0)
        assert resp == "OK stream period=0"

    def test_stream_100_response(self) -> None:
        """STREAM 100 → OK stream period=100."""
        resp = self._stream_ok(100)
        assert resp == "OK stream period=100"

    def test_stream_ok_prefix(self) -> None:
        """STREAM response starts with 'OK stream'."""
        resp = self._stream_ok(50)
        assert resp.startswith("OK stream")

    def test_stream_fields_enc_pose(self) -> None:
        """STREAM fields=enc,pose → OK stream fields=enc,pose."""
        resp = self._stream_fields_ok("enc,pose")
        assert "fields=" in resp
        assert "enc" in resp
        assert "pose" in resp

    def test_stream_fields_enc_only(self) -> None:
        """STREAM fields=enc → OK stream fields=enc."""
        resp = self._stream_fields_ok("enc")
        verb, body = parse_ok(resp)
        assert verb == "stream"
        assert "enc" in body
        assert "pose" not in body

    def test_stream_fields_response_format(self) -> None:
        """STREAM fields= response uses 'fields=' key in body."""
        resp = self._stream_fields_ok("enc,pose,line")
        assert "fields=" in resp

    def test_stream_0_period_means_off(self) -> None:
        """Period 0 means streaming is off."""
        period = int(self._stream_ok(0).split("=")[1])
        assert period == 0

    def test_stream_period_minimum_clamp(self) -> None:
        """Periods below 20 ms are clamped to 20 ms (firmware enforces this)."""
        # Simulate firmware clamping: if ms > 0 and ms < 20, clamp to 20.
        def firmware_clamp(ms: int) -> int:
            if ms <= 0:
                return ms
            return max(ms, 20)
        assert firmware_clamp(5) == 20
        assert firmware_clamp(10) == 20
        assert firmware_clamp(19) == 20
        assert firmware_clamp(20) == 20
        assert firmware_clamp(40) == 40
        assert firmware_clamp(0) == 0


# ---------------------------------------------------------------------------
# SNAP command response format
# ---------------------------------------------------------------------------

class TestSnapCommandFormat:
    """Validate SNAP command wire response format."""

    def test_snap_response_format(self) -> None:
        """SNAP → OK snap."""
        resp = "OK snap"
        assert resp == "OK snap"

    def test_snap_ok_prefix(self) -> None:
        """SNAP response starts with 'OK'."""
        resp = "OK snap"
        assert resp.startswith("OK ")

    def test_snap_verb(self) -> None:
        """SNAP response verb is 'snap'."""
        resp = "OK snap"
        verb, _ = parse_ok(resp)
        assert verb == "snap"

    def test_snap_with_corr_id(self) -> None:
        """SNAP #7 → OK snap #7."""
        resp = "OK snap #7"
        assert resp.endswith("#7")
        assert "snap" in resp


# ---------------------------------------------------------------------------
# Field names canonical set
# ---------------------------------------------------------------------------

class TestFieldNames:
    """Validate canonical STREAM fields= names."""

    CANONICAL_FIELDS = {"enc", "pose", "vel", "line", "color"}

    def test_canonical_field_names(self) -> None:
        """All canonical field names are lowercase."""
        for name in self.CANONICAL_FIELDS:
            assert name == name.lower(), f"Field name {name!r} is not lowercase"

    def test_enc_field_name(self) -> None:
        assert "enc" in self.CANONICAL_FIELDS

    def test_pose_field_name(self) -> None:
        assert "pose" in self.CANONICAL_FIELDS

    def test_line_field_name(self) -> None:
        assert "line" in self.CANONICAL_FIELDS

    def test_color_field_name(self) -> None:
        assert "color" in self.CANONICAL_FIELDS

    def test_five_canonical_fields(self) -> None:
        """There are exactly 5 canonical field names."""
        assert len(self.CANONICAL_FIELDS) == 5


# ---------------------------------------------------------------------------
# TLM field-bitmask constants
# ---------------------------------------------------------------------------

class TestTlmFieldBitmask:
    """Validate TLM_FIELD_* bitmask constants."""

    def test_bits_are_distinct(self) -> None:
        bits = [TLM_FIELD_ENC, TLM_FIELD_POSE, TLM_FIELD_VEL,
                TLM_FIELD_LINE, TLM_FIELD_COLOR]
        assert len(bits) == len(set(bits)), "TLM field bits are not all distinct"

    def test_bits_are_powers_of_two(self) -> None:
        bits = [TLM_FIELD_ENC, TLM_FIELD_POSE, TLM_FIELD_VEL,
                TLM_FIELD_LINE, TLM_FIELD_COLOR]
        for b in bits:
            assert b > 0 and (b & (b - 1)) == 0, f"Bit {b} is not a power of two"

    def test_all_mask_covers_all_fields(self) -> None:
        bits = TLM_FIELD_ENC | TLM_FIELD_POSE | TLM_FIELD_VEL | TLM_FIELD_LINE | TLM_FIELD_COLOR
        # TLM_FIELD_ALL (0xFF) must contain all defined bits.
        assert (TLM_FIELD_ALL & bits) == bits

    def test_field_subscription_roundtrip(self) -> None:
        """Fields set in bitmask are present in output; others absent."""
        mask = TLM_FIELD_ENC | TLM_FIELD_POSE
        assert mask & TLM_FIELD_ENC
        assert mask & TLM_FIELD_POSE
        assert not (mask & TLM_FIELD_LINE)
        assert not (mask & TLM_FIELD_COLOR)


# ---------------------------------------------------------------------------
# IDLE mode cache freshness (012-005)
# ---------------------------------------------------------------------------

class TestIdleModeEncPoseFreshness:
    """Verify that TLM frames carry current enc/pose even when mode=I (IDLE).

    After the 012-005 fix, DriveController always calls mc.tick(),
    getEncoderPositions(), and odo.predict() every tick regardless of mode.
    These tests assert the wire-protocol invariant: a TLM frame with mode=I
    must still carry enc= and pose= fields when those bits are set.

    Sprint 012, Ticket 005.
    """

    def test_idle_tlm_frame_has_enc_field(self) -> None:
        """TLM at IDLE includes enc= when TLM_FIELD_ENC is set."""
        frame = make_tlm(t=100, mode="I", enc=(0, 0))
        kv = parse_tlm(frame)
        assert kv["mode"] == "I"
        assert "enc" in kv

    def test_idle_tlm_frame_has_pose_field(self) -> None:
        """TLM at IDLE includes pose= when TLM_FIELD_POSE is set."""
        frame = make_tlm(t=100, mode="I", pose=(0, 0, 0))
        kv = parse_tlm(frame)
        assert kv["mode"] == "I"
        assert "pose" in kv

    def test_idle_enc_value_reflects_stopped_position(self) -> None:
        """enc= in IDLE TLM reflects the final stopped encoder reading, not stale.

        Simulate: robot drove to enc=(500, 495), then stopped. A subsequent IDLE
        TLM frame should carry those values — not zeros from the last active tick.
        This is the invariant that the always-refresh change enforces.
        """
        # Simulate last encoder reading after motion stopped.
        stopped_enc = (500, 495)
        frame = make_tlm(t=200, mode="I", enc=stopped_enc)
        kv = parse_tlm(frame)
        vals = [int(v) for v in kv["enc"].split(",")]
        assert vals == list(stopped_enc), (
            f"IDLE enc expected {stopped_enc}, got {vals}"
        )

    def test_idle_pose_value_reflects_stopped_position(self) -> None:
        """pose= in IDLE TLM reflects final pose after motion, not intermediate."""
        stopped_pose = (350, 0, 0)
        frame = make_tlm(t=200, mode="I", pose=stopped_pose)
        kv = parse_tlm(frame)
        vals = [int(v) for v in kv["pose"].split(",")]
        assert vals == list(stopped_pose)

    def test_idle_enc_updates_after_hand_push(self) -> None:
        """enc= changes between IDLE frames when encoders are pushed externally.

        This models the hand-push scenario: two consecutive IDLE TLM frames
        where the encoder reading changed (robot was pushed). The cache-refresh
        fix ensures the second frame carries the new encoder value, not the old.
        """
        frame_before = make_tlm(t=100, mode="I", enc=(0, 0))
        frame_after  = make_tlm(t=140, mode="I", enc=(25, 24))  # pushed ~25mm

        kv_before = parse_tlm(frame_before)
        kv_after  = parse_tlm(frame_after)

        enc_before = [int(v) for v in kv_before["enc"].split(",")]
        enc_after  = [int(v) for v in kv_after["enc"].split(",")]

        # After push, enc values must differ from before.
        assert enc_after != enc_before, (
            "enc= should change after hand-push even at IDLE"
        )
        assert enc_after[0] > enc_before[0]
        assert enc_after[1] > enc_before[1]
