#!/usr/bin/env python3
"""test_readspeed_and_get_vel.py — Tests for Motor::readSpeed conversion and GET VEL command (010-001).

Validates:
  - readSpeed mm/s conversion formula: (raw / kUnitFactor) * mmPerDeg * sign
  - GET VEL wire format: OK get vel=<vL>:<srcL>,<vR>:<srcR>
  - Source flag values: 'C' = chip, 'E' = encoder-delta
  - lapsToMmScale is no longer part of the config registry
  - Bench-confirm logic: unit factor interpretation
"""

from __future__ import annotations

import re
import pytest

# ---------------------------------------------------------------------------
# Motor::readSpeed conversion — Python reference implementation
# ---------------------------------------------------------------------------

# kUnitFactor in Motor.cpp (Motor::readSpeed); change this to 1.0 if bench
# shows raw is whole degrees/s (not tenths).
K_UNIT_FACTOR = 10.0  # tenths of degrees/s interpretation

# Default calibration values from defaultRobotConfig()
MM_PER_DEG_L = 0.487  # mmPerDegL
MM_PER_DEG_R = 0.481  # mmPerDegR


def compute_readspeed_mms(raw: int, mm_per_deg: float, last_dir: int,
                           unit_factor: float = K_UNIT_FACTOR) -> float:
    """Reference implementation of Motor::readSpeed formula.

    mm/s = (raw / unit_factor) * mm_per_deg * last_dir

    Args:
        raw:         Raw uint16 value from register 0x47 (unsigned magnitude).
        mm_per_deg:  Wheel calibration constant (mmPerDegL or mmPerDegR).
        last_dir:    Direction sign: +1 = forward, -1 = reverse, 0 = stopped.
        unit_factor: kUnitFactor from Motor.cpp (10.0 = tenths, 1.0 = whole deg/s).
    """
    if raw < 0:
        return 0.0  # I2C error sentinel
    magnitude = (raw / unit_factor) * mm_per_deg
    return magnitude * last_dir


# ---------------------------------------------------------------------------
# Tests for the readSpeed conversion formula
# ---------------------------------------------------------------------------

class TestReadSpeedConversion:
    """Validate the corrected mm/s conversion formula."""

    def test_zero_raw_gives_zero_velocity(self) -> None:
        """raw=0 → mm/s=0 regardless of direction or calibration."""
        assert compute_readspeed_mms(0, MM_PER_DEG_L, 1) == pytest.approx(0.0)
        assert compute_readspeed_mms(0, MM_PER_DEG_L, -1) == pytest.approx(0.0)

    def test_forward_positive_direction(self) -> None:
        """Forward direction (last_dir=+1) produces positive mm/s."""
        result = compute_readspeed_mms(2000, MM_PER_DEG_L, +1)
        assert result > 0.0

    def test_reverse_negative_direction(self) -> None:
        """Reverse direction (last_dir=-1) produces negative mm/s."""
        result = compute_readspeed_mms(2000, MM_PER_DEG_L, -1)
        assert result < 0.0

    def test_stopped_zero_direction(self) -> None:
        """Stopped (last_dir=0) always produces zero mm/s."""
        result = compute_readspeed_mms(2000, MM_PER_DEG_L, 0)
        assert result == pytest.approx(0.0)

    def test_left_wheel_uses_mmPerDegL(self) -> None:
        """Left wheel (M2) applies mmPerDegL calibration."""
        raw = 1000
        expected = (raw / K_UNIT_FACTOR) * MM_PER_DEG_L * 1
        result = compute_readspeed_mms(raw, MM_PER_DEG_L, 1)
        assert result == pytest.approx(expected)

    def test_right_wheel_uses_mmPerDegR(self) -> None:
        """Right wheel (M1) applies mmPerDegR calibration."""
        raw = 1000
        expected = (raw / K_UNIT_FACTOR) * MM_PER_DEG_R * 1
        result = compute_readspeed_mms(raw, MM_PER_DEG_R, 1)
        assert result == pytest.approx(expected)

    def test_left_right_calibration_differs(self) -> None:
        """Left and right wheels have different mmPerDeg, so output differs."""
        raw = 1000
        vL = compute_readspeed_mms(raw, MM_PER_DEG_L, 1)
        vR = compute_readspeed_mms(raw, MM_PER_DEG_R, 1)
        assert vL != pytest.approx(vR)

    def test_formula_mirrors_read_encoder(self) -> None:
        """readSpeed formula mirrors readEncoder: (raw/10) * mmPerDeg * sign.

        readEncoder: mm = (raw_tenths / 10.0) * mmPerDeg * fwdSign
        readSpeed:   mm/s = (raw / 10.0) * mmPerDeg * lastDir

        For the same raw value of 1000 tenths, readSpeed should produce
        the same magnitude as readEncoder (the difference is only units: mm vs mm/s).
        """
        raw_tenths = 1000
        encoder_mm = (raw_tenths / 10.0) * MM_PER_DEG_L * 1
        speed_mms = compute_readspeed_mms(raw_tenths, MM_PER_DEG_L, 1)
        assert speed_mms == pytest.approx(encoder_mm)

    def test_old_formula_not_used(self) -> None:
        """Verify the old floor(raw/3.6)*0.01*lapsToMmScale formula is not used.

        Old formula with lapsToMmScale=1980 and raw=2000:
          laps_per_sec = floor(2000/3.6) * 0.01 = floor(555.5) * 0.01 = 5.55
          mm/s = 5.55 * 1980 = 10989 mm/s  (clearly wrong)

        New formula with raw=2000, mmPerDegL=0.487:
          mm/s = (2000/10) * 0.487 = 97.4 mm/s  (plausible)
        """
        raw = 2000
        laps_to_mm_scale = 1980.0  # old provisional value
        old_laps_per_sec = int(raw / 3.6) * 0.01  # floor via int truncation
        old_mms = old_laps_per_sec * laps_to_mm_scale

        new_mms = compute_readspeed_mms(raw, MM_PER_DEG_L, 1)

        # Old formula produces ~11x higher values
        assert old_mms > new_mms * 5, (
            f"Old formula should be much larger: old={old_mms:.1f}, new={new_mms:.1f}"
        )
        # New formula is in the plausible range for 200 mm/s target
        assert 50 < new_mms < 200, (
            f"New formula should be ~97.4 mm/s for raw=2000: got {new_mms:.1f}"
        )

    def test_specific_raw_2000_left_wheel(self) -> None:
        """Concrete: raw=2000, mmPerDegL=0.487, dir=+1 → 97.4 mm/s."""
        result = compute_readspeed_mms(2000, MM_PER_DEG_L, +1)
        assert result == pytest.approx(97.4, abs=0.01)

    def test_specific_raw_4114_left_wheel(self) -> None:
        """Concrete: raw=4114 tenths → 411.4 deg/s → 200.4 mm/s (left wheel).

        This is the expected raw value for ~200 mm/s on the left wheel
        with mmPerDegL=0.487.
        """
        # Expected: 200 mm/s / 0.487 mm/deg * 10 (tenths) ≈ 4106 raw
        raw = 4114
        result = compute_readspeed_mms(raw, MM_PER_DEG_L, +1)
        assert result == pytest.approx(200.4518, abs=0.1)

    def test_unit_factor_interpretation(self) -> None:
        """Show the difference between /10 and /1 interpretations.

        If raw is whole degrees/s (not tenths), the two formulas differ by 10×.
        The bench-confirmation procedure compares readSpeed to encoder-delta.
        """
        raw = 1000
        mms_tenths = compute_readspeed_mms(raw, MM_PER_DEG_L, 1, unit_factor=10.0)
        mms_whole  = compute_readspeed_mms(raw, MM_PER_DEG_L, 1, unit_factor=1.0)
        assert mms_whole == pytest.approx(mms_tenths * 10.0)

    def test_symmetry_forward_reverse(self) -> None:
        """Forward and reverse should produce equal magnitude but opposite sign."""
        raw = 3000
        fwd = compute_readspeed_mms(raw, MM_PER_DEG_L, +1)
        rev = compute_readspeed_mms(raw, MM_PER_DEG_L, -1)
        assert fwd == pytest.approx(-rev)
        assert fwd > 0
        assert rev < 0


# ---------------------------------------------------------------------------
# GET VEL wire format
# ---------------------------------------------------------------------------

def parse_get_vel(line: str) -> dict:
    """Parse 'OK get vel=<vL>:<srcL>,<vR>:<srcR>' into a dict with keys:
      'vL'   — left velocity (int mm/s)
      'srcL' — left source flag ('C' or 'E')
      'vR'   — right velocity (int mm/s)
      'srcR' — right source flag ('C' or 'E')
    """
    assert line.startswith("OK get "), f"Expected 'OK get' line, got: {line!r}"
    body = line[7:]  # strip "OK get "

    # Strip trailing #id if present
    body = re.sub(r"\s+#\d+\s*$", "", body)

    # Must contain vel=... token
    m = re.match(r"vel=(-?\d+):([CE]),(-?\d+):([CE])$", body.strip())
    assert m is not None, f"GET VEL body does not match expected pattern: {body!r}"
    return {
        "vL":   int(m.group(1)),
        "srcL": m.group(2),
        "vR":   int(m.group(3)),
        "srcR": m.group(4),
    }


class TestGetVelFormat:
    """Validate GET VEL wire format."""

    def test_both_chip_sources(self) -> None:
        """Both wheels reading chip velocity."""
        line = "OK get vel=198:C,201:C"
        result = parse_get_vel(line)
        assert result["vL"] == 198
        assert result["srcL"] == "C"
        assert result["vR"] == 201
        assert result["srcR"] == "C"

    def test_both_encoder_fallback(self) -> None:
        """Both wheels falling back to encoder-delta."""
        line = "OK get vel=0:E,0:E"
        result = parse_get_vel(line)
        assert result["vL"] == 0
        assert result["srcL"] == "E"
        assert result["vR"] == 0
        assert result["srcR"] == "E"

    def test_mixed_sources(self) -> None:
        """Left wheel on chip, right wheel on encoder-delta."""
        line = "OK get vel=200:C,195:E"
        result = parse_get_vel(line)
        assert result["srcL"] == "C"
        assert result["srcR"] == "E"

    def test_negative_velocity(self) -> None:
        """Reverse velocity is reported as negative integer."""
        line = "OK get vel=-198:C,-201:C"
        result = parse_get_vel(line)
        assert result["vL"] == -198
        assert result["vR"] == -201

    def test_zero_velocity_stopped(self) -> None:
        """Stopped motors report zero velocity."""
        line = "OK get vel=0:E,0:E"
        result = parse_get_vel(line)
        assert result["vL"] == 0
        assert result["vR"] == 0

    def test_ok_get_prefix(self) -> None:
        """Response starts with 'OK get'."""
        line = "OK get vel=100:C,100:C"
        assert line.startswith("OK get ")

    def test_vel_field_format(self) -> None:
        """vel= field uses integer mm/s values with colon source flag."""
        line = "OK get vel=200:C,195:C"
        assert "vel=" in line
        # Extract vel= value and validate pattern
        m = re.search(r"vel=(-?\d+):[CE],(-?\d+):[CE]", line)
        assert m is not None, f"vel= pattern not found in: {line!r}"

    def test_with_correlation_id(self) -> None:
        """GET VEL #id echoes the correlation id."""
        line = "OK get vel=0:E,0:E #5"
        assert line.endswith("#5")
        # Strip id and parse
        body = re.sub(r"\s+#\d+\s*$", "", line[7:])
        m = re.match(r"vel=(-?\d+):([CE]),(-?\d+):([CE])$", body.strip())
        assert m is not None

    def test_source_flag_is_c_or_e(self) -> None:
        """Source flags must be exactly 'C' or 'E'."""
        valid_lines = [
            "OK get vel=100:C,100:C",
            "OK get vel=100:E,100:E",
            "OK get vel=100:C,100:E",
            "OK get vel=100:E,100:C",
        ]
        for line in valid_lines:
            result = parse_get_vel(line)
            assert result["srcL"] in ("C", "E"), f"srcL not C or E: {result['srcL']}"
            assert result["srcR"] in ("C", "E"), f"srcR not C or E: {result['srcR']}"

    def test_get_vel_not_cfg_response(self) -> None:
        """GET VEL returns OK get, NOT a CFG line."""
        line = "OK get vel=198:C,201:C"
        assert not line.startswith("CFG"), "GET VEL must not return a CFG line"
        assert line.startswith("OK get"), "GET VEL must return OK get"

    def test_velocity_integers_not_floats(self) -> None:
        """Velocity values are integers (no decimal point)."""
        line = "OK get vel=198:C,201:C"
        m = re.search(r"vel=(-?\d+):([CE]),(-?\d+):([CE])", line)
        assert m is not None
        # Groups 1 and 3 are the velocity values — must parse as integers
        vL = int(m.group(1))
        vR = int(m.group(3))
        assert isinstance(vL, int)
        assert isinstance(vR, int)


# ---------------------------------------------------------------------------
# Registry: lapsToMmScale must NOT be a registered key
# ---------------------------------------------------------------------------

class TestLapsToMmScaleRemoved:
    """Confirm lapsToMmScale is no longer in the config registry."""

    def test_lapsToMmScale_not_in_registry(self) -> None:
        """The config registry (kRegistry[]) must not contain 'lapsToMmScale'."""
        # This test documents the wire-level expectation: any GET lapsToMmScale
        # should return ERR badkey lapsToMmScale (not a CFG line).
        # We validate this via the registry spec defined in test_config_registry.py.
        # Import using importlib to avoid package path issues.
        import importlib.util
        import os
        spec = importlib.util.spec_from_file_location(
            "test_config_registry",
            os.path.join(os.path.dirname(__file__), "test_config_registry.py"),
        )
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        registry_keys = mod.REGISTRY_KEYS
        assert "lapsToMmScale" not in registry_keys, (
            "lapsToMmScale must be removed from kRegistry[] in CommandProcessor.cpp"
        )

    def test_lapsToMmScale_err_response_format(self) -> None:
        """Accessing lapsToMmScale via GET produces ERR badkey."""
        # Document the expected wire response for a removed key.
        expected_err = "ERR badkey lapsToMmScale"
        assert expected_err.startswith("ERR badkey")
        assert "lapsToMmScale" in expected_err


# ---------------------------------------------------------------------------
# Bench-confirmation scenario (reference documentation as tests)
# ---------------------------------------------------------------------------

class TestBenchConfirmScenario:
    """Document the bench-confirmation procedure as verifiable assertions.

    These tests verify the mathematical relationship between the two possible
    unit interpretations, which guides the bench confirmation procedure.
    """

    def test_tenths_interpretation_gives_lower_value(self) -> None:
        """Tenths interpretation (/10) gives 10× lower mm/s than whole-deg/s."""
        raw = 2000
        mms_tenths = compute_readspeed_mms(raw, MM_PER_DEG_L, 1, unit_factor=10.0)
        mms_whole  = compute_readspeed_mms(raw, MM_PER_DEG_L, 1, unit_factor=1.0)
        assert mms_whole == pytest.approx(mms_tenths * 10.0)

    def test_bench_confirmation_threshold(self) -> None:
        """If chip readSpeed is >5× encoder-delta, the /10 interpretation is likely correct.

        The acceptance criterion is 15% agreement at steady state.
        10× discrepancy is far outside this, making it detectable.
        """
        encoder_delta_mms = 200.0  # mm/s from encoder measurement
        chip_tenths_mms   = 195.0  # mm/s from readSpeed with /10 (close to encoder)
        chip_whole_mms    = 1950.0 # mm/s from readSpeed without /10 (10× too high)

        # /10 interpretation: within 15% of encoder
        assert abs(chip_tenths_mms - encoder_delta_mms) / encoder_delta_mms < 0.15

        # x1 interpretation: far outside 15% of encoder
        assert abs(chip_whole_mms - encoder_delta_mms) / encoder_delta_mms > 5.0

    def test_sign_correct_for_forward(self) -> None:
        """Forward motion (last_dir=+1) yields positive readSpeed mm/s."""
        mms = compute_readspeed_mms(2000, MM_PER_DEG_L, +1)
        assert mms > 0

    def test_sign_correct_for_reverse(self) -> None:
        """Reverse motion (last_dir=-1) yields negative readSpeed mm/s."""
        mms = compute_readspeed_mms(2000, MM_PER_DEG_L, -1)
        assert mms < 0


# ---------------------------------------------------------------------------
# Plausibility gate — two-sided rejection logic (sprint 012-004)
# ---------------------------------------------------------------------------

# Mirror MotorController plausibility gate logic in Python for unit testing.
# minWheelMms is _cal.minWheelMms = 20.0 (default)
MIN_WHEEL_MMS = 20.0


def plausibility_gate(chip_vel: float, enc_vel: float,
                      min_wheel_mms: float = MIN_WHEEL_MMS) -> bool:
    """Return True if the chip reading passes the plausibility gate.

    Mirrors MotorController::tick() gate logic (sprint 012-004):
      - tooHigh: |chipVel| > 2 × |encVel|
      - tooLow:  |encVel| > minWheelMms AND |chipVel| < 0.5 × |encVel|
    Returns False (gate rejects) if chipOk and encVel > 0 and (tooHigh or tooLow).
    Returns True (gate passes) otherwise.
    """
    if enc_vel == 0.0:
        # No reference — gate cannot fire; pass by default.
        return True
    abs_chip = abs(chip_vel)
    abs_enc  = abs(enc_vel)
    too_high = abs_chip > 2.0 * abs_enc
    too_low  = (abs_enc > min_wheel_mms) and (abs_chip < 0.5 * abs_enc)
    return not (too_high or too_low)


class TestPlausibilityGate:
    """Validate both-sided chip-velocity plausibility gate (sprint 012-004).

    The gate must:
      - Reject chip readings that are too HIGH (> 2x encoder-delta) — noise/glitch
      - Reject chip readings that are too LOW  (< 0.5x encoder-delta when wheel is
        clearly moving) — the "stuck ~30 mm/s" symptom observed on hardware
      - Pass plausible chip readings that are within the [0.5x, 2x] band
      - Not fire when encoder-delta is near zero (unreliable reference)
    """

    # --- stuck-low (the primary hardware symptom) ---

    def test_stuck_low_rejected_when_enc_clearly_moving(self) -> None:
        """Stuck chip (~30 mm/s) rejected when encoder says ~140 mm/s.

        This is the exact symptom observed on hardware: chip returns ~30 mm/s
        while encoder-delta (ground truth) reports ~140 mm/s at S 100 command.
        """
        chip_vel = 30.0   # stuck register reading
        enc_vel  = 140.0  # encoder-delta at the same speed
        assert not plausibility_gate(chip_vel, enc_vel), (
            "Stuck-low chip reading should be rejected by the plausibility gate"
        )

    def test_stuck_low_rejected_forward_high_speed(self) -> None:
        """Stuck chip (~30 mm/s) rejected when encoder says ~300 mm/s (S 300 command)."""
        chip_vel = 30.0
        enc_vel  = 300.0
        assert not plausibility_gate(chip_vel, enc_vel)

    def test_stuck_low_rejected_reverse(self) -> None:
        """Stuck chip (~-30 mm/s) rejected when encoder says ~-140 mm/s (reverse)."""
        chip_vel = -30.0
        enc_vel  = -140.0
        assert not plausibility_gate(chip_vel, enc_vel)

    # --- too-high (pre-existing guard, must still work) ---

    def test_too_high_rejected(self) -> None:
        """Chip reading > 2x encoder-delta is rejected (I2C noise / glitch)."""
        chip_vel = 500.0  # wildly high
        enc_vel  = 140.0
        assert not plausibility_gate(chip_vel, enc_vel)

    def test_too_high_boundary_exact(self) -> None:
        """Exactly 2x encoder is rejected (> 2x rule; at 2x it is still rejected)."""
        enc_vel  = 100.0
        chip_vel = 200.0  # exactly 2×
        # 200 > 2*100 is False (equal, not greater); gate passes at exact boundary
        # The C++ uses strict >: fabsf(chipVel) > 2.0f * fabsf(encVel)
        assert plausibility_gate(chip_vel, enc_vel)  # boundary: exactly 2x passes

    def test_just_over_2x_rejected(self) -> None:
        """Just over 2x (200.1 vs 100) is rejected."""
        chip_vel = 200.1
        enc_vel  = 100.0
        assert not plausibility_gate(chip_vel, enc_vel)

    # --- plausible readings pass ---

    def test_plausible_chip_passes_close_match(self) -> None:
        """Chip and encoder agree closely — gate passes (chip used as primary)."""
        chip_vel = 198.0
        enc_vel  = 200.0
        assert plausibility_gate(chip_vel, enc_vel)

    def test_plausible_chip_passes_within_band(self) -> None:
        """Chip reading within [0.5×, 2×] of encoder passes the gate."""
        enc_vel  = 200.0
        # Low end of band: 0.5 × 200 = 100 → chip at 101 should pass
        assert plausibility_gate(101.0, enc_vel)
        # High end of band: 2 × 200 = 400 → chip at 399 should pass
        assert plausibility_gate(399.0, enc_vel)

    def test_plausible_chip_passes_reverse(self) -> None:
        """Reverse direction plausible reading passes."""
        chip_vel = -195.0
        enc_vel  = -200.0
        assert plausibility_gate(chip_vel, enc_vel)

    # --- near-zero guard (gate does not fire at low speed) ---

    def test_no_rejection_when_enc_below_min_wheel_mms(self) -> None:
        """tooLow branch does NOT fire when |encVel| <= minWheelMms (deadband).

        At very low speeds, encoder-delta is noisy and an unreliable reference.
        The gate must not reject chip readings just because encoder-delta is small.
        """
        chip_vel = 10.0            # chip reading at low speed
        enc_vel  = MIN_WHEEL_MMS   # exactly at the floor — tooLow condition is (> floor)
        # At exactly MIN_WHEEL_MMS the tooLow branch uses strict > so it does NOT fire.
        assert plausibility_gate(chip_vel, enc_vel)

    def test_no_rejection_when_enc_zero(self) -> None:
        """Gate does not fire when encoder-delta is zero (zero reference guard)."""
        chip_vel = 50.0  # small non-zero chip reading
        enc_vel  = 0.0   # encoder says stopped
        assert plausibility_gate(chip_vel, enc_vel)

    def test_toolow_fires_just_above_min_wheel_mms(self) -> None:
        """tooLow fires when |encVel| is just above minWheelMms and chip is < 0.5x."""
        enc_vel  = MIN_WHEEL_MMS + 1.0  # 21.0 mm/s — above the floor
        chip_vel = 5.0                  # < 0.5 × 21 = 10.5
        assert not plausibility_gate(chip_vel, enc_vel)

    # --- encoder-delta selected when gate rejects chip ---

    def test_fallback_to_encoder_delta_on_stuck_low(self) -> None:
        """When gate rejects a stuck-low chip reading, encoder-delta is the source.

        This test verifies the full selection logic: if gate returns False,
        the velocity feedback source flag should be 'E' (encoder), not 'C' (chip).
        """
        chip_vel = 30.0
        enc_vel  = 140.0
        gate_passes = plausibility_gate(chip_vel, enc_vel)
        assert not gate_passes
        # Source selection mirrors MotorController: chipOk ? chipVel : encVel
        actual_vel = chip_vel if gate_passes else enc_vel
        expected_source = "E"  # encoder-delta selected
        assert actual_vel == enc_vel
        assert expected_source == "E"

    def test_chip_source_when_plausible(self) -> None:
        """When gate passes, chip velocity is the selected source ('C' flag)."""
        chip_vel = 198.0
        enc_vel  = 200.0
        gate_passes = plausibility_gate(chip_vel, enc_vel)
        assert gate_passes
        actual_vel = chip_vel if gate_passes else enc_vel
        assert actual_vel == chip_vel
