"""
test_config_set.py — Sim tests for SET validation: typed parse, range checks,
atomic apply (ticket 028-004).

Tests verify:
  - SET tw=0         → ERR badval tw=0;   live cfg.tw unchanged (GET tw shows original)
  - SET tw=abc       → ERR badval tw;     non-numeric value rejected at parse stage
  - SET pid.kp=1.5 tw=0  → ERR badval tw=0; pid.kp unchanged (atomicity)
  - SET pid.kp=1.5 pid.ki=0.05  → OK set; GET confirms both applied
  - SET ctrlPeriod=0 → ERR badval ctrlPeriod=0  (controlPeriodMs > 0 invariant)
  - SET vWheelMax=10 steerHeadroom=50 → ERR badval (vWheelMax > steerHeadroom cross-field)
  - SET rotSlip=0.3  → ERR badval rotSlip=0.300  (rotationalSlip [0.5, 1.0] invariant)

The Sim fixture is defined in conftest.py; it extends sTimeout=60000 to avoid
watchdog interference.
"""
from __future__ import annotations
import re


# ---------------------------------------------------------------------------
# Parse helpers
# ---------------------------------------------------------------------------

def _get_val(sim, key: str) -> str:
    """Send GET <key> and return the bare value string from the CFG reply."""
    reply = sim.send_command(f"GET {key}")
    m = re.search(rf"{re.escape(key)}=([^\s]+)", reply)
    assert m is not None, f"Could not find {key}= in GET reply: {reply!r}"
    return m.group(1)


# ---------------------------------------------------------------------------
# Range/invariant rejection tests
# ---------------------------------------------------------------------------

class TestSetValidationRejection:
    """SET validation rejects out-of-range values and leaves live config unchanged."""

    def test_set_tw_zero_rejected(self, sim) -> None:
        """SET tw=0 → ERR badval; reply mentions 'tw'."""
        # Record original tw before the SET attempt.
        original_tw = _get_val(sim, "tw")

        reply = sim.send_command("SET tw=0")
        assert "ERR" in reply, f"Expected ERR for tw=0, got {reply!r}"
        assert "badval" in reply, f"Expected badval in reply, got {reply!r}"
        assert "tw" in reply, f"Expected 'tw' in ERR reply, got {reply!r}"
        assert "OK" not in reply, f"Must not emit OK for invalid SET, got {reply!r}"

    def test_set_tw_zero_live_config_unchanged(self, sim) -> None:
        """After SET tw=0 is rejected, GET tw returns the original value."""
        original_tw = _get_val(sim, "tw")
        sim.send_command("SET tw=0")
        after_tw = _get_val(sim, "tw")
        assert original_tw == after_tw, (
            f"tw changed after rejected SET: was {original_tw!r}, now {after_tw!r}"
        )

    def test_set_tw_abc_rejected(self, sim) -> None:
        """SET tw=abc → ERR badval (non-numeric parse failure)."""
        reply = sim.send_command("SET tw=abc")
        assert "ERR" in reply, f"Expected ERR for tw=abc, got {reply!r}"
        assert "badval" in reply, f"Expected badval in reply, got {reply!r}"
        assert "tw" in reply, f"Expected 'tw' in ERR reply, got {reply!r}"
        assert "OK" not in reply, f"Must not emit OK for non-numeric value, got {reply!r}"

    def test_set_tw_abc_live_config_unchanged(self, sim) -> None:
        """After SET tw=abc is rejected, GET tw returns the original value."""
        original_tw = _get_val(sim, "tw")
        sim.send_command("SET tw=abc")
        after_tw = _get_val(sim, "tw")
        assert original_tw == after_tw, (
            f"tw changed after rejected SET: was {original_tw!r}, now {after_tw!r}"
        )

    def test_set_mixed_valid_invalid_rejected(self, sim) -> None:
        """SET pid.kp=1.5 tw=0 → ERR badval; pid.kp unchanged (atomicity)."""
        original_kp = _get_val(sim, "pid.kp")

        reply = sim.send_command("SET pid.kp=1.5 tw=0")
        assert "ERR" in reply, f"Expected ERR for mixed valid/invalid SET, got {reply!r}"
        assert "badval" in reply, f"Expected badval in reply, got {reply!r}"
        assert "OK" not in reply, f"Must not emit OK for partially-invalid SET, got {reply!r}"

    def test_set_mixed_pid_kp_unchanged(self, sim) -> None:
        """After SET pid.kp=1.5 tw=0 is rejected, pid.kp must be unchanged (atomicity)."""
        original_kp = _get_val(sim, "pid.kp")
        sim.send_command("SET pid.kp=1.5 tw=0")
        after_kp = _get_val(sim, "pid.kp")
        assert original_kp == after_kp, (
            f"pid.kp changed after rejected SET (atomicity violated): "
            f"was {original_kp!r}, now {after_kp!r}"
        )


# ---------------------------------------------------------------------------
# Valid multi-key SET applies atomically
# ---------------------------------------------------------------------------

class TestSetValidationAccept:
    """Valid SETs apply atomically and reply OK set."""

    def test_set_pid_kp_ki_ok(self, sim) -> None:
        """SET pid.kp=1.5 pid.ki=0.05 → OK set containing both keys."""
        reply = sim.send_command("SET pid.kp=1.5 pid.ki=0.05")
        assert "OK" in reply, f"Expected OK for valid SET, got {reply!r}"
        assert "set" in reply, f"Expected 'set' verb in OK reply, got {reply!r}"
        assert "pid.kp" in reply, f"Expected pid.kp in OK body, got {reply!r}"
        assert "pid.ki" in reply, f"Expected pid.ki in OK body, got {reply!r}"

    def test_set_pid_kp_reads_back(self, sim) -> None:
        """SET pid.kp=1.5 → GET pid.kp returns 1.500."""
        sim.send_command("SET pid.kp=1.5 pid.ki=0.05")
        val = _get_val(sim, "pid.kp")
        assert val == "1.500", f"Expected pid.kp=1.500 after SET, got {val!r}"

    def test_set_pid_ki_reads_back(self, sim) -> None:
        """SET pid.ki=0.05 → GET pid.ki returns 0.050."""
        sim.send_command("SET pid.kp=1.5 pid.ki=0.05")
        val = _get_val(sim, "pid.ki")
        assert val == "0.050", f"Expected pid.ki=0.050 after SET, got {val!r}"


# ---------------------------------------------------------------------------
# Additional invariant checks
# ---------------------------------------------------------------------------

class TestSetInvariants:
    """Validate each invariant check individually."""

    def test_ctrlperiod_zero_rejected(self, sim) -> None:
        """SET ctrlPeriod=0 → ERR badval (controlPeriodMs > 0 invariant)."""
        original = _get_val(sim, "ctrlPeriod")
        reply = sim.send_command("SET ctrlPeriod=0")
        assert "ERR" in reply, f"Expected ERR for ctrlPeriod=0, got {reply!r}"
        assert "badval" in reply, f"Expected badval in reply, got {reply!r}"
        # Config unchanged
        after = _get_val(sim, "ctrlPeriod")
        assert original == after, (
            f"ctrlPeriod changed after rejected SET: was {original!r}, now {after!r}"
        )

    def test_ctrlperiod_negative_rejected(self, sim) -> None:
        """SET ctrlPeriod=-1 → ERR badval (negative wraps to large uint32 in scheduler)."""
        original = _get_val(sim, "ctrlPeriod")
        reply = sim.send_command("SET ctrlPeriod=-1")
        assert "ERR" in reply, f"Expected ERR for ctrlPeriod=-1, got {reply!r}"
        assert "badval" in reply, f"Expected badval in reply, got {reply!r}"
        after = _get_val(sim, "ctrlPeriod")
        assert original == after, (
            f"ctrlPeriod changed after rejected SET: was {original!r}, now {after!r}"
        )

    def test_rotslip_out_of_range_rejected(self, sim) -> None:
        """SET rotSlip=0.3 → ERR badval (must be in [0.5, 1.0])."""
        original = _get_val(sim, "rotSlip")
        reply = sim.send_command("SET rotSlip=0.3")
        assert "ERR" in reply, f"Expected ERR for rotSlip=0.3, got {reply!r}"
        assert "badval" in reply, f"Expected badval in reply, got {reply!r}"
        after = _get_val(sim, "rotSlip")
        assert original == after, (
            f"rotSlip changed after rejected SET: was {original!r}, now {after!r}"
        )

    def test_rotslip_above_one_rejected(self, sim) -> None:
        """SET rotSlip=1.5 → ERR badval (must be in [0.5, 1.0])."""
        original = _get_val(sim, "rotSlip")
        reply = sim.send_command("SET rotSlip=1.5")
        assert "ERR" in reply, f"Expected ERR for rotSlip=1.5, got {reply!r}"
        assert "badval" in reply, f"Expected badval in reply, got {reply!r}"
        after = _get_val(sim, "rotSlip")
        assert original == after, (
            f"rotSlip changed after rejected SET: was {original!r}, now {after!r}"
        )

    def test_vwheelmax_cross_field_rejected(self, sim) -> None:
        """SET vWheelMax=10 rejected when vWheelMax <= current steerHeadroom (20)."""
        # Default steerHeadroom=20; vWheelMax=10 would violate vWheelMax > steerHeadroom.
        original_vwm = _get_val(sim, "vWheelMax")
        reply = sim.send_command("SET vWheelMax=10")
        assert "ERR" in reply, f"Expected ERR for vWheelMax=10 (below steerHeadroom), got {reply!r}"
        assert "badval" in reply, f"Expected badval in reply, got {reply!r}"
        after_vwm = _get_val(sim, "vWheelMax")
        assert original_vwm == after_vwm, (
            f"vWheelMax changed after rejected SET: was {original_vwm!r}, now {after_vwm!r}"
        )

    def test_vwheelmax_valid_accepted(self, sim) -> None:
        """SET vWheelMax=350 (above steerHeadroom=20) → OK."""
        reply = sim.send_command("SET vWheelMax=350")
        assert "OK" in reply, f"Expected OK for valid vWheelMax=350, got {reply!r}"
        val = _get_val(sim, "vWheelMax")
        assert val == "350.000", f"Expected vWheelMax=350.000 after SET, got {val!r}"

    def test_set_float_key_with_trailing_garbage_rejected(self, sim) -> None:
        """SET ml=0.5abc → ERR badval ml (trailing garbage fails end-pointer check)."""
        original = _get_val(sim, "ml")
        reply = sim.send_command("SET ml=0.5abc")
        assert "ERR" in reply, f"Expected ERR for ml=0.5abc, got {reply!r}"
        assert "badval" in reply, f"Expected badval in reply, got {reply!r}"
        after = _get_val(sim, "ml")
        assert original == after, (
            f"ml changed after rejected SET: was {original!r}, now {after!r}"
        )

    def test_tw_valid_accepted(self, sim) -> None:
        """SET tw=120 (positive integer) → OK and reads back as 120."""
        reply = sim.send_command("SET tw=120")
        assert "OK" in reply, f"Expected OK for valid tw=120, got {reply!r}"
        val = _get_val(sim, "tw")
        assert val == "120", f"Expected tw=120 after SET, got {val!r}"
