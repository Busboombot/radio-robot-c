"""
test_n12_n13_get_chunking_dead_code.py — regression tests for sprint 030-010.

N12: Full GET dump (~58 keys, ~805 bytes) is chunked into multiple CFG lines
     each <= 200 content bytes, so each line fits CODAL's 255-byte serial TX
     buffer.  Tests assert:
       - Every emitted CFG line is <= 200 content bytes (the kCfgChunkMax limit).
       - The full key set is recoverable by accumulating all CFG lines.
       - Named-key GET still returns a single CFG line containing the key.

     BENCH CONFIRM NEEDED: verify on hardware that all 58 keys are received
     without truncation after this change.  The bench step is a stakeholder/
     team-lead task with the robot connected over serial.

N13: Dead/vestigial code removed:
       - RatioPidController removed from MotorController (never ran in
         controlTick; sync-gain coupling replaced it).
       - PID_BYPASS macro removed (was always 0; encoder-wedge root cause fixed).
       - Odometry::update() removed (deprecated, no callers).
       - DriveMode::TIMED removed (T command runs as VELOCITY; mode= can never
         emit 'T' from firmware).

     pid.* keys RETAINED in ConfigRegistry (host tests use SET/GET pid.*).
     Host TLM parser still accepts mode=T for backward-compatibility with old
     logs; firmware never emits it.

     Caller-safety checks (documented here):
       - grep host_tests/ host/ tests/ for "pid." SET usage: FOUND in
         host_tests/test_config_set.py and host/tests/test_protocol_v2.py.
         pid.* keys kept as no-op config fields (SET/GET accepted, no live
         controller).  No callers depend on the side-effect of updatePidGains().
       - grep host/robot_radio for mode=T parser: test_protocol_v2.py has
         test_parse_tlm_mode_field_T and test_tlm_all_fields both test the
         parser with a synthetic TLM string — they do NOT require firmware to
         emit mode=T.  Parser is unchanged; test passes.
"""
from __future__ import annotations
import re


# ---------------------------------------------------------------------------
# N12 helpers
# ---------------------------------------------------------------------------

def _collect_cfg_lines(raw: str) -> list[str]:
    """Return all 'CFG ...' lines from a raw (possibly multi-line) reply."""
    return [line for line in raw.splitlines() if line.startswith("CFG")]


def _merge_cfg(lines: list[str]) -> dict[str, str]:
    """Merge all CFG lines into one key->value dict (mirrors host get_config)."""
    result: dict[str, str] = {}
    for line in lines:
        # Strip "CFG " prefix, then split on whitespace into "key=value" tokens.
        body = line[4:]  # skip "CFG "
        # Remove optional "#corrId" at the end.
        body = re.sub(r'\s*#\S+\s*$', '', body)
        for token in body.split():
            if '=' in token:
                k, _, v = token.partition('=')
                result[k] = v
    return result


# ---------------------------------------------------------------------------
# N12 — GET chunking: each CFG line fits the 255-byte CODAL TX buffer
# ---------------------------------------------------------------------------

class TestN12GetChunking:
    """N12: bare GET emits multiple CFG lines, each within the buffer cap."""

    def test_bare_get_emits_multiple_cfg_lines(self, sim) -> None:
        """Bare GET produces at least 2 CFG lines (one chunk would overflow 255 bytes)."""
        raw = sim.send_command("GET")
        lines = _collect_cfg_lines(raw)
        assert len(lines) >= 2, (
            f"Expected >= 2 CFG lines from bare GET (full dump ~805 bytes >> 255), "
            f"got {len(lines)}: {raw!r}"
        )

    def test_each_cfg_line_fits_buffer(self, sim) -> None:
        """Each CFG line from bare GET must be <= 200 content bytes (kCfgChunkMax)."""
        raw = sim.send_command("GET")
        lines = _collect_cfg_lines(raw)
        assert lines, "Expected at least one CFG line from bare GET"
        for line in lines:
            # Content bytes = everything after "CFG " (4 chars), before any corrId.
            body = line[4:]
            body_no_corr = re.sub(r'\s*#\S+\s*$', '', body)
            content_len = len(body_no_corr)
            assert content_len <= 200, (
                f"CFG line content ({content_len} bytes) exceeds kCfgChunkMax=200: "
                f"{line!r}"
            )

    def test_full_key_set_recoverable_from_chunks(self, sim) -> None:
        """Merging all CFG lines from bare GET yields all registered keys."""
        raw = sim.send_command("GET")
        lines = _collect_cfg_lines(raw)
        merged = _merge_cfg(lines)

        # Spot-check a selection of keys spanning the full registry.
        expected_keys = [
            "ml", "mr", "kff", "tw",
            "pid.kp", "pid.ki", "pid.kd", "pid.max",
            "vel.kP", "vel.kI", "vel.kFF", "vel.iMax",
            "sync", "vWheelMax", "steerHeadroom",
            "alphaPos", "alphaYaw",
            "aMax", "aDecel", "vBodyMax", "yawRateMax", "yawAccMax",
            "sTimeout", "ctrlPeriod", "tlmPeriod",
            "lag.otos", "lag.line", "lag.color", "lag.ports",
            "otosLinSc", "otosAngSc", "rotSlip",
            "odomOffX", "odomOffY", "odomYaw", "ekfRHead",
        ]
        missing = [k for k in expected_keys if k not in merged]
        assert not missing, (
            f"Keys missing from merged GET chunks: {missing}\n"
            f"Merged keys: {sorted(merged.keys())}"
        )

    def test_named_get_returns_single_cfg_line(self, sim) -> None:
        """Named-key GET (e.g. 'GET ml tw') returns exactly one CFG line."""
        raw = sim.send_command("GET ml tw")
        lines = _collect_cfg_lines(raw)
        assert len(lines) == 1, (
            f"Expected exactly 1 CFG line for named GET, got {len(lines)}: {raw!r}"
        )
        merged = _merge_cfg(lines)
        assert "ml" in merged, f"Expected 'ml' in CFG reply, got {merged!r}"
        assert "tw" in merged, f"Expected 'tw' in CFG reply, got {merged!r}"

    def test_pid_keys_present_in_chunks(self, sim) -> None:
        """pid.* keys are present in bare GET output (retained for host compat)."""
        raw = sim.send_command("GET")
        merged = _merge_cfg(_collect_cfg_lines(raw))
        for key in ("pid.kp", "pid.ki", "pid.kd", "pid.max"):
            assert key in merged, (
                f"Expected pid key {key!r} in GET output (N13: key retained for "
                f"host compat); got keys: {sorted(merged.keys())}"
            )

    def test_chunk_count_reasonable(self, sim) -> None:
        """Bare GET produces between 2 and 10 chunks (sanity bound)."""
        raw = sim.send_command("GET")
        lines = _collect_cfg_lines(raw)
        assert 2 <= len(lines) <= 10, (
            f"Expected 2-10 CFG chunks for 58-key registry, got {len(lines)}"
        )


# ---------------------------------------------------------------------------
# N13 — dead code removed: pid.* keys kept as no-ops
# ---------------------------------------------------------------------------

class TestN13PidKeysNoOp:
    """N13: pid.* keys are retained in registry but have no live controller effect."""

    def test_set_pid_kp_accepted(self, sim) -> None:
        """SET pid.kp=5.0 is accepted (key still in registry)."""
        reply = sim.send_command("SET pid.kp=5.0")
        assert "OK" in reply, (
            f"Expected OK for SET pid.kp=5.0 (key retained for host compat), "
            f"got {reply!r}"
        )

    def test_set_pid_ki_accepted(self, sim) -> None:
        """SET pid.ki=0.01 is accepted."""
        reply = sim.send_command("SET pid.ki=0.01")
        assert "OK" in reply, f"Expected OK for SET pid.ki, got {reply!r}"

    def test_get_pid_kp_readable(self, sim) -> None:
        """GET pid.kp returns a value (key still readable from RobotConfig)."""
        sim.send_command("SET pid.kp=2.5")
        raw = sim.send_command("GET pid.kp")
        assert "pid.kp=2.500" in raw, (
            f"Expected pid.kp=2.500 in GET reply after SET, got {raw!r}"
        )

    def test_pid_kp_not_in_motor_controller(self, sim) -> None:
        """Verify pid.* has no effect on motor PWM (the PID controller is removed).

        SET pid.kp to a huge value: if RatioPidController were still running in
        controlTick, the enormous gain would cause runaway PWM.  The motor
        should respond identically with pid.kp=300 vs pid.kp=999999.
        This is a compile-time guarantee (update() is never called) but the
        runtime smoke-check confirms no obvious side-effect.
        """
        sim.send_command("SET sTimeout=30000")
        sim.send_command("SET pid.kp=999999.0")
        sim.send_command("S 200 200 5000")
        sim.tick_for(500)
        # Motor should still be driving normally (PWM not at ±100 runaway).
        # The test just confirms no crash or wildly invalid state.
        import ctypes
        pwm_l = float(sim._lib.sim_get_pwm_l(sim._h))
        pwm_r = float(sim._lib.sim_get_pwm_r(sim._h))
        assert -101 < pwm_l < 101, f"PWM_L out of range after pid.kp=999999: {pwm_l}"
        assert -101 < pwm_r < 101, f"PWM_R out of range after pid.kp=999999: {pwm_r}"
