---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 027 Use Cases

---

## SUC-001: Field-profile CI gate

- **Actor**: CI / developer
- **Preconditions**: The sim shared library is built; existing motion-control
  tests exist in `host_tests/`.
- **Main Flow**:
  1. Developer pushes a firmware change (e.g., curvature clamp, OTOS gating).
  2. CI runs `pytest host_tests/` with both the exact profile (default) and the
     field profile (turn slip ≈ 0.26, OTOS fusion ON, motor deadband, ~15 ms
     command latency).
  3. Both profiles must pass; a PR that passes only the exact profile is
     rejected.
  4. The four incident scenario regression tests (SUC-002 through SUC-005)
     run in the field profile and pass (or fail before the Dx fix lands and
     pass after).
- **Postconditions**: Every merged firmware change is validated under
  conditions that reproduce the known real-world failure modes.
- **Acceptance Criteria**:
  - [ ] `conftest.py` (or a parametrize decorator) runs all motion-control
        tests in both profiles.
  - [ ] Four scenario regression tests exist and are tagged as field-profile.
  - [ ] CI command documented in sprint README or CI config.

---

## SUC-002: Scenario — G-into-boards does not happen (D8)

- **Actor**: Robot (autonomous motion), field operator
- **Preconditions**: Sprint 026 single dispatch path is operational; field
  profile is enabled in the test environment; pursuit law uses current
  unbounded curvature.
- **Main Flow**:
  1. Host issues `G tx ty speed` to a target that requires PRE_ROTATE followed
     by PURSUE.
  2. A fused-pose correction or motor-slip event places the target laterally
     or slightly behind the robot mid-pursuit.
  3. The curvature clamp limits `|κ| ≤ 2 / max(d_remaining, 2·arriveTolMm)`.
  4. If bearing flips > 90° for 3 consecutive ticks, PURSUE drops back to
     PRE_ROTATE.
  5. The robot converges on the target; position stop fires within the widened
     `arriveTolMm` (20–25 mm); `EVT done G` emitted.
- **Postconditions**: Robot stops at the target; no unbounded orbital looping;
  no board collision.
- **Acceptance Criteria**:
  - [ ] Sim field-profile: targets at 0°, ±90°, 180°, and 30 mm lateral
        offset all converge; orbit count < 1.5 revolutions.
  - [ ] `arriveTolMm` and related constants in `tovez.json`; DefaultConfig
        regenerated.

---

## SUC-003: Scenario — Keepalive does not kill a TURN (D6)

- **Actor**: Host (keepalive daemon), firmware (TURN command)
- **Preconditions**: A `TURN` or `G` MotionCommand is active; host streams
  `VW` or `S` commands as keepalives.
- **Main Flow**:
  1. Host issues `TURN 9000`; firmware starts a TURN MotionCommand.
  2. Host keepalive daemon sends `S 0 0` or `VW 0 0` while the TURN is
     executing.
  3. `handleVW` no-stop-params branch detects the active command has
     `Origin != VW`; it resets the watchdog, replies `OK vw busy=TURN`, and
     does NOT call `setTarget`.
  4. TURN MotionCommand continues uninterrupted; HEADING stop fires at the
     commanded angle; `EVT done TURN` emitted.
- **Postconditions**: TURN completes at the intended heading; subsequent G
  commands use the correct pose.
- **Acceptance Criteria**:
  - [ ] Sim (queue-wired): start TURN, inject `S 0 0` mid-turn → TURN
        completes at commanded heading.
  - [ ] `test_d6_cannot_stomp_turn` (xfail from 026-003) is promoted to
        passing.
  - [ ] `protocol.py` docstrings for `vw()` / `drive()` no longer recommend
        the destructive keepalive pattern.

---

## SUC-004: Scenario — No spin on robot placement (D9)

- **Actor**: Robot (OTOS sensor, EKF), field operator
- **Preconditions**: Robot is mid-motion (G, PRE_ROTATE); operator lifts and
  replaces the robot to reposition it.
- **Main Flow**:
  1. Robot is lifted: OTOS optical tracking loses the floor; STATUS register
     sets tilt/tracking-invalid bits; I2C read may return zeros.
  2. `otosCorrect()` reads the STATUS register before consuming pose/velocity;
     detects invalid state; sets `state.inputs.otos.valid = false`; skips EKF
     fusion that tick.
  3. After ~500 ms of continuous invalidity during active motion, firmware
     emits one-shot `EVT otos lost`.
  4. Robot is placed back on the floor; OTOS re-acquires; STATUS bits clear;
     fusion resumes.
  5. No unbounded pre-rotate spin occurs because: (a) garbage velocity is not
     fed to the EKF, so heading estimate does not diverge arbitrarily fast,
     and (b) PRE_ROTATE is supervised (D5, sprint 024) and has its own TIME
     net.
- **Postconditions**: Robot stops or completes the interrupted command cleanly;
  no full-speed spin; `EVT otos lost` is visible to the host.
- **Acceptance Criteria**:
  - [ ] Hardware: lift robot mid-G → `EVT otos lost` appears; no spin on
        replacement; pose recovers after `SI`/camera fix.
  - [ ] STATUS register path added to `OtosSensor`; valid flag propagated to
        `otosCorrect`.

---

## SUC-005: Scenario — TURN under-rotate is caught in sim (D2 / regression)

- **Actor**: Developer, CI
- **Preconditions**: Field profile is active (turn slip 0.26, OTOS fusion ON).
- **Main Flow**:
  1. Sim executes `TURN 9000` in the exact profile → passes (encoder IS truth).
  2. Sim executes `TURN 9000` in the field profile → the `rotationalSlip`
     correction (sprint 024) keeps the encoder-heading close to the mock-OTOS
     truth; the test verifies convergence within ±5°.
  3. The field profile is the regression guard for the "TURN-under-rotate"
     incident scenario.
- **Postconditions**: Field-profile TURN regression test exists and passes
  with corrected slip.
- **Acceptance Criteria**:
  - [ ] `test_scenario_turn_under_rotate` in `host_tests/` passes in field
        profile.

---

## SUC-006: Hardware smoke ritual gate

- **Actor**: Developer / team-lead agent (before/after firmware flash)
- **Preconditions**: Firmware flashed; robot powered and communicating;
  `smoke_ritual.py` script exists in `tests/bench/`.
- **Main Flow**:
  1. Developer runs `uv run python tests/bench/smoke_ritual.py`.
  2. Script runs in sequence: SAFE query, TURN×4 orientation closure, G square,
     lift test, stream drop-rate print.
  3. Each check prints PASS or FAIL.
  4. Results appended to `docs/knowledge/field-log.md` with timestamp + git SHA.
- **Postconditions**: Dated, SHA-stamped record in field-log.md; clear PASS/FAIL
  per check.
- **Acceptance Criteria**:
  - [ ] Script exists and runs end-to-end against the robot.
  - [ ] Field log entry written with date + SHA.
  - [ ] TURN×4 closure test exercises D1/D2 heading truth.
  - [ ] Lift test exercises D9 (EVT otos lost + no spin).
  - [ ] Drop-rate print uses TLM `seq` gaps (requires D10 TLM seq numbers
        added here or stubbed).

---

## SUC-007: Bench programs abort on runaway/stall

- **Actor**: Developer (bench test author), robot
- **Preconditions**: A bench or dev program is driving the robot; a fault
  condition occurs (frozen encoders, full-tilt with no motion, host arrival
  logic wrong).
- **Main Flow**:
  1. Bench program detects fault: full-tilt with no encoder motion, or
     no-progress toward target for N seconds, or telemetry silence beyond
     grace window.
  2. Program sends `X` immediately and raises a descriptive exception.
  3. On any program exit (normal, exception, Ctrl-C), a `finally` block sends
     `X` and clears any stream.
  4. Preflight liveness check runs before any motion; hard-fails if robot
     silent.
- **Postconditions**: Robot motors stopped within the detection window;
  host process exits with a clear error; no orphaned drive commands.
- **Acceptance Criteria**:
  - [ ] Shared safety wrapper module exists in `tests/bench/`.
  - [ ] Induced runaway (e.g. frozen-encoder simulation) sends `X` within the
        detection window and exits.
  - [ ] All new and existing bench programs use the wrapper.
  - [ ] Interrupted bench program (Ctrl-C) leaves the robot stopped.

---

## SUC-008: field-024 spin diagnosis closed

- **Actor**: Developer, team-lead agent
- **Preconditions**: sprint-024 anomalies documented in
  `.clasi/issues/field-024-full-speed-spin-unresolved.md`.
- **Main Flow**:
  1. The "host abandons G without X" lead is closed by the runaway wrapper
     (SUC-007): every bench program now sends `X` on exit.
  2. The SNAP TLM discrepancy (`enc=0`/`mode=IDLE` while spinning) is
     investigated: compare `buildTlmFrame` logic on the SNAP vs STREAM paths.
     If the root cause requires only a one-line fix (e.g. wrong field read on
     SNAP), fix it here; if it requires D10 firmware TLM restructuring, file
     a cross-reference to sprint 028 and mark this lead as deferred.
  3. OTOS validity gating (SUC-004) removes the garbage-input trigger that
     caused the fused heading to diverge, eliminating the root cause of the
     PRE_ROTATE gate never closing.
- **Postconditions**: Both open leads from field-024 are either resolved or
  explicitly deferred with a sprint 028 reference; the issue can be closed
  or marked as partially resolved.
- **Acceptance Criteria**:
  - [ ] Bench programs send `X` on exit (covered by SUC-007).
  - [ ] SNAP TLM discrepancy investigated; fix applied or sprint 028 reference
        filed.
  - [ ] field-024 issue updated with status and resolution notes.
