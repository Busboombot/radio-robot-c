---
status: approved
---

# Sprint 033 Use Cases

## SUC-001: Bench harness communicates with robot over USB serial

- **Actor**: Developer running bench validation
- **Preconditions**: Robot is physically present on bench stand with USB cable connected
- **Main Flow**:
  1. Developer runs `tests/bench/bench_validation_032.py` (or `enc_balance_test.py`)
  2. Harness opens the robot's USB serial port directly (no relay)
  3. Harness sends plain commands and receives replies including DBG output
  4. DBG replies (`OK dbg otos bench=1`) are visible in the harness
- **Postconditions**: All DBG command replies are received; relay ambiguity is eliminated
- **Acceptance Criteria**:
  - [ ] `bench_validation_032.py` uses `SerialConnection(port, mode="direct")` or equivalent direct serial open
  - [ ] `enc_balance_test.py` uses direct serial (no `!GO` data-plane protocol)
  - [ ] Script imports and connection path are correct; hardware execution is post-sprint

---

## SUC-002: `DBG OTOS BENCH 1` successfully enables bench mode on firmware

- **Actor**: Developer running bench validation
- **Preconditions**: Robot firmware flashed; USB serial connected
- **Main Flow**:
  1. Developer sends `DBG OTOS BENCH 1` over USB serial
  2. Firmware handler parses the enable flag, calls `setOtosBench(true)`
  3. `isBenchMode()` returns true; `otos()` returns BenchOtosSensor
  4. Reply `OK dbg otos bench=1` is sent over USB serial
- **Postconditions**: Bench OTOS is active; subsequent OTOS reads use the synthetic sensor
- **Acceptance Criteria**:
  - [ ] `DBG OTOS BENCH 1` replies `bench=1` over USB serial
  - [ ] `DBG OTOS BENCH 0` replies `bench=0`
  - [ ] A host-side sim test or seam confirms `isBenchMode()` state after enable

---

## SUC-003: Fused body velocity (`twist`) tracks encoder velocity when OTOS is invalid

- **Actor**: Firmware / telemetry consumer
- **Preconditions**: Robot driving; OTOS reports tracking-invalid (lifted stand or out-of-range)
- **Main Flow**:
  1. Robot drives with non-zero wheel velocity
  2. OTOS validity gate triggers; `otosCorrect()` early-returns
  3. Encoder velocity is fused into EKF unconditionally (not behind the OTOS gate)
  4. `state.inputs.fusedV` and `fusedOmega` are updated from encoder-derived velocity
  5. `twist=` TLM field shows non-zero values tracking the encoder motion
- **Postconditions**: `twist=` is non-zero during driving regardless of OTOS validity
- **Acceptance Criteria**:
  - [ ] Sim test: OTOS invalid + wheels moving → `fusedV` nonzero, `fusedOmega` nonzero
  - [ ] `enc_omega` observation gated on both encoders healthy (no phantom omega when wedged)

---

## SUC-004: `D` command travels full distance even after a `TURN` without `ZERO enc`

- **Actor**: Motion command processor
- **Preconditions**: Robot has completed a TURN command; encoders not zeroed; next command is D
- **Main Flow**:
  1. `TURN` leaves non-zero average encoder value in `state.inputs`
  2. `D <left> <right> <distance>` is issued
  3. `beginDistance()` zeros `state.inputs.encLMm/R` before snapshotting the baseline
  4. `MotionCommand::start()` captures `enc0Mm = 0`
  5. Robot drives the full commanded distance
- **Postconditions**: Distance traveled equals the commanded distance
- **Acceptance Criteria**:
  - [ ] Sim test: `D` → `TURN` → `D` with no `ZERO enc` between; second `D` travels full distance
  - [ ] No instant-complete on first evaluate when prior encoder average ≥ target

---

## SUC-005: Encoder wedge detector provides self-disambiguating diagnostics

- **Actor**: Firmware diagnostic system / developer
- **Preconditions**: Robot driving; one or both encoders may be stuck (stall, filter-hold, or hardware)
- **Main Flow**:
  1. Encoder reports identical filtered value for ≥ 10 consecutive ticks
  2. Wedge EVT includes raw encoder read alongside filtered value
  3. Outlier-filter hold streaks are counted and emitted as EVT/TLM
  4. `ZERO enc` offset snapshot uses median-of-3 and verifies readback ≈ 0
  5. Wedge arming has grace period at drive start (prevents battery-droop false positives)
  6. Odometry stops integrating dTheta differential from a wedged wheel
  7. `enc_omega` observation suppressed while any wheel is wedged
- **Postconditions**: EVT is self-disambiguating; odometry doesn't drift from a single bad wheel
- **Acceptance Criteria**:
  - [ ] `EVT enc_wedged` includes raw value field
  - [ ] Filter-hold streak counter increments and emits EVT at threshold
  - [ ] `ZERO enc` readback verifies |value| ≈ 0 after reset (retries on failure)
  - [ ] Sim test: mock garbage ZERO-enc read → readback verification rejects, retries
  - [ ] Sim test: wedged wheel → no phantom dTheta in odometry; `enc_omega` suppressed
  - [ ] Arming grace: wheels must move at least once before wedge can arm
