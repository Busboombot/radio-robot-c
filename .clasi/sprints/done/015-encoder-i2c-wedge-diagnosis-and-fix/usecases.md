---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 015 Use Cases

## SUC-001: Reproducible Wedge Rate Measurement
Parent: (diagnostic tooling)

- **Actor**: Developer running bench experiments
- **Preconditions**: Robot is connected via USB serial; firmware is flashed and
  running; `SET sTimeout` accepted.
- **Main Flow**:
  1. Developer invokes `tests/bench/wedge_repro.py` with a mode flag
     (`--watchdog-stop` or `--clean-stop`) and a cycle count.
  2. Script connects without DTR reset (dtr=False, dsrdtr=False), PINGs the
     robot for liveness, and sets a wide sTimeout.
  3. Script drives a sequence of drive→stop→drive cycles, maintaining the S
     keepalive during each drive phase.
  4. In `--watchdog-stop` mode the script intentionally lets the keepalive
     lapse so the firmware S-watchdog fires the stop.
  5. After each stop, the script reads back encoder deltas for the next drive
     phase and classifies the cycle as "wedged" (commanded to move but
     enc delta ≈ 0) or "clean".
  6. After N cycles the script prints a wedge-rate summary (X wedged / N
     total) and exits.
- **Postconditions**: Developer has a numeric wedge-rate for each stop-trigger
  mode; a mode that reliably produces wedges within ~20 cycles is identified
  (or absence of a reliable trigger is documented).
- **Acceptance Criteria**:
  - [ ] Script produces a wedge-rate number for both `--clean-stop` and
        `--watchdog-stop` modes.
  - [ ] Wedge detection is automatic (no manual inspection of logs required).
  - [ ] A documented mode produces wedges within ~20 cycles when the wedge is
        triggerable; running with the mode that historically works passes ≥1
        observed wedge in a 50-cycle run on the hardware bench.

---

## SUC-002: I2C Error Visibility from Firmware
Parent: (diagnostic instrumentation)

- **Actor**: Developer analyzing wedge root cause
- **Preconditions**: Firmware includes `I2CBus` wrapper; robot is driving or has
  driven.
- **Main Flow**:
  1. Developer sends `DBG I2C` over serial.
  2. Firmware responds with one line: per-device transaction counts, per-device
     error counts, last error code per device, re-entrancy violation count, and
     a consecutive-identical-encoder-read stuck-counter.
  3. Developer correlates error counts and re-entrancy violations with observed
     wedge timing.
- **Postconditions**: Developer can see whether I2C errors preceded the wedge
  and whether re-entrancy was observed.
- **Acceptance Criteria**:
  - [ ] `DBG I2C` responds without crashing on a connected robot.
  - [ ] Response fits within one 255-byte serial TX.
  - [ ] Counters increment correctly: driving 50 cycles accumulates txn counts
        for all active devices.

---

## SUC-003: Firmware-Side Wedge Event Emission
Parent: (diagnostic instrumentation)

- **Actor**: Wedge reproduction script / developer monitoring serial
- **Preconditions**: Firmware includes I2CBus wrapper and encoder-stuck
  detection; robot is in a drive cycle.
- **Main Flow**:
  1. Firmware detects N consecutive encoder reads returning the same value
     while commanded PWM is non-zero.
  2. Firmware emits `EVT enc_wedged ...` once with bus error and re-entrancy
     stats at the moment of detection.
  3. Wedge-repro script or developer serial monitor receives the EVT line.
- **Postconditions**: The exact bus state at the moment of wedge is recorded;
  developer can correlate with I2CBus counters.
- **Acceptance Criteria**:
  - [ ] `EVT enc_wedged` is emitted when a wedge is induced on hardware.
  - [ ] The EVT line includes bus error count, re-entrancy violation count, and
        last error code.
  - [ ] No duplicate EVT spam: only emitted once per wedge event.

---

## SUC-004: Re-entrancy Guard Measurement
Parent: (diagnostic instrumentation)

- **Actor**: Developer verifying T3 concurrency hypothesis
- **Preconditions**: I2CBus wrapper is in place with `_inUse` flag and
  target_disable_irq/enable_irq guard.
- **Main Flow**:
  1. Developer runs a 100-cycle wedge-repro session on hardware.
  2. After the session, developer sends `DBG I2C`.
  3. Response shows whether re-entrancy-violation counter is zero (confirming
     T3 is ruled out) or non-zero (proving an overlapping caller exists).
- **Postconditions**: T3 concurrency hypothesis is either ruled out or
  confirmed by measured data, not by analysis alone.
- **Acceptance Criteria**:
  - [ ] Re-entrancy violation counter is present in `DBG I2C` output.
  - [ ] Counter remains zero across a clean 50-cycle session in cooperative-loop
        mode (expected: concurrency should NOT be the cause in a single-loop
        architecture).
  - [ ] If the counter is non-zero, the offending address pair is captured.
