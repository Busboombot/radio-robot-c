---
id: '005'
title: Encoder wedge detector and odometry hardening
status: done
use-cases:
- SUC-005
depends-on:
- '003'
issue: fr-bench-right-encoder-wedge.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Encoder wedge detector and odometry hardening

## Description

The encoder wedge detector (`MotorController::controlTick`, `_stuckCountL/R`, `kWedgeThreshold=10`)
correctly detects "filtered value identical for 10 consecutive ticks" but cannot distinguish
three causes: (1) true chip/I2C wedge, (2) motor stall / battery droop, (3) outlier-filter
hold from a corrupted ZERO-enc offset. The odometry has no defense — a wedged wheel injects
phantom `dTheta` into fused pose with no opposing observation.

This ticket implements five hardening items from the diagnosis (items a-e):

**(a) ZERO enc readback verification** (`Motor::resetEncoder()`, `source/hal/Motor.cpp:131-141`):
after `_encOffset += readEncoderAtomic()`, read back the encoder value and require |result| ≈ 0;
retry the offset snapshot on failure. Consider median-of-3 for the snapshot read to eliminate
single garbage reads (`Robot.cpp:123-125` documents the ~149 mm garbage read behavior).

**(b) Outlier-filter hold instrumentation** (`Robot::controlCollectSplitPhase()`,
`source/robot/Robot.cpp:114-163`): count consecutive rejected reads per wheel; emit an EVT (or
include the streak count in TLM) when the streak exceeds a small threshold (e.g. 3 ticks). A
silent permanent hold is currently invisible.

**(c) Raw read in wedge EVT** (`MotorController::controlTick()`,
`source/control/MotorController.cpp:246-343`): include the raw encoder read alongside the filtered
value in the `EVT enc_wedged` payload. raw frozen + filtered frozen → likely real wedge/stall;
raw moving + filtered frozen → filter-hold. Makes the EVT self-disambiguating.

**(d) Arming grace at drive start** (`source/control/MotorController.cpp`): require the wheel to
have moved at least once since the command started before the wedge latch can arm. This prevents
the spin-up lag of a drained battery from firing the detector prematurely (the 032 bench run's
`EVT enc_wedged` fired exactly in this regime).

**(e) Odometry wedge defense** (`source/control/Odometry.cpp` + `source/control/MotorController.h`):
expose per-wheel wedge state from `MotorController` (e.g. `bool wheelWedged(Side)` returning the
stuck-counter latch state). While a wheel is wedged: stop integrating the differential into `dTheta`
in `Odometry::predict()` (hold heading; optionally estimate `dCenter` from the healthy wheel alone).
Wire the `enc_omega` suppression gate introduced in T003 to use `wheelWedged()` instead of the stub.

Physical wedge root cause (battery vs. chip fault) is out of scope.

## Acceptance Criteria

- [ ] **(a)** `Motor::resetEncoder()` verifies readback ≈ 0 after offset snapshot; retries on failure
- [ ] **(b)** Outlier-filter consecutive-reject streak counter implemented; EVT/TLM emitted at threshold
- [ ] **(c)** `EVT enc_wedged` payload includes `raw=<value>` field alongside filtered value
- [ ] **(d)** Wedge latch does not arm until the wheel has moved at least once since command start
- [ ] **(e)** `MotorController` exposes `wheelWedged(Side)` accessor; `Odometry::predict()` skips
      `dTheta` differential while a wheel is wedged; `enc_omega` observation (from T003) is suppressed
      when either wheel is wedged
- [ ] Sim test: mock garbage ZERO-enc read → readback verification catches it and retries
- [ ] Sim test: wedged right wheel (frozen R encoder) → no phantom dTheta in odometry output
- [ ] Sim test: `enc_omega` is 0 when a wheel is wedged (wires the T003 stub)
- [ ] `python3 build.py` clean build passes
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes

## Testing

- **Existing tests to run**: `uv run --with pytest python -m pytest host_tests/ host/tests/`
- **New tests to write**:
  - Sim test: inject garbage return from `readEncoderAtomic()` → `resetEncoder()` retries and
    produces a clean offset
  - Sim test: freeze one encoder in the sim, run predict(), assert `dTheta` is 0 (or equal to
    the unfrozen-wheel-only estimate)
  - Sim test: wedged wheel → `enc_omega` observation 0 in the EKF update (T003 coupling)
- **Verification command**: `uv run --with pytest python -m pytest host_tests/ host/tests/`

## Implementation Plan

### Approach

Work item by item in order (a → e); each is independently testable.

**(a)** In `Motor::resetEncoder()`: after setting `_encOffset`, call `readEncoderAtomic()`
again and check |result| < threshold (e.g. 2 mm equivalent). If not, retry once or twice.
Optionally take 3 reads before the offset assignment and use the median.

**(b)** In `Robot::controlCollectSplitPhase()`: add `_filterRejectStreakL/R` counters (uint8).
Increment on each outlier rejection, reset on acceptance. When streak crosses threshold (e.g. 3),
emit `EVT enc_filter_hold wheel=L/R streak=N` (or add a field to the existing TLM).

**(c)** In `MotorController::controlTick()`, before the `EVT enc_wedged` emit: read the raw
encoder value (call `_motorL/_motorR.readEncoderMm()` or equivalent raw path) and include it
as `raw=N` in the format string.

**(d)** In `MotorController::controlTick()`: add a per-wheel "started moving" latch. On command
start (when tgt changes from 0 to non-zero, or on `reset()`), clear the latch. Set it when the
encoder delta > 0 for the first time. Gate the wedge increment (`if (_stuckCountL < 255)
++_stuckCountL`) on the latch being set.

**(e)** In `MotorController.h`: add `bool wheelWedgedL() const { return _wedgeEmittedL; }`
and `bool wheelWedgedR() const { return _wedgeEmittedR; }`. Pass these flags into
`Odometry::predict()` (add parameters or a struct). In `predict()`, when `wedgedL || wedgedR`,
set `dTheta = 0` before the EKF predict call. Also pass the wedge flags to the `enc_omega`
gate added in T003.

### Files to Modify

- `source/hal/Motor.cpp` — readback verification in `resetEncoder()` (item a)
- `source/robot/Robot.cpp` — filter-hold streak counter (item b)
- `source/control/MotorController.cpp` — raw field in wedge EVT (item c); arming grace (item d)
- `source/control/MotorController.h` — `wheelWedgedL()/R()` accessors (item e)
- `source/control/Odometry.cpp` — dTheta suppression + wire enc_omega gate from T003 (item e)
- `source/control/Odometry.h` — update `predict()` signature if needed (item e)
- `host_tests/` — add three new sim tests

### Documentation Updates

Update `MotorController.h` comment on `_wedgeEmittedL/R` to note these are exposed via the
new accessors for odometry defense.
