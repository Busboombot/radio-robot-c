---
id: '005'
title: 'D9: OTOS validity gating and hardware smoke ritual'
status: open
use-cases:
  - SUC-004
  - SUC-006
depends-on:
  - "027-002"
  - "027-003"
  - "027-004"
github-issue: ''
issue:
- d09-otos-validity-gating.md
- hardware-smoke-ritual-and-field-log.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 027-005: D9 — OTOS validity gating and hardware smoke ritual

## Description

`OtosSensor::readTransformed` and `readVelocityTransformed` never read the
chip's STATUS register (0x1F). A lifted or just-placed robot feeds zeros or
garbage into the EKF. The velocity update (v=0, ω=0) sits inside the χ²
gate and drags fused velocity to zero while the controller fights it. The
position update may be Mahalanobis-gated but the velocity update is not.
Heading and velocity poisoning during PRE_ROTATE causes the gate never to
close, producing the "spin on placement" failure.

Additionally, I2C read failures leave int16 buffers at 0 with no error
signal. The mounting-offset transform subtracts offsets as world constants
instead of applying the lever-arm rotation (dormant — offsets are zero in
`tovez.json` — but wrong).

This ticket also delivers the hardware smoke ritual (`tests/bench/smoke_ritual.py`)
because check_4 (lift test → EVT otos lost) requires D9 to have landed.

Do a `--clean` build before every bench verification in this ticket.

## Acceptance Criteria

### D9 firmware changes

- [ ] `OtosSensor.h` declares `REG_STATUS = 0x1F` as a private constant.
- [ ] `OtosSensor` adds public method `bool readStatus(uint8_t& out) const`
      that reads register 0x1F via `readReg8` and returns true on I2C success.
- [ ] `OtosSensor` adds private `bool _lastReadOk = false` updated by
      `readXYH`; exposed via `bool lastReadOk() const`.
- [ ] `readTransformed` and `readVelocityTransformed` gain a
      `float headingRad = 0.0f` parameter for the lever-arm fix.
- [ ] `IOtosSensor` virtual interface updated to match the new signatures
      (both methods gain the `headingRad` parameter with default `0.0f`).
- [ ] The mounting-offset lever-arm transform in `readTransformed` is
      corrected: `odomOffX/Y` are applied rotated by `headingRad` (no-op
      for zero offsets).
- [ ] `Robot::otosCorrect()` calls `otos.readStatus(status)` and
      `otos.lastReadOk()` before calling `odometry.correctEKF`; on non-zero
      status or I2C failure, sets `state.inputs.otos.valid = false` and
      returns without fusing.
- [ ] `Robot` gains private fields `uint32_t _otosInvalidStartMs` and
      `bool _otosLostEmitted`.
- [ ] After ~500 ms of continuous OTOS invalidity during active motion
      (i.e., `motionController.hasActiveCommand()` is true),
      `otosCorrect()` emits `EVT otos lost` exactly once per invalidity
      cycle (reset `_otosLostEmitted` when OTOS becomes valid again).
- [ ] `Robot::otosCorrect()` passes `state.inputs.poseHrad` as `headingRad`
      to `otos.readTransformed()` and `otos.readVelocityTransformed()`.
- [ ] `sim_api.cpp` `MockOtosSensor` (or equivalent OTOS model) updated to
      override the new virtual signatures (ignoring `headingRad` is fine —
      offsets are zero in tests).
- [ ] Firmware builds clean.

### EVT emission path (Open Question 3 resolution)

The programmer must resolve how `Robot::otosCorrect()` emits the EVT given
that it has no command-context reply channel. Acceptable paths:
- Call `motionController.emitToActiveChannel("EVT otos lost")` — add this
  method to `MotionController` if it does not exist.
- Or: call the `LoopScheduler`'s broadcast mechanism.
- Document the chosen path in an inline comment.

### Hardware bench checks (requires the robot)

- [ ] Flash firmware (`mbdeploy deploy robot --clean`) after `--clean` build.
- [ ] OTOS reports valid after boot and motion (no spurious invalidity on
      the bench stand).
- [ ] Lift test: hold the robot up off the floor mid-G; within 1 s observe
      `EVT otos lost` in the stream; replace robot on floor; OTOS re-acquires
      (stream shows valid OTOS pose again).
- [ ] No full-speed spin on placement (the TIME net from D5 may still fire if
      the fused heading is wrong, but the velocity poison is removed).

### Smoke ritual

- [ ] `tests/bench/smoke_ritual.py` exists and is executable.
- [ ] Script runs all five checks end-to-end with the robot:
  1. SAFE query — prints PASS if response is `on`.
  2. TURN×4 orientation closure — prints PASS if final heading is within 10°
     of start (uses OTOS heading readback via `GET otosH`).
  3. G square (4 legs, ~300 mm per side) — prints PASS if return-to-start
     error < 50 mm (uses camera or OTOS for ground truth).
  4. Lift test — prints PASS if `EVT otos lost` received and no spin on
     placement (within 5 s window).
  5. TLM drop-rate print — runs STREAM 40 for 10 s, counts frames, reports
     observed rate and any apparent drops.
- [ ] Script appends a dated SHA-stamped entry to
      `docs/knowledge/field-log.md` (creates the file if absent).
- [ ] Script uses `BenchRun` (from 027-002) for any motion commands.

### Regression tests

- [ ] `test_scenario_spin_on_placement` in `test_incident_scenarios.py`:
      with D9 landed, OTOS validity gating means the sim's OTOS-frozen test
      no longer poisons EKF velocity. The test's assertion (no runaway spin)
      should still pass; verify it does not need any adjustment.
- [ ] All `host_tests/` pass.

## Implementation Plan

### Approach

**`OtosSensor` changes:** Add `REG_STATUS` constant, `readStatus()` method,
`_lastReadOk` field. `readXYH` is already a private helper with an implicit
I2C failure risk; update it to set `_lastReadOk = (i2cResult == MICROBIT_OK)`
(check the return type of `_i2c.read()` in `I2CBus.cpp` to confirm the error
code). `readTransformed` adds the `headingRad` parameter and applies the
rotated lever-arm correction.

**`IOtosSensor`:** Add the default-parameter versions of both virtual methods.
All concrete overrides (including mock) gain the parameter and can safely
ignore it.

**`Robot::otosCorrect()` gating:** Add three lines before
`odometry.correctEKF`:
```cpp
uint8_t otosStatus = 0;
bool statusOk = otos.readStatus(otosStatus);
if (!statusOk || otosStatus != 0 || !otos.lastReadOk()) {
    state.inputs.otos.valid = false;
    // EVT otos lost logic (see below)
    return;
}
state.inputs.otos.valid = true;
_otosLostEmitted = false;  // reset when valid
_otosInvalidStartMs = 0;
```

**EVT otos lost:** After the `return` in the invalid path:
```cpp
if (motionController.hasActiveCommand()) {
    if (_otosInvalidStartMs == 0) _otosInvalidStartMs = now_ms;
    if (!_otosLostEmitted && (now_ms - _otosInvalidStartMs) >= 500) {
        // emit EVT otos lost via active channel
        motionController.emitToActiveChannel("EVT otos lost");
        _otosLostEmitted = true;
    }
}
```

**`_startPreRotate` helper (if needed by D8 — check if 027-004 already
added it):** If `_startPreRotate` is already in `MotionController.cpp`
from 027-004, this ticket does not need to add it.

**Smoke ritual script:** Follow the `tests/bench/square_run.py` and
`tests/bench/goto_tag.py` patterns. Use `rogo` via subprocess or the
existing robot API. The lift test check_4 reads the event stream for
`EVT otos lost` within a 5-second window after the robot is lifted
(operator prompt: `input("Lift the robot now and press Enter when lifted...")`,
then poll the stream for 5 s).

### Files to modify/create

- `source/hal/OtosSensor.h` — REG_STATUS, readStatus(), lastReadOk(),
  headingRad parameter.
- `source/hal/OtosSensor.cpp` — implement readStatus(), update readXYH
  _lastReadOk, update readTransformed/readVelocityTransformed lever-arm.
- `source/hal/IOtosSensor.h` — update virtual signatures.
- `source/robot/Robot.h` — new private fields.
- `source/robot/Robot.cpp` — otosCorrect() gating + EVT emission.
- `source/control/MotionController.h` — declare `emitToActiveChannel()` if
  needed.
- `source/control/MotionController.cpp` — implement `emitToActiveChannel()`.
- `host_tests/sim_api.cpp` — update MockOtosSensor virtual override.
- `tests/bench/smoke_ritual.py` — new file.
- `docs/knowledge/field-log.md` — created by smoke ritual on first run.

### Testing plan

```
python3 build.py     # must be --clean
uv run pytest host_tests/ -v
# then with robot connected:
uv run python tests/bench/smoke_ritual.py
```

Confirm: all unit tests pass; smoke ritual check_4 (lift test) shows PASS
with `EVT otos lost` received.

### Documentation updates

`docs/knowledge/field-log.md` first entry is created by the ritual run.
Inline comments in `otosCorrect()` explaining the STATUS register check.

## Notes

- The `emitToActiveChannel` method resolution (Open Question 3) must be
  decided before writing `otosCorrect`. The simplest path: expose
  `emitEvt(const char* evt, TargetState& t)` as a helper that `Robot` can
  call by passing `motionController.target()`. Check whether `MotionController`
  already exposes a `target()` accessor.
- `MockOtosSensor` in `sim_api.cpp` needs the parameter but can ignore it.
  The mock's `readStatus` can always return `true` (success) with
  `statusOut = 0` (valid). This keeps existing tests passing.
- check_3 of the smoke ritual (G square < 50 mm return error) requires the
  camera or an external ground truth. If the camera is not available in the
  bench context, substitute OTOS pose comparison (reset at start, read at
  return-to-origin). Document the choice in the script.
- This is the last firmware ticket; the smoke ritual is the acceptance gate
  for the full sprint on hardware.
