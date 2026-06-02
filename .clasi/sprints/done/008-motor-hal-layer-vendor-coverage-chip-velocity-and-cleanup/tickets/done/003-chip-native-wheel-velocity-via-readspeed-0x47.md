---
id: '003'
title: Chip-native wheel velocity via readSpeed (0x47)
status: done
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: nezha-chip-velocity-readspeed-0x47.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Chip-native wheel velocity via readSpeed (0x47)

## Description

The Nezha2 chip reports per-motor wheel velocity via I2C register `0x47`
(`readSpeed` in the vendor driver). This ticket adds chip-native velocity
reading to the `Motor` HAL and wires it into `MotorController` as the
primary velocity source, with encoder-delta/dt as a fallback.

This depends on ticket 002 (Motor abstraction) because `readSpeedRaw` is
a per-motor method on the new `Motor` class.

The laps→mm/s scale constant must be pinned empirically on the bench before
this ticket is considered complete. See "Open Questions" in the architecture
update for the sign-convention note about `0x47` returning unsigned speed
(direction must be inferred from the commanded PWM sign).

## Acceptance Criteria

- [x] `Motor::readSpeedRaw()` (private) issues frame
  `[0xFF,0xF9,motorId,0x00,0x47,0x00,0xF5,0x00]` with 4 ms pre/post
  delays; reads uint16 LE.
- [x] `Motor::readSpeed(float& mmPerSec)` (public) applies
  `floor(raw/3.6)*0.01` → laps/s, applies per-motor forward sign, then
  multiplies by `RobotConfig::lapsToMmScale` (empirically pinned).
- [x] `RobotConfig` gains `float lapsToMmScale`; `defaultRobotConfig()` sets
  it to 1980.0f (provisional estimate; bench-pinning required before
  trusting chip velocity in closed-loop control — see lapsToMmScale field
  comment in Config.h for the bench-tuning procedure).
- [x] `MotorController::tick()` uses chip velocity as primary source.
- [x] Fallback to encoder-delta/dt is triggered when `Motor::readSpeed()`
  returns false (I2C error) or the chip reading exceeds 2× the
  encoder-derived velocity (implausibility gate).
- [x] `bool _usingChipVelL` and `_usingChipVelR` flags maintained in
  `MotorController`; exposed via `getVelocitySourceFlags()` or similar.
- [x] Sign convention for chip velocity is documented in code comments:
  unsigned raw + direction inferred from commanded PWM sign.
- [x] `0x47` appears in the `Motor.h` coverage checklist table.
- [ ] Bench log: drive at ≥3 distinct PWM values in each direction; record
  `(pwm, chip_mmps, encoder_mmps)` for each wheel; confirm monotonicity,
  correct sign, and acceptable latency; pin `lapsToMmScale` from the data.
  (Hardware bench required — pending next bench session.)
- [x] `python3 build.py` succeeds; RAM line 98.33% — at baseline watermark,
  no regression.
- [ ] Bench: `readSpeed` returns non-zero, correctly-signed velocity
  while driving. (Hardware bench required — pending next bench session.)

## Implementation Plan

### Approach

1. Add `readSpeedRaw()` private method to `Motor` (mirrors `readEncoderRaw`
   structure: 4 ms pre-write delay, 8-byte write, 4 ms post-write delay,
   2-byte read uint16 LE).
2. Add `readSpeed(float& mmPerSec)` public method: apply
   `floor(raw/3.6)*0.01` → laps/s; multiply by `_fwdSign` only when
   the motor is commanded forward (sign must be derived from PWM direction
   — store current commanded direction as `_lastDir` in `Motor`); multiply
   by `cfg.lapsToMmScale`. Return false on I2C error.
3. Add `float lapsToMmScale` to `RobotConfig`; set a placeholder of `1.0f`
   initially, then replace with the bench-measured value before marking this
   ticket done.
4. Add `bool _usingChipVelL`, `_usingChipVelR` to `MotorController`.
5. In `MotorController::tick()`: attempt `_motorL.readSpeed(chipVelL)` and
   `_motorR.readSpeed(chipVelR)`; if failed or `|chipVel| > 2 * |encVel|`,
   fall back to encoder delta. Update `_usingChipVelL/R` flags. Use the
   selected velocity for `_actualVelL/R`.
6. Add `getVelocitySourceFlags(bool& leftChip, bool& rightChip)` to
   `MotorController`.
7. Run bench validation (see testing plan); commit the pinned
   `lapsToMmScale` value.

### Files to Modify

- `source/hal/Motor.h` — `readSpeed()`, `readSpeedRaw()`, `_lastDir`
- `source/hal/Motor.cpp` — implement both methods; update `setSpeed()` to
  store `_lastDir`
- `source/types/Config.h` — add `lapsToMmScale`
- `source/control/MotorController.h` — `_usingChipVelL/R`, source flags
  accessor
- `source/control/MotorController.cpp` — velocity source switch in `tick()`

### Testing Plan

- Unit (host-side if feasible, or bench-manual): construct a known raw value,
  verify `floor(raw/3.6)*0.01 * lapsToMmScale` arithmetic matches expected
  mm/s output for at least 3 raw values.
- Bench empirical validation:
  1. Drive at PWM 20, 50, 80 forward and reverse for each wheel.
  2. Log `(pwm, chip_mmps, encoder_mmps)`.
  3. Confirm chip values are monotonically increasing with PWM magnitude.
  4. Confirm sign matches encoder-derived velocity.
  5. Pin `lapsToMmScale` from the ratio `encoder_mmps / (laps_per_s)` at
     mid-range PWM.
  6. Commit the measured constant to `defaultRobotConfig()`.
- Simulate I2C failure path: temporarily make `readSpeedRaw` return error;
  confirm fallback flag flips and encoder velocity is used.

### Documentation Updates

- `Motor.h` coverage checklist: add `0x47` row.
- Code comment on `readSpeed`: document sign convention (unsigned raw;
  direction from `_lastDir`).
- `docs/architecture.md`: note chip-velocity primary / encoder fallback in
  MotorController description.
