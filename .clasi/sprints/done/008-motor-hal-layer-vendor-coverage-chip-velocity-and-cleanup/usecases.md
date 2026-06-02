---
status: final
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 008 Use Cases

---

## SUC-001: Vendor Advisory Reference In-Repo

- **Actor**: Developer auditing Nezha2 I2C command coverage
- **Preconditions**: `vendor/pxt-nezha2/` directory does not exist in
  `radio-robot-c`.
- **Main Flow**:
  1. Developer opens `vendor/pxt-nezha2/main.ts` to see the full Nezha2
     I2C command surface.
  2. Developer reads `vendor/pxt-nezha2/README` to confirm it is advisory
     (not compiled into firmware) and identifies the authoritative source.
- **Postconditions**: `vendor/pxt-nezha2/main.ts` (and `test.ts`) is
  present in the repo. `README` notes advisory status. File is excluded
  from the build system.
- **Acceptance Criteria**:
  - [ ] `vendor/pxt-nezha2/main.ts` and `test.ts` present.
  - [ ] `vendor/pxt-nezha2/README` describes advisory-only status.
  - [ ] Build succeeds; the TypeScript is never compiled.

---

## SUC-002: Per-Motor Motor Abstraction

- **Actor**: Control-layer code (MotorController, DriveController)
- **Preconditions**: `NezhaV2` is a single object driving both wheels;
  forward direction is hardcoded via `LEFT_FWD`/`RIGHT_FWD` constants.
- **Main Flow**:
  1. Firmware constructs two `Motor` instances — one per wheel — each
     holding its own encoder offset and forward-direction sign from
     `RobotConfig`.
  2. `MotorController` uses two `Motor` instances instead of one
     `NezhaV2` instance.
  3. `RobotConfig` supplies per-motor forward-direction values (`fwdSignL`,
     `fwdSignR`).
- **Postconditions**: `NezhaV2` class and files are renamed to `Motor`.
  Each `Motor` object represents one wheel. Forward direction is
  configurable from `RobotConfig`, not hardcoded. Existing drive behavior
  is unchanged.
- **Acceptance Criteria**:
  - [ ] Class/files renamed: `NezhaV2` → `Motor`.
  - [ ] `RobotConfig` has `int8_t fwdSignL` and `int8_t fwdSignR` (default
    +1 and −1 to preserve current behavior).
  - [ ] `MotorController` and `Robot` updated to use two `Motor` instances.
  - [ ] `LEFT_FWD`/`RIGHT_FWD` constants removed from `Motor`.
  - [ ] Firmware builds. Wheels drive and encoders read correctly on bench.

---

## SUC-003: Chip-Native Wheel Velocity (readSpeed 0x47)

- **Actor**: MotorController (velocity telemetry and future velocity PID)
- **Preconditions**: `Motor` HAL exists (SUC-002 done); `readSpeed`
  register `0x47` is not yet wrapped.
- **Main Flow**:
  1. `MotorController::tick()` requests velocity from the chip via
     `Motor::readSpeed()`.
  2. If the chip returns a valid, plausible reading, it is used as the
     velocity source for that wheel.
  3. If the I2C call fails or the reading is implausible, the fallback
     encoder-delta/dt velocity is used.
  4. The active velocity source (chip or encoder) is exposed via telemetry.
- **Postconditions**: Chip velocity is primary; encoder-delta is fallback.
  The laps→mm/s scale is pinned empirically. A bench validation log
  confirms monotonicity and sign agreement with encoder-derived velocity.
- **Acceptance Criteria**:
  - [ ] `Motor::readSpeedRaw(motorId)` issues correct 8-byte frame for
    `0x47` with 4 ms pre/post delays.
  - [ ] `Motor::readSpeed(bool leftWheel, float& mmPerSec)` applies
    `floor(raw/3.6)*0.01` → laps/s conversion, per-wheel sign, and
    laps→mm/s scale.
  - [ ] `MotorController` uses chip velocity as primary source; encoder
    delta as fallback on I2C error or implausible reading.
  - [ ] Active source (chip/encoder) is exposed in telemetry.
  - [ ] Bench log shows non-zero, correctly-signed chip velocity while
    driving; monotonicity confirmed; laps→mm/s scale pinned.

---

## SUC-004: Full Vendor I2C Command Coverage

- **Actor**: Developer using Nezha2 HAL at full chip capacity
- **Preconditions**: `Motor` HAL exists (SUC-002 done); registers `0x70`,
  `0x5D`, `0x1D`, `0x77`, `0x88` are not wrapped.
- **Main Flow**:
  1. Developer calls `Motor::timedMove()` / `Motor::moveToAngle()` /
     `Motor::resetHome()` / `Motor::setGlobalSpeed()` / `Motor::readVersion()`.
  2. Correct 8-byte I2C frames are sent; `0x5D` enforces its BUG-critical
     4 ms post-write delay with no task interleave.
  3. Developer consults the coverage checklist in `Motor.h` to confirm all
     vendor registers have a corresponding HAL method.
- **Postconditions**: All vendor I2C registers have HAL wrappers. The
  `0x5D` post-write delay is preserved. Coverage checklist is green.
- **Acceptance Criteria**:
  - [ ] `0x70` timed move (turns/deg/sec) wrapped and tested (frame bytes).
  - [ ] `0x5D` abs-angle wrapped; 4 ms post-write no-task-interleave delay
    present and commented.
  - [ ] `0x1D` reset/home wrapped and tested.
  - [ ] `0x77` global servo speed wrapped and tested.
  - [ ] `0x88` firmware version read wrapped; returns plausible version on
    hardware.
  - [ ] Coverage checklist table in `Motor.h` shows all registers green.

---

## SUC-005: Configurable Servo (180° / 360°)

- **Actor**: Robot firmware constructing a servo peripheral
- **Preconditions**: `GripperServo` is hardcoded to 0–180° range.
- **Main Flow**:
  1. Robot constructs a `Servo` with a configurable range (180° or 360°
     continuous-rotation).
  2. `setAngle()` clamps to the configured range.
- **Postconditions**: Class/files renamed `GripperServo` → `Servo`. Range
  is a constructor parameter with a default of 180°. Existing gripper
  behavior is unchanged.
- **Acceptance Criteria**:
  - [ ] Class/files renamed: `GripperServo` → `Servo`.
  - [ ] Constructor accepts `uint16_t maxDegrees` (default 180).
  - [ ] `setAngle()` clamps to `[0, maxDegrees]`.
  - [ ] `Robot` and `CommandProcessor` updated to use `Servo`.
  - [ ] Firmware builds; gripper still responds correctly on bench.

---

## SUC-006: LineSensor Per-Channel Calibration

- **Actor**: Robot operator calibrating the line sensor for a new surface
- **Preconditions**: `LineSensor` returns raw 0–255 grayscale with no
  normalization.
- **Main Flow**:
  1. Operator triggers calibration sweep; robot captures per-channel min
     and max values.
  2. Subsequent `readValues()` calls return normalized readings scaled to
     a consistent range.
  3. Optional smoothing reduces noise in the normalized output.
- **Postconditions**: `LineSensor` holds per-sensor min/max. `readValues()`
  returns normalized, optionally smoothed values. Raw reads are still
  available.
- **Acceptance Criteria**:
  - [ ] `LineSensor` stores per-channel `uint16_t min[4]`, `max[4]`.
  - [ ] `captureCalibMin()` and `captureCalibMax()` snapshot current raw
    readings.
  - [ ] `readNormalized(out[4])` scales each channel to 0–1000 (0=white,
    1000=black).
  - [ ] Optional EMA smoothing configurable per-sensor or globally.
  - [ ] Firmware builds; normalized values respond correctly to line/no-line
    on bench.
