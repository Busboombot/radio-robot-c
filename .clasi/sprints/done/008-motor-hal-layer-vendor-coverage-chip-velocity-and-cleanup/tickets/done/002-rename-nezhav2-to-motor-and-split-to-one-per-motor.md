---
id: '002'
title: Rename NezhaV2 to Motor and split to one-per-motor
status: done
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: source-fixme-cleanup.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Rename NezhaV2 to Motor and split to one-per-motor

## Description

This is the backbone ticket for sprint 008. `NezhaV2` is a two-wheel
board-level object; the FIXME markers called for a per-motor abstraction
where each object owns one channel's encoder offset and forward sign.

Changes:
- Rename `source/hal/NezhaV2.{h,cpp}` → `source/hal/Motor.{h,cpp}`.
- Rewrite `Motor` to represent a single motor channel. Constructor takes
  `MicroBitI2C&`, `uint8_t motorId` (1=M1/right, 2=M2/left), and
  `int8_t fwdSign` (from `RobotConfig`).
- Replace the shared `int32_t _encOffset[4]` with a single `int32_t _encOffset`.
- Remove `LEFT_FWD`/`RIGHT_FWD` hardcoded constants from `Motor`.
- Add `int8_t fwdSignL` and `int8_t fwdSignR` to `RobotConfig` (defaults
  `+1` and `−1` to preserve existing behavior).
- `RobotConfig::defaultRobotConfig()` updated with the new fields.
- `Robot` constructs `Motor _motorL` (motorId=2, fwdSign=cfg.fwdSignL) and
  `Motor _motorR` (motorId=1, fwdSign=cfg.fwdSignR) instead of one
  `NezhaV2 _motor`.
- `MotorController` takes `Motor& left, Motor& right`; update all internal
  calls accordingly. `setPwm` logic moves inside each `Motor` object.
- `CommandProcessor` updated for renamed accessor if needed.
- Update `#include "NezhaV2.h"` → `#include "Motor.h"` everywhere.
- Update `docs/architecture.md` HAL section: `NezhaV2` → `Motor`.

## Acceptance Criteria

- [x] `source/hal/NezhaV2.{h,cpp}` deleted; `source/hal/Motor.{h,cpp}` exists.
- [x] `Motor` constructor signature: `Motor(MicroBitI2C&, uint8_t motorId, int8_t fwdSign)`.
- [x] `Motor` owns a single `int32_t _encOffset` (not an array of 4).
- [x] `LEFT_FWD` and `RIGHT_FWD` constants removed from `Motor`.
- [x] `RobotConfig` has `int8_t fwdSignL` (default +1) and `int8_t fwdSignR`
  (default -1); `defaultRobotConfig()` sets them.
- [x] `Robot` constructs two `Motor` instances; `NezhaV2 _motor` member removed.
- [x] `MotorController` constructor: `MotorController(Motor& left, Motor& right,
  const RobotConfig& cal)`.
- [x] All `NezhaV2` includes and references replaced with `Motor` throughout.
- [x] `python3 build.py` produces `MICROBIT.hex` without errors.
- [x] RAM line reported from build output; must be <= prior baseline from ticket 001.
  RAM: 120768 B / 122816 B = 98.33% — exactly at baseline, no regression.
- [ ] Bench: wheels drive forward/backward; encoders read correctly (sign and magnitude
  match pre-refactor behavior). (Requires hardware bench — deferred to deployment.)

## Implementation Plan

### Approach

1. Create `source/hal/Motor.h` — single-motor interface. Copy protocol
   constants from `NezhaV2.h` (ADDR, DIR_CW, DIR_CCW); replace `LEFT_MOTOR`/
   `RIGHT_MOTOR` with a constructor-provided `_motorId`; replace
   `LEFT_FWD`/`RIGHT_FWD` with constructor-provided `_fwdSign`; shrink
   `_encOffset` to a scalar.
2. Create `source/hal/Motor.cpp` — port `setPwm` to single-motor
   `setSpeed(int8_t pct)` (applies `_fwdSign` internally); port
   `readEncoderRaw` and `readEncoder` unchanged except use `_motorId` and
   `_fwdSign`; port `resetEncoders` to reset only `_encOffset`.
3. Update `source/types/Config.h` — add `fwdSignL`, `fwdSignR`,
   `defaultRobotConfig()` sets them.
4. Update `source/control/MotorController.{h,cpp}` — change member from
   `NezhaV2& _motor` to `Motor& _motorL, _motorR`; update `encoderMm(bool)`,
   `stop()`, and the `setPwm` call in `tick()` to call `_motorL.setSpeed()`
   and `_motorR.setSpeed()` separately.
5. Update `source/robot/Robot.{h,cpp}` — replace `NezhaV2 _motor` with
   `Motor _motorL, _motorR`; update constructor args for `MotorController`.
6. Update all `#include` references; delete `NezhaV2.{h,cpp}`.
7. Update `docs/architecture.md`.

### Files to Create

- `source/hal/Motor.h`
- `source/hal/Motor.cpp`

### Files to Modify

- `source/types/Config.h` — add `fwdSignL`, `fwdSignR`
- `source/control/MotorController.h` — constructor + member types
- `source/control/MotorController.cpp` — call sites
- `source/robot/Robot.h` — member declarations + includes
- `source/robot/Robot.cpp` — construction + accessor
- `source/app/CommandProcessor.cpp` — include and accessor updates (if any)
- `docs/architecture.md` — HAL section

### Files to Delete

- `source/hal/NezhaV2.h`
- `source/hal/NezhaV2.cpp`

### Testing Plan

- `python3 build.py` must succeed; report RAM line.
- Bench: drive forward and reverse; read encoders via the `E` command;
  confirm sign and magnitude match pre-refactor behavior.
  Specifically: left wheel forward = positive encoder delta; right wheel
  forward = positive encoder delta.

### Documentation Updates

- `docs/architecture.md`: HAL layer diagram and `NezhaV2` subsystem
  description updated to `Motor`.
