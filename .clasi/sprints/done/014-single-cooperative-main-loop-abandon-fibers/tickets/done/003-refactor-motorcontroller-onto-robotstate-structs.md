---
id: '003'
title: Refactor MotorController onto RobotState structs
status: done
use-cases:
- SUC-001
- SUC-003
depends-on:
- '001'
- '002'
github-issue: ''
issue: plan-single-cooperative-main-loop-abandon-fibers.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Refactor MotorController onto RobotState structs

## Description

Slim `MotorController` from a self-contained tick loop into a control-task
orchestrator that reads/writes the `RobotStateContainer` structs. The private
encoder and velocity cache fields (`_encLMm`, `_encRMm`, `_actualVelL`,
`_actualVelR`, `_usingChipVelL`, `_usingChipVelR`) move out of `MotorController`
and into `HardwareState`.

Replace `MotorController::tick(float dt_s)` with a new entry point
`controlTick(HardwareState& inputs, MotorCommands& cmds, float dt_s)` that
reads encoder/velocity values from `inputs` and writes PWM results to `cmds`.
The previous-encoder snapshot (`_prevEncL`, `_prevEncR`) stays as a member of
`MotorController` — it is intermediate compute state needed to differentiate
velocity, not robot state.

The control-task collect step (calling `Motor::collectEncoder()` and converting
to mm) belongs in `Robot::controlCollect()`, which writes `inputs.encLMm/R`
before calling `MotorController::controlTick()`.

## Files to Modify

- `source/control/MotorController.h` — replace `tick()` with `controlTick()`;
  remove the six private cache fields; update query methods accordingly.
- `source/control/MotorController.cpp` — implement `controlTick()`; remove
  `encoderMm()` and the encoder-read call chain.
- `source/robot/Robot.{h,cpp}` — add `controlCollect(now_ms)` that calls
  `Motor::collectEncoder()`, converts to mm, writes `_state.inputs.enc*`, then
  calls `_mc.controlTick(...)`. Keep `Robot::controlTick()` stub calling
  `controlCollect()` so `main.cpp` compiles unchanged until ticket 007.

## Acceptance Criteria

- [x] `MotorController` has no `_encLMm`, `_encRMm`, `_actualVelL`,
  `_actualVelR`, `_usingChipVelL`, `_usingChipVelR` private fields.
- [x] `MotorController::controlTick(HardwareState&, MotorCommands&, float)`
  reads `inputs.encLMm/R`, writes `inputs.velLMms/R`, runs `VelocityController`
  ×2, writes `cmds.pwmL/R`, and calls `Motor::setSpeed()`.
- [x] `MotorController::tick(float)` (old signature) is removed.
- [x] `encoderMm()` private helper is removed.
- [x] Firmware builds cleanly; `main.cpp` continues to compile without changes.
- [x] `uv run --with pytest python -m pytest` passes — specifically
  `test_readspeed_and_get_vel.py`, `test_vw_command.py`,
  `test_saturation_wiring.py`.

## Implementation Plan

1. Add `controlTick(HardwareState& inputs, MotorCommands& cmds, float dt_s)` to
   `MotorController`:
   - Read `encLMm = inputs.encLMm`, `encRMm = inputs.encRMm`.
   - Compute `encVelL = (encLMm - _prevEncL) / dt_s`; update `_prevEncL/R`.
   - Write `inputs.velLMms = encVelL`, `inputs.velRMms = encVelR`.
   - Run `_vcL.update(cmds.tgtLMms, inputs.velLMms, dt_s)` → `uL`;
     run `_vcR` → `uR`.
   - Write `cmds.pwmL = clamp(uL)`, `cmds.pwmR = clamp(uR)`.
   - Call `_motorL.setSpeed(cmds.pwmL)`, `_motorR.setSpeed(cmds.pwmR)`.
2. Remove `tick(float dt_s)`, `encoderMm()`, and the six private cache fields.
3. Update `getActualVelocity()` to take `const HardwareState&`, or remove it
   (callers read `inputs.velLMms/R` directly from `Robot::state()`).
4. In `Robot.cpp`, implement `controlCollect(now_ms)`:
   - Call `_motorL.collectEncoder()`, convert to mm via `mmPerDegL`,
     write `_state.inputs.encLMm`.
   - Same for right wheel.
   - Compute `dt_s` from `_lastControlMs`; call
     `_mc.controlTick(_state.inputs, _state.commands, dt_s)`.
5. Update `Robot::controlTick()` to call `controlCollect()` to keep
   `main.cpp` compiling unchanged.

## Testing Plan

- **Build verification**: `python build.py` — no new errors.
- **Automated tests**: `uv run --with pytest python -m pytest` — focus on
  `test_readspeed_and_get_vel.py`, `test_vw_command.py`,
  `test_saturation_wiring.py`.
- **Hardware bench**: Deferred to ticket 009.
