---
id: '005'
title: 'BVC unification: S and G PRE_ROTATE through BVC + system watchdog'
status: done
use-cases:
- SUC-005
- SUC-006
depends-on:
- 020-001
github-issue: ''
issue: issue-motion-system-overhaul.md
completes_issue: false
---

# BVC unification: S and G PRE_ROTATE through BVC + system watchdog

## Description

Route the two remaining direct-to-motor code paths through BVC, and replace the two
per-mode keepalive watchdogs with a single system watchdog on LoopScheduler.

**S command (`beginStream`)**: currently calls `MotorController::setTarget` directly.
Change to call `BodyKinematics::forward(vL, vR)` → `(v, ω)`, then
`bvc.seedCurrent(v, omega)` + `bvc.setTarget(v, omega)`. Remove `_lastSMs` member.

**G PRE_ROTATE**: currently calls `mc.startDriveClean(sL, sR)` directly. Change to
`bvc.seedCurrent(0, omega)` + `bvc.setTarget(0, omega)`.

**System watchdog**: add `uint32_t _watchdogMs` to LoopScheduler. Reset in
`runCommsIn()` on every inbound command. Check at the top of each tick: if
`now_ms - _watchdogMs > robot.config.sTimeoutMs`, emit `EVT safety_stop` and inject
`cmd.process("X", activeFn, activeCtx)`. Remove VW's embedded TIME keepalive stop.

Also remove `MotionCommand::armTime()` and `setDoneEvt()` as they only served VW's keepalive.

## Acceptance Criteria

- [x] `beginStream` uses `BodyKinematics::forward()` then `bvc.seedCurrent + bvc.setTarget`; no direct `MotorController::setTarget` call.
- [x] `MotionController::_lastSMs` member removed; no S-mode `_lastSMs` check in `driveAdvance`.
- [x] G PRE_ROTATE path uses `bvc.seedCurrent(0, omega) + bvc.setTarget(0, omega)`.
- [x] `LoopScheduler` has `uint32_t _watchdogMs` field; reset in `runCommsIn()` on every command.
- [x] Watchdog fires after `sTimeoutMs` of inbound silence: emits `EVT safety_stop`; injects `cmd.process("X", ...)`.
- [x] VW's embedded TIME stop condition and keepalive re-arm logic removed from `MotionController::beginVelocity`.
- [x] `MotionCommand::armTime()` removed; `setDoneEvt()` retained (used by T/D/G/R/TURN commands).
- [ ] Bench verification: `S 100 100` drives normally; keepalive timeout fires `EVT safety_stop` with no `_lastSMs` path.
- [ ] Bench verification: `VW v=300 w=0` + send `+` keepalives keeps robot running; stop keepalives → `EVT safety_stop`.
- [ ] Bench verification: `G x=500 y=0 speed=200` drives to position; PRE_ROTATE works.
- [x] `python3 build.py --clean` passes.
- [x] `uv run --with pytest python -m pytest` passes.

## Implementation Plan

### Approach

1. In `MotionController.cpp`: rewrite `beginStream` to use forward kinematics + BVC.
   Remove `_lastSMs` from header and all uses. Rewrite PRE_ROTATE block.
2. In `MotionController.cpp`: remove VW keepalive TIME stop from `beginVelocity`.
3. In `MotionCommand.h/.cpp`: remove `armTime()` and `setDoneEvt()`.
4. In `LoopScheduler.h/.cpp`: add `_watchdogMs`; add reset call in `runCommsIn()`;
   add watchdog check at top of `run_blocks()` loop body.
5. Build and bench verify S, VW, G.

### Files to Modify

- `source/control/MotionController.h` — remove `_lastSMs`
- `source/control/MotionController.cpp` — rewrite `beginStream`; remove S-mode watchdog; remove VW keepalive; rewrite PRE_ROTATE
- `source/control/MotionCommand.h` — remove `armTime()`, `setDoneEvt()` declarations
- `source/control/MotionCommand.cpp` — remove implementations
- `source/control/LoopScheduler.h` — add `_watchdogMs`
- `source/control/LoopScheduler.cpp` — reset watchdog in `runCommsIn()`; watchdog check in tick body

### Testing Plan

1. `python3 build.py --clean` — zero warnings.
2. Flash via `mbdeploy deploy robot --clean`.
3. Bench: `S 100 100` → drives; silence > sTimeoutMs → `EVT safety_stop`.
4. Bench: `VW v=300 w=0` + `+` keepalives → motor runs; stop keepalives → `EVT safety_stop`.
5. Bench: `G x=500 y=0 speed=200` → robot drives to position normally.
6. `uv run --with pytest python -m pytest` — no regressions.

### Notes

- `BodyKinematics::forward(float vL, float vR, float trackwidthMm, float& v, float& omega)` —
  verify this function exists in `BodyKinematics.h/.cpp` and its signature before using it.
  If it does not exist, add it (it is the inverse of the existing `inverse()` function).
- The watchdog check must use signed delta: `int32_t delta = (int32_t)(now_ms - _watchdogMs);`
  to avoid the uint32 underflow bug documented in memory notes.
- The PRE_ROTATE ω should be computed from the existing `rotationGainPos/Neg` config
  values (same math as before, just fed to BVC instead of direct setTarget).
- Do NOT remove the MotionCommand stop array used by T/D/G/TURN — only VW's keepalive
  is removed.
