---
id: '007'
title: 'Refactor Robot: expose granular task entry points; add RobotStateContainer
  ownership'
status: done
use-cases:
- SUC-001
- SUC-003
- SUC-004
- SUC-006
depends-on:
- '003'
- '004'
- '005'
- '006'
github-issue: ''
issue: plan-single-cooperative-main-loop-abandon-fibers.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Refactor Robot: expose granular task entry points; add RobotStateContainer ownership

## Description

Complete the `Robot` refactor: formalize the `RobotStateContainer` as a
declared member (the stubs from tickets 003–005 already wrote into it);
remove the legacy `controlTick`/`telemetryTick` public methods; and finalize
all ten task entry-point methods that `LoopScheduler` calls.

Tickets 003–005 introduced `Robot::controlCollect()`, `odometryPredict()`,
`otosCorrect()`, `driveAdvance()` as stubs or partial implementations. This
ticket completes them, adds the remaining ones (`commsIn()`, `lineRead()`,
`colorRead()`, `portsRead()`, `telemetryEmit()`, `controlFireRequest()`), and
removes the legacy two-fiber API (`controlTick`, `telemetryTick`).

The telemetry-emit task reads from `_state.inputs.*` (sensor snapshots in
`HardwareState`) rather than calling I2C directly, eliminating the duplicate
sensor reads that existed in `Robot::telemetryTick()`.

## Files to Modify

- `source/robot/Robot.h` — declare `RobotStateContainer _state`; add all task
  entry-point declarations; remove `controlTick` / `telemetryTick`.
- `source/robot/Robot.cpp` — implement the complete set of task entry points;
  remove old tick implementations; update `telemetryTick` → `telemetryEmit`
  to read from `_state.inputs` instead of calling sensor I2C directly.

## Acceptance Criteria

- [x] `Robot` has a `RobotStateContainer _state` member (declared in `Robot.h`).
- [x] `Robot` exposes exactly these task entry points (all `public`):
  - `controlCollect(uint32_t now_ms)` — collect encoder, compute velocity,
    run PID, write PWM.
  - `controlFireRequest()` — fire encoder request for the alternating wheel.
  - `commsIn()` — implemented in `LoopScheduler` (per plan decision; Robot
    free of CommandProcessor dependency).
  - `driveAdvance(uint32_t now_ms)` — advance drive FSMs, emit inline EVTs.
  - `odometryPredict()` — `Odometry::predict` from `_state.inputs`.
  - `otosCorrect(uint32_t now_ms)` — read OTOS, write `_state.inputs.otos*`,
    call `Odometry::correct`.
  - `lineRead()` — read line sensor into `_state.inputs.line[4]`.
  - `colorRead()` — read color sensor into `_state.inputs.colorR/G/B/C`.
  - `portsRead()` — read digital/analog ports into `_state.inputs.digitalIn/analogIn`.
  - `telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx)` — assemble TLM
    frame from `_state.inputs` (no direct I2C calls).
- [x] `Robot::controlTick(uint32_t now_ms)` is removed.
- [x] `Robot::telemetryTick(uint32_t now_ms, ReplyFn, void*)` is removed.
- [x] The `telemetryEmit` implementation reads `_state.inputs.line`,
  `_state.inputs.colorR/G/B/C`, `_state.inputs.pose*` rather than calling
  sensor I2C.
- [x] `uv run --with pytest python -m pytest` passes — specifically
  `test_tlm_stream.py` (telemetry reads from snapshots).
- [x] Firmware builds cleanly.

## Implementation Plan

1. In `Robot.h`, add `RobotStateContainer _state;` as a private member (after
   the existing subsystem members). Add `#include "RobotState.h"`.
2. Add the ten task entry-point declarations to the public interface.
3. Remove `controlTick` / `telemetryTick` declarations.
4. In `Robot.cpp`, implement the remaining entry points:
   - `commsIn()`: drain serial (while `_serial.readLine(buf)`) and radio
     (while `_radio.poll(buf)`) into `_cmd.process()` — but `commsIn()` needs
     a reference to `CommandProcessor`. Either `Robot` stores a `CommandProcessor*`
     (set by `LoopScheduler`) or `commsIn()` is implemented in `LoopScheduler`
     directly (preferred — keeps `Robot` free of `CommandProcessor` dependency).
     **Decision**: Move `commsIn` to `LoopScheduler` (it already holds `_cmd`);
     `Robot` does not need this entry point.
   - `lineRead()`: call `_line.readValues(_state.inputs.line)` if `_linePresent`;
     update `_state.inputs.lineVS.lastUpdMs`.
   - `colorRead()`: call `_color.pollRGBC(...)` if `_colorPresent`; write results.
   - `portsRead()`: read digital/analog inputs into `_state.inputs.digitalIn/analogIn`.
   - `telemetryEmit(now_ms, fn, ctx)`: copy existing TLM assembly from
     `telemetryTick` but replace direct sensor I2C calls with reads from
     `_state.inputs.*`.
5. Remove `controlTick()` and `telemetryTick()` implementations.
6. Verify all callers updated — the only caller was `main.cpp`, updated in
   ticket 008.

## Testing Plan

- **Build verification**: `python build.py` — no new errors.
- **Automated tests**: `uv run --with pytest python -m pytest` — full suite
  must pass. Focus on `test_tlm_stream.py` for telemetry snapshot behavior.
- **Hardware bench**: Deferred to ticket 009.
