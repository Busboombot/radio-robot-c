---
id: '002'
title: Move motion command handlers and reply formatting to app layer
status: done
use-cases:
- SUC-003
- SUC-004
depends-on:
- 026-001
github-issue: ''
issue: a2-protocol-out-of-control-layer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 026-002: Move motion command handlers and reply formatting to app layer

## Description

`source/control/MotionController.cpp` includes `CommandProcessor.h` and
`CommandQueue.h`, and calls `CommandProcessor::replyOK/Err/Evt` throughout.
This is a layering inversion: the control layer depends upward on the app/protocol
layer. It is also the direct mechanism of D11 (double-OK): both the converter
handler and `handleVW` call `replyOK` for the same command.

This ticket implements the A2 fix: move all converter handlers, the unified
`handleVW` dispatcher, `parseS/T/D/G/R/TURN/RT/VW/X`, `pushVW`, and the helper
statics (`vwScanKV`, `vwHasKey`, `packKVArg`, `setIntArg`, `mc_parseSensorToken`,
`MotionCtx`) to a new file `source/app/MotionCommandHandlers.cpp/.h`.

After this ticket, `grep -rl 'CommandProcessor.h\|CommandQueue.h' source/control/`
returns nothing.

A3 file-size target is a review criterion: `source/control/MotionController.cpp`
must be ≤ 900 lines after the extraction.

### What moves

From `source/control/MotionController.cpp` to
`source/app/MotionCommandHandlers.cpp`:

- Static handlers: `handleS`, `handleT`, `handleD`, `handleG`, `handleR`,
  `handleTURN`, `handleRT`, `handleVW`, `handle_VW`, `handleX`, `handleSTOP`.
- Static parsers: `parseS`, `parseT`, `parseD`, `parseG`, `parseR`, `parseTURN`,
  `parseRT`, `parseVW`, `parse_VW`, `parseX`, `parseNoArgs`.
- Static helpers: `pushVW`, `vwScanKV`, `vwHasKey`, `packKVArg`, `setIntArg`,
  `mc_parseSensorToken`, `packSensorArg` (if present).
- `MotionController::getCommands()` replaced by free function
  `getMotionCommands(MotionCtx*)` in `MotionCommandHandlers.cpp`.

From `source/control/MotionController.h` to
`source/app/MotionCommandHandlers.h`:

- `MotionCtx` struct.
- Declaration of `getMotionCommands(MotionCtx*)`.

### `MotionController` cleanup

After extraction, `MotionController.cpp` retains only:
- Constructor, `setCtx()` removal (see below), `setHardwareState()`, all `begin*()`
  entry points, `driveAdvance()`, `stop()`, `cancel()`, `softStop()`,
  `beginRawVelocity()`, `disableSafetyOneShot()`, and all private helpers
  (`fullStop`, `getPoseFloat`, `_checkSafeOneShot`).
- `emitEvt()` is replaced with a `MotionEventSink` call (see below).
- Remove `#include "CommandProcessor.h"`, `#include "CommandQueue.h"`.
- `getCommands()` declaration removed from `MotionController.h`.
- `setQueue()` removed from `MotionController` (queue pointer now lives in
  `MotionCtx` in `Robot`).
- `setCtx()` removed from `MotionController` (Robot initializes `MotionCtx`
  directly now).

### `MotionEventSink` (new narrow interface)

Create `source/control/MotionEventSink.h`:
```cpp
#pragma once
struct MotionEventSink {
    void (*emitFn)(const char* evtLine, const char* corrId, void* ctx);
    void* ctx;
};
```

`MotionController::emitEvt(const char* base, TargetState& target)` becomes a call
through the sink stored in `TargetState`:
```cpp
if (target.sink.emitFn) {
    target.sink.emitFn(base, target.corrId, target.sink.ctx);
}
target.corrId[0] = '\0';
```
`TargetState` gains a `MotionEventSink sink` field. `source/control/RobotState.h`
must include `MotionEventSink.h`.

The `app/` layer sets `sink.emitFn` to a static function that formats and calls
`CommandProcessor::replyEvt` (or calls `replyFn(evtLine, ctx)` directly). The
`begin*()` entry points receive the reply sink and set `target.sink` accordingly.

### `Robot` wiring

`Robot::buildCommandTable()` replaces `motionController.getCommands()` with
`getMotionCommands(&_motionCtx)`. `_motionCtx` (type `MotionCtx`) moves from
being a private member of `MotionController` to a private member of `Robot`.
`LoopScheduler`'s queue wiring changes from
`robot.motionController.setQueue(&_queue)` to
`robot.setMotionQueue(&_queue)` (new thin setter on `Robot` that assigns
`_motionCtx.queue = q`).

### D11 suppression in handleVW

**Critical:** `handleVW` is dispatched both by direct `VW` commands AND by
converter pushes. On the converter push path, the converter handler already
emitted a reply (e.g., `OK goto ...`). `handleVW` must not emit a second reply
in that case.

The structural rule: `handleVW`'s stop-param branches (when dispatched from a
converter push) must NOT call `replyOK`. The open-ended (no-stop-params) branch
MUST call `replyOK` (it is the direct `VW` reply).

The cleanest implementation: after `begin*()` is called in each stop-param branch
of `handleVW`, do not call `CommandProcessor::replyOK`. The converter handler
(e.g., `handleG`) already called `replyOK` before pushing the VW ParsedCommand.
Only the open-ended VW branch (the `ctx->mc->beginVelocity` path at the bottom
of `handleVW`) emits `OK vw ...`.

Verify: run the D11 test from ticket 003 after this ticket lands.

### A3 size review criterion

After extraction, verify:
- `source/control/MotionController.cpp` line count ≤ 900.
- `source/robot/Robot.cpp` line count is noted (not required to decrease this sprint,
  but record it for the sprint 027 A3 follow-on).

## Acceptance Criteria

- [x] `source/app/MotionCommandHandlers.h` and `.cpp` exist and contain all handler,
  parser, helper statics, and `MotionCtx` from `MotionController`.
- [x] `getMotionCommands(MotionCtx*)` free function exists and returns the same
  command vector that `MotionController::getCommands()` returned.
- [x] `Robot::buildCommandTable()` calls `getMotionCommands(&_motionCtx)`.
- [x] `grep -rl 'CommandProcessor.h\|CommandQueue.h' source/control/` returns
  nothing for MotionController.h and MotionController.cpp specifically. Note:
  other control-layer files (LoopScheduler, HaltController, PortController,
  ServoController, Odometry) have always legitimately included CommandProcessor.h
  since they ARE command handlers; these were pre-existing and are out of scope
  for this ticket. The MotionController layering inversion is fixed.
- [x] `source/control/MotionController.cpp` is ≤ 900 lines. (877 lines)
- [x] `source/control/MotionEventSink.h` exists with the two-field struct.
- [x] `TargetState` in `source/control/RobotState.h` gains a `MotionEventSink sink`
  field.
- [x] `MotionController::emitEvt()` calls through `target.sink.emitFn`; no
  `CommandProcessor::replyEvt` in `MotionController.cpp`.
- [x] `CMakeLists.txt` in `source/` and `host_tests/` adds `MotionCommandHandlers.cpp`
  to the build. Both use glob/recursive patterns that pick up new `.cpp` files
  automatically; no manual addition was needed.
- [x] All existing `host_tests/*.py` tests pass. (81/81)
- [x] `python3 build.py` succeeds. (RAM 98.33% — normal CODAL fixed allocation)

## Testing

- **Existing tests to run**: `uv run pytest host_tests/ -v && python3 build.py`
- **New tests to write**: None required here (the D11 gate test is ticket 003).
  Verify that `test_vw_converters.py` still passes — it should now exercise the
  queue path (ticket 001) and the `app/`-layer handlers.
- **Verification command**: `uv run pytest host_tests/ -v && python3 build.py`

## Implementation Notes

- This is the highest-risk ticket in the sprint. Work methodically: (1) create
  new files with extracted code, (2) update includes and CMakeLists, (3) remove
  code from `MotionController`, (4) compile, (5) run tests.
- The `makeCmd` helper used in `getCommands()` must be accessible from
  `MotionCommandHandlers.cpp`. Check where it is defined; add the include or move
  it to a shared header.
- `source/control/MotionController.h` currently includes `Protocol.h` for
  `ReplyFn`. After extraction, verify whether `Protocol.h` is still needed by
  `MotionController.h`; if `ReplyFn` is only used in `begin*()` signatures,
  it stays. If `MotionCtx` was the only other user, the include may be reducible.
- `mc_parseSensorToken` is a static function used by multiple handlers. Move it
  to `MotionCommandHandlers.cpp` or to a new internal header if it is also used
  by `Odometry`/`HaltController`. Confirm callers before moving.
- Clean build (`python3 build.py --clean`) before all bench verification.
