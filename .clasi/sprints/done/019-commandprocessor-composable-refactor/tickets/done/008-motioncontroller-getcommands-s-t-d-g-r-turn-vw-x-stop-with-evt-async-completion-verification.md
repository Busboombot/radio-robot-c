---
id: 008
title: "MotionController::getCommands() \u2014 S, T, D, G, R, TURN, VW, X, STOP with\
  \ EVT async completion verification"
status: done
use-cases:
- SUC-001
- SUC-007
depends-on:
- '007'
github-issue: ''
issue: ''
completes_issue: false
---

# MotionController::getCommands() — S, T, D, G, R, TURN, VW, X, STOP with EVT async completion verification

## Description

Implement `MotionController::getCommands()` returning descriptors for all nine motion
commands: S, T, D, G, R, TURN, VW, X, STOP. This is the highest-risk migration step
because each handler must capture `replyFn`, `replyCtx`, and `corrId` and pass them
to the corresponding `begin*()` entry point exactly as the old switch did, so that EVT
async completions (`done T`, `done D`, `done G`, `done R`, `done TURN`) fire on the
correct channel.

The old switch still handles all motion commands. `main.cpp` continues using the old
constructor so no commands are routed through the new path yet. This ticket implements
the handlers but does not activate them.

## Acceptance Criteria

- [x] `MotionController::getCommands()` (in `source/control/MotionController.cpp`) returns descriptors for:
  - `"S"` — parse: float leftMms, float rightMms; handler calls `beginStream(...)` with current time from `robot->systemTime()`
  - `"T"` — parse: float leftMms, float rightMms, int durationMs; optional `sensor=` kv; handler calls `beginTimed(...)`, replies `OK T`
  - `"D"` — parse: float leftMms, float rightMms, int targetMm; optional `sensor=` kv; handler calls `robot->distanceDrive(...)`
  - `"G"` — parse: float tx, float ty, float speedMms; handler calls `beginGoTo(...)`
  - `"R"` — parse: float speedMms, float radiusMm; handler calls `beginArc(...)`
  - `"TURN"` — parse: int headingCdeg; optional `eps=` kv; handler calls `beginTurn(...)`
  - `"VW"` — parse: float v, float omega; handler calls `beginVelocity(...)` or re-arms keepalive
  - `"X"` — no parse; handler calls `cancel(now_ms, replyFn, ctx)`
  - `"STOP"` — no parse; handler calls `cancel(now_ms, replyFn, ctx)` (same as X)
- [x] Each motion handler captures `replyFn`, `replyCtx`, and `corrId` and passes them to `begin*()` identically to the corresponding old switch case
- [x] `MotionCtx` struct defined in `MotionController.h`: `{ MotionController* mc; Robot* robot; }`
- [x] `python3 build.py` passes with no errors
- [ ] Bench EVT verification (via old path — old constructor still active): `D 200 200 300 #1` produces `EVT done D #1`; `T 200 200 1000 #2` produces `EVT done T #2`

## Implementation Plan

### Approach

Read every motion switch case in `CommandProcessor.cpp` in full before writing any
handler. The key pattern for async-completion commands: the handler receives
`(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx, void* handlerCtx)`.
It must unpack `handlerCtx` to `MotionCtx*`, then call `mc->beginDistance(...)` (or
equivalent) with `replyFn`, `replyCtx`, `corrId` as the final parameters — identical
to how the old switch calls them.

VW keepalive logic: if `mc->hasActiveCommand()`, re-arm the watchdog with
`mc->activeCmd().setTarget(v, omega)` rather than calling `beginVelocity` again —
same as the old switch logic.

### Files to Modify

- `source/control/MotionController.h` — add `MotionCtx` struct (if not already added in T004)
- `source/control/MotionController.cpp` — implement `getCommands()`

### Testing Plan

- Build: `python3 build.py` must pass.
- Unit: Read all existing motion tests under `tests/dev/`; run them via `uv run --with pytest python -m pytest tests/dev/`.
- Bench EVT: With the robot on the stand, verify `D`, `T`, `G`, `TURN`, `R` completions fire `EVT done <verb>` with correct corrId over the wire. These tests use the old switch path — the new handlers are verified by visual code review against the old cases.

### Documentation Updates

Update architecture-update.md Open Question #2 (system command context) if resolved during this ticket.
