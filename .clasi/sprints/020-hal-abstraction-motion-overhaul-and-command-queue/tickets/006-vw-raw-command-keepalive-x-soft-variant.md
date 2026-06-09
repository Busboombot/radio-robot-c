---
id: '006'
title: _VW raw command, + keepalive, X soft variant
status: open
use-cases:
  - SUC-007
  - SUC-008
  - SUC-006
depends-on:
  - 020-005
github-issue: ''
issue: issue-motion-system-overhaul.md
completes_issue: false
---

# _VW raw command, + keepalive, X soft variant

## Description

Add three new wire verbs to complement the BVC-unified motion system from ticket 020-005:

**`_VW v omega`** — raw velocity command. Calls `bvc.seedCurrent(v, omega)` then
`bvc.setTarget(v, omega)`. The profiler starts at the target immediately — no trapezoid
ramp. For host-managed trajectory planners.

**`+`** — keepalive-only command. Resets the system watchdog (`_watchdogMs = now_ms`
in LoopScheduler). No motion side-effect. Replies `OK keepalive`.

**`X soft`** — soft stop. Sets BVC target to (0, 0) and lets the profiler ramp down
under aMax. Emits `EVT done` when speed reaches zero. Hard `X` retains existing
immediate-stop behavior (no change).

## Acceptance Criteria

- [ ] `_VW v omega` registered in `MotionController::getCommands()`; handler calls `bvc.seedCurrent(v, omega)` then `bvc.setTarget(v, omega)`; replies `OK _VW`.
- [ ] After `_VW 300 0`, `bvc.currentV()` returns 300.0 (or close) on the very next `driveAdvance` tick — no ramp from zero.
- [ ] `+` registered in `MotionController::getCommands()` (or in system commands in Robot); handler resets `sched._watchdogMs`; replies `OK keepalive`.
- [ ] `X soft` handled: when the token following `X` is `soft`, calls new `MotionController::softStop()` method; hard `X` (no suffix) unchanged.
- [ ] `MotionController::softStop()` sets `_bvc.setTarget(0, 0)` (ramp to zero); MotionCommand continues until BVC reaches target; then emits `EVT done`.
- [ ] Bench: `_VW 300 0` → motor immediately at ~300 mm/s with no visible ramp.
- [ ] Bench: `VW v=300 w=0; X soft` → motor ramps to zero; `EVT done` received.
- [ ] Bench: `+` while robot idle → no motion change; no error.
- [ ] `python3 build.py --clean` passes.
- [ ] `uv run --with pytest python -m pytest` passes.

## Implementation Plan

### Approach

1. Add `beginRawVelocity` method to `MotionController`: seeds + sets BVC, no stop
   conditions, no ramp. Register as `_VW` in `getCommands()`.
2. Add `softStop()` to `MotionController`: `_bvc.setTarget(0, 0)`, let ramp handle it,
   emit `EVT done` when at target.
3. Update X handler in `MotionController::getCommands()` to check for `soft` token.
4. Add `+` handler: either in MotionController or as a system command in Robot. The
   handler needs access to `LoopScheduler::_watchdogMs`. Pass `LoopScheduler*` via ctx,
   or expose a `resetWatchdog(uint32_t now_ms)` method on LoopScheduler.

### Files to Modify

- `source/control/MotionController.h` — add `beginRawVelocity()`, `softStop()`
- `source/control/MotionController.cpp` — implement `beginRawVelocity`, `softStop`, `_VW` + `X soft` handlers
- `source/control/LoopScheduler.h` — expose `resetWatchdog(uint32_t)` or make `_watchdogMs` accessible
- `source/robot/Robot.cpp` — add `+` to system command table (or MotionController)

### Testing Plan

1. `python3 build.py --clean` — zero warnings.
2. Flash via `mbdeploy deploy robot --clean`.
3. Bench: `_VW 300 0` — motor immediately at speed (no visible ramp delay).
4. Bench: `VW v=300 w=0; X soft` — smooth deceleration; `EVT done` received.
5. Bench: `VW v=300 w=0; + ; + ; +` — motor maintains speed; silence → watchdog fires.
6. `uv run --with pytest python -m pytest` — no regressions.

### Notes

- `X soft` parser: the `X` handler currently checks if args are empty. Extend to check
  for a `soft` positional arg (args[0].sval == "soft"). Both paths go through the same
  handler function; branch on the arg.
- `EVT done` for soft stop: reuse the existing MotionCommand soft-stop emission path.
  When BVC reaches zero, `MotionCommand::tick()` should detect `atTarget() == true`
  and the stop style is SOFT — the existing EVT emission fires naturally. Verify the
  MotionCommand SOFT stop path works when target is (0,0).
- The `+` handler's need to reset `_watchdogMs` on LoopScheduler creates a cross-layer
  dependency. Prefer a `resetWatchdog()` method on LoopScheduler over exposing the
  field directly.
