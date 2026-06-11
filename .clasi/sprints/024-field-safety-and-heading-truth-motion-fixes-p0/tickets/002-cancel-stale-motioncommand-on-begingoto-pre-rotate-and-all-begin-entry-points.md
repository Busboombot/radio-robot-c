---
id: '002'
title: Cancel stale MotionCommand on beginGoTo PRE_ROTATE and all begin* entry points
status: open
use-cases:
  - SUC-001
  - SUC-002
depends-on:
  - '001'
github-issue: ''
issue: d07-motioncommand-ownership-in-pre-rotate.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 024-002 — Cancel stale MotionCommand on beginGoTo PRE_ROTATE and all begin* entry points

**Completes issue:** `d07-motioncommand-ownership-in-pre-rotate.md`
**Chain:** D7 (depends on 024-001 — PRE_ROTATE must be a proper MotionCommand before cancel-if-active is meaningful there)

## Description

`MotionController::beginGoTo()`'s PRE_ROTATE branch does not touch `_activeCmd`
before configuring the new motion. If any MotionCommand is already active (a VW
keepalive session, or a prior G/TURN not yet completed), `driveAdvance()`'s top
branch keeps ticking the stale command. The stale command's stop conditions —
wrong baselines, wrong EVT label — then govern when the robot stops, emitting the
wrong completion event. This is a race condition with chaotic field symptoms that
compounds D5.

The fix adds `if (_activeCmd.active()) _activeCmd.cancel(HARD)` before `configure()`
in every `begin*()` entry point: `beginGoTo()` (both branches), `beginTurn()`,
`beginVelocity()`, and `beginArc()`. This makes the transition from any prior command
observable on the wire (an explicit cancellation EVT) rather than silently absorbed.

## Files to Touch

- `source/control/MotionController.cpp` — insert cancel guard in: `beginGoTo()`
  PRE_ROTATE branch (before the `_activeCmd.configure()` added by ticket 001);
  `beginGoTo()` PURSUE branch (before existing `_activeCmd.configure()`);
  `beginTurn()` (before `_activeCmd.configure()`);
  `beginVelocity()` (before `_activeCmd.configure()`);
  `beginArc()` (before `_activeCmd.configure()`).
- `source/control/MotionController.h` — no interface changes needed.
- `host_tests/` — add test: issue TURN, then issue G mid-flight, assert exactly
  one active command post-transition, no duplicate EVT labels.

## Acceptance Criteria

- [ ] Every `begin*()` entry point — `beginGoTo()` (PRE_ROTATE and PURSUE
  branches), `beginTurn()`, `beginVelocity()`, `beginArc()` — executes
  `if (_activeCmd.active()) _activeCmd.cancel(HARD)` immediately before the
  first `_activeCmd.configure()` call.
- [ ] **Sim:** start a TURN (leave command active), then issue `G` mid-flight →
  exactly one command is active afterward (the new G command), no stale/duplicate
  EVT labels, robot pre-rotates under the PRE_ROTATE MotionCommand stops from ticket 001.
- [ ] **Unit:** assert that `_activeCmd.active()` is false immediately after
  `beginGoTo()` returns when a prior TURN command was active on entry. Assert
  the prior command's cancellation EVT appears in the reply stream.
- [ ] **Field-profile sim (slip on, fusion on):** back-to-back G commands produce
  no duplicate or mismatched EVT labels; each G produces exactly one
  `EVT done G` or `EVT timeout G`.
- [ ] **Hardware:** issuing G to cancel a running TURN emits a cancellation EVT for
  the TURN followed by normal G flow; no ghost commands appear in the TLM stream.
- [ ] Existing exact-profile host_tests pass unmodified.

## Implementation Plan

### Approach

The pattern is uniform: before every `_activeCmd.configure(...)` in every `begin*()`
method, insert:

```cpp
if (_activeCmd.active()) {
    _activeCmd.cancel(HARD);
}
```

`cancel(HARD)` already emits the cancellation EVT via the stored reply sink.
`configure()` then clears stale state. Verify that `cancel` does not free the
reply sink pointer before `configure()` re-assigns it.

### Testing Plan

1. Host_tests `test_cancel_on_begin_goto`: configure a TURN, then call
   `beginGoTo()` before TURN completes; assert exactly one active command,
   cancellation EVT emitted, new command has PRE_ROTATE structure from ticket 001.
2. Spot-check `beginVelocity()` and `beginArc()` with analogous micro-tests.
3. Run full `uv run pytest host_tests/` suite.

### Documentation Updates

None required. `tests/dev/safe_cmd_bench.py` is updated in ticket 003 where SAFE
one-shot semantics land.
