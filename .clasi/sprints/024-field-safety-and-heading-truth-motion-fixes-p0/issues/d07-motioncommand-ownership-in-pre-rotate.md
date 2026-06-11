---
status: in-progress
sprint: '024'
tickets:
- 024-002
---

# D7 — beginGoTo PRE_ROTATE must own/cancel the active MotionCommand

## Context

`MotionController::beginGoTo()`'s PURSUE branch calls `_activeCmd.configure(...)`
(which implicitly resets a previous command), but the **PRE_ROTATE branch does not
touch `_activeCmd`** ([source/control/MotionController.cpp:387-389](../../source/control/MotionController.cpp#L387)).
If any MotionCommand is still active when G arrives (a VW keepalive session, or a
prior G/TURN not yet completed), `driveAdvance()`'s top branch keeps ticking the
**stale** command — now with the BVC seeded to the pre-rotate spin — and the stale
command's stop conditions (wrong baselines, wrong EVT label) decide when the robot
stops, emitting the wrong completion event. Race condition with chaotic field
symptoms; compounds D5.

## Fix (improvement-plan P0.1.2 / P0.1.4)

- In `beginGoTo` PRE_ROTATE (and as a uniform rule across `beginTurn` /
  `beginVelocity` / `beginArc`): if `_activeCmd.active()`, call
  `_activeCmd.cancel(HARD)` first so the stale command's cancellation is emitted
  explicitly rather than silently absorbed, then configure the new command.
- `configure()` already clears stale state; the explicit cancel makes the
  transition observable on the wire.

## Acceptance

- **Sim:** start a TURN (or leave a VW session active), then issue `G` mid-flight →
  exactly one command is active afterward, no stale/duplicate EVT labels, robot
  pre-rotates under the new command's stops (from D5).
- **Unit:** assert `_activeCmd` identity/baseline is reset on the PRE_ROTATE entry
  when a prior command was active.

## Source
Defect **D7** in the 2026-06-11 sim2real review; fix P0.1.2/P0.1.4. Pairs with D5.
