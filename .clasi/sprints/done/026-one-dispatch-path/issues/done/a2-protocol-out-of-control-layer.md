---
status: done
sprint: '026'
tickets:
- 026-002
---

# A2 — Move protocol dispatch and reply formatting out of the firmware control layer

## Context

`source/control/` depends upward on the app/protocol layer: six .cpp files include
`CommandProcessor.h` (MotionController, LoopScheduler, Odometry, HaltController,
PortController, ServoController; plus MotionCommand.h, MotorController.h,
RobotState.h). Concretely:

- Control code formats wire replies itself — `CommandProcessor::replyOK/Err/Evt`
  calls and inline `snprintf` of `EVT …` lines in `MotionController::emitEvt`,
  `LoopScheduler` (safety_stop), and `HaltController`.
- `MotionController` contains the protocol command handlers themselves
  (`handleS/T/D/G/TURN/RT/VW/X/STOP`), builds `ParsedCommand` objects, and pushes to
  the app-layer `CommandQueue` (`MotionCtx` holds `CommandQueue*` and a
  `CommandDescriptor vwDesc`).

This inversion is not cosmetic: the converter → queue → `handleVW` double-dispatch
inside the control layer is the direct mechanism of the duplicate-OK defect (D11)
and of the sim/hardware dispatch split (sim never wires the queue). While the motion
state machine and the protocol front-end are the same class, every protocol change
risks motion behavior and vice versa, and neither can be tested in isolation.

## Fix

1. Command parsing/conversion (the S/T/D/G/TURN/RT→VW converters) and all reply/EVT
   formatting move to `app/`.
2. `control/` exposes typed `begin*/cancel/advance` APIs and reports completion and
   safety events through a narrow callback or event struct; `app/` turns those into
   `OK`/`EVT` lines. `control/` no longer includes `CommandProcessor.h`,
   `CommandQueue.h`, or `Protocol.h` reply types.
3. Outcome by construction: one dispatch path (kills the sim/hw split at its root,
   complements `sim-runs-real-dispatch-path`), one reply per command (subsumes the
   mechanism behind `d11-single-ok-per-command` — coordinate the two issues).

## Acceptance

- `grep -rl 'CommandProcessor.h\|CommandQueue.h' source/control/` returns nothing.
- Converter/dispatch unit tests live against `app/`; motion state machine tests run
  with a stub event sink and no protocol headers.
- D11 double-OK test passes; sim and hardware dispatch are the same code.

## Priority suggestion

**High — schedule alongside or immediately after `sim-runs-real-dispatch-path` and
`d11-single-ok-per-command`**; doing those two without this refactor patches the
symptoms and leaves the structure that generated them. Large diff; should be its own
sprint item, not a rider.

## Source
Finding **A2** in `docs/code_review/2026-06-11-architecture-modularity-review.md`.
