---
status: done
---

# A3 — Split the two firmware god objects (MotionController.cpp, Robot.cpp)

## Context

- `source/control/MotionController.cpp` — **1953 lines**: 8+ `begin*` state
  machines, the pursuit law, stop-condition wiring, protocol command
  handlers/converters, queue manipulation, reply emission.
- `source/robot/Robot.cpp` — **1490 lines**: facade, sensor orchestration,
  `buildTlmFrame()` telemetry formatting (10+ snprintf fields), the entire command
  table (`buildCommandTable` registering HELLO/PING/GET/SET/STREAM/ZERO/…), config
  plumbing.

Largely a consequence of A2, but even after that layering fix these files need
splitting before any agent can modify one subsystem without context-window-sized
diffs and accidental coupling between unrelated changes.

## Fix

After A2 removes the handlers/converters/reply code from MotionController:

1. MotionController: separate motion *laws* (pursuit, arc, rotation math) from mode
   *machinery* (active-command lifecycle, stop evaluation, BVC interaction).
2. Robot: extract a telemetry formatter (owns `buildTlmFrame`) and move the command
   table registration to `app/` next to the dispatch code. Robot keeps facade +
   sensor orchestration only.

## Acceptance

- No single firmware .cpp over ~600 lines on these paths; telemetry format changes
  touch one file; adding a motion mode touches control/ only.

## Priority suggestion

**Medium — explicitly sequenced after A2** (most of the mass moves there anyway).
Don't schedule as an independent "cleanup" sprint; fold the split into the A2
refactor's review criteria.

## Source
Finding **A3** in `docs/code_review/2026-06-11-architecture-modularity-review.md`.
