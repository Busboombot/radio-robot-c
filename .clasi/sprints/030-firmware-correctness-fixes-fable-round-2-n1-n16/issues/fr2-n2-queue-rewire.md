---
status: in-progress
sprint: '030'
tickets:
- 030-002
---

# FR2-N2 (High) — Firmware no longer runs the queue path: main.cpp Phase 3 wipes `cmd._queue`

## Context

Source: `docs/code_review/2026-06-12-Fable-correctness-review/findings.md` §N2.

`LoopScheduler`'s constructor wires `_cmd.setQueue(&_queue)` (`LoopScheduler.cpp:108`),
but main.cpp Phase 3 reassigns the processor:
`cmd = CommandProcessor(robot.buildCommandTable(&dbgCmd, &sched))` (`main.cpp:215`).
The implicit move-assign copies the temporary's `_queue == nullptr`, so on entry to
`run_blocks()` the queue is detached and `process()` dispatches every inbound command
immediately (the pre-026 path). `run_test()` re-wires and names the bug
(`LoopScheduler.cpp:139`); `run_blocks()` does not.

Consequences:
1. **Sim/real split inverted** — sim wires+tests the queue path; firmware runs the
   immediate path. (Motion still works only because `robot.setMotionQueue(&_queue)`
   survives, so converter VW pushes drain via `dequeueOne` by accident.)
2. **Mid-session mode flip** — the watchdog/halt emergency path does
   `setQueue(nullptr); process("X"); setQueue(&queue)` (`LoopTickOnce.cpp:61-63,76-83`);
   the restore *arms* the never-armed queue. After the first safety stop/halt the
   firmware permanently switches to queued dispatch (1 cmd/tick drain, overflow
   possible, SNAP staleness changes). Behavior depends on whether a safety stop ever
   fired.

## Fix

Re-wire `_cmd.setQueue(&_queue)` at the top of `run_blocks()` (one line, mirroring
`run_test()`), or give `CommandProcessor` an assignment operator that preserves
wiring. Then the `setQueue(nullptr)/restore` dance in `loopTickOnce` is consistent.

## Acceptance

- A firmware-config boot test asserts `cmd` still has its queue after Phase 3.
- Dispatch path is identical in sim and firmware (queue path) before and after any
  safety stop.
