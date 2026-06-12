---
id: '002'
title: "N2: Re-wire queue in run_blocks() \u2014 restore firmware queue-path dispatch\
  \ from boot"
status: done
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: fr2-n2-queue-rewire.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# N2: Re-wire queue in run_blocks() — restore firmware queue-path dispatch from boot

## Description

`LoopScheduler`'s constructor wires `_cmd.setQueue(&_queue)` (`LoopScheduler.cpp:108`).
main.cpp Phase 3 then reassigns the processor:
`cmd = CommandProcessor(robot.buildCommandTable(&dbgCmd, &sched))` (`main.cpp:215`).
The implicit move-assign copies the temporary's `_queue == nullptr`, so on entry to
`run_blocks()` the queue is detached and `process()` dispatches every inbound command
immediately (the pre-026 direct path). `run_test()` already has a one-line fix with a
comment naming the bug (`LoopScheduler.cpp:139`); `run_blocks()` does not.

Two consequences: (1) Sim and firmware have been running different dispatch paths
since sprint 026. (2) The watchdog path does `setQueue(nullptr); process("X");
setQueue(&queue)` — the restore arms the never-armed queue, so after the first
safety stop the firmware permanently switches to queued dispatch. Behavior depends on
whether a safety stop has ever fired.

## Acceptance Criteria

- [x] `run_blocks()` re-wires `_cmd.setQueue(&_queue)` at its entry (one line,
      mirroring the existing `run_test()` pattern).
- [x] New firmware-config boot test: asserts `cmd._queue` is non-null after Phase 3
      reassignment and before the first command is dispatched.
- [x] Dispatch path in sim and firmware is the queue path both before and after a
      simulated safety stop (no mode flip).
- [x] `python3 build.py` clean build passes.
- [x] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.

## Implementation Plan

### Approach

One-line fix in `run_blocks()`, mirroring the `run_test()` pattern already in the
same file. No interface changes; no new abstractions.

### Files to modify

- `source/app/LoopScheduler.cpp`
  - At the top of `run_blocks()`, add: `_cmd.setQueue(&_queue);`
  - (Optional) Add a comment pointing to the Phase-3 reassignment as the reason,
    mirroring the existing comment in `run_test()`.
- `host_tests/` or `host/tests/` — add:
  - `test_queue_wired_after_phase3`: construct a LoopScheduler, simulate Phase 3
    reassignment, assert queue is non-null before `run_blocks()` dispatches.
  - `test_no_mode_flip_after_safety_stop`: safety-stop sim, verify queue path still
    active for subsequent commands.

### Testing plan

Run: `uv run --with pytest python -m pytest host_tests/ host/tests/ -v`

Build: `python3 build.py` (clean).

### Notes

- Independent of ticket 001 (different files).
- The `setQueue(nullptr)/restore` dance in `LoopTickOnce.cpp:61-63,76-83` is
  intentional (X processing without queue); do not remove it. The fix here makes it
  consistent because `run_blocks()` now armed the queue first.
