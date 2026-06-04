---
id: '006'
title: Implement LoopScheduler with task table and cooperative loop algorithm
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-004
depends-on:
- '003'
- '004'
- '005'
github-issue: ''
issue: plan-single-cooperative-main-loop-abandon-fibers.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement LoopScheduler with task table and cooperative loop algorithm

## Description

Create `source/control/LoopScheduler.{h,cpp}` implementing the single
cooperative main loop described in the architecture update. `LoopScheduler`
owns the `Task` table, the round-robin cursor, the loop algorithm, and the
reply-sink adapters (`serialReply`/`radioReply`) that move from `main.cpp`.

The loop algorithm (see architecture-update.md "Loop Execution Sequence"):
1. `controlCollect(now)` — collect encoder, run PID, write PWM.
2. Update `controlDeadline = now + controlPeriodMs`.
3. Round-robin sweep: for each task from cursor, check `due()`, check budget
   gate (`now + estCostMs > controlDeadline`); run if passing both; re-check
   deadline after each task; break if deadline hit.
4. `controlFireRequest()` — fire encoder request for the other wheel.
5. `uBit.sleep(controlDeadline - now)` — the program's only sleep.

At this stage `LoopScheduler` is constructed but `main.cpp` is not yet
updated — that happens in ticket 007. `LoopScheduler::run()` must be callable
as a drop-in replacement for the existing two-fiber `main()` loop body.

## Files to Create

- `source/control/LoopScheduler.h`
- `source/control/LoopScheduler.cpp`

## Files to Modify

None at this ticket (the CMakeLists.txt / build system source list must be
updated to include the two new files).

## Acceptance Criteria

- [x] `LoopScheduler.h` defines `Task { name, periodMs, lastRunMs, estCostMs,
  due(*this, now), run(*this, now) }` exactly as described in the architecture
  and the design issue.
- [x] `LoopScheduler::run()` implements the loop algorithm with:
  - Control task (not in the rotated table) runs first every iteration.
  - Round-robin low-priority sweep with persistent cursor.
  - Budget gate (`now + estCostMs > controlDeadline`) before each task start.
  - Post-task deadline re-check with break.
  - `controlFireRequest()` called last before sleep.
  - Single `uBit.sleep(deadline - now)` per iteration.
- [x] The task table includes all eight low-priority tasks in priority order:
  comms-in, drive-advance, odometry-predict, otos-correct, line-read,
  color-read, ports-read, telemetry-emit — with correct `periodMs` and
  `estCostMs` values (see architecture-update.md table).
- [x] `pendingWheel` alternates L/R per iteration; first-iteration guard
  (skip collect if no request has been fired yet).
- [x] Reply-sink adapters `serialReply` and `radioReply` are defined in
  `LoopScheduler.cpp` (moved from `main.cpp`).
- [x] Firmware builds cleanly with the new files added to the build.
- [x] `uv run --with pytest python -m pytest` passes.

## Implementation Plan

1. `LoopScheduler.h`: declare the `Task` struct and `LoopScheduler` class.
   Members: `Robot& _robot`, `CommandProcessor& _cmd`, `MicroBit& _uBit`,
   `Task _table[N]`, `int _cursor`, `int _pendingWheel` (0=none, 1=left, 2=right),
   `uint32_t _controlDeadline`.
2. `LoopScheduler.cpp`: implement `run()` per the pseudocode in the architecture
   update. The task table is a static array initialized in the constructor
   (or as a constant array in the `.cpp`).
3. Task table entries (periodMs / estCostMs):
   - comms-in: 0 / 0
   - drive-advance: 0 / 0
   - odometry-predict: 0 / 0
   - otos-correct: 100 / 2 (read from `cfg.lagOtosMs`)
   - line-read: 50 / 1 (read from `cfg.lagLineMs`)
   - color-read: 100 / 1 (read from `cfg.lagColorMs`)
   - ports-read: 50 / 0 (read from `cfg.lagPortsMs`)
   - telemetry-emit: `cfg.tlmPeriodMs` / 2
4. Each `due()` function checks `now - task.lastRunMs >= task.periodMs` (with
   special-case for `periodMs == 0` always returning true).
5. Each `run()` function calls the appropriate `Robot::*` task entry point.
6. Add `serialReply` / `radioReply` static functions (moved from `main.cpp`).
7. Add the two new `.cpp` files to `source/CMakeLists.txt` (or equivalent
   build system source list).

## Testing Plan

- **Build verification**: `python build.py` — no new errors with new files
  in the build.
- **Automated tests**: `uv run --with pytest python -m pytest` — full suite
  must pass. At this stage `LoopScheduler` is compiled but not yet called
  from `main.cpp`; the existing two-fiber loop still runs.
- **Integration**: Verified end-to-end in ticket 008 (main.cpp switchover)
  and ticket 009 (bench gate).
