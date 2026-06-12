---
id: '001'
title: Extract tickOnce and wire CommandQueue in sim
status: open
use-cases:
  - SUC-001
  - SUC-002
depends-on: []
github-issue: ''
issue: sim-runs-real-dispatch-path.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 026-001: Extract tickOnce and wire CommandQueue in sim

## Description

The simulator exercises a different dispatch path than hardware because
`host_tests/sim_api.cpp` never wires a `CommandQueue`. Converter commands (S,
T, D, G, TURN, RT) take the direct `begin*()` fallback in sim; on hardware they
go through `converter → queue → handleVW`. This is why D11 (double-OK) and D6
(keepalive-stomp) cannot be reproduced in sim.

Additionally, `sim_tick()` hand-mirrors the `LoopScheduler::run_blocks()` loop
body — a divergence generator (search: "MUST mirror LoopScheduler.cpp exactly").
Any change to the loop body must be made twice and can silently drift.

This ticket addresses both issues:

1. Extract `LoopScheduler::tickOnce(uint32_t now)` from `run_blocks()`.
2. Wire a `CommandQueue` in `SimHandle`; replace `sim_tick()`'s hand-mirrored
   body with a call to `tickOnce()`.

### Step 1: Extract `LoopScheduler::tickOnce(uint32_t now)`

Move the loop body of `run_blocks()` into a new method
`LoopScheduler::tickOnce(uint32_t now)`. The method takes `now` as a parameter
and does not call `_uBit.systemTime()` internally — the caller supplies the
current time. This is required for the sim's clock injection (`g_sim_now_ms`)
to work correctly.

The per-block last-timestamps (`lastOtos`, `lastLine`, `lastColor`, `lastPorts`,
`lastTlm`) that are currently local variables in `run_blocks()` become private
fields of `LoopScheduler` (e.g., `_lastOtos`, `_lastLine`, etc.), initialized
to `0` in the constructor (the phase-spread offsets that currently stagger first
runs can be initialized in `tickOnce()`'s first-call path, or in the constructor).

`run_blocks()` becomes a shell: initialize `controlDeadline`, loop — call
`controlCollectSplitPhase(now, 0)`, then `tickOnce(now)`, then sleep to
`controlDeadline`. The control phase and sleep stay in `run_blocks()` because
they require `_uBit`.

`tickOnce(now)` contains: `runCommsIn`, `cmd.dequeueOne(_queue)`, watchdog check,
halt evaluation, `driveAdvance`, `odometry.predict`, conditional OTOS/line/color/
ports/TLM blocks using `_lastOtos` etc. as fields.

### Step 2: Wire CommandQueue in SimHandle

In `host_tests/sim_api.cpp`:

1. Add `CommandQueue _queue;` as a field of `SimHandle`.
2. In `SimHandle`'s constructor, after `cmd` is constructed, call:
   `cmd.setQueue(&_queue)` and `robot.motionController.setQueue(&_queue)`.
3. Replace `sim_tick()`'s hand-mirrored block with a call to `tickOnce()`.

To call `tickOnce()` from `sim_tick()`, either: (a) `SimHandle` owns a
`LoopScheduler` instance (requires a sim-compatible constructor), or (b) extract
`tickOnce()` as a free function `loopTickOnce(Robot&, CommandProcessor&,
CommandQueue&, uint32_t now, ...)`. **Recommended: option (b)** — a free function
avoids the `MicroBit&` constructor constraint. `run_blocks()` calls the same
free function. All state that `tickOnce()` needs (the last-timestamps, the reply
sinks) either comes from `LoopScheduler`'s fields (passed by ref) or from
`SimHandle`'s fields.

The hand-mirrored watchdog, halt, odometry, and OTOS blocks in `sim_tick()` are
removed — they are now in `tickOnce()`. The `SimHandle::watchdogMs` field is
removed. The `fuseOtos` flag integration into `tickOnce()` must be preserved:
pass it as a parameter or via a sim-specific enable flag.

The existing `sim_command()`, `sim_get_async_evts()`, and all `sim_get_*/sim_set_*`
functions are **unchanged** in signature. Python test code requires no changes.

## Acceptance Criteria

- [ ] `LoopScheduler::tickOnce(uint32_t now)` exists and is declared in
  `source/control/LoopScheduler.h`.
- [ ] `tickOnce()` does not call `_uBit.systemTime()` internally.
- [ ] `run_blocks()` calls `tickOnce(now)` and contains no duplicated block logic.
- [ ] `SimHandle` gains a `CommandQueue _queue` field; it is wired to both `cmd`
  and `robot.motionController` at construction.
- [ ] `sim_tick()` calls `tickOnce()` (or the equivalent free function) and
  contains no hand-mirrored copies of watchdog, halt, drive, odometry, or OTOS
  block logic.
- [ ] `SimHandle::watchdogMs` field is removed.
- [ ] Converter commands sent via `sim_command()` travel through the queue path
  (not the direct `begin*()` fallback). Verify by temporarily adding a `printf`
  or by checking that the existing fallback-path comment in `test_vw_converters.py`
  no longer applies and all converter tests still pass.
- [ ] All existing `host_tests/*.py` tests pass.
- [ ] `python3 build.py` succeeds (firmware compiles clean with `--clean`).

## Testing

- **Existing tests to run**: `uv run pytest host_tests/` (entire suite — covers
  watchdog, halt, motion controller, BVC, cancel-on-begin, mock HAL, VW converters,
  watchdog exemption).
- **New tests to write**: None required for this ticket; the suite coverage is
  sufficient to detect regressions. Optionally add a `test_queue_path_active` test
  that asserts the converter path was taken (by checking that a T command with a
  corr-id produces no double-OK, as a forward-compatibility check for ticket 003).
- **Verification command**: `uv run pytest host_tests/ -v && python3 build.py`

## Implementation Notes

- The "MUST mirror" comment must be deleted as part of this ticket (the lint in
  ticket 004 will enforce its absence).
- `test_vw_converters.py` currently has the comment: "The sim does not wire a
  CommandQueue into MotionController, so all converter handlers take the fallback
  path." This comment must be updated or removed once this ticket lands.
- Clean build is mandatory before bench testing (project memory: stale incremental
  builds produce broken binaries).
- `fuseOtos` and the related sim-model controls in `sim_tick()` must be integrated
  into or called from the new `tickOnce()` path — do not lose OTOS fusion in sim.
