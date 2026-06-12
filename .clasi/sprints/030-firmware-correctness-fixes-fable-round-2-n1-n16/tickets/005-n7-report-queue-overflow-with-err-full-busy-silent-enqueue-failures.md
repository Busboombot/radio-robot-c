---
id: '005'
title: "N7: Report queue overflow with ERR full/busy — silent enqueue failures"
status: open
use-cases:
  - SUC-006
depends-on:
  - '002'
github-issue: ''
issue: fr2-n7-queue-full-err.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# N7: Report queue overflow with ERR full/busy — silent enqueue failures

## Description

`CommandProcessor::dispatchTable()` ignores the `_queue->push_back()` return
(`CommandProcessor.cpp:148`). All seven converters ignore `pushVW()` failure
(`MotionCommandHandlers.cpp:247` etc.). Queue capacity is 4 (`CommandQueue.h:18`),
drain rate is one command per ~10-25 ms tick.

A 5-command host burst drops the 5th command silently — the host just times out.
Worse for converters: the converter already replied `OK drive …`, so the host believes
motion started but the VW was never enqueued. Before ticket 002 this only bit the sim
and the post-first-safety-stop firmware; after 002 it bites all hardware traffic.

Depends on ticket 002 because that's when the queue path becomes permanently active.

## Acceptance Criteria

- [ ] `dispatchTable()` checks `push_back()` return and replies `ERR full` (or
      `ERR busy`) for dropped commands.
- [ ] All converter sites that call `pushVW()` check the return. On failure, the
      converter either (a) suppresses the early `OK` (preferred — no OK if VW not
      enqueued) or (b) emits a follow-up `ERR` after having sent `OK`.
- [ ] New sim test: burst of 5 commands (queue capacity 4) — the 5th gets an
      explicit `ERR full`/`ERR busy` response.
- [ ] New sim test: a converter whose `pushVW` fails returns no bare `OK` (or
      follows up with `ERR`).
- [ ] `python3 build.py` clean build passes.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.

## Implementation Plan

### Approach

Two sites to fix:
1. `CommandProcessor::dispatchTable()` — check the `_queue->push_back()` bool return
   and call the reply function with an ERR string on false.
2. Each converter in `MotionCommandHandlers.cpp` that calls `pushVW()` — check
   return and suppress or follow up. The cleanest approach is to suppress the early
   OK and only reply OK after the VW succeeds; a follow-up ERR is acceptable if the
   code structure makes the early OK hard to suppress.

### Files to modify

- `source/app/CommandProcessor.cpp`
  - `dispatchTable()`: check `_queue->push_back()` return; reply ERR on false.
- `source/app/MotionCommandHandlers.cpp`
  - All converter `pushVW()` call sites (~7 locations near line 247 etc.): check
    return; suppress OK or emit follow-up ERR.
- `host_tests/` or `host/tests/` — add burst-overflow and converter-VW-fail tests.

### Testing plan

Run: `uv run --with pytest python -m pytest host_tests/ host/tests/ -v`

Build: `python3 build.py` (clean).

### Notes

- Depends on ticket 002 so the queue path is active when testing.
- Do not change `CommandQueue.h` capacity — 4 is sufficient; the fix is in error
  reporting, not capacity expansion.
- The ERR token can be `ERR full` or `ERR busy` — pick whichever is consistent with
  existing ERR tokens in the codebase.
