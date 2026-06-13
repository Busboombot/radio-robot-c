---
id: '002'
title: Fix DBG OTOS BENCH enable bug
status: in-progress
use-cases:
- SUC-002
depends-on: []
issue: fr-bench-dbg-otos-no-reply.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix DBG OTOS BENCH enable bug

## Description

`DBG OTOS BENCH 1` always replies `bench=0` — bench mode never engages. This was confirmed
over the robot's USB serial (transport ruled out as a factor). The handler
`handleDbgOtosBench` in `DebugCommandable.cpp` parses `enable = atoi(tokens[0])` and calls
`nh->setOtosBench(enable != 0)`, then reads back `nh->isBenchMode()`. The
`setOtosBench(bool)` / `isBenchMode()` pair in `NezhaHAL.h:63-75` swaps / compares
`_otosActive` against `static_cast<IOtosSensor*>(&_benchOtos)`. Something in the
token→parse→swap→pointer-comparison chain does not flip the state.

Investigate the full chain — token/arg plumbing, pointer assignment in `setOtosBench`,
pointer comparison in `isBenchMode()` (watch for cast/const-adjustment mismatch or a
separate HAL instance in the call path). Fix the bug so the round-trip works. Add a
host-reachable sim test or seam so `isBenchMode()` state can be asserted in `host_tests/`.

## Acceptance Criteria

- [ ] Over USB serial: `DBG OTOS BENCH 1` → `OK dbg otos bench=1`
- [ ] Over USB serial: `DBG OTOS BENCH 0` → `OK dbg otos bench=0`
- [ ] A sim test asserts that after `handleDbgOtosBench(enable=1)`, `isBenchMode()` is true,
      and after `handleDbgOtosBench(enable=0)`, it is false
- [ ] `python3 build.py` clean build passes
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes

## Testing

- **Existing tests to run**: `uv run --with pytest python -m pytest host_tests/ host/tests/`
- **New tests to write**: Sim test in `host_tests/` (or `host/tests/`) that calls the DBG
  OTOS BENCH handler via the sim harness and asserts `isBenchMode()` flips correctly
- **Verification command**: `uv run --with pytest python -m pytest host_tests/ host/tests/`

## Implementation Plan

### Approach

1. Trace the full call chain in `DebugCommandable.cpp`:
   - `parseDbgOtosBench` (parser function registered in `makeCmd`) — verify it puts the
     enable token in `args[0]`
   - `handleDbgOtosBench` — verify `atoi(tokens[0])` (or however args are accessed) actually
     receives the digit; check for off-by-one or arg indexing differences between the parser
     and handler conventions used elsewhere
   - `nh->setOtosBench(enable != 0)` — trace through `NezhaHAL.h:63-68`; confirm `_otosActive`
     is actually assigned `&_benchOtos`
   - `nh->isBenchMode()` — `NezhaHAL.h:73-75` compares `_otosActive ==
     static_cast<const IOtosSensor*>(&_benchOtos)`; check that the const-cast matches the
     non-const cast used in `setOtosBench`, and that the `nh` pointer used in the handler is
     the same HAL instance exposed to the robot loop (not a copy or temporary)

2. Fix whatever is broken. The most likely candidates are: (a) arg indexing bug (token[0] is
   not the enable digit); (b) cast mismatch in the pointer comparison; (c) the `nh` context
   pointer delivered to the handler is not the live HAL instance.

3. Add a sim test (host-side) that exercises the enable/disable round-trip.

### Files to Modify

- `source/app/DebugCommandable.cpp` — fix `handleDbgOtosBench` / `parseDbgOtosBench`
- `source/hal/NezhaHAL.h` — fix `setOtosBench` or `isBenchMode` if the cast mismatch is there
- `host_tests/` or `host/tests/` — add bench-mode enable/disable sim test

### Documentation Updates

None required.
