---
id: '002'
title: NezhaHAL wiring and Robot::benchOtosTick
status: done
use-cases:
- SUC-001
- SUC-002
depends-on:
- 031-001
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# NezhaHAL wiring and Robot::benchOtosTick

## Description

Wire `BenchOtosSensor` into the live firmware stack: add the active-pointer
swap to `NezhaHAL`, add `Robot::benchOtosTick()`, and insert the per-tick feed
call in `LoopTickOnce.cpp`. After this ticket the firmware compiles with bench
mode wiring but no DBG commands yet (those come in ticket 003).

**NezhaHAL changes** (`source/hal/NezhaHAL.h` / `NezhaHAL.cpp`):
- Add `BenchOtosSensor _benchOtos` as a private value member (constructed
  default — no args needed).
- Add `IOtosSensor* _otosActive` pointer, initialized to `&_otos` in the
  constructor.
- Change `otos()` override from `return _otos;` to `return *_otosActive;`.
- Add `void setOtosBench(bool on)` — sets `_otosActive = on ? &_benchOtos : &_otos`.
- Add `BenchOtosSensor* benchOtosPtr()` — returns `&_benchOtos` (used by
  `Robot::isBenchOtosActive()` and the DBG handler to call `setNoise()`).
- `#include "BenchOtosSensor.h"` in `NezhaHAL.h`.

**Robot changes** (`source/robot/Robot.h` / `Robot.cpp`):
- Add `void benchOtosTick(uint32_t now_ms)`:
  - Compute `dt_ms` from `_lastBenchTickMs` (new private uint32 member,
    init 0); update `_lastBenchTickMs = now_ms`.
  - If bench mode is off, return immediately (no-op path).
  - Downcast `hal` to `NezhaHAL*` (or use a virtual accessor on `Hardware` —
    see OQ-1 in the architecture doc). If the downcast returns nullptr (MockHAL
    in host tests), return immediately.
  - Call `hal_nezha->benchOtosPtr()->tick(state.commands.tgtLMms,
      state.commands.tgtRMms, config.trackwidthMm, dt_ms)`.
- Add `bool isBenchOtosActive() const`:
  - Downcast `hal` to `NezhaHAL*`; if nullptr return false.
  - Return `hal_nezha->benchOtosPtr() == &hal_nezha->otos()` is false... more
    simply: add a `bool isBenchMode() const` on `NezhaHAL` that returns
    `_otosActive == &_benchOtos`, and call that.
- Add the `_lastBenchTickMs` private member.

**LoopTickOnce changes** (`source/control/LoopTickOnce.cpp`):
- Immediately before the `// ===== OTOS:` block (currently line 95), insert:
  ```
  robot.benchOtosTick(now);
  ```
  One line. No new conditional in `loopTickOnce` itself — the no-op lives
  inside `benchOtosTick`.

**Note on HOST_BUILD**: `NezhaHAL` includes CODAL headers and is excluded from
host builds. `Robot::benchOtosTick()` must guard the downcast so it compiles
cleanly when `hal` is a `MockHAL`. The simplest guard is:

```cpp
#ifndef HOST_BUILD
    auto* nh = dynamic_cast<NezhaHAL*>(&hal);
    if (!nh) return;
    // ... rest of impl
#endif
```

Or add a non-virtual `Hardware::benchOtosPtr()` returning `nullptr` by default
and override in `NezhaHAL`. Either is fine; pick the lower-churn approach.

## Acceptance Criteria

- [x] `NezhaHAL::otos()` returns `*_otosActive`; behavior with bench off is
  identical to pre-sprint (pointer points to `_otos`).
- [x] `NezhaHAL::setOtosBench(true)` makes `otos()` return `_benchOtos`
  reference; `setOtosBench(false)` restores `_otos`.
- [x] `Robot::benchOtosTick(now)` is a no-op when bench mode is off (no crash,
  no accumulator advance).
- [x] `LoopTickOnce.cpp` calls `robot.benchOtosTick(now)` immediately before
  the OTOS block.
- [x] `python3 build.py` clean build passes (device firmware compiles).
- [x] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes
  (no regressions — MockHAL path returns false/no-op cleanly).

## Implementation Plan

### Approach

Modify three existing files; no new files. Keep the no-op path fast and safe
so the MockHAL host-test path is unaffected.

### Files to Modify

- `source/hal/NezhaHAL.h` — add `_benchOtos`, `_otosActive`, `setOtosBench()`,
  `benchOtosPtr()`, `isBenchMode()`; change `otos()` return
- `source/hal/NezhaHAL.cpp` — init `_otosActive = &_otos` in ctor;
  implement `setOtosBench()`
- `source/robot/Robot.h` — declare `benchOtosTick()`, `isBenchOtosActive()`,
  `_lastBenchTickMs`
- `source/robot/Robot.cpp` — implement `benchOtosTick()`,
  `isBenchOtosActive()`
- `source/control/LoopTickOnce.cpp` — one-line insert before OTOS block

### Testing Plan

Build + host-test suite. No new test files in this ticket; the integrator test
is ticket 004.

### Post-Sprint Validation Note

Hardware flash and live bench validation is the team-lead's job after sprint
closes.
