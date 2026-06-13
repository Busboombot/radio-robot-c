---
id: '006'
title: Exclude BenchOtosSensor and DBG OTOS commands from production build
status: open
use-cases:
- SUC-034-004
depends-on:
- '004'
- '005'
github-issue: ''
issue: hardware-tick-actuator-state.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Exclude BenchOtosSensor and DBG OTOS commands from production build

## Description

`BenchOtosSensor.cpp` lives under `source/hal/` (auto-globbed into the firmware build) and is currently compiled into every firmware build, including production. Similarly, the `DBG OTOS` and `DBG OTOS BENCH` command registrations in `DebugCommandable::getCommands()` appear in every build. This ticket adds a production-exclusion guard so that bench-testing infrastructure is absent from fielded firmware.

### Approach options (choose one, verify with CMakeLists.txt)

**Option A — CMakeLists.txt filter**: add a `list(FILTER SOURCE_FILES EXCLUDE REGEX ".*/BenchOtosSensor.*")` line for production builds, mirroring the existing `hal/mock/` filter. This requires a production-vs-debug build configuration toggle in CMakeLists.txt.

**Option B — `#ifndef BENCH_BUILD` guard in BenchOtosSensor.cpp**: wrap the entire `.cpp` content in `#ifndef BENCH_BUILD ... #endif`. In production (no `BENCH_BUILD` define), the translation unit compiles to nothing. Simpler than CMake filter.

Check which approach is consistent with how `source/hal/mock/` is excluded. If mock is excluded via CMake filter, use Option A for consistency. If the project has no build-configuration toggle yet (only `HOST_BUILD`), Option B may be simpler.

For the `DBG OTOS` command registrations: wrap the relevant entries in `DebugCommandable::getCommands()` with `#ifdef BENCH_BUILD` (or `#ifndef BENCH_BUILD` skips them). This ensures the production command table contains no DBG OTOS handlers.

### Completeness

After this ticket, both linked issues are fully addressed:
- `hardware-tick-actuator-state.md`: all six changes done (tickets 001–006).
- `bench-otos-synthetic-otos-sensor-for-full-stack-bench-testing.md`: the one loose end (DBG OTOS hardware readout, F1) fixed in ticket 004; production exclusion done here.

## Files to Modify

- `source/hal/BenchOtosSensor.cpp` (Option B) — add `#ifndef BENCH_BUILD` guard OR leave untouched if using Option A.
- `CMakeLists.txt` (Option A) — add source filter for `BenchOtosSensor` in production builds.
- `source/app/DebugCommandable.cpp` — wrap `DBG OTOS` and `DBG OTOS BENCH` command registrations in `#ifdef BENCH_BUILD` guards inside `getCommands()`.

## Acceptance Criteria

- [ ] A production build (`python3 build.py` without bench define) does not link `BenchOtosSensor` symbols. Confirm by checking that `BenchOtosSensor` class methods are not present in the build output (e.g. grep the `.map` file if available, or verify by removing the class and confirming the build still succeeds).
- [ ] `DBG OTOS` and `DBG OTOS BENCH` commands are absent from the production command table (wrapped in `#ifdef BENCH_BUILD`).
- [ ] The bench build (with `BENCH_BUILD` define) still compiles `BenchOtosSensor` and registers the DBG commands — bench functionality is preserved.
- [ ] `python3 build.py` exits clean (production configuration).
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` exits green (sim suite; HOST_BUILD has its own guards unaffected by this).

## Implementation Plan

1. Read `CMakeLists.txt` to understand how `source/hal/mock/` is excluded (CMake filter vs `HOST_BUILD` guard). Determine which approach is consistent for `BenchOtosSensor`.
2. Implement the exclusion for `BenchOtosSensor.cpp` using the chosen approach.
3. In `DebugCommandable::getCommands()`, locate the `DBG OTOS BENCH` and `DBG OTOS` registration entries. Wrap them in `#ifdef BENCH_BUILD ... #endif`.
4. Build production configuration; verify `BenchOtosSensor` is absent.
5. Build bench configuration (if a separate build target/define exists); verify it still compiles.
6. Run sim suite.

## Testing

- **Build gate**: `python3 build.py` clean (production).
- **Sim gate**: `uv run --with pytest python -m pytest host_tests/ host/tests/` green.
- No new tests for this ticket; the acceptance grep/build checks are the gate.
