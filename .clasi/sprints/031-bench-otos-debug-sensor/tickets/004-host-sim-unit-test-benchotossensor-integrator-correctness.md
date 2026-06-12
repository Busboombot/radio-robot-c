---
id: "004"
title: "Host-sim unit test: BenchOtosSensor integrator correctness"
status: open
use-cases: [SUC-004]
depends-on: ["031-001"]
github-issue: ""
issue: ""
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host-sim unit test: BenchOtosSensor integrator correctness

## Description

Write `host_tests/test_bench_otos.cpp` (or the equivalent test file location
for this project's host-sim test suite — check existing test files under
`host_tests/` to determine naming convention and CMake/pytest hook).

**Why**: The `BenchOtosSensor` integrator produces synthesized pose that the
EKF fuses. A correctness test running pre-flash catches integration bugs (wrong
heading convention, missing dt scaling, drift sign error) that would otherwise
only show up during bench sessions.

**Test cases**:

1. **Zero-noise oracle** — Construct `BenchOtosSensor` with
   `setNoise(0, 0, 0)`. Call `begin()`. Drive straight: call
   `tick(100.0f, 100.0f, trackwidth, 10)` 100 times (100 ms total, 10 ms each).
   Expected: `idealPose()` returns `x ≈ 10.0 mm` (100 mm/s * 0.1 s), `y ≈ 0`,
   `h ≈ 0`. Tolerance: ±0.1 mm, ±0.001 rad. Also verify `readTransformed()`
   returns `true` and `poseOut.x ≈ 10.0 mm` (zero noise → ideal == otos).

2. **Arc turn oracle** — Zero noise. Call `tick(100.0f, 0.0f, 100.0f, 10)` 100
   times (left wheel at 100 mm/s, right at 0, trackwidth=100 mm). Expected arc:
   radius = 50 mm, angular rate = 1.0 rad/s, after 0.1 s heading ≈ 0.1 rad,
   `x ≈ sin(0.1)*50 mm`, `y ≈ (1-cos(0.1))*50 mm`. Tolerance: ±0.1 mm, ±0.001 rad.

3. **Noise band** — Construct with `setNoise(0.02f, 0.001f, 0.0f)` (2% linear
   noise, 0.001 rad heading noise). Drive straight 100 ticks as in case 1.
   Run 50 independent trials (reset sensor between trials). Assert that the
   mean `x` is within ±0.5 mm of 10.0 and the standard deviation of `x` is
   less than 1.0 mm. (Statistical, not deterministic — uses fixed HOST_BUILD
   seed for reproducibility.)

4. **Zero-dt no-op** — Call `tick(100.0f, 100.0f, 100.0f, 0)`. Assert pose
   unchanged (still at origin).

**Read first**: Look at existing test files in `host_tests/` to understand the
test framework used (e.g., pytest-based C extension via ctypes, or a standalone
C++ test runner). Match the convention exactly — do not introduce a new test
framework. Also check `CMakeLists.txt` or `build.py` to see how host test
sources are registered; register the new file the same way.

## Acceptance Criteria

- [ ] `host_tests/test_bench_otos.cpp` (or equivalent) exists and compiles
  under `HOST_BUILD`.
- [ ] Zero-noise straight-drive oracle test passes: `x ≈ 10.0 mm` within
  ±0.1 mm after 100 ticks at 100 mm/s.
- [ ] Zero-noise arc-turn oracle test passes within ±0.1 mm / ±0.001 rad.
- [ ] Noise-band test passes (statistical, reproducible via fixed HOST_BUILD
  seed).
- [ ] Zero-dt no-op test passes.
- [ ] `python3 build.py` clean build passes.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes,
  including the new test.

## Implementation Plan

### Approach

New test file only. Follow the existing host-test convention. Register in build
system as needed.

### Files to Create

- `host_tests/test_bench_otos.cpp` (or matching filename per convention)

### Files to Modify

- `CMakeLists.txt` or `build.py` (only if new test source must be registered
  explicitly — check whether existing tests are auto-discovered or listed)

### Testing Plan

This ticket IS the test. The acceptance gate is that all four test cases pass
under `uv run --with pytest python -m pytest host_tests/ host/tests/`.

### Post-Sprint Validation Note

Hardware bench validation is the team-lead's job after sprint closes.
