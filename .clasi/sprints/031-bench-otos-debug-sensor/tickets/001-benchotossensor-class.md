---
id: "001"
title: "BenchOtosSensor class"
status: open
use-cases: [SUC-001, SUC-002, SUC-004]
depends-on: []
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# BenchOtosSensor class

## Description

Create `source/hal/BenchOtosSensor.h` and `source/hal/BenchOtosSensor.cpp`:
a concrete `IOtosSensor` that synthesizes OTOS pose by integrating commanded
wheel velocity each control tick. This is the foundation the rest of the sprint
builds on.

**Why**: The real OtosSensor sees no motion when the robot is on a stand. This
class gives the firmware a plausible OTOS input so the full stack (EKF, distance
stops, TLM) can be validated on the bench.

**Approach**:
- Port the arc-integration math from `source/hal/mock/MockOtosSensor.cpp`
  (`tick()` method) — do not `#include` MockOtosSensor, copy-adapt the math.
- Maintain two independent accumulators:
  - `_idealX/Y/H` — noiseless integration (ground truth for `DBG OTOS` query)
  - `_otosX/Y/H` — errored integration (what `readTransformed` returns)
- Error model: Gaussian noise on each arc step (linear sigma `_noiseXY`,
  heading sigma `_noiseH`) plus a slow yaw drift added each tick (`_driftRad`
  per second, scaled by dt).
- PRNG: `#ifdef HOST_BUILD` use a deterministic Box-Muller with a fixed-seed
  LCG (or any deterministic normal approximation); `#else` use `microbit_random`
  (returns uint32, scale to [-1,1] then multiply by sigma). `std::mt19937` and
  `std::normal_distribution` are NOT available on CODAL.
- `readTransformed()` — writes `_otosX/Y/H` to `poseOut`, returns `true`
  (always-valid). Ignores `cfg` and `headingRad` (no lever-arm in bench mode).
- `readVelocityTransformed()` — returns last-tick velocity derived from the arc
  step (`dC/dt` and `dTh/dt`), returns `true`.
- `readStatus()` — returns `true`, writes `out = 0` (valid status).
- `lastReadOk()` — returns `true`.
- `readAccelTransformed()` — returns `{0,0}`.
- Calibration stubs (`init`, `calibrateImu`, `resetTracking`, `getPositionRaw`,
  `setPositionRaw`, `getLinearScalar`, `setLinearScalar`, `getAngularScalar`,
  `setAngularScalar`) — all no-ops; `get*` return 0.
- `begin()` — sets `is_initialized()` true, always succeeds.
- Heading wrapping: keep `_idealH` and `_otosH` in `[-pi, pi]`.
- Public interface for callers:
  - `tick(float tgtLMms, float tgtRMms, float trackwidthMm, uint32_t dt_ms)` —
    advances both accumulators one step. When `dt_ms == 0` or the sensor is not
    initialized, this is a no-op.
  - `idealPose(OtosPose& out) const` — fills `out` with `_idealX/Y/H` (for
    the `DBG OTOS` query).
  - `setNoise(float noiseXY, float noiseH, float driftRadPerSec)` — updates
    error model params at runtime (called by `DBG OTOS BENCH` with optional KV
    args).

**Read first**: `source/hal/IOtosSensor.h` for exact post-030 interface
signatures before writing any code. `source/hal/mock/MockOtosSensor.cpp` for
the arc integration pattern to port.

## Acceptance Criteria

- [ ] `source/hal/BenchOtosSensor.h` exists and declares `BenchOtosSensor`
  inheriting `IOtosSensor`.
- [ ] `source/hal/BenchOtosSensor.cpp` implements all `IOtosSensor` virtual
  methods; calibration stubs compile without warnings.
- [ ] `readTransformed()` and `readVelocityTransformed()` return `true`.
- [ ] `tick()` with zero dt is a no-op (no accumulator change).
- [ ] With zero noise/drift, a single tick with `tgtL = tgtR = 100 mm/s` for
  10 ms produces `_idealX` approximately `+1.0 mm` and `_otosX` within float
  precision of `_idealX`.
- [ ] `python3 build.py` clean build passes (device firmware compiles).
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes
  (no existing tests regress).

## Implementation Plan

### Approach

New files only; no existing files modified in this ticket. Port the tick math
from `MockOtosSensor::tick()`, adapt for dual accumulators and the
firmware-safe PRNG.

### Files to Create

- `source/hal/BenchOtosSensor.h` — class declaration + public interface
- `source/hal/BenchOtosSensor.cpp` — implementation

### Files to Modify

None in this ticket.

### Testing Plan

The formal unit tests live in ticket 004. This ticket's acceptance gate is:
(a) clean build passes, and (b) existing host tests pass without regression.
Manually verify tick math with the spot-check in acceptance criteria above.

### Post-Sprint Validation Note

Hardware flash and `DBG OTOS BENCH 1` live bench validation is the
team-lead's job after the sprint closes. Do not gate this ticket on hardware
execution.
