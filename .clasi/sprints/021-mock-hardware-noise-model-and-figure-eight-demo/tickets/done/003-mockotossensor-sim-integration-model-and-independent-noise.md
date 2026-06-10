---
id: '003'
title: MockOtosSensor sim integration model and independent noise
status: done
use-cases:
- SUC-003
depends-on:
- '002'
github-issue: ''
issue: ''
completes_issue: false
---

# MockOtosSensor sim integration model and independent noise

## Description

`MockOtosSensor` currently returns only a zero pose or an injected pose. This ticket
adds a sim-driven integration model that accumulates pose from true motor velocities
with independent Gaussian linear and yaw noise. When enabled, `readTransformed()`
returns the accumulated noisy pose rather than the injected pose.

The model is disabled by default so all existing tests continue to use the injection
API unchanged. `MockHAL::tick()` calls `_otos.tick(...)` each step (the TODO comment
left by ticket 002 is filled in here).

Noise magnitudes that match real OTOS characteristics:
- `linearNoise = 0.01` (1% fractional noise on displacement per step)
- `yawNoise = 0.025` (2.5% fractional noise on angle per step)

These are defaults; all are settable at runtime.

## Acceptance Criteria

- [x] `MockOtosSensor.h` adds fields: `_useSimModel` (bool, default false),
  `_linearNoiseSigma` (float, default 0.0f), `_yawNoiseSigma` (float, default 0.0f),
  `_odomX/Y/H` (float, default 0.0f each), per-object `std::mt19937 _rng`.
- [x] `MockOtosSensor.h` adds methods: `enableSimModel(bool on)`, `setLinearNoise(float sigma)`,
  `setYawNoise(float sigma)`, and `tick(float velLMms, float velRMms, float trackwidthMm,
  uint32_t dt_ms)`.
- [x] `MockOtosSensor::tick()` is a no-op when `!_useSimModel`. When enabled:
  integrates `dC = (vL+vR)/2 * dt_s`, `dTh = (vR-vL)/tw * dt_s`, applies fractional
  Gaussian noise to both (`noisyDC = dC*(1+N(0,linearSigma))`,
  `noisyDTh = dTh*(1+N(0,yawSigma))`), then does midpoint integration into
  `_odomX/Y/H`.
- [x] `MockOtosSensor::readTransformed()` returns `OtosPose{_odomX, _odomY, _odomH}`
  when `_useSimModel`, and the existing injected pose otherwise. The `const` qualifier
  is preserved (no mutable fields needed since `tick()` is called before `readTransformed()`).
- [x] `MockOtosSensor::setInjectedPose()` also resets `_odomX = x`, `_odomY = y`,
  `_odomH = h` so camera fixes reset the OTOS accumulator.
- [x] `MockHAL::tick()` calls `_otos.tick(_motorL.trueVelocityMms(),
  _motorR.trueVelocityMms(), _trackwidthMm, udt)` (replacing the TODO left by ticket 002).
- [x] All RNG/`<random>` references in the header are guarded with `#ifdef HOST_BUILD`.
- [x] `libfirmware_host` builds cleanly.
- [x] `uv run --with pytest python -m pytest` passes with no regressions.
- [x] OTOS pose drifts differently from encoder dead-reckoning after enabling the
  model (verified manually: different noise realisations, different drift vectors).

## Implementation Plan

### Approach

Extend `MockOtosSensor.h` with new fields and methods, following the same `<random>`
guard pattern as ticket 001. Extend `MockOtosSensor.cpp::readTransformed()` with a
branch on `_useSimModel`. Add `tick()` implementation to `MockOtosSensor.cpp`.

Complete the OTOS tick call in `MockHAL::tick()` (which was left as a TODO comment
in ticket 002).

### `OtosPose` return type

Confirm the exact type returned by `readTransformed()` — it is `OtosPose` (a struct
with `x, y, h` or similar fields). Check `IOtosSensor.h` for the definition before
writing the return statement.

### `_odomH` wraparound

The yaw accumulator should be wrapped to `[-pi, pi]` to avoid unbounded growth. Use a
simple `wrapPi(h)` helper (same as the one in `Odometry.cpp` if accessible, or
implement inline: `while (h > M_PI) h -= 2*M_PI; while (h < -M_PI) h += 2*M_PI;`).

### Files to modify

- `source/hal/mock/MockOtosSensor.h` — add fields, methods
- `source/hal/mock/MockOtosSensor.cpp` — implement `tick()`, update `readTransformed()`,
  update `setInjectedPose()`
- `source/hal/mock/MockHAL.cpp` — complete the `_otos.tick(...)` call in `tick()`

### Testing plan

- Existing pytest suite must pass unchanged (model disabled by default).
- Manual: enable model, set `linearNoise=0`, `yawNoise=0`. Drive straight. `get_otos_pose()`
  should match `get_exact_pose()` (noise-free integration).
- Manual: enable model, set `linearNoise=0.01`, `yawNoise=0.025`. Drive a figure-eight
  loop. `get_otos_pose()` should diverge from `get_exact_pose()` by a different amount
  than encoder dead-reckoning diverges (demonstrating independent noise).

### Documentation updates

No additional docs needed; architecture update describes the sim model fully.
