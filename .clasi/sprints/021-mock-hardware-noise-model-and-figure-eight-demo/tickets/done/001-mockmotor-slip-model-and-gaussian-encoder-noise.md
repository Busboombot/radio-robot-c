---
id: '001'
title: MockMotor slip model and Gaussian encoder noise
status: done
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: plan-demo-figure-eight-ipynb-pure-pursuit-with-sensor-fusion.md
completes_issue: false
---

# MockMotor slip model and Gaussian encoder noise

## Description

`MockMotor.tick()` currently integrates commanded speed into encoder mm with no error.
This ticket adds a slip model and Gaussian encoder noise so the simulated encoder
dead-reckoning produces realistic drift (~1% per metre).

The slip model applies a fractional under-report to the encoder accumulation:
`slip = slipStraight + slipTurnExtra * turnRate`. Turn rate is a value in [0,1]
set by `MockHAL` before each `tick()` call. Gaussian noise uses `std::mt19937` +
`std::normal_distribution<float>` (host-only; `<random>` is available in the host
build).

A `_trueVelMms` field stores the pre-slip velocity so `MockHAL` can build an oracle
pose from it via `ExactPoseTracker` (ticket 002).

## Acceptance Criteria

- [x] `MockMotor.h` adds fields: `_turnRate`, `_slipStraight`, `_slipTurnExtra`,
  `_encoderNoiseSigma`, `_trueVelMms`, per-object `std::mt19937 _rng`.
- [x] `MockMotor.h` adds methods: `setSlip(float straight, float turnExtra)`,
  `setEncoderNoise(float sigmaMm)`, `setTurnRate(float r)`, `trueVelocityMms() const`.
- [x] The no-op `setNoiseMms()` stub is removed (confirm no callers with grep first).
- [x] `MockMotor::tick()` computes `_trueVelMms = vel`, then `slip = _slipStraight +
  _slipTurnExtra * _turnRate`, then `noisy = vel * (1 - slip) + gaussianNoise(...)`,
  then `_encoderMm += noisy * dt_s`.
- [x] With `setSlip(0.005, 0.03)` and `setEncoderNoise(0.0)`, driving straight at
  400 mm/s for 1000 ms produces `encoderMm()` approximately 1% less than
  `trueVelocityMms() * 1.0s`.
- [x] With `_turnRate = 1.0`, slip equals `slipStraight + slipTurnExtra`.
- [x] With zero slip and zero noise (defaults), `tick()` behaviour is identical to
  the pre-sprint implementation (no regressions in existing tests).
- [x] All RNG fields are guarded with `#ifdef HOST_BUILD` or `<random>` is included
  only in the `.cpp` translation unit (never pulled into firmware compilation).
- [x] `libfirmware_host` builds cleanly: `cmake --build host_tests/build`.
- [x] `uv run --with pytest python -m pytest` passes with no regressions.

## Implementation Plan

### Approach

Extend `MockMotor.h` with new fields and setters. Extend `MockMotor.cpp::tick()` to
apply slip and noise. Keep defaults at zero so existing tests see no change.

### `<random>` guard

The `std::mt19937` member must not appear in a header included by firmware compilation.
Options:
1. Declare `_rng` in the header inside `#ifdef HOST_BUILD` ... `#endif`.
2. Or: include `<random>` in `MockMotor.cpp` only; forward-declare the RNG as
   `uint32_t _rngState[2]` and reinterpret in `.cpp` (more complex).

Option 1 is simplest. Use it unless the CMake build flags make `HOST_BUILD` unavailable
in the header context (it is defined by the host CMake target).

### Files to modify

- `source/hal/mock/MockMotor.h` — add fields, setters, `trueVelocityMms()`; remove
  `setNoiseMms()`
- `source/hal/mock/MockMotor.cpp` — extend `tick()`; add `gaussianNoise()` helper

### Gaussian noise helper

```cpp
// In MockMotor.cpp — include <random> at top (HOST_BUILD-only compilation unit)
static float gaussianNoise(std::mt19937& rng, float sigma) {
    if (sigma <= 0.0f) return 0.0f;
    std::normal_distribution<float> dist(0.0f, sigma);
    return dist(rng);
}
```

Seed `_rng` in the `MockMotor` default constructor with a fixed constant (e.g., `42u`)
for reproducibility.

### Testing plan

- Existing pytest suite must pass unchanged (slip=0, noise=0 default).
- Manual verification: construct a `MockMotor`, call `setSlip(0.01, 0.0)`,
  `setTurnRate(0.0)`, `tick(1000)` at speed 100 → expect `encoderMm()` ~= 396 mm
  (400 * 0.99) and `trueVelocityMms()` ~= 400 mm/s.
- Manual: with `setTurnRate(1.0)` and `setSlip(0.005, 0.03)`, slip = 0.035 →
  `encoderMm()` ~= 400 * 0.965 = 386 mm.

### Documentation updates

Architecture update section "MockMotor" already documents the new interface. No
additional docs needed.
