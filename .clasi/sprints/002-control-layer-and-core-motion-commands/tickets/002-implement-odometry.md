---
id: '002'
title: Implement Odometry
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: plan-c-port-of-radio-robot-firmware
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement Odometry

## Description

Create `source/control/Odometry.h` and `source/control/Odometry.cpp`.

Odometry implements differential-drive dead-reckoning. It maintains internal
floating-point pose state (x, y, heading in radians) and exposes integer
protocol output (mm for position, centidegrees for heading). It has no
dependencies on any hardware or other sprint-2 modules — it is a pure math
class called by CommandProcessor on each tick.

This replaces the OTOS sensor pose queries (SO command previously read from
the SparkFun OTOS chip via `otos.getPositionRaw()`). In this sprint,
`SO` returns dead-reckoning data from this class.

## Header — `source/control/Odometry.h`

```cpp
#pragma once
#include <stdint.h>

/**
 * Odometry — differential-drive dead-reckoning pose tracker.
 *
 * Internal state is float for accuracy; protocol output is integer.
 * Heading convention: 0 = +X axis, positive = CCW (standard math).
 * Output convention: centidegrees (360 degrees = 36000 cdeg).
 *
 * Caller (CommandProcessor) must call update() once per tick with
 * the encoder deltas for that tick in mm.
 */
class Odometry {
public:
    Odometry();

    // Integrate one tick's wheel travel.
    // dL_mm, dR_mm: signed mm traveled by left and right wheels this tick.
    // trackwidthMm: distance between wheel contact points in mm.
    void update(float dL_mm, float dR_mm, float trackwidthMm);

    // Read current pose. x_mm and y_mm are integer mm; h_cdeg is
    // centidegrees (-18000..+18000 clamped).
    void getPose(int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg) const;

    // Overwrite pose (used by SI command).
    // h_cdeg is centidegrees; stored internally as radians.
    void setPose(int32_t x_mm, int32_t y_mm, int32_t h_cdeg);

    // Zero pose: equivalent to setPose(0, 0, 0).
    void zero();

private:
    float _x;          // mm, float internal
    float _y;          // mm, float internal
    float _headingRad; // radians

    static constexpr float PI_F       = 3.14159265f;
    static constexpr float RAD_TO_CDEG = 18000.0f / 3.14159265f;
    static constexpr float CDEG_TO_RAD = 3.14159265f / 18000.0f;
};
```

## Implementation — `source/control/Odometry.cpp`

### Constructor

Zero all state: `_x = _y = _headingRad = 0.0f`.

### `update(float dL_mm, float dR_mm, float trackwidthMm)`

Differential drive integration:

```
float dCenter = (dL_mm + dR_mm) * 0.5f;
float dTheta  = (dR_mm - dL_mm) / trackwidthMm;

_x          += dCenter * cosf(_headingRad);
_y          += dCenter * sinf(_headingRad);
_headingRad += dTheta;
```

Include `<cmath>` (or `<math.h>`) for `cosf` and `sinf`.
No heading wrapping is needed — the protocol output layer handles clamping.

### `getPose(int32_t& x_mm, int32_t& y_mm, int32_t& h_cdeg) const`

```
x_mm  = static_cast<int32_t>(_x);
y_mm  = static_cast<int32_t>(_y);
float cdeg = _headingRad * RAD_TO_CDEG;
if (cdeg >  18000.0f) cdeg =  18000.0f;
if (cdeg < -18000.0f) cdeg = -18000.0f;
h_cdeg = static_cast<int32_t>(cdeg);
```

### `setPose(int32_t x_mm, int32_t y_mm, int32_t h_cdeg)`

```
_x          = static_cast<float>(x_mm);
_y          = static_cast<float>(y_mm);
_headingRad = static_cast<float>(h_cdeg) * CDEG_TO_RAD;
```

### `zero()`

Call `setPose(0, 0, 0)`.

## Wire Format Note

The `SO` command response is formatted by CommandProcessor, not Odometry.
Format: `SO+XXXX-YYYY+HHHH` — each field carries a mandatory sign prefix.
Example: `SO+0500+0000+00000` (500 mm forward, no lateral drift, 0 heading).
CommandProcessor uses `getPose()` output to assemble this string.

## Files to Create

- `source/control/Odometry.h`
- `source/control/Odometry.cpp`

## Files to Read First

- `source/types/Config.h` — confirm no existing types conflict
- `/Volumes/Proj/proj/league-projects/scratch/radio-robot/src/command.ts` lines 113–118 — `reportOdo()` for the original wire format (uses OTOS; this ticket replaces with dead-reckoning)

## Acceptance Criteria

- [x] `source/control/Odometry.h` exists with the exact public interface above
- [x] `source/control/Odometry.cpp` exists and implements all four methods
- [x] `update()` uses correct differential-drive integration: `dCenter = (dL+dR)/2`, `dTheta = (dR-dL)/trackwidth`
- [x] `getPose()` returns heading in centidegrees clamped to -18000..+18000
- [x] `setPose()` and `zero()` correctly update internal float state
- [ ] `python build.py` compiles without errors after this ticket

## Testing

- **Hardware-in-the-loop only** — CODAL does not support off-device unit tests
- **Build verification**: `python build.py` must succeed with zero errors
- **Functional test** (after ticket 003 wires it in): drive forward ~500 mm,
  then send `SO` over serial — verify x_mm is approximately 500, y_mm near 0,
  h_cdeg near 0
- **Verification command**: `python build.py`
