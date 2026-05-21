---
id: '001'
title: Add ratio PID params to Config.h and create RatioPidController
status: done
use-cases: []
depends-on: []
github-issue: ''
issue:
- nezha-ratio-pid-algorithm.md
- firmware-ratio-pid-and-g-command.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add ratio PID params to Config.h and create RatioPidController

## Description

Sprint 4 replaces MotorController's existing PI+FF tick body with a cumulative-distance
ratio PID algorithm. Before that replacement (ticket 002), this ticket establishes two
foundations:

1. **Audit `source/types/Config.h`** — confirm which CalibParams fields are already present
   from prior sprints and add any that are missing. The sprint.md lists the full set of
   required fields.

2. **Create `source/control/RatioPidController.h` and `.cpp`** — a standalone discrete PID
   class that the new MotorController tick body will use.

### Config.h Audit

Read `source/types/Config.h`. The current file already has:

- `mmPerDegL`, `mmPerDegR`, `kFF` — encoder and FF gains (sprint 1)
- `kScaleLF`, `kScaleLB`, `kScaleRF`, `kScaleRB` — per-direction PWM scale factors
- `kAdjThreshold`, `kAdjGain` — slower-wheel adjustment params
- `trackwidthMm`, `ratioPidKp`, `ratioPidKi`, `ratioPidKd`, `ratioPidMax` — ratio PID and arc params
- `turnThresholdMm`, `doneTolMm` — G-command completion params

All required fields are present. **No fields need to be added** in this ticket. The K command
names that map to these fields are:

| K command | CalibParams field   | Default |
|-----------|---------------------|---------|
| KLF       | kScaleLF            | 1.0     |
| KLB       | kScaleLB            | 1.0     |
| KRF       | kScaleRF            | 1.0     |
| KRB       | kScaleRB            | 1.0     |
| KCP       | ratioPidKp          | 300.0   |
| KCI       | ratioPidKi          | 0.0     |
| KCD       | ratioPidKd          | 0.0     |
| KCC       | ratioPidMax         | 30.0    |
| KAT       | kAdjThreshold       | 0.5     |
| KAG       | kAdjGain            | 0.05    |
| KTW       | trackwidthMm        | 120.0   |
| KGT       | turnThresholdMm     | 5.0     |
| KGD       | doneTolMm           | 3.0     |

Note: `turnThresholdMm` and `doneTolMm` name the degrees/mm threshold for G-command turn
decision and done tolerance respectively. The sprint.md spec uses KGT (turn threshold degrees,
default 50) but the existing field is `turnThresholdMm = 5.0`. Confirm with actual CommandProcessor
code what these fields are used for currently. If the existing defaults conflict with the
G-command design, update `defaultCalibParams()` — the defaults for KGT should be 50.0 (degrees)
and for KGD should be 5.0 (mm done tolerance).

### RatioPidController

Create two new files: `source/control/RatioPidController.h` and `source/control/RatioPidController.cpp`.

**Header (`RatioPidController.h`):**

```cpp
#pragma once

/**
 * RatioPidController — standard discrete PID with anti-windup integral clamp.
 *
 * Used by MotorController to compute faster-wheel correction in the
 * cumulative-distance ratio PID algorithm.
 *
 * The `integral` field is public so the slower-wheel adjustment in
 * MotorController can read it directly without a getter.
 */
class RatioPidController {
public:
    RatioPidController(float kP, float kI, float kD, float iClamp);

    /**
     * Compute one PID step.
     * @param error  Normalized error (dimensionless fraction).
     * @param dtS    Elapsed time since last call in seconds.
     * @return       Correction in PWM% units.
     */
    float update(float error, float dtS);

    /** Reset integrator and derivative state. Call on new command start. */
    void reset();

    float integral;  // public — read by slower-wheel adjustment logic

private:
    float _kP;
    float _kI;
    float _kD;
    float _iClamp;
    float _prevError;
    bool  _firstCall;
};
```

**Implementation (`RatioPidController.cpp`):**

The `update()` algorithm (standard discrete PID):

```
integral += kI * error * dtS
integral = clamp(integral, -iClamp, +iClamp)
if (_firstCall):
    deriv = 0
    _firstCall = false
else:
    deriv = (error - _prevError) / dtS
output = _kP * error + integral + _kD * deriv
_prevError = error
return output
```

The `reset()` method sets `integral = 0`, `_prevError = 0`, `_firstCall = true`.

The constructor sets all fields from parameters and calls `reset()`.

Use a static `clamp` helper: `static float clamp(float v, float lo, float hi)`.

## Acceptance Criteria

- [x] `source/control/RatioPidController.h` exists with the class declaration shown above
- [x] `source/control/RatioPidController.cpp` exists and implements `update()`, `reset()`, and the constructor
- [x] `update()` with kI=0, kD=0: output == kP * error on every call regardless of dtS
- [x] `update()` first call: derivative term is 0 (no division by dtS on first call)
- [x] `reset()` zeroes `integral` and `_prevError` and sets `_firstCall = true`
- [x] `integral` does not exceed `iClamp` magnitude (anti-windup clamp)
- [x] `source/types/Config.h` has all 13 K-command fields with correct defaults (audit confirms — no changes needed if already present)
- [x] `defaultCalibParams()` sets `turnThresholdMm = 50.0` (degrees) and `doneTolMm = 5.0` (mm) matching G-command intent; update if current defaults differ
- [x] Project builds without errors: `python build.py`

## Implementation Plan

### Files to create

- `source/control/RatioPidController.h` — class declaration
- `source/control/RatioPidController.cpp` — implementation

### Files to modify

- `source/types/Config.h` — audit and update `defaultCalibParams()` defaults if turnThresholdMm or doneTolMm are wrong
- Add `RatioPidController.cpp` to the build system if the project uses an explicit source list (check `build.py` or `CMakeLists.txt` or `Makefile` for how `source/control/MotorController.cpp` is included — follow the same pattern)

### Testing

This ticket has no hardware-in-the-loop tests. Correctness is verified by building without
errors and by inspecting the implementation against the algorithm above. The real validation
comes in ticket 002 (MotorController) and ticket 004 (hardware deploy).

- **Build verification**: `python build.py` must complete without errors.
- **No unit test framework** exists on this embedded target — correctness is by inspection
  and integration test in ticket 004.
