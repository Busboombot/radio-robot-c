---
id: '002'
title: Replace MotorController tick body with ratio PID algorithm
status: done
use-cases: []
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Replace MotorController tick body with ratio PID algorithm

## Description

Replace the existing PI + feed-forward tick body in MotorController with the
cumulative-distance ratio PID algorithm. The public interface (`setTarget()`, `stop()`,
`tick()`, `getActualVelocity()`, `getEncoderPositions()`) is **unchanged** — only the
internal implementation changes, plus two new public methods are added (`startDrive()` and
`startDriveClean()`).

The algorithm is confirmed working in TypeScript (340/339 mm over 2 s, 0.3% error). The
canonical spec is `.clasi/issues/nezha-ratio-pid-algorithm.md`. This ticket reproduces the
full algorithm so the programmer does not need to read other files.

### Background

The Nezha motors are dumb DC motors driven by PWM. The existing PI+FF controller compares
instantaneous velocities per tick, which is noisy at 20 ms. The ratio PID instead tracks
**cumulative encoder distance since the command started** and keeps the ratio of left:right
distance equal to the ratio of commanded speeds. Feed-forward is the primary drive signal;
PID provides correction only.

### New Private State in MotorController

Add these fields to `MotorController`'s private section (in `MotorController.h`).
Also add `#include "RatioPidController.h"` at the top of `MotorController.h`.

```cpp
// Ratio PID state (sprint 4 replacement of PI integrators)
RatioPidController _pid;         // constructed with cal params
float  _cmdEncStartL;            // encoder mm snapshot at command start (left)
float  _cmdEncStartR;            // encoder mm snapshot at command start (right)
float  _cmdRatio;                // |fasterSpeed| / |slowerSpeed|, always >= 1.0
bool   _fasterIsRight;           // true if right wheel is the commanded-faster wheel
float  _tgtLMms;                 // current speed targets in mm/s
float  _tgtRMms;
```

The existing `_integralL`, `_integralR` (sprint 2 PI state) are no longer used and should
be removed. The existing `_prevEncL`, `_prevEncR`, `_actualVelL`, `_actualVelR` are still
needed for `getActualVelocity()` — keep them.

### New Public Methods

Add to the public section of `MotorController.h`:

```cpp
/**
 * startDriveClean — used by T, D, and G commands.
 * Full clean start: snapshot encoders, compute ratio, reset PID.
 * Always call this when starting a new bounded command.
 */
void startDriveClean(float leftMms, float rightMms);

/**
 * startDrive — used by the S (streaming) command only.
 * Re-seeds cmdEncStart to preserve accumulated ratio history across keepalive re-sends.
 * Does NOT reset PID unless the faster/slower assignment changes.
 */
void startDrive(float leftMms, float rightMms);
```

### Constructor Change

The constructor `MotorController(NezhaV2& motor, const CalibParams& cal)` must initialize
`_pid` with `cal.ratioPidKp`, `cal.ratioPidKi`, `cal.ratioPidKd`, `cal.ratioPidMax`.
Also initialize `_cmdEncStartL = _cmdEncStartR = 0.0f`, `_cmdRatio = 1.0f`,
`_fasterIsRight = false`, `_tgtLMms = _tgtRMms = 0.0f`.

In C++ you cannot initialize a member that has no default constructor in the header field
declaration when you also define it in the class body. Initialize `_pid` in the constructor
initializer list:

```cpp
MotorController::MotorController(NezhaV2& motor, const CalibParams& cal)
    : _motor(motor), _cal(cal),
      _pid(cal.ratioPidKp, cal.ratioPidKi, cal.ratioPidKd, cal.ratioPidMax),
      _cmdEncStartL(0.0f), _cmdEncStartR(0.0f),
      _cmdRatio(1.0f), _fasterIsRight(false),
      _tgtLMms(0.0f), _tgtRMms(0.0f),
      _prevEncL(0), _prevEncR(0),
      _actualVelL(0.0f), _actualVelR(0.0f)
{
    // remove old PI field initialization
}
```

### `setTarget()` Change

`setTarget(leftMms, rightMms)` now stores the values in `_tgtLMms` / `_tgtRMms` only. It
does NOT call `startDriveClean` or `startDrive` — CommandProcessor is responsible for
calling the appropriate start method at command-start time (not every tick).

### `stop()` Change

`stop()` zeroes `_tgtLMms` and `_tgtRMms`, calls `_pid.reset()`, sets `_cmdEncStartL` and
`_cmdEncStartR` to the current encoder positions, then writes zero PWM to both motors via
the NezhaV2 API. This ensures the next command starts clean.

### `resetIntegrators()` Change

Now delegates to `_pid.reset()`. Remove old PI integrator reset.

---

## Full Algorithm

### Private Helper: `encoderMm(bool left)`

Read the NezhaV2 encoder and convert to mm. Match the sign convention used in sprint 2 so
that forward motion returns a positive value on both wheels.

```cpp
float MotorController::encoderMm(bool left) {
    // Check existing sprint-2 tick() for the correct motor enum and sign.
    // Pattern: degrees = _motor.readRelAngle(left ? M2 : M1)
    // Apply LEFT_FWD_SIGN / RIGHT_FWD_SIGN negation if present in sprint-2 code.
    float deg = static_cast<float>(_motor.readRelAngle(left ? MotorNum::M2 : MotorNum::M1));
    return deg * (left ? _cal.mmPerDegL : _cal.mmPerDegR);
}
```

### `startDriveClean(float leftMms, float rightMms)`

```cpp
void MotorController::startDriveClean(float leftMms, float rightMms) {
    _tgtLMms = leftMms;
    _tgtRMms = rightMms;
    _fasterIsRight = (fabsf(rightMms) >= fabsf(leftMms));
    float fasterAbs = _fasterIsRight ? fabsf(rightMms) : fabsf(leftMms);
    float slowerAbs = _fasterIsRight ? fabsf(leftMms)  : fabsf(rightMms);
    _cmdRatio = (slowerAbs > 0.0f) ? (fasterAbs / slowerAbs) : 1.0f;
    _cmdEncStartL = encoderMm(true);
    _cmdEncStartR = encoderMm(false);
    _pid.reset();
}
```

### `startDrive(float leftMms, float rightMms)`

The S command is re-sent every ~150 ms as a keepalive. Re-seed cmdEncStart so the PID
accumulates continuously without a startup spike on each keepalive.

```cpp
void MotorController::startDrive(float leftMms, float rightMms) {
    _tgtLMms = leftMms;
    _tgtRMms = rightMms;

    bool newFasterIsRight = (fabsf(rightMms) >= fabsf(leftMms));
    float newFasterAbs = newFasterIsRight ? fabsf(rightMms) : fabsf(leftMms);
    float newSlowerAbs = newFasterIsRight ? fabsf(leftMms)  : fabsf(rightMms);
    float newRatio = (newSlowerAbs > 0.0f) ? (newFasterAbs / newSlowerAbs) : 1.0f;

    float curL = encoderMm(true);
    float curR = encoderMm(false);
    float curFaster  = newFasterIsRight ? curR : curL;
    float curSlower  = newFasterIsRight ? curL : curR;
    float startFaster = newFasterIsRight ? _cmdEncStartR : _cmdEncStartL;
    float prevDeltaFaster = fabsf(curFaster - startFaster);

    float seedFaster = fmaxf(prevDeltaFaster, newFasterAbs);
    float seedSlower = seedFaster / newRatio;

    float signFaster = ((newFasterIsRight ? rightMms : leftMms) >= 0.0f) ? 1.0f : -1.0f;
    float signSlower = ((newFasterIsRight ? leftMms  : rightMms) >= 0.0f) ? 1.0f : -1.0f;

    float newStartFaster = curFaster - signFaster * seedFaster;
    float newStartSlower = curSlower - signSlower * seedSlower;

    if (newFasterIsRight) {
        _cmdEncStartR = newStartFaster;
        _cmdEncStartL = newStartSlower;
    } else {
        _cmdEncStartL = newStartFaster;
        _cmdEncStartR = newStartSlower;
    }

    if (newFasterIsRight != _fasterIsRight) _pid.reset();
    _fasterIsRight = newFasterIsRight;
    _cmdRatio = newRatio;
}
```

### `tick(float dt_s)` — Full Replacement Body

Replace the entire existing tick() body with the following. Check the existing sprint-2 code
for the correct NezhaV2 API to set motor PWM (the line marked `// SET PWM`).

```cpp
void MotorController::tick(float dt_s) {
    // Step 1: Read encoder positions
    float encLMm = encoderMm(true);
    float encRMm = encoderMm(false);

    // Update velocity for getActualVelocity()
    _actualVelL = (encLMm - static_cast<float>(_prevEncL)) / dt_s;
    _actualVelR = (encRMm - static_cast<float>(_prevEncR)) / dt_s;
    _prevEncL = static_cast<int32_t>(encLMm);
    _prevEncR = static_cast<int32_t>(encRMm);

    // If no drive command active, ensure motors are stopped
    if (_tgtLMms == 0.0f && _tgtRMms == 0.0f) {
        // SET PWM: write 0, 0 via NezhaV2
        return;
    }

    // Step 2: Cumulative deltas since command start
    float fDL = encLMm - _cmdEncStartL;
    float fDR = encRMm - _cmdEncStartR;
    float fasterDelta = _fasterIsRight ? fabsf(fDR) : fabsf(fDL);
    float slowerDelta  = _fasterIsRight ? fabsf(fDL) : fabsf(fDR);

    // Step 3: Normalized error
    float expected = slowerDelta * _cmdRatio;
    float normErr  = (expected - fasterDelta) / fmaxf(1.0f, expected);

    // Step 4: PID update
    float correction = _pid.update(normErr, dt_s);

    // Step 5: Feed-forward base PWM
    float scaleL = (_tgtLMms >= 0.0f) ? _cal.kScaleLF : _cal.kScaleLB;
    float scaleR = (_tgtRMms >= 0.0f) ? _cal.kScaleRF : _cal.kScaleRB;
    float tgtFasterAbs = _fasterIsRight ? fabsf(_tgtRMms) : fabsf(_tgtLMms);
    float tgtSlowerAbs = _fasterIsRight ? fabsf(_tgtLMms) : fabsf(_tgtRMms);
    float scaleFaster  = _fasterIsRight ? scaleR : scaleL;
    float scaleSlower  = _fasterIsRight ? scaleL : scaleR;
    float baseFaster = _cal.kFF * tgtFasterAbs * scaleFaster;
    float baseSlower = _cal.kFF * tgtSlowerAbs * scaleSlower;

    // Step 6: Slower-wheel adjustment
    float excess = _pid.integral - _cal.kAdjThreshold;
    float adj = (excess > 0.0f) ? (-_cal.kAdjGain * excess * baseFaster) : 0.0f;

    // Step 7: Compute and clamp final PWM
    float uFaster = clamp(baseFaster + correction, 0.0f, 100.0f);
    float uSlower = clamp(baseSlower + adj,        0.0f, 100.0f);

    // Apply direction signs
    float uL, uR;
    if (_fasterIsRight) {
        uL = (_tgtLMms >= 0.0f) ?  uSlower : -uSlower;
        uR = (_tgtRMms >= 0.0f) ?  uFaster : -uFaster;
    } else {
        uL = (_tgtLMms >= 0.0f) ?  uFaster : -uFaster;
        uR = (_tgtRMms >= 0.0f) ?  uSlower : -uSlower;
    }

    // SET PWM: use the same NezhaV2 call as sprint-2 tick body
    // e.g. _motor.motorRun(MotorNum::M1, direction, speed) x2, or a setPwm wrapper
    // Cast uL, uR to int via roundf()
}
```

---

## Acceptance Criteria

- [x] `source/control/MotorController.h` declares `startDriveClean()` and `startDrive()` with the signatures above
- [x] `source/control/MotorController.h` includes `RatioPidController.h` and has `RatioPidController _pid` in private state
- [x] `source/control/MotorController.cpp` initializes `_pid` in the constructor initializer list with `cal.ratioPidKp/Ki/Kd/Max`
- [x] `tick()` body implements all 7 steps of the ratio PID algorithm
- [x] `stop()` calls `_pid.reset()` and writes zero PWM
- [x] `startDriveClean()` calls `_pid.reset()` and snapshots encoder positions
- [x] `startDrive()` re-seeds cmdEncStart without resetting PID (unless faster/slower assignment changes)
- [x] `setTarget()` only stores `_tgtLMms` / `_tgtRMms` — does not call startDrive or startDriveClean
- [x] Old `_integralL`, `_integralR` PI fields are removed from `MotorController.h`
- [x] Public signatures of `setTarget()`, `stop()`, `resetIntegrators()`, `tick()`, `getActualVelocity()`, `getEncoderPositions()`, `resetEncoderAccumulators()` are unchanged
- [ ] Project builds without errors: `python build.py`

## Testing

Build verification only at this stage — hardware tests in ticket 004.

- **Build verification**: `python build.py` must complete without errors
- **Inspection**: confirm tick() body matches the 7-step algorithm above
- **No hardware tests yet** — integration testing in ticket 004
