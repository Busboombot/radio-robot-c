---
id: '002'
title: MockHAL ExactPoseTracker and turn-rate coordination
status: done
use-cases:
- SUC-001
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---

# MockHAL ExactPoseTracker and turn-rate coordination

## Description

`MockHAL::tick()` currently advances each motor by a fixed dt. This ticket extends
`tick()` to: (a) compute turn rate from current motor commands and feed it to each
motor before ticking, and (b) accumulate an oracle ground-truth pose via a new inline
`ExactPoseTracker` struct.

`ExactPoseTracker` uses midpoint integration identical to `Odometry::predict` but reads
`trueVelocityMms()` (pre-slip) from each motor, so it remains unaffected by the slip
and noise model added in ticket 001.

`_trackwidthMm` is needed for the kinematic integration. It must be set from `SimHandle`
after constructing the `RobotConfig` (in the `SimHandle` constructor body, not the
initialiser list).

## Acceptance Criteria

- [x] `MockHAL.h` defines an inline `ExactPoseTracker` struct with fields
  `{float x=0, y=0, h=0}`, method `reset()`, and method
  `update(float velLMms, float velRMms, float trackwidthMm, uint32_t dt_ms)`.
- [x] `ExactPoseTracker::update()` uses midpoint integration:
  `dC = (vL + vR)/2 * dt_s`, `dTh = (vR - vL) / tw * dt_s`,
  `hMid = h + dTh/2`, `x += dC*cosf(hMid)`, `y += dC*sinf(hMid)`, `h += dTh`.
- [x] `MockHAL.h` adds fields `_exactPose` (ExactPoseTracker) and
  `_trackwidthMm = 0.0f`.
- [x] `MockHAL.h` adds accessors `exactPoseMock()` returning `ExactPoseTracker&`, and
  `setTrackwidth(float mm)`.
- [x] `MockHAL::tick(now_ms)` is extended to (in order):
  1. Compute `aL = fabsf(_motorL.cmdSpeed())`, `aR = fabsf(_motorR.cmdSpeed())`;
     `turnRate = (aL+aR > 0.5f) ? fabsf(_motorR.cmdSpeed() - _motorL.cmdSpeed()) / (aL+aR) : 0.0f`.
  2. Call `_motorL.setTurnRate(turnRate)` and `_motorR.setTurnRate(turnRate)`.
  3. Call `_motorL.tick(udt)` and `_motorR.tick(udt)`.
  4. Call `_exactPose.update(_motorL.trueVelocityMms(), _motorR.trueVelocityMms(), _trackwidthMm, udt)`.
  5. Call `_otos.tick(...)` (ticket 003 adds this; leave a TODO comment here for now).
- [x] `libfirmware_host` builds cleanly.
- [x] `uv run --with pytest python -m pytest` passes with no regressions.
- [x] Manual check: with straight drive (equal L/R speed), `exactPoseMock().x` grows;
  with slip enabled (ticket 001), `exactPoseMock().x` grows faster than encoder-based
  pose because `trueVelocityMms()` is pre-slip.

## Implementation Plan

### Approach

Add `ExactPoseTracker` as an inline struct at the top of `MockHAL.h`, above the
`MockHAL` class declaration. It has no CODAL dependency and no `<random>` dependency.
Extend `MockHAL::tick()` in `MockHAL.cpp`. Turn rate uses `cmdSpeed()` (already
accessible via the existing MockMotor accessor). The formula uses `fabsf` from
`<cmath>`; confirm `<cmath>` is already included in `MockHAL.cpp` before adding.

### Files to modify

- `source/hal/mock/MockHAL.h` — add `ExactPoseTracker` struct; add `_exactPose`,
  `_trackwidthMm`; add `exactPoseMock()`, `setTrackwidth()`
- `source/hal/mock/MockHAL.cpp` — extend `tick()` with turn-rate computation and
  exact-pose update

### Testing plan

- Existing pytest suite must pass unchanged.
- Manual: construct `MockHAL`, call `setTrackwidth(126.0f)`. Set both motors to
  speed 50 (half of 100%), tick 1000 ms. Expect `exactPoseMock().x` ≈ 200 mm
  (`kNominalMaxMms * 0.5 * 1.0s = 400 * 0.5 = 200`). `exactPoseMock().y` ≈ 0.
- Manual: set L=−50, R=50 (point turn). `turnRate` should equal 1.0. `exactPoseMock()`
  should show rotation around origin (x≈0, h nonzero).

### Documentation updates

No additional docs needed; architecture update already describes `ExactPoseTracker`.
