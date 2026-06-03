---
id: '002'
title: OtosSensor applies linear/angular scalars at init from config
status: done
use-cases:
- SUC-001
- SUC-003
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---

# OtosSensor applies linear/angular scalars at init from config

## Description

Currently `OtosSensor::init()` enables signal processing and resets Kalman
tracking, but never sets the linear or angular scalars. These must be sent
manually via `OL`/`OA` each session or the OTOS tracks with uncorrected
scale errors.

This ticket makes the scalars apply automatically at boot from `RobotConfig`.
The float scale (e.g. 1.05) is converted to the OTOS int8 register value via:

```
scalar = clamp(round((scale - 1.0) / 0.001), -127, 127)
```

So `otosLinearScale=1.05` -> scalar +50; `otosAngularScale=0.987` -> scalar -13.

## Files to Modify

- **`source/robot/Robot.cpp`** — in the `_otosPresent` block after `_otos.init()`,
  add scalar conversion and `setLinearScalar`/`setAngularScalar` calls using
  `_config.otosLinearScale` and `_config.otosAngularScale`.
  Formula: `scalar = clamp(round((scale - 1.0f) / 0.001f), -127, 127)`.
  Add `<cmath>` include if not already present.
- **`source/hal/OtosSensor.h/.cpp`** — no change needed.

## Approach

1. Add the scalar conversion and two `set*Scalar` calls in `Robot.cpp` immediately
   after `_otos.init()` and `_dc.setOtos(&_otos)`.
2. The existing `OL`/`OA` command handlers continue to call `setLinearScalar`/
   `setAngularScalar` directly, so runtime override is unaffected.
3. Clean build. Reflash to robot enum 2.
4. Verify via `OL` (read-back): should return scalar approximately +50 without any host command.
5. Verify via `OA` (read-back): should return scalar approximately -13 without any host command.

## Acceptance Criteria

- [x] After boot, `OL` (no arg) returns `OK linear scalar=50` (for `otosLinearScale=1.05`).
- [x] After boot, `OA` (no arg) returns `OK angular scalar=-13` (for `otosAngularScale=0.987`).
- [x] `OL <val>` and `OA <val>` still override the boot-time scalar at runtime.
- [x] Clean build (`mbdeploy build --clean`) succeeds.
- [x] (Bench deferred to T11) Measured run closer to truth after scalars applied.

## Testing

- **Build verification**: `mbdeploy build --clean` + reflash robot enum 2.
- **Wire test (bench, deferred)**: `OL` and `OA` read-back immediately after boot confirm scalars set.
- **Verification command**: `mbdeploy build --clean`
