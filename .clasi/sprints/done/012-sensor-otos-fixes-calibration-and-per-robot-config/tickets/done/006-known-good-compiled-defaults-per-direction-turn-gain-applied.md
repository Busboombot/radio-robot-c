---
id: '006'
title: Known-good compiled defaults + per-direction turn gain applied
status: done
use-cases:
- SUC-001
- SUC-003
- SUC-005
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---

# Known-good compiled defaults + per-direction turn gain applied

## Description

Two parts:

**Part A — Defaults**: `defaultRobotConfig()` has `trackwidthMm=120`, which is
wrong for the nezha robot (correct: 126). OTOS scalars and per-direction turn
gains are not present yet (added in T01). This ticket bakes the known-good
values into the defaults.

**Part B — Per-direction turn gain applied**: The `rotationGainPos`/`rotationGainNeg`
and `rotationOffsetDeg`/`rotationOffsetDegNeg` fields added in T01 are not yet
used by any firmware logic. This ticket applies them in the turn path.

**Do NOT change**: `mmPerDegL` (0.487) or `mmPerDegR` (0.481) — they are correct
and match the prior known-good system.

## Files to Modify

**Part A — Defaults in `source/types/Config.h` `defaultRobotConfig()`**:
- `trackwidthMm`: 120.0f -> **126.0f**
- `otosLinearScale`: **1.05f** (new field from T01)
- `otosAngularScale`: **0.987f** (new field from T01)
- `rotationGainPos`: **1.0f** (new field from T01)
- `rotationGainNeg`: **1.17f** (new field from T01)
- `rotationOffsetDeg`: **0.0f** (new field from T01)
- `rotationOffsetDegNeg`: **0.0f** (new field from T01)
- `rotationalSlip`: **0.74f** (new field from T01)

**Part B — Per-direction gain applied in `source/control/DriveController.cpp`**:
- Identify the in-place rotate path and the go-to pre-rotate path.
- When commanding a turn, select gain and offset based on direction:
  - Positive rotation (CCW): use `rotationGainPos`, `rotationOffsetDeg`
  - Negative rotation (CW): use `rotationGainNeg`, `rotationOffsetDegNeg`
- The effective commanded angle is: `(target_deg - offset) / gain`
- This compensates for startup loss (offset) and proportional under/overshoot (gain).
- Verify the formula composes correctly with the OTOS correction loop in go-to.

**Tests to update**:
- Any test asserting `defaultRobotConfig().trackwidthMm == 120` -> update to 126.
- Candidate files: `tests/test_odometry_midpoint.py`, `tests/test_otos_fusion.py`,
  any file that constructs a default config and checks trackwidth.

## Approach

1. Update `defaultRobotConfig()` in Config.h with all known-good values.
2. Search codebase for hard-coded `120` in tests related to trackwidth; update to `126`.
3. Find the rotate/turn path in DriveController. Apply per-direction gain/offset
   compensation formula. Confirm the formula does not interfere with the OTOS
   feedback loop (OTOS correction already runs at idle and during turns).
4. Clean build. Reflash robot enum 2.
5. Verify `GET tw` = 126 at boot.
6. Run test suite to catch any regressions from trackwidth change.

## Acceptance Criteria

- [x] `GET tw` returns 126 at boot without any SET command.
- [x] `GET otosLinSc` returns 1.050, `GET otosAngSc` returns 0.987 at boot.
- [x] `GET rotGainNeg` returns 1.170 at boot.
- [x] Tests previously hard-coding trackwidth 120 are updated and passing.
- [x] Per-direction gain/offset is applied in the rotate path (CCW uses `rotationGainPos`; CW uses `rotationGainNeg`).
- [x] Per-direction gain composes correctly with go-to pre-rotate (no oscillation on bench).
- [x] `uv run pytest` passes with no regressions.
- [x] Clean build (`mbdeploy build --clean`) succeeds.
- [ ] (Bench deferred to T11) 90° CCW and CW turns are symmetric within tolerance.

## Testing

- **Existing tests to update**: `tests/test_odometry_midpoint.py`, `tests/test_otos_fusion.py`
  (search for hard-coded `120`).
- **Verification command**: `mbdeploy build --clean && uv run pytest`
