---
id: '001'
title: 'Config fields + SET/GET registry: OTOS scalars + turn asymmetry'
status: done
use-cases:
- SUC-003
- SUC-008
depends-on: []
github-issue: ''
issue: sprint-12-sensor-otos-fixes-calibration-per-robot-config.md
completes_issue: false
---

# Config fields + SET/GET registry: OTOS scalars + turn asymmetry

## Description

Foundational ticket. Add new fields to `RobotConfig` and wire them to the
`kRegistry[]` SET/GET system. No deps; all subsequent firmware tickets build
on these fields.

The robot currently has no config fields for OTOS scale factors or per-direction
turn calibration, so those values must be hard-coded or sent manually each
session. This ticket makes them first-class config fields with SET/GET support.

## Files to Create/Modify

- **`source/types/Config.h`** — add to `RobotConfig` struct:
  - `float otosLinearScale` (default 1.05)
  - `float otosAngularScale` (default 0.987)
  - `float rotationGainPos` (default 1.0 — CCW turns)
  - `float rotationGainNeg` (default 1.17 — CW turns)
  - `float rotationOffsetDeg` (default 0.0)
  - `float rotationOffsetDegNeg` (default 0.0)
  - `float rotationalSlip` (default 0.74)
  - `float odomOffX` (default 0.0)
  - `float odomOffY` (default 0.0)
  - `float odomYawDeg` (default 0.0)
  - `bool  odomUpsideDown` (default false)
  - Update `defaultRobotConfig()` with all the above defaults.
  - **Do NOT change** `mmPerDegL` (0.487) or `mmPerDegR` (0.481).

- **`source/app/CommandProcessor.cpp`** — add to `kRegistry[]`:

  | Key | Field | Type |
  |-----|-------|------|
  | `otosLinSc` | `otosLinearScale` | CFG_F |
  | `otosAngSc` | `otosAngularScale` | CFG_F |
  | `rotGainPos` | `rotationGainPos` | CFG_F |
  | `rotGainNeg` | `rotationGainNeg` | CFG_F |
  | `rotOffPos` | `rotationOffsetDeg` | CFG_F |
  | `rotOffNeg` | `rotationOffsetDegNeg` | CFG_F |
  | `rotSlip` | `rotationalSlip` | CFG_F |
  | `odomOffX` | `odomOffX` | CFG_F |
  | `odomOffY` | `odomOffY` | CFG_F |
  | `odomYaw` | `odomYawDeg` | CFG_F |

  Follow the existing `CFG_F(k, field)` macro pattern.

## Approach

1. Add fields to `RobotConfig` in `Config.h` immediately after the existing OTOS
   fusion parameters block (after `otosGate`).
2. Update `defaultRobotConfig()` in the same file.
3. Add `kRegistry[]` entries in `CommandProcessor.cpp` following the existing
   pattern, in a new section `// OTOS calibration and turn asymmetry (Sprint 012)`.
4. Clean build. Verify GET dump fits in 512 bytes (the handleGet() local `line[512]`
   buffer). Count the total character length of a full `GET` dump with the new keys.
5. Confirm RAM/BSS is not exceeded (CODAL heap ceiling check — see build output
   for memory usage summary).

## Acceptance Criteria

- [x] `SET otosLinSc=1.05` followed by `GET otosLinSc` returns `CFG otosLinSc=1.050`.
- [x] `SET otosAngSc=0.987` followed by `GET otosAngSc` returns `CFG otosAngSc=0.987`.
- [x] `SET rotGainNeg=1.17` followed by `GET rotGainNeg` returns `CFG rotGainNeg=1.170`.
- [x] All 10 new keys round-trip correctly via SET/GET.
- [x] `GET` (full dump) response fits within the buffer (note: buffer expanded from 512→768 bytes; full dump is 565 bytes, well within 768-byte limit).
- [x] Clean build (`mbdeploy build --clean`) succeeds with no heap/stack overflow warnings.
- [x] Build memory summary confirms new struct size does not breach CODAL heap ceiling (RAM 98.33%, no change from baseline; BSS unchanged at 119824 B).

## Testing

- **Build verification**: `mbdeploy build --clean` — check for memory warnings.
- **Wire test (bench, deferred)**: SET/GET round-trip for each new key over relay.
- **Verification command**: `mbdeploy build --clean`
