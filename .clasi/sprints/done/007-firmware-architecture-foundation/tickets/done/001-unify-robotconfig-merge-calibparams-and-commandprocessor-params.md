---
id: '001'
title: "Unify RobotConfig \u2014 merge CalibParams and CommandProcessor::Params"
status: done
use-cases:
- SUC-001
- SUC-006
depends-on: []
github-issue: ''
issue: firmware-architecture-refactor.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 001 — Unify RobotConfig — merge CalibParams and CommandProcessor::Params

## Description

`CalibParams` (in `source/types/Config.h`) and `CommandProcessor::Params` both hold
`mmPerDegL/R` and `trackwidthMm`. A K-command update writes to `CommandProcessor::params`
but `MotorController` reads from the `CalibParams` reference injected at construction —
the two copies can diverge silently, producing wrong encoder-distance math or arc
geometry downstream.

This ticket creates a single `RobotConfig` struct that merges both, deletes both
originals, and rewires all consumers to the unified ref. No behavior changes; only the
data path is unified.

## Acceptance Criteria

- [x] `RobotConfig` struct exists in `source/types/Config.h` with all fields from
  `CalibParams` + `CommandProcessor::Params`; no duplicate `mmPerDegL/R` or
  `trackwidthMm`.
- [x] `defaultRobotConfig()` factory function exists and returns correct defaults
  (matching current `defaultCalibParams()` and `CommandProcessor` param defaults).
- [x] `CalibParams` struct and `defaultCalibParams()` are deleted from `Config.h`.
- [x] `CommandProcessor::Params` struct is deleted from `CommandProcessor.h`.
- [x] `MotorController` ctor takes `const RobotConfig&`; compiles without error.
- [x] Firmware builds via `mbdeploy deploy --build` (no compile errors).
- [ ] **Bench gate**: Deploy to robot. Run:
  - `HELLO` → `DEVICE:Nezha2:…`
  - `EZ` / `ENC` → `ACK:EZ` then `ENC+0+0`
  - `S+150+150` → wheels spin, streamed `ENC…` values climb
  - `X` → `ACK:X`; encoders hold
  - `KmmPerDegL+0.500` → `ACK:KmmPerDegL`; subsequent `S+200+200` drive confirms
    encoder distances reflect the updated value (no stale copy in MotorController).

## Implementation Plan

### Approach

Atomic swap: introduce `RobotConfig`, update all references, delete the old structs in
a single coherent change. The firmware will not compile in a half-migrated state, so all
consumers are updated together.

### Files to Modify

| File | Change |
|---|---|
| `source/types/Config.h` | Add `RobotConfig` with all fields; add `defaultRobotConfig()`; delete `CalibParams`, `defaultCalibParams()` |
| `source/robot/Robot.h` | `CalibParams _cal` → `RobotConfig _config` |
| `source/robot/Robot.cpp` | Init list: `_config(defaultRobotConfig())`, `_mc(_motor, _config)`; update `_cmd.setCalib` → `_cmd.setConfig(_config)` |
| `source/app/CommandProcessor.h` | Delete `struct Params params`; delete `CalibParams* _cal`; add `RobotConfig* _config` pointer (full thinning in Ticket 005); update `setCalib` → `setConfig` |
| `source/app/CommandProcessor.cpp` | Update K-command handlers to write to `_config` fields; ctor defaults; `init()` |
| `source/control/MotorController.h` | `const CalibParams& _cal` → `const RobotConfig& _cal`; ctor signature |
| `source/control/MotorController.cpp` | Ctor init list update; field accesses unchanged by name |

### Field Mapping

| Old source | Field(s) | Destination in RobotConfig |
|---|---|---|
| `CalibParams` | `mmPerDegL/R`, `trackwidthMm`, `kFF`, `kScale*`, `kAdj*`, `ratioPid*`, `turnThresholdMm`, `doneTolMm` | direct 1:1 |
| `CommandProcessor::Params` | `mmPerDegL/R`, `trackwidthMm` | removed (duplicates; use CalibParams copy) |
| `CommandProcessor::Params` | `distScale`, `turnScale`, `minSpeedMms`, `tickMs`, `sTimeoutMs`, `encReportEvery` | added to `RobotConfig` |

### Testing Plan

- Build and flash: `mbdeploy deploy --build`.
- Serial smoke sequence as in Acceptance Criteria above.
- Confirm K-command write propagates to encoder path: set `KmmPerDegL+0.500`, drive
  `S+200+200`, check `ENC` values accumulate at the new rate.

### Documentation Updates

None needed this ticket. Full architecture doc update at sprint close.
