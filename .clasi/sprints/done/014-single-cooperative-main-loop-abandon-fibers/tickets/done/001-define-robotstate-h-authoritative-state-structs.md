---
id: '001'
title: Define RobotState.h authoritative state structs
status: done
use-cases:
- SUC-003
- SUC-006
depends-on: []
github-issue: ''
issue: plan-single-cooperative-main-loop-abandon-fibers.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Define RobotState.h authoritative state structs

## Description

Create `source/control/RobotState.h` with the three authoritative state
structs that will replace the per-subsystem private caches scattered across
`MotorController`, `Odometry`, and `DriveController`. Also add the four lag
fields to `RobotConfig` in `source/types/Config.h`.

This is the foundation ticket — all other tickets in this sprint depend on
these type definitions being in place before they can refactor subsystems.

## Files to Create

- `source/control/RobotState.h` — new file

## Files to Modify

- `source/types/Config.h` — add `lagOtosMs`, `lagLineMs`, `lagColorMs`,
  `lagPortsMs` fields to `RobotConfig`; add defaults in `defaultRobotConfig()`.

## Acceptance Criteria

- [x] `source/control/RobotState.h` exists and defines:
  - `ValueSet { uint32_t lagMs; uint32_t lastUpdMs; bool valid; }`
  - `MotorCommands` with `tgtLMms`, `tgtRMms`, `pwmL`, `pwmR`,
    `digitalOut[4]`, `analogOut[4]`, `digitalDirty[4]`, `analogDirty[4]`
  - `HardwareState` with `encLMm`, `encRMm`, `enc` (ValueSet),
    `velLMms`, `velRMms`, `poseX`, `poseY`, `poseHrad`, `pose` (ValueSet),
    `otosX`, `otosY`, `otosH`, `otos` (ValueSet),
    `line[4]`, `lineVS` (ValueSet),
    `colorR/G/B/C`, `colorVS` (ValueSet),
    `digitalIn[4]`, `analogIn[4]`, `portsVS` (ValueSet)
  - `TargetState` with `mode`, `targetXWorld`, `targetYWorld`,
    `targetSpeedMms`, `distanceTargetMm`, `deadlineMs`,
    `replyFn`, `replyCtx`, `corrId[16]`
  - `RobotStateContainer { MotorCommands commands; HardwareState inputs; TargetState target; }`
  - `defaultInputs()` helper that seeds `ValueSet.lagMs` defaults from
    `RobotConfig` lag fields (otos=100, line=50, color=100, ports=50)
- [x] `RobotConfig` in `Config.h` gains `uint32_t lagOtosMs`, `lagLineMs`,
  `lagColorMs`, `lagPortsMs`; `defaultRobotConfig()` sets them to
  100, 50, 100, 50 respectively.
- [x] `RobotState.h` has no CODAL or MicroBit includes — pure C++ data types only.
- [x] Firmware still builds cleanly (no new errors; the new header is not yet
  included by any subsystem at this stage — that happens in tickets 003–005).

## Implementation Plan

1. Create `source/control/RobotState.h`:
   - Include only `<stdint.h>` and `"Config.h"` (for `ReplyFn` and `DriveMode`).
   - Define `ValueSet` first; then `MotorCommands`, `HardwareState`,
     `TargetState`, `RobotStateContainer` in the order listed above.
   - Define `inline RobotStateContainer defaultInputs(const RobotConfig& cfg)`
     that zero-initializes the container then seeds each `ValueSet.lagMs` from
     the corresponding `cfg.lag*Ms` field.
2. Edit `source/types/Config.h`:
   - Add the four `uint32_t lag*Ms` fields at the end of `RobotConfig`.
   - Add their defaults in `defaultRobotConfig()`.

## Testing Plan

- **Build verification** (CI): `python build.py` must succeed with no new errors.
- **No unit tests for this ticket** — it is a pure type definition; behavior is
  verified when subsystems use the structs in tickets 003–006.
- Verification command: `python build.py` (Docker CODAL build).
