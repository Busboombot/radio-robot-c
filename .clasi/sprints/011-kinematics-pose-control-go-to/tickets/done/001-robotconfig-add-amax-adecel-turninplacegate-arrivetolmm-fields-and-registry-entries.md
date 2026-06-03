---
id: '001'
title: 'RobotConfig: add aMax, aDecel, turnInPlaceGate, arriveTolMm fields and registry
  entries'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
depends-on: []
github-issue: ''
issue: kinematics-pose-control-goto.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 011-001: RobotConfig — add aMax, aDecel, turnInPlaceGate, arriveTolMm fields and registry entries

## Description

Foundation ticket. All pursuit-arc and VW tickets depend on these config fields
and their SET/GET registry entries. Must land first.

Add four fields to `RobotConfig` in `source/types/Config.h` and register them
in the `kRegistry[]` table in `source/app/CommandProcessor.cpp`.

**Unit convention for `turnInPlaceGate`**: store in **degrees** in `RobotConfig`
(float, default 45.0°). `DriveController` converts to radians at use-site with
`turnInPlaceGate * (π/180)`. This sidesteps the CFG_FI unit-conversion gap
identified in the architecture review (CFG_FI uses `atoi` with no conversion;
storing degrees means the integer wire value equals the float field directly).

### Fields to add to `RobotConfig`

```cpp
float aMax;             // acceleration limit, mm/s²   (default 300.0)
float aDecel;           // deceleration limit for v_cap, mm/s²  (default 250.0)
float turnInPlaceGate;  // bearing threshold for in-place rotate, degrees (default 45.0)
float arriveTolMm;      // go-to arrival tolerance, mm  (default 5.0)
```

Add default initializers to `defaultRobotConfig()`:
```cpp
p.aMax            = 300.0f;
p.aDecel          = 250.0f;
p.turnInPlaceGate = 45.0f;
p.arriveTolMm     = 5.0f;
```

### Registry entries to add in `kRegistry[]`

```cpp
CFG_F ("aMax",      aMax),
CFG_F ("aDecel",    aDecel),
CFG_FI("turnGate",  turnInPlaceGate),   // wire: integer degrees
CFG_FI("arriveTol", arriveTolMm),       // wire: integer mm
```

Add these after the existing `doneTol` entry so the GET dump order is intuitive.

### HELP verb update

Add `VW` to the verb list in the `HELP` handler (the `VW` command itself is
added in ticket 005, but the HELP string is a single constant — update it here
to avoid a two-touch on the same line).

## Acceptance Criteria

- [x] `Config.h` compiles with four new fields; `defaultRobotConfig()` sets them.
- [x] `SET aMax=400` responds `OK set aMax=400`; `GET aMax` responds `CFG aMax=400.000`.
- [x] `SET aDecel=300` / `GET aDecel` round-trips correctly.
- [x] `SET turnGate=60` responds `OK set turnGate=60`; `GET turnGate` responds `CFG turnGate=60` (integer, degrees).
- [x] `SET arriveTol=10` / `GET arriveTol` round-trips as integer mm.
- [x] `GET` (full dump) includes all four new keys without breaking the 512-byte
  buffer limit (32 total keys, GET dump is 409 bytes — well under 512 bytes).
- [x] `SET badkey=1` still returns `ERR badkey badkey` (no regression in error path).
- [x] All existing tests pass (no struct layout break, no registry collision).

## Implementation Plan

### Approach

Pure data/table change — no control logic. Two files touched.

### Files to modify

- `source/types/Config.h` — add four fields to struct + defaults
- `source/app/CommandProcessor.cpp` — add four entries to `kRegistry[]`, update HELP string

### Testing plan

- Run full test suite to confirm no regression.
- Manual: `SET aMax=500 aDecel=400 turnGate=60 arriveTol=8` then `GET` to verify
  all four appear in CFG dump.
- Manual: `GET` (no args) to verify full dump does not truncate.

### Documentation updates

None required (protocol-v2.md Named Key Table will be updated in ticket 005
alongside the VW verb documentation).
