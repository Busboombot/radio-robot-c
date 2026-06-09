---
id: '002'
title: "Extract ConfigRegistry \u2014 kRegistry[], handleGet, handleSet to source/robot/ConfigRegistry.h/.cpp"
status: done
use-cases:
- SUC-005
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---

# Extract ConfigRegistry — kRegistry[], handleGet, handleSet to source/robot/ConfigRegistry.h/.cpp

## Description

Move the config key-offset table (`kRegistry[]`) and its `handleGet`/`handleSet`
functions from `CommandProcessor.cpp` into a dedicated `source/robot/ConfigRegistry.h/.cpp`.
This severs the config registry from the monolithic command processor translation unit
and prepares for `Robot::buildCommandTable()` to register GET/SET commands independently.

`CommandProcessor.cpp` continues to `#include "ConfigRegistry.h"` and call
`handleGet`/`handleSet` via the old switch — behavior is completely unchanged. This is
a pure code-movement ticket.

## Acceptance Criteria

- [x] `source/robot/ConfigRegistry.h` declares:
  - `CfgCtx` struct: `{ RobotConfig* cfg; MotorController* mc; }`
  - `ConfigEntry` struct with `key`, `type` (`ConfigFieldType` enum), and `offset` fields
  - `extern const ConfigEntry kRegistry[]` and `extern const int kRegistryCount`
  - `handleGet` — `HandlerFn`-compatible signature
  - `handleSet` — `HandlerFn`-compatible signature
- [x] `source/robot/ConfigRegistry.cpp` contains `kRegistry[]` (all ~40 entries), `kRegistryCount`, `handleGet`, and `handleSet` implementations
- [x] `CommandProcessor.cpp` no longer defines `kRegistry[]`, `handleGet`, or `handleSet`; it `#include`s `ConfigRegistry.h` instead
- [x] `python3 build.py` passes with no errors
- [ ] GET and SET round-trip correctly over the wire (`uv run rogo`: `GET vel.kP`, `SET vel.kP 3.0`)

## Implementation Plan

### Approach

1. Create `source/robot/ConfigRegistry.h` with the declarations above.
2. Create `source/robot/ConfigRegistry.cpp` by moving `kRegistry[]`, `kRegistryCount`,
   `ConfigFieldType` enum, `ConfigEntry` struct, and `handleGet`/`handleSet` bodies
   verbatim from `CommandProcessor.cpp`.
3. In `CommandProcessor.cpp`, remove the moved code and add `#include "ConfigRegistry.h"`.
4. The old switch still calls `handleGet`/`handleSet` — no logic change needed.

The `handleGet`/`handleSet` function signatures must match `HandlerFn` (from T001) even
though they are called from the old switch in this ticket. This ensures they slot
directly into `CommandDescriptor` entries in T010 without signature changes.

### Files to Create

- `source/robot/ConfigRegistry.h`
- `source/robot/ConfigRegistry.cpp`

### Files to Modify

- `source/app/CommandProcessor.cpp` — remove moved code, add include

### Testing Plan

- Build: `python3 build.py` must pass.
- Smoke test: Connect robot, send `GET vel.kP` and `SET vel.kP 3.0` via `uv run rogo`;
  verify correct `OK GET` / `OK SET` responses.
- Existing tests: `uv run --with pytest python -m pytest` (build passing is the primary gate).
