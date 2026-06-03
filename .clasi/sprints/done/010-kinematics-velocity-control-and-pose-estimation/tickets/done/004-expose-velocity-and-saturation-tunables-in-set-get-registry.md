---
id: '004'
title: Expose velocity and saturation tunables in SET/GET registry
status: done
use-cases:
- SUC-004
depends-on:
- 010-003
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Expose velocity and saturation tunables in SET/GET registry

## Description

Tickets 001–003 add new `RobotConfig` fields (`velKp`, `velKi`, `velKff`,
`minWheelMms`, `vWheelMax`, `steerHeadroom`). The Sprint 009 SET/GET registry
in `CommandProcessor.cpp` must be extended so developers can tune these live
from the host without reflashing. Also, the deleted `lapsToMmScale` field must
be removed from the registry to avoid an ERR on any existing scripts.

## Acceptance Criteria

- [x] `kRegistry[]` in `CommandProcessor.cpp` has entries for:
  - `vel.kP` → `velKp` (CFG_FLOAT)
  - `vel.kI` → `velKi` (CFG_FLOAT)
  - `vel.kFF` → `velKff` (CFG_FLOAT)
  - `minWheelMms` → `minWheelMms` (CFG_FLOAT)
  - `vWheelMax` → `vWheelMax` (CFG_FLOAT)
  - `steerHeadroom` → `steerHeadroom` (CFG_FLOAT)
- [x] `lapsToMmScale` entry is absent from `kRegistry[]` (already deleted by
  Ticket 001; verify here that no stale reference remains).
- [x] `SET vel.kP=0.4` round-trips: `GET vel.kP` returns `CFG vel.kP=0.400`.
- [x] `SET vWheelMax=350` round-trips: `GET vWheelMax` returns `CFG vWheelMax=350`.
- [x] All six new keys are present in the output of `GET ALL` (or equivalent
  full-config dump command if it exists).

## Implementation Plan

**Approach**: Pure `CommandProcessor.cpp` edit — add 6 `CFG_F` macro rows to
`kRegistry[]`; confirm `lapsToMmScale` is absent.

**Files to modify**:
- `source/app/CommandProcessor.cpp` — add 6 entries to `kRegistry[]`.

**Testing plan**:
- Bench: issue SET/GET round-trips for each new key via serial terminal.
- Verify that setting `vel.kP` to a new value and then commanding motion uses
  the updated gain (observe motor behavior change).

**Documentation updates**:
- None required; the registry is self-documenting via GET ALL.
