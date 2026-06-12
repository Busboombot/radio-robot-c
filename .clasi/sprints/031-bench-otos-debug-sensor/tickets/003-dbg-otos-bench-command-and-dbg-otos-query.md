---
id: "003"
title: "DBG OTOS BENCH command and DBG OTOS query"
status: open
use-cases: [SUC-001, SUC-003]
depends-on: ["031-002"]
github-issue: ""
issue: ""
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# DBG OTOS BENCH command and DBG OTOS query

## Description

Add two new debug commands to `DebugCommandable`, modeled on the existing
`DBG WEDGE` handler pattern in `source/app/DebugCommandable.cpp`:

**`DBG OTOS BENCH 0|1 [noiseXY=<f>] [noiseH=<f>] [drift=<f>]`**
- First positional arg: 0 = disable bench mode, 1 = enable.
- Optional KV args `noiseXY`, `noiseH`, `drift`: if present, call
  `benchOtosPtr()->setNoise(noiseXY, noiseH, drift)` after toggling.
- Calls `robot->isBenchOtosActive()` to read current mode (via NezhaHAL).
  In HOST_BUILD / MockHAL, `isBenchOtosActive()` returns false; the command
  still parses and replies OK (no crash, just a no-op toggle).
- Reply: `OK dbg otos bench=<0|1>`.
- This command needs `CMD_ACCESS_HARDWARE` flag (same as `DBG WEDGE`) because
  it modifies hardware routing.

**`DBG OTOS`**
- No args. Query the current three-way pose.
- Build reply line: `ideal=<x>,<y>,<h> otos=<x>,<y>,<h> fused=<x>,<y>,<h>`
  - `ideal`: from `benchOtosPtr()->idealPose()` (noiseless accumulator). If
    bench mode is off, this will be `0,0,0` (that is acceptable per OQ-2 in
    architecture doc).
  - `otos`: from `benchOtosPtr()->readTransformed()` return value in poseOut.
    If bench mode is off, this is also `0,0,0`.
  - `fused`: from `robot->state.inputs.otosX`, `otosY`, `otosH` — the
    EKF-fused values.
- Reply: the pose line, then `OK dbg otos`.
- In HOST_BUILD / MockHAL, `benchOtosPtr()` returns nullptr; guard the call
  and emit `0,0,0` for ideal/otos in that case.

**Parser pattern**: Follow `parseDbgWedge` — accept positional int/float args
and KV pairs. For `DBG OTOS BENCH`, the parse fn should handle: arg[0] = INT
(0 or 1), then optional KV pairs for noiseXY/noiseH/drift as FLOAT.

**Prefix ordering in `getCommands()`**: `"DBG OTOS BENCH"` must appear before
`"DBG OTOS"` in the returned vector so the longest-prefix dispatch wins.

**Read first**: `source/app/DebugCommandable.cpp` (full file — understand the
parse/handle pattern, the `dbgCtxFrom()` helper, and the `getCommands()` table
ordering). `source/app/DebugCommandable.h` for `DbgCtx` fields available.
`source/robot/Robot.h` for `state.inputs` field names (otosX/Y/H or similar —
check actual field names before writing).

## Acceptance Criteria

- [ ] `DBG OTOS BENCH 1` reply is `OK dbg otos bench=1`.
- [ ] `DBG OTOS BENCH 0` reply is `OK dbg otos bench=0`.
- [ ] `DBG OTOS BENCH 1 noiseXY=0.5 noiseH=0.01 drift=0.001` is accepted;
  noise params updated (verify via subsequent `DBG OTOS` showing non-zero
  errored vs ideal after a drive tick).
- [ ] `DBG OTOS` reply contains `ideal=`, `otos=`, and `fused=` fields.
- [ ] `"DBG OTOS BENCH"` appears before `"DBG OTOS"` in the command table.
- [ ] `python3 build.py` clean build passes.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.

## Implementation Plan

### Approach

Extend `DebugCommandable.cpp` with two new static parse/handle function pairs
and register them in `getCommands()`. No new files; no changes outside
`DebugCommandable.{h,cpp}` (plus any forward-declare needed in the header).

### Files to Modify

- `source/app/DebugCommandable.h` — add forward declares if needed (e.g.,
  `class BenchOtosSensor;` if accessed via pointer in the handler)
- `source/app/DebugCommandable.cpp` — add `parseDbgOtosBench`,
  `handleDbgOtosBench`, `parseDbgOtos`, `handleDbgOtos`; register both in
  `getCommands()` with `DBG OTOS BENCH` first

### Testing Plan

Build + host-test suite. The `DBG OTOS BENCH` toggle with MockHAL returns
false/no-op; the build still validates the parser and reply path. Functional
verification is the post-sprint hardware bench session.

### Post-Sprint Validation Note

Hardware flash + `DBG OTOS BENCH 1` + drive + `DBG OTOS` query is the
team-lead's job after sprint closes.
