---
id: '004'
title: 'Rework DebugCommandable DBG OTOS handlers: remove hal downcast, fix integer formatting (F1)'
status: open
use-cases:
- SUC-034-003
- SUC-034-005
depends-on:
- '003'
github-issue: ''
issue: bench-otos-synthetic-otos-sensor-for-full-stack-bench-testing.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Rework DebugCommandable DBG OTOS handlers: remove hal downcast, fix integer formatting (F1)

## Description

Rework the two `DBG OTOS` handlers in `DebugCommandable.cpp` to:
1. Remove the per-call `static_cast<NezhaHAL*>(&ctx.robot->hal)` downcasts from inside the handlers (ticket 003 deleted `Robot::setBenchOtosEnabled` and `isBenchOtosActive` which the handlers were calling through; those calls also need updating here).
2. Fix F1: replace `%f`/`%g`/`%.1f` format specifiers in `handleDbgOtos` with scaled-integer formatting, since CODAL/newlib-nano does not support float printf and the reply currently prints empty fields on hardware.

### Handler restructuring

`DebugCommandable` is firmware-only (excluded from HOST_BUILD by existing guards). It may hold a `NezhaHAL*` directly. The approach:

- Add a `NezhaHAL* _nhal` member to `DebugCommandable` (declared after `_ctx` in `DebugCommandable.h`). Initialize it in the `DebugCommandable` constructor (which is called from `main.cpp` where `NezhaHAL` is visible). In `main.cpp`, pass the `NezhaHAL&` to the `DebugCommandable` constructor as a new parameter (or cast from `Hardware&` at the single site where `DebugCommandable` is constructed — this is acceptable since `main.cpp` is firmware-only).
- `handleDbgOtosBench`: replace `static_cast<NezhaHAL*>(&ctx.robot->hal)` with `ctx_->nhal` (or however the NezhaHAL pointer is stored). Replace calls to `ctx.robot->setBenchOtosEnabled(...)` with `nh->setOtosBench(...)` directly. Replace `ctx.robot->isBenchOtosActive()` with `nh->isBenchMode()` directly.
- `handleDbgOtos`: same downcast replacement. Additionally, rework the `snprintf` format string (F1 fix — see below).

### F1: integer formatting for DBG OTOS

Current `handleDbgOtos` (DebugCommandable.cpp:515-522) formats with `%f`:
```cpp
snprintf(..., "ideal=%.1f,%.1f,%.4f otos=...", idealX, idealY, idealH, ...);
```

Replace with integer-scaled format matching SNAP:
- Position fields (x, y): `%d` mm (round to nearest integer: `(int)roundf(val)`)
- Heading field (h): `%d` cdeg (centidegrees: `(int)roundf(val * (18000.0f / 3.14159265f))`)
- `roundf` is available in newlib-nano without float printf; confirm or use integer arithmetic if needed.

New reply format (example):
```
ideal=0,0,0 otos=2,-1,45 fused=150,20,1745 err=-2,1,-45
```

Note: the F1 bug is hardware-only (host sim libc has float printf). The host sim test `test_dbg_otos_query_returns_pose_fields` currently checks for float fields — it must be updated to expect integer fields after this change, or a separate assertion is added.

### Impact on `DbgCtx` struct

`DbgCtx` (DebugCommandable.h) currently has `robot*`, `sched*`, `bus*`. We do NOT add `NezhaHAL*` to `DbgCtx` (that would spread the concrete-type dependency). Instead, store `NezhaHAL* _nhal` as a private member of `DebugCommandable`, NOT in the shared context struct.

## Files to Modify

- `source/app/DebugCommandable.h` — add `NezhaHAL* _nhal = nullptr` private member. Update constructor signature to accept `NezhaHAL*` (or `Hardware&` with downcast in ctor — but prefer explicit `NezhaHAL*` for clarity).
- `source/app/DebugCommandable.cpp` — update constructor; rework `handleDbgOtosBench` and `handleDbgOtos` as described.
- `source/app/main.cpp` — update `DebugCommandable` construction to pass `NezhaHAL*` (or cast at the single call site).
- `host_tests/` — update any test that checks `DBG OTOS` reply format to expect integer output instead of float output. Search for `ideal=` in test files.

## Acceptance Criteria

- [ ] `grep -n "static_cast<NezhaHAL\*>" source/app/DebugCommandable.cpp` returns no hits inside handler function bodies (any remaining cast is in the constructor or a private init method).
- [ ] `handleDbgOtosBench` calls `_nhal->setOtosBench(...)` and `_nhal->isBenchMode()` directly (no Robot method call for this).
- [ ] `handleDbgOtos` uses `%d` integer format for all numeric fields; no `%f`, `%g`, `%.1f`, or similar in the pose reply line.
- [ ] `DBG OTOS` reply format: `ideal=<xmm>,<ymm>,<hcdeg> otos=... fused=... err=...` (all integers, mm and cdeg).
- [ ] Existing host sim test for `DBG OTOS` reply fields updated to match integer format and passes.
- [ ] New host sim test: given a known non-zero pose in the bench sensor, `DBG OTOS` reply contains non-blank integer fields in the expected range.
- [ ] `python3 build.py` exits clean.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` exits green.
- [ ] Stakeholder hardware verification: `DBG OTOS` on the physical micro:bit (after `DBG OTOS BENCH 1` + a drive command) shows non-empty integer fields. (This is a manual step post-flash; not automated.)

## Testing

- **Build gate**: `python3 build.py` clean.
- **Sim gate**: `uv run --with pytest python -m pytest host_tests/ host/tests/` green.
- **New host test**: assert integer format produces non-empty, in-range values. Example: set `idealX=500.0f, idealY=-100.0f, idealH=1.5708f` (90 deg); expect `ideal=500,-100,9000` (cdeg = 1.5708 * 18000/pi ≈ 9000).
- **Hardware test (stakeholder)**: `DBG OTOS` reply on hardware shows non-empty integer pose fields after bench mode activated and drive issued. Report: does F1 symptom (`ideal=,,`) appear? Should not.
- Note: this is the ONLY ticket where the F1 fix can be verified on hardware; the host sim cannot catch the `%f`-emits-nothing bug because host libc has full float printf.
