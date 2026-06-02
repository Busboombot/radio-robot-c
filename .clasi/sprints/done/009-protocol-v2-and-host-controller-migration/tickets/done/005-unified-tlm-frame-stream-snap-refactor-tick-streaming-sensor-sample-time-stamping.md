---
id: '005'
title: Unified TLM frame + STREAM/SNAP (refactor tick() streaming, sensor-sample-time
  stamping)
status: done
use-cases:
- SUC-004
depends-on:
- '002'
- '004'
issue: protocol-v2-raw250-hard-break.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 009-005: Unified TLM frame + STREAM/SNAP

## Description

Replace the scattered per-sensor streaming callbacks (`SSE`/`SSO`/`SSC`/`SSL` +
separate `ENC`/`SO`/`CS`/`LS` lines) with a single `TLM` frame assembled in
`Robot::tick()`.

**Wire format**:
```
TLM t=12345 mode=S enc=1024,1019 pose=350,-12,1780 line=120,340,330,118 color=21,30,18,80
```
- `t` = robot `systemTime()` captured *at sensor read*, not at string-format time.
- `mode` = current drive mode char (`I`=IDLE, `S`=STREAMING, `T`=TIMED, `D`=DISTANCE, `G`=GO_TO).
- Fields for absent/unconfigured sensors are omitted.
- `vel` field: **omit** in this sprint (depends on Sprint 008 chip-velocity work; see Open Question 2 in architecture update).

**New commands**:
```
STREAM <ms>              → OK stream period=<ms>  (0 = off)
STREAM fields=enc,pose   → OK stream fields=enc,pose  (subset subscription)
SNAP                     → (emits one TLM frame immediately, then OK snap)
```

**Config change**: `RobotConfig` gains `int32_t tlmPeriodMs = 0` (zero = streaming
off) and `uint8_t tlmFields` bitmask. Add these fields to `Config.h` and to the
`SET`/`GET` registry (ticket 004 added `tlmPeriod` to the registry plan).

**`DriveController` cleanup**: Remove `encReportEvery`, `sensorReport` callback,
and all per-sensor streaming enable flags. `DriveController::tick()` no longer
emits any telemetry lines directly — it only advances drive state machines and
emits `EVT` events for completions (ticket 006).

**TLM assembly location**: `Robot::tick()`. Sequence:
1. Read encoder deltas (always).
2. If `tlmPeriodMs > 0` and `now_ms - _lastTlmMs >= tlmPeriodMs` or `_tlmSnapPending`:
   a. Capture `t_sample = uBit.systemTime()` before reading sensors.
   b. Read OTOS pose (if present).
   c. Read line sensor (if present and `enc` or `line` field requested).
   d. Read color sensor (if present and `color` field requested).
   e. Assemble and emit `TLM t=… mode=… enc=… pose=… …` line.
   f. Clear `_tlmSnapPending`; update `_lastTlmMs`.

**Field names for `STREAM fields=`**: `enc`, `pose`, `line`, `color`. All fields
present by default when streaming is on.

## Acceptance Criteria

- [x] `STREAM 40` → robot emits one `TLM` line per 40 ms.
- [x] Each `TLM` frame has `t=<ms>` that advances monotonically.
- [x] `t=` is captured at sensor-read time, not at snprintf time (no send-latency bias).
- [x] `SNAP` → one immediate `TLM` frame, then `OK snap`.
- [x] `STREAM 0` → telemetry stops.
- [x] `STREAM fields=enc,pose` → subsequent frames contain only `enc=` and `pose=` fields.
- [x] `mode=` field reflects current drive state correctly.
- [x] `DriveController` no longer emits `ENC`, `SO`, `CS`, `LS` lines.
- [x] `encReportEvery` field removed from `RobotConfig`.
- [ ] [BENCH] Frames arrive at ~40 ms cadence on hardware; `t=` values advance by ~40 each frame.

## Implementation Plan

**Approach**: Refactor `Robot::tick()` to assemble TLM; clean up `DriveController`
streaming infrastructure; add `STREAM` and `SNAP` handlers in `CommandProcessor`.

**Files to modify**:
- `source/types/Config.h` — add `tlmPeriodMs`, `tlmFields`, `tlmSnapPending`; remove `encReportEvery`
- `source/app/Robot.cpp` (and `Robot.h`) — add `_lastTlmMs`, `_tlmSnapPending`; add TLM assembly in `tick()`; remove `sensorReport` static
- `source/control/DriveController.h` / `DriveController.cpp` — remove streaming callback, per-sensor flags
- `source/app/CommandProcessor.cpp` — add `STREAM` and `SNAP` handlers

**TLM line length estimate**: `TLM t=12345 mode=S enc=1024,1019 pose=350,-12,1780 line=120,340,330,118 color=21,30,18,80` ≈ 90 bytes. Well within 512.

**Testing**:
- Serial: `STREAM 100` → observe TLM lines arriving every ~100 ms.
- Serial: `SNAP` → one TLM line; then silence.
- Serial: `STREAM 0` after streaming → silence.
- Serial: `STREAM fields=enc` → only `enc=` field in frames.
- Verify `t=` values increment by approximately the stream period.
