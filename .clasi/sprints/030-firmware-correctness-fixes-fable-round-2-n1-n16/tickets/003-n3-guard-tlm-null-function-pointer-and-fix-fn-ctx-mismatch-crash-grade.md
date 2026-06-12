---
id: '003'
title: 'N3: Guard TLM null function pointer and fix fn/ctx mismatch (crash-grade)'
status: done
use-cases:
- SUC-003
depends-on: []
github-issue: ''
issue: fr2-n3-tlm-null-ctx.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# N3: Guard TLM null function pointer and fix fn/ctx mismatch (crash-grade)

## Description

`Robot::telemetryEmit()` calls `fn(tlmBuf, ctx)` with no null check (`Robot.cpp:448`).
`loopTickOnce` invokes it whenever `cfg.tlmPeriodMs > 0` (`LoopTickOnce.cpp:130-134`).

Problem 1 (null call): `_tlmBoundFn` stays nullptr until STREAM binds the channel.
But `tlmPeriodMs` is also settable via `SET tlmPeriod=100` (`ConfigRegistry.cpp:81`),
which does not bind. `SET tlmPeriod=100` with no prior STREAM results in a null
function-pointer call on the next TLM tick — HardFault on the micro:bit. The header
comment on `Robot.h:148-149` says "nullptr means TLM is suppressed" — but nothing
implements that guard.

Problem 2 (fn/ctx mismatch): TLM is emitted with `ts.activeCtx` (the channel of the
last command received, `LoopTickOnce.cpp:132`) rather than the bound stream channel's
ctx. STREAM over serial + any later radio command causes `serialReplyTlm(msg, &radio)`
to cast `Radio*` to `SerialPort*` and call `sendReliable` on it — undefined behavior.
Mixed serial+radio is the normal field setup.

## Acceptance Criteria

- [x] `telemetryEmit()` guards `fn == nullptr` and suppresses TLM emission (no crash,
      no ERR response — silent suppression matches the header comment).
- [x] TLM is emitted using `_tlmBoundCtx` (the bound channel ctx), not `ts.activeCtx`.
- [x] New sim test: `SET tlmPeriod=100` with no prior STREAM does not crash; no TLM
      is emitted.
- [x] New sim test: STREAM over serial followed by a radio command keeps TLM on the
      serial channel (fn and ctx are the serial-bound pair).
- [x] `python3 build.py` clean build passes.
- [x] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.

## Implementation Plan

### Approach

Two changes in `Robot.cpp` / `LoopTickOnce.cpp`:
1. Add null guard to `telemetryEmit()` (or in the `loopTickOnce` call site).
2. Pass `_tlmBoundCtx` and `_tlmBoundFn` to the emit call, not the active ctx.

### Files to modify

- `source/robot/Robot.cpp`
  - `telemetryEmit()`: add `if (fn == nullptr) return;` before the fn call.
  - Ensure `_tlmBoundCtx` is accessible to the emit path. If it is a member of
    `Robot`, pass it alongside `_tlmBoundFn` to the call site, or read it inside
    `telemetryEmit()` directly.
- `source/app/LoopTickOnce.cpp`
  - `loopTickOnce()` TLM emit call (`:130-134`): change `ts.activeCtx` to
    `robot._tlmBoundCtx` (or an accessor). Pass `robot._tlmBoundFn` and
    `robot._tlmBoundCtx` together.
- `source/robot/Robot.h` — add accessor for `_tlmBoundCtx`/`_tlmBoundFn` if they
  are private, or expose them via a struct (keep it minimal).
- `host_tests/` or `host/tests/` — add the two regression tests above.

### Testing plan

Run: `uv run --with pytest python -m pytest host_tests/ host/tests/ -v`

Build: `python3 build.py` (clean).

### Notes

- Independent of tickets 001 and 002.
- Do NOT change the TLM wire format. Only the guard and ctx-source change.
- Do not reject `SET tlmPeriod` at the SET handler — silent suppression is
  preferred to a breaking wire change (see architecture decision §5, N3).
