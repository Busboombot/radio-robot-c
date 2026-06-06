---
id: '003'
title: DBG I2C dump command and EVT enc_wedged firmware detector
status: done
use-cases:
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '002'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 015-003: DBG I2C dump command and EVT enc_wedged firmware detector

## Description

Two firmware additions that make the encoder wedge observable from the serial
console and from `wedge_repro.py`:

**1. `DBG I2C` command** (`source/app/CommandProcessor.cpp`):
Emits one serial line with per-device transaction counts, error counts, last
error codes, re-entrancy violation count, and a consecutive-identical-encoder-
read stuck-counter. Requires `I2CBus` from Ticket 2.

**2. `EVT enc_wedged` emitter** (`source/control/MotorController.cpp`):
Detects when N consecutive encoder reads return the same value while commanded
PWM is non-zero, and emits `EVT enc_wedged ...` once per event with the bus
error and re-entrancy stats at that moment.

This is the second half of Phase 1 instrumentation from the issue plan.
`DBG I2C` confirms that counters are accumulating and that re-entrancy is or
is not occurring. `EVT enc_wedged` gives the exact bus state at the moment the
wedge is detected, which is the key correlating data for Phase 2 experiments.

## Acceptance Criteria

- [x] `DBG I2C` over serial returns exactly one line without crashing.
- [x] The response line fits within 255 bytes for expected counter magnitudes
  (snprintf with length check; truncated safely if overflow).
- [x] Response format includes: per-device txn/err counts for 0x10/0x17/0x1A/0x43,
  last error code per device, re-entrancy violation count, stuck-encoder count.
  Example: `I2C 0x10:txn=142 err=0 last=0 0x17:txn=28 err=0 last=0 0x1A:txn=142 err=0 last=0 0x43:txn=142 err=0 last=0 reentry=0 stuck=L:0,R:0`
- [ ] After driving 50 cycles, `DBG I2C` shows non-zero txn counts for active
  devices. (Hardware bench — not verified offline)
- [ ] `EVT enc_wedged` is emitted when a wedge is induced on hardware (encoder
  value frozen while S command is active). (Hardware bench — not verified offline)
- [x] `EVT enc_wedged` is emitted only once per wedge event (no spam); the
  consecutive counter resets when a new non-identical read is seen.
- [x] `EVT enc_wedged` line includes: stuck count, bus error count, re-entrancy
  violation count, last error code at detection time.
  Actual format: `EVT enc_wedged wheel=L enc=<v> n=<count> err=<busErr> reentry=<n> lastErr=<code>`
- [x] Build passes clean: `python3 build.py --clean`.
- [x] Host test suite passes: `uv run --with pytest python -m pytest`.

## Implementation Plan

### Approach

**DBG I2C**: Add a new `else if (strcmp(sub, "I2C") == 0)` branch in the
`DBG` dispatch block in `CommandProcessor.cpp`. The `CommandProcessor` needs
access to the `I2CBus` instance. The cleanest path is:
- Add `I2CBus& i2cBus()` getter to `Robot` (which already holds all devices),
  storing a reference passed through the constructor; OR
- Pass `I2CBus&` directly to `CommandProcessor`'s constructor.
Either approach is acceptable; pick the one with fewer constructor changes
(likely threading through `Robot`).

**EVT enc_wedged**: In `MotorController::controlTick()` (or wherever encoder
values are collected per tick), maintain a `uint8_t _stuckCount` member.
After collecting both encoder readings:
- If commanded PWM is non-zero AND current enc values equal previous enc values
  (both L and R within ±1 mm), increment `_stuckCount`.
- If `_stuckCount` reaches threshold (5) and `_wedgeEmitted` is false:
  - Call the active reply function (or a dedicated serial emit path) with the
    EVT line.
  - Set `_wedgeEmitted = true`.
- If enc values change (non-identical read), reset `_stuckCount` to 0 and
  `_wedgeEmitted = false`.

The EVT line is constructed with `snprintf` into a <=255-byte buffer and sent
via the existing `replyFn`/`Communicator` serial path (same pattern as other
EVT lines in the codebase).

### Files to Modify

- `source/app/CommandProcessor.cpp` -- add `DBG I2C` branch; wire I2CBus access
- `source/app/CommandProcessor.h` -- add I2CBus parameter if threading via constructor
- `source/control/MotorController.cpp` -- add `_stuckCount`, `_wedgeEmitted`,
  enc-stuck detection + EVT emit in controlTick()
- `source/control/MotorController.h` -- add stuck/emit members; I2CBus& ref for EVT stats
- `source/robot/Robot.cpp` / `source/robot/Robot.h` -- if threading I2CBus through Robot,
  add `I2CBus& _bus` member and `i2cBus()` getter

### Testing Plan

- Build and flash; send `DBG I2C` and confirm the response line appears.
- Run `wedge_repro.py --watchdog-stop --cycles 20`; confirm `EVT enc_wedged`
  appears in serial output when a wedge occurs.
- Run `uv run --with pytest python -m pytest` for regressions.

### Documentation Updates

No documentation file. The EVT line format is self-documenting; add a comment
in `MotorController.cpp` near the detection logic citing the issue file:
`.clasi/issues/residual-motor-encoder-wedge-after-stop.md`.
