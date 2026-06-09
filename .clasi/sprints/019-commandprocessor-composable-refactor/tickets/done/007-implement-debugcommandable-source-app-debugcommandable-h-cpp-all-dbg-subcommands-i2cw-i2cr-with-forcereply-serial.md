---
id: '007'
title: "Implement DebugCommandable \u2014 source/app/DebugCommandable.h/.cpp, all\
  \ DBG subcommands + I2CW/I2CR with ForceReply::SERIAL"
status: done
use-cases:
- SUC-006
depends-on:
- '006'
github-issue: ''
issue: ''
completes_issue: false
---

# Implement DebugCommandable — source/app/DebugCommandable.h/.cpp, all DBG subcommands + I2CW/I2CR with ForceReply::SERIAL

## Description

Create `DebugCommandable` — a `Commandable` that owns all diagnostic commands: the
DBG family (LOOP, LOOP RESET, I2C, I2CLOG, IRQGUARD, WEDGE) plus I2CW and I2CR. All
descriptors use `ForceReply::SERIAL` so debug output always goes to the serial port
regardless of which channel the command arrived on.

This ticket also wires `DebugCommandable` into `main.cpp` with a temporary path that
exercises the new dispatch path, and verifies that DBG LOOP sent via radio still
appears on serial.

## Acceptance Criteria

- [x] `source/app/DebugCommandable.h` declares:
  - `DbgCtx` struct: `{ LoopScheduler* sched; I2CBus* bus; Robot* robot; }`
  - `class DebugCommandable : public Commandable` with constructor `DebugCommandable(DbgCtx ctx)`
  - `virtual int getCommands(CommandDescriptor* buf, int max) const override`
- [x] `source/app/DebugCommandable.cpp` implements `getCommands()` returning descriptors for:
  - `"DBG LOOP RESET"` — ForceReply::SERIAL
  - `"DBG LOOP"` — ForceReply::SERIAL
  - `"DBG I2CLOG"` — ForceReply::SERIAL
  - `"DBG I2C"` — ForceReply::SERIAL
  - `"DBG IRQGUARD"` — ForceReply::SERIAL
  - `"DBG WEDGE"` — ForceReply::SERIAL
  - `"I2CW"` — ForceReply::SERIAL
  - `"I2CR"` — ForceReply::SERIAL
  - Descriptors must be ordered longest-prefix-first (LOOP RESET before LOOP, I2CLOG before I2C)
- [x] All handlers produce wire output identical to the existing DBG/I2CW/I2CR switch cases
- [x] `main.cpp` constructs a `DebugCommandable dbgCmd` and registers it; `setSerialReply` called on `CommandProcessor`
- [x] `python3 build.py` passes with no errors
- [ ] Bench: `DBG LOOP` sent via radio produces output on serial (ForceReply::SERIAL verification)
- [ ] Bench: `DBG I2C` reply appears on serial; `I2CW`/`I2CR` work correctly

## Implementation Plan

### Approach

Read all DBG, I2CW, and I2CR switch cases in `CommandProcessor.cpp` carefully before
writing handlers. Copy the handler logic verbatim; do not simplify or combine. The
`DbgCtx` is stored in `DebugCommandable` as a value member and passed as `handlerCtx`
in each descriptor.

Multi-word prefix ordering: `getCommands()` must fill descriptors in a consistent order.
The longest-prefix dispatcher picks the best match regardless of order, but conventional
ordering (longest first) is easier to read.

### Files to Create

- `source/app/DebugCommandable.h`
- `source/app/DebugCommandable.cpp`

### Files to Modify

- `source/main.cpp` — add `DebugCommandable dbgCmd`; call `cmd.setSerialReply(serialFn, serialCtx)`

### Testing Plan

- Build: `python3 build.py` must pass.
- Bench: Send `DBG LOOP` via radio; observe output on serial monitor.
- Bench: Send `I2CW` and `I2CR` commands; verify correct replies on serial.
- Existing DBG switch cases remain active (old path); no regression possible until T011 cutover.
