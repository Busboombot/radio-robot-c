---
id: '002'
title: I2CBus wrapper with re-entrancy guard and return-code capture
status: done
use-cases:
- SUC-002
- SUC-004
depends-on: []
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 015-002: I2CBus wrapper with re-entrancy guard and return-code capture

## Description

Create `source/hal/I2CBus.h` and `source/hal/I2CBus.cpp`: a thin class wrapping
`MicroBitI2C&` that intercepts every `write()` and `read()` call to provide:

- Per-device transaction counters (keyed by I2C address).
- Per-device error counters and last error code (CODAL status int).
- A re-entrancy guard (`bool _inUse` flag, atomically checked via
  `target_disable_irq()`/`target_enable_irq()`) that captures violations with
  the in-flight address pair.

Then thread all four device classes (`Motor`, `OtosSensor`, `LineSensor`,
`ColorSensor`) through `I2CBus&` instead of `MicroBitI2C&`, and construct one
`I2CBus` in `source/main.cpp`.

This is Phase 1 instrumentation from the issue plan. The wrapper is a
diagnostic, NOT a lock. A plain mutex is a no-op in the single-loop
architecture (see issue T3); the re-entrancy guard measures the concurrency
assumption instead of asserting it.

## Acceptance Criteria

- [x] `source/hal/I2CBus.h` declares the class with:
  - `I2CBus(MicroBitI2C& bus)` constructor.
  - `int write(uint16_t address, uint8_t* data, int len, bool repeated)` --
    returns the CODAL status int.
  - `int read(uint16_t address, uint8_t* data, int len, bool repeated)` --
    returns the CODAL status int.
  - `uint32_t txnCount(uint16_t addr)` and `uint32_t errCount(uint16_t addr)` --
    per-device counters.
  - `int lastErr(uint16_t addr)` -- last non-OK CODAL status for that device.
  - `uint32_t reentryViolations()` -- total re-entrancy violation count.
  - `uint16_t reentryInFlightAddr()` and `uint16_t reentryNewAddr()` -- addresses
    captured at the most recent violation.
- [x] `write()` and `read()` check/set `_inUse` atomically via
  `target_disable_irq()`/`target_enable_irq()` around the flag access only
  (NOT around the full I2C transaction).
- [x] On re-entrancy, violation counter increments and address pair is captured;
  transaction proceeds (guard is diagnostic, not blocking).
- [x] `Motor`, `OtosSensor`, `LineSensor`, `ColorSensor` constructors take
  `I2CBus&` instead of `MicroBitI2C&`; stored field type updated accordingly.
  All `_i2c.write()` / `_i2c.read()` call sites are unchanged in logic.
- [x] `source/main.cpp`: one `static I2CBus bus(uBit.i2c)` constructed before
  device objects; all four device constructors receive `bus`.
- [x] Build passes clean: `python3 build.py --clean` with zero errors and zero
  warnings introduced by this ticket.
- [x] Host test suite passes: `uv run --with pytest python -m pytest` (no
  regressions).

## Implementation Plan

### Approach

`I2CBus` is a pure wrapper -- it stores a `MicroBitI2C&` and forwards all calls.
Per-device counters use a small fixed-size table keyed by address (4 devices;
a linear scan over 4 entries is fine). The `_inUse` flag window is the 2-3
instructions around the check-and-set only, not the full I2C transaction.

### Files to Create

- `source/hal/I2CBus.h` -- new file
- `source/hal/I2CBus.cpp` -- new file

### Files to Modify

- `source/hal/Motor.h` -- constructor: `MicroBitI2C&` -> `I2CBus&`; stored field type
- `source/hal/Motor.cpp` -- constructor parameter type
- `source/hal/OtosSensor.h` -- same
- `source/hal/OtosSensor.cpp` -- same
- `source/hal/LineSensor.h` -- same
- `source/hal/LineSensor.cpp` -- same
- `source/hal/ColorSensor.h` -- same
- `source/hal/ColorSensor.cpp` -- same
- `source/main.cpp` -- add `static I2CBus bus(uBit.i2c)`; update 4 constructor calls
- `source/CMakeLists.txt` (or equivalent build file) -- add `I2CBus.cpp` to sources

### I2CBus internal structure

```
// Per-device slot (4 entries, linear scan by addr)
struct DeviceSlot {
    uint16_t addr;
    uint32_t txnCount;
    uint32_t errCount;
    int      lastErr;
};
static const int kMaxDevices = 8;  // headroom beyond current 4

bool     _inUse;
uint32_t _reentryViolations;
uint16_t _reentryInFlightAddr;
uint16_t _reentryNewAddr;
DeviceSlot _devices[kMaxDevices];
int        _deviceCount;
```

`findOrAdd(addr)` returns the slot index for a given address (inserts new entry
up to kMaxDevices; if table full, returns a discard index so counters saturate
safely without crashing).

### Testing Plan

- Build clean after each file change; verify zero new warnings.
- Run `uv run --with pytest python -m pytest` after completion.
- On hardware: after flashing, `DBG I2C` (from T3) will confirm counters are
  incrementing -- this bench validation is part of Ticket 3 acceptance, not
  Ticket 2.

### Documentation Updates

Add a one-sentence comment in `source/main.cpp` explaining why `I2CBus` sits
between `uBit.i2c` and the device objects.
