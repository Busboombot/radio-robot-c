---
id: '006'
title: "Build verification \u2014 run python build.py and fix all compile errors"
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on:
- '005'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Build verification — run python build.py and fix all compile errors

## Description

This is a validation ticket, not a code-writing ticket. The programmer agent
runs `python build.py` and iterates on fixing compile errors until the build
succeeds and produces a `.hex` file. No new files are created here — only
fixes to files created in tickets 001–005.

The build is Docker-based and will download the CODAL library dependencies
on the first run. The first run may take several minutes. Subsequent runs are
faster.

## What to Do

1. Run `python build.py` from the project root.
2. Read the compiler output carefully. Fix errors one at a time.
3. Common categories of errors to expect and how to fix them:
   - **Missing headers**: Add the correct `#include` to the affected `.h` file.
   - **Undefined types**: Add forward declarations or includes.
   - **CODAL API mismatch**: Read the actual CODAL header in `libraries/codal-microbit-v2/inc/`
     (available after first build) and use the correct method name/signature.
   - **`MICROBIT_NO_DATA` undefined**: Find the correct constant name in CODAL error headers.
   - **`uBit` not declared**: Ensure no sample file references the old global `uBit`; either
     remove conflicting sample files or adjust the samples to not compile into the main binary.
   - **Multiple definition of `uBit`**: The old global `uBit` in `main.cpp` must be removed
     since `uBit` now lives inside `Robot`.
   - **`ManagedString::toCharArray()` not found**: The method may be `getCharArray()` or
     require a `.operator const char*()` cast — check CODAL `ManagedString.h`.
   - **`setServoValue` not found on MicroBitPin**: Use `setAnalogValue()` with
     `setAnalogPeriodUs(20000)` to generate a 50 Hz servo signal instead.
4. After each fix, re-run `python build.py` to check progress.
5. When the build produces no errors and the `.hex` exists in the output, the ticket is done.

## CODAL API Lookup

After the first build, CODAL headers are available at:
```
libraries/codal-microbit-v2/inc/
libraries/codal-microbit-v2/libraries/codal-core/inc/
```

Use these to verify:
- `MicroBitI2C` write/read method signatures and address format
- `MicroBitSerial` read/send method names and constants
- `MicroBitRadio` datagram API
- `ManagedString` character access method
- `PacketBuffer` access API (`.length()`, `operator[]`)
- `MicroBitPin` servo API

## Acceptance Criteria

- [x] `python build.py` completes with exit code 0
- [x] A `.hex` file is produced in the build output directory
- [x] Zero compiler errors; zero compiler warnings treated as errors
- [x] No modifications to files outside `source/` (no CMakeLists.txt changes)
- [x] The `source/samples/` directory compiles cleanly or has been adjusted to avoid conflicts

## Testing

The passing build is the test. Hardware verification (flashing and observing
serial output) is part of the sprint acceptance criteria but is done manually
by the operator after this ticket closes:

1. `python scripts/deploy.py` — flash the `.hex`
2. Open serial at 115200 baud; confirm `DEVICE:Nezha2:` appears within 3 s
3. Send `HELLO\n`; confirm announcement is re-emitted
4. Leave running 60 s; confirm no panic pattern
