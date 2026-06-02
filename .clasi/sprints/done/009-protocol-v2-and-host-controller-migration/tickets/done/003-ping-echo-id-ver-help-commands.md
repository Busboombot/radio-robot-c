---
id: '003'
title: PING, ECHO, ID, VER, HELP commands
status: done
use-cases:
- SUC-001
- SUC-002
depends-on:
- '002'
issue: protocol-v2-raw250-hard-break.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 009-003: PING, ECHO, ID, VER, HELP commands

## Description

Implement the five liveness/identity/diagnostic commands in `CommandProcessor`:

- `PING` — replies `OK pong t=<uBit.systemTime()>` (also the clock-sync probe).
- `ECHO <text…>` — replies `OK echo <text…>` preserving the payload exactly. This is the primary fragmentation+reassembly test.
- `ID` — replies `ID model=Nezha2 name=<name> serial=<uBit serial or placeholder> fw=<VERSION_STRING> proto=2 caps=otos,line,color,gripper,portio`. Omit caps that are absent at runtime.
- `VER` — replies `OK ver fw=<VERSION_STRING> proto=2`.
- `HELP` — replies a compact one-line command index (all v2 verbs, comma-separated or brief descriptions).

`ECHO` is the critical test for the buffer ceiling raise (ticket 001) and the fragmentation path. A 200-byte `ECHO` must round-trip intact through the relay.

## Acceptance Criteria

- [x] `PING` → `OK pong t=<ms>` where `<ms>` is `uBit.systemTime()` at the moment of handling.
- [x] `PING #5` → `OK pong t=<ms> #5` (correlation echoed).
- [x] `ECHO hello world` → `OK echo hello world` (payload preserved, case preserved).
- [x] `ECHO` of a 200-byte ASCII payload → `OK echo <payload>` intact (no truncation).
- [x] `ID` → contains `proto=2` and non-empty `caps=` field.
- [x] `VER` → contains `proto=2`.
- [x] `HELP` → non-empty; lists at least `PING ECHO ID VER HELP SET GET STREAM SNAP S T D G STOP GRIP ZERO`.
- [ ] [BENCH] `ECHO` of 200-byte payload round-trips intact over the relay (fragmentation both directions). DEFERRED — bench test at sprint end.

## Implementation Plan

**Approach**: Add five verb handlers in `CommandProcessor::process()` using the v2 token infrastructure from ticket 002.

**Files to modify**:
- `source/app/CommandProcessor.cpp` — add PING, ECHO, ID, VER, HELP handlers
- `source/types/Protocol.h` — add `PROTO_VERSION 2` and firmware `VERSION_STRING` if not already there
- `source/robot/Robot.h` / `source/app/Robot.cpp` — confirm `serialNumber()` or use a placeholder string

**Exact wire formats**:
```
PING            → OK pong t=12345
ECHO hi there   → OK echo hi there
ID              → ID model=Nezha2 name=GUTOV serial=… fw=0.20260601.14 proto=2 caps=otos,line,color,gripper,portio
VER             → OK ver fw=0.20260601.14 proto=2
HELP            → OK help PING ECHO ID VER HELP SET GET STREAM SNAP S T D G STOP GRIP ZERO
```

**`ECHO` implementation note**: The payload is everything after the verb token. Reconstruct from the raw input (not the tokenized version) to preserve spacing and case exactly. The response is `"OK echo "` + payload. Total response length for a 200-byte payload is `"OK echo "` (8 bytes) + 200 bytes = 208 bytes — fits in the 512-byte buffer.

**`caps=` field**: Check at runtime whether each optional subsystem is present (otos, line sensor, color sensor, servo, portio). Omit absent hardware from `caps=`.

**Testing**:
- Serial: `PING` → `OK pong t=<n>` (n is a plausible ms value).
- Serial: `ECHO abc` → `OK echo abc`.
- Serial: construct a 200-byte string and send as `ECHO <payload>` → verify full payload returned.
- [BENCH] Same 200-byte ECHO test over the relay.

**Documentation**: Wire formats feed directly into the spec-doc ticket (009).
