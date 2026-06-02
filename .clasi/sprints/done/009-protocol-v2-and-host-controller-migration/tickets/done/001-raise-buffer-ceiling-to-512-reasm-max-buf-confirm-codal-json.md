---
id: '001'
title: Raise buffer ceiling to 512 (REASM_MAX, _buf, confirm codal.json)
status: done
use-cases:
- SUC-001
- SUC-003
depends-on: []
issue: protocol-v2-raw250-hard-break.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 009-001: Raise buffer ceiling to 512 (REASM_MAX, _buf, confirm codal.json)

## Description

The current firmware caps single-message size at 255 bytes:
- `Radio::REASM_MAX = 256` in `source/hal/Radio.h` (reassembly buffer and
  completed-message buffer are both this size).
- The main-loop receive buffer in `main.cpp` (or `Robot.h`) is `char _buf[256]`.

A full `GET` config dump (~24 params, ~290 bytes) and a large `SET` or `ECHO`
payload will overflow these buffers. The relay reassembles up to 1024 bytes; the
RAW250 transport already fragments transparently. This ticket raises both constants
to 512 and confirms `MICROBIT_RADIO_MAX_PACKET_SIZE=250` in `codal.json` (must be
250 to match the relay's 247-byte MTU; set in Sprint 007 but confirm).

This is the prerequisite for all v2 parser work and must land first.

## Acceptance Criteria

- [x] `Radio::REASM_MAX` changed from 256 to 512 in `source/hal/Radio.h`; both
      `_reasm[]` and `_msg[]` arrays reflect the new constant.
- [x] Main-loop receive buffer raised to 512 (find the actual declaration —
      likely a stack-local in `main.cpp` passed to `radio.poll()` / `serial.readLine()`).
- [x] `codal.json` contains `"MICROBIT_RADIO_MAX_PACKET_SIZE": 250`; update only
      if incorrect.
- [x] Firmware builds cleanly after these changes.
- [ ] [BENCH] After ticket 002 lands: `ECHO` of a 200-byte payload round-trips
      intact over the relay, confirming reassembly at the new ceiling. (DEFERRED — bench test at sprint end)

## Implementation Plan

**Approach**: Two-constant change plus one config file confirmation. No logic changes.

**Files to inspect then modify**:
- `source/hal/Radio.h` — `REASM_MAX 256` → `REASM_MAX 512`
- `main.cpp` (or wherever the receive buffer is declared) — raise to 512
- `codal.json` — read; update only if `MICROBIT_RADIO_MAX_PACKET_SIZE` is missing or wrong

**Testing**:
- Build firmware; verify no compile errors.
- The TX path (`Radio::send()`) fragments at MTU=247 per frame; no TX buffer
  constant needs changing.
- Full round-trip verification deferred to ticket 002 once `ECHO` exists.

**Notes**:
- Do not raise `MICROBIT_RADIO_MAX_PACKET_SIZE` above 250 — it must match the relay.
