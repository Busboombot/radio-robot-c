---
id: 009
title: Protocol v2 specification document (docs/protocol-v2.md)
status: done
use-cases:
- SUC-008
depends-on:
- '003'
- '004'
- '005'
- '006'
- '007'
- 008
issue: protocol-v2-raw250-hard-break.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 009-009: Protocol v2 specification document

## Description

Write `docs/protocol-v2.md` — the single source of truth for the v2 wire
protocol. This document mirrors the relay's `radio-relay-protocol.md` in style
and completeness. It is written last (after all command tickets are implemented)
so the spec reflects what was actually built, not what was planned.

**Required sections**:
1. **Overview** — one paragraph: line-oriented text, whitespace-delimited tokens,
   `key=value`, proto=2, hard break from v1.
2. **Grammar** — formal BNF or pseudocode:
   ```
   message  ::= verb [token…] ['#' corr_id] '\n'
   verb     ::= UPPERCASE-WORD
   token    ::= positional | key_value
   key_value ::= key '=' value
   ```
3. **Response taxonomy** — table: `OK`, `ERR`, `EVT`, `TLM`, `CFG`, `ID` with
   description and example.
4. **Error codes** — table: `unknown`, `badarg`, `badkey`, `nodev`, `range` with
   meaning.
5. **`#id` correlation** — how it works; when to use it; EVT responses do not carry it.
6. **Liveness / identity** — `PING`, `ECHO`, `ID`, `VER`, `HELP` with exact formats.
7. **Config** — `SET`/`GET` with named-key table (all ~22 keys, types, ranges,
   old K* equivalents).
8. **Telemetry** — `TLM` frame format, all fields, `STREAM`/`SNAP`; note that
   `t=` is stamped at sensor-sample time.
9. **Time synchronization** — min-RTT PING offset algorithm; why robot clock is
   never set from host.
10. **Motion** — `S`, `T`, `D`, `G`, `STOP`, `GRIP`, `ZERO`; async `EVT done`
    and `EVT safety_stop`.
11. **OTOS / port I/O** — `OI`, `OZ`, `OR`, `OP`, `OV`, `OL`, `OA`, `P`, `PA`.
12. **Buffer / framing note** — max message size 511 bytes (512-byte buffer);
    RAW250 transport fragments transparently.
13. **Verification examples** — copy the verification items from the issue:
    ECHO round-trip, GET dump, TLM frame, host end-to-end, clock-sync alignment.

**Style**: factual and terse, like a reference manual. Include exact wire examples
for every command. No implementation notes (those stay in code comments).

## Acceptance Criteria

- [x] `docs/protocol-v2.md` exists and covers all 13 sections above.
- [x] Every command implemented in tickets 003–006 appears in the document.
- [x] Named-key table in §7 matches the registry implemented in ticket 004.
- [x] No legacy commands (K*, ENC, SO, SSE, SSO, SSC, SSL, HELLO, DEVICE:, X) appear as commands (they may appear in a "removed from v1" note).
- [x] `docs/overview.md` is updated: remove the paragraph "The Python host must connect … No protocol changes are permitted." (that statement is now false).

## Implementation Plan

**Approach**: Author `docs/protocol-v2.md` by compiling the wire formats from
tickets 003–006 and the issue. Update `docs/overview.md` to remove the v1
compatibility claim.

**Files to create**:
- `docs/protocol-v2.md`

**Files to modify**:
- `docs/overview.md` — remove stale v1 compatibility paragraph (lines 14–15 and the last paragraph of the "Why It Exists" section)

**Testing**: No automated tests. Review checklist:
- [ ] All command examples in the document work against the implemented firmware (spot-check 5 commands over serial).
- [ ] No commands in the document are unimplemented.
- [ ] A reviewer can onboard to the protocol using only this document.
