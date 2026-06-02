---
id: 009
title: Protocol v2 and Host Controller Migration
status: done
branch: sprint/009-protocol-v2-and-host-controller-migration
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
issues:
- protocol-v2-raw250-hard-break.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 009: Protocol v2 + Host Controller Migration

## Goals

Rebuild the command/telemetry surface on a clean v2 wire format (a hard
break — no legacy compatibility), riding on the thin parser from 007:

- **Buffers & framing**: raise `REASM_MAX`/`_buf`/TX to ~512 so a full
  `GET` dump / large `SET` fits in one message; confirm
  `MICROBIT_RADIO_MAX_PACKET_SIZE=250`. (The RAW250 transport itself is
  already done.)
- **Parser core**: tokenizer (verb + positional + `key=value`),
  verb-only upper-casing, `#id` request correlation, and the
  `OK/ERR/EVT/TLM/CFG/ID` response taxonomy. Hard-remove the legacy
  packed single-letter parsing and `DEVICE:`/`HELLO`.
- **Liveness/identity**: `PING` (also the clock-sync probe), `ECHO`
  (fragmentation round-trip test), `ID`, `VER`, `HELP`.
- **Config**: `SET`/`GET` named-key registry replacing the per-constant
  `K` commands; decimals only where fractional, distances stay integer
  mm, no implicit scaling.
- **Telemetry**: one unified `TLM` frame + `STREAM`/`SNAP`; stamp `TLM t=`
  at sensor-sample time, not send time.
- **Time synchronization**: host-side offset/skew estimator over a `PING`
  burst (min-RTT filtering), translating robot `t` → host time. Robot
  side is just the existing `PING t=`.
- **Motion verbs + `GRIP` de-overload + `ZERO` umbrella**.
- **Host controller migration**: copy `robot_radio` into this repo and
  adapt its protocol layer (`protocol.py`/`nezha.py` equivalents) to v2.
- **Docs**: a `protocol-v2` spec.

## Issues Addressed

- `protocol-v2-raw250-hard-break.md` — full RAW250 text redesign (hard
  break) + host controller migration.

## Rationale for Grouping

This is a single large, cohesive protocol issue that stands alone as its
own sprint. It rebuilds the entire command vocabulary on the clean
parse-and-dispatch structure produced by 007, and must land **before**
the kinematics command surface so the new motion/velocity commands
(go-to, `(v,ω)` primitive) are authored directly in the v2 format rather
than the terse legacy framing — the issue's own "reconcile during sprint
planning" note. The host-controller copy belongs here because it must
speak v2 from the start.

## Dependency Notes

- **Depends on:** 007 — rewrites only the wire format/taxonomy on the
  already-clean `CommandProcessor`; relies on the reply-sink routing and
  thin parser from the foundation sprint.
- **Blocks:** 011 — the go-to / `(v,ω)` command surface is expressed in
  v2, so v2 lands before pose control. (009 and 010 are independent and
  may proceed in parallel; 011 needs both.)
- Independent of 008 and 010 at the code level (different layers).

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Raise buffer ceiling to 512 (REASM_MAX, _buf, confirm codal.json) | — |
| 002 | Parser core v2: tokenizer, verb-only uppercasing, #id correlation, OK/ERR/EVT taxonomy, hard-remove legacy | 001 |
| 003 | PING, ECHO, ID, VER, HELP commands | 002 |
| 004 | SET/GET named-key config registry (replaces K* commands) | 002 |
| 005 | Unified TLM frame + STREAM/SNAP (refactor tick() streaming, sensor-sample-time stamping) | 002, 004 |
| 006 | Motion verbs v2: S, T, D, G, STOP, GRIP de-overload, ZERO umbrella | 002 |
| 008 | Host controller migration: copy robot_radio into repo, rewrite protocol layer for v2 | 003, 004, 005, 006 |
| 007 | Host-side clock-sync module (min-RTT PING burst, robot-to-host time translation) | 008 |
| 009 | Protocol v2 specification document (docs/protocol-v2.md) | 003, 004, 005, 006, 007, 008 |

Tickets execute serially in the order listed. Tickets 003, 004, and 006 may be
parallelized (all depend only on 002). Ticket 005 requires 004 first. Ticket 008
(host migration) requires all firmware commands to be in place. Ticket 007
(clock-sync) requires the host package from 008. Ticket 009 (spec doc) closes
the sprint.
