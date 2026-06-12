---
id: '025'
title: Trustworthy host I/O
status: planning-docs
branch: sprint/025-trustworthy-host-i-o
use-cases: []
issues:
- d11a-serial-conn-stops-eating-input-buffer
- a5-serial-transport-encapsulation
- a8-config-registry-sync-lint
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 025: Trustworthy host I/O

## Goals

Every subsequent test and field report can be believed. The host serial
stream survives full drive cycles without silently discarding telemetry or
completion events. The transport boundary is enforced so d11a's reader-thread
guarantee can't be bypassed. Config registry/struct/usage drift is caught by
CI so calibration keys never go silently unread again.

## Problem

Three host-side defects corrupt all observability:

1. `SerialConnection.send()` calls `reset_input_buffer()` before every write,
   discarding in-flight TLM frames, EVT done lines, and safety_stop events
   (d11a). This is the primary cause of "the stream keeps dying" and of
   `wait_for_evt_done()` blocking forever when the firmware did emit the event.
2. Multiple call sites bypass `SerialConnection` and reach `_ser` directly
   (protocol.py, cli.py, cutebot.py, sim_conn.py faking `_ser = None`). Even
   after d11a's reader thread lands, any of these bypasses reintroduces the
   same class of bug (a5).
3. Config struct fields exist with no registry entry (unreachable via GET/SET)
   and registered keys are read by nothing in firmware — drift that caused D2
   (rotationalSlip calibrated but unused for months) and will recur without a
   mechanical check (a8).

## Solution

**d11a first:** delete `reset_input_buffer()` from `send()`; replace the
read-after-write pattern with a single reader thread that demultiplexes
incoming lines into (a) a reply queue keyed by corr-id, (b) a TLM stream
queue, (c) an EVT queue.

**a5 immediately after:** add whatever narrow methods the `_ser` reach-arounds
actually need on `SerialConnection`; convert all offenders; delete `_ser` stub
from sim_conn.py; add a CI grep guard.

**a8 in parallel (independent):** script a CI lint that cross-checks three
sets — fields in `types/Config.h`, entries in `ConfigRegistry.cpp`, references
in `source/` outside DefaultConfig/ConfigRegistry. Resolve current offenders
(register or remove the three unregistered fields). Optionally extend
`gen_default_config.py` to generate `ConfigRegistry.cpp` entries to make
one direction of drift mechanically impossible.

## Success Criteria

- Stream survives idle → drive → idle + burst of SET/GET/SNAP polls without
  losing TLM frames or `EVT done`; `wait_for_evt_done()` never misses an event
  that was emitted (d11a acceptance).
- `grep -rn '_ser' host/robot_radio | grep -v io/serial_conn.py` returns
  nothing; guard is in CI (a5 acceptance).
- Config lint runs in CI and passes; adding an unregistered config field breaks
  the build (a8 acceptance).

## Scope

### In Scope

- `host/robot_radio/io/serial_conn.py` — remove `reset_input_buffer()`, add
  reader-thread demux (d11a).
- `host/robot_radio/robot/protocol.py`, `io/cli.py`, `robot/cutebot.py`,
  `io/sim_conn.py` — remove all direct `_ser` reaches (a5).
- `SerialConnection` — narrow interface additions to support the above (a5).
- CI grep guard for `_ser` outside `io/serial_conn.py` (a5).
- `scripts/` — config sync lint; `types/Config.h` / `ConfigRegistry.cpp`
  offender resolution (a8).
- CI integration of lint (a8).

### Out of Scope

- Firmware changes of any kind.
- D10 firmware-side telemetry items (seq numbers, idle rate, channel binding)
  — those are sprint 028.
- Any host navigation or calibration logic changes.

## Test Strategy

- Host unit tests confirming the reader thread correctly routes reply/TLM/EVT
  lines; existing protocol tests must continue passing.
- Stress test: send SET/GET/SNAP bursts concurrently with a streaming drive; no
  TLM frames lost in 60 s.
- CI: `_ser` grep guard + config lint gate both run on every PR.

## Architecture Notes

The d11a → a5 sequencing is strict: land the reader thread first, then close
the bypass points, so there is never a window where bypasses coexist with the
new demux path. a8 is independent and can overlap.

The `field-024-full-speed-spin-unresolved` issue notes that the host bench
program (`square_run.py`) abandoned an autonomous G without sending X, and that
SNAP vs STREAM TLM showed a discrepancy. The d11a reader thread and a5
transport seal are prerequisites for trusting those observations; the
spin-unresolved anomalies are not directly addressed here but the tooling
improvements land first. See sprint 027 for behavioral fixes.

## Why First

Cheap (no firmware risk), entirely host-side, completes in roughly one session.
Every later sprint's verification — sim tests, hardware smoke ritual, field
bench tools — reads through this plumbing. Fixing observability before
changing behavior avoids chasing ghosts in a broken stream.

## Sizing

Small — approximately 1 focused session.

## GitHub Issues

(None yet — link when created.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Reader thread demux — stop clearing input buffer on send | — |
| 002 | Seal transport boundary — remove all _ser reaches outside serial_conn.py | 001 |
| 003 | Config registry sync lint in CI | — (independent) |

Tickets 001 and 002 execute serially in that order. Ticket 003 is independent
and may be worked in parallel with 001/002 or sequenced after them.
