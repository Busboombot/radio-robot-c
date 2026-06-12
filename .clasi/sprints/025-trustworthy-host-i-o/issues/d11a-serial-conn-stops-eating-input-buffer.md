---
status: in-progress
sprint: '025'
tickets:
- 025-001
---

# D11a — Host `serial_conn.py` stops discarding the input buffer (it eats EVT/TLM)

## Context

`SerialConnection.send()` calls `self._ser.reset_input_buffer()` before every write,
so **all buffered-but-unread input is discarded** — in-flight TLM frames, async
`EVT done` / `EVT safety_stop` lines, everything. Any periodic host activity that
uses `send()` (SNAP, SET, GET, CLI/MCP status polls) randomly punches holes in the
stream and can eat the very completion event a `wait_for_evt_done()` elsewhere is
blocking on. This is a first-order cause of "the telemetry stream keeps dying" and
of motions that "never complete" on the host side even though the firmware emitted
the EVT. It is host-side and cheap — likely resolves most stream complaints
independent of firmware work.

## Fix (improvement-plan P2.2.0)

- Delete `reset_input_buffer()` from `send()`.
- Replace the read-after-write pattern with a single reader thread that demultiplexes
  incoming lines into: (a) a reply queue keyed by corr-id, (b) a TLM stream queue,
  (c) an EVT queue — so synchronous sends and stream consumers stop fighting over one
  input buffer.

## Acceptance

- Stream survives an idle→drive→idle cycle and a burst of SET/GET/SNAP polls without
  losing TLM frames or `EVT done`; `wait_for_evt_done()` never misses an event that
  was emitted.

## Source
Defect **D11a** in the 2026-06-11 sim2real review (+ scenario 4.5); fix P2.2.0.
Pure host-side change; no firmware dependency — good early win.
