---
status: in-progress
sprint: '025'
tickets:
- 025-002
---

# A5 — Enforce the host transport boundary: nothing outside io/ touches `_ser`

## Context

Companion to `d11a-serial-conn-stops-eating-input-buffer` (which fixes
`send()`'s buffer-clearing and adds the single reader thread). Beyond that defect,
the transport boundary is leaky everywhere — multiple call sites read/write the
pyserial object directly, bypassing `SerialConnection`:

- `robot/protocol.py:269` — `ser = self._conn._ser` (raw port access inside the
  protocol layer).
- `io/cli.py:287–289` — raw `conn._ser.reset_input_buffer()` /
  `_ser.write(b"HELLO\n")` / `flush()`.
- `robot/cutebot.py:93–94` — raw `_conn._ser.write(...)`.
- `io/sim_conn.py:69` — must fake `_ser = None` purely because outside callers
  reach for the private member.

Several uncoordinated readers/writers compete for one input buffer, so TLM/EVT
lines are randomly consumed depending on which code path is active. Until this is
closed, the d11a reader-thread fix can be silently bypassed by any of these call
sites, reintroducing the same class of bug.

## Fix

1. Land d11a's single-reader design first.
2. Add whatever narrow methods the offenders actually need on `SerialConnection`
   (e.g. `send_raw_line()`, an explicit `handshake()` for the CLI HELLO probe) and
   convert all `_ser` reaches to them.
3. Delete the `_ser` stub from `sim_conn.py`; sim and serial connections expose the
   same interface.
4. Guard: a unit test or CI grep asserting `_ser` is not referenced outside
   `io/serial_conn.py`.

## Acceptance

- `grep -rn '_ser' host/robot_radio | grep -v io/serial_conn.py` returns nothing;
  guard in CI; cutebot/protocol/cli behave identically over sim_conn and serial.

## Priority suggestion

**Medium-high — schedule as the immediate follow-on to d11a in the same sprint.**
Small, mechanical, host-only; without it d11a's guarantees don't hold.

## Source
Finding **A5** in `docs/code_review/2026-06-11-architecture-modularity-review.md`.
