---
id: '001'
title: Rewrite bench harness to use robot USB serial
status: done
use-cases:
- SUC-001
depends-on: []
issue: fr-bench-dbg-otos-no-reply.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Rewrite bench harness to use robot USB serial

## Description

`tests/bench/bench_validation_032.py` and `tests/bench/enc_balance_test.py` currently open
the relay's serial port and use the `!GO` data-plane protocol to reach the robot. DBG replies
(`ForceReply::SERIAL`) are routed to the robot's own USB serial port — a listener on the relay
never sees them. This is the root cause of the sprint 032 bench session confusion.

Rewrite both scripts to open the **robot's USB serial port directly** using
`SerialConnection(port, mode="direct")` or equivalent (`serial.Serial` directly). This is
confirmed working: plain commands + corr-ids, no relay, no `!GO`.

No firmware change. This is a tooling-only ticket.

## Acceptance Criteria

- [x] `bench_validation_032.py` opens the robot's USB serial directly — no `!GO`, no relay
      data-plane protocol; uses `SerialConnection(port, mode="direct")` or equivalent
- [x] `enc_balance_test.py` similarly opens the robot's direct USB serial
- [x] Both scripts accept a `--port` argument pointing at the robot's USB serial device
      (default to a robot USB device path, not the relay path)
- [x] Script imports are correct; no NameErrors on import
- [x] Hardware re-run is NOT an acceptance gate for this ticket — that is a post-sprint
      team-lead task

## Testing

- **Existing tests to run**: `uv run --with pytest python -m pytest host_tests/ host/tests/`
  (no firmware change so this is a regression check only)
- **New tests to write**: None for tooling scripts; verify by inspection that the connection
  path is correct and `!GO` / relay preamble are absent
- **Verification command**: `uv run --with pytest python -m pytest host_tests/ host/tests/`

## Implementation Plan

### Approach

Replace the `Relay` class in `bench_validation_032.py` (and equivalent relay setup in
`enc_balance_test.py`) with a direct serial wrapper. Check whether
`robot_radio.connection.SerialConnection(port, mode="direct")` is importable and suitable
— if so, use it directly. Otherwise, open `serial.Serial(port, 115200, timeout=0.2)` and
send plain `cmd\n` lines, reading replies with a timeout loop.

### Files to Modify

- `tests/bench/bench_validation_032.py` — replace `Relay` class and `main()` connection
  setup; update `--port` default to a robot USB serial device; update docstring to reflect
  direct-serial usage
- `tests/bench/enc_balance_test.py` — replace relay-based `!GO` connection; update `--port`
  default and docstring

### Documentation Updates

Update both script docstrings: remove relay/`!GO` references, document that the robot's USB
serial port is required (not the relay port).
