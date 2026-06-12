---
id: '002'
title: "Seal transport boundary \u2014 remove all _ser reaches outside serial_conn.py"
status: done
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: a5-serial-transport-encapsulation.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Seal transport boundary — remove all _ser reaches outside serial_conn.py

## Description

Four call sites outside `io/serial_conn.py` bypass `SerialConnection` and
read or write `_ser` directly. Even after ticket 001 lands the reader thread,
any of these bypass points reintroduces the same class of bug by competing
with the reader for the input buffer or writing bytes that the reader never
sees routed correctly.

This ticket converts all four bypass sites to named `SerialConnection` methods
(added in ticket 001), removes the `_ser = None` stub from `sim_conn.py`, and
adds a CI grep guard that enforces the boundary on every PR.

### Bypass sites to fix

| File | Line(s) | Current bypass | Replacement |
|---|---|---|---|
| `host/robot_radio/robot/protocol.py` | 269–273 | `self._conn._ser` / `in_waiting` / bulk `read()` | `self._conn.read_pending_lines()` |
| `host/robot_radio/io/cli.py` | 287–289 | `conn._ser.reset_input_buffer()` / `write(b"HELLO\n")` / `flush()` | `conn.handshake(b"HELLO\n")` |
| `host/robot_radio/robot/cutebot.py` | 93–94 | `self._conn._ser.write()` / `flush()` | `self._conn.send_fast(cmd)` (method already exists) |
| `host/robot_radio/io/sim_conn.py` | 69 | `_ser = None` stub | remove the attribute entirely |

### Detail: protocol.py

`NezhaProtocol.read_pending_lines()` (line 267–273) peeks `_conn._ser.in_waiting`
to avoid a blocking read, then does a bulk `read()` and splits on newlines. After
ticket 001, `SerialConnection.read_pending_lines()` provides exactly this: a
non-blocking queue drain. Replace the method body with:

```python
def read_pending_lines(self) -> list[str]:
    """Drain the pending queues without blocking."""
    return self._conn.read_pending_lines()
```

### Detail: cli.py

The HELLO probe in `detect_device()` (lines 284–295) bypasses `SerialConnection`
to send a raw `b"HELLO\n"` (no relay prefix) because the relay responds to HELLO
itself and the host needs to detect it before knowing the mode. After ticket 001,
`SerialConnection.handshake(b"HELLO\n")` sends a raw line under `_write_lock`
without a relay prefix. The `reset_input_buffer()` call on line 287 should be
dropped (the reader thread must not have stale bytes cleared externally; if this
is called before the reader starts it is benign, but the intent is to probe fresh
— `handshake` sends the line cleanly).

Replace lines 287–289 with a single `conn.handshake(b"HELLO\n")` call.

### Detail: cutebot.py

`_send_and_wait_enc()` (lines 90–94) builds the relay-prefix wire string manually
and calls `_conn._ser.write()` directly, bypassing `SerialConnection.send_fast()`.
`send_fast()` already handles relay-prefix formatting, the `_write_lock`, and
the `on_send` callback. Replace lines 90–94 with:

```python
self._conn.send_fast(cmd)
```

where `cmd` is the bare command string (without prefix or newline), letting
`send_fast()` apply the prefix.

### Detail: sim_conn.py

The `_ser = None` class attribute exists only because `protocol.py` reaches for
`_conn._ser`. Once protocol.py uses `read_pending_lines()`, no external code
accesses `_conn._ser` on a `SimConnection`. Remove the class attribute.
`SimConnection.read_pending_lines()` should also be added (returns `[]`, as the
sim has no in_waiting concept).

### CI guard

Add a step to `.github/workflows/build.yml` (or a separate
`.github/workflows/host-lint.yml`) that runs:

```bash
result=$(grep -rn '_ser' host/robot_radio | grep -v 'io/serial_conn.py')
if [ -n "$result" ]; then
  echo "Transport boundary violation: _ser accessed outside io/serial_conn.py"
  echo "$result"
  exit 1
fi
```

This step must run on every PR targeting `master`.

## Acceptance Criteria

- [x] `grep -rn '_ser' host/robot_radio | grep -v io/serial_conn.py` returns
      nothing (zero matches). CI guard uses `\b_ser\b` to avoid false positives
      from `list_serial_ports` and `stdio_server` substrings.
- [x] `protocol.py:read_pending_lines()` delegates to `self._conn.read_pending_lines()`.
- [x] `cli.py` HELLO probe uses `conn.handshake(b"HELLO\n")` instead of raw `_ser`
      writes; `reset_input_buffer()` call is removed.
- [x] `cutebot.py:_send_and_wait_enc()` uses `self._conn.send_fast(cmd)`.
- [x] `sim_conn.py` has no `_ser` attribute.
- [x] `SimConnection.read_pending_lines()` exists and returns `[]`.
- [x] CI grep guard is in the workflow and fails on any `_ser` match outside
      `io/serial_conn.py`.
- [x] Existing tests pass with no regressions (protocol, CLI, cutebot behaviour
      unchanged): `uv run --with pytest python -m pytest -q tests/dev/`

## Implementation Plan

### Approach

All changes are mechanical replacements. Work file by file in this order:
1. `protocol.py` — simplest, one method body replacement.
2. `cutebot.py` — one method, two lines replaced.
3. `cli.py` — one block replaced.
4. `sim_conn.py` — remove the attribute, add `read_pending_lines()`.
5. Add CI grep guard.

### Files to modify

- `host/robot_radio/robot/protocol.py`
- `host/robot_radio/io/cli.py`
- `host/robot_radio/robot/cutebot.py`
- `host/robot_radio/io/sim_conn.py`
- `.github/workflows/build.yml` (or new `host-lint.yml`)

### Testing plan

No new test file needed — the CI grep guard is the enforcement mechanism.

Verify manually that each converted call site compiles and the existing tests
pass. The protocol and CLI have existing tests; cutebot does not have dedicated
unit tests but is exercised indirectly.

Run: `uv run --with pytest python -m pytest -q tests/dev/`

Run the grep command locally to confirm zero matches before submitting.

### Documentation updates

Update `SerialConnection` docstring to note that `_ser` is private and must
not be accessed outside `io/serial_conn.py`. Add a comment in `sim_conn.py`
explaining why `read_pending_lines()` returns an empty list (no buffered input
in sim).
