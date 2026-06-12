---
id: '001'
title: "Reader thread demux — stop clearing input buffer on send"
status: open
use-cases:
  - SUC-001
depends-on: []
github-issue: ''
issue: d11a-serial-conn-stops-eating-input-buffer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Reader thread demux — stop clearing input buffer on send

## Description

`SerialConnection.send()` calls `self._ser.reset_input_buffer()` before every
write, discarding all buffered-but-unread bytes — TLM frames, EVT done lines,
EVT safety_stop events. Any command that uses `send()` (SNAP, SET, GET, status
polls) randomly punches holes in the stream. This is a first-order cause of
`wait_for_evt_done()` blocking forever even when firmware did emit the event.

Replace the read-after-write pattern with a single background reader thread
that owns all `_ser.readline()` calls and demultiplexes incoming lines into
three queues:

- `_reply_queues` — keyed by corr-id, for synchronous OK/ERR/CFG responses.
- `_tlm_queue` — for TLM frames consumed by `read_lines()`.
- `_evt_queue` — for EVT lines consumed by `read_lines()` and
  `wait_for_evt_done()`.

`send()` writes the command and blocks on the appropriate reply queue instead
of reading from the port directly. `read_lines()` drains `_tlm_queue` and
`_evt_queue` without touching `_ser`.

## Scope

File to modify: `host/robot_radio/io/serial_conn.py`

### Internal additions to SerialConnection

Add to `__init__`:
- `self._reply_queues: dict[str, queue.Queue]` — one queue per in-flight
  corr-id, created before write and deleted after reply.
- `self._tlm_queue: queue.Queue` — bounded (256 frames default).
- `self._evt_queue: queue.Queue` — unbounded.
- `self._reader_thread: threading.Thread | None = None`
- `self._reader_stop: threading.Event`
- `self._corr_counter: int = 0` — monotonically incrementing corr-id source.
- `self._reply_lock: threading.Lock` — guards `_reply_queues` dict mutations.

Add `_reader_loop()`:
- Loops on `_ser.readline()` while not stopped and port is open.
- Classifies each decoded, stripped line:
  - Starts with `TLM` → `_tlm_queue.put_nowait()` (drop oldest on full).
  - Starts with `EVT` → `_evt_queue.put()`.
  - Contains `#<id>` suffix and starts with `OK`/`ERR`/`CFG` → extract id,
    route to `_reply_queues[id]` if present.
  - `OK`/`ERR`/`CFG` with no corr-id → route to `_reply_queues[""]` catch-all.
  - Contains `keepalive` → drop silently.
- On any exception (port closed, decode error) → break and exit silently.

Start the reader thread in `connect()` after `_poll_ready` returns and
`start_keepalive()` is called. Stop it in `disconnect()` before closing `_ser`,
mirroring the keepalive thread lifecycle.

Modify `send()`:
- Remove `self._ser.reset_input_buffer()`.
- Increment `_corr_counter`; form `corr_id = str(_corr_counter)`.
- Append ` #<corr_id>` to the command string before encoding.
- Under `_reply_lock`, create `self._reply_queues[corr_id] = queue.Queue()`.
- Write and flush under `_write_lock` as before.
- Block on `_reply_queues[corr_id].get(timeout=read_ms/1000 + 0.5)`.
- Clean up the queue entry from `_reply_queues` under `_reply_lock`.
- Collect additional lines for `stop_token` matching from the reply (OK is
  typically one line; the stop_token logic can remain but drains the queue
  instead of the port).

Modify `read_lines()`:
- Replace `_ser.readline()` loop with repeated `get_nowait()` from both
  `_tlm_queue` and `_evt_queue`, sleeping 5 ms between drain attempts until
  the deadline.

Add `read_pending_lines() -> list[str]`:
- Non-blocking drain of `_tlm_queue` and `_evt_queue` via `get_nowait()`.
- Returns immediately with whatever is queued (may be empty).
- Required by ticket 002: replaces the `_conn._ser.in_waiting` peek in
  `protocol.py`.

Add `handshake(line: bytes) -> None`:
- Writes `line` directly to `_ser` under `_write_lock`, no relay prefix, no
  corr-id.
- Valid only before the reader thread starts (device detection phase in
  `cli.py`). Document this constraint in the docstring.
- Required by ticket 002: replaces raw `_ser.write(b"HELLO\n")` in `cli.py`.

### Intentional remaining internal _ser accesses (not bypass points)

- `_poll_ready` — `reset_input_buffer()`, `write()`, `readline()` before
  reader starts.
- `_keepalive_loop` — `write()` / `flush()` under `_write_lock`.
- `handshake()` — `write()` / `flush()` under `_write_lock`, pre-reader.
- `_reader_loop` — sole owner of `readline()`.

## Acceptance Criteria

- [ ] `send()` contains no `reset_input_buffer()` call.
- [ ] A reader thread is started by `connect()` after `_poll_ready` returns and
      stopped by `disconnect()`.
- [ ] `read_lines()` reads from `_tlm_queue` and `_evt_queue`; does not call
      `_ser.readline()`.
- [ ] `read_pending_lines()` exists and performs a non-blocking drain of both
      queues.
- [ ] `handshake(line: bytes)` exists and writes a raw line under `_write_lock`.
- [ ] Reader correctly routes TLM to `_tlm_queue`, EVT to `_evt_queue`, OK/ERR
      to the corr-id keyed queue, keepalive acks to nowhere.
- [ ] Concurrent `send()` calls do not receive each other's replies (each has
      its own queue entry keyed by corr-id).
- [ ] Unit tests in `tests/dev/test_serial_conn_reader.py` all pass.
- [ ] All existing tests pass: `uv run --with pytest python -m pytest -q tests/dev/`

## Implementation Plan

### Approach

Add the three queues and reader thread infrastructure first. Switch `send()`
to use the reply queue. Switch `read_lines()` to drain the TLM/EVT queues.
Add `read_pending_lines()` and `handshake()`. Keep all public method signatures
unchanged.

### Files to create

- `tests/dev/test_serial_conn_reader.py` — unit tests for reader thread routing.
  Uses a mock/stub serial object (feed bytes via a queue or StringIO-like object);
  no real serial port needed.

### Files to modify

- `host/robot_radio/io/serial_conn.py`

### Testing plan

New tests in `tests/dev/test_serial_conn_reader.py`:

1. `test_tlm_line_routed_to_tlm_queue` — inject `b"TLM t=1 mode=I enc=0,0\n"`;
   verify `_tlm_queue` receives the line, `_evt_queue` is empty.
2. `test_evt_line_routed_to_evt_queue` — inject `b"EVT done T #3\n"`; verify
   `_evt_queue` receives it.
3. `test_ok_with_corr_id_routed_to_reply_queue` — inject `b"OK pong t=0 #7\n"`;
   verify reply queue keyed `"7"` receives it.
4. `test_ok_no_corr_id_goes_to_catchall` — inject `b"OK pong\n"`; verify `""`
   catch-all queue receives it.
5. `test_keepalive_ack_dropped` — inject `b"OK keepalive\n"`; verify nothing
   is queued anywhere.
6. `test_read_lines_drains_tlm_and_evt` — pre-fill `_tlm_queue` and `_evt_queue`
   with several lines; call `read_lines(100)`; verify all lines returned.
7. `test_read_pending_lines_non_blocking` — pre-fill queues; call
   `read_pending_lines()`; verify immediate return with queued lines.

Regression: `uv run --with pytest python -m pytest -q tests/dev/`

### Documentation updates

Update `SerialConnection` class docstring to describe the reader thread design.
Update `send()`, `read_lines()`, `read_pending_lines()`, `handshake()` docstrings.
