---
id: '003'
title: Callback-driven go_to and turn with _run_until_done loop
status: done
use-cases:
- SUC-003
- SUC-004
- SUC-005
depends-on:
- '001'
- '002'
github-issue: ''
issue: plan-stateful-robot-object-playfield-object-for-the-host-module.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Callback-driven go_to and turn with _run_until_done loop

## Description

`Nezha.go_to` (nezha.py:190) is currently blocking: it sends `G`, then calls
`wait_for_evt_done`. There is no per-tick callback and no `turn` method exposed
at all (the wire op exists at protocol.py:563 but `Nezha` does not wrap it).

This ticket converts `go_to` to optionally run a tick loop and adds `turn` as a
new public method. Both share a private `_run_until_done(verb, on_tick, timeout_s)`
loop.

**`_run_until_done` loop contract:**
1. Reads lines via `self._proto._conn.read_lines(duration_ms=50)` (the same
   primitive `wait_for_evt_done` uses internally).
2. For each line: parse it with `parse_response`; if TLM, call `_apply_tlm` then
   call `on_tick(self)`.
3. If `on_tick` returns `False` — send `X` via `self._proto._conn.send_fast("X")`,
   return `"aborted"`.
4. If `EVT done <verb>` arrives — disable stream (`STREAM 0`), return `"done"`.
5. If `EVT safety_stop` arrives — disable stream, return `"safety_stop"`.
6. If wall-clock `timeout_s` exceeded — send `X`, disable stream, return
   `"timeout"`.
7. If no telemetry arrived and keepalive interval elapsed — send `+` keepalive.
   (The `SerialConnection` keepalive daemon already does this, so this is just
   a safety belt for when the daemon is unavailable.)

**Updated `go_to` signature:**
```python
def go_to(self, x_mm, y_mm, speed_mms, on_tick=None, timeout_s=15.0) -> tuple[int,int,str]
```
- `on_tick is None` (default): preserves current behaviour — call
  `wait_for_evt_done("G", timeout_ms)` exactly as today. No STREAM enabled, no
  state update per tick. Return type unchanged.
- `on_tick` provided: enable `STREAM 80` before issuing `G`, then enter
  `_run_until_done("G", on_tick, timeout_s)`. Disable STREAM on exit.

**New `turn` method:**
```python
def turn(self, heading_cdeg, on_tick=None, eps_cdeg=None, timeout_s=10.0) -> str
```
- `on_tick is None`: send `TURN`, call `wait_for_evt_done("TURN", timeout_ms)`,
  return outcome string.
- `on_tick` provided: enable `STREAM 80`, issue `TURN`, enter `_run_until_done`.

## Acceptance Criteria

- [x] `Nezha.go_to(x, y, speed)` (no `on_tick`) produces identical behaviour to
      the pre-sprint version: sends `G`, waits for `EVT done G`, returns
      `(enc_l, enc_r, outcome)`. The existing Navigator callers are unaffected.
- [x] `Nezha.go_to(x, y, speed, on_tick=cb)` enables STREAM, issues `G`, calls
      `cb(robot)` on each TLM tick, updates `robot.state`, exits on `EVT done G`
      or `EVT safety_stop`, disables STREAM, returns `(enc_l, enc_r, outcome)`.
- [x] When `on_tick` returns `False`: `X` is sent, outcome is `"aborted"`,
      `robot.state` reflects the last TLM frame before abort.
- [x] `Nezha.turn(heading_cdeg)` (no `on_tick`) sends `TURN` and waits for
      `EVT done TURN`; returns outcome string `"done" | "safety_stop" | "timeout"`.
- [x] `Nezha.turn(heading_cdeg, on_tick=cb)` enables STREAM, issues `TURN`, calls
      `cb(robot)` per tick, exits on `EVT done TURN`; `on_tick=False` aborts.
- [x] `uv run --with pytest python -m pytest host/tests/test_robot_go_to_callback.py
      host/tests/test_nezha_drive.py -q` passes.

## Implementation Plan

### Approach

Add `_run_until_done(verb, on_tick, timeout_s)` as a private method in
`host/robot_radio/robot/nezha.py`. Modify `go_to` to branch on `on_tick`. Add
`turn` as a new method. The `NezhaProtocol.turn()` wire op already exists at
protocol.py:563; use `self._proto.turn(heading_cdeg, eps_cdeg)` directly.

`read_lines` is accessible as `self._proto._conn.read_lines` (the same attribute
path `wait_for_evt_done` uses). `send_fast` is `self._proto._conn.send_fast`.
`parse_response` is already imported from `protocol.py`.

### Files to Modify

- `host/robot_radio/robot/nezha.py` — add `_run_until_done`; modify `go_to`
  signature and body; add `turn`.

### Files to Create

- `host/tests/test_robot_go_to_callback.py`

### Testing Plan

New file `host/tests/test_robot_go_to_callback.py`. Drive via `SimConnection`
with a mock that feeds scripted lines (TLM frames, then EVT done).

1. `test_go_to_no_callback_blocking` — `go_to(x,y,speed)` with `on_tick=None`
   calls `wait_for_evt_done`; returns `(enc_l, enc_r, "done")`. Confirm STREAM
   is NOT enabled.
2. `test_go_to_callback_receives_ticks` — mock read_lines to return two TLM
   frames then `EVT done G`; assert callback called twice; `state` updated each
   time; outcome `"done"`.
3. `test_go_to_callback_abort_on_false` — mock callback returning `False` after
   first tick; assert `X` sent; outcome `"aborted"`.
4. `test_go_to_callback_safety_stop` — inject `EVT safety_stop`; assert outcome
   `"safety_stop"` and STREAM disabled.
5. `test_turn_no_callback` — `turn(9000)` sends `TURN 9000` and waits for
   `EVT done TURN`; returns `"done"`.
6. `test_turn_callback_abort` — callback returns `False`; outcome `"aborted"`.

Verification: `uv run --with pytest python -m pytest host/tests/test_robot_go_to_callback.py
host/tests/test_robot_state.py host/tests/test_nezha_drive.py -q`

### Documentation Updates

Update `Nezha.go_to` docstring with the new `on_tick` parameter semantics.
Add `Nezha.turn` docstring explaining heading convention (cdeg, CCW-positive,
same as `OTOS` convention).
