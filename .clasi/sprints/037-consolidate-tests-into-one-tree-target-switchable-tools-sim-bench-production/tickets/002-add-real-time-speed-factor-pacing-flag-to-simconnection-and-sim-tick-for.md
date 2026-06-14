---
id: '002'
title: Add real_time/speed_factor pacing flag to SimConnection and Sim.tick_for
status: open
use-cases:
- SUC-006
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add real_time/speed_factor pacing flag to SimConnection and Sim.tick_for

## Description

`SimConnection` currently runs ticks at full CPU speed with no wall-clock pacing. This is ideal for CI but means interactive tool runs finish instantly, with no sense of real motion duration.

Add `real_time: bool = False` and `speed_factor: float = 1.0` to `SimConnection.__init__`. When `real_time=True`, the `_advance` inner loop sleeps `tick_step_ms / 1000 / speed_factor` seconds after each tick, pacing the simulation to wall-clock time. Default is `False` — CI and all existing sim tests are unaffected.

Mirror the flag in `firmware.py` `Sim.tick_for` for the direct ctypes-level callers (sim pytest fixtures).

This ticket depends on T001 because `make_target` in `testkit/target.py` passes `real_time=` through to `SimConnection`. T001 must exist before `make_target` can reference this parameter.

## Files to Modify

- `host/robot_radio/io/sim_conn.py` — add `real_time: bool = False`, `speed_factor: float = 1.0` to `__init__`; add `time.sleep` in `_advance` loop.
- `host_tests/firmware.py` (before T004 moves it to `tests/sim/firmware.py`) — add `real_time: bool = False` to `Sim.tick_for`; add sleep in its tick loop.

## Implementation Details

### `SimConnection._advance` change

In the `_advance` inner `while self._t < end_t:` loop, after `self._t += dt` and the state-record line, add:

```python
if self._real_time:
    time.sleep(dt / 1000.0 / self._speed_factor)
```

Add `import time` at the module top level (not inline).

### `Sim.tick_for` change

In `host_tests/firmware.py` (or `tests/sim/firmware.py` if T004 has run), `tick_for` gains `real_time=False` and `speed_factor=1.0` parameters. After each tick step:

```python
if real_time:
    time.sleep(tick_step_ms / 1000.0 / speed_factor)
```

## Acceptance Criteria

- [ ] `SimConnection.__init__` accepts `real_time: bool = False` and `speed_factor: float = 1.0`.
- [ ] `SimConnection(real_time=False)` (default) produces no measurable slowdown vs. current behavior.
- [ ] `SimConnection(real_time=True)` paces ticks to wall-clock: a 500 ms sim run takes ≥ 490 ms wall time.
- [ ] `SimConnection(real_time=True, speed_factor=2.0)` runs at 2× real-time: a 500 ms sim run takes ≥ 240 ms wall time.
- [ ] `Sim.tick_for(ms, real_time=True)` in `firmware.py` paces ticks similarly.
- [ ] All existing sim unit tests still pass (they use `real_time=False` default).
- [ ] `make_target("sim", real_time=True).conn._real_time is True`.

## Testing Plan

**Approach**: Timing assertions use generous tolerances (wall time ≥ sim_ms × 0.95). Mark tight assertions as `pytest.mark.slow` or skip on CI via `@pytest.mark.skipif(os.environ.get("CI"), ...)`.

**New tests to write** in `tests/unit/test_sim_realtime.py`:

1. `test_simconn_default_is_fast` — `SimConnection(real_time=False)` for 200 ms sim time; assert wall time < 1.0 s.
2. `test_simconn_realtime_pacing` — `SimConnection(real_time=True)` for 200 ms sim time; assert wall time ≥ 190 ms.
3. `test_simconn_speed_factor` — `SimConnection(real_time=True, speed_factor=4.0)` for 200 ms sim time; assert wall time ≥ 45 ms.
4. `test_make_target_sim_realtime` — `make_target("sim", real_time=True).conn._real_time is True`.

**Existing tests to run**: `uv run --with pytest python -m pytest host_tests/unit/ -q`

**Verification command**: `uv run --with pytest python -m pytest host_tests/unit/ tests/unit/ -q`
