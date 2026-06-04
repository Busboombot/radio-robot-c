---
id: '003'
title: Rewrite Nezha high-level driver for v2 and add tests
status: done
use-cases:
- SUC-002
- SUC-003
- SUC-004
depends-on:
- '002'
github-issue: ''
issue: ''
completes_issue: false
---

# Rewrite Nezha high-level driver for v2 and add tests

## Description

`host/robot_radio/robot/nezha.py` is the high-level robot driver that callers interact with. It currently speaks a mix of v1 and partially-ported v2. This ticket rewrites it fully for v2:

- `connect()`: runs liveness preflight (`ping()` + `get_id()`) before any other operation; raises clearly if the robot is silent.
- `speed_for_time(spd, ms)`: issues `protocol.timed(l, r, ms)` then `wait_for_evt_done("T")` — blocking, no watchdog.
- `speed_for_distance(spd, mm)`: hop loop using `protocol.distance(l, r, hop_mm)` + `wait_for_evt_done("D")` for each hop; total distance accounting unchanged.
- `go_to(x, y, spd)`: `protocol.go_to(x, y, spd)` + `wait_for_evt_done("G")`.
- `stream_drive(l, r)`: sends `S l r` and schedules keepalive resends at ≤30 % of `sTimeoutMs` (default 500 ms → resend every ≤150 ms).
- `stop()`: `protocol.stop()`.
- `speed(l, r)` / `drive(...)` generator: preserved; uses `protocol.drive(l, r)` (v2 `S l r`).
- OTOS helpers: delegate to `protocol.otos_*`.
- Port helper: `set_port(p, v)` → `protocol.port(p, v)`.
- `NezhaState` is updated from each received `TLMFrame`; heading stored in radians.

Also update `robot/nezha_state.py` to consume `TLMFrame` fields directly (enc, pose, vel, line, color).
Confirm `robot/robot.py` imports and exposes the updated `Nezha` without error.

## Acceptance Criteria

- [x] `connect()` sends `PING` then `ID`; raises `RobotNotFoundError` (or equivalent) if either times out.
- [x] `speed_for_time(spd, ms)` sends `T l r ms` (v2 format) and blocks until `EVT done T`.
- [x] `speed_for_distance(spd, mm)` uses a hop loop with `D l r hop_mm` + `wait_for_evt_done("D")` for each hop.
- [x] `go_to(x, y, spd)` sends `G x y spd` and blocks until `EVT done G`.
- [x] `stream_drive(l, r)` resends `S l r` at ≤150 ms intervals (30 % of 500 ms watchdog).
- [x] `stop()` sends `STOP`.
- [x] `NezhaState.encoders`, `.otos_pose`, `.heading_rad` are updated from `TLMFrame` fields.
- [x] Heading is stored as radians (centidegrees from TLM are converted: `cdeg / 18000.0 * math.pi`).
- [x] No v1 command strings in `nezha.py` (`S+`, `T+`, `D+`, sign-prefix formatting).
- [x] `uv run --with pytest python -m pytest host/tests` — all tests pass.

## Implementation Plan

**Approach**: Read `nezha.py` fully. Rewrite each method referencing the architecture-update §3 table. Mock `SerialConnection` via `unittest.mock.MagicMock` for unit tests.

**Files to modify**:
- `host/robot_radio/robot/nezha.py` — full rewrite for v2.
- `host/robot_radio/robot/nezha_state.py` — update to consume `TLMFrame`.
- `host/robot_radio/robot/robot.py` — confirm import compat; fix if broken.

**Files to create**:
- `host/tests/test_nezha_drive.py` — unit tests with mock `SerialConnection`.
- `host/tests/test_stream_keepalive.py` — keepalive timing test.

**New test cases**:

`test_nezha_drive.py`:
- `test_connect_preflight_success` — mock returns `OK pong` + `ID`; connect succeeds.
- `test_connect_preflight_timeout` — mock returns nothing; connect raises.
- `test_speed_for_time_sends_T` — assert `T` command sent; mock returns `EVT done T`; assert returns.
- `test_speed_for_time_safety_stop` — mock returns `EVT safety_stop`; assert raises.
- `test_speed_for_distance_hops` — mock returns `EVT done D` per hop; assert correct hop count.
- `test_go_to_sends_G` — assert `G` command; mock returns `EVT done G`; assert returns.
- `test_stop_sends_STOP` — assert `STOP\n` sent.
- `test_state_updated_from_tlm` — send mock `TLMFrame`; assert `state.encoders` and `state.heading_rad` updated.

`test_stream_keepalive.py`:
- `test_keepalive_interval` — start `stream_drive`; record timestamps of `S` writes; assert max interval ≤150 ms over 500 ms observation window.

**Testing plan**: Run `uv run --with pytest python -m pytest host/tests -v` after changes. All 44 existing tests must remain green.
