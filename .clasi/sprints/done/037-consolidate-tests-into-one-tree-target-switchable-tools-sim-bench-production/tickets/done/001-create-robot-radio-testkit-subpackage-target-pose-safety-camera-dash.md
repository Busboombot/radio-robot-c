---
id: '001'
title: Create robot_radio.testkit subpackage (target, pose, safety, camera, dash)
status: done
use-cases:
- SUC-003
- SUC-004
- SUC-005
- SUC-007
- SUC-008
depends-on: []
github-issue: ''
issue: plan-consolidate-tests-into-one-tree-target-switchable-tools-sim-bench-production.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Create robot_radio.testkit subpackage (target, pose, safety, camera, dash)

## Description

Create `host/robot_radio/testkit/` with six modules. This is the foundational ticket: all tool-porting (T003) and the directory move (T004) depend on `testkit` existing.

The testkit provides a uniform API for constructing a target-appropriate robot (`make_target`), reading pose through a single interface (`PoseSource`), applying safety guardrails (`SafeRun`), averaging camera poses (`read_camera_pose`), and rendering a live dashboard (`dash.py`). No tool or test file should contain target-switching branches after this exists.

All tests in this ticket run against `SimConnection` (no hardware required). The live-hardware verification (bench + production) is deferred to team-lead.

## Files to Create

- `host/robot_radio/testkit/__init__.py` — re-exports: `make_target`, `TestRobot`, `PoseSource`, `FirmwarePose`, `CameraPose`, `SafeRun`, `read_camera_pose`
- `host/robot_radio/testkit/target.py` — `TestRobot` dataclass, `make_target` factory
- `host/robot_radio/testkit/pose.py` — `PoseSource` protocol, `FirmwarePose`, `CameraPose`
- `host/robot_radio/testkit/safety.py` — `SafeRun` (generalized `BenchRun`)
- `host/robot_radio/testkit/camera.py` — `read_camera_pose` (circular-mean averaging)
- `host/robot_radio/testkit/dash.py` — matplotlib dashboard + CSV logging extracted from `velocity_chart.py`

## Files to Modify

- `tests/bench/bench_safety.py` — change to a one-liner re-export shim: `from robot_radio.testkit.safety import SafeRun as BenchRun`
- `host/robot_radio/__init__.py` (if it exists) — ensure `testkit` is importable without a live daemon (all camera/aprilcam imports in `camera.py` and `CameraPose` in `pose.py` must be lazy/guarded)

## Implementation Details

### `target.py` — `make_target`

```
def make_target(target, *, real_time=False, sim_otos=None,
                port=None, camera=None, config=None) -> TestRobot
```

- **sim**: Construct `SimConnection(real_time=real_time)`; call `connect()`; construct `Nezha(NezhaProtocol(conn))`; `sim_otos` defaults `True` → send `DBG OTOS BENCH 1`; pose = `FirmwarePose(robot)`.
- **bench**: Call `make_robot(port, ...)` from `robot_radio.robot.connection`; `sim_otos` defaults `True` → send `DBG OTOS BENCH 1` after connection; pose = `FirmwarePose(robot)`.
- **production**: Call `make_robot(port, ...)`; `sim_otos` defaults `False`; if `camera` supplied open `Playfield` and set pose = `CameraPose(playfield, tag_id=100)`; else pose = `FirmwarePose(robot)`.
- `sim_otos=True/False` overrides per-target default on all three paths.
- `make_robot` call for bench/production: use a minimal `args` namespace (`args.port = port`). Do not replicate `make_robot`'s argparse logic.

`TestRobot` dataclass fields: `robot: Nezha`, `conn`, `playfield: Playfield | None`, `pose: PoseSource`, `target: str`, `real_time: bool`.

### `pose.py`

- `PoseSource` — protocol with `def read(self) -> tuple[float, float, float]` returning `(x_cm, y_cm, yaw_rad)`.
- `FirmwarePose(robot)` — calls `robot.refresh()` then reads from the SNAP/TLM state. Coordinate units: convert from firmware mm to cm for the return value (or match what `navigator.py` and camera tools already use — check `robot_state.py` for the field units and return consistently).
- `CameraPose(playfield, tag_id=100)` — calls `read_camera_pose(playfield, tag_id)` and returns the result. This is a thin wrapper.

### `camera.py` — `read_camera_pose`

Extract and consolidate the circular-mean averaging logic from the three duplicate sites:
- `host_tests/playfield_tour/playfield_tour_camera.py`
- `host_tests/playfield_tour/playfield_random_tour.py`
- `tests/playfield_tour/tour_goto.py`

The consolidated implementation should match the best of the three (n=5 readings, timeout=4.0 s, circular mean for yaw via `scipy.stats.circmean` or equivalent). Make `n` and `timeout` parameters with those defaults.

### `safety.py` — `SafeRun`

Generalize `tests/bench/bench_safety.py` `BenchRun`:
- Constructor: `SafeRun(testrobot: TestRobot | Nezha, max_seconds=60, runaway=True)`
- `preflight()` — for sim targets (detected from `testrobot.target == "sim"` or by checking connection type), this is a no-op. For hardware targets, sends PING, checks ID.
- SIGINT handler → send `robot.stop()`.
- Wall-clock cap: raises or calls `robot.stop()` after `max_seconds`.
- Runaway detection (if `runaway=True`): check speed against a reasonable max; call `robot.stop()` if exceeded.
- Context manager (`__enter__`/`__exit__`) for use in `with SafeRun(...):` blocks.

Keep `tests/bench/bench_safety.py` as a shim: `from robot_radio.testkit.safety import SafeRun as BenchRun`.

### `dash.py` — Dashboard

Extract from `tests/bench/velocity_chart.py`:
- `Dashboard(title, fields)` class with `update(data_dict)` and `save_csv(path)` methods.
- Live matplotlib multi-panel layout (one panel per field). Uses `plt.pause()` for interactivity.
- `fields` is a list of `(panel_title, y_label, [series_name, ...])` or equivalent simple spec.
- Do not hardcode velocity-chart-specific field names; keep it generic so `playfield_tour.py` can also use it.

## Acceptance Criteria

- [x] `from robot_radio.testkit import make_target, TestRobot, SafeRun, PoseSource, FirmwarePose, CameraPose, read_camera_pose` succeeds with no daemon running.
- [x] `make_target("sim")` returns a `TestRobot` with `target="sim"`, a connected `Nezha` backed by `SimConnection`, and `pose` as `FirmwarePose`.
- [x] `make_target("sim", sim_otos=True)` sends `DBG OTOS BENCH 1` to the sim (verify via `conn.state_log` or command echo).
- [x] `make_target("sim", sim_otos=False)` does not send `DBG OTOS BENCH 1`.
- [x] `FirmwarePose(robot).read()` returns a 3-tuple `(x_cm, y_cm, yaw_rad)` from a live sim.
- [x] `read_camera_pose` is importable but its body may raise if no camera daemon is present (lazy import guard).
- [x] `SafeRun` with a sim `TestRobot` does not raise during construction or `preflight()`.
- [x] `SafeRun` used as a context manager calls `robot.stop()` on exit.
- [x] `tests/bench/bench_safety.py` re-exports `BenchRun = SafeRun`; existing `from bench_safety import BenchRun` still works.
- [x] `from robot_radio.testkit.dash import Dashboard` succeeds.

## Testing Plan

**Approach**: Write `tests/unit/test_testkit.py` (or `test_testkit_target.py`, `test_testkit_safety.py` — split if files grow large). All tests use `SimConnection` (sim target); no hardware required.

**New tests to write** in `tests/unit/` (or `host_tests/unit/` until T004 moves them):

1. `test_make_target_sim_returns_testrobot` — `make_target("sim")` returns `TestRobot` with correct fields.
2. `test_make_target_sim_otos_on_by_default` — verify `DBG OTOS BENCH 1` is sent when `sim_otos` defaults.
3. `test_make_target_sim_otos_override_false` — verify bench command is NOT sent with `sim_otos=False`.
4. `test_firmware_pose_read` — construct sim `TestRobot`, tick a few steps, call `FirmwarePose.read()`, assert returns a 3-tuple of floats.
5. `test_safe_run_sim_no_preflight_error` — `SafeRun` with sim target completes construction without raising.
6. `test_safe_run_context_manager` — `with SafeRun(tr):` block exits cleanly.
7. `test_bench_safety_shim` — `from robot_radio.testkit.safety import SafeRun as BenchRun` works; so does `from tests.bench.bench_safety import BenchRun` (path may need adjustment post-T004).
8. `test_dashboard_update` — construct `Dashboard(...)`, call `update(...)` without a display (use matplotlib non-interactive backend `matplotlib.use("Agg")`).

**Existing tests to run**: `uv run --with pytest python -m pytest host_tests/unit/ host/tests/ -q` (pre-move; these should still pass).

**Verification command**: `uv run --with pytest python -m pytest host_tests/unit/ host/tests/ tests/unit/ -q` (adjust paths per T004 merge state).
