---
id: '003'
title: Port velocity_chart and playfield_tour to tests/tools/ using make_target
status: open
use-cases:
- SUC-007
- SUC-008
depends-on:
- '001'
- '002'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Port velocity_chart and playfield_tour to tests/tools/ using make_target

## Description

Create `tests/tools/` and write two new target-agnostic tools: `velocity_chart.py` and `playfield_tour.py`. These replace the scattered per-target variants; the old originals are retired to `tests/old/` in this ticket.

Both tools are thin drivers: they parse args, call `make_target(...)`, wrap with `SafeRun`, and delegate motion and display logic to `testkit`. No target-branching in tool code.

The live hardware verification (bench robot on stand, real playfield with camera) is deferred to team-lead.

## Source files to study (do not reinvent)

- `tests/bench/velocity_chart.py` — main logic, dashboard extraction target for `testkit.dash`
- `host_tests/playfield_tour/playfield_tour_drive.py` — `drive_leg` pattern to model the unified tour loop on
- `host_tests/playfield_tour/playfield_tour_camera.py` — camera pose + bounds abort pattern
- `host_tests/playfield_tour/playfield_random_tour.py` — waypoints from JSON + `on_tick` bounds abort
- `tests/playfield_tour/tour_goto.py` — real-robot tour (serial target)

## Files to Create

- `tests/tools/__init__.py` (empty)
- `tests/tools/velocity_chart.py` — target-switchable velocity chart tool
- `tests/tools/playfield_tour.py` — unified playfield tour tool

## Files to Retire (move to `tests/old/`)

- `tests/bench/velocity_chart.py` → `tests/old/velocity_chart_bench.py`
- `tests/playfield_tour/tour_goto.py` → `tests/old/tour_goto.py`
- `host_tests/playfield_tour/playfield_tour_drive.py` → `tests/old/playfield_tour_drive.py`
- `host_tests/playfield_tour/playfield_tour_camera.py` → `tests/old/playfield_tour_camera.py`
- `host_tests/playfield_tour/playfield_random_tour.py` → `tests/old/playfield_random_tour.py`
- `host_tests/playfield_tour/camera_drive.py` → `tests/old/camera_drive.py`
- `host_tests/playfield_tour/playfield_tour.ipynb` → `tests/old/`

Note: `tests/old/` directory should be created if it does not already exist.

## Implementation Details

### `tests/tools/velocity_chart.py`

CLI entry point: `python3 tests/tools/velocity_chart.py [--target {sim,bench,production}] [--real-time | --full-speed] [--port PORT] [--duration SECS]`

Structure:
1. Parse args.
2. Call `tr = make_target(args.target, real_time=args.real_time, port=args.port)`.
3. Wrap with `SafeRun(tr, max_seconds=args.duration + 5)`.
4. Construct `Dashboard` from `testkit.dash` with velocity panels.
5. Call `tr.robot.stream_drive(left_mm_s, right_mm_s)` in a loop; read state via SNAP/TLM; call `dashboard.update(data)` each tick; stop after duration.
6. Call `dashboard.save_csv(path)` on exit.

### `tests/tools/playfield_tour.py`

CLI entry point: `python3 tests/tools/playfield_tour.py [--target {sim,bench,production}] [--pose {firmware,camera,auto}] [--real-time | --full-speed] [--port PORT] [--camera CAMERA_ARG]`

Structure:
1. Parse args. `--pose auto` = firmware for sim/bench, camera for production.
2. Load waypoints from `data/aprilcam/playfield.json` (rectangles/named squares, not hardcoded).
3. Call `tr = make_target(args.target, real_time=args.real_time, port=args.port, camera=camera_arg_if_production)`.
4. Wrap with `SafeRun(tr)`.
5. For each waypoint leg:
   - Convert waypoint to robot-relative mm via `tr.pose.read()` + standard world→robot transform.
   - Call `tr.robot.go_to(forward_mm, left_mm, on_tick=cb)` where `cb` checks bounds via `Playfield` and returns `False` to abort if out-of-bounds.
   - For camera pose: call `tr.pose.read()` inside the `on_tick` callback to update `Playfield.add_path` track.
   - Wait for `EVT done G`.

Model the `drive_leg` pattern from `playfield_tour_drive.py`: it separates the leg setup, on_tick callback, and completion handling cleanly. The key insight from `playfield_random_tour.py` is that `on_tick` returning `False` is the bounds-abort mechanism in sprint 036's `Nezha.go_to`.

## Acceptance Criteria

- [ ] `tests/tools/` directory exists with `velocity_chart.py` and `playfield_tour.py`.
- [ ] `python3 tests/tools/velocity_chart.py --target sim --full-speed` completes without error (sim target, no hardware).
- [ ] `python3 tests/tools/playfield_tour.py --target sim --full-speed` drives the sim through at least two waypoints from `playfield.json` without error.
- [ ] `--real-time` flag is accepted and passed to `make_target`.
- [ ] Superseded tour variants are moved to `tests/old/`.
- [ ] `tests/bench/velocity_chart.py` is moved to `tests/old/velocity_chart_bench.py`.
- [ ] No target-switching branches (`if target == "sim": ...`) exist in either tool.

## Testing Plan

**Approach**: Smoke-test the tools with `--target sim --full-speed` (no hardware, no camera). Verify they produce output and exit cleanly. Functional bench/production testing is deferred to team-lead.

**New tests to write** in `tests/unit/test_tools_smoke.py`:

1. `test_velocity_chart_sim_imports` — `import tests.tools.velocity_chart` succeeds (or run via subprocess with `--help`).
2. `test_playfield_tour_sim_smoke` — run `playfield_tour.py --target sim --full-speed` as a subprocess for a short duration; assert exit code 0. Uses a monkeypatched `playfield.json` with two simple waypoints if the real file is not available.

**Existing tests to run**: `uv run --with pytest python -m pytest host_tests/unit/ -q` (ensure retirement of old files did not break imports).

**Verification command**: `uv run --with pytest python -m pytest host_tests/unit/ tests/unit/ -q`
