---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 037 Use Cases

## SUC-001: Run the full test suite from one directory

- **Actor**: Developer
- **Preconditions**: Repo is checked out; sim library is built (`python3 build.py --with-sim`).
- **Main Flow**:
  1. Developer runs `uv run --with pytest python -m pytest tests/ -q`.
  2. pytest collects all maintained tests: robot_radio library tests (formerly `host/tests/`), firmware-logic unit tests (formerly `tests/dev/test_*.py`), and firmware-sim tests (formerly `host_tests/unit/`).
  3. All tests pass in a single run.
- **Postconditions**: One command runs the full maintained test suite; `host_tests/` and `host/tests/` are no longer separate roots.
- **Acceptance Criteria**:
  - [ ] `uv run --with pytest python -m pytest tests/ -q` collects and passes all previously passing tests.
  - [ ] `from firmware import Sim` works in `tests/unit/` via conftest `sys.path` without per-file edits.
  - [ ] `host_tests/` directory is removed (contents migrated or retired).

## SUC-002: Build the firmware sim library from its new location

- **Actor**: Developer / CI
- **Preconditions**: CMake and a C++ compiler are available.
- **Main Flow**:
  1. Developer (or CI) runs `python3 build.py --with-sim` or `python3 build.py` (default-both).
  2. build.py invokes `cmake -S tests/sim -B tests/sim/build` and `cmake --build tests/sim/build`.
  3. `libfirmware_host.{dylib,so}` is produced at `tests/sim/build/`.
- **Postconditions**: The sim library is at `tests/sim/build/`; `sim_conn.py` finds it there.
- **Acceptance Criteria**:
  - [ ] `python3 build.py --with-sim` succeeds and produces the lib at `tests/sim/build/`.
  - [ ] `tests/sim/CMakeLists.txt` REPO_ROOT points to `../..` (was `../`).
  - [ ] `host/robot_radio/io/sim_conn.py` dlopen path resolves to `tests/sim/build/`.

## SUC-003: Instantiate a target-appropriate robot with one call

- **Actor**: Test author / tool developer
- **Preconditions**: `robot_radio` package is installed; the sim library is built (for sim target) or a serial port is available (for bench/production).
- **Main Flow**:
  1. Caller invokes `make_target("sim")`, `make_target("bench", port="/dev/...")`, or `make_target("production", port="/dev/...", camera=daemon)`.
  2. `make_target` returns a `TestRobot` dataclass holding a connected `Nezha`, the raw connection, a `PoseSource`, the `Playfield` (or None), and target/real_time metadata.
  3. Caller uses `test_robot.robot` with the standard `Nezha` API — identical regardless of target.
- **Postconditions**: Target-specific wiring (SimConnection vs SerialConnection, sim-OTOS vs real OTOS, camera-pose vs firmware-pose) is hidden from the caller.
- **Acceptance Criteria**:
  - [ ] `make_target("sim")` constructs `Nezha(NezhaProtocol(SimConnection()))` and sets `sim_otos=True` by default.
  - [ ] `make_target("bench", ...)` builds via `make_robot(...)` and sends `DBG OTOS BENCH 1`.
  - [ ] `make_target("production", ...)` builds via `make_robot(...)` with real OTOS (no bench mode).
  - [ ] `sim_otos=True/False` overrides the per-target default on all three targets.
  - [ ] `make_target` is importable from `robot_radio.testkit`.

## SUC-004: Read pose through a uniform interface regardless of target

- **Actor**: Test/tool code
- **Preconditions**: A `TestRobot` has been constructed by `make_target`.
- **Main Flow**:
  1. Tool or test calls `test_robot.pose.read()`.
  2. For sim/bench targets, `PoseSource` delegates to firmware SNAP (`FirmwarePose`).
  3. For production with a camera, `PoseSource` delegates to the aprilcam circular-mean averager (`CameraPose`).
- **Postconditions**: Pose is returned as `(x_cm, y_cm, yaw_rad)` regardless of target; no conditional target branches in calling code.
- **Acceptance Criteria**:
  - [ ] `FirmwarePose(robot).read()` returns `(x_cm, y_cm, yaw_rad)` from `robot.refresh().pose` / SNAP.
  - [ ] `CameraPose(playfield, tag_id).read()` returns `(x_cm, y_cm, yaw_rad)` using circular-mean averaging.
  - [ ] `read_camera_pose` consolidates the three duplicate circular-mean implementations.

## SUC-005: Apply safety guardrails uniformly in tests and tools

- **Actor**: Test author / tool developer
- **Preconditions**: A `TestRobot` exists; test/tool code is about to drive the robot.
- **Main Flow**:
  1. Caller wraps a driving function in `SafeRun(test_robot, max_seconds=..., runaway=True)`.
  2. `SafeRun` performs a liveness preflight (PING/ID), installs a SIGINT handler that sends STOP, and enforces a wall-clock cap.
  3. For `sim` targets, preflight and SIGINT are no-ops; the wall-clock cap still applies when `real_time=True`.
- **Postconditions**: Safety guardrails apply consistently; the caller never writes bespoke SIGINT/preflight code.
- **Acceptance Criteria**:
  - [ ] `SafeRun` generalizes `BenchRun` from `tests/bench/bench_safety.py`.
  - [ ] `tests/bench/bench_safety.py` re-exports `SafeRun` as `BenchRun` for backward compatibility.
  - [ ] `SafeRun` works with sim targets (preflight/SIGINT are no-ops).

## SUC-006: Run the sim at full speed (CI) or paced to wall-clock (interactive)

- **Actor**: Developer / CI
- **Preconditions**: `SimConnection` is constructed.
- **Main Flow**:
  1. Default (`real_time=False`): sim runs at full CPU speed; CI and unit tests complete fast.
  2. `real_time=True` (and optional `speed_factor`): `SimConnection` sleeps `tick_step_ms / 1000 / speed_factor` after each tick step, pacing the simulation to wall-clock.
  3. `Sim.tick_for(..., real_time=False)` in `firmware.py` mirrors the same flag.
- **Postconditions**: CI timing is unchanged; interactive tool runs reflect real motion duration.
- **Acceptance Criteria**:
  - [ ] `SimConnection(real_time=True)` paces ticks to wall-clock; a 1-second sim run takes ≈1 second wall time.
  - [ ] `SimConnection(real_time=False)` (default) is not measurably slower than before.
  - [ ] `Sim.tick_for(ms, real_time=True)` in `firmware.py` delegates the flag through.
  - [ ] A unit test asserts that a `real_time=True` run of N ms takes ≥ N ms wall time.

## SUC-007: Run velocity_chart against any target

- **Actor**: Developer
- **Preconditions**: `robot_radio.testkit` is available; target-appropriate hardware/sim is ready.
- **Main Flow**:
  1. Developer runs `python3 tests/tools/velocity_chart.py --target {sim,bench,production} [--real-time]`.
  2. Tool calls `make_target(target, real_time=...)` and drives via `robot.stream_drive`/`robot.vw`.
  3. Live matplotlib multi-panel dashboard (from `testkit.dash`) updates in real time; CSV is logged.
- **Postconditions**: One tool replaces three target-specific variants; the dashboard logic lives in `testkit.dash`.
- **Acceptance Criteria**:
  - [ ] `velocity_chart --target sim` drives the sim and renders the dashboard.
  - [ ] `velocity_chart --target bench` connects to the real robot and renders the dashboard.
  - [ ] `testkit.dash` contains the extracted dashboard + CSV-logging logic.
  - [ ] The old `tests/bench/velocity_chart.py` is retired to `tests/old/`.

## SUC-008: Run the playfield tour against any target with a single tool

- **Actor**: Developer / operator
- **Preconditions**: `robot_radio.testkit` is available; target-appropriate hardware/sim is ready; `data/aprilcam/playfield.json` exists for waypoints.
- **Main Flow**:
  1. Operator runs `python3 tests/tools/playfield_tour.py --target sim --full-speed` for a fast sim run, or `--target bench --real-time`, or `--target production --pose camera`.
  2. Tool calls `make_target(...)`, constructs a `SafeRun`, reads waypoints from `playfield.json`, and drives legs via `Nezha.go_to(..., on_tick=cb)`.
  3. Pose is read through `test_robot.pose`; camera runs draw the track via `Playfield.add_path` and abort legs via `on_tick` returning `False` on bounds violation.
- **Postconditions**: One tool replaces `tour_goto.py`, `playfield_tour_drive.py`, `playfield_tour_camera.py`, `playfield_random_tour.py`; superseded variants are retired to `tests/old/`.
- **Acceptance Criteria**:
  - [ ] `playfield_tour --target sim --full-speed` completes a multi-leg tour without hardware.
  - [ ] `playfield_tour --target bench` drives the real robot with sim-OTOS active.
  - [ ] `playfield_tour --target production --pose camera` uses camera pose and draws track.
  - [ ] Waypoints load from `data/aprilcam/playfield.json`; no hardcoded targets.
  - [ ] Superseded tour variants are moved to `tests/old/`.

## SUC-009: Find any test, tool, or helper in a predictable location

- **Actor**: Developer
- **Preconditions**: Developer knows the category of what they are looking for (unit test, bench script, calibration tool, sim infra, development tool, retired one-off).
- **Main Flow**:
  1. Developer looks in the single `tests/` tree under the obvious sub-directory.
  2. `tests/unit/` for all maintained pytest; `tests/sim/` for sim CMake/firmware infra; `tests/tools/` for interactive tools; `tests/bench/` for bench scripts; `tests/calibrate/` for calibration; `tests/old/` for retired one-offs.
- **Postconditions**: No guessing between `tests/`, `host_tests/`, and `host/tests/`; `tests/CLAUDE.md` documents the layout.
- **Acceptance Criteria**:
  - [ ] `host_tests/` is removed; `host/tests/` pytest are merged into `tests/unit/`.
  - [ ] `tests/CLAUDE.md` documents the new tree layout.
  - [ ] `host_tests/CLAUDE.md` is removed.
  - [ ] Retired one-offs and superseded tour variants are in `tests/old/`.
