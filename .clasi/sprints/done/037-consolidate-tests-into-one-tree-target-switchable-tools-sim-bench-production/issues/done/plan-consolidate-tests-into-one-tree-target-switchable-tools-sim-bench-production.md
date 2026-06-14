---
status: done
sprint: '037'
tickets:
- 037-004
- 037-005
---

# Plan: Consolidate tests into one tree + target-switchable tools (sim / bench / production)

## Context

The repo has **three** separate test roots, which forces constant guessing about
where a test lives and which backend it talks to:
- `tests/` — firmware-logic pytest (`tests/dev/test_*.py`), real-robot bench scripts
  (`tests/bench/`), calibration (`tests/calibrate/`), a real-robot tour
  (`tests/playfield_tour/tour_goto.py`), one-off/probe scripts.
- `host_tests/` — the firmware **simulation** infrastructure (`conftest.py`,
  `firmware.py` = the `Sim` ctypes wrapper, `CMakeLists.txt`, `sim_api.cpp`),
  the maintained sim pytest suite (`host_tests/unit/`), sim+bench tour variants
  (`host_tests/playfield_tour/`), demo notebooks (`host_tests/dev/`).
- `host/tests/` — the `robot_radio` library pytest suite.

We want **one** test directory (eliminate `host_tests/`), a reusable **testkit**
of common helpers, the keep-worthy tools (`velocity_chart`, `playfield_tour`)
runnable against **three targets** via a switch, a **sim speed flag**
(full-speed vs real-time), a **real-playfield** version of the tour, and the
dead weight swept into `old/`.

The enabling insight (verified): `host/robot_radio/io/sim_conn.py`
`SimConnection` is already a drop-in for `SerialConnection`, so
`Nezha(NezhaProtocol(SimConnection()))` drives the in-process firmware sim with
the **same `Nezha` API** used on the real robot. So the target switch is a
**connection factory** that returns a connected `Nezha`; `DBG OTOS BENCH 1`
(works identically on MockHAL and NezhaHAL) supplies simulated OTOS on any
target; and because sim time is host-injected (`sim_api.cpp::sim_tick(h, now)`),
full-speed vs real-time is just whether the host paces the tick loop to the wall
clock.

Locked decisions: **(1)** single `tests/` tree, all pytest merged into
`tests/unit/`; **(2)** helpers ship in the package as **`robot_radio.testkit`**;
**(3)** `playfield_tour` is **one tool** with `--target` (and a `--pose`
override); **(4)** delivered as **one sprint**.

---

## Part A — `robot_radio.testkit` (new subpackage)
New: `host/robot_radio/testkit/{__init__,target,pose,safety,camera,dash}.py`.

### A1. The target factory — `target.py`
```python
@dataclass
class TestRobot:
    robot: Nezha
    conn: object            # SimConnection | SerialConnection
    playfield: Playfield | None
    pose: PoseSource
    target: str             # "sim" | "bench" | "production"
    real_time: bool

def make_target(target, *, real_time=False, sim_otos=None,
                port=None, camera=None, config=None) -> TestRobot
```
- **sim** → `Nezha(NezhaProtocol(SimConnection(real_time=real_time)))`; `sim_otos`
  defaults **on**. Pose source = firmware (or sim ground-truth oracle).
- **bench** → build via `robot_radio.robot.connection.make_robot(port, ...)`
  (handles direct/relay + the `!GO` handshake from sprint 036); `sim_otos`
  defaults **on** → send `DBG OTOS BENCH 1`. Pose source = firmware SNAP.
- **production** → `make_robot(...)`; `sim_otos` defaults **off** (real OTOS).
  Pose source = camera if `camera`/playfield supplied, else firmware.
- `sim_otos=None` means "use the per-target default"; pass `True/False` to force.
- Opens a `Playfield` (`robot_radio.field.playfield.Playfield.open(camera)`) when a
  camera is requested (real-playfield runs).

### A2. Pose source — `pose.py`
`PoseSource` returns `(x_cm, y_cm, yaw_rad)`:
- `FirmwarePose(robot)` — from `robot.refresh().pose` / SNAP (sim + bench).
- `CameraPose(playfield, tag_id=100)` — from `Playfield.get_tag` with circular-mean
  averaging (see A4). Used for the real playfield, where firmware pose drifts.
Test code reads pose through this one interface regardless of target.

### A3. Safety — `safety.py`
Generalize `tests/bench/bench_safety.py` `BenchRun` into `SafeRun(testrobot,
max_seconds=..., runaway=True)`: liveness preflight (PING/ID), SIGINT→STOP,
wall-clock cap, runaway detection. It accepts a `TestRobot`/`Nezha`. On `sim`,
preflight + SIGINT are no-ops (sim can't run away), but the wall-clock cap still
applies in real-time mode. Keep `tests/bench/bench_safety.py` as a thin
re-export shim so existing bench scripts keep importing it.

### A4. Camera + dashboard helpers — `camera.py`, `dash.py`
- `read_camera_pose(playfield, tag_id, n=5, timeout=4.0)` — the circular-mean
  averaging currently **duplicated** in `playfield_tour_camera.py`,
  `playfield_random_tour.py`, and `tests/playfield_tour/tour_goto.py`. Consolidate.
- `dash.py` — extract the live matplotlib multi-panel dashboard + CSV logging from
  `tests/bench/velocity_chart.py` so the tool is a thin driver over it.

---

## Part B — Sim speed flag (full-speed vs real-time)
Files: `host/robot_radio/io/sim_conn.py`, `tests/sim/firmware.py` (moved `Sim`).

Add `real_time: bool = False` (and optional `speed_factor: float = 1.0`) to
`SimConnection`. In its internal tick/advance loop (the `read_lines`/advance path
that calls `sim_tick`), after advancing sim time by `tick_step_ms`, if
`real_time` then `sleep(tick_step_ms/1000 / speed_factor)`; else run flat-out.
Mirror with `Sim.tick_for(..., real_time=False)` in `firmware.py`. **Default off**
so CI and all existing sim tests keep their current (fast) timing. Bench and
production are inherently wall-clock, so the flag is a documented no-op there.

---

## Part C — Port the two tools (target-agnostic) → `tests/tools/`
- **`velocity_chart.py`**: drive via `make_target(target, real_time=...)`; add
  `--target {sim,bench,production}` and `--real-time/--full-speed`. Keep the live
  dashboard (now in `testkit.dash`); steer via `robot.stream_drive`/`robot.vw`.
- **`playfield_tour.py`** (ONE tool): `--target {sim,bench,production}`,
  `--pose {firmware,camera}` (auto: firmware for sim/bench, camera for a real
  playfield), `--real-time/--full-speed`. One control loop modeled on the existing
  `host_tests/playfield_tour/playfield_tour_drive.py` `drive_leg` pattern, but
  driving through `Nezha.go_to(..., on_tick=cb)` and reading pose through the
  `PoseSource`. Camera runs draw the track with `Playfield.add_path` and bounds-abort
  via the `on_tick` returning `False` (the sprint-036 pattern in
  `playfield_random_tour.py`). Targets/waypoints load from
  `data/aprilcam/playfield.json` (rectangles), not hardcoded.

---

## Part D — The directory move (one tree)
Final layout:
```
tests/
  sim/        # host_tests/{CMakeLists.txt, sim_api.cpp, firmware.py, conftest.py}
  unit/       # ALL maintained pytest: host_tests/unit/ + tests/dev/test_*.py + host/tests/
  tools/      # velocity_chart.py, playfield_tour.py (ported)
  bench/      # real-robot bench scripts (square_run, four_corners, bench_safety shim, …) — stays
  calibrate/  # calibration tools — stays
  old/        # retired one-offs / probes / demo notebooks / superseded tour variants
```
**Path/config updates this move forces (do atomically):**
- Root `pyproject.toml [tool.pytest.ini_options]`: `testpaths = ["tests"]`;
  `norecursedirs` += `tests/old`, `tests/sim/build`, keep `tests/bench` excluded if
  desired. Drop `host/tests` / `host_tests` references.
- `build.py` `build_host_sim()` (just added, OOP): change `host_tests` →
  `tests/sim` (`cmake -S tests/sim -B tests/sim/build`) and the summary path.
- `tests/sim/conftest.py`: `_HOST_TESTS`/`_BUILD_DIR` → `tests/sim` paths; add
  `sys.path` entries so merged `tests/unit/` can `from firmware import Sim` and
  `from robot_radio.testkit import ...`.
- `tests/sim/CMakeLists.txt`: it derives `REPO_ROOT` as `../` from `host_tests/`;
  from `tests/sim/` that becomes `../..`. Fix the relative root.
- `host/robot_radio/io/sim_conn.py`: it dlopens `libfirmware_host` from
  `host_tests/build` — point it at `tests/sim/build`.
- Unit tests that `from firmware import Sim` keep working via conftest `sys.path`
  (no per-file edits) — verify.
- Remove `host_tests/CLAUDE.md`; update `tests/CLAUDE.md` to document the new tree.

## Part E — Weed to `old/`
Rule: maintained `test_*.py` → `tests/unit/`; active bench/calibrate tools and the
two kept tools → their dirs; everything else that is a one-off / repro / probe /
superseded demo → `tests/old/`. Representative (not exhaustive): `tests/dev/`
non-`test_*.py` scripts (`wedge_repro.py`, `hang_repro.py`, `stand_soak.py`,
`enc_watch.py`, `vel_tune.py`, `velchart_repro.py`, …), `host_tests/dev/*.ipynb`
demo notebooks, and the superseded tour variants once `playfield_tour.py` subsumes
them (`playfield_tour_drive.py`, `playfield_tour_camera.py`, `camera_drive.py`,
`playfield_random_tour.py`, `tour_goto.py` — fold their unique logic into the one
tool + testkit first, then retire).

---

## Critical files
- **Create:** `host/robot_radio/testkit/{__init__,target,pose,safety,camera,dash}.py`.
- **Modify:** `host/robot_radio/io/sim_conn.py` (real_time pacing + lib path),
  `build.py` (sim-build path), root `pyproject.toml` (pytest config).
- **Move:** `host_tests/{CMakeLists.txt,sim_api.cpp,firmware.py,conftest.py}` →
  `tests/sim/`; `host_tests/unit/*` + `tests/dev/test_*.py` + `host/tests/*` →
  `tests/unit/`; tools → `tests/tools/`; retire the rest to `tests/old/`.
- **Reuse (do not reinvent):** `bench_safety.BenchRun`, `nav/camera_goto.py`,
  `nav/pose.py` (`Pose`, `heading_error`), `robot/robot_state.py` (`RobotState`),
  `field/playfield.py` (`Playfield`), `robot/connection.py` (`make_robot`),
  `io/sim_conn.py` (`SimConnection`), the `playfield_tour_drive.py` backend pattern.

## Verification
1. `uv run --with pytest python -m pytest tests/ -q` — the merged suite (library +
   firmware-sim + firmware-logic) collects and passes from the single tree; the sim
   lib still builds (`python3 build.py --with-sim` or conftest `build_lib`).
2. **Sim, both speeds:** `playfield_tour --target sim --full-speed` completes fast;
   `--target sim --real-time` paces to wall-clock — assert a leg takes ≈ its sim
   duration (timing assertion in a `tests/unit/` test).
3. **Bench + sim-OTOS:** `velocity_chart --target bench` and `playfield_tour
   --target bench` over serial (robot on stand) — confirm `DBG OTOS BENCH 1` active
   and motion telemetry flows.
4. **Real playfield:** `playfield_tour --target production --pose camera` with the
   aprilcam daemon live — camera-localized legs draw the track via `add_path`,
   bounds-abort works.
5. Confirm `build.py` default-both still builds the sim lib at its new
   `tests/sim/build` path, and `import robot_radio` (no camera/daemon) still works
   (testkit camera/daemon imports stay lazy).

## Notes / risks
- Biggest risk is the move breaking pytest collection, the CMake `REPO_ROOT`
  globs, `build.py --with-sim`, and `sim_conn.py`'s lib path simultaneously —
  update all in one commit and gate on the full suite (verification #1).
- Interaction with the just-landed `build.py --with-sim` (OOP): its sim path moves
  with `host_tests/` → `tests/sim/`.
- `from firmware import Sim` is the most widespread import in the sim tests; keep it
  working via conftest `sys.path` rather than editing every test.
