---
id: '037'
title: Consolidate tests into one tree; target-switchable tools (sim/bench/production)
status: ticketing
branch: sprint/037-consolidate-tests-into-one-tree-target-switchable-tools-sim-bench-production
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
- SUC-009
issues:
- plan-consolidate-tests-into-one-tree-target-switchable-tools-sim-bench-production.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 037: Consolidate tests into one tree; target-switchable tools (sim/bench/production)

## Goals

1. Replace the three test roots (`tests/`, `host_tests/`, `host/tests/`) with a single `tests/` tree so one command runs all maintained tests.
2. Add a `robot_radio.testkit` subpackage providing `make_target`, `PoseSource`, `SafeRun`, `read_camera_pose`, and a live dashboard — the shared foundation for all tools and tests.
3. Add a `real_time`/`speed_factor` pacing flag to `SimConnection` so tools can run at human-observable speed without slowing CI.
4. Port `velocity_chart` and `playfield_tour` as single target-agnostic tools (`--target sim/bench/production`) in a new `tests/tools/` directory.
5. Sweep superseded one-offs, probes, and demo notebooks into `tests/old/`; update `tests/CLAUDE.md`.

## Problem

Three separate test roots force constant guessing about where a test lives and what backend it targets. The shared helpers (`BenchRun`, circular-mean camera averaging, the matplotlib dashboard) are duplicated across files. Tools are target-specific. Adding a test requires choosing a root without clear rules.

## Solution

A connection factory (`make_target`) unifies all three targets behind one `Nezha` API — feasible because `SimConnection` is already a drop-in for `SerialConnection` (verified sprint 036). The target switch is a construction-time concern; test and tool code stays target-agnostic. The directory move is sequenced last (after testkit and tools are written and green) so the highest-risk step is gated on a passing suite.

## Success Criteria

- `uv run --with pytest python -m pytest tests/ -q` collects and passes all maintained tests from one tree.
- `python3 tests/tools/playfield_tour.py --target sim --full-speed` completes a multi-leg tour without hardware.
- `python3 tests/tools/velocity_chart.py --target sim --full-speed` drives the sim and renders the dashboard.
- `from robot_radio.testkit import make_target` works with no daemon running.
- `host_tests/` and `host/tests/` directories are removed.

## Scope

### In Scope

- `robot_radio.testkit` subpackage: `target.py`, `pose.py`, `safety.py`, `camera.py`, `dash.py`.
- `real_time`/`speed_factor` on `SimConnection` and `Sim.tick_for`.
- `tests/tools/velocity_chart.py` and `tests/tools/playfield_tour.py` (target-agnostic).
- Directory move: `host_tests/{conftest,firmware,CMakeLists,sim_api}` → `tests/sim/`; all maintained pytest → `tests/unit/`.
- Atomic path updates: `pyproject.toml`, `build.py`, `tests/sim/conftest.py`, `tests/sim/CMakeLists.txt` (REPO_ROOT), `sim_conn.py` dlopen path.
- `tests/bench/bench_safety.py` shim re-export.
- Retirement of one-offs to `tests/old/`; updated `tests/CLAUDE.md`.

### Out of Scope

- Live hardware verification (bench robot on stand, real playfield with camera) — deferred to team-lead post-sprint.
- Firmware changes.
- Changes to `rogo` CLI or MCP tools.
- New navigation features.

## Test Strategy

- All new testkit modules have unit tests using `SimConnection` (no hardware required).
- `real_time` flag has timing-assertion tests with generous tolerances; tight assertions marked `pytest.mark.slow`.
- Tools are smoke-tested with `--target sim --full-speed` in subprocess tests.
- The directory-move ticket gates on `pytest tests/ -q` passing with the same count as pre-move.
- Live hardware verification (bench + production targets) is a deferred team-lead gate.

## Architecture Notes

- Key decision: `make_target` is a connection factory in `testkit` — all target-specific wiring in one place; tools contain no target branches.
- `real_time` flag lives in `SimConnection._advance` (single tick path), not in callers.
- Camera/daemon imports are lazy in `testkit`; `import robot_radio.testkit` never fails without a daemon.
- Sequencing: tools and testkit land first (T001–T003), directory move lands last (T004), cleanup last (T005). Highest-risk step is gated on green.
- Architecture review: APPROVED — no circular deps, all modules cohesive, migration risks mitigated.

## GitHub Issues

None.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Create robot_radio.testkit subpackage (target, pose, safety, camera, dash) | — |
| 002 | Add real_time/speed_factor pacing flag to SimConnection and Sim.tick_for | 001 |
| 003 | Port velocity_chart and playfield_tour to tests/tools/ using make_target | 001, 002 |
| 004 | Atomic directory move: merge all pytest into tests/unit/, move sim infra to tests/sim/, update all paths | 001, 002, 003 |
| 005 | Retire superseded scripts to tests/old/ and update tests/CLAUDE.md | 004 |

Tickets execute serially in the order listed.
