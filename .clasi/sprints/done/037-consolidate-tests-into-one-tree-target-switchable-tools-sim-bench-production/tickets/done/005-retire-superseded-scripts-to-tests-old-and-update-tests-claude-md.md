---
id: '005'
title: Retire superseded scripts to tests/old/ and update tests/CLAUDE.md
status: done
use-cases:
- SUC-009
depends-on:
- '004'
github-issue: ''
issue: plan-consolidate-tests-into-one-tree-target-switchable-tools-sim-bench-production.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Retire superseded scripts to tests/old/ and update tests/CLAUDE.md

## Description

After T004 completes the directory move, this ticket sweeps the remaining one-off scripts, probe programs, and demo notebooks into `tests/old/` and updates the CLAUDE.md documentation to describe the final tree layout. It also removes `host_tests/CLAUDE.md` (the directory no longer exists).

This is the cleanup and documentation pass. It does not move any maintained tests (those moved in T003 and T004).

## Files to Move to `tests/old/`

From `tests/dev/` (non-`test_*.py` one-offs not already moved in T003/T004):

- `wedge_repro.py`
- `hang_repro.py`
- `stand_soak.py`
- `enc_watch.py`
- `vel_tune.py`
- `velchart_repro.py`
- any other non-`test_*.py` scripts in `tests/dev/`

From `host_tests/dev/` (if not already removed by T004):

- `*.ipynb` demo notebooks

Any remaining files in `tests/playfield_tour/` that are not the unified `playfield_tour.py` tool (if this directory still exists after T003/T004, archive it).

## Files/Directories to Remove

- `host_tests/CLAUDE.md` — directory no longer exists after T004.
- `tests/dev/` — directory should be empty after T003/T004 moved `test_*.py` and this ticket moves one-offs; remove it.
- `tests/playfield_tour/` — if empty after T003, remove it.

## `tests/CLAUDE.md` — Update

Rewrite (or create if not present) `tests/CLAUDE.md` to document the final tree layout:

```
tests/
  sim/          Firmware sim CMake infra: CMakeLists.txt, sim_api.cpp,
                firmware.py (Sim ctypes wrapper), conftest.py.
                Build: python3 build.py --with-sim → tests/sim/build/
  unit/         All maintained pytest. Run: uv run --with pytest python -m pytest tests/ -q
                Includes: robot_radio library tests, firmware-logic tests, firmware-sim tests.
                from firmware import Sim  works via tests/sim/conftest.py sys.path.
  tools/        Interactive target-switchable tools (not collected by pytest).
                velocity_chart.py, playfield_tour.py
                Run: python3 tests/tools/<tool>.py --target {sim,bench,production} [--real-time]
  bench/        Real-robot bench scripts. Not collected by pytest.
                bench_safety.py is a re-export shim → robot_radio.testkit.safety.SafeRun.
  calibrate/    Calibration tools. Not collected by pytest.
  old/          Retired one-offs, probes, demo notebooks, superseded tour variants.
                Not collected by pytest. Kept for reference.
```

Include a "How to run" section with:

1. Run all tests: `uv run --with pytest python -m pytest tests/ -q`
2. Build sim lib: `python3 build.py --with-sim`
3. Run a tool: `python3 tests/tools/playfield_tour.py --target sim --full-speed`

## Acceptance Criteria

- [ ] `tests/old/` contains all retired one-offs, probes, and demo notebooks listed above.
- [ ] `tests/dev/` directory is removed (was only for non-`test_*.py` scripts and `test_*.py` already moved in T004).
- [ ] `tests/playfield_tour/` directory is removed (if it still existed after T003).
- [ ] `host_tests/CLAUDE.md` is removed (directory gone after T004).
- [ ] `tests/CLAUDE.md` documents the final tree layout including all six subdirectories.
- [ ] `uv run --with pytest python -m pytest tests/ -q` still passes (no maintained tests accidentally moved to `tests/old/`).

## Testing Plan

**Approach**: Verify no `test_*.py` files were moved to `tests/old/` (grep). Run the full suite to confirm nothing broke.

**No new tests to write**: this ticket is a cleanup and documentation pass.

**Verification commands**:
1. `find tests/old -name "test_*.py"` — should return empty (no maintained tests in old/).
2. `uv run --with pytest python -m pytest tests/ -q` — all tests pass.
