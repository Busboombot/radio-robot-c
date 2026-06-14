---
id: '004'
title: 'Atomic directory move: merge all pytest into tests/unit/, move sim infra to tests/sim/, update all paths'
status: open
use-cases:
- SUC-001
- SUC-002
- SUC-009
depends-on:
- '001'
- '002'
- '003'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Atomic directory move: merge all pytest into tests/unit/, move sim infra to tests/sim/, update all paths

## Description

This is the highest-risk ticket: it rearranges the repository test tree so that a single `uv run --with pytest python -m pytest tests/ -q` runs all maintained tests. Five interdependent path references must all be updated in one commit; a partial update breaks the suite.

This ticket is gated on T001, T002, and T003 completing successfully (testkit written, real_time flag added, tools ported). At the time of this ticket, all maintained tests must be green before the move begins. After the move, `pytest tests/ -q` must produce the same green result.

## The Five Atomic Changes (must land in one commit)

All five must be correct simultaneously. Verify each before committing, then run the suite.

### 1. Root `pyproject.toml` — pytest config

Update `[tool.pytest.ini_options]`:

```toml
testpaths = ["tests"]
norecursedirs = [
    "tests/old",
    "tests/sim/build",
    "tests/bench",
    "tests/calibrate",
    "tests/tools",
    ".git",
    "__pycache__",
    "*.egg-info",
    "build",
    "dist",
]
```

Drop any references to `host/tests` or `host_tests`.

### 2. `build.py` — `build_host_sim()` path

Change:
```python
cmake -S host_tests -B host_tests/build
cmake --build host_tests/build
```
To:
```python
cmake -S tests/sim -B tests/sim/build
cmake --build tests/sim/build
```

Update the summary/progress message path to match.

### 3. `tests/sim/conftest.py` — paths and sys.path

After moving `host_tests/conftest.py` to `tests/sim/conftest.py`, update:
- `_HOST_TESTS` (or equivalent) → path to `tests/sim/`
- `_BUILD_DIR` → path to `tests/sim/build/`
- `sys.path` inserts: ensure both `tests/sim/` (for `from firmware import Sim`) and the repo root `host/` (for `from robot_radio.testkit import ...`) are on the path.

The conftest fixture `build_lib` (or whatever it is named) that builds the sim library if needed must point to `tests/sim/build/`.

### 4. `tests/sim/CMakeLists.txt` — REPO_ROOT

After moving `host_tests/CMakeLists.txt` to `tests/sim/CMakeLists.txt`, the `REPO_ROOT` variable currently uses `../` (one level up from `host_tests/`). From `tests/sim/` it must be `../..`:

Find the line like:
```cmake
set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
```
Change to:
```cmake
set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
```

Verify by checking that `${REPO_ROOT}/source/` resolves correctly.

### 5. `host/robot_radio/io/sim_conn.py` — dlopen path

Line 37 currently:
```python
_DEFAULT_LIB = (_HERE / "../../../host_tests/build" / _LIB_NAME).resolve()
```

Change to:
```python
_DEFAULT_LIB = (_HERE / "../../../tests/sim/build" / _LIB_NAME).resolve()
```

(`_HERE` = `host/robot_radio/io/`, so `../../../` = repo root, then `tests/sim/build/`.)

## File Moves

### Sim infra: `host_tests/` → `tests/sim/`

- `host_tests/CMakeLists.txt` → `tests/sim/CMakeLists.txt` (update REPO_ROOT, see above)
- `host_tests/sim_api.cpp` → `tests/sim/sim_api.cpp` (no content change)
- `host_tests/firmware.py` → `tests/sim/firmware.py` (content already updated in T002)
- `host_tests/conftest.py` → `tests/sim/conftest.py` (update paths, see above)

### Unit tests: three roots → `tests/unit/`

- `host_tests/unit/test_*.py` → `tests/unit/` (all files)
- `tests/dev/test_*.py` → `tests/unit/` (all files)
- `host/tests/test_*.py` (and any `conftest.py`) → `tests/unit/`

For `host/tests/conftest.py` (if it exists): merge its fixtures into `tests/unit/conftest.py` or `tests/sim/conftest.py` as appropriate.

### Remove `host_tests/` directory

After all moves, `host_tests/` should be empty (only `__pycache__`, `build/`, and the items moved to `tests/old/` in T003 remain). Remove the directory entirely.

### Remove `host/tests/` directory (or leave as empty stub)

After moving all tests to `tests/unit/`, remove `host/tests/`.

## Verification Checklist (run before committing)

1. `python3 build.py --with-sim` — builds `tests/sim/build/libfirmware_host.{dylib,so}`.
2. `python3 -c "from robot_radio.io.sim_conn import SimConnection; c = SimConnection(); print(c._lib_path)"` — lib path ends with `tests/sim/build/`.
3. `uv run --with pytest python -m pytest tests/ -q` — all previously passing tests collected and pass.
4. `from firmware import Sim` works inside `tests/unit/` (verified by the sim pytest suite).
5. `from robot_radio.testkit import make_target` works in a fresh Python session.

## Acceptance Criteria

- [ ] `uv run --with pytest python -m pytest tests/ -q` collects and passes all tests that passed before the move.
- [ ] `python3 build.py --with-sim` produces `tests/sim/build/libfirmware_host.*`.
- [ ] `host/robot_radio/io/sim_conn.py` `_DEFAULT_LIB` resolves to `tests/sim/build/`.
- [ ] `tests/sim/CMakeLists.txt` REPO_ROOT resolves to the repo root (verified by `cmake -S tests/sim -B tests/sim/build` succeeding).
- [ ] `from firmware import Sim` works in `tests/unit/` tests via conftest `sys.path` with no per-file changes.
- [ ] `host_tests/` directory is removed.
- [ ] `host/tests/` directory is removed.
- [ ] Root `pyproject.toml` `testpaths` contains only `["tests"]`.

## Testing Plan

**Approach**: The test of this ticket IS the test suite. Before committing, run `pytest tests/ -q` and confirm the count matches what was collected before the move (pre-move: run `pytest host_tests/unit/ host/tests/ tests/dev/ -q --collect-only` and count; post-move: `pytest tests/ -q --collect-only` must show the same count).

**No new tests to write**: this ticket moves files and updates config; it does not add logic.

**Verification command**: `uv run --with pytest python -m pytest tests/ -q`
