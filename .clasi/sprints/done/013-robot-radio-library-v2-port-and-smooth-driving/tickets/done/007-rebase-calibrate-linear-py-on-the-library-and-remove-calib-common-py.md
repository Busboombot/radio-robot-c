---
id: '007'
title: Rebase calibrate_linear.py on the library and remove calib_common.py
status: done
use-cases:
- SUC-007
depends-on:
- '003'
- '004'
github-issue: ''
issue: ''
completes_issue: false
---

# Rebase calibrate_linear.py on the library and remove calib_common.py

## Description

`tests/calibrate/calibrate_linear.py` currently hand-rolls raw serial via `tests/calibrate/calib_common.py`. This is the only remaining raw-serial code outside the library. This ticket rewrites `calibrate_linear.py` to use only library abstractions and deletes `calib_common.py`.

The existing calibration behavior is preserved:
- Per trial: laser (port 4) activated via `protocol.port(4, 1)`; blocking `D` drive issued via `nezha.speed_for_distance(spd, mm)` (or directly via `protocol.distance`); camera (AprilTag 100) and OTOS/encoder distances read via `CamTracker`/`OdomTracker`; stakeholder enters tape-measure ground truth.
- Calibration math: compute updated `mm_per_deg_l`/`mm_per_deg_r` from encoder vs. tape; compute updated `otos_linear_scale` from OTOS vs. tape. Push to robot via `protocol.set_param("ml", ...)`, `protocol.set_param("mr", ...)`, `protocol.otos_set_linear_scalar(...)`.
- Write back: `RobotConfig` updated and `data/robots/tovez.json` written on exit.
- CLI: `--no-write` flag, `--distance` flag, `'q'` to quit loop — all preserved.

`calib_common.py` is deleted (git remove). Any README in `tests/calibrate/` that references it is updated.

## Acceptance Criteria

- [x] `tests/calibrate/calibrate_linear.py` imports no raw serial module (no `serial`, `SerialConnection` directly, no `calib_common`).
- [x] All robot calls in the script go through `Nezha` or `NezhaProtocol`.
- [x] `tests/calibrate/calib_common.py` does not exist (deleted, git removed).
- [x] Laser activation uses `protocol.port(4, 1)` / `protocol.port(4, 0)`.
- [x] Drive uses `protocol.distance(l, r, mm)` + `protocol.wait_for_evt_done("D")` (or equivalent `Nezha` method).
- [x] Calibration push uses `protocol.set_param("ml", ...)`, `protocol.set_param("mr", ...)`, `protocol.otos_set_linear_scalar(n)`.
- [x] `data/robots/tovez.json` write path uses `RobotConfig` from `host/robot_radio/config/robot_config.py`.
- [x] `uv run --with pytest python -m pytest host/tests` — all tests pass.
- [x] New test `host/tests/test_calibrate_linear.py` covers: calibration math, JSON write path, no raw serial imports.

## Implementation Plan

**Approach**: Read `calibrate_linear.py` and `calib_common.py` fully. Identify every raw serial call in `calib_common.py`. Replace each with the library equivalent. Delete `calib_common.py`. Add unit test.

**Files to modify**:
- `tests/calibrate/calibrate_linear.py` — rewrite to library-only.

**Files to delete**:
- `tests/calibrate/calib_common.py` — `git rm`.

**Files to create**:
- `host/tests/test_calibrate_linear.py` — unit test with mocked `Nezha` + `CamTracker`.

**New test cases in `test_calibrate_linear.py`**:
- `test_calibration_math_mm_per_deg` — given tape=900, encoder=880, current mm_per_deg=0.484; assert updated value is closer to 0.484 × (900/880).
- `test_calibration_math_otos_scale` — given tape=900, otos_reading=945, current scale=1.05; assert updated scalar moves toward correction.
- `test_json_write` — mock `RobotConfig.write()`; run calibration exit path; assert JSON write called with updated values.
- `test_no_raw_serial` — import `calibrate_linear`; confirm `serial` and `calib_common` are not in `sys.modules` afterward.

**Testing plan**: Run `uv run --with pytest python -m pytest host/tests -v` after changes. Run `uv run python tests/calibrate/calibrate_linear.py --help` to confirm script is importable and flags work.
