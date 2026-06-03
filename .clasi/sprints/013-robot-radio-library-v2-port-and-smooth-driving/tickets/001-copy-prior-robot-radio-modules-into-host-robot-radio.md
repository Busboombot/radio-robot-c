---
id: '001'
title: Copy prior robot_radio modules into host/robot_radio
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: plan-port-v2-update-the-robot-radio-package-new-sprint.md
completes_issue: false
---

# Copy prior robot_radio modules into host/robot_radio

## Description

The prior library at `/Volumes/Proj/proj/league-projects/scratch/radio-robot/robot_radio` has a richer architecture than what is currently in `host/robot_radio/`: it includes nav, path, controllers, kinematics, media, and additional sensors that have already been ported into this repo. This ticket verifies the current state of `host/robot_radio/` against the prior library's module list and ensures all subpackages are present and importable. Since the git history shows these modules already exist in the repo, the primary work is:

1. Audit `host/robot_radio/` against the prior library's 49-file inventory to confirm all subpackages are present.
2. Confirm each subpackage has an `__init__.py` and is importable.
3. Identify any files present in the prior library but absent from this repo and add them (additive only — do not overwrite existing v2 files).
4. Run the existing 44-test suite and confirm all pass.

The v2 `NezhaProtocol`, `SerialConnection`, `RobotConfig`, `OdomTracker`, `CamTracker`, and all test files in `host/tests/` are canonical and must not be modified or overwritten.

## Acceptance Criteria

- [x] `host/robot_radio/` contains all subpackages from the prior library: `robot/`, `sensors/`, `nav/`, `path/`, `controllers/`, `kinematics/`, `io/`, `config/`, `media/` (if present).
- [x] Every subpackage has an `__init__.py`; none are empty (or the `__init__.py` exists and the module is importable).
- [x] `uv run python -c "from robot_radio import nav, path, controllers, kinematics"` exits 0.
- [x] `uv run python -c "from robot_radio.robot import Nezha, NezhaProtocol"` exits 0.
- [x] `uv run --with pytest python -m pytest host/tests` — all 44 (or more) existing tests pass.
- [x] No existing v2 file in `host/robot_radio/robot/protocol.py`, `host/robot_radio/io/serial_conn.py`, `host/robot_radio/config/robot_config.py`, or `host/tests/` is modified.

## Implementation Plan

**Approach**: Audit-and-fill. Read the prior library's directory listing and compare with `host/robot_radio/`. Copy missing files additive-only; skip any file that already exists in this repo.

**Files to check/create**:
- `host/robot_radio/nav/` — `__init__.py`, `navigator.py`, `pose.py`, `pose_align.py`, `nav_params.py`, `_approach_utils.py`.
- `host/robot_radio/path/` — `__init__.py`, `arc.py`, `bezier.py`, `builder.py`, `catmull_rom.py`, `obstacle.py`, `path_helper.py`, `patterns.py`, `sampled_path.py`.
- `host/robot_radio/controllers/` — `__init__.py`, `base.py`, `ltv.py`, `pid.py`, `pure_pursuit.py`, `stanley.py`.
- `host/robot_radio/kinematics/` — `__init__.py`, `differential_drive.py`.
- `host/robot_radio/media/` — if present in prior lib.
- `host/robot_radio/robot/` — confirm `robot.py`, `nezha.py`, `nezha_state.py`, `nezha_kinematic.py`, `clock_sync.py`, `cutebot.py` are all present.
- `host/robot_radio/sensors/` — confirm `color.py`, `motion_monitor.py`, `odometry.py`, `otos.py`, `calibration.py`, `odom_tracker.py`, `cam_tracker.py` are all present.

**Testing plan**: Run `uv run --with pytest python -m pytest host/tests` after all additions. No new tests in this ticket — that is T002's job.

**Documentation**: No README changes needed yet (T008 covers docs).
