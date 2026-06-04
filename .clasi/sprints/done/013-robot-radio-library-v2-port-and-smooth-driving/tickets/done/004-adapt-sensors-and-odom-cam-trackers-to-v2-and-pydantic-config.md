---
id: '004'
title: Adapt sensors and odom/cam trackers to v2 and pydantic config
status: done
use-cases:
- SUC-006
depends-on:
- '003'
github-issue: ''
issue: ''
completes_issue: false
---

# Adapt sensors and odom/cam trackers to v2 and pydantic config

## Description

The sensor layer needs three adjustments after T003's `Nezha` rewrite:

1. **OdomTracker** (`sensors/odom_tracker.py`): Remove or deprecate `parse_so()` (v1 `SO` stream); the tracker now receives `TLMFrame.pose` (already in mm from v2 firmware). Wire tracker init to `RobotConfig` fields (`trackwidth_mm`, `mm_per_deg_l`, `mm_per_deg_r`).

2. **CamTracker** (`sensors/cam_tracker.py`): Confirm AprilTag filtering uses tag ID `100` (robot's tag in this repo). Confirm pose units are mm (cm→mm conversion was done in sprint 012 — verify it remains correct). Wire to `RobotConfig` if any config fields are needed (e.g., camera-to-robot offset).

3. **OtosSensor helper** (`sensors/otos.py`): Confirm all methods use v2 verbs only (`OP`, `OZ`, `OR`, `OI`, `OL n`, `OA n`, `OV`). No `SO`/`OO`/`OK` remnants. Confirm `OP` raw position is INT16 LSB × 0.305176 mm/LSB.

4. Confirm `sensors/color.py`, `sensors/motion_monitor.py`, `sensors/odometry.py`, `sensors/calibration.py` import cleanly with no v1 wire calls.

## Acceptance Criteria

- [x] `OdomTracker` accepts `TLMFrame` directly (not raw `SO` strings); `parse_so()` is removed or clearly marked deprecated with no callers.
- [x] `OdomTracker.__init__` accepts a `RobotConfig` (or equivalent keyword args) for `trackwidth_mm`, `mm_per_deg_l/r`.
- [x] `CamTracker` filters to AprilTag ID 100; ignores other tag IDs.
- [x] `CamTracker` pose units are mm; no cm-to-mm conversion needed at the caller.
- [x] `sensors/otos.py` contains no v1 verb strings (`SO`, `OO`, `OK` as OTOS-complete ack).
- [x] All four peripheral sensor modules (`color`, `motion_monitor`, `odometry`, `calibration`) import without error.
- [x] Existing `host/tests/test_odom_tracker.py` still passes; extend with `TLMFrame`-based input test if not already present.
- [x] `uv run --with pytest python -m pytest host/tests` — all tests pass.

## Implementation Plan

**Approach**: Read each sensor file. Make targeted changes only — do not restructure modules that already work. Add/extend tests.

**Files to modify**:
- `host/robot_radio/sensors/odom_tracker.py` — remove `parse_so`; accept `TLMFrame`; wire `RobotConfig`.
- `host/robot_radio/sensors/cam_tracker.py` — confirm tag 100; confirm mm units.
- `host/robot_radio/sensors/otos.py` — remove any v1 verb remnants.
- `host/tests/test_odom_tracker.py` — add `TLMFrame` input test case.

**New test cases**:
- `test_odom_tracker_from_tlm` — construct an `OdomTracker`; feed it a `TLMFrame(pose=(100, 50, 900))`; assert `tracker.x_mm == 100`, `tracker.y_mm == 50`, `tracker.heading_cdeg == 900`.
- `test_cam_tracker_accepts_tag_100` — construct a `CamTracker`; feed it a tag-100 observation; assert pose updated.
- `test_cam_tracker_rejects_other_tags` — feed a tag-99 observation; assert pose unchanged (if tag filtering is implemented).

**Testing plan**: Run `uv run --with pytest python -m pytest host/tests -v` after changes.
