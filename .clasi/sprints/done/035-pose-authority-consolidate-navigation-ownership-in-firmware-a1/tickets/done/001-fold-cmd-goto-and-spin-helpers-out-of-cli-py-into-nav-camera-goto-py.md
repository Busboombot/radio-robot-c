---
id: '001'
title: Fold cmd_goto and spin helpers out of cli.py into nav/camera_goto.py
status: done
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: a1a-fold-camera-goto-out-of-cli.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 035-001: Fold cmd_goto and spin helpers out of cli.py into nav/camera_goto.py

## Description

Move the inline camera-feedback controller from `cli.py` into a new module
`host/robot_radio/nav/camera_goto.py`. This is a pure refactor — the control
logic is unchanged; only the location moves. This is independent of the
ownership decision and safe to execute now that the design doc is committed.

Also delete `_spin_to_world_yaw` (confirmed dead code per OQ-5 in
`docs/decisions/029-pose-authority.md`): it has no callers, its dependency on
the old homography path was removed 2026-05-29, and it would fail at first call.

Do this ticket FIRST. Ticket 002 (a1b deletions) depends on this landing.

Source issue: `.clasi/issues/a1a-fold-camera-goto-out-of-cli.md`
Design doc: `docs/decisions/029-pose-authority.md` (Section 5.1 and Section 7)

## Acceptance Criteria

- [ ] `host/robot_radio/nav/camera_goto.py` is created containing:
  - `go_to_world_camera(proto, read_pose, target_x, target_y, cruise, turn_speed, gate_deg, arrive_cm, max_secs)` — extracted from `cmd_goto`
  - `spin_to_yaw_camera(proto, read_pose, target_deg, speed, tol_deg, max_secs)` — extracted from `_daemon_spin_to_yaw`
  - `crawl_drive_distance(robot, speed_mms, target_mm)` — extracted from `_crawl_drive_distance`
- [ ] `_spin_to_world_yaw` (cli.py lines ~857-934, ~78 lines) is deleted (confirmed no callers via grep before deleting).
- [ ] `cli.py::cmd_goto` is rewritten to call `nav.camera_goto.go_to_world_camera(...)`. It contains no `while` loop driving motors.
- [ ] `cli.py::cmd_turnto` is rewritten to call `nav.camera_goto.spin_to_yaw_camera(...)`. It contains no `while` loop driving motors.
- [ ] `rogo goto <x> <y>` produces identical observable output on the bench (same convergence, same final error reporting).
- [ ] `rogo turnto <deg>` produces identical observable output on the bench.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes with no new failures.
- [ ] `cli.py` line count is reduced by approximately 289 lines (the moved/deleted functions).
- [ ] No import cycle: `camera_goto.py` does NOT import from `cli.py`.
- [ ] `host/robot_radio/nav/__init__.py` is updated if needed to export the new module.

## Implementation Plan

### Approach

1. Grep `_spin_to_world_yaw` across the repo to confirm zero callers:
   `grep -rn "_spin_to_world_yaw" host/` — must return only the definition.
2. Create `host/robot_radio/nav/camera_goto.py`.
3. Copy `_daemon_spin_to_yaw` into it as `spin_to_yaw_camera(proto, read_pose, target_deg, speed, tol_deg, max_secs)`.
4. Copy `cmd_goto` core loop into it as `go_to_world_camera(proto, read_pose, target_x, target_y, cruise, turn_speed, gate_deg, arrive_cm, max_secs)`.
5. Copy `_crawl_drive_distance` into it as `crawl_drive_distance(robot, speed_mms, target_mm)`.
6. Rewrite `cmd_goto` in cli.py to: set up argparse as before, then call `nav.camera_goto.go_to_world_camera(...)`. No `while` loop in cli.py.
7. Rewrite `cmd_turnto` in cli.py to: set up argparse as before, then call `nav.camera_goto.spin_to_yaw_camera(...)`. No `while` loop in cli.py.
8. Delete `_spin_to_world_yaw` from cli.py (lines ~857-934).
9. Remove the now-empty private functions `_daemon_spin_to_yaw` and `_crawl_drive_distance` from cli.py.
10. Update `host/robot_radio/nav/__init__.py` if needed.
11. Run `uv run --with pytest python -m pytest host_tests/ host/tests/`.

### Files to create

- `host/robot_radio/nav/camera_goto.py`

### Files to modify

- `host/robot_radio/io/cli.py` — remove inline control loops, add import from camera_goto
- `host/robot_radio/nav/__init__.py` — add export if needed

### Files to delete

- Nothing (the functions are moved out of cli.py, not deleted from the repo)

### Key constraint

`camera_goto.py` must NOT import from `cli.py`. Imports should only flow from
cli.py into camera_goto.py. Check with `grep -n "from.*cli import\|import.*cli"
host/robot_radio/nav/camera_goto.py` after writing.

### Testing Plan

- Run `uv run --with pytest python -m pytest host_tests/ host/tests/` — full suite must pass.
- No new unit tests required (logic is unchanged; it is a move, not a rewrite).
- Bench smoke (operator-run): `rogo goto <x> <y>` and `rogo turnto <deg>` before and after the change.

### Documentation Updates

None beyond the architecture-update.md already written for this sprint.
