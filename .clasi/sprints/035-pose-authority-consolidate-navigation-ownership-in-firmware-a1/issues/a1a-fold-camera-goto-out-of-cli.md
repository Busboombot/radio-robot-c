---
status: in-progress
sprint: '035'
tickets:
- 035-001
---

# A1a — Fold cmd_goto and spin helpers out of cli.py into nav/camera_goto.py

> **Provenance:** sprint 029 (navigation-ownership) ticket 002, closed unimplemented.
> Full ticket content preserved below.
> **Gate:** the pose-authority design doc that authorises this work is written and
> committed at `docs/decisions/029-pose-authority.md`.
> **Sequencing:** do before A1b (controller deletion). Independent of the ownership
> decision — a refactor, not a deletion.

## Description

This is the "first casualty regardless of decision" from the a1 issue. The inline
camera-feedback controller in cli.py (`cmd_goto`, `_daemon_spin_to_yaw`,
`_spin_to_world_yaw`, `_crawl_drive_distance`) is moved to a new module
`host/robot_radio/nav/camera_goto.py`. This is independent of the ownership
decision (it is a refactor, not a deletion) and safe to execute as soon as the
design doc is signed off.

The control logic is unchanged. Only the location moves.

**Note on `_spin_to_world_yaw`**: This function (cli.py line 1086) uses the local
`Playfield`/`Camera` object path, which has been superseded by `_daemon_spin_to_yaw`
(which reads from the aprilcam daemon). It appears to have no callers in the live
path. Verify with grep before moving — if it has no callers, delete it rather than
move it. (See OQ-5 in the design doc.)

## Acceptance Criteria

- [ ] `host/robot_radio/nav/camera_goto.py` is created containing:
      - `go_to_world_camera(proto, read_pose, ...)` — extracted from `cmd_goto`
      - `spin_to_yaw_camera(proto, read_pose, ...)` — extracted from `_daemon_spin_to_yaw`
  - [ ] `_spin_to_world_yaw` is either moved (if it has callers) or deleted (if dead code).
- [ ] `cli.py::cmd_goto` and `cmd_turnto` are rewritten to import and call
      `nav/camera_goto.py` functions. They contain no `while` loops driving motors.
- [ ] `_crawl_drive_distance` is moved to `nav/camera_goto.py` or `nav/` (it is a
      motion primitive, not a CLI concern).
- [ ] `rogo goto <x> <y>` and `rogo turnto <deg>` produce identical observable
      output on the bench (same convergence, same final error reporting).
- [ ] `uv run pytest` passes with no new failures.
- [ ] `cli.py` line count is reduced (actual delta is implementation-dependent;
      approximately -300 lines from the moved functions).
- [ ] No new import cycles introduced (camera_goto must not import from cli.py).

## Implementation Plan

### Approach

1. Create `host/robot_radio/nav/camera_goto.py`.
2. Copy `_daemon_spin_to_yaw` into it as `spin_to_yaw_camera(proto, read_pose,
   target_deg, speed, tol_deg, max_secs)`.
3. Copy `cmd_goto` core loop into it as `go_to_world_camera(proto, read_pose,
   target_x, target_y, cruise, turn_speed, gate_deg, arrive_cm, max_secs)`.
4. Copy `_crawl_drive_distance` into it as `crawl_drive_distance(robot, speed_mms,
   target_mm)`.
5. Grep for callers of `_spin_to_world_yaw`. If zero callers, delete. If callers
   exist, move it too.
6. Rewrite `cmd_goto` in cli.py to call `nav.camera_goto.go_to_world_camera(...)`.
   Keep the argparse setup unchanged.
7. Rewrite `cmd_turnto` in cli.py to call `nav.camera_goto.spin_to_yaw_camera(...)`.
8. Remove the now-empty private functions from cli.py.
9. Update `host/robot_radio/nav/__init__.py` if needed to export the new module.

### Files to create

- `host/robot_radio/nav/camera_goto.py`

### Files to modify

- `host/robot_radio/io/cli.py` — remove inline control loops, import camera_goto
- `host/robot_radio/nav/__init__.py` — add export if needed

### Testing Plan

- Manual bench test: `rogo goto <x> <y>` and `rogo turnto <deg>` before and after.
- `uv run pytest` — full suite must pass.
- No new unit tests required (the logic is unchanged; it is a move, not a rewrite).

### Documentation Updates

None — the architecture-update.md already documents this change.

## Source

Sprint 029 ticket 002; design doc `docs/decisions/029-pose-authority.md`;
issue a1-navigation-and-pose-ownership (now in `.clasi/issues/done/`).
