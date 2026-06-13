---
id: '002'
title: Delete host-side steering controllers and demote navigator.py to route planner
status: open
use-cases:
- SUC-003
depends-on:
- '035-001'
github-issue: ''
issue: a1b-delete-host-steering-controllers.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 035-002: Delete host-side steering controllers and demote navigator.py to route planner

## Description

This ticket executes the ownership decision from
`docs/decisions/029-pose-authority.md`: delete the host-side steering loop
and its controller library. After this ticket, no host code sends S/T commands
in a loop to steer toward a waypoint (the `nav/camera_goto.py` CLI path is the
sole exception and is intentional).

**HARD GATE — PREREQUISITES before starting**:
1. Ticket 035-001 (a1a) must be done.
2. Re-read `docs/decisions/029-pose-authority.md` and confirm it sanctions the
   specific deletions listed below. (It does: Sections 4/5 resolve all OQs.)
3. Stakeholder must confirm sprints 026-027 bench-validated the firmware G path
   (OQ-6 in the design doc). Do not begin if OQ-6 is unconfirmed.
4. Grep every file before deleting: `grep -rn "pure_pursuit\|stanley\|ltv" host/`
   and confirm zero production callers outside navigator.py.

Source issue: `.clasi/issues/a1b-delete-host-steering-controllers.md`
Design doc: `docs/decisions/029-pose-authority.md` (Sections 4, 5, 6)

## What the Design Doc Resolves (read before acting)

### OQ-1: MCP tools navigate_to / follow_path / follow_pose_path

Per design doc Section 4 (OQ-1) and Section 5.2:

- `navigate_to` and `follow_path` MCP tools: **REIMPLEMENT** as thin G-command
  wrappers. Signatures unchanged. Implementation changes to issue firmware G
  commands and wait for `EVT done G`.
- `follow_pose_path` MCP tool: **REMOVE**. No clean G-command equivalent.
  Agents should use `navigate_to` with a `sync_pose` call before each waypoint.

### OQ-2: navigator.py route-planner methods to retain vs. delete

Per design doc Section 4 (OQ-2) and Section 5.2:

**DELETE** from navigator.py:
- `navigate` method (dual-PID steering loop, calls `ChaseController`)
- `ChaseController` class (~173 lines)
- `_build_controller` helper
- `follow_path` method (path-following loop, calls `_run_controller`)
- `follow_pose_path` method (3-phase planner + loop)
- `_spin_to_heading` method (host-side spin loop)
- `_run_controller` method (shared loop runner)

**RETAIN** in navigator.py (rewrite where noted):
- `visit_tags` — currently calls `self.navigate()`; must be rewritten to call
  the new G-command wrapper method after `navigate` is replaced
- `approach` — uses two-phase distance-based controller; NOT a continuous
  steering loop; retain as-is
- `grab_at` — calls `self.navigate()` internally; must be rewritten to call
  G-command wrapper
- `release_at` — delegates to `grab_at`; no change needed beyond grab_at fix
- `read_pose` — camera pose reader; retain unchanged
- `_get_playfield`, `reset_camera`, `status`, `get_next_tags` — plumbing; retain
- `adaptive_turn` — uses `speed_for_time` in a short bounded convergence loop;
  not a continuous steering loop; retain for now
- `gripper_position`, `_read_pose_from_field`, `_drive_straight` — utility methods; retain

**REIMPLEMENT** as G-command wrappers on Navigator class (replacing old methods):
- `navigate(target_xy, timeout=30.0, ...)` — converts world-cm to robot-relative
  mm, sends firmware G command, waits for done response, returns same dict schema
- `follow_path(path, timeout=30.0, ...)` — sequences one `navigate()` call per
  consecutive waypoint, waits for done before advancing

### OQ-5: _spin_to_world_yaw

Already deleted in ticket 035-001. Nothing to do here.

## Acceptance Criteria

- [ ] **HARD GATE**: Design doc `docs/decisions/029-pose-authority.md` re-read
      and confirms the deletions below are sanctioned.
- [ ] **HARD GATE**: Stakeholder has confirmed sprints 026-027 bench-validated
      the firmware G path (OQ-6).
- [ ] Grep passes before any deletion: `grep -rn "pure_pursuit\|stanley\|ltv" host/`
      returns zero lines outside navigator.py and test files.
- [ ] `controllers/pure_pursuit.py` is deleted.
- [ ] `controllers/stanley.py` is deleted.
- [ ] `controllers/ltv.py` is deleted.
- [ ] `controllers/__init__.py` no longer exports `PurePursuitTracker` or `StanleyController`.
- [ ] `navigator.py` contains no `while` loop sending motor S/T commands in a
      steering loop. (`approach` and `adaptive_turn` may retain their bounded loops.)
- [ ] `navigator.py` has `navigate` and `follow_path` reimplemented as G-command
      wrappers (send firmware G command, wait for done response).
- [ ] `visit_tags` and `grab_at` call the new G-command `navigate` wrapper
      (not the deleted steering-loop version).
- [ ] `robot_mcp.py` `navigate_to` handler works via the new G-command navigator.
- [ ] `robot_mcp.py` `follow_path` handler works via the new G-command navigator.
- [ ] `robot_mcp.py` `follow_pose_path` handler is removed.
- [ ] `host_tests/test_imports_smoke.py` lines importing `PurePursuitTracker` and
      `StanleyController` are removed.
- [ ] All other test files importing the deleted controllers are updated/removed.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes with
      no new failures.
- [ ] Bench smoke ritual (operator-run): `rogo go`, `rogo goto`, `rogo turn` still
      work after the deletion.

## Implementation Plan

### Approach

1. Confirm both hard gates (re-read design doc; confirm stakeholder OQ-6 sign-off).
2. Grep for callers of the deleted files before touching anything:
   - `grep -rn "pure_pursuit\|PurePursuitTracker" host/`
   - `grep -rn "stanley\|StanleyController" host/`
   - `grep -rn "ltv\|LTVController" host/`
   - Must confirm zero production callers outside navigator.py.
3. Delete `host/robot_radio/controllers/pure_pursuit.py`, `stanley.py`, `ltv.py`.
4. Update `host/robot_radio/controllers/__init__.py` — remove deleted exports.
5. Delete steering-loop code from `navigator.py` (keep `pid.py` import if used elsewhere):
   - `ChaseController` class and `_build_controller` helper
   - `navigate` method (old dual-PID steering loop)
   - `follow_path` method (old path-following loop)
   - `follow_pose_path` method
   - `_spin_to_heading` method
   - `_run_controller` method
6. Add G-command wrappers to `navigator.py`:
   - `navigate(target_xy, timeout=30.0, ...)` — check `host/robot_radio/robot/`
     for the Robot API for sending G commands and waiting for firmware events.
     Convert world-cm to robot-relative mm, issue G command, poll for EVT done G.
     Return `{"success": bool, "elapsed": float, ...}`.
   - `follow_path(path, timeout=30.0, ...)` — sequence one `navigate()` per
     consecutive waypoint pair, waiting for done before advancing.
7. Update `visit_tags` to call `self.navigate(...)` (the new G wrapper).
8. Update `grab_at` to call `self.navigate(...)` (the new G wrapper).
9. Update `robot_mcp.py`:
   - `navigate_to` and `follow_path` handlers already call `_navigator.navigate()`
     and `_navigator.follow_path()` — they will work without changes once the
     navigator methods are the G wrappers.
   - Remove `follow_pose_path` handler (lines ~893-930 in robot_mcp.py).
10. Update `host_tests/test_imports_smoke.py`: remove lines importing
    `PurePursuitTracker` and `StanleyController`.
11. Scan all test files for other imports of deleted modules:
    `grep -rn "pure_pursuit\|stanley\|ltv\|PurePursuitTracker\|StanleyController\|LTVController" host_tests/ host/tests/`
    Update or remove any found.
12. Run `uv run --with pytest python -m pytest host_tests/ host/tests/`.

### G-command protocol reference

Per `docs/decisions/029-pose-authority.md` Section 4 (OQ-1):
- Single waypoint: send G command via `self._robot.send(...)`, then poll for
  done response (`EVT done G` or equivalent).
- Check `host/robot_radio/robot/` for the actual Robot API before writing the
  wrapper. Do not assume the exact method names.

### Files to delete

- `host/robot_radio/controllers/pure_pursuit.py`
- `host/robot_radio/controllers/stanley.py`
- `host/robot_radio/controllers/ltv.py`

### Files to modify

- `host/robot_radio/controllers/__init__.py` — remove deleted exports
- `host/robot_radio/nav/navigator.py` — delete steering methods, add G wrappers, update visit_tags/grab_at
- `host/robot_radio/io/robot_mcp.py` — remove follow_pose_path handler
- `host_tests/test_imports_smoke.py` — remove PurePursuitTracker/StanleyController imports
- Any other test files importing deleted controllers (found by grep above)

### Testing Plan

- `uv run --with pytest python -m pytest host_tests/ host/tests/` — full suite.
- Bench smoke (operator-run): `rogo go <dist>`, `rogo goto <x> <y>`, `rogo turn <deg>`.
- If `navigate_to` and `follow_path` MCP tools are testable via bench MCP client,
  verify them after the rewrite.

### Documentation Updates

None for this ticket. Ticket 035-003 handles the architecture docs.
