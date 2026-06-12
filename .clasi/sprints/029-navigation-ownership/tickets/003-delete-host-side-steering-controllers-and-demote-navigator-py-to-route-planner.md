---
id: '003'
title: Delete host-side steering controllers and demote navigator.py to route planner
status: open
use-cases:
  - SUC-003
depends-on:
  - '001'
  - '002'
github-issue: ''
issue: a1-navigation-and-pose-ownership.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Delete host-side steering controllers and demote navigator.py to route planner

## Description

With the design doc signed off (Ticket 001) and the CLI fold-in done (Ticket 002),
this ticket executes the ownership decision: delete the host-side steering loop
and its controller library.

**Hard gate**: Do not begin this ticket until the stakeholder-approved design doc
explicitly authorises deletion of the specific files listed below. If the design
doc arrives at a different conclusion (e.g., retain the host-side loop for
multi-waypoint paths), the scope of this ticket must be revised before execution.

**What to delete** (per the suggested ownership split from architecture-update.md):
- `nav/controllers/pure_pursuit.py`, `stanley.py`, `ltv.py` — host-side steering
  controllers with zero callers outside navigator.py.
- `navigator.py` methods: `navigate`, `follow_path`, `_run_controller`,
  `_spin_to_heading` — the steering-loop methods.
- Corresponding `robot_mcp.py` tool handlers that called these navigator methods
  (update or remove `navigate`, `follow_path`, `follow_pose_path` MCP tools).

**What to retain** (minimum, subject to design doc answer to OQ-2):
- `navigator.py` route-planning methods if the design doc determines they are
  needed (e.g., `visit_tags` rewritten to issue G commands, `approach` method).
- `controllers/pid.py` if it has callers outside the steering path.

The actual keep/delete list must be confirmed against the design doc before
execution. The executor must grep all callers before deleting any file.

## Acceptance Criteria

- [ ] **BLOCKED on Ticket 001**: Design doc is signed off and explicitly authorises
      the specific deletions below before this ticket may begin.
- [ ] `controllers/pure_pursuit.py`, `stanley.py`, `ltv.py` are deleted (confirmed
      zero callers in production path by grep).
- [ ] `navigator.py` contains no `while` loop sending motor commands (drive/S/T).
- [ ] `robot_mcp.py` MCP tools `navigate` and `follow_path` are either removed or
      rewritten to issue firmware G commands (per design doc decision on OQ-1).
- [ ] All tests that imported the deleted controllers are removed or rewritten.
- [ ] `uv run pytest` passes with no new failures.
- [ ] Smoke ritual passes after deletion.

## Implementation Plan

### Approach

1. Confirm the design doc answer to OQ-1 (MCP tools) and OQ-2 (navigator route
   planner retention).
2. Grep every deleted file for callers: `grep -rn "pure_pursuit\|stanley\|ltv" host/`
   and confirm zero production callers.
3. Delete `controllers/pure_pursuit.py`, `stanley.py`, `ltv.py`.
4. Remove the steering-loop methods from `navigator.py`. If the route-planner
   methods (`visit_tags`, `grab_at`, etc.) survive, rewrite them to use
   `robot.go_to()` / firmware G commands instead of `_run_controller`.
5. Update `robot_mcp.py`: remove or rewrite `navigate`, `follow_path`,
   `follow_pose_path` handlers per the design doc.
6. Update or remove tests that import the deleted modules.
7. Run smoke ritual: `mbdeploy deploy robot --clean` + bench test.

### Files to delete

- `host/robot_radio/controllers/pure_pursuit.py`
- `host/robot_radio/controllers/stanley.py`
- `host/robot_radio/controllers/ltv.py`
- `host/robot_radio/controllers/__init__.py` (if empty after deletions)

### Files to modify

- `host/robot_radio/nav/navigator.py` — delete steering-loop methods
- `host/robot_radio/io/robot_mcp.py` — update/remove navigate/follow_path tools
- `host/robot_radio/controllers/__init__.py` — remove deleted exports
- Any test files importing the deleted controllers

### Testing Plan

- `uv run pytest` — full suite; confirm tests for deleted modules are removed.
- Bench smoke ritual: verify `rogo go`, `rogo goto`, `rogo turn` still work.
- If `robot_mcp.py` navigate tool is rewritten (not removed), test it via MCP
  client against the bench robot.

### Documentation Updates

None — architecture-update.md already documents the deletions.
