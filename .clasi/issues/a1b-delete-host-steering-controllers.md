---
status: pending
---

# A1b — Delete host-side steering controllers and demote navigator.py to route planner

> Re-filed from sprint 029 (navigation-ownership), ticket 003, closed unimplemented.
> Gated on the committed design doc `docs/decisions/029-pose-authority.md` and on
> A1a (camera_goto fold-in) landing first.

## Context

The host runs a parallel steering loop that duplicates the firmware's job. The
pose-authority decision is that firmware owns short-horizon motion + pose fusion;
the host owns route planning and camera-based pose *corrections* (OV / SI resets),
not its own steering loop. That makes the host steering library dead weight.

## Fix

**Hard gate:** re-read `docs/decisions/029-pose-authority.md` and confirm it
authorises deleting the specific files below (and its answers to OQ-1 / OQ-2)
before deleting anything. Grep all callers first.

- Delete `host/robot_radio/controllers/pure_pursuit.py`, `stanley.py`, `ltv.py`
  (confirm zero production callers), and `controllers/__init__.py` if it ends empty.
- Remove steering-loop methods from `nav/navigator.py` (`navigate`, `follow_path`,
  `_run_controller`, `_spin_to_heading`). If route-planner methods (`visit_tags`,
  `approach`, …) are retained per OQ-2, rewrite them to issue firmware G commands
  via `robot.go_to()` instead of `_run_controller`.
- Update/remove `io/robot_mcp.py` MCP tools `navigate` / `follow_path` /
  `follow_pose_path` per the design doc (OQ-1). **Breaking MCP API change** — name
  and update all known callers.
- Remove or rewrite tests importing the deleted controllers.

## Acceptance

- `navigator.py` contains no `while` loop sending motor commands.
- `uv run pytest` passes; bench smoke ritual passes (`rogo go`/`goto`/`turn` work).

## Priority suggestion

**Medium.** This is the actual ownership cleanup. Sequenced after A1a. Confirm the
design doc's keep/delete list before scoping a sprint.

## Source

Sprint 029 ticket 003; design doc `docs/decisions/029-pose-authority.md`;
issue a1-navigation-and-pose-ownership (now in `.clasi/issues/done/`).
