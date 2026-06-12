---
status: pending
---

# A1a — Fold cmd_goto and spin helpers out of cli.py into nav/camera_goto.py

> Re-filed from sprint 029 (navigation-ownership), ticket 002, which was closed
> unimplemented. The pose-authority design doc that gates this work IS written and
> committed: `docs/decisions/029-pose-authority.md`. This is the "first casualty
> regardless of the ownership decision" — a pure refactor, safe to do on its own.

## Context

The inline camera-feedback controller lives in `host/robot_radio/io/cli.py`:
`cmd_goto`, `_daemon_spin_to_yaw`, `_spin_to_world_yaw`, `_crawl_drive_distance`.
These are motion-control loops sitting in the CLI module. The control logic is fine;
only its location is wrong (~300 lines that bloat cli.py, currently ~2262 lines).

## Fix

Create `host/robot_radio/nav/camera_goto.py` containing:
- `go_to_world_camera(proto, read_pose, ...)` — extracted from `cmd_goto`
- `spin_to_yaw_camera(proto, read_pose, ...)` — extracted from `_daemon_spin_to_yaw`
- `crawl_drive_distance(robot, speed_mms, target_mm)` — extracted from `_crawl_drive_distance`

Rewrite `cli.py::cmd_goto` and `cmd_turnto` to import and call these (no `while`
loops driving motors left in cli.py). Grep `_spin_to_world_yaw` (cli.py:1086) for
callers — it is likely dead (superseded by the daemon path, OQ-5); delete if zero
callers, move if live. No new import cycles (camera_goto must not import cli.py).

## Acceptance

- `rogo goto <x> <y>` and `rogo turnto <deg>` produce identical bench behaviour
  (same convergence, same final error reporting) before and after.
- `cli.py` line count drops ~300 lines; `uv run pytest` passes with no new failures.

## Priority suggestion

**Low–medium.** Independent of the ownership decision; can be picked up anytime.
Naturally sequenced before A1b (controller deletion).

## Source

Sprint 029 ticket 002; design doc `docs/decisions/029-pose-authority.md`;
issue a1-navigation-and-pose-ownership (now in `.clasi/issues/done/`).
