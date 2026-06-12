---
status: in-progress
sprint: 028
tickets:
- 028-003
---

# A6 — Extract library logic trapped in cli.py (2262 lines)

## Context

`io/cli.py` contains, beyond argument parsing: closed-loop controllers
(`cmd_goto`'s inline pure-pursuit, `_spin_to_world_yaw`, `_daemon_spin_to_yaw`,
`_crawl_drive_distance` — see A1), calibration push logic (`_push_calibration`,
`_scale_to_int8` duplicating `io/calibrate.py`), session/port caching, TLM snapshot
parsing, and robot construction policy (`_make_robot`). `io/robot_mcp.py` (1016
lines) is a second front-end that needs the same behaviors and cannot import them
cleanly, so logic drifts between the two — the MCP path the main agent uses and the
CLI path a human uses are not the same code.

## Fix

Move anything both front-ends need into library modules: control loops → `nav/`
(per A1), calibration push → the consolidated calibration package (A7), robot
construction/port resolution → `robot/` or `config/`. cli.py and robot_mcp.py
become thin adapters over the same library calls.

## Acceptance

- cli.py is arg-parsing + printing only (rough proxy: < ~800 lines, no `while`
  control loops); robot_mcp.py imports the same library functions instead of
  reimplementing; no duplicated helpers between the two.

## Priority suggestion

**Medium — do opportunistically as A1 and A7 pull their pieces out**, rather than
as a standalone sprint item. The exception: if the main agent operates via
robot_mcp.py, pull `_make_robot`/port-resolution extraction forward, since CLI/MCP
drift directly causes "works for the human, fails for the agent" reports.

## Source
Finding **A6** in `docs/code_review/2026-06-11-architecture-modularity-review.md`.
