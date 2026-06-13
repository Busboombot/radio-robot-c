---
id: '035'
title: 'Pose authority: consolidate navigation ownership in firmware (A1)'
status: done
branch: sprint/035-pose-authority-consolidate-navigation-ownership-in-firmware-a1
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
issues:
- a1a-fold-camera-goto-out-of-cli.md
- a1b-delete-host-steering-controllers.md
- a1c-pose-authority-architecture-statement.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 035: Pose authority: consolidate navigation ownership in firmware (A1)

## Goals

Consolidate navigation/pose ownership: the firmware EKF becomes the sole
authoritative steering loop for short-horizon motion; the host stops running
a parallel steering loop. This is the "A1" cluster from the architecture review,
executing the plan laid out in `docs/decisions/029-pose-authority.md`.

## Problem

Three independent go-to-point implementations (G1 firmware, G2 navigator/MCP,
G3 CLI inline) and four pose estimators coexist with no defined authority.
Every navigation bug must be hunted in three stacks. Firmware fixes from
sprints 024–027 (PRE_ROTATE timeout, uint32 signed delta, PURSUE backstop,
arrive-tolerance tuning) have no effect when an agent uses the host-side
navigator or CLI inline controller.

## Solution

Three sequential tickets:

1. **Fold CLI controller** (a1a) — Move `cmd_goto`, `_daemon_spin_to_yaw`,
   `_crawl_drive_distance` from `cli.py` into a new module
   `host/robot_radio/nav/camera_goto.py`. Delete dead code `_spin_to_world_yaw`.
   Purely a refactor — control logic unchanged.

2. **Delete host steering controllers** (a1b) — Delete
   `controllers/pure_pursuit.py`, `stanley.py`, `ltv.py`; remove the
   steering-loop methods from `navigator.py`; reimplement `navigate` and
   `follow_path` as G-command wrappers; update `robot_mcp.py` accordingly.

3. **Architecture docs** (a1c) — Write the pose-authority section in
   `docs/architecture.md`, run `consolidate-architecture`, mark the a1 issue done.

## Success Criteria

- `rogo goto`, `rogo turnto`, `rogo go`, `rogo turn` all work after all three tickets.
- `uv run --with pytest python -m pytest host_tests/ host/tests/` passes (>=719 tests, no new failures).
- No host method sends S/T commands in a loop to steer toward a waypoint (outside `camera_goto.py`).
- `nav/camera_goto.py` exists; `cli.py` contains no inline motor control loops.
- MCP tools `navigate_to` and `follow_path` work via firmware G commands.
- `docs/architecture.md` has a Navigation Architecture section matching the post-sprint code.

## Scope

### In Scope

- `host/robot_radio/io/cli.py` — remove inline loops, import `nav/camera_goto`
- `host/robot_radio/nav/camera_goto.py` (new)
- `host/robot_radio/nav/navigator.py` — delete steering methods, add G wrappers
- `host/robot_radio/controllers/pure_pursuit.py`, `stanley.py`, `ltv.py` (delete)
- `host/robot_radio/io/robot_mcp.py` — update navigate_to, follow_path, remove follow_pose_path
- `host_tests/` — remove/update tests that imported deleted controllers
- `docs/architecture.md` — pose-authority section
- Consolidated architecture document (via `consolidate-architecture` skill)

### Out of Scope

- Firmware changes (none required)
- G4 path (`NezhaKinematic.go_to_world` / `OdomTracker`) — deferred to a future sprint (OQ-3)
- Automatic camera correction loop during traversal — deferred (OQ-4)
- `controllers/pid.py` — retained, not modified

## Test Strategy

- Automated: `uv run --with pytest python -m pytest host_tests/ host/tests/` after each ticket.
- Smoke ritual (operator-run on hardware): `rogo goto`, `rogo turnto`, `rogo go`, `rogo turn` after a1b completes.
- Grep-based caller checks before every deletion: zero production callers required.

## Architecture Notes

- Authoritative design doc: `docs/decisions/029-pose-authority.md`
- OQ-6 hardware gate: stakeholder must confirm sprints 026-027 bench-validated the firmware G path before ticket 002 (a1b) executes.
- `nav/camera_goto.py` must NOT import from `cli.py` (no import cycles).
- Tests run with `uv run --with pytest python -m pytest host_tests/ host/tests/`, NOT bare `uv run pytest`.

## GitHub Issues

(No GitHub issues linked.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 035-001 | Fold cmd_goto and spin helpers out of cli.py into nav/camera_goto.py | — |
| 035-002 | Delete host-side steering controllers and demote navigator.py to route planner | 035-001 |
| 035-003 | Write pose-authority architecture statement and consolidate docs | 035-002 |

Tickets execute serially in the order listed.
