---
id: '003'
title: Write pose-authority architecture statement and consolidate docs
status: done
use-cases:
- SUC-004
depends-on:
- 035-002
github-issue: ''
issue: a1c-pose-authority-architecture-statement.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 035-003: Write pose-authority architecture statement and consolidate docs

## Description

Write the navigation architecture / pose-authority section in
`docs/architecture.md`, run the `consolidate-architecture` skill to produce a
new consolidated baseline, and mark the `a1-navigation-and-pose-ownership` issue done.

This ticket MUST execute after ticket 035-002 (a1b) is done. The documentation
must describe the code as it actually exists post-deletion, not as it was before.

Source issue: `.clasi/issues/a1c-pose-authority-architecture-statement.md`
Parent issue: `.clasi/issues/done/a1-navigation-and-pose-ownership.md` (to be marked done)

## Acceptance Criteria

- [ ] `docs/architecture.md` has a "Navigation Architecture" or "Pose Authority"
      section that:
  - Names the authoritative pose source for short-horizon motion (firmware EKF, P1).
  - Describes the camera-correction mechanism (OV/SI pose resets via `rogo sync pose`).
  - States that the host does not run a parallel steering loop.
  - Names `nav/camera_goto.py` as the CLI navigation module (for `rogo goto` / `rogo turnto`).
  - Names `nav/navigator.py` as the route-planner (for MCP `navigate_to` / `follow_path`).
  - Notes that `navigate_to` and `follow_path` MCP tools issue firmware G commands.
- [ ] The written statement matches the post-a1b code state (verify by reading
      the actual `navigator.py` and `robot_mcp.py` after 035-002 is done).
- [ ] `consolidate-architecture` skill is run and produces a new consolidated
      architecture document in `.clasi/architecture/`.
- [ ] Issue `a1-navigation-and-pose-ownership.md` (in `.clasi/issues/done/`) is
      confirmed done (it was already moved there; confirm its status field is `done`
      or update it).

## Implementation Plan

### Approach

1. Read `docs/architecture.md` to find the correct insertion point for the new section.
2. Read the post-a1b state of `navigator.py` and `robot_mcp.py` to verify the
   code matches what you are about to document.
3. Write the "Navigation Architecture" section in `docs/architecture.md`.
4. Run the `consolidate-architecture` skill (invoke via the Skill tool).
5. Confirm `.clasi/issues/done/a1-navigation-and-pose-ownership.md` has `status: done`.

### Section content guidance

The section should cover:
- **Pose Authority**: Firmware EKF (OTOS + encoders) is authoritative for
  short-horizon motion. Host does not run a parallel steering loop.
- **Camera corrections**: Host reads aprilcam daemon pose and sends OV/SI
  firmware commands to seed the EKF. `rogo sync pose` is the manual trigger.
- **CLI navigation** (`rogo goto`, `rogo turnto`): Handled by
  `nav/camera_goto.py` (`go_to_world_camera`, `spin_to_yaw_camera`). Uses
  real-time camera feedback and S/T motor commands for precise point-to-point.
- **Route planning / MCP navigation**: `nav/navigator.py` sequences firmware
  G commands for multi-waypoint navigation. MCP tools `navigate_to` and
  `follow_path` call these wrappers. No host-side steering loop.
- **Out of scope (future)**: G4 path (`NezhaKinematic.go_to_world`), automatic
  camera correction during traversal.

### Files to modify

- `docs/architecture.md` — add Navigation Architecture / Pose Authority section

### Files to create

- New consolidated architecture document (produced by `consolidate-architecture` skill)

### Testing Plan

No automated tests. Documentation only.

Verify by reading `navigator.py` and `robot_mcp.py` after 035-002 and confirming
the section matches the actual code state before committing.

### Documentation Updates

- `docs/architecture.md` — pose-authority section added.
- New consolidated architecture baseline in `.clasi/architecture/`.
