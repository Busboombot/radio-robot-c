---
id: '004'
title: Write pose-authority statement in docs/architecture.md and consolidate architecture
status: open
use-cases:
  - SUC-004
depends-on:
  - '001'
  - '003'
github-issue: ''
issue: a1-navigation-and-pose-ownership.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Write pose-authority statement in docs/architecture.md and consolidate architecture

## Description

This ticket closes the a1 issue by writing the pose-authority statement into
`docs/architecture.md` and then running the `consolidate-architecture` skill to
merge all sprint update documents into a new consolidated architecture baseline.

This ticket may only execute after Ticket 003 is done, because the documentation
must describe the code as it actually exists, not as it was before.

## Acceptance Criteria

- [ ] `docs/architecture.md` has a "Navigation Architecture" or "Pose Authority"
      section that:
      - Names the authoritative pose source for short-horizon motion (firmware EKF).
      - Describes the camera-correction mechanism (OV / SI pose resets).
      - States that the host does not run a parallel steering loop.
      - Names `nav/camera_goto.py` as the CLI navigation module.
      - States the route-planner role (if navigator.py was retained per OQ-2).
- [ ] The written statement matches what the code actually does (verify by reading
      the post-Ticket-003 state of navigator.py and robot_mcp.py).
- [ ] `consolidate-architecture` skill is run and a new consolidated architecture
      document is produced in `.clasi/architecture/`.
- [ ] Sprint 029 is closed via the `close-sprint` skill.
- [ ] Issue `a1-navigation-and-pose-ownership.md` is marked done.

## Implementation Plan

### Approach

1. Read the current `docs/architecture.md` to find the right insertion point.
2. Write the "Navigation Architecture" section based on the post-deletion code
   state (read navigator.py and robot_mcp.py after Ticket 003 is done).
3. Run the `consolidate-architecture` skill.
4. Run `close-sprint` skill.

### Files to modify

- `docs/architecture.md` — add pose-authority section

### Files to create

- New consolidated architecture document (via `consolidate-architecture` skill)

### Testing Plan

No tests. Documentation only.

### Documentation Updates

- `docs/architecture.md` — pose-authority section added/updated.
- Consolidated architecture document created by the skill.
