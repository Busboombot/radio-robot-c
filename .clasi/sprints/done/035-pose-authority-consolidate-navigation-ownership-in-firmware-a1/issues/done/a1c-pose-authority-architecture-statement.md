---
status: done
sprint: '035'
tickets:
- 035-003
---

# A1c — Write pose-authority statement in docs/architecture.md and consolidate

> **Provenance:** sprint 029 (navigation-ownership) ticket 004, closed unimplemented.
> Full ticket content preserved below. This ticket originally completed the a1 issue.
> **Sequencing:** must follow A1b — the docs must describe the code as it actually
> ends up, not as the design doc proposed.

## Description

This closes the a1 issue by writing the pose-authority statement into
`docs/architecture.md` and then running the `consolidate-architecture` skill to
merge all sprint update documents into a new consolidated architecture baseline.

This may only execute after A1b is done, because the documentation must describe the
code as it actually exists, not as it was before.

## Acceptance Criteria

- [ ] `docs/architecture.md` has a "Navigation Architecture" or "Pose Authority"
      section that:
      - Names the authoritative pose source for short-horizon motion (firmware EKF).
      - Describes the camera-correction mechanism (OV / SI pose resets).
      - States that the host does not run a parallel steering loop.
      - Names `nav/camera_goto.py` as the CLI navigation module.
      - States the route-planner role (if navigator.py was retained per OQ-2).
- [ ] The written statement matches what the code actually does (verify by reading
      the post-A1b state of navigator.py and robot_mcp.py).
- [ ] `consolidate-architecture` skill is run and a new consolidated architecture
      document is produced in `.clasi/architecture/`.
- [ ] Issue `a1-navigation-and-pose-ownership.md` is marked done.

## Implementation Plan

### Approach

1. Read the current `docs/architecture.md` to find the right insertion point.
2. Write the "Navigation Architecture" section based on the post-deletion code
   state (read navigator.py and robot_mcp.py after A1b is done).
3. Run the `consolidate-architecture` skill.

### Files to modify

- `docs/architecture.md` — add pose-authority section

### Files to create

- New consolidated architecture document (via `consolidate-architecture` skill)

### Testing Plan

No tests. Documentation only.

### Documentation Updates

- `docs/architecture.md` — pose-authority section added/updated.
- Consolidated architecture document created by the skill.

## Source

Sprint 029 ticket 004; design doc `docs/decisions/029-pose-authority.md`;
issue a1-navigation-and-pose-ownership (now in `.clasi/issues/done/`).
