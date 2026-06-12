---
status: pending
---

# A1c — Write pose-authority statement in docs/architecture.md and consolidate

> Re-filed from sprint 029 (navigation-ownership), ticket 004, closed unimplemented.
> Must follow A1b — the docs must describe the code as it actually ends up, not as
> the design doc proposed.

## Context

The pose-authority decision is captured in `docs/decisions/029-pose-authority.md`,
but `docs/architecture.md` does not yet state it as the architecture baseline, and
the per-sprint architecture-update docs are not consolidated.

## Fix

After A1b lands, add a "Navigation Architecture" / "Pose Authority" section to
`docs/architecture.md` that:
- Names the authoritative pose source for short-horizon motion (firmware EKF).
- Describes the camera-correction mechanism (OV / SI pose resets).
- States the host runs no parallel steering loop.
- Names `nav/camera_goto.py` as the CLI navigation module.
- States the route-planner role if `navigator.py` was retained (OQ-2).

Verify the prose matches the post-A1b state of `navigator.py` and `robot_mcp.py`.
Then run the `consolidate-architecture` skill to produce a new consolidated baseline.

## Acceptance

- `docs/architecture.md` has the pose-authority section, matching real code.
- A consolidated architecture document is produced via `consolidate-architecture`.

## Priority suggestion

**Low.** Documentation close-out; sequenced last, after A1b. Completes the original
a1 navigation-and-pose-ownership issue.

## Source

Sprint 029 ticket 004; design doc `docs/decisions/029-pose-authority.md`;
issue a1-navigation-and-pose-ownership (now in `.clasi/issues/done/`).
