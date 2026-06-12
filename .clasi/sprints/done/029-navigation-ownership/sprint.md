---
id: 029
title: Navigation ownership
status: done
branch: sprint/029-navigation-ownership
use-cases: []
issues:
- a1-navigation-and-pose-ownership
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 029: Navigation ownership

## Goals

One go-to-point implementation per regime, one pose authority. The
duplication of three independent closed-loop controllers and three
pose estimators is resolved by decision, then by deletion. The decision
itself — a written design doc with stakeholder sign-off — is an explicit
deliverable before any code moves.

## Problem

The same "drive to a world point" capability exists in three stacks with
no shared code, parameters, or pose state (a1):

1. **Firmware** — `MotionController::beginGoTo` + PURSUE law (EKF pose).
2. **Host library** — `nav/navigator.py` (1349 lines) + `controllers/`
   (pure_pursuit, stanley, ltv, pid) from camera pose via `sensors/odometry.py`.
3. **CLI inline** — `io/cli.py::cmd_goto` (~165 lines, its own gains), plus
   `_spin_to_world_yaw`, `_daemon_spin_to_yaw`, `_crawl_drive_distance`.

Three pose estimators with no defined authority: firmware EKF,
`sensors/odom_tracker.py` (TLM integration), `sensors/odometry.py` (camera +
OTOS fallback). Every navigation bug must be hunted in three stacks. The P0/P1
firmware fixes (sprints 024–027) do nothing for an agent that happens to invoke
`cmd_goto` or `navigator`.

Until sprints 025–027 prove the firmware G path trustworthy on the field,
consolidating onto it risks consolidating onto a broken target. Sprint 029 is
deliberately last for this reason.

## Solution

**Decision-doc first (stakeholder deliverable):** A short design document
assigning ownership per regime. Suggested split: firmware owns short-horizon
motion + pose fusion (it has the 10 ms loop and all the safety machinery);
host owns route planning and camera-based pose *corrections* sent as pose
resets — not its own steering loops. The sprint planner must treat this
decision as a deliverable that requires stakeholder sign-off before any code
moves. The design doc is the first ticket; implementation is gated on approval.

**cmd_goto → nav/ fold-in:** If not already pulled forward (it is independent
and anytime), fold cli.py's inline pure-pursuit + spin helpers into `nav/` as
the first code change. This is the "first casualty regardless of decision" step
from the a1 issue.

**Delete/demote per the decision:** Remove or demote the redundant controllers
and pose trackers per the signed-off ownership document. cli.py becomes
arg-parsing + calls to nav/ only (no control loops). Document the pose
authority statement in `docs/architecture.md`; the code must match it.

**Architecture consolidation:** After a1 is resolved, run the `consolidate-
architecture` skill to merge all sprint update documents into a new baseline.

## Success Criteria

- A written pose-authority design doc exists and has stakeholder sign-off before
  any controller deletion begins.
- Exactly one implementation of go-to-point per regime at close.
- `cli.py` contains no control loops (proxy: < ~800 lines, no `while` loops
  driving motion).
- Pose-authority statement in `docs/architecture.md` matches the code.
- Architecture documents consolidated into a new baseline.

## Scope

### In Scope

- Design doc: pose authority + regime ownership decision (stakeholder
  deliverable, gating all implementation work).
- `io/cli.py::cmd_goto`, `_spin_to_world_yaw`, `_daemon_spin_to_yaw`,
  `_crawl_drive_distance` → fold into `nav/`.
- `nav/navigator.py` + `nav/controllers/` — deletion or demotion per the
  decision.
- `sensors/odometry.py`, `sensors/odom_tracker.py` — pose authority
  clarification per the decision.
- `docs/architecture.md` — written pose-authority statement.
- Any remaining a6 extraction (TLM snapshot parsing, robot_mcp controller
  drift) left over from sprint 028.
- Architecture consolidation (`consolidate-architecture` skill).

### Out of Scope

- Any firmware motion changes (firmware navigation path is settled by sprints
  024–027 before this sprint begins).
- D12 items not yet landed (handle as anytime fillers).

## Test Strategy

- Design doc reviewed and signed off before code changes begin (process gate).
- Regression: all existing nav/ and CLI integration tests pass after fold-in.
- Smoke ritual before and after.
- Architecture consolidation review.

## Architecture Notes

**The decision-doc is the linchpin.** The sprint planner for sprint 029 must
surface the specific ownership decision choices (firmware vs. host, camera
corrections as pose resets vs. independent loop, how navigator.py is demoted)
to the stakeholder as an explicit question before writing any other artifact.
If no agreement is reached, the sprint is blocked — do not proceed with
implementation under ambiguity.

**Suggested architecture** (from a1 issue): firmware owns short-horizon motion
+ pose fusion; host owns route planning + camera corrections sent as pose
resets. Under this split, `navigator.py` becomes a route planner that issues
G commands and sends camera-based pose corrections via SET/ZERO, not a
steering loop. The host-side controllers in `nav/controllers/` are deleted.

**cmd_goto fold-in** is independent of the decision and safe to pull into any
earlier sprint. If it lands in sprint 027 or 028, note it here and remove from
this sprint's scope.

**Anytime filler items** (carry over from sprint 028 if not yet done):
- `d12-numerical-and-timing-hygiene`
- `set-config-validation`
- Any remaining `cmd_goto → nav/` fold-in

## Why Last

Consolidating navigation onto the firmware G path only makes sense after
sprints 026–027 have proven that path trustworthy on the field. Doing a1
earlier consolidates onto an unproven target. The "first casualty"
(cmd_goto fold-in) can be pulled forward at any time — it is independent of
the ownership decision.

## Sizing

Medium plus a decision gate — approximately 2 focused sessions after sign-off,
but the design-doc round-trip with the stakeholder may add calendar time.

## GitHub Issues

(None yet — link when created.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] **Stakeholder has signed off on the pose-authority design doc** (this gate
      is mandatory before any controller deletion tickets are created)

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Pose-authority design document and stakeholder sign-off gate | — |
| 002 | Fold cmd_goto and spin helpers out of cli.py into nav/camera_goto.py | 001 |
| 003 | Delete host-side steering controllers and demote navigator.py to route planner | 001, 002 |
| 004 | Write pose-authority statement in docs/architecture.md and consolidate architecture | 001, 003 |

Tickets execute serially in the order listed.
