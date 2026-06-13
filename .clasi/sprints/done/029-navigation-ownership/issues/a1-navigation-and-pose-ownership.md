---
status: done
sprint: 029
tickets:
- 029-001
- 029-002
- 029-003
- 029-004
---

# A1 — Decide and enforce single ownership of go-to-point and pose estimation

## Context

The same closed-loop "drive to a world point" capability exists in three stacks that
share no code, parameters, or pose state:

1. **Firmware** — `MotionController::beginGoTo` + PURSUE law, driving from the
   encoder/OTOS EKF pose.
2. **Host library** — `nav/navigator.py` (1349 lines) + `controllers/` (pure_pursuit,
   stanley, ltv, pid), driving from camera pose via `sensors/odometry.py`.
3. **CLI inline** — `io/cli.py::cmd_goto` (~165 lines of pure-pursuit with its own
   gains: TICK_S, AIM_GATE, STEER_KP, SLOW_RADIUS…), plus `_spin_to_world_yaw`,
   `_daemon_spin_to_yaw`, `_crawl_drive_distance`.

Likewise three pose estimators with no defined authority or reconciliation: firmware
EKF, host `sensors/odom_tracker.py` (TLM integration), host `sensors/odometry.py`
(camera + OTOS fallback). Every navigation bug must be hunted in three stacks; the
P0/P1 firmware fixes do nothing for an agent that happens to invoke `cmd_goto` or
`navigator`. This is the largest conceptual architecture problem in the project.

## Fix

1. **Design decision first** (stakeholder + team-lead): assign ownership per regime.
   Suggested split: firmware owns short-horizon motion + pose fusion (it has the
   10 ms loop and the safety machinery); host owns route planning and camera-based
   pose *corrections* sent as pose resets — not its own steering loops.
2. Delete or demote the redundant controllers per that decision. First casualty
   regardless of decision: `cmd_goto`'s inline controller folds into `nav/`.
3. Document which pose source is authoritative when, and the correction mechanism,
   in `docs/architecture.md`.

## Acceptance

- Exactly one implementation of go-to-point per regime; `cli.py` contains no control
  loops. A written pose-authority statement exists and the code matches it.

## Priority suggestion

**High importance, but gated on a design decision, and sequenced after the P0/P1
firmware fixes prove out** — don't consolidate onto the firmware G path until it
works on the field. The cheap first step (fold `cmd_goto` into `nav/`) can be pulled
forward into any sprint.

## Source
Finding **A1** in `docs/code_review/2026-06-11-architecture-modularity-review.md`.
