---
id: '001'
title: Pose-authority design document and stakeholder sign-off gate
status: open
use-cases:
  - SUC-001
depends-on: []
github-issue: ''
issue: a1-navigation-and-pose-ownership.md
completes_issue: false
prerequisite: >
  SPRINT GATE: Sprints 026–027 must have proven the firmware G path trustworthy
  on the field before this ticket executes. Confirm with the stakeholder before
  starting.
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Pose-authority design document and stakeholder sign-off gate

## Description

This is the mandatory Definition-of-Ready gate for sprint 029. No controller
deletion or consolidation ticket (002–004) may execute until this ticket is done
and the stakeholder has approved the design document.

Write a short design document (`docs/decisions/029-pose-authority.md`) that:
1. States the confirmed ownership split per regime.
2. Lists every artifact to be deleted or demoted (with filenames and line counts).
3. Answers open questions OQ-1 through OQ-6 from the architecture update.
4. Calls out the breaking change to the MCP `navigate`/`follow_path` API and
   names all known callers.

The suggested split (from a1 issue and architecture-update): firmware owns
short-horizon motion + pose fusion (10 ms loop, hardware EKF, safety watchdog);
host owns route planning and camera-based pose *corrections* sent as pose resets
(OV / SI commands) — not its own steering loop. This is a proposal, not a given:
the stakeholder may choose a different split.

**This ticket is complete only when the stakeholder has approved the document.**
Tickets 002–004 are explicitly blocked on this ticket.

## Acceptance Criteria

- [ ] `docs/decisions/029-pose-authority.md` is written and committed.
- [ ] Document answers all six open questions from `architecture-update.md §Step 7`.
- [ ] Document names every module to be deleted/demoted with exact file paths.
- [ ] Document calls out the MCP API change and lists all known callers of
      `navigate` / `follow_path` in `robot_mcp.py`.
- [ ] **STAKEHOLDER HAS APPROVED THE DOCUMENT** (this checkbox is the gate;
      mark it only after receiving explicit sign-off).
- [ ] Sprint prerequisite confirmed: sprints 026–027 field-proven on the bench.

## Implementation Plan

### Approach

Pure documentation. No code changes in this ticket.

### Files to create

- `docs/decisions/029-pose-authority.md`

### Files to read for inventory

- `host/robot_radio/io/cli.py` (cmd_goto, _daemon_spin_to_yaw, _spin_to_world_yaw)
- `host/robot_radio/nav/navigator.py` (navigate, follow_path, follow_pose_path,
  _spin_to_heading, _run_controller)
- `host/robot_radio/io/robot_mcp.py` (all calls to _navigator.navigate/follow_path)
- `host/robot_radio/controllers/` (pure_pursuit.py, stanley.py, ltv.py)
- `source/control/MotionController.cpp` (beginGoTo, driveAdvance PURSUE section)

### Testing Plan

No tests. This is a documentation/decision deliverable.

### Documentation Updates

Create `docs/decisions/029-pose-authority.md`.
