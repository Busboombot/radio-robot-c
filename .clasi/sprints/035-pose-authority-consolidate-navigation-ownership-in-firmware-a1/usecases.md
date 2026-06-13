---
status: draft
sprint: '035'
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 035 Use Cases

## SUC-001: Developer moves robot to a world coordinate from the CLI

- **Actor**: Developer / operator
- **Preconditions**: Robot connected via relay; aprilcam daemon running; robot tag visible
- **Main Flow**:
  1. Operator runs `rogo goto <x> <y>`.
  2. CLI dispatches to `nav/camera_goto.go_to_world_camera(...)`.
  3. Camera feedback loop converges the robot on the target.
  4. CLI reports final error and success/failure.
- **Postconditions**: Robot is within the arrive tolerance of the target; cli.py contains no inline control loops.
- **Acceptance Criteria**:
  - [ ] `rogo goto <x> <y>` produces identical observable output before and after the refactor.
  - [ ] `cmd_goto` in cli.py is a thin wrapper with no `while` loop driving motors.
  - [ ] `nav/camera_goto.py` exists and contains the extracted control logic.
  - [ ] No import cycle: `camera_goto.py` does not import from `cli.py`.

## SUC-002: Developer turns robot to a heading from the CLI

- **Actor**: Developer / operator
- **Preconditions**: Robot connected via relay; aprilcam daemon running; robot tag visible
- **Main Flow**:
  1. Operator runs `rogo turnto <deg>`.
  2. CLI dispatches to `nav/camera_goto.spin_to_yaw_camera(...)`.
  3. Camera feedback loop converges the robot on the heading.
  4. CLI reports final heading error and success/failure.
- **Postconditions**: Robot heading is within tolerance of target; cli.py contains no inline spin loop.
- **Acceptance Criteria**:
  - [ ] `rogo turnto <deg>` produces identical observable output before and after the refactor.
  - [ ] `cmd_turnto` in cli.py is a thin wrapper with no `while` loop driving motors.
  - [ ] Dead function `_spin_to_world_yaw` is deleted from cli.py.

## SUC-003: Agent issues a navigate_to or follow_path MCP tool call

- **Actor**: Claude agent (MCP client)
- **Preconditions**: Robot MCP server running; firmware G path proven on bench; A1a landed
- **Main Flow**:
  1. Agent calls `navigate_to(x, y)` or `follow_path(path)` via MCP.
  2. `robot_mcp.py` issues firmware G commands (one per waypoint) and waits for `EVT done G`.
  3. Tool returns success/failure dict with same schema as before.
- **Postconditions**: Robot reached the target via firmware steering; no host-side S/T loop ran.
- **Acceptance Criteria**:
  - [ ] MCP tool signatures (`navigate_to`, `follow_path`) unchanged.
  - [ ] Implementation no longer calls `_navigator.navigate` or `_navigator.follow_path` steering loops.
  - [ ] `follow_pose_path` tool removed or replaced per design doc decision.
  - [ ] `visit_tags`, `grab_at`, `release_at`, `approach` MCP tools continue to work.

## SUC-004: Developer reads the architecture and understands navigation ownership

- **Actor**: Developer (future maintainer)
- **Preconditions**: A1a and A1b are complete; code is in the post-deletion state
- **Main Flow**:
  1. Developer reads `docs/architecture.md`.
  2. Developer finds a "Navigation Architecture" section naming the firmware EKF as authoritative pose source.
  3. Developer finds `nav/camera_goto.py` named as the CLI navigation module.
  4. Developer finds that the host does not run a parallel steering loop.
- **Postconditions**: Documentation matches code; consolidated architecture baseline is updated.
- **Acceptance Criteria**:
  - [ ] `docs/architecture.md` contains a Navigation Architecture / Pose Authority section.
  - [ ] Section matches post-A1b code state.
  - [ ] Consolidated architecture document produced in `.clasi/architecture/`.
  - [ ] Issue `a1-navigation-and-pose-ownership.md` is marked done.
