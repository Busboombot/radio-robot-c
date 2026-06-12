---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 029 Use Cases

## SUC-001: Stakeholder signs off on pose-authority design document
Parent: A1 (navigation ownership issue)

- **Actor**: Stakeholder (team lead acting for stakeholder)
- **Preconditions**: Sprint 029 is in planning-docs phase; sprints 026–027 have
  proven the firmware G path trustworthy on the field.
- **Main Flow**:
  1. Sprint planner produces a short design doc enumerating every go-to-point
     implementation and pose tracker found in the codebase.
  2. The design doc proposes an ownership split and lists the specific artifacts
     (controllers, modules) that will be deleted or demoted.
  3. Stakeholder reads the doc and approves or requests changes.
  4. Stakeholder sign-off is recorded as a ticket-acceptance checkbox before any
     deletion ticket enters execution.
- **Postconditions**: A written, stakeholder-approved pose-authority document exists
  and is checked in. The remaining sprint tickets are unblocked.
- **Acceptance Criteria**:
  - [ ] Design doc is committed under `docs/decisions/` (or a path approved by
        stakeholder).
  - [ ] The doc names the authoritative pose source per regime (firmware-only vs.
        camera-correction-as-reset).
  - [ ] Stakeholder approval is recorded in the ticket acceptance (checkbox marked).

## SUC-002: CLI go-to command delegates to nav/ (no inline control loop)
Parent: A1, A6

- **Actor**: Developer / agent using `rogo goto`
- **Preconditions**: `cli.py` currently contains ~165 lines of inline pure-pursuit
  plus `_spin_to_world_yaw`, `_daemon_spin_to_yaw`, `_crawl_drive_distance`.
- **Main Flow**:
  1. Developer calls `rogo goto <x> <y>` or `rogo turnto <deg>`.
  2. `cli.py` parses the arguments and delegates to a function in `nav/`.
  3. `nav/` executes the closed-loop move using the aprilcam daemon, exactly as
     the current inline code does.
  4. Result (final error, elapsed time) is printed to stdout.
- **Postconditions**: `cli.py` contains no `while` loop that drives motors.
  The same observable behaviour is preserved.
- **Acceptance Criteria**:
  - [ ] `cmd_goto`, `_daemon_spin_to_yaw`, `_spin_to_world_yaw` are removed from
        `cli.py`; their replacements live under `nav/`.
  - [ ] `rogo goto` and `rogo turnto` produce identical output on the bench.
  - [ ] `cli.py` line count drops (A6 trajectory: net reduction from 2262).

## SUC-003: Redundant host-side steering controllers are deleted or demoted
Parent: A1

- **Actor**: Developer maintaining the navigation stack
- **Preconditions**: Stakeholder design doc (SUC-001) is signed off.
  Firmware G path proven on the field.
- **Main Flow**:
  1. Per the approved ownership split, `nav/navigator.py` steering loops
     (pure-pursuit, stanley, ltv) are deleted or reduced to route-planning only.
  2. `nav/controllers/` (pure_pursuit.py, stanley.py, ltv.py) are deleted or
     archived if they have no remaining callers.
  3. `robot_mcp.py` navigation calls are updated to use the firmware G path
     (via `go_to` / `go_to_world`) instead of the Navigator steering loop.
  4. Tests are updated to match.
- **Postconditions**: Exactly one go-to-point implementation per regime exists.
  `nav/controllers/` has no live callers outside navigator.py; navigator.py has
  no steering control loop.
- **Acceptance Criteria**:
  - [ ] `nav/controllers/pure_pursuit.py`, `stanley.py`, `ltv.py` are deleted or
        have zero callers in the production path.
  - [ ] `navigator.py` contains no `while` loop sending motor commands.
  - [ ] MCP tool `navigate` / `follow_path` either removed or rewritten to issue
        firmware G commands.
  - [ ] Smoke ritual passes after deletion.

## SUC-004: Pose authority is documented in docs/architecture.md and matches the code
Parent: A1

- **Actor**: Developer diagnosing a navigation bug
- **Preconditions**: Ownership decision (SUC-001) is signed off; controller
  consolidation (SUC-002, SUC-003) is done.
- **Main Flow**:
  1. Developer reads `docs/architecture.md` and finds a clear statement of which
     pose source is authoritative in which regime.
  2. Developer reads the code and confirms the statement matches: the firmware EKF
     is the short-horizon pose authority; camera corrections arrive as pose resets
     (OV / SI); the host does not run an independent steering loop.
- **Postconditions**: `docs/architecture.md` contains a pose-authority section
  matching the code.
- **Acceptance Criteria**:
  - [ ] `docs/architecture.md` has a "Pose Authority" or "Navigation Architecture"
        section added/updated.
  - [ ] Section explicitly names: firmware EKF as the short-horizon authority;
        camera corrections mechanism (OV command / pose-reset path); host-side
        route planning role.
  - [ ] Architecture documents are consolidated (`consolidate-architecture` skill)
        after this sprint closes.
