# Decision: Pose Authority and Navigation Ownership (Sprint 029)

**Status: AWAITING STAKEHOLDER SIGN-OFF**
**Date: 2026-06-11**
**Sprint: 029**
**Issue: a1-navigation-and-pose-ownership**

---

## 1. Problem

Three independent go-to-point implementations and four separate pose estimators
coexist in the codebase with no defined authority or reconciliation protocol.
Every navigation bug must be hunted in three stacks. Firmware fixes delivered in
sprints 024–027 have no effect when an agent uses the host-side navigator
(G2/navigator.py) or the CLI inline controller (G3/cmd_goto).

This document defines the recommended ownership split, answers all six open
questions from the sprint architecture, and inventories every artifact to be
deleted or demoted so that tickets 002–004 can execute once the stakeholder
signs off.

---

## 2. Current Redundancy

### Go-to-point implementations

| ID | Location | Key file | Lines | Pose source | State |
|----|----------|----------|-------|-------------|-------|
| G1 | Firmware | `source/control/MotionController.cpp` | 927 total (~150 for `beginGoTo` + PURSUE) | Firmware EKF (OTOS + encoders) | Field-proven after S024–S027 |
| G2 | Host library | `host/robot_radio/nav/navigator.py` | 1349 | Camera (`sensors/odometry.py`) | Active; used by robot_mcp.py |
| G3 | CLI inline | `host/robot_radio/io/cli.py::cmd_goto` | ~161 | Camera via aprilcam daemon | Active; used by `rogo goto` |
| G3a | CLI inline | `cli.py::_daemon_spin_to_yaw` | ~50 | Camera via aprilcam daemon | Called by cmd_goto and cmd_turnto |
| G3b | CLI inline | `cli.py::_spin_to_world_yaw` | ~78 | Local Playfield/Camera (legacy) | No callers — dead code |
| G4 | Host library | `host/robot_radio/robot/nezha_kinematic.py::go_to_world` | ~40 | OdomTracker (TLM dead-reckoning) | Thin wrapper around G1; not in scope this sprint |

### Controllers (host-side steering, all inside G2)

| File | Lines | Type | Used by |
|------|-------|------|---------|
| `host/robot_radio/controllers/pure_pursuit.py` | 216 | Pure Pursuit | navigator.py only |
| `host/robot_radio/controllers/stanley.py` | 198 | Stanley | navigator.py only |
| `host/robot_radio/controllers/ltv.py` | 293 | LTV | navigator.py only |
| `host/robot_radio/controllers/pid.py` | 54 | PID | navigator.py + speed loop |

All three steering controllers (pure_pursuit, stanley, ltv) have zero callers
outside navigator.py.

### Pose estimators

| ID | Location | File | Source | Used by |
|----|----------|------|--------|---------|
| P1 | Firmware | `source/odometry/Odometry.cpp` + `source/ekf/EKF.cpp` | OTOS + encoders, fused via EKF | G1 |
| P2 | Host library | `host/robot_radio/sensors/odometry.py` (225 lines) | Camera (AprilTag) + OTOS fallback | G2 |
| P3 | Host library | `host/robot_radio/sensors/odom_tracker.py` (357 lines) | Firmware TLM (dead-reckoning) | G4 |
| P4 | CLI inline | `cli.py` (via daemon) | aprilcam daemon | G3 |

No defined reconciliation between P1–P4. OV/SI commands exist to seed P1 from
an external source but are currently called manually (`rogo sync pose`) rather
than automatically.

---

## 3. Recommended Ownership Split

### RECOMMENDATION (Option A)

**Firmware owns short-horizon motion and pose fusion. Host owns route planning
and camera-based pose corrections delivered as pose resets.**

Specifically:

- **Firmware G path (G1) is the sole steering loop.** It runs at 10 ms on the
  firmware, fuses OTOS + encoders via EKF, enforces the safety watchdog, and has
  all sprint 024–027 fixes (arrive-tolerance, PRE_ROTATE timeout, PURSUE TIME
  backstop, signed-delta uint32 watchdog fix). No host code sends S/T commands
  in a loop to steer the robot toward a waypoint.

- **Host owns route planning.** When the agent needs multi-waypoint navigation,
  the host sequences individual firmware G commands — one G per waypoint,
  waiting for `EVT done G` before issuing the next. This is route planning, not
  steering.

- **Camera corrections are pose resets, not steering inputs.** The host reads
  camera pose (via the aprilcam daemon) and sends OV/SI to seed the firmware
  EKF. The camera is a correction trigger, not a continuous steering source.

- **`cmd_goto` / `_daemon_spin_to_yaw` are refactored into `nav/camera_goto.py`
  regardless of the ownership decision.** This is pure A6 cleanup (control loops
  out of CLI argument-parsing code). It does not change behaviour.

### Rationale

G1 is the correct long-term owner because:
1. It runs at 10 ms vs. 30 ms for the host loop — 3x tighter control.
2. It has a hardware safety watchdog; the host loop does not.
3. Sprints 024–027 were explicitly sequenced to prove it before this
   consolidation. All known issues (PRE_ROTATE no-timeout, uint32 underflow,
   PURSUE runaway) were fixed in those sprints.
4. Gain constants need only be calibrated once (in the firmware / robot JSON),
   not in three separate stacks.

### Alternative: Option B — Retain host-side navigator as primary steering

Keep G2 (navigator.py) as the steering loop and deprecate G1 for anything above
a single-point drive. Rejected because: (a) G1 is the proven, safety-bounded
implementation; (b) the host loop lacks a watchdog and runs at 30 ms; (c) this
does not reduce the three-stack maintenance burden. Only choose Option B if
field testing reveals an unfixable deficiency in G1 for multi-waypoint work.

### Alternative: Option C — Keep both (status quo)

Rejected. Status quo means every navigation bug is hunted in three stacks.
Calibration of gains must be done three times. The sprint rationale depends on
consolidation.

---

## 4. Open Questions — Concrete Answers

### OQ-1 (High): What happens to MCP tools `navigate_to` and `follow_path`?

**RECOMMENDATION:** Reimplement as thin wrappers that issue firmware G commands
and wait for `EVT done G`, one G per waypoint (Option b).

- `navigate_to(x, y)` → converts world-cm to robot-relative mm, calls
  `_robot.send("G+<x>+<y>+<speed>")`, polls for `G+DONE`.
- `follow_path(path)` → sequences one G per consecutive pair of waypoints,
  waiting for `EVT done G` before advancing.

Do **not** remove the tools entirely (Option a): agents rely on them and the
breaking-change surface would be larger than necessary. Do **not** retain the
host-side PID loop as-is (Option c): that defeats the purpose of the sprint.

The new implementation is thinner, faster to reason about, and shares the
firmware's safety watchdog. The MCP tool **signatures** do not change; only
the implementation behind them changes. See Section 6 for the full breaking-
change analysis.

### OQ-2 (High): Is `navigator.py`'s route-planning logic still needed, or should the file be deleted entirely?

**RECOMMENDATION:** Delete the steering loop methods; retain a minimal route
planner.

- **Delete** (or replace with G-command stubs): `navigate`, `follow_path`,
  `follow_pose_path`, `_spin_to_heading`, `_run_controller`, and all
  `ChaseController` / `_build_controller` code.
- **Retain** in a reduced form: `visit_tags`, `approach`, `grab_at`,
  `release_at`, `read_pose`, and the camera/Playfield management plumbing
  (`_get_playfield`, `reset_camera`, `status`).
- **Reimplemented** to call firmware G: `navigate` and `follow_path` become
  thin wrappers (per OQ-1).

If the stakeholder prefers a clean break, the entire file can be deleted and
robot_mcp.py can contain the G-command wrappers directly. The routing functions
(`visit_tags`, `grab_at`) would move to a new `nav/route.py`. Either layout is
acceptable; the key constraint is that no method in the retained code runs a
host-side S/T steering loop.

### OQ-3 (Medium): Does `NezhaKinematic.go_to_world` (G4) overlap with the consolidated path?

**RECOMMENDATION:** Leave G4 unchanged in this sprint; schedule demotion in a
future sprint.

G4 wraps firmware G but uses OdomTracker (TLM dead-reckoning) for pose tracking
rather than the firmware EKF. After sprint 029, it is the only remaining host
path that tracks a separate pose. It is not actively used in any MCP tool or CLI
command that has been inventoried. A future sprint should evaluate whether
OdomTracker provides value beyond the firmware EKF and either demote or delete
G4 at that time. It is out of scope here to avoid increasing sprint risk.

### OQ-4 (Medium): Is `rogo sync pose` / OV command sufficient for the "camera corrections as pose resets" model, or does a tighter automatic correction loop need to be specified?

**RECOMMENDATION:** `rogo sync pose` is sufficient for this sprint. Specify a
tighter loop as a future enhancement.

The existing `cmd_sync_pose` (cli.py line 263) reads the daemon world pose and
sends OV to firmware. This is correct and tested. For the navigation ownership
consolidation, the host can call `sync pose` before issuing a G command whenever
camera confidence is high enough. A fully automatic correction loop (e.g., send
OV every N seconds during a long traverse) is a desirable enhancement but is
**not required** to prove the consolidated path. Add it as a future ticket once
the G1 path is the sole steering loop and the correction mechanism can be
validated cleanly.

### OQ-5 (Low): Is `_spin_to_world_yaw` dead code? Safe to delete?

**CONFIRMATION: Yes, it is dead code. Safe to delete.**

Evidence:
1. The grep results show `_spin_to_world_yaw` is defined at cli.py line 857 and
   referenced only in the docstring of `_daemon_spin_to_yaw` (line 1028, as a
   comment, not a call). There is no other call site in the repository.
2. `_spin_to_world_yaw` reads the robot tag yaw from a local `Playfield`/`Camera`
   object (the old homography path). The local homography was deleted on
   2026-05-29 (stale ~30% scale error; see data/CLAUDE.md). The function would
   fail at import-time or first call due to the missing `_get_tag_yaw` import
   from `robot_radio.io.calibrate`.
3. `cmd_turnto` (the only command that historically called it) was superseded by
   `_daemon_spin_to_yaw`, which reads from the aprilcam daemon. The `cmd_turnto`
   function at line 937 calls `_daemon_spin_to_yaw` (line 990) directly.

**Action:** Delete `_spin_to_world_yaw` (cli.py lines 857–934, ~78 lines) in the
fold-in ticket. No callers to update.

### OQ-6 (Dependency): Sprint gate — sprints 026–027 must have proven the firmware G path trustworthy on the bench before any deletion ticket executes.

**GATE — STAKEHOLDER MUST CONFIRM.**

This checkbox is the second unresolved gate for this ticket. Sprints 026 and 027
delivered: PRE_ROTATE timeout (026-001), MotionEventSink abstraction (026-002),
arrive-tolerance tuning (026-003/004), and EKF / velocity-loop fixes (027-xxx).
The firmware is believed field-worthy but the stakeholder must confirm hardware
bench validation before any code is deleted. This is the prerequisite field-test
gate in the ticket frontmatter.

---

## 5. Deletion and Demotion Inventory

All line counts are from `wc -l` on the source files as of 2026-06-11.

### 5.1 Functions to delete from `cli.py` (2033 lines total)

| Function | Lines | Reason |
|----------|-------|--------|
| `cmd_goto` (lines 1077–1238) | ~161 | Move to `nav/camera_goto.py` (refactor, not delete — same logic, new location) |
| `_daemon_spin_to_yaw` (lines 1024–1074) | ~50 | Move to `nav/camera_goto.py` |
| `_spin_to_world_yaw` (lines 857–934) | ~78 | Delete — confirmed dead code (OQ-5) |
| `_crawl_drive_distance` (lines 560–599) | ~39 | Move to `nav/camera_goto.py` (called by cmd_goto) |

Net effect: cli.py shrinks by ~289 lines from deletion/move. The `goto` and
`turnto` subcommands remain as thin dispatch wrappers that import from
`nav/camera_goto.py`.

### 5.2 `nav/navigator.py` (1349 lines total)

| Component | Disposition |
|-----------|-------------|
| `navigate` method (dual-PID steering loop) | Delete / replace with G-command wrapper |
| `follow_path` method (path-following loop) | Delete / replace with G-command wrapper |
| `follow_pose_path` method (3-phase planner + loop) | Delete / replace or stub |
| `_spin_to_heading` method (host-side spin loop) | Delete |
| `_run_controller` method (shared loop runner) | Delete |
| `ChaseController` class (~173 lines) | Delete with navigate |
| `_build_controller` helper | Delete with follow_path |
| `visit_tags`, `approach`, `grab_at`, `release_at`, `read_pose` | Retain |
| `_get_playfield`, `reset_camera`, `status` | Retain |

Approximate retained size: ~400 lines. Approximate deleted size: ~950 lines.

### 5.3 Controllers to delete (all have zero callers outside navigator.py)

| File | Lines | Disposition |
|------|-------|-------------|
| `host/robot_radio/controllers/pure_pursuit.py` | 216 | Delete after navigator steering loop removed |
| `host/robot_radio/controllers/stanley.py` | 198 | Delete after navigator steering loop removed |
| `host/robot_radio/controllers/ltv.py` | 293 | Delete after navigator steering loop removed |
| `host/robot_radio/controllers/pid.py` | 54 | **Retain** — used by speed-loop primitives; may still be needed |

Note: `host_tests/test_imports_smoke.py` imports `PurePursuitTracker` and
`StanleyController` (lines 356, 367). Those smoke-test entries must be updated
or removed when the modules are deleted.

### 5.4 Pose estimators — demotion, not deletion

| File | Lines | Disposition |
|------|-------|-------------|
| `host/robot_radio/sensors/odometry.py` | 225 | **Retain** — demoted from steering-loop input to camera-monitoring / correction trigger. Used by otos_align and read_pose_fused in robot_mcp.py. |
| `host/robot_radio/sensors/odom_tracker.py` | 357 | **Retain** — used by G4 (NezhaKinematic); not in scope this sprint. |

---

## 6. MCP API Breaking Change

### What breaks

The MCP tools `navigate_to`, `follow_path`, and `follow_pose_path` currently
call `_navigator.navigate`, `_navigator.follow_path`, and
`_navigator.follow_pose_path` respectively, which run a host-side PID/pure-
pursuit steering loop that sends S/T commands to firmware at ~30 ms ticks.

After this sprint, those methods will either be deleted (if navigator.py is
gutted) or reimplemented to issue G commands and wait for `EVT done G`. The
**tool names and signatures do not change**, but the **execution behaviour
changes**:

| Aspect | Before (host PID loop) | After (firmware G commands) |
|--------|------------------------|-----------------------------|
| Steering | Host sends S+T at ~30 ms | Firmware loop at 10 ms |
| Pose source during motion | Camera (P2) | Firmware EKF (P1) |
| Safety watchdog | None in host loop | Firmware hardware watchdog |
| Timeout behaviour | Host wall-clock | Firmware PURSUE TIME backstop + host deadline |
| Camera use | Real-time steering input | Correction trigger (OV before G) |
| Multi-waypoint | Path sampled, controller tracks | One G per waypoint, sequential |

**Camera-only workflows**: Any agent that relied on the host loop because the
firmware G path was not trusted should now use the firmware G path with an
optional `sync pose` call before each G command to seed the firmware EKF from
the camera.

### Known callers

Grepped across the entire repository (excluding `.pyc`, `__pycache__`, and
`.clasi/`).

| Caller | Location | Tool called | Impact |
|--------|----------|-------------|--------|
| `robot_mcp.py` `call_tool`, `name == "navigate_to"` | line 690 | `_navigator.navigate` | Implementation changes; signature stable |
| `robot_mcp.py` `call_tool`, `name == "follow_path"` | line 703 | `_navigator.follow_path` | Implementation changes; signature stable |
| `robot_mcp.py` `call_tool`, `name == "follow_pose_path"` | line 893 | `_navigator.follow_pose_path` | Implementation changes or tool removed |

No other callers of `_navigator.navigate` or `_navigator.follow_path` were found
outside `robot_mcp.py`. No test files call these methods (grep confirmed).

The MCP tool `approach` calls `_navigator.approach`, which uses a two-phase
distance-based controller. This is NOT a continuous steering loop and is not
covered by this consolidation — it will be evaluated separately.

### Agent impact

If any Claude agent (outside this codebase) calls the `navigate_to` or
`follow_path` MCP tools, it will observe behaviour changes but not signature
changes. Specifically: the robot will move more precisely (firmware EKF, 10 ms)
and the call will take the same or shorter time. The success/error dict format
should be preserved to avoid breaking agent parsing.

---

## 7. New Module: `nav/camera_goto.py`

The fold-in of `cmd_goto`, `_daemon_spin_to_yaw`, and `_crawl_drive_distance`
into `nav/camera_goto.py` is **independent of the ownership decision** and is
the "first casualty" regardless of which option the stakeholder selects. It is
pure refactoring: same control law, new file. The CLI commands `rogo goto` and
`rogo turnto` will delegate to the new module.

The new module boundary:
- **Inside:** control loop, convergence logic, heading math, crawl pulse train.
- **Outside:** serial connection handling, argument parsing, global state.
- Accepts a pose-reader callable `(timeout_s) -> (x, y, yaw_rad) | None` and
  a protocol reference. No argparse imports.

---

## 8. Sprint Gate Summary

Two gates must be cleared before any deletion ticket (002–004) executes:

1. **Stakeholder sign-off on this document** (this gate).
2. **Sprints 026–027 field-proven on the bench** (OQ-6 hardware gate).

Tickets 002–004 are explicitly blocked until both gates are satisfied.

---

## 9. Acceptance Checklist (for programmer verification)

- [x] `docs/decisions/029-pose-authority.md` is written and committed.
- [x] Document answers all six open questions (OQ-1 through OQ-6).
- [x] Document names every module to be deleted/demoted with exact file paths
      and line counts.
- [x] Document calls out the MCP API change and lists all known callers of
      `navigate` / `follow_path` / `follow_pose_path` in `robot_mcp.py`.
- [ ] **STAKEHOLDER HAS APPROVED THE DOCUMENT** — mark only after explicit
      sign-off.
- [ ] Sprint prerequisite confirmed: sprints 026–027 field-proven on the bench.
