---
id: "005"
title: "Navigation Layer"
status: open
branch: sprint/005-navigation-layer
use-cases: []
issues:
  - plan-c-port-of-radio-robot-firmware
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 005: Navigation Layer

## Goals

Deliver pluggable PathFollower and PoseProvider interfaces with PurePursuit and Stanley
implementations, wired into Robot and CommandProcessor so the Python host can command
autonomous navigation. Switching between pose sources (OTOS vs. dead reckoning) or path
algorithms (PurePursuit vs. Stanley) requires only changing the active pointer in Robot —
no protocol changes.

## Problem

After sprint 3 the robot responds to all commands but has no autonomous path following. The
TypeScript original has no navigation layer either — it was always controlled by the Python
host sending S commands at 10 Hz. This sprint adds on-device navigation, which is a key
design goal of the C++ rewrite: offload path following to the firmware so the Python host
sends waypoints rather than continuous speed commands.

## Solution

Create `source/nav/` with the four pure-virtual interfaces and four concrete implementations
specified in the issue. Wire the active PathFollower and PoseProvider into Robot as pointer
members (swappable at runtime). Add a `NAV` command to CommandProcessor that accepts waypoints
and starts the follower. Streaming output in `tick()` includes the PoseProvider source name
so the Python host knows which sensor is active.

## Success Criteria

- PurePursuit completes a 4-waypoint rectangular route (e.g. 0,0 → 500,0 → 500,500 → 0,500 → 0,0) without manual correction
- Stanley completes the same route
- Switching between PurePursuit and Stanley requires only changing the active follower pointer in Robot; no protocol or CommandProcessor changes
- Streaming output includes `SRC:otos` or `SRC:dr` (dead-reckoning) to identify PoseProvider in use
- `OtosPoseProvider` marks pose invalid when OtosSensor returns a read error; `DeadReckoningPoseProvider` is always valid
- Waypoints copy into the follower at `setPath()` time; the caller's buffer can be freed immediately after

## Scope

### In Scope

**Navigation (`source/nav/`)**
- `PoseProvider.h` — pure virtual interface + `Pose` struct `{int32_t x_mm, y_mm, h_cdeg; bool valid}`
  - `virtual void update() = 0`
  - `virtual bool getPose(Pose& out) = 0`
  - `virtual const char* sourceName() const = 0`
- `OtosPoseProvider.h/.cpp` — PoseProvider backed by OtosSensor; converts raw LSB → mm/centidegrees; marks `valid=false` on read error
- `DeadReckoningPoseProvider.h/.cpp` — PoseProvider backed by Odometry; always `valid=true`
- `PathFollower.h` — pure virtual interface + `Waypoint` struct `{int32_t x_mm, y_mm}`
  - `virtual void setPath(const Waypoint* wps, uint8_t count) = 0`
  - `virtual bool compute(const Pose& pose, int16_t& leftMms, int16_t& rightMms) = 0`
  - `virtual void reset() = 0`
  - `virtual bool isFinished() const = 0`
  - `virtual const char* name() const = 0`
- `PurePursuitFollower.h/.cpp` — lookahead κ = 2×d_lateral/Lf²; static `Waypoint _path[32]` (256 bytes); tunable: lookahead_mm, trackwidth_mm, base_speed_mms, stop_dist_mm; MAX_WAYPOINTS=32
- `StanleyFollower.h/.cpp` — δ = θ_e + atan2(k×e, v_soft+v); static `Waypoint _path[32]`; tunable: k, omega_gain, goal_tol_mm

**App integration**
- `Robot` — adds `PathFollower* activeFollower` and `PoseProvider* activePoseProvider` pointers; defaults: `PurePursuitFollower` + `OtosPoseProvider` (falls back to `DeadReckoningPoseProvider` if OTOS absent); `run()` calls `activePoseProvider->update()` then `activeFollower->compute()` when following is active
- `CommandProcessor` — `NAV` command: accepts a sequence of waypoints (`NAV+X1+Y1+X2+Y2...`), calls `activeFollower->setPath()`, starts following; `NAVSTOP` command halts following; streaming output adds `SRC:<sourceName>` field when following is active

**Waypoint copy semantics**
- Each follower's `setPath()` copies waypoints into its static `_path[]` array. Caller buffer can be freed immediately. MAX_WAYPOINTS=32.

### Out of Scope

- ExternalCameraPoseProvider (receives pose via SI command from Python host) — future sprint
- Ratio PID motor control (sprint 4) — navigation uses the sprint 2 MotorController; accuracy improves automatically when sprint 4 is merged
- G go-to command (sprint 4) — uses arc math + ratio PID, not PathFollower
- Path recording or persistence
- Multi-robot coordination

## Test Strategy

Hardware-in-the-loop:

1. Build and deploy with OTOS attached
2. Dead-reckoning follower: set `activePoseProvider = DeadReckoningPoseProvider`; send `NAV+500+0` — robot drives ~500 mm forward and stops
3. 4-waypoint route with PurePursuit: `NAV+500+0+500+500+0+500+0+0` — robot traces rectangle; observe via visual tracking or string measurement
4. Same route with Stanley: swap follower pointer in Robot and repeat
5. OTOS PoseProvider: confirm `SRC:otos` appears in streaming output
6. Pose invalid fallback: disconnect OTOS mid-run; follower should stop or switch to dead reckoning depending on validity flag
7. `NAVSTOP` during route — robot halts within one tick
8. Waypoint count > 32 — firmware clamps to MAX_WAYPOINTS without panic

**Reference implementations (Python host, for algorithm validation):**
- `radio-robot/robot_radio/pure_pursuit.py`
- `radio-robot/robot_radio/stanley.py`
- `radio-robot/robot_radio/controllers.py`

## Architecture Notes

**Dependency direction: `nav/` → `control/` → `hal/`.** PoseProvider implementations depend
on OtosSensor (HAL) and Odometry (control). PathFollower implementations depend only on
`Pose`, `Waypoint`, and the output of `MotorController::setTarget()`. Neither `nav/` class
depends on CommandProcessor.

**No virtual dispatch in the hot MotorController tick.** PathFollower::compute() runs in
Robot::run() before the motor tick. The result (leftMms, rightMms) is passed to
`MotorController::setTarget()` which is a non-virtual call. Virtual dispatch only occurs in
`PoseProvider::update()` and `PathFollower::compute()`, which run at the 20 ms tick rate —
acceptable overhead.

**Static waypoint arrays, no heap.** Each follower holds `Waypoint _path[32]` (256 bytes).
With two followers (PurePursuit + Stanley) instantiated in Robot, total static nav memory is
512 bytes — well within micro:bit V2's 128 KB RAM.

**PoseProvider validity flag drives follower behaviour.** When `Pose.valid == false`,
PathFollower::compute() should call `stop()` and return false. This prevents the robot from
blindly continuing without a valid position estimate.

**Reference files:**
- `radio-robot/robot_radio/pure_pursuit.py` — reference lookahead algorithm
- `radio-robot/robot_radio/stanley.py` — reference Stanley algorithm

## GitHub Issues

None linked yet.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
