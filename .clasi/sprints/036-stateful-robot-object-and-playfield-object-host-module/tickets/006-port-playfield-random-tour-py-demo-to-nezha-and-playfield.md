---
id: '006'
title: Port playfield_random_tour.py demo to Nezha and Playfield
status: done
use-cases:
- SUC-012
depends-on:
- '001'
- '002'
- '003'
- '005'
github-issue: ''
issue: plan-stateful-robot-object-playfield-object-for-the-host-module.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Port playfield_random_tour.py demo to Nezha and Playfield

## Description

`host_tests/playfield_tour/playfield_random_tour.py` is the end-to-end integration
proof for this sprint. It currently:
- Opens `serial.Serial(RELAY, ...)` directly and sends `!GO` to enter the relay
  data plane.
- Calls `DaemonControl.get_tags()` and `dc.where_is()` directly.
- Manages its own `STREAM`, `SI`, keepalives, and `paths.json` writes.

This ticket rewrites it to use:
- `make_robot(...)` from `robot_radio.robot.connection` (relay detection + `!GO` +
  construction + keepalive daemon).
- `Playfield.open()` for all camera/world-geometry operations.
- `robot.go_to(..., on_tick=_on_tick)` for each leg, with the tick callback
  handling the camera poll, both tracks, and the bounds check.
- `robot.update_world_pose(loc.x, loc.y, loc.yaw)` for the SI firmware fix.

**New per-leg flow:**
```python
loc = field.get_tag(ROBOT_TAG)        # SUC-008: live tag
robot.update_world_pose(loc.x, loc.y, loc.yaw)  # SUC-007: camera fix
target = field.get_object(slug)        # SUC-009: static feature
fwd, lft = compute_g(loc, target)     # robot-relative mm (unchanged logic)
robot.go_to(round(fwd), round(lft), SPEED, on_tick=on_tick_cb)  # SUC-003/004
```

The `on_tick_cb(robot)` function:
1. Reads `field.get_tag(ROBOT_TAG)` for camera track.
2. Reads `robot.state.pose` for odometry track (x_mm/10 → cm).
3. Calls `field.add_path("camera", CAM_TRACK)` and `field.add_path("odometry",
   ODO_TRACK)` (SUC-011).
4. Checks bounds: if camera position is outside the safe box, returns `False`
   (abort — SUC-004).

**What is removed:**
- `import serial` and `serial.Serial(RELAY, ...)`.
- `send()`, `keepalive()`, `stream_until_done()` functions.
- `write_paths()`, `draw_paths()` functions (replaced by `field.add_path` /
  `field.clear_paths`).
- `_POSE_RE`, `extract_odometry()` (state comes from `robot.state.pose`).
- Manual `!GO` / `STREAM` / `SI` / `X` sends in the main block.
- `p.read(8192)` drain calls.

**What is retained (logic unchanged):**
- Constants: `ROBOT_TAG`, `SPEED`, `STREAM_MS`, `HOPS`, `ARRIVE_CM`, `MAX_G_PER_LEG`,
  `ABORT_X`, `ABORT_Y`.
- `select_location()` — now calls `field.get_tag(ROBOT_TAG)` N times and averages.
- `select_target()`, `distance()`, `in_bounds()`, `compute_g()` — pure math, unchanged.
- Config pushes: `robot.set_config(sTimeout=60000, turnGate=35, alphaYaw=0,
  yawRateMax=60)` (replaces manual `SET` sends).
- The `hop()` outer structure with `MAX_G_PER_LEG` attempts.
- The `try/finally` cleanup: `robot.stop()`, `field.close()`.

## Acceptance Criteria

- [x] `import serial` is removed; no `serial.Serial(...)` in the file.
- [x] All `send(...)`, `keepalive()`, `stream_until_done()` free functions removed.
- [x] Robot constructed via `make_robot(...)`, not a raw serial port.
- [x] `Playfield.open()` is used; no direct `DaemonControl` calls in main script
      (all daemon access through `Playfield`).
- [x] `robot.go_to(..., on_tick=on_tick_cb)` drives each leg.
- [x] `on_tick_cb` returns `False` when camera detects bounds excursion; the
      result is outcome `"aborted"` and demo prints `[BOUNDS-ABORT]` and continues.
- [x] Camera + odometry tracks drawn via `field.add_path` at each tick and on
      leg completion.
- [ ] The demo runs end-to-end with the bench hardware:
      `uv run --group calibrate python host_tests/playfield_tour/playfield_random_tour.py`.
      (DEFERRED TO TEAM-LEAD: requires live robot + camera bench run)
- [ ] Bench acceptance gate: robot on stand, `VER` confirms live firmware before
      trusting bench results (per project memory). Robot reaches each target within
      `ARRIVE_CM`; bounds abort terminates the leg cleanly.
      (DEFERRED TO TEAM-LEAD: requires live robot + camera bench run)

## Implementation Plan

### Approach

Rewrite `playfield_random_tour.py` in place. This is a complete replacement of the
script; keep the same filename and location (`host_tests/playfield_tour/`). The
logic of `hop()`, `select_location()`, `select_target()`, `compute_g()`,
`distance()`, `in_bounds()` is preserved verbatim or nearly so.

Key implementation notes:
- `make_robot()` signature: `make_robot(port=None, baud=115200, timeout=0.3)` —
  accepts the same `RELAY` path. See `host/robot_radio/robot/connection.py`.
- `Playfield.open()` uses `DaemonControl.connect_default(Config.load())` internally.
- `select_location()` now calls `field.get_tag(ROBOT_TAG)` in a loop (replacing
  the direct `dc.get_tags(cam)` call). The averaging logic is unchanged.
- In `on_tick_cb`, `robot.state.pose` carries `(x_mm, y_mm, heading_rad)` — divide
  mm by 10 to get cm for the `ODO_TRACK`.
- The `STREAM_MS` constant drives `go_to`'s internal `STREAM` enable — the callback
  form of `go_to` enables STREAM internally. Remove the manual `STREAM` send from
  the startup block.
- `robot.set_config(sTimeout=60000, turnGate=35, alphaYaw=0, yawRateMax=60)`
  replaces the manual `SET` loop (or use `robot.send("SET ...")` if `set_config`
  does not accept keyword integers — check `nezha.py:280`).

### Files to Modify

- `host_tests/playfield_tour/playfield_random_tour.py` — full rewrite.

### Testing Plan

No new unit tests for this ticket (it is an integration demo, not a unit-testable
module). Acceptance is bench hardware verification.

Pre-bench checklist:
1. Run unit suite: `uv run --with pytest python -m pytest host/tests/ -q` — all
   green including T001–T005 new tests.
2. VER check: `uv run rogo ver` (or equivalent) — confirm `fw=<version>` matches
   the build. Per project memory: bench reads need raw pyserial; verify the relay
   is connected and the robot responds before running the demo.
3. Run demo: `uv run --group calibrate python host_tests/playfield_tour/playfield_random_tour.py`.
4. Observe in live view: camera track (cyan dots) and odometry track (yellow
   crosses) draw. Robot reaches each rectangle. Manually verify a bounds excursion
   aborts the leg.

Verification command (unit tests only — bench gate is manual):
`uv run --with pytest python -m pytest host/tests/ -q`

### Documentation Updates

Update the module docstring at the top of `playfield_random_tour.py` to reflect
the new class-based API and remove references to the old serial plumbing.
