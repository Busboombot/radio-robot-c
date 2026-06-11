---
id: '005'
title: Host RobotState dataclass, TLMFrame twist field, NezhaState wiring
status: done
use-cases:
- SUC-006
depends-on:
- '004'
issue: ekf-velocity-fusion-and-robot-state.md
---

# T005: Host RobotState dataclass, TLMFrame twist field, NezhaState wiring

## Description

Three host-side Python changes that together give consumers a unified motion state:

1. **New `RobotState` dataclass** (`host/robot_radio/robot/robot_state.py`) —
   frozen composite `{pose: Pose, v: float, omega: float, accel: tuple|None, stamp: float}`.
   `Pose` is unchanged (pure position+heading, no velocity fields added to it).

2. **`TLMFrame.twist` field** (`host/robot_radio/robot/protocol.py`) — add
   `twist: tuple[int, int] | None = None` to the `TLMFrame` dataclass; extend
   `parse_tlm()` to parse the `twist=v,omega` key-value pair.

3. **`NezhaState.robot_state`** (`host/robot_radio/robot/nezha_state.py`) — add a
   `robot_state: RobotState | None` attribute; build it in `_process_line()` when
   a TLM frame carries both `pose=` and `twist=`.

These changes are purely additive — all existing attributes on `NezhaState` and
all existing `TLMFrame` fields are preserved unchanged.

## Acceptance Criteria

**`robot_state.py` (new file, SUC-006):**
- [x] `host/robot_radio/robot/robot_state.py` created with:
  ```python
  from dataclasses import dataclass
  from robot_radio.nav.pose import Pose

  @dataclass(frozen=True)
  class RobotState:
      pose: Pose
      v: float       # body linear speed, mm/s
      omega: float   # yaw rate, rad/s
      accel: tuple[float, float] | None  # (ax_mmps2, ay_mmps2) or None
      stamp: float   # host monotonic time, seconds
  ```
- [x] The file has a module docstring explaining units and the pose/velocity split
  (matches WPILib `Pose2d` / `ChassisSpeeds` convention).

**`TLMFrame` and `parse_tlm()` (SUC-006):**
- [x] `TLMFrame` gains `twist: tuple[int, int] | None = None`
  where the tuple is `(v_mmps, omega_mradps)` as integers (firmware snprintf output).
- [x] `parse_tlm()` parses `twist=v,omega` following the exact same pattern as
  the existing `vel` parsing (split on `,`, 2 parts, `int()` conversion).
- [x] `parse_tlm("TLM t=1000 pose=100,200,1800 twist=300,500")` returns a
  `TLMFrame` with `twist = (300, 500)`.
- [x] `parse_tlm("TLM t=1000 pose=100,200,1800")` returns a `TLMFrame` with
  `twist = None`.
- [x] All existing `parse_tlm()` behavior is unchanged (no regressions in
  `test_command_processor.py` or any protocol-level test).

**`NezhaState` (SUC-006):**
- [x] `NezhaState.__init__()` initialises `self.robot_state: RobotState | None = None`.
- [x] `_process_line()` extended: when `tlm.pose is not None`:
  - Build `Pose(x=x_mm/10.0, y=y_mm/10.0, heading=h_cdeg/18000.0*pi)` from
    `tlm.pose`. Note: `Pose` uses centimetres, TLM uses mm; divide x/y by 10.
  - If `tlm.twist is not None`: `v = tlm.twist[0] * 1.0` (mm/s stays as mm/s),
    `omega = tlm.twist[1] / 1000.0` (mrad/s → rad/s).
  - If `tlm.twist is None`: `v = 0.0`, `omega = 0.0`.
  - Build `RobotState(pose=pose, v=v, omega=omega, accel=None, stamp=time.monotonic())`.
  - Update `self.robot_state` under `_lock`.
- [x] `NezhaState.robot_state` is `None` at construction and after `__init__()`.
- [x] All existing attributes (`otos_pose`, `heading_rad`, `encoders`, etc.)
  are updated as before — `robot_state` is additive, not a replacement.
- [x] Thread safety: `robot_state` is read/written under `_lock`.

**Build / test:**
- [x] `uv run --with pytest python -m pytest -v` passes, including all existing
  tests that exercise `NezhaState` and `parse_tlm`.

## Implementation Plan

### Approach

Three independent file changes:

**1. `robot_state.py`** — new file. No existing file to modify. Place it in
`host/robot_radio/robot/` alongside `nezha_state.py`. The `Pose` import comes
from `robot_radio.nav.pose`.

**2. `protocol.py`** — `TLMFrame` is a `@dataclass`; add the field at the end
of the field list (after `color`). `parse_tlm()` already has a chain of
`if "x" in kv:` blocks — add a `twist` block after the `vel` block:
```python
if "twist" in kv:
    try:
        parts = kv["twist"].split(",")
        if len(parts) == 2:
            frame.twist = (int(parts[0]), int(parts[1]))
    except ValueError:
        pass
```

**3. `nezha_state.py`** — add import of `RobotState` from `robot_radio.robot.robot_state`.
Add `self.robot_state = None` to `__init__()`. Extend `_process_line()`.

**Unit conversion note:** `Pose` uses centimetres for x/y but TLM carries mm.
The conversion `x_cm = x_mm / 10.0` must be applied when building the `Pose`.
The existing `otos_pose` attribute preserves mm (for backward compatibility) and
is not changed. The new `RobotState.pose` uses cm to match the `Pose` convention.

### Files to create

- `host/robot_radio/robot/robot_state.py`

### Files to modify

- `host/robot_radio/robot/protocol.py` — `TLMFrame`, `parse_tlm()`
- `host/robot_radio/robot/nezha_state.py` — `__init__()`, `_process_line()`

### Testing plan

```
uv run --with pytest python -m pytest -v
```

Key tests to verify:
- `test_command_processor.py` (or any file testing `parse_tlm`) — must pass.
- New assertions in T006 will cover `parse_tlm` with `twist=` and the
  `RobotState` construction path.

### Documentation updates

Add a docstring to `RobotState` explaining the unit conventions:
- `pose.x`, `pose.y` in centimetres (Pose convention).
- `v` in mm/s (matches firmware TLM field).
- `omega` in rad/s (converted from mrad/s TLM field).
- `accel` in mm/s^2 per axis, or None if not yet received.
