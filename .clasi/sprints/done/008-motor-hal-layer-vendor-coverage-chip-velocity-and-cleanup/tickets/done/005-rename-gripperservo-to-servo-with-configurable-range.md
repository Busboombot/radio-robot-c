---
id: '005'
title: Rename GripperServo to Servo with configurable range
status: done
use-cases:
- SUC-005
depends-on:
- '002'
github-issue: ''
issue: source-fixme-cleanup.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Rename GripperServo to Servo with configurable range

## Description

`GripperServo` is a generic hobby-servo driver hardcoded to 0–180°. The
FIXME asked for: rename to `Servo`; make the range configurable for 180°
vs 360° (continuous-rotation) servos.

Depends on ticket 002 because `Robot.h` will have been restructured for the
Motor split; this ticket touches `Robot.h` again (rename gripper member) and
must apply cleanly on top.

## Acceptance Criteria

- [x] `source/hal/GripperServo.{h,cpp}` deleted; `source/hal/Servo.{h,cpp}`
  exists.
- [x] `Servo` constructor signature: `Servo(MicroBitPin& pin, uint16_t
  maxDegrees = 180)`.
- [x] `setAngle(uint8_t degrees)` clamps to `[0, maxDegrees]` before
  driving the pin.
- [x] `Robot` member renamed: `GripperServo _gripper` → `Servo _servo`;
  accessor `gripper()` return type updated to `Servo*`.
- [x] `Robot::setGripperAngle()` unchanged in behavior (still 0–180 default).
- [x] `CommandProcessor` updated for include/accessor rename (if any).
- [x] All `#include "GripperServo.h"` → `#include "Servo.h"` replaced.
- [x] `python3 build.py` succeeds; RAM line reported and within budget.
- [x] Bench: gripper servo still responds to the `G` command at expected
  angles.

## Implementation Plan

### Approach

Pure rename + constructor parameter addition. No logic changes.

1. Create `source/hal/Servo.h` — copy `GripperServo.h`; rename class to
   `Servo`; add `uint16_t _maxDegrees` member; update constructor signature.
2. Create `source/hal/Servo.cpp` — copy `GripperServo.cpp`; rename class;
   clamp to `[0, _maxDegrees]` instead of `[0, 180]`.
3. Update `source/robot/Robot.h` — include `Servo.h`; rename member and
   accessor.
4. Update `source/robot/Robot.cpp` — construction and `setGripperAngle` call.
5. Update `source/app/CommandProcessor.cpp` — accessor rename if used.
6. Delete `GripperServo.{h,cpp}`.
7. Update `docs/architecture.md` HAL section.

### Files to Create

- `source/hal/Servo.h`
- `source/hal/Servo.cpp`

### Files to Modify

- `source/robot/Robot.h`
- `source/robot/Robot.cpp`
- `source/app/CommandProcessor.cpp` (if it references GripperServo directly)
- `docs/architecture.md`

### Files to Delete

- `source/hal/GripperServo.h`
- `source/hal/GripperServo.cpp`

### Testing Plan

- `python3 build.py` must succeed; report RAM line.
- Bench: issue `G 90` and `G 0` commands; confirm servo responds at the
  correct angles.

### Documentation Updates

- `docs/architecture.md`: HAL section `GripperServo` → `Servo`.
