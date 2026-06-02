---
status: pending
sprint: 008
---

# Source FIXME Cleanup (omnibus)

Catalogs every `FIXME` found in `source/` as of capture, so the markers can be
removed from the code while the work stays tracked. Some items overlap existing
issues (noted); the rest are tracked here. Found via `grep -rniE "fix ?me" source/`.

## HAL — NezhaV2 (`source/hal/NezhaV2.h`)

These four cluster into one theme: **turn `NezhaV2` into a proper per-motor
`Motor` abstraction.** The deeper pieces overlap existing issues (cross-linked);
this issue owns the naming/structure cleanup.

1. **Rename `NezhaV2` → `Motor`** (was line 23). "Nezha" is the whole controller
   board, not a motor; the class models one motor channel. Rename the class (and
   file) accordingly.
2. **One motor per object** (was line 42). The object should represent a *single*
   motor, owning its own PID controller, velocity/position state, commanded
   velocity, and encoder offset (as the vendor code structures it) — rather than
   driving both wheels from one object. Overlaps [[kinematics-velocity-control-layer]]
   (per-wheel `VelocityController` + state) and [[firmware-architecture-refactor]].
3. **Per-motor direction parameter** (was line 50). `LEFT_FWD = +1` / `RIGHT_FWD
   = -1` are hardcoded; forward sign should be a **per-motor config value**
   (+1/−1 depending on mounting), living in `RobotConfig` rather than as
   constants. Overlaps [[firmware-architecture-refactor]] (unified config).
4. **Fill in remaining vendor methods** (was line 36) — e.g. a `readSpeed`
   analog. Already covered by [[nezha-chip-velocity-readspeed-0x47]] (velocity)
   and [[nezha-full-vendor-i2c-coverage]] (the rest); listed here only for
   completeness — no separate work.

## HAL — GripperServo (`source/hal/GripperServo.h`)

5. **Rename `GripperServo` → `Servo` and make it servo-type configurable** (was
   line 10). The class is a generic hobby-servo driver, not gripper-specific.
   Rename, and make the range configurable for **180° vs 360°** (continuous-
   rotation) servos rather than hardcoding the 0..180 clamp.

## HAL — LineSensor (`source/hal/LineSensor.h`)

6. **Line-sensor calibration & conditioning** (was line 21). Add a calibration
   flow where the user sweeps the robot to capture per-sensor **min/max**, then
   scale each channel to a normalized/linearized response that is robust to
   lighting. Optionally **smooth/filter** the readings to reduce noise. (Feature
   work, independent of the other items.)

## Action taken at capture

The six `FIXME` comments were removed from `source/` (their substance is recorded
above). No behavior change — comment-only edits.

## Verification

- `grep -rniE "fix ?me" source/` returns nothing.
- Firmware still compiles (comment-only removals; no code touched).
