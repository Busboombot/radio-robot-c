# Source Code Review Findings for `source/`

Date: 2026-06-08
Scope: read-only review of firmware code under [source](../../source), using [review-plan.md](review-plan.md) and [source-code-review-rubric.md](source-code-review-rubric.md).

Active runtime path reviewed as authoritative: [source/main.cpp](../../source/main.cpp#L210) starts `LoopScheduler::run_blocks()`.

## Findings

### High: Timed and distance-drive deadlines are not rollover-safe

[source/control/DriveController.cpp](../../source/control/DriveController.cpp#L150) and [source/control/DriveController.cpp](../../source/control/DriveController.cpp#L187) compute `TIMED` and `DISTANCE` deadlines from `_lastTickMs` even though `now_ms` is passed into the begin methods. Completion then uses direct unsigned comparisons at [source/control/DriveController.cpp](../../source/control/DriveController.cpp#L339) and [source/control/DriveController.cpp](../../source/control/DriveController.cpp#L353).

Risk: Around the 32-bit millisecond rollover, a timed or distance command can complete immediately or run much longer than intended. The stale `_lastTickMs` baseline also means command duration is measured from the previous drive-advance tick rather than command acceptance. Streaming mode already uses signed elapsed-time logic, so this is inconsistent with the safer pattern in the same file.

Minimal correction: Set deadlines from `now_ms`, store the intended deadline, and test expiry with signed elapsed comparisons such as `(int32_t)(now_ms - deadlineMs) >= 0`. Bound or saturate the distance timeout calculation before adding it to `now_ms`.

Review views: Command-to-Motion Execution Paths; Embedded Runtime, Timing, and Concurrency; Robotics Model, Numerical Methods, and Hardware Safety.

### High: Encoder and pose reset paths can desynchronize odometry state

`DISTANCE` starts by resetting motor encoder accumulators in [source/control/DriveController.cpp](../../source/control/DriveController.cpp#L177), then `Robot::distanceDrive()` manually zeros the cached encoder inputs at [source/robot/Robot.cpp](../../source/robot/Robot.cpp#L110-L111). However, `Odometry` keeps its own previous-encoder snapshot and `predict()` computes deltas against that snapshot at [source/control/Odometry.cpp](../../source/control/Odometry.cpp#L17-L25). In the active loop, commands are processed before odometry prediction in the same iteration: [source/control/LoopScheduler.cpp](../../source/control/LoopScheduler.cpp#L612-L623).

Risk: If an encoder reset happens after odometry has seen nonzero encoder values, the next `predict()` can treat the reset-to-zero as real backward wheel travel and move the pose. The public `ZERO enc` command is also exposed through [source/app/CommandProcessor.cpp](../../source/app/CommandProcessor.cpp#L1313) and calls [source/robot/Robot.cpp](../../source/robot/Robot.cpp#L134-L137), which resets motor/controller state but not the cached encoder state or odometry snapshot. Conversely, `ZERO pose` calls [source/control/Odometry.cpp](../../source/control/Odometry.cpp#L86-L98), where `setPose()` resets `_prevEncL/_prevEncR` to zero rather than the current encoder snapshot, so a pose reset with nonzero encoders can create a forward jump on the next prediction.

Minimal correction: Create one robot-level reset operation that synchronizes all relevant state atomically: hardware encoder accumulators, `MotorController` velocity baselines, `HardwareState` encoder fields, and `Odometry` previous-encoder snapshots. `Odometry::setPose()` should snapshot current encoder inputs rather than assuming zero.

Review views: Command-to-Motion Execution Paths; Robotics Model, Numerical Methods, and Hardware Safety; Interpretability, Dead Code, and Change Safety.

### High: `SET` can write invalid live control configuration

The config registry exposes safety-critical fields such as `tw`, `vWheelMax`, `steerHeadroom`, and `ctrlPeriod` in [source/app/CommandProcessor.cpp](../../source/app/CommandProcessor.cpp#L63-L112). `handleSet()` parses with `atof()`/`atoi()` and writes directly into `RobotConfig` via offsets at [source/app/CommandProcessor.cpp](../../source/app/CommandProcessor.cpp#L465-L476), with no parse-failure detection, range checks, invariant checks, or atomic all-or-nothing application.

Risk: A malformed or merely out-of-range `SET` can break the active control model. Examples: `tw=0` divides by zero in odometry and kinematics ([source/control/Odometry.cpp](../../source/control/Odometry.cpp#L24), [source/control/BodyKinematics.cpp](../../source/control/BodyKinematics.cpp#L22)); `vWheelMax < steerHeadroom` makes the saturation ceiling negative at [source/control/BodyKinematics.cpp](../../source/control/BodyKinematics.cpp#L29-L36); negative `ctrlPeriod` is cast to `uint32_t` in the scheduler and drive controller ([source/control/LoopScheduler.cpp](../../source/control/LoopScheduler.cpp#L605), [source/control/DriveController.cpp](../../source/control/DriveController.cpp#L308). Direct drive commands have range checks, but the live config path bypasses equivalent constraints.

Minimal correction: Add typed parsing with end-pointer validation, central `RobotConfig` validation for field ranges and cross-field invariants, and apply multi-key `SET` changes only after the whole candidate config validates. Keep dependent controller updates tied to successful commits.

Review views: Architecture, Modularity, and Cohesion; Command-to-Motion Execution Paths; Robotics Model, Numerical Methods, and Hardware Safety.

### Medium: Sensor reads discard bus status and validity is sticky

`I2CBus::write()` and `I2CBus::read()` return CODAL status and track errors in [source/hal/I2CBus.cpp](../../source/hal/I2CBus.cpp#L34-L87), but OTOS burst reads ignore those statuses in [source/hal/OtosSensor.cpp](../../source/hal/OtosSensor.cpp#L104-L112). `Robot::otosCorrect()` then marks the reading valid and feeds it into pose correction at [source/robot/Robot.cpp](../../source/robot/Robot.cpp#L275-L294). Color-sensor helpers similarly ignore low-level read/write status in [source/hal/ColorSensor.cpp](../../source/hal/ColorSensor.cpp#L107-L141). Line and color state set `valid = true` on successful reads at [source/robot/Robot.cpp](../../source/robot/Robot.cpp#L410-L430), but telemetry later checks only the sticky valid bit, not freshness, at [source/robot/Robot.cpp](../../source/robot/Robot.cpp#L485-L490).

Risk: A failed OTOS transaction can produce zero-filled or stale values that look valid enough to enter complementary pose correction. Line/color telemetry can continue to publish old data after the sensor stops updating. This weakens the review-plan requirement that validity, freshness, and fault modes be explicit.

Minimal correction: Make sensor APIs return success/failure for every read path, update `HardwareState` only on success, and clear or age out `valid` after missed periods. For telemetry and control fusion, require freshness such as `now_ms - lastUpdMs <= lagMs` instead of checking only `valid`.

Review views: Embedded Runtime, Timing, and Concurrency; Robotics Model, Numerical Methods, and Hardware Safety; Interpretability, Dead Code, and Change Safety.

### Medium: Active code still carries obsolete control/scheduler paths

The active firmware starts `run_blocks()` in [source/main.cpp](../../source/main.cpp#L210), but `LoopScheduler` still exposes inactive `run_tasks()` and `run_all()` paths at [source/control/LoopScheduler.cpp](../../source/control/LoopScheduler.cpp#L346) and [source/control/LoopScheduler.cpp](../../source/control/LoopScheduler.cpp#L487). `CommandProcessor` help still describes debug controls that take effect in `run_all()` at [source/app/CommandProcessor.cpp](../../source/app/CommandProcessor.cpp#L826). Separately, `RatioPidController` is retained but bypassed in normal drive in [source/control/MotorController.h](../../source/control/MotorController.h#L18) and [source/control/MotorController.h](../../source/control/MotorController.h#L151-L152), while `PID_BYPASS` remains as a compile-time switch at [source/control/MotorController.cpp](../../source/control/MotorController.cpp#L12) and [source/control/MotorController.cpp](../../source/control/MotorController.cpp#L359).

Risk: The project has multiple plausible control-loop stories in production source. That raises change risk: a maintainer can fix or test the wrong scheduler path, tune the bypassed PID controller, or trust debug commands that do not affect the active loop. This is not the same severity as an active motion bug, but it directly hurts interpretability and reviewability.

Minimal correction: Either remove obsolete paths or isolate them behind explicit test-only/build-time modules. Ensure command help and debug commands describe only the active runtime path unless a non-active path is deliberately selected at build time.

Review views: Architecture, Modularity, and Cohesion; Interpretability, Dead Code, and Change Safety.

## Scorecard

| View | Score | Rationale |
| --- | ---: | --- |
| Architecture, Modularity, and Cohesion | 3 / 5 | Strong composition root and clear major classes, but `CommandProcessor` owns parsing, registry mutation, debug controls, and hardware escape hatches, while `Robot` exposes broad mutable internals. |
| Command-to-Motion Execution Paths | 3 / 5 | S/T/D/G paths are readable, but deadline handling and reset ownership need hardening before motion commands are mechanically safe. |
| Embedded Runtime, Timing, and Concurrency | 3 / 5 | `run_blocks()` is simpler than older scheduler variants and uses signed elapsed checks in several places, but rollover handling is inconsistent and sensor failure state is too implicit. |
| Robotics Model, Numerical Methods, and Hardware Safety | 3 / 5 | Odometry prediction uses midpoint/exact-arc integration and wheel saturation preserves curvature, but reset synchronization, config validation, and sensor validity weaken the numerical guarantees. |
| Interpretability, Dead Code, and Change Safety | 2 / 5 | Historical comments are useful, but inactive scheduler/control paths and bypassed PID code make it too easy to reason about the wrong system. |

## Positive Signals

- [source/control/Odometry.cpp](../../source/control/Odometry.cpp#L16-L31) uses midpoint integration for differential-drive odometry rather than plain forward Euler in the active `predict()` path.
- [source/control/DriveController.cpp](../../source/control/DriveController.cpp#L318-L333) already documents and uses signed elapsed-time logic for the streaming watchdog; this gives a local pattern for fixing T/D deadlines.
- [source/robot/Robot.cpp](../../source/robot/Robot.cpp#L332-L396) documents the encoder read ordering and outlier strategy in enough detail to preserve the hardware workaround during refactors.

## Residual Risk

No build, unit tests, or bench tests were run for this review. The review was intentionally read-only. The highest-value follow-up is to add focused host-side or firmware-level tests for rollover deadlines, encoder/pose reset synchronization, config validation, and sensor-read failure behavior.