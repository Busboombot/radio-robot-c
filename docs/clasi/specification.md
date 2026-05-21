# radio-robot-c: Feature Specification

## Hardware

- **Robot**: DFRobot QBot Pro
- **MCU board**: micro:bit V2
- **Motor board**: Nezha V2 (I2C-attached motor and servo controller)
- **Pose sensor**: SparkFun OTOS (Optical Tracking Odometry Sensor, I2C)
- **Line sensor**: analog or digital line-following sensor on port IO
- **Color sensor**: I2C color sensor
- **Gripper**: servo on Nezha V2 servo port, commanded by angle 0-180 degrees
- **Framework**: CODAL (Component Oriented Device Abstraction Layer)
- **Build**: `python build.py` (Docker CODAL toolchain) → `.hex` output
- **Deploy**: `python scripts/deploy.py`

## Communication Protocol

### Serial

- Baud rate: 115200
- Direction: bidirectional between Python host and micro:bit USB serial
- Commands arrive as newline-terminated ASCII strings
- Responses are newline-terminated ASCII strings

### Radio

- micro:bit radio group: 10
- Commands arriving over radio with `>` prefix are relayed to the serial command processor
- Responses sent back over radio carry `<` prefix
- Enables wireless operation when the Python host communicates via a second micro:bit acting as a USB-serial bridge

### Encoding

- All numeric values are encoded as sign-prefixed integers: positive values carry explicit `+` sign, negative values carry `-` sign
- Example: `+1234`, `-42`, `+0`
- No floating-point values on the wire; fixed-point scaling is used where necessary

## Class Architecture

The codebase is organized into five layers. Files reside under `source/`.

### Layer: types (`source/types/`)

Shared data structures and constants used across all layers.

- **Types.h**: `Vec2` (2D point, integer mm), `Pose` (x, y, heading in integer units), `WheelSpeeds` (left/right signed PWM), `CommandResult` (enum), protocol constants (baud rate, radio group, sign-prefix format).

### Layer: hal (`source/hal/`)

Hardware abstraction — wraps CODAL primitives.

- **SerialPort**: wraps `uBit.serial`; provides `readLine()`, `writeLine(str)`; configured at 115200 baud.
- **NezhaV2**: I2C driver for the Nezha V2 motor board; provides `setMotorPwm(motor, pwm)`, `setServoDegrees(servo, angle)`, `readEncoder(motor) -> int32_t`; uses `MicroBitI2C`.
- **OtosSensor**: I2C driver for the OTOS sensor; provides `init()`, `calibrateImu()`, `getPose() -> Pose`, `setPose(Pose)`, `setOffset(x, y, heading)`, `setLinearScalar(s)`, `setAngularScalar(s)`.
- **LineSensor**: reads line sensor value from port IO; provides `read() -> int`.
- **ColorSensor**: I2C color sensor driver; provides `read() -> ColorReading` (R, G, B, clear).
- **GripperServo**: wraps NezhaV2 servo port; provides `setAngle(degrees)`.
- **PortIO**: manages robot port general-purpose I/O; provides `read(port) -> int`, `write(port, value)`, `readAll() -> int[]`.

### Layer: control (`source/control/`)

Motor control algorithms, independent of hardware abstraction.

- **MotorController**: owns drive state machine; calls NezhaV2 for PWM output and encoder reads; delegates ratio correction to RatioPidController; implements `startDriveClean()` (hard reset for T/D/G commands), `startDrive()` (re-seed without reset for S command), `update()` (called from main loop), `stop()`.
- **RatioPidController**: implements the cumulative-distance ratio PID algorithm (see Motor Control Algorithm section); stateless between `reset()` calls; provides `compute(leftMm, rightMm, expectedLeft, expectedRight) -> correction`.
- **Odometry**: dead-reckoning pose estimator; integrates encoder deltas each loop; provides `getPose() -> Pose`, `setPose(Pose)`, `reset()`.

### Layer: app (`source/app/`)

Command parsing and dispatch.

- **CommandProcessor**: parses incoming ASCII command strings; dispatches to subsystem methods; formats and emits ASCII response strings; owns the command table for all 30+ commands.
- **Robot**: top-level singleton; owns all subsystem instances (all static, no heap); runs the main loop (`uBit.sleep`-based); wires subsystems together.
- **Announcer**: periodically broadcasts `HELLO` / `DEVICE:` discovery messages over radio and serial.

### Layer: nav (`source/nav/`)

Navigation algorithms operating on pose and waypoint data.

- **PathFollower** (pure-virtual interface): `computeSpeeds(currentPose, path) -> WheelSpeeds`; implemented by PurePursuit and Stanley.
- **PurePursuit**: implements pure-pursuit path following; lookahead distance is a tunable parameter.
- **Stanley**: implements Stanley controller path following; gain is a tunable parameter.
- **PoseProvider** (pure-virtual interface): `getPose() -> Pose`; implemented by OtosPoseProvider (wraps OtosSensor) and DeadReckoningPoseProvider (wraps Odometry). Future: ExternalPoseProvider fed by SI command.
- **ArcComputer**: stateless helper; `computeArc(tx, ty, trackwidthMm) -> (leftMm, rightMm)`; used by the G command.

## Motor Control Algorithm (Ratio PID)

### Overview

The ratio PID controller tracks cumulative encoder distance traveled by each wheel since the start of a drive command. It computes a normalized error between the expected and actual distance ratio of the faster wheel, then applies a PID correction to that wheel's feed-forward PWM. The slower wheel is adjusted only when the integral exceeds the `KAT` threshold.

### Algorithm Steps

1. At command start, call `startDriveClean()` (T/D/G commands) or `startDrive()` (S command).
   - `startDriveClean()`: zero cumulative encoder distances, zero PID state, set target speeds and direction.
   - `startDrive()`: re-seed cumulative distances from current encoder values without resetting PID integrator.

2. Each control loop iteration:
   a. Read current encoder counts for left and right wheels.
   b. Compute `leftMm` and `rightMm` as cumulative distance traveled since command start.
   c. Determine which wheel is the "faster" wheel based on the commanded speed ratio.
   d. Compute expected cumulative distance for the faster wheel at this point in time (from feed-forward model).
   e. Compute normalized error:
      ```
      normalized_error = (expected_faster_mm - actual_faster_mm) / max(1, expected_faster_mm)
      ```
   f. Run PID on normalized error (gains: KCP proportional, KCI integral, KCD derivative).
   g. Add PID correction to faster wheel's feed-forward PWM. Clamp to valid PWM range.
   h. If integral exceeds `KAT` threshold: apply `KAG` gain correction to slower wheel as well.

3. Termination:
   - T command: stop when elapsed time exceeds commanded duration.
   - D command: stop when cumulative distance of both wheels meets or exceeds commanded distance.
   - G command: stop when both wheel encoder targets are met within `KGD` tolerance; emit `G+DONE`.
   - S command: runs until explicitly stopped by X/STOP command.

### Per-Direction Motor Scale Parameters

Four scale parameters compensate for motor asymmetry:
- `KLF`: left motor forward scale
- `KLB`: left motor backward scale
- `KRF`: right motor forward scale
- `KRB`: right motor backward scale

These scale the feed-forward PWM before PID correction is applied.

### Confirmed Performance

340/339 mm final encoder over a 2-second run = 0.3% error.

## Navigation

### PathFollower Interface

```
class PathFollower {
  virtual WheelSpeeds computeSpeeds(Pose current, Path& path) = 0;
  virtual bool isDone() = 0;
};
```

Implementations are selected at runtime by the command processor. The active implementation is set by a command or configuration parameter.

### PurePursuit

Selects a lookahead point on the path at a fixed distance ahead of the robot. Computes the arc curvature needed to reach that point. Scales wheel speeds to follow the arc.

### Stanley Controller

Uses the cross-track error at the front axle and heading error to compute a steering correction. Gain is a tunable parameter.

### PoseProvider Interface

```
class PoseProvider {
  virtual Pose getPose() = 0;
  virtual void setPose(Pose p) = 0;
};
```

Active implementation is selected at runtime:
- **OtosPoseProvider**: reads from OTOS sensor; most accurate.
- **DeadReckoningPoseProvider**: integrates encoder odometry; used when OTOS is unavailable.
- **ExternalPoseProvider** (future): accepts pose injected by the SI command, enabling external camera-based localization.

## G Command (Arc-to-Goal Navigation)

### Command Format

`G+X+Y+Speed` — navigate to position (X mm, Y mm) relative to the robot's current pose at the given speed.

### Two-Phase Execution

**Phase 1 — Pre-rotate** (conditional):

If the heading error to the target exceeds `KGT` degrees, the robot rotates in place to face the target before arc driving. This avoids backing into curves with large initial heading errors.

**Phase 2 — Arc drive**:

1. Call `computeArc(tx, ty, KTW)` where `tx` and `ty` are the target coordinates and `KTW` is the trackwidth in mm.
2. Arc formula:
   ```
   R = (tx^2 + ty^2) / (2 * ty)
   alpha = atan2(ty, tx + R)
   leftMm  = (R - KTW/2) * alpha
   rightMm = (R + KTW/2) * alpha
   ```
3. Set encoder targets: left encoder target = current + leftMm, right encoder target = current + rightMm.
4. Drive using `startDriveClean()` with ratio PID active.
5. When both encoder targets are met within `KGD` tolerance, stop motors and emit `G+DONE`.

## Command Reference

All commands use sign-prefixed integer encoding. Commands listed by subsystem.

### Drive Commands

| Command | Format | Description |
|---------|--------|-------------|
| S | `S+LS+RS` | Set motor speeds (left speed, right speed); runs until stopped |
| T | `T+LS+RS+DUR` | Drive at speeds for DUR milliseconds |
| D | `D+LS+RS+DIST` | Drive at speeds for DIST mm (encoder-based) |
| X | `X` | Stop immediately |
| STOP | `STOP` | Stop immediately (alias for X) |

### Odometry / Pose Commands

| Command | Format | Description |
|---------|--------|-------------|
| ENC | `ENC` | Query encoder positions; returns `ENC+L+R` |
| EZ | `EZ` | Zero encoder counts |
| SO | `SO` | Query dead-reckoning odometry; returns `SO+X+Y+H` |
| SZ | `SZ` | Zero dead-reckoning odometry |
| SI | `SI+X+Y+H` | Set dead-reckoning odometry (and future external pose provider) |

### OTOS Sensor Commands

| Command | Format | Description |
|---------|--------|-------------|
| OI | `OI` | Initialize OTOS sensor |
| OK | `OK` | Calibrate OTOS IMU |
| OZ | `OZ` | Zero OTOS pose |
| OR | `OR` | Reset OTOS sensor |
| OP | `OP` | Query OTOS pose; returns `OP+X+Y+H` |
| OV | `OV` | Query OTOS velocity |
| OL | `OL` | Set OTOS linear scalar (uses current K params) |
| OA | `OA` | Set OTOS angular scalar (uses current K params) |

### Sensor Commands

| Command | Format | Description |
|---------|--------|-------------|
| LS | `LS` | Read line sensor; returns `LS+VAL` |
| CS | `CS` | Read color sensor; returns `CS+R+G+B+C` |

### Port IO Commands

| Command | Format | Description |
|---------|--------|-------------|
| P | `P+PORT` | Read GPIO port; returns `P+PORT+VAL` |
| PA | `PA+PORT+VAL` | Write GPIO port |

### Gripper and Servo

| Command | Format | Description |
|---------|--------|-------------|
| G (gripper) | Note: gripper uses angle 0-180 on a dedicated servo port; this may be the same G prefix disambiguated by argument shape — see G command below |

### Navigation Command

| Command | Format | Description |
|---------|--------|-------------|
| G | `G+X+Y+Speed` | Navigate to relative XY position using arc math; emits `G+DONE` on completion |

### Calibration Commands (K*)

| Command | Format | Description |
|---------|--------|-------------|
| KML | `KML+VAL` | Set left motor scale (legacy/unified) |
| KMR | `KMR+VAL` | Set right motor scale (legacy/unified) |
| KFF | `KFF+VAL` | Set feed-forward gain |
| KSP | `KSP+VAL` | Set speed proportional gain |
| KSI | `KSI+VAL` | Set speed integral gain |
| KIC | `KIC+VAL` | Set integral clamp |
| KSR | `KSR+VAL` | Set speed ramp rate |
| KSM | `KSM+VAL` | Set speed minimum PWM |
| KSS | `KSS+VAL` | Set speed scaling |
| KTR | `KTR+VAL` | Set turn ratio |
| KER | `KER+VAL` | Set encoder resolution (ticks/mm) |
| KLF | `KLF+VAL` | Set left motor forward scale |
| KLB | `KLB+VAL` | Set left motor backward scale |
| KRF | `KRF+VAL` | Set right motor forward scale |
| KRB | `KRB+VAL` | Set right motor backward scale |
| KCP | `KCP+VAL` | Set ratio PID proportional gain |
| KCI | `KCI+VAL` | Set ratio PID integral gain |
| KCD | `KCD+VAL` | Set ratio PID derivative gain |
| KCC | `KCC+VAL` | Set ratio PID correction clamp |
| KAT | `KAT+VAL` | Set slower-wheel adjustment integral threshold |
| KAG | `KAG+VAL` | Set slower-wheel adjustment gain |
| KTW | `KTW+VAL` | Set robot trackwidth in mm |
| KGT | `KGT+VAL` | Set G command pre-rotate threshold in degrees |
| KGD | `KGD+VAL` | Set G command done tolerance in mm |

### Discovery Commands

| Command | Format | Description |
|---------|--------|-------------|
| HELLO | `HELLO` | Trigger device announcement |
| DEVICE: | Emitted by robot | Discovery response; format `DEVICE:+name+version` |

## Calibration Parameters Table

| Parameter | Description | Units | Default |
|-----------|-------------|-------|---------|
| KML | Legacy left motor scale | - | 1.0 |
| KMR | Legacy right motor scale | - | 1.0 |
| KFF | Feed-forward gain | - | TBD |
| KSP | Speed P gain | - | TBD |
| KSI | Speed I gain | - | TBD |
| KIC | Integral clamp | - | TBD |
| KSR | Speed ramp rate | PWM/loop | TBD |
| KSM | Speed minimum PWM | PWM | TBD |
| KSS | Speed scaling | - | TBD |
| KTR | Turn ratio | - | TBD |
| KER | Encoder resolution | ticks/mm | TBD |
| KLF | Left motor forward scale | - | 1.0 |
| KLB | Left motor backward scale | - | 1.0 |
| KRF | Right motor forward scale | - | 1.0 |
| KRB | Right motor backward scale | - | 1.0 |
| KCP | Ratio PID P gain | - | TBD |
| KCI | Ratio PID I gain | - | TBD |
| KCD | Ratio PID D gain | - | TBD |
| KCC | Ratio PID correction clamp | PWM | TBD |
| KAT | Slower-wheel adj threshold | integral units | TBD |
| KAG | Slower-wheel adj gain | - | TBD |
| KTW | Robot trackwidth | mm | TBD |
| KGT | G pre-rotate threshold | degrees | TBD |
| KGD | G done tolerance | mm | TBD |

## Memory and Language Constraints

- Language standard: C++14
- No heap allocation in the hot path; no `new` or `malloc` during command execution or sensor reads
- All subsystem instances are static (created once at startup in Robot)
- No STL containers that allocate dynamically in the hot path
- Stack-allocated buffers for serial line reading

## Build and Deploy

- **Build**: `python build.py` — runs Docker container with CODAL ARM toolchain; produces `.hex` file
- **Deploy**: `python scripts/deploy.py` — copies `.hex` to micro:bit mass-storage mount
- **Source layout**:
  - `source/app/` — Robot, CommandProcessor, Announcer
  - `source/control/` — MotorController, RatioPidController, Odometry
  - `source/hal/` — SerialPort, NezhaV2, OtosSensor, LineSensor, ColorSensor, GripperServo, PortIO
  - `source/nav/` — PathFollower, PurePursuit, Stanley, PoseProvider, ArcComputer
  - `source/types/` — Types.h, shared constants
