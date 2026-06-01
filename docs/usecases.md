# radio-robot-c: Use Cases

## Actors

- **Python host**: the `robot_radio/` Python package running on a connected PC; communicates via USB serial or via micro:bit radio bridge.
- **Robot firmware**: the C++ firmware running on the micro:bit V2 / QBot Pro.

---

## UC-001: Drive Robot at Continuous Speed

**ID**: UC-001
**Title**: Drive robot at continuous speed (S command)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running and serial/radio connection is established.
- Robot is not executing a timed or distance-bounded drive.

**Main Flow**:
1. Python host sends `S+LS+RS` where LS and RS are sign-prefixed left and right motor speeds.
2. CommandProcessor parses the command and calls `MotorController.startDrive(leftSpeed, rightSpeed)`.
3. MotorController re-seeds cumulative encoder distances without resetting PID integrator, sets target speeds, and begins driving.
4. RatioPidController runs each control loop to maintain the commanded speed ratio.
5. Motors run until an explicit stop command is received.
6. Firmware sends no acknowledgment response (speed is ongoing state).

**Postconditions**:
- Both motors are running at the commanded speeds with ratio PID correction active.

**Error Flows**:
- If speed values are out of range, MotorController clamps to valid PWM range.
- If serial connection drops, motors continue until an X or STOP command is received or firmware resets.

---

## UC-002: Drive Robot for Timed Duration

**ID**: UC-002
**Title**: Drive robot for a timed duration (T command)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running and connection is established.

**Main Flow**:
1. Python host sends `T+LS+RS+DUR` where DUR is duration in milliseconds.
2. CommandProcessor parses the command and calls `MotorController.startDriveClean(leftSpeed, rightSpeed, durationMs)`.
3. MotorController zeros cumulative encoder distances, zeros PID state, sets target speeds and duration.
4. RatioPidController corrects wheel ratio each loop iteration.
5. When elapsed time meets or exceeds DUR, MotorController calls `stop()`.
6. Firmware emits a completion response.

**Postconditions**:
- Both motors have stopped. Encoders reflect distance traveled.

**Error Flows**:
- If an X/STOP command arrives during execution, motors stop immediately and timed drive is cancelled.

---

## UC-003: Drive Robot a Specific Distance

**ID**: UC-003
**Title**: Drive robot a specific distance (D command)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running and connection is established.
- Encoder resolution (`KER`) has been calibrated.

**Main Flow**:
1. Python host sends `D+LS+RS+DIST` where DIST is distance in mm.
2. CommandProcessor parses the command and calls `MotorController.startDriveClean(leftSpeed, rightSpeed, distanceMm)`.
3. MotorController zeros cumulative encoder distances, zeros PID state, sets target speeds and distance target.
4. Each loop iteration: RatioPidController corrects ratio; MotorController checks cumulative distance against target.
5. When both wheels have traveled >= DIST mm, MotorController calls `stop()`.
6. Firmware emits a completion response.

**Postconditions**:
- Both motors have stopped. Both encoders reflect the commanded distance within ratio PID tolerance.

**Error Flows**:
- If an X/STOP command arrives during execution, motors stop immediately and distance drive is cancelled.
- If encoder reads fail, MotorController stops motors and reports an error.

---

## UC-004: Stop Robot Immediately

**ID**: UC-004
**Title**: Stop robot immediately (X/STOP command)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running. Motors may or may not be active.

**Main Flow**:
1. Python host sends `X` or `STOP`.
2. CommandProcessor dispatches to `MotorController.stop()`.
3. MotorController sets all motor PWM to 0 via NezhaV2, clears drive state.
4. Firmware emits a stop acknowledgment.

**Postconditions**:
- All motors are stopped. Drive state is cleared.

**Error Flows**:
- None. Stop is always honored regardless of current drive state.

---

## UC-005: Query Encoder Positions

**ID**: UC-005
**Title**: Query encoder positions (ENC command)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running and connection is established.

**Main Flow**:
1. Python host sends `ENC`.
2. CommandProcessor calls `NezhaV2.readEncoder(left)` and `NezhaV2.readEncoder(right)`.
3. CommandProcessor formats response as `ENC+L+R` using sign-prefixed integers.
4. Firmware sends the response.

**Postconditions**:
- Python host has current encoder tick counts for both wheels.

**Error Flows**:
- If I2C read fails, firmware returns an error response.

---

## UC-006: Query and Zero Dead-Reckoning Odometry

**ID**: UC-006
**Title**: Query and zero dead-reckoning odometry (SO/SZ commands)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running. Odometry has been integrating encoder deltas since last reset.

**Main Flow — Query (SO)**:
1. Python host sends `SO`.
2. CommandProcessor calls `Odometry.getPose()`.
3. Firmware formats and sends `SO+X+Y+H` (position in mm, heading in integer units).

**Main Flow — Zero (SZ)**:
1. Python host sends `SZ`.
2. CommandProcessor calls `Odometry.reset()`.
3. Odometry zeroes accumulated X, Y, and heading.
4. Firmware sends acknowledgment.

**Postconditions**:
- For SO: Python host has current dead-reckoning pose.
- For SZ: Odometry pose is (0, 0, 0).

**Error Flows**:
- None under normal operation. Odometry is computed from encoder deltas in RAM.

---

## UC-007: Set Odometry from External Source

**ID**: UC-007
**Title**: Set odometry from external source (SI command)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running. An external pose source (e.g., camera) has computed a corrected pose.

**Main Flow**:
1. Python host sends `SI+X+Y+H`.
2. CommandProcessor parses X, Y, H and calls `Odometry.setPose(Pose{X, Y, H})`.
3. If an ExternalPoseProvider is active (future), it also receives the injected pose.
4. Firmware sends acknowledgment.

**Postconditions**:
- Dead-reckoning odometry is updated to the injected pose. Subsequent SO queries reflect the new pose.

**Error Flows**:
- None. SI is always accepted; it is the caller's responsibility to provide valid values.

---

## UC-008: Read Line Sensor

**ID**: UC-008
**Title**: Read line sensor (LS command)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running. Line sensor is connected to the configured port.

**Main Flow**:
1. Python host sends `LS`.
2. CommandProcessor calls `LineSensor.read()`.
3. Firmware formats and sends `LS+VAL`.

**Postconditions**:
- Python host has the current line sensor reading.

**Error Flows**:
- If the sensor read fails, firmware returns an error response.

---

## UC-009: Read Color Sensor

**ID**: UC-009
**Title**: Read color sensor (CS command)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running. Color sensor is connected via I2C.

**Main Flow**:
1. Python host sends `CS`.
2. CommandProcessor calls `ColorSensor.read()`.
3. Firmware formats and sends `CS+R+G+B+C` (red, green, blue, clear channels).

**Postconditions**:
- Python host has the current RGBC color reading.

**Error Flows**:
- If I2C read fails, firmware returns an error response.

---

## UC-010: Control Gripper Servo

**ID**: UC-010
**Title**: Control gripper servo (gripper angle command)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running. Gripper servo is connected to Nezha V2 servo port.

**Main Flow**:
1. Python host sends the gripper angle command with angle in degrees (0-180).
2. CommandProcessor calls `GripperServo.setAngle(degrees)`.
3. GripperServo calls `NezhaV2.setServoDegrees(servoPort, degrees)`.
4. Firmware sends acknowledgment.

**Postconditions**:
- Gripper servo is positioned at the commanded angle.

**Error Flows**:
- Angle values outside 0-180 are clamped. I2C errors are reported.

---

## UC-011: Read/Write GPIO Ports

**ID**: UC-011
**Title**: Read and write GPIO ports (P/PA commands)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running. GPIO ports are configured.

**Main Flow — Read (P)**:
1. Python host sends `P+PORT`.
2. CommandProcessor calls `PortIO.read(port)`.
3. Firmware sends `P+PORT+VAL`.

**Main Flow — Write (PA)**:
1. Python host sends `PA+PORT+VAL`.
2. CommandProcessor calls `PortIO.write(port, value)`.
3. Firmware sends acknowledgment.

**Postconditions**:
- For P: Python host has current port value.
- For PA: Port is set to the commanded value.

**Error Flows**:
- Invalid port number returns an error response.

---

## UC-012: Initialize and Read OTOS Sensor

**ID**: UC-012
**Title**: Initialize and read OTOS sensor (OI/OP commands)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running. OTOS sensor is connected via I2C.

**Main Flow — Initialize (OI)**:
1. Python host sends `OI`.
2. CommandProcessor calls `OtosSensor.init()`.
3. OTOS sensor initializes over I2C and begins tracking.
4. Firmware sends acknowledgment.

**Main Flow — Query Pose (OP)**:
1. Python host sends `OP`.
2. CommandProcessor calls `OtosSensor.getPose()`.
3. Firmware formats and sends `OP+X+Y+H`.

**Postconditions**:
- For OI: OTOS sensor is initialized and tracking.
- For OP: Python host has current OTOS-reported pose.

**Error Flows**:
- If I2C communication fails during OI, firmware returns an error. OP before OI returns zero pose or error.

---

## UC-013: Calibrate OTOS IMU

**ID**: UC-013
**Title**: Calibrate OTOS IMU (OK command)
**Actor**: Python host

**Preconditions**:
- OTOS sensor is initialized (UC-012).
- Robot must be stationary during calibration.

**Main Flow**:
1. Python host sends `OK`.
2. CommandProcessor calls `OtosSensor.calibrateImu()`.
3. OTOS performs IMU calibration (robot must not move during this period).
4. Firmware sends acknowledgment when calibration completes.

**Postconditions**:
- OTOS IMU is calibrated. Heading drift is minimized.

**Error Flows**:
- If the robot moves during calibration, OTOS calibration quality is degraded. Firmware does not detect motion during calibration.

---

## UC-014: Tune Calibration Parameters at Runtime

**ID**: UC-014
**Title**: Tune calibration parameters at runtime (K* commands)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running.

**Main Flow**:
1. Python host sends any K* command (e.g., `KCP+500`, `KTW+142`).
2. CommandProcessor parses the parameter name and value.
3. CommandProcessor updates the corresponding parameter in the firmware's parameter store.
4. The updated parameter takes effect immediately on the next relevant computation.
5. Firmware sends acknowledgment.

**Postconditions**:
- The named calibration parameter has been updated. Motor control, navigation, and G command behavior reflect the new value.

**Error Flows**:
- Unknown K* parameter name returns an error response.
- Value out of valid range is clamped or rejected with an error response.

---

## UC-015: Drive to Relative XY Position

**ID**: UC-015
**Title**: Drive to relative XY position (G command)
**Actor**: Python host

**Preconditions**:
- Robot firmware is running. Encoder resolution (`KER`) and trackwidth (`KTW`) are calibrated.

**Main Flow**:
1. Python host sends `G+X+Y+Speed`.
2. CommandProcessor calls `ArcComputer.computeArc(X, Y, KTW)` to derive `leftMm` and `rightMm`.
3. **Phase 1 — Pre-rotate** (if heading error > KGT degrees): MotorController rotates robot in place to face the target. Completes before Phase 2 begins.
4. **Phase 2 — Arc drive**: CommandProcessor calls `MotorController.startDriveClean()` with left and right encoder targets derived from `leftMm` and `rightMm`.
5. Each control loop: RatioPidController corrects wheel ratio; MotorController checks both encoder targets against done tolerance `KGD`.
6. When both encoder targets are met within `KGD` mm, MotorController stops motors.
7. Firmware emits `G+DONE`.

**Postconditions**:
- Robot has moved to approximately (X, Y) relative to its starting pose. `G+DONE` has been sent.

**Error Flows**:
- If X/STOP arrives during execution, motors stop immediately and `G+DONE` is not emitted.
- If `ty` (Y component) is zero, arc radius is infinite (straight line); `leftMm == rightMm == distance`. Handled as a degenerate arc.

---

## UC-016: Path Following with PurePursuit

**ID**: UC-016
**Title**: Path following with PurePursuit algorithm
**Actor**: Robot firmware (triggered by path-following command or API)

**Preconditions**:
- A waypoint path has been loaded into the firmware.
- PurePursuit is selected as the active PathFollower.
- A PoseProvider is active (OTOS or dead-reckoning).

**Main Flow**:
1. Main loop calls `PathFollower.computeSpeeds(currentPose, path)` each iteration.
2. PurePursuit selects the lookahead point at a fixed distance ahead on the path.
3. PurePursuit computes arc curvature needed to reach the lookahead point.
4. PurePursuit returns `WheelSpeeds` for left and right motors.
5. MotorController applies the commanded speeds with ratio PID correction.
6. Loop continues until `PathFollower.isDone()` returns true.

**Postconditions**:
- Robot has traversed the waypoint path. Motors are stopped.

**Error Flows**:
- If pose provider fails, path following halts and an error is reported.

---

## UC-017: Path Following with Stanley Controller

**ID**: UC-017
**Title**: Path following with Stanley controller
**Actor**: Robot firmware (triggered by path-following command or API)

**Preconditions**:
- A waypoint path has been loaded into the firmware.
- Stanley controller is selected as the active PathFollower.
- A PoseProvider is active.

**Main Flow**:
1. Main loop calls `PathFollower.computeSpeeds(currentPose, path)` each iteration.
2. Stanley controller computes cross-track error at the front axle and heading error relative to the closest path segment.
3. Stanley controller computes steering correction using the Stanley gain parameter.
4. Stanley controller returns `WheelSpeeds` scaled to apply the correction.
5. MotorController applies the commanded speeds with ratio PID correction.
6. Loop continues until `PathFollower.isDone()` returns true.

**Postconditions**:
- Robot has traversed the waypoint path with Stanley cross-track correction. Motors are stopped.

**Error Flows**:
- If pose provider fails, path following halts and an error is reported.

---

## UC-018: Device Discovery

**ID**: UC-018
**Title**: Device discovery via HELLO/DEVICE announcement
**Actor**: Python host / Robot firmware

**Preconditions**:
- Robot firmware is running. Serial or radio connection is available.

**Main Flow — Host-initiated**:
1. Python host sends `HELLO`.
2. CommandProcessor triggers Announcer to emit the discovery response.
3. Firmware sends `DEVICE:+name+version` over the same channel (serial or radio).

**Main Flow — Firmware-initiated (periodic)**:
1. Announcer fires on a periodic timer.
2. Firmware broadcasts `DEVICE:+name+version` over radio and serial.
3. Python host receives the announcement and identifies the robot.

**Postconditions**:
- Python host has identified the robot's name and firmware version.

**Error Flows**:
- If neither serial nor radio is available, announcement is silently skipped.

---

## UC-019: Radio Relay Mode

**ID**: UC-019
**Title**: Radio relay mode (> prefix commands, < prefix replies)
**Actor**: Python host (via radio bridge micro:bit)

**Preconditions**:
- Robot firmware is running.
- A second micro:bit acts as a USB-serial bridge on the host side.
- Both micro:bits are on radio group 10.

**Main Flow**:
1. Python host sends a command prefixed with `>` over the bridge micro:bit's serial port.
2. Bridge micro:bit transmits the command over radio to the robot micro:bit.
3. Robot firmware receives the radio packet, strips the `>` prefix, and passes the command string to CommandProcessor.
4. CommandProcessor processes the command normally.
5. Firmware formats the response, prefixes it with `<`, and transmits it over radio.
6. Bridge micro:bit receives the radio packet and forwards it over USB serial to the Python host.

**Postconditions**:
- Python host receives the response as if communicating over serial directly.

**Error Flows**:
- Radio packet loss causes the Python host to time out waiting for a response. Retry is the host's responsibility.
- Commands that exceed the radio packet MTU are rejected with an error.
