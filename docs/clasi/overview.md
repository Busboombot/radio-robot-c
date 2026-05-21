# radio-robot-c: Project Overview

## What It Is

radio-robot-c is a C++ firmware port of the radio-robot TypeScript/micro:bit firmware. It runs on a DFRobot QBot Pro — a micro:bit V2 paired with a Nezha V2 motor board — using the CODAL framework. The firmware receives movement and sensor commands from a Python host over serial and micro:bit radio, executes them on the robot hardware, and returns telemetry.

## Why It Exists

The original TypeScript firmware was developed as a functional prototype. This port replaces it with a clean, object-oriented C++ implementation that:

- Is maintainable and testable at the module level
- Enables advanced motor control algorithms not practical in TypeScript (ratio PID, arc-to-goal navigation)
- Runs closer to the metal for tighter timing on encoder-based odometry
- Keeps the Python host stack (`robot_radio/` package) entirely unchanged — full wire protocol compatibility is required

The Python host must connect, issue commands, and receive responses identically to the TypeScript version. No protocol changes are permitted.

## Key Technical Differentiators

**Pluggable path-following architecture.** A `PathFollower` pure-virtual interface decouples the path-following algorithm from the command processor. PurePursuit and Stanley controller implementations are provided. A `PoseProvider` pure-virtual interface similarly decouples pose estimation, with OTOS sensor and dead-reckoning implementations and a future hook for external camera pose via the SI command.

**Ratio PID motor control.** Rather than simple velocity PI with ratio cross-coupling, the firmware tracks cumulative encoder distance since each command start and applies a PID controller on the normalized distance ratio between wheels. This eliminates drift over long runs. Confirmed accuracy: 340/339 mm final encoder over a 2-second run (0.3% error).

**Arc-to-goal G command.** The G command computes an arc from the robot's current pose to a relative XY target, optionally pre-rotating when the heading error exceeds a threshold, then drives the arc using encoder targets derived from the arc geometry. This enables point-to-point navigation without continuous pose feedback.

**No heap allocation in the hot path.** All subsystem instances are static. No dynamic allocation occurs during command execution or sensor reads. The firmware targets C++14 and is built via Docker CODAL toolchain with `python build.py`.

## How Success Is Measured

- The Python host connects over serial at 115200 baud and over micro:bit radio at group 10 without modification.
- All 30+ commands (drive, stop, encoder, odometry, sensor, servo, port IO, calibration) produce responses matching the TypeScript firmware's wire protocol.
- The robot drives a straight 2-meter course with less than 1% encoder divergence between wheels.
- The G command navigates to a specified XY offset within the done-tolerance (`KGD`) parameter.
- PurePursuit and Stanley path following complete a defined waypoint route without manual tuning beyond the calibration parameter set.
