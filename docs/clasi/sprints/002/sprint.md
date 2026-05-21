---
id: "002"
title: "Control Layer and Core Motion Commands"
status: open
branch: sprint/002-control-layer-and-core-motion-commands
use-cases: []
issues:
  - plan-c-port-of-radio-robot-firmware
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 002: Control Layer and Core Motion Commands

## Goals

Enable a Python host to connect, drive the robot, read encoders, and query dead-reckoning
odometry. This sprint delivers the control layer (MotorController + Odometry), the
CommandProcessor infrastructure with drive-mode state machine, and all motion, encoder, and
calibration commands. After this sprint, `python robot_radio/qbot_pro.py` can connect and
drive the robot over serial or radio.

## Problem

Sprint 1 produces a robot that boots and answers HELLO but cannot be driven. There is no motor
control, no command dispatch, and no wire-protocol compatibility with the existing Python host
stack. This sprint fills the core gap: making the robot driveable and queryable.

## Solution

Implement MotorController (simple PI + feed-forward — ratio PID comes in sprint 5) and Odometry
in `source/control/`. Implement CommandProcessor in `source/app/` with the drive-mode state
machine (IDLE / STREAMING / TIMED / DISTANCE), S-mode watchdog (200 ms timeout), streaming
encoder/odometry output, and all motion + calibration K commands. Wire CommandProcessor into
Robot's `run()` loop with injected peripheral pointers.

The MotorController interface (`setTarget`, `stop`, `tick(dt_s)`) is designed so sprint 5 can
replace the tick body with the ratio PID without changing any callers.

## Success Criteria

- `S+200+200` over serial drives the robot forward
- `ENC` returns signed encoder positions in mm (format `ENC+NNNN-MMMM`)
- `SO` returns dead-reckoning pose as `SO+XXXX-YYYY+HHH`
- `K` dumps all calibration params with K-command names
- `X` and `STOP` halt the robot immediately
- S-mode watchdog fires and emits `LOG:SAFETY_STOP` after 200 ms without an S refresh
- `python robot_radio/qbot_pro.py` connects successfully and drives the robot
- Streaming encoder output appears every `encReportEvery` (default 2) ticks

## Scope

### In Scope

**Control (`source/control/`)**
- `MotorController` — public `MotorGains gains` (kFF=0.15, kP=0.05, kI=0.20, iClamp=60, kRatio=0.01); `setTarget(leftMms, rightMms)`, `stop()`, `resetIntegrators()`; `tick(dt_s)` runs PI + FF + ratio cross-coupling, applies co-clamp, calls `NezhaV2::setPwm()`; `getActualVelocity(l, r)`, `getEncoderPositions(l, r)`, `resetEncoderAccumulators()`. Integrators reset only on mode change, not on watchdog refresh
- `Odometry` — float internal state; int32_t protocol output (mm, centidegrees); `update(dL_mm, dR_mm, trackwidth_mm)` differential drive integration; `getPose(x,y,h_cdeg)`, `setPose(...)`, `zero()`

**App (`source/app/`)**
- `CommandProcessor` — public `CalibParams calib` (mmPerDegL=0.487, mmPerDegR=0.481, distScale, turnScale, minSpeedMms, tickMs=20, sTimeoutMs=200, encReportEvery=2); drive mode enum (IDLE / STREAMING / TIMED / DISTANCE); command dispatch via `CmdEntry { prefix, handler* }` table (longer prefixes sort first, e.g. `STOP` before `S`); `process(line, replyFn, ctx)` — parse + dispatch; `tick(now_ms)` — motor update, mode timeout checks, streaming output; injected via `init()`: `OtosSensor*`, `LineSensor*`, `ColorSensor*`, `GripperServo*`, `PortIO*` (nullable — all null in this sprint)

**Commands implemented in this sprint:**
- Motion: `X`, `STOP`, `S` (streaming + watchdog), `T` (timed), `D` (distance)
- Encoder/odometry: `ENC`, `EZ`, `SO`, `SZ`, `SI`, `K` (dump all params)
- Calibration setters: `KML`, `KMR`, `KFF`, `KSP`, `KSI`, `KIC`, `KSR`, `KSM`, `KSS`, `KTR`, `KER`, `KSD`, `KST`

**Wire protocol format** (must match TypeScript exactly):
- Arguments: mandatory sign prefix, no spaces (`S+200-150`)
- Relay mode: `>` prefix inbound, `<` prefix outbound
- Responses: `ACK:S +200 +150`, `SO+1234-0567+090`, `ERR:command`, `LOG:SAFETY_STOP`
- Baud: 115200; Radio: group 10, channel 0, power 7

### Out of Scope

- OTOS commands O/OI/OK/OZ/OR/OP/OV/OL/OA (sprint 3)
- Sensor commands LS/CS (sprint 3)
- Peripheral commands G (gripper), P, PA (sprint 3)
- Navigation layer PoseProvider/PathFollower (sprint 4)
- Ratio PID replacement of MotorController tick (sprint 5)
- G go-to command (sprint 5)

## Test Strategy

Hardware-in-the-loop only (CODAL does not support off-device unit testing):

1. Build and deploy: `python build.py` then `python scripts/deploy.py`
2. Motor test: `S+100+100` — robot drives forward; `X` — stops immediately
3. Differential test: `S+100-100` — robot spins in place
4. Timed test: `T+200+200+1000` — drives 1 s then stops automatically, emits ACK
5. Encoder test: `ENC` response format matches `ENC+NNNN-MMMM`; `EZ` zeroes then `ENC` returns `ENC+0000+0000`
6. Odometry test: drive ~500 mm forward manually measured; `SO` shows ~500 mm in forward axis
7. Watchdog test: send `S+100+100`, wait 300 ms without refresh — verify `LOG:SAFETY_STOP`
8. K dump: `K` response includes all parameter names and current values
9. Python host: run `robot_radio/qbot_pro.py` end-to-end connect + drive sequence

## Architecture Notes

**CommandProcessor owns drive mode; MotorController knows only targets and gains.** SRP
separation — MotorController has no protocol knowledge. CommandProcessor calls `setTarget()`
and `stop()` based on parsed commands and mode timeouts.

**Integrators survive watchdog refresh.** `resetIntegrators()` called only on explicit mode
change (new command with different mode or explicit X/STOP). On S re-send (keepalive), mode
stays STREAMING and integrators accumulate. Direct port of confirmed TypeScript behavior —
prevents step-response jerk on each keepalive cycle.

**Command dispatch table.** `CmdEntry[]` linear scan, O(N) at ~30 entries is fast enough at
20 ms ticks. Longer prefixes sort first so `STOP` is found before `S`, `OI` before `O`.

**Sprint 5 interface contract.** MotorController's `tick()` signature is `void tick(float dt_s)`.
Sprint 5 replaces the body only; callers in CommandProcessor::tick are unchanged. Design the
sprint 2 body so the swap is a clean replacement with no callee-visible side effects lost.

**Reference files from the TypeScript original:**
- `radio-robot/src/command.ts` — exact command handler logic to port (893 lines)
- `radio-robot/src/nezha.ts` — PI + FF tick loop, motor gain defaults

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
