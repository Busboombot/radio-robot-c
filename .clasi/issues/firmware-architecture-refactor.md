---
status: pending
---

# Plan: Firmware architecture refactor — ownership, Robot interface, thin CommandProcessor, visible main loop

## Context

The current firmware structure conflates several responsibilities and the
stakeholder wants it restructured:

- **`MicroBit` is wrong-placed.** `MicroBit uBit` is the first *member of
  `Robot`* (`source/app/Robot.h:35`). MicroBit is the hardware; `Robot` is the
  abstraction *around* it. MicroBit should be declared in `main.cpp` (a
  singleton/global), and `Robot` constructed from references to it.
- **`CommandProcessor` does far too much.** Beyond parsing, it owns the drive
  state machines (S/T/D/G), the S-mode watchdog, streaming telemetry, odometry
  delta tracking, gripper state, and the per-loop `tick()`. It also has hardware
  pointers injected (`init(motor, mc, odo, otos, line, color, gripper, portio)`)
  and two parameter structs (`Params` + `CalibParams*`). It should be a **pure
  parse-and-dispatch** layer that calls public methods on `Robot`.
- **Config is duplicated and can diverge.** `CommandProcessor::Params` and
  `CalibParams` both hold `mmPerDegL/R` and `trackwidthMm`; updating one at
  runtime doesn't update the other (encoder conversion vs odometry/arc use
  different copies).
- **The main loop is hidden** inside `Robot::run()`, and its `tick()` replies are
  hardwired to **serial** even when the command arrived over **radio** — async
  completions (`T+DONE`/`D+DONE`/`G+DONE`/`SAFETY_STOP`) go out the wrong channel.

**Outcome:** `main.cpp` owns `MicroBit` and a **visible main loop**; `Robot` is a
clean hardware abstraction with a public command interface + component accessors +
one config object; `CommandProcessor` only parses and dispatches; replies route to
the channel a command came in on.

**Stakeholder decisions (locked):** one `DriveController` on `Robot` for the
S/T/D/G state machines (MotorController stays wheel-level PID); unify
`Params`+`CalibParams` into a single `RobotConfig`; ship this as its **own issue,
sequenced before** the [[protocol-v2-raw250-hard-break]] parser rewrite.

## Target architecture

**`main.cpp`** — owns the hardware and the loop (visible, not hidden):
- Declare `MicroBit uBit;` as a file-scope singleton; call `uBit.init()`.
- Construct `Robot robot(uBit);` and `CommandProcessor cmd(robot);`.
- Run the main loop here (a free `runLoop(...)` or small `MainLoop` object in
  `main.cpp` is fine). Each iteration: drain serial → `cmd.process(line, sink)`;
  drain radio → `cmd.process(line, sink)`; at the configured cadence call
  `robot.tick(now_ms, activeSink)`. Track the **active reply sink** (serial vs
  radio) from the last command source so robot-driven completions/telemetry go
  back on the right channel (fixes the hardwired-serial bug).

**`Robot`** — abstraction around the micro:bit; does **not** own `MicroBit`:
- Ctor takes `MicroBit&` (or the specific peripheral refs); owns subsystems,
  `RobotConfig`, `DriveController`, `Odometry`, and the optional sensors.
- **Public action interface** (called by CommandProcessor): `stop()`,
  `streamDrive(l,r)`, `timedDrive(l,r,ms)`, `distanceDrive(l,r,mm)`,
  `goTo(x,y,spd)`, `setGripperAngle(deg)`, `zeroEncoders()`, `setPose(...)`,
  `zeroOdometry()`, OTOS init/calibrate/reset/set, digital/analog port I/O.
- **Query interface** returning small structs (not formatted strings):
  encoders, pose, OTOS position/velocity, line, color, ports.
- **Component accessors for configuration** (the stakeholder's model — Robot
  hands out its parts): `config()`, `motor()`, `driveController()`,
  `odometry()`, `otos()`, `lineSensor()`, `colorSensor()`, `gripper()`,
  `portIO()`. CommandProcessor's K*/O* setters reach config/subsystems through
  these.
- **`tick(now_ms, sink)`** — advances `DriveController` + odometry + streaming;
  emits completions/telemetry through the injected reply `sink`. No `while`
  loop here.

**`DriveController`** (new, `source/control/DriveController.{h,cpp}`):
- Holds `DriveMode` + S-watchdog (`_lastSMs`,`sTimeoutMs`) + T deadline + D
  distance snapshot/target + the G `PRE_ROTATE/ARC` state machine + streaming
  counter. Calls `MotorController` for wheel control and `Odometry` for pose.
- Exposes `begin*`/`stop` entry points (called by Robot's drive methods) and a
  `tick(dt, sink)` returning/emitting completion events. This is exactly the
  state inventory currently misplaced in CommandProcessor (`_mode`, `_tEndMs`,
  `_dEnc*`, `_gPhase`, `_gArc*`, `_encTickCount`, `_prevOdoEnc*`).

**`RobotConfig`** (new, in `source/types/Config.h`):
- One struct merging `CommandProcessor::Params` + `CalibParams` with the
  duplicates collapsed (single `mmPerDegL/R`, single `trackwidthMm`). Owned by
  `Robot`, passed **by reference** to `MotorController`, `DriveController`,
  `Odometry`, and `NezhaV2::readEncoder`. Setters mutate this one source of
  truth — no more divergence. `defaultRobotConfig()` replaces
  `defaultCalibParams()`.

**`CommandProcessor`** — pure parser:
- Single member `Robot& _robot`. `process(line, sink)` tokenizes and calls
  `Robot` public methods / component setters via accessors. **Removed:**
  `tick()`, `Params params`, `CalibParams* _cal`, all hardware pointers, all
  drive/go-to/streaming/odometry/gripper state, `init(...)`, `setCalib(...)`.
- Keep the `ReplyFn`/sink mechanism for now; the v2 issue later changes only the
  wire format and reply taxonomy on this already-clean structure.

**`Announcer`** stays as-is (HELLO/identity), invoked from the loop.

## Migration phases (robot stays buildable each step)

1. **`RobotConfig`** — merge the two structs, repoint subsystems to the unified
   ref, remove the duplicate copies. Behavior unchanged.
2. **MicroBit ownership** — move `MicroBit` to `main.cpp`; `Robot` ctor takes
   refs; remove the `uBit` member.
3. **Extract `DriveController`** — move the S/T/D/G/watchdog/streaming state and
   the `tick()` body out of CommandProcessor into the new controller owned by
   `Robot`; add `Robot` drive methods that delegate to it.
4. **Visible main loop + reply routing** — move the `while(true)` from
   `Robot::run()` into `main.cpp`; add `Robot::tick(now_ms, sink)`; track the
   active sink so completions return on the originating channel.
5. **Thin the parser** — reduce `CommandProcessor` to `Robot&` + `process()`;
   add the `Robot` public action/query methods and component accessors; queries
   return structs that the parser formats (unchanged wire strings for now).
6. **Cleanup** — delete dead state/fallbacks; confirm no `?? default` null-cal
   paths remain.

## Critical files

- `source/main.cpp` — MicroBit singleton + visible loop + reply-sink routing.
- `source/app/Robot.{h,cpp}` — refs not ownership; public interface + accessors +
  `tick()`.
- `source/app/CommandProcessor.{h,cpp}` — reduce to `Robot&` + `process()`.
- `source/types/Config.h` — new unified `RobotConfig` + `defaultRobotConfig()`.
- **new** `source/control/DriveController.{h,cpp}` — S/T/D/G state machine.
- `source/control/MotorController.{h,cpp}`, `source/control/Odometry.{h,cpp}` —
  consume `RobotConfig&`; Odometry absorbs its own `_prevOdoEnc*` delta state.
- `source/hal/NezhaV2.{h,cpp}` — `readEncoder` takes `RobotConfig&`.

## Relationships

- **Sequence before** [[protocol-v2-raw250-hard-break]]; that issue then rewrites
  only the parser's wire format/taxonomy on this clean structure.
- [[nezha-chip-velocity-readspeed-0x47]] touches `MotorController` independently;
  add the velocity-source toggle as a `RobotConfig` field when that lands.

## Verification

- Firmware builds (CMake/`codal.json`); deploy via `mbdeploy`.
- **Behavioral parity:** every existing command (X/S/T/D/G, ENC/EZ/SO/SZ/SI, K*
  dump+setters, O*/OTOS, LS/CS, gripper, P/PA) behaves identically over **both**
  serial and radio.
- **Bug fixed:** async completions (`T+DONE`/`D+DONE`/`G+DONE`/`SAFETY_STOP`)
  return on the channel the command arrived on (verify a radio-issued T reports
  done over radio).
- **No config divergence:** set `mmPerDeg`/`trackwidth` via command, confirm both
  encoder conversion and odometry/arc use the new value (single `RobotConfig`).
- Unit-test where feasible: `CommandProcessor` dispatch (parser → mock Robot),
  `DriveController` state transitions, `RobotConfig` defaults.
