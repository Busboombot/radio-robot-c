---
status: final
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 007 Use Cases

## SUC-001: Unified configuration — one source of truth for calibration parameters
Parent: firmware-architecture-refactor

- **Actor**: Operator (via K* commands over serial or radio)
- **Preconditions**: Robot firmware is running; `mmPerDegL/R` or `trackwidthMm` may differ between `CommandProcessor::Params` and `CalibParams`.
- **Main Flow**:
  1. Operator issues a `K` parameter-set command (e.g. `KmmPerDegL+0.500`).
  2. CommandProcessor writes to the single `RobotConfig` instance owned by `Robot`.
  3. Both encoder-distance conversion (MotorController) and odometry/arc computation (Odometry, DriveController) read the same field.
- **Postconditions**: All subsystems that consume the updated parameter immediately use the new value; no stale copy exists.
- **Acceptance Criteria**:
  - [ ] `RobotConfig` struct exists in `source/types/Config.h`, replacing `CalibParams` and `CommandProcessor::Params`.
  - [ ] `defaultRobotConfig()` replaces `defaultCalibParams()`.
  - [ ] `MotorController`, `DriveController`, and `Odometry` all hold a `RobotConfig&`.
  - [ ] Setting a `K` parameter via serial verifies convergence: encoder-derived distance and odometry both reflect the new value.

---

## SUC-002: Hardware ownership — MicroBit singleton lives in main.cpp
Parent: firmware-architecture-refactor

- **Actor**: Firmware build system / runtime init sequence
- **Preconditions**: `MicroBit uBit` is currently the first member of `Robot`; `Robot` currently owns and initialises the CODAL singleton.
- **Main Flow**:
  1. `main.cpp` declares `MicroBit uBit;` as a file-scope static and calls `uBit.init()`.
  2. `Robot` is constructed with explicit references to the CODAL peripherals it needs (`uBit.i2c`, `uBit.serial`, `uBit.radio`, etc.).
  3. `Robot` no longer holds a `MicroBit` member; its subsystems receive peripheral references at construction time.
- **Postconditions**: `MicroBit` is not a member of `Robot`; firmware builds and all subsystems initialise correctly.
- **Acceptance Criteria**:
  - [ ] `Robot.h` contains no `MicroBit` member.
  - [ ] `main.cpp` declares `MicroBit uBit;` and calls `uBit.init()` before constructing `Robot`.
  - [ ] Firmware builds with CMake/codal.json; deploys via `mbdeploy deploy --build`.
  - [ ] On the stand: `HELLO` returns `DEVICE:…`; encoders and sensors respond.

---

## SUC-003: Drive state machine encapsulated in DriveController
Parent: firmware-architecture-refactor

- **Actor**: Robot firmware (tick loop)
- **Preconditions**: S/T/D/G drive state machines, S-mode watchdog, streaming counter, and tick() body are currently inside `CommandProcessor`.
- **Main Flow**:
  1. `Robot` constructs a `DriveController` that holds `DriveMode`, S-watchdog, T deadline, D distance snapshot/target, G two-phase state machine, and streaming counter.
  2. `Robot`'s public drive methods (`stop()`, `streamDrive()`, `timedDrive()`, `distanceDrive()`, `goTo()`) delegate to `DriveController::begin*()`.
  3. On each tick, `Robot::tick()` calls `DriveController::tick(dt_ms, sink)`, which advances the state machines and emits completions/telemetry through the sink.
  4. `CommandProcessor` calls Robot drive methods; it holds no drive state.
- **Postconditions**: `CommandProcessor` contains no `DriveMode`, no watchdog timer, no encoder delta fields, no streaming counter.
- **Acceptance Criteria**:
  - [ ] `source/control/DriveController.{h,cpp}` exist.
  - [ ] All drive state variables (`_mode`, `_tEndMs`, `_dEnc*`, `_gPhase`, `_encTickCount`, `_prevOdoEnc*`) are members of `DriveController`, not `CommandProcessor`.
  - [ ] On the stand: S+150+150 spins wheels and streams `ENC…`; T/D/G complete with `*+DONE`; X stops; watchdog fires `SAFETY_STOP` after 200 ms silence.

---

## SUC-004: Visible main loop with reply-sink routing
Parent: firmware-architecture-refactor

- **Actor**: Operator (issuing commands over serial or radio)
- **Preconditions**: The main loop is hidden inside `Robot::run()`; async completions (`T+DONE`, `D+DONE`, `G+DONE`, `SAFETY_STOP`) are hardwired to the serial sink regardless of which channel the command arrived on.
- **Main Flow**:
  1. `main.cpp` contains the `while(true)` loop: drain serial → process with serial sink; drain radio → process with radio sink; call `robot.tick(now_ms, activeSink)`.
  2. The loop tracks `activeSink` — set to the serial or radio sink based on the most recent command source.
  3. `Robot::tick(now_ms, sink)` calls `DriveController::tick(dt_ms, sink)` and passes that same sink for all completions and telemetry.
- **Postconditions**: `Robot::run()` does not exist; the loop lives in `main.cpp`; completions travel back on the channel the triggering command arrived on.
- **Acceptance Criteria**:
  - [ ] `Robot::run()` is removed; `main.cpp` contains the loop.
  - [ ] `Robot::tick(uint32_t now_ms, ReplyFn fn, void* ctx)` exists with no `while` loop inside.
  - [ ] On the stand: issue `T+500+500+2000` over radio; confirm `T+DONE` arrives over radio (not serial). Issue same over serial; confirm `T+DONE` arrives over serial.

---

## SUC-005: Thin CommandProcessor — pure parse-and-dispatch
Parent: firmware-architecture-refactor

- **Actor**: Operator (any command over serial or radio)
- **Preconditions**: `CommandProcessor` injects and stores 8 hardware pointers plus two config structs and owns drive/gripper/odometry state.
- **Main Flow**:
  1. After the refactor `CommandProcessor` holds only `Robot& _robot`.
  2. `process(line, sink)` tokenises and calls `Robot` public action/query methods and component accessors.
  3. K*/O* setters reach subsystems via `robot.config()`, `robot.motor()`, etc.
  4. Query commands call query methods that return value structs; the parser formats them into wire strings.
- **Postconditions**: `CommandProcessor` has no `init()`, no `tick()`, no hardware pointers, no drive/gripper/odometry state.
- **Acceptance Criteria**:
  - [ ] `CommandProcessor` members: `Robot& _robot` only (plus parse helpers).
  - [ ] All commands (X/S/T/D/G, ENC/EZ/SO/SZ/SI, K*/O*/OTOS, LS/CS, gripper, P/PA) work identically over serial and radio.
  - [ ] No `_cal`, `_motor`, `_mc`, `_odo`, `_otos`, `_line`, `_color`, `_gripper`, `_portio` members remain.
  - [ ] On the stand: full smoke sequence passes (HELLO, EZ/ENC, S drive, X stop, SO, LS, CS, K set/dump, gripper).

---

## SUC-006: Clean codebase — no dead state or divergence paths
Parent: firmware-architecture-refactor

- **Actor**: Firmware maintainer
- **Preconditions**: Migration phases may leave fallback null-pointer paths, deprecated structs, or dead fields as scaffolding.
- **Main Flow**:
  1. `defaultCalibParams()` and the old `CommandProcessor::Params` struct are removed.
  2. All `?? default` or null-calibration guard paths inside subsystems are deleted.
  3. All subsystems reference `RobotConfig` exclusively.
- **Postconditions**: No dead code; a single `RobotConfig` path through every subsystem.
- **Acceptance Criteria**:
  - [ ] `CalibParams` struct and `defaultCalibParams()` do not exist in `source/types/Config.h`.
  - [ ] `CommandProcessor::Params` struct does not exist.
  - [ ] No null-cal guard (`_cal == nullptr`) paths remain in MotorController, Odometry, or NezhaV2.
  - [ ] Firmware builds clean (zero warnings on relevant paths).
