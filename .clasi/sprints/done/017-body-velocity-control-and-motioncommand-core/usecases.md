---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 017 Use Cases

## SUC-001: Smooth velocity ramp on VW command
Parent: body-level motion control

- **Actor**: Host program (or operator via rogo)
- **Preconditions**: Robot is idle or already in motion. `BodyVelocityController` is
  owned by `DriveController` and configured with `vBodyMax`, `aMax`, `aDecel`,
  `yawRateMax`, `yawAccMax`.
- **Main Flow**:
  1. Host sends `VW <v_mms> <omega_mrads>`.
  2. Firmware parses the verb, constructs a `MotionCommand` with target `(v, ω)` and
     a single TIME stop condition (duration = `sTimeoutMs`), and starts it.
  3. `BodyVelocityController` receives the target; on each PID dt tick it ramps the
     live `(v, ω)` toward target under `aMax` / `aDecel` / `yawAccMax`, then calls
     `BodyKinematics::inverse` → `saturate` → `MotorController::setTarget`.
  4. Robot accelerates smoothly — no instantaneous wheel-speed step.
  5. Host sends additional `VW` packets within `sTimeoutMs` intervals (keepalive
     re-arms the TIME condition baseline).
- **Postconditions**: Robot travels at the commanded body twist; velocity ramp slope
  equals `aMax` (accelerating) or `aDecel` (decelerating).
- **Acceptance Criteria**:
  - [ ] Host unit test: step-change `v` from 0 to 300 mm/s; measured slope per tick
        equals `aMax` ± 1 mm/s².
  - [ ] Host unit test: step-down from 300 to 0 mm/s; slope equals `aDecel`.
  - [ ] Host unit test: `ω` ramp slope equals `yawAccMax` (deg/s² → rad/s²).
  - [ ] Host unit test: spin-in-place (`v=0, ω>0`) does not set zero wheel targets.
  - [ ] Host unit test: straight drive (`ω=0`) produces `vL == vR` within float tol.
  - [ ] `(v,ω)` within limits: no saturation applied (vL == vR == v).

## SUC-002: Keepalive loss triggers safety stop
Parent: safety / watchdog

- **Actor**: Firmware watchdog (TIME stop condition)
- **Preconditions**: Robot is running a VW MotionCommand with a TIME stop condition
  equal to `sTimeoutMs`. Host has stopped sending keepalive re-sends.
- **Main Flow**:
  1. Each incoming `VW` re-send bumps the TIME stop condition's baseline (`t0` reset to
     `now_ms`).
  2. When no keepalive arrives within `sTimeoutMs`, the TIME condition fires.
  3. `MotionCommand::tick` detects the fire, invokes SOFT-stop (target `(0,0)`, profiler
     ramps down), then emits `EVT safety_stop` and transitions to IDLE.
- **Postconditions**: Motors stop; no stranded command in flight.
- **Acceptance Criteria**:
  - [ ] Host unit test: TIME stop condition with threshold=500 ms fires when
        `now_ms - t0 >= 500`; does not fire one tick before.
  - [ ] Host unit test: re-arming (`t0` bumped) delays firing by the full threshold.
  - [ ] Bench (deferred): keepalive loss → motors ramp to zero within `sTimeoutMs` + ramp
        duration; `EVT safety_stop` emitted.

## SUC-003: Immediate cancel via X (or STOP) verb
Parent: operator control

- **Actor**: Host or operator sending `X` or `STOP`
- **Preconditions**: A `MotionCommand` is active (VW or any other command using the
  framework).
- **Main Flow**:
  1. Host sends `X` (or `STOP`).
  2. `CommandProcessor` calls `DriveController::cancel()`.
  3. `cancel()` calls `MotionCommand::cancel(HARD)`, which calls
     `BodyVelocityController::reset()` and `MotorController::stop()`.
  4. Firmware emits `EVT cancelled`; mode transitions to IDLE.
  5. Host `cancel()` wrapper in `protocol.py` sends the verb and returns.
- **Postconditions**: Motors stop immediately (no ramp); `EVT cancelled` received.
- **Acceptance Criteria**:
  - [ ] Host unit test: `cancel(HARD)` immediately zeros the velocity controller state
        and calls `MotorController::stop()` (mock verified).
  - [ ] Host unit test: SOFT cancel ramps to zero; HARD cancel does not wait.
  - [ ] `STOP` verb produces same teardown as `X`.
  - [ ] Bench (deferred): `X` during active VW → robot stops within one control tick.

## SUC-004: Stop condition fires at correct threshold
Parent: motion termination framework

- **Actor**: Firmware (MotionCommand ticking)
- **Preconditions**: A `MotionCommand` is configured with one or more `StopCondition`
  entries of various kinds.
- **Main Flow**:
  1. `MotionCommand::tick` evaluates each `StopCondition::evaluate()` against current
     `HardwareState` and the captured `MotionBaseline`.
  2. The first condition to return `true` terminates the command (OR-across-array).
  3. The firing condition index is available for EVT reporting.
- **Postconditions**: Command terminates exactly when the first condition fires; a
  zero-condition command never self-terminates.
- **Acceptance Criteria**:
  - [ ] Host unit test: TIME fires when elapsed >= threshold (off-by-one at threshold-1 ms).
  - [ ] Host unit test: DISTANCE fires when `|(encL+encR)/2 - enc0| >= a`; uses raw
        encoder sum (not filtered).
  - [ ] Host unit test: HEADING fires when `|wrap(heading - heading0 - a)| < b (eps)`.
  - [ ] Host unit test: POSITION fires when `sqrt((x-ax)^2+(y-ay)^2) < radius`.
  - [ ] Host unit test: SENSOR fires for both GE and LE comparison directions.
  - [ ] Host unit test: OR-across-array — second condition fires when first has not yet.
  - [ ] Host unit test: zero-condition command evaluates false every tick (never fires).
  - [ ] Host unit test: NONE kind always returns false.

## SUC-005: Config params round-trip via SET/GET
Parent: tuning / configuration

- **Actor**: Operator via `rogo SET`
- **Preconditions**: Firmware built with five new `RobotConfig` fields and corresponding
  `kRegistry[]` entries: `vBodyMax`, `yawRateMax`, `yawAccMax`, `jMax`, `yawJerkMax`.
- **Main Flow**:
  1. Operator sends `SET vBodyMax=300`.
  2. Firmware updates `robot.config.vBodyMax`; replies `OK set vBodyMax=300`.
  3. Operator sends `GET vBodyMax`; firmware replies `CFG vBodyMax=300.000`.
  4. `BodyVelocityController` reads the field live on the next tick.
- **Postconditions**: Config updated; controller respects new limit on the next advance.
- **Acceptance Criteria**:
  - [ ] Host unit test: each of the five new keys present in a mock registry table;
        round-trip encode/decode matches defaults.
  - [ ] Clean build: no compile errors with the five new `RobotConfig` fields.
  - [ ] Bench (deferred): `SET vBodyMax=200` then `VW 300 0` → robot clamps to 200 mm/s.

## SUC-006: MotionCommand lifecycle (configure → start → tick → terminate)
Parent: motion abstraction

- **Actor**: DriveController (internal)
- **Preconditions**: A `MotionCommand` instance is owned by `DriveController`;
  `BodyVelocityController` is also owned.
- **Main Flow**:
  1. `beginXxx()` calls `configure()`, `addStop()` × N, `setReplySink()`, then `start()`.
  2. `start()` snapshots `MotionBaseline` (t0, enc0, heading0, pose0) from `HardwareState`
     and seeds or resets the velocity controller.
  3. Each `driveAdvance` tick calls `MotionCommand::tick()`.
  4. On the tick a stop fires: SOFT → target `(0,0)` → profiler ramps → `EVT done`; HARD →
     `MotorController::stop()` → `EVT cancelled`.
  5. After termination, `active()` returns false; DriveController enters IDLE.
- **Postconditions**: Exactly one EVT emitted per command lifecycle; `active()` false.
- **Acceptance Criteria**:
  - [ ] Host unit test: SOFT teardown — tick sequence reaches `atTarget()==true` before
        `EVT done` emitted.
  - [ ] Host unit test: HARD teardown — `EVT cancelled` emitted on same tick as cancel().
  - [ ] Host unit test: `active()` transitions false only after termination.
  - [ ] Host unit test: reconfiguring the same command instance for a new command (start
        called twice) resets baseline correctly.
  - [ ] Host unit test: SOFT-stop absolute deadline (≈3 s) fires even if `atTarget()` never
        clears (controller stuck at low speed).
