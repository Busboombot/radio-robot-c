---
status: ready
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 014 Use Cases

## SUC-001: Deterministic Control Rate at Configured Period

**Parent**: firmware runtime behavior

- **Actor**: Firmware scheduler
- **Preconditions**: Robot is powered on; `controlPeriodMs` is set (default 10 ms).
- **Main Flow**:
  1. `LoopScheduler::run()` fires the control task at the top of every loop iteration.
  2. The control task collects the previously-requested encoder reading, computes per-wheel velocity, runs both `VelocityController` instances, and writes PWM to both motors.
  3. The scheduler computes the next control deadline and sweeps low-priority tasks in the remaining window.
  4. The next encoder request is fired last, just before the idle sleep.
  5. `uBit.sleep()` idles until the deadline.
- **Postconditions**: The control task executes at a stable cadence ≥ 40 Hz (better than the prior two-fiber rate). No busy-wait delays occur in the hot path.
- **Acceptance Criteria**:
  - [ ] Control task wall cost < 1 ms (no I2C blocking in the control task body).
  - [ ] Effective control rate ≥ 40 Hz confirmed on hardware bench (tick-count telemetry or oscilloscope).
  - [ ] Low-priority tasks do not cause control deadline misses under normal sensor load.

---

## SUC-002: Split-Phase Non-Blocking Encoder I2C

**Parent**: firmware HAL

- **Actor**: `LoopScheduler` (via control task), `Motor` HAL
- **Preconditions**: Both motor chips are powered; encoder request/collect methods exist on `Motor`.
- **Main Flow**:
  1. At the top of the control task, `collectEncoder(wheel)` reads back the 4-byte response for the wheel whose request was fired the previous iteration. No busy-wait.
  2. Velocity is computed from the collected encoder delta.
  3. PID runs; PWM is written.
  4. After the low-priority sweep, `requestEncoder(otherWheel)` fires the `0x46` write for the alternate wheel and returns immediately.
  5. The idle sleep provides the vendor-required settling time (≥ 4 ms; actual gap ≥ controlPeriodMs).
- **Postconditions**: No busy-wait loops execute in the motor I2C path. Each wheel's encoder is sampled once every two control iterations (~20 ms at 10 ms period). Sensor I2C tasks run safely between collect and request.
- **Acceptance Criteria**:
  - [ ] `readEncoderRaw()` busy-wait loops deleted from `Motor.cpp`.
  - [ ] `requestEncoder(wheel)` and `collectEncoder(wheel)` are the only encoder I2C entry points used by the loop.
  - [ ] Bench: `GET VEL` returns plausible mm/s values; encoders advance correctly under `S 200 200`.
  - [ ] No I2C corruption observed when sensor tasks (OTOS, line, color) run between request and collect.

---

## SUC-003: Authoritative State Structs Carry All Robot State

**Parent**: firmware internal architecture

- **Actor**: All firmware subsystems
- **Preconditions**: `RobotState.h` defines `MotorCommands`, `HardwareState`, `TargetState`, and `RobotStateContainer`.
- **Main Flow**:
  1. `Robot` owns one `RobotStateContainer` and exposes it to subsystems.
  2. The control task reads `HardwareState.enc*` and writes `HardwareState.vel*` and `MotorCommands.pwm*`.
  3. Odometry reads `HardwareState.enc*` and writes `HardwareState.pose*`.
  4. DriveController reads `HardwareState.pose*` / `inputs.enc*` and `TargetState`, writes `MotorCommands.tgt*Mms`.
  5. Comms-in task writes `MotorCommands` / `TargetState` via `CommandProcessor`.
  6. Telemetry-emit reads all three structs and formats the TLM frame — no sensor I2C occurs at telemetry time.
- **Postconditions**: No subsystem caches a parallel copy of encoder, velocity, or pose state. Private fields in `MotorController` and `Odometry` that duplicated robot state are removed.
- **Acceptance Criteria**:
  - [ ] `HardwareState.enc*`, `vel*`, `pose*` are the single source of truth read by all tasks.
  - [ ] `MotorController` private `_encLMm`, `_encRMm`, `_actualVelL`, `_actualVelR` removed; replaced by reads/writes to `HardwareState`.
  - [ ] Telemetry task does not call any I2C method directly; reads `inputs.*` snapshots.
  - [ ] Pytest suite passes with no changes to the protocol or test fixtures.

---

## SUC-004: Round-Robin Low-Priority Task Scheduling with Budget Gate

**Parent**: firmware runtime behavior

- **Actor**: `LoopScheduler`
- **Preconditions**: Task table is initialized with period, estimated cost, and run/due function pointers for each low-priority task.
- **Main Flow**:
  1. After the control task runs, the scheduler sweeps low-priority tasks from a persistent round-robin cursor.
  2. Before launching each task, the scheduler checks: `now + task.estCostMs > controlDeadline`. If true, the sweep breaks and control runs again.
  3. If the task is not due (`now - lastRunMs < periodMs`), the cursor advances without running.
  4. If due and within budget, the task runs; the cursor advances.
  5. After each task, `systemTime() >= controlDeadline` is rechecked; if true, the sweep breaks.
  6. After the sweep, the next encoder request fires, then `uBit.sleep()` until the deadline.
- **Postconditions**: Expensive I2C tasks (OTOS ≈ 2 ms, line ≈ 1 ms, color ≈ 1 ms) run only when their lag period has elapsed and there is budget remaining before the next control deadline. The round-robin cursor ensures no task is permanently starved.
- **Acceptance Criteria**:
  - [ ] Setting `lag.otos 500` delays OTOS correction without affecting control rate.
  - [ ] Setting `lag.line 0` causes line sensor to refresh every loop iteration (budget permitting).
  - [ ] Budget gate prevents any single low-priority task from causing a control deadline miss.
  - [ ] Bench: stress test with rapid radio commands — control rate holds steady.

---

## SUC-005: Inline Drive Completion Events (EVT Ring Removed)

**Parent**: firmware drive protocol

- **Actor**: `DriveController` (drive-advance task), host operator
- **Preconditions**: A `T`, `D`, or `G` drive command has been issued; `TargetState.replyFn/replyCtx` captures the originating channel.
- **Main Flow**:
  1. The drive-advance task advances the S/T/D/G state machine.
  2. When a completion condition is met (time expired, distance reached, goal converged, or streaming watchdog fired), the task calls `TargetState.replyFn` directly with the EVT string.
  3. The originating channel (serial or radio) receives `EVT done T/D/G` or `EVT safety_stop`.
- **Postconditions**: The EVT ring buffer and `enqueueEvt`/`drainEvents` are removed. Completions are delivered inline on the same thread with no delay beyond the next time the drive-advance task runs (which is every loop iteration for `periodMs=0` tasks).
- **Acceptance Criteria**:
  - [ ] `DriveController` contains no `_evtQueue`, `_evtHead`, `_evtTail`, `enqueueEvt`, `drainEvents` members or methods.
  - [ ] Bench: `T 200 200 1000` → `EVT done T` delivered to the originating channel within one loop iteration of completion.
  - [ ] Streaming watchdog: `S 200 200` with no subsequent keepalive → `EVT safety_stop` delivered inline.
  - [ ] `G` command completion EVT routes to the originating channel even when comms arrived on a different channel.

---

## SUC-006: Per-Sensor Lag Registry (Runtime Tunable)

**Parent**: firmware configuration

- **Actor**: Host operator via `SET lag.* <ms>` commands
- **Preconditions**: `lag.otos`, `lag.line`, `lag.color`, `lag.ports` entries exist in `CommandProcessor`'s `kRegistry`.
- **Main Flow**:
  1. Operator issues `SET lag.otos 200`.
  2. `CommandProcessor` writes the value to the corresponding `uint32_t` field in `RobotConfig` (or `HardwareState.*.lagMs` — see architecture for field placement).
  3. The OTOS task's `due()` check uses the updated lag.
  4. `GET lag.otos` returns `200`.
- **Postconditions**: Sensor update rates are independently tunable at runtime without reflashing. Setting a lag to `0` forces that sensor to refresh every loop iteration.
- **Acceptance Criteria**:
  - [ ] `SET lag.otos 0` causes OTOS to read every loop; `SET lag.otos 500` limits it to ~2 Hz.
  - [ ] `GET lag.*` returns the currently active value.
  - [ ] Default lags match the issue spec: otos=100, line=50, color=100, ports=50.
  - [ ] Lag values survive within a session (no reset on unrelated commands).

---

## SUC-007: Single-Loop Firmware Build and Hardware Bench Verification

**Parent**: sprint acceptance gate

- **Actor**: Developer, hardware bench
- **Preconditions**: All sprint tickets are complete; firmware builds cleanly with `--clean`.
- **Main Flow**:
  1. Clean build + flash; HELLO banner and boot icon appear.
  2. `S 200 200` → both wheels spin; `GET VEL` returns plausible mm/s; encoders advance.
  3. Streaming watchdog fires: send `S 200 200` then stop sending → `EVT safety_stop` after `sTimeoutMs`; motors stop.
  4. `T 200 200 1000` → wheels drive; `EVT done T` arrives on the originating channel.
  5. `D` and `G` commands → completions delivered inline.
  6. Lag tuning: `SET lag.line 0` → fresh line values every loop; `SET lag.color 500` → slow updates while control rate unaffected.
  7. Stress: rapid radio commands while driving → control rate holds steady.
  8. Pytest suite (`uv run --with pytest python -m pytest`) passes clean.
- **Postconditions**: The single-loop firmware is verified end-to-end on real hardware. Sprint 013's deferred bench gate is closed.
- **Acceptance Criteria**:
  - [ ] Clean build produces no warnings about removed symbols.
  - [ ] All pytest tests pass.
  - [ ] Hardware bench smoke sequence completes without errors.
  - [ ] Control rate ≥ 40 Hz confirmed (tick-count or timing telemetry).
  - [ ] No motor throb or velocity anomalies observed during steady driving.
