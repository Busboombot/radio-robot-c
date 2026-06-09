---
sprint: "020"
status: draft
---

# Use Cases — Sprint 020: HAL Abstraction, Motion Overhaul, and Command Queue

## SUC-001: Firmware builds with HAL layer, behavior unchanged

- **Actor**: Developer / CI
- **Preconditions**: Sprint 019 firmware on main; `python3 build.py --clean` passes.
- **Main Flow**:
  1. Developer introduces HAL interfaces (IMotor, IOtosSensor, etc.) and NezhaHAL.
  2. Existing concrete classes inherit their interface.
  3. Robot constructor changes to accept `Hardware& hal` instead of 7 device refs.
  4. main.cpp constructs NezhaHAL and passes it to Robot.
  5. Developer runs `python3 build.py --clean`.
- **Postconditions**: Build succeeds with no warnings. Robot behavior on hardware is unchanged.
- **Acceptance Criteria**:
  - [ ] `python3 build.py --clean` succeeds.
  - [ ] Flash and send `D dist=500`; EVT done D received with correct encoder telemetry.
  - [ ] `uv run --with pytest python -m pytest` still passes.

---

## SUC-002: MockHAL simulates motor physics without hardware

- **Actor**: Developer / test harness
- **Preconditions**: HAL interfaces exist (SUC-001 complete).
- **Main Flow**:
  1. Test code instantiates MockHAL with default config.
  2. Sets MockMotor command speed to 100 (full forward).
  3. Calls MockHAL::tick() 100 times at 24 ms intervals.
  4. Reads MockMotor encoder value.
- **Postconditions**: MockHAL compiles on host (native gcc, no CODAL). Encoder reflects accumulated travel.
- **Acceptance Criteria**:
  - [ ] MockMotor encoder grows at >= 80% of kNominalMaxMms after 1 s of ticks.
  - [ ] MockMotor at 0% speed: encoder stable.
  - [ ] MockMotor at -100%: encoder decreases.
  - [ ] `sim_set_enc_l()` injection: read-back matches injected value after one tick.

---

## SUC-003: Host CMake target compiles firmware control logic for native execution

- **Actor**: Developer
- **Preconditions**: MockHAL classes exist (SUC-002 complete).
- **Main Flow**:
  1. Developer runs `cmake -S . -B host_tests/build && cmake --build host_tests/build`.
  2. Python loads the shared library via ctypes.
- **Postconditions**: Shared library produced. No CODAL or MicroBit runtime required.
- **Acceptance Criteria**:
  - [ ] Build command exits 0.
  - [ ] `import ctypes; ctypes.CDLL('./host_tests/build/libfirmware_host.dylib')` loads without error.
  - [ ] `HOST_BUILD=1` macro guards any CODAL-only code paths.

---

## SUC-004: Python host test harness exercises firmware control layers

- **Actor**: Developer / CI
- **Preconditions**: Host CMake build passes (SUC-003 complete).
- **Main Flow**:
  1. Developer runs `uv run --with pytest python -m pytest host_tests/ -v`.
  2. Each test file exercises a layer: MockHAL physics, MotorController PID, motion state machines, command parsing.
- **Postconditions**: All tests pass; no flashing hardware required.
- **Acceptance Criteria**:
  - [ ] `test_mock_hal.py` passes: encoder physics, injection, speed sign.
  - [ ] `test_motor_controller.py` passes: PID convergence, windup clamp, stop.
  - [ ] `test_motion_controller.py` passes: D command completes, keepalive timeout stops motors.
  - [ ] `test_command_processor.py` passes: PING/HELLO/ERR routing, VW convergence, SET/GET.

---

## SUC-005: All motion commands route through BodyVelocityController

- **Actor**: Host operator / teleop script
- **Preconditions**: Phase A complete. BVC has `seedCurrent()` method.
- **Main Flow**:
  1. Operator sends `S 100 100` (streaming) over serial.
  2. Operator sends `G x=500 y=0 speed=200` (goto).
  3. Operator sends `T 200 200 2000` (timed).
- **Postconditions**: All three routes through BVC. No direct MotorController::setTarget bypass exists.
- **Acceptance Criteria**:
  - [ ] S command: `beginStream` converts (vL,vR) to (v,omega) and calls `bvc.seedCurrent` + `bvc.setTarget`. No direct setTarget call to MotorController from S handler.
  - [ ] G PRE_ROTATE: replaced with `bvc.seedCurrent(0, omega) + bvc.setTarget(0, omega)`.
  - [ ] `_lastSMs` member removed from MotionController; system watchdog (`_watchdogMs` on LoopScheduler) handles all modes.
  - [ ] Existing T/D/G/TURN bench behavior unchanged after migration.

---

## SUC-006: Single system watchdog covers all motion modes

- **Actor**: Host operator
- **Preconditions**: Motion BVC unification complete (SUC-005).
- **Main Flow**:
  1. Operator sends `VW v=300 w=0`.
  2. Stops sending all commands for sTimeoutMs milliseconds.
- **Postconditions**: `EVT safety_stop` emitted; motors stop.
- **Acceptance Criteria**:
  - [ ] `EVT safety_stop` fires after sTimeoutMs of no inbound commands.
  - [ ] Sending `+` resets the watchdog; ceasing keepalives after that fires it again.
  - [ ] No per-command watchdog code remains (no `_lastSMs` in MotionController).
  - [ ] S mode also subject to single watchdog (no separate `_lastSMs` path).

---

## SUC-007: Raw VW seeds BVC without ramp

- **Actor**: Trajectory planner / developer
- **Preconditions**: BVC unification complete.
- **Main Flow**:
  1. Robot is stationary.
  2. Operator sends `_VW v=300 w=0`.
- **Postconditions**: BVC current state set to (300, 0) immediately; no trapezoid ramp.
- **Acceptance Criteria**:
  - [ ] After `_VW`, `bvc.currentV()` returns approximately 300 (within one tick tolerance).
  - [ ] No ramp delay: motor PWM reaches target region within the first control tick.

---

## SUC-008: X soft ramps to zero via BVC

- **Actor**: Host operator / HaltController
- **Preconditions**: Robot driving at 300 mm/s via VW.
- **Main Flow**:
  1. Operator sends `X soft`.
- **Postconditions**: BVC target set to (0,0); robot decelerates under aMax; `EVT done` emitted.
- **Acceptance Criteria**:
  - [ ] `X soft` causes BVC-controlled deceleration, not immediate stop.
  - [ ] `EVT done` received once motor speed reaches zero.
  - [ ] Hard `X` still stops immediately (existing behavior unchanged).

---

## SUC-009: HaltController fires named stop condition and emits EVT

- **Actor**: Host operator
- **Preconditions**: Phase B motion overhaul complete.
- **Main Flow**:
  1. Operator sends `ZERO T` (reset time baseline).
  2. Sends `VW v=300 w=0`.
  3. Sends `HALT TIME 1500`.
  4. Waits.
- **Postconditions**: Robot stops at ~1500 ms; EVT emitted with halt ID.
- **Acceptance Criteria**:
  - [ ] `EVT halt id=0` received approximately 1500 ms after ZERO T.
  - [ ] `HALT INFO 0` returns original command string.
  - [ ] `HALT CLEAR` clears all conditions while driving; robot continues without stopping.
  - [ ] After a halt fires, all registered conditions are cleared automatically.

---

## SUC-010: HALT DIST stops robot at a measured distance

- **Actor**: Host operator
- **Preconditions**: Phase B complete.
- **Main Flow**:
  1. Operator sends `ZERO D` (reset distance baseline).
  2. Sends `VW v=300 w=0`.
  3. Sends `HALT DIST 400`.
  4. Waits.
- **Postconditions**: Robot stops at approximately 400 mm encoder average.
- **Acceptance Criteria**:
  - [ ] `EVT halt id=0` fires when encoder average delta reaches 400 mm.
  - [ ] `HALT DIST 500 SOFT` triggers BVC ramp to zero (soft stop).

---

## SUC-011: HALT LINE fires when any line sensor crosses threshold

- **Actor**: Host operator
- **Preconditions**: Phase B complete; line sensor functional.
- **Main Flow**:
  1. Operator sends `VW v=200 w=0`.
  2. Sends `HALT LINE ANY GE 200`.
  3. Robot drives over a line.
- **Postconditions**: EVT fires on first sensor crossing threshold.
- **Acceptance Criteria**:
  - [ ] `HALT LINE ANY GE 200` fires when any of line[0..3] >= 200.
  - [ ] `HALT LINE 2 GE 200` fires only when channel 2 crosses threshold.

---

## SUC-012: ACCESS_HARDWARE flag enables command filtering in test mode

- **Actor**: Developer / test harness
- **Preconditions**: Phase C command-flags work complete; `run_test()` enabled in main.cpp.
- **Main Flow**:
  1. Serial commands sent: `S 100 100`, then `OZ`, then `GET velKp`.
- **Postconditions**: S handler pushes VW to queue; VW is skipped; OZ is skipped; GET runs.
- **Acceptance Criteria**:
  - [ ] `S 100 100` runs handler (not flagged); handler pushes VW to queue; VW is skipped with `DBG skip VW`.
  - [ ] `OZ` produces `DBG skip OZ` (ACCESS_HARDWARE).
  - [ ] `GET velKp` produces `OK get velKp=<value>` (not ACCESS_HARDWARE).

---

## SUC-013: CommandQueue ring buffer enqueues and dequeues in order

- **Actor**: CommandProcessor (internal)
- **Preconditions**: Phase C queue work complete.
- **Main Flow**:
  1. push_back A, B, C.
  2. push_front Z.
  3. pop_front four times.
- **Postconditions**: Items dequeue in order Z, A, B, C.
- **Acceptance Criteria**:
  - [ ] pop_front returns items in correct order.
  - [ ] push_back on full buffer returns false.
  - [ ] empty() and size() return correct values.

---

## SUC-014: OP reads cached state, not OTOS device

- **Actor**: Host operator
- **Preconditions**: Phase C OP refactor complete.
- **Main Flow**:
  1. Operator sends `OP` normally.
  2. Operator sends `OP` inside `run_test()` mode.
- **Postconditions**: In both cases, OP returns values from `state.inputs.otosX/Y/H`.
- **Acceptance Criteria**:
  - [ ] `OP` response values match `state.inputs.otos*` fields, not a fresh OTOS device read.
  - [ ] In `run_test()` mode, `OP` is NOT listed as `DBG skip OP` (not ACCESS_HARDWARE).
