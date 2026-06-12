---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 030 Use Cases

## SUC-001: Robot executes a multi-segment motion sequence without pose corruption
Parent: Firmware correctness

- **Actor**: Host controller issuing a queued D-then-G sequence
- **Preconditions**: Robot is powered on, OTOS fusion is OFF (encoder-only odometry)
- **Main Flow**:
  1. Host issues a D (distance drive) command
  2. D completes; host issues a G (go-to) command
  3. Robot drives to the G target using the pose computed immediately after the D
- **Postconditions**: Robot arrives at the correct world-frame destination; no backward
  pose jump; `ekf_rej` does not spike when fusion is ON
- **Acceptance Criteria**:
  - [ ] D-then-G sim test passes with fusion OFF (pose continuity, no jump)
  - [ ] `ekf_rej` count after a D command with fusion ON is zero or negligible
  - [ ] `ZERO enc` with nonzero accumulators produces no frozen-encoder window

## SUC-002: Firmware queue path is active on hardware from first boot
Parent: Firmware correctness

- **Actor**: Host controller sending commands over serial/radio
- **Preconditions**: Firmware has completed boot Phase 3 (main.cpp)
- **Main Flow**:
  1. Host sends a command (e.g. G or T) after boot
  2. Firmware dispatches it through the queue path (not the immediate path)
  3. A safety stop fires; host sends another command
  4. Firmware continues dispatching through the queue path (no mode flip)
- **Postconditions**: Dispatch semantics are identical to the sim; behavior is not
  history-dependent on whether a safety stop has ever fired
- **Acceptance Criteria**:
  - [ ] Boot test asserts `cmd._queue` is non-null after Phase 3 reassignment
  - [ ] Dispatch path is the queue path before and after a safety stop in sim

## SUC-003: `SET tlmPeriod` without a prior STREAM does not crash the firmware
Parent: Firmware correctness

- **Actor**: Host or developer issuing `SET tlmPeriod=100` on a fresh connection
- **Preconditions**: Firmware running; no STREAM command has been issued
- **Main Flow**:
  1. Host issues `SET tlmPeriod=100`
  2. Firmware validates that no TLM function is bound
  3. Firmware either suppresses TLM emission or returns ERR (no HardFault)
- **Postconditions**: Robot continues operating normally; no HardFault
- **Acceptance Criteria**:
  - [ ] `SET tlmPeriod=100` with no STREAM does not crash (sim test)
  - [ ] STREAM over serial followed by a radio command keeps TLM on the serial channel

## SUC-004: Self-terminating motion commands preempt cleanly with correct terminal events
Parent: Firmware correctness

- **Actor**: Host controller sending S or a new motion verb while TURN/G/T/D is active
- **Preconditions**: A self-terminating motion command is active
- **Main Flow**:
  1. Host issues `S 0 0` mid-TURN (or issues a T while a G is active)
  2. Firmware cancels the active command, emits `EVT cancelled` for its corrId
  3. New command starts cleanly (no BVC jump, no zombie stop condition)
- **Postconditions**: Host receives exactly one terminal event per corrId; no orphaned
  stop conditions remain active
- **Acceptance Criteria**:
  - [ ] TURN + `S 0 0` mid-turn: TURN completes (P1.1 verify scenario)
  - [ ] G preempted by T: host receives `EVT cancelled` for the G's corrId before T runs
  - [ ] No BVC seed jump when S preempts an active command

## SUC-005: Invalid config values are rejected before reaching the motion pipeline
Parent: Firmware correctness

- **Actor**: Host or developer issuing SET with pathological values
- **Preconditions**: Firmware running normally
- **Main Flow**:
  1. Host issues `SET aDecel=-100` (or aMax=0, sTimeout=0, vBodyMax=0, yawRateMax=0)
  2. Firmware validation rejects the value and returns `ERR badval`
  3. Live config is unchanged; motion continues normally
- **Postconditions**: No runaway, no NaN in the trapezoid path, no X storm
- **Acceptance Criteria**:
  - [ ] `SET aDecel=-100`, `SET aMax=0`, `SET sTimeout=0` each return `ERR badval`
  - [ ] Existing config test suite passes without regression

## SUC-006: Queue overflow is reported to the host
Parent: Firmware correctness

- **Actor**: Host controller sending a rapid burst of commands
- **Preconditions**: Firmware in queue mode (post N2 fix)
- **Main Flow**:
  1. Host sends 5 commands in rapid succession (queue capacity is 4)
  2. The 5th command cannot be enqueued
  3. Firmware replies `ERR full` for the dropped command
- **Postconditions**: Host knows which command was dropped and can retry
- **Acceptance Criteria**:
  - [ ] Burst-overflow sim test: 5th command gets `ERR full`/`ERR busy`
  - [ ] A converter whose pushVW fails does not leave the host with a bare OK

## SUC-007: Stale sensor data is suppressed from TLM
Parent: Firmware correctness

- **Actor**: Operator reading robot telemetry during a session
- **Preconditions**: Line or color sensor wedges after successful initial read;
  OR OTOS read fails mid-session
- **Main Flow**:
  1. Sensor stops updating (bus wedge or disconnect)
  2. TLM tick fires
  3. Firmware checks freshness (`now - lastUpdMs <= 2*lagMs`) before publishing
  4. Stale fields are omitted or flagged in TLM
- **Postconditions**: TLM does not publish stale sensor readings indefinitely;
  OTOS fusion skips zero-filled I2C failures on the same tick
- **Acceptance Criteria**:
  - [ ] Stalled line/color sensor stops publishing after ~2×lag (sim test)
  - [ ] Same-tick OTOS read failure does not fuse (0,0,0)/(0,0) into EKF

## SUC-008: HALT TIME/DIST triggers relative to registration time, not boot
Parent: Firmware correctness

- **Actor**: Host registering a HALT TIME 5000 condition during a session
- **Preconditions**: Robot has been running for more than 5 seconds; no prior `ZERO T`
- **Main Flow**:
  1. Host issues `HALT TIME 5000`
  2. Firmware baselines the time at registration (captures `now`)
  3. Halt fires ~5000 ms after registration, not ~5000 ms after boot
- **Postconditions**: HALT does not trip on the next tick; slot table is reusable
- **Acceptance Criteria**:
  - [ ] `HALT TIME 5000` long after boot does not fire immediately (sim test)
  - [ ] Repeated add/remove does not exhaust the 8-slot table

## SUC-009: Low-severity correctness cleanup — spurious events, truncation, dead code
Parent: Firmware correctness

- **Actor**: Developer/operator interacting with the robot
- **Preconditions**: Robot running; host using correlation ids up to 16 chars
- **Main Flow**:
  1. PURSUE backtrack re-gate does not emit `EVT cancelled` for the G's corrId
  2. A 16-char corrId round-trips intact on the queue path
  3. EKF Q effect is invariant to loop rate
  4. Invalid `sensor=` on the queue path returns ERR before OK
  5. GET over serial returns the full config without truncation
  6. Dead code (RatioPidController, PID_BYPASS, Odometry::update(), DriveMode::TIMED)
     removed from the firmware
- **Postconditions**: Host-side event correlation is reliable; EKF is tuned
  independent of bus load; no vestigial code paths
- **Acceptance Criteria**:
  - [ ] PURSUE re-gate emits no spurious `EVT cancelled` for the G's corrId (sim)
  - [ ] 16-char corrId round-trips intact on queue path (sim)
  - [ ] EKF Q scaling sim test passes (Q effect invariant to loop rate)
  - [ ] Invalid `sensor=` on queue path returns ERR (sim)
  - [ ] Dead code removed; build passes
  - [ ] GET over serial returns full config (bench-confirmed or chunked)
