---
status: final
sprint: '012'
---

# Sprint 012 Use Cases

## SUC-001 — Accurate Linear Distance

As a student driving the robot forward a commanded distance, the robot
travels within a few percent of the target, and the fused `pose=` x-value
reports the traveled distance in mm (not raw OTOS LSB).

- **Actor**: Student / operator
- **Preconditions**: Robot powered, connected over relay, OTOS present
- **Main Flow**:
  1. Issue `D 200 200 1000` (1 m forward).
  2. Robot drives, TLM emits `pose=` updates.
  3. At stop, `pose=` x is approximately 1000 (mm).
- **Postconditions**: Fused pose x is in mm; matches tape-measure ground truth within a few percent.
- **Acceptance Criteria**:
  - [ ] `pose=` x after 1 m drive is approximately 1000 mm (not ~3279 LSB).
  - [ ] TLM `pose=` values are always in the fused odometry frame.

**Covered by:** T03 (pose units fix), T06 (trackwidth default 126)

---

## SUC-002 — Straight-Line Driving

As a student commanding the robot to drive straight, the robot holds a
straight line with minimal lateral drift because the velocity PID receives
correct per-wheel speed feedback from chip register 0x47.

- **Actor**: Student
- **Preconditions**: Robot powered, chip velocity reads correctly after T04 fix
- **Main Flow**:
  1. Issue `S 200 200` for several seconds.
  2. Robot drives forward over a marked line.
  3. Lateral drift is minimal.
- **Postconditions**: Robot traveled in a straight line; `vel=` shows non-stuck values scaled to command.
- **Acceptance Criteria**:
  - [ ] `vel=` values scale with commanded speed, not pinned at ~30 mm/s.
  - [ ] Chip source flag 'C' shown on `GET VEL`.
  - [ ] Minimal lateral drift during straight-line drive.

**Covered by:** T04 (chip velocity fix), T05 (idle freshness)

---

## SUC-003 — Accurate Symmetric Turns

As a student commanding 90° or 180° in-place turns both CCW and CW, both
directions land within tolerance, thanks to per-direction turn gain/offset
and the correct trackwidth default.

- **Actor**: Student
- **Preconditions**: Robot powered, calibration config loaded, trackwidth=126
- **Main Flow**:
  1. Issue multiple in-place 90°/180° turn commands CCW and CW.
  2. Compare arrival heading to OTOS/camera ground truth.
- **Postconditions**: Both directions within tolerance (e.g. <5° of target).
- **Acceptance Criteria**:
  - [ ] `GET tw` = 126 after boot.
  - [ ] 90° CCW and CW turns are symmetric within tolerance on bench.
  - [ ] Per-direction gain/offset composes correctly with OTOS-corrected go-to pre-rotate.

**Covered by:** T01 (config fields), T06 (defaults + per-direction gain applied)

---

## SUC-004 — Correct Fused Pose Telemetry

As a student or operator reading `pose=` from TLM, the values are in mm and
centidegrees from the fused odometry (encoder + OTOS blend), not raw OTOS LSB.
`OP` retains raw OTOS LSB for cross-check.

- **Actor**: Student / operator
- **Preconditions**: OTOS present and initialized with correct scalars
- **Main Flow**:
  1. Issue `STREAM 100`.
  2. Drive or hand-push the robot.
  3. Observe `pose=` values in TLM.
- **Postconditions**: `pose=` x,y in mm; h in centidegrees. `OP` still returns raw LSB.
- **Acceptance Criteria**:
  - [ ] After a 1 m drive, `pose=` x is approximately 1000 (not ~3279).
  - [ ] `test_tlm_stream.py` and `test_otos_fusion.py` assert mm-scale pose.
  - [ ] `OP` continues to return raw OTOS LSB (labeled as raw).

**Covered by:** T03 (TLM pose fix)

---

## SUC-005 — Go-To Works

As a student issuing `G x y speed`, the robot navigates to the target using a
correct fused pose (mm), arrives within `arriveTol`, and the final pose matches
camera ground truth.

- **Actor**: Student
- **Preconditions**: Correct fused pose in mm (T03 done), trackwidth 126 (T06 done)
- **Main Flow**:
  1. Zero encoders and pose.
  2. Issue `G 500 0 200` (500 mm forward, 0 lateral).
  3. Robot pre-rotates, drives, arrives.
- **Postconditions**: Robot within `arriveTol` of target; final pose matches camera.
- **Acceptance Criteria**:
  - [ ] Robot arrives within `arriveTol` of target.
  - [ ] Final `pose=` values match camera ground truth.
  - [ ] No broken go-to due to LSB-scale pose input.

**Covered by:** T03 (pose fix), T06 (trackwidth/gains)

---

## SUC-006 — Idle Telemetry Freshness

As an operator issuing `SNAP` at rest or hand-pushing the robot, `enc=` and
`pose=` in the TLM frame update immediately, reflecting current position. Motor
actuation remains gated to non-IDLE mode (no unintended twitch).

- **Actor**: Operator / developer
- **Preconditions**: Robot stationary in IDLE mode, TLM streaming or SNAP issued
- **Main Flow**:
  1. Let robot come to rest (IDLE mode).
  2. Issue `SNAP`.
  3. Hand-push robot slightly.
  4. Issue `SNAP` again.
- **Postconditions**: Both snaps return current `enc=`/`pose=`; no motor movement.
- **Acceptance Criteria**:
  - [ ] `SNAP` at rest after motion returns current encoder/pose values.
  - [ ] Hand-push at idle updates `enc=`/`pose=` in next `SNAP`.
  - [ ] No motor twitch at idle.

**Covered by:** T05 (encoder + odometry refresh every tick)

---

## SUC-007 — Chip Velocity Feedback

As a developer monitoring `vel=` or issuing `GET VEL`, the reported per-wheel
velocity scales with commanded speed across the operating range (not stuck at
~30 mm/s). Source flag 'C' indicates chip; 'E' indicates encoder-delta fallback.

- **Actor**: Developer / tuner
- **Preconditions**: T04 fix applied; robot running at various commanded speeds
- **Main Flow**:
  1. Issue `S 100 100`, observe `GET VEL` or `vel=` in TLM.
  2. Increase to `S 200 200`, `S 300 300`.
  3. Confirm velocity readings scale proportionally.
- **Postconditions**: Chip velocity tracks commanded speed; PID receives good feedback.
- **Acceptance Criteria**:
  - [ ] `GET VEL` chip source ('C') scales with command; not pinned at ~30 mm/s.
  - [ ] At idle (speed=0), `vel=` is approximately 0.
  - [ ] Falls back to 'E' (encoder-delta) only on a genuine bad chip read.
  - [ ] `tests/test_readspeed_and_get_vel.py` updated and passing.

**Covered by:** T04 (chip readSpeed fix + plausibility gate)

---

## SUC-008 — Per-Robot Config Loads on Connect

As an operator connecting to the robot over the relay, the host automatically
identifies the robot (by v2 ID device name or serial), loads the matching JSON
config from `data/robots/`, and pushes all calibration values to firmware using
v2 verbs. A subsequent `GET` confirms values match the robot JSON.

- **Actor**: Operator (host, rogo CLI)
- **Preconditions**: `data/robots/<robot>.json` exists; `active_robot.json` or `ROBOT_CONFIG` set
- **Main Flow**:
  1. Connect via `rogo connect` (relay mode).
  2. Host reads ID response (device name/serial).
  3. Host loads matching robot JSON.
  4. Host pushes calibration via v2 SET/OL/OA/OI verbs.
  5. Operator issues `GET tw ml mr`.
- **Postconditions**: Firmware config matches robot JSON (tw=126, ml/mr=known-good, OL/OA set).
- **Acceptance Criteria**:
  - [ ] Host identifies robot by name/serial from v2 ID response.
  - [ ] `_push_calibration()` emits only v2 verbs (SET ml, SET mr, SET tw, OL, OA, OI); no KML/OO/OK.
  - [ ] Post-connect `GET tw/ml/mr` matches robot JSON values.
  - [ ] `OL`/`OA` after connect match scalar computed from JSON otos scales.

**Covered by:** T08 (schema + loader), T09 (v2 connect-time push)
