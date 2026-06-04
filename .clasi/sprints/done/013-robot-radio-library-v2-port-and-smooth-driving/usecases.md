---
sprint: '013'
status: final
---

# Sprint 013 Use Cases

## SUC-001: Unified library import

**Actor**: Developer / test runner  
**Goal**: Import the full `robot_radio` library in one step.

**Preconditions**: Project virtual environment is active; all prior-library modules have been brought into `host/robot_radio/`.

**Main Flow**:
1. Developer runs `from robot_radio.robot import Nezha, NezhaProtocol`.
2. Import succeeds with no missing-module errors.
3. All public subpackages (`robot`, `sensors`, `nav`, `path`, `controllers`, `kinematics`, `io`, `config`) are importable.

**Postconditions**: Library is importable; no `ImportError` or broken cross-module references.

**Acceptance Criteria**:
- [ ] `uv run python -c "from robot_radio.robot import Nezha, NezhaProtocol"` exits 0.
- [ ] `uv run python -c "from robot_radio import nav, path, controllers, kinematics"` exits 0.

---

## SUC-002: Robot liveness preflight

**Actor**: Any caller that opens a connection to the robot

**Preconditions**: `SerialConnection` can be instantiated (real or mock).

**Main Flow**:
1. Caller calls `connect()`.
2. Library sends `PING`; firmware responds `OK pong t=…`.
3. Library sends `ID`; firmware responds `ID model=Nezha2 name=… …`.
4. Connection is declared live; commands may be sent.

**Failure Flow**: If `PING` or `ID` times out, connection raises a clear error; no further commands are sent.

**Postconditions**: On success, `NezhaProtocol` is in a live, ready state. On failure, an exception is raised before any drive command is sent.

**Acceptance Criteria**:
- [ ] Test with mock serial simulating silence causes `connect()` to raise within the preflight timeout.
- [ ] Test with well-formed `OK pong` + `ID` responses causes `connect()` to succeed.

---

## SUC-003: Smooth blocking drive

**Actor**: Calibration script or any single-move scenario

**Preconditions**: Connection is live (SUC-002 passed); firmware `sTimeoutMs` is 500 ms.

**Main Flow**:
1. Caller calls `robot.speed_for_distance(spd, mm)` or `robot.speed_for_time(spd, ms)`.
2. Library sends `D l r mm` or `T l r ms` (v2 space-delimited format).
3. Library calls `wait_for_evt_done("D"/"T")` — blocking; no streaming watchdog involved.
4. `EVT done D/T` received; function returns.

**Postconditions**: Robot has stopped; encoder/pose state updated from TLM received during the move.

**Acceptance Criteria**:
- [ ] Test confirms `D`/`T` command is encoded in v2 space-delimited format (no sign-prefix `±`).
- [ ] `wait_for_evt_done` returns on `EVT done D/T`.
- [ ] `wait_for_evt_done` raises on `EVT safety_stop`.

---

## SUC-004: Smooth continuous streaming drive (keepalive)

**Actor**: Teleop or path-following controller

**Preconditions**: Connection is live; firmware `sTimeoutMs` is 500 ms.

**Main Flow**:
1. Caller calls `stream_drive(left_mmps, right_mmps)`.
2. Library emits `S l r` and resends at ≤30 % of the watchdog interval (≤150 ms for 500 ms).
3. Robot moves smoothly; no `EVT safety_stop` fires.

**Postconditions**: Robot is moving; keepalive is running; caller can stop by calling `stop()`.

**Acceptance Criteria**:
- [ ] Test confirms resend period is ≤150 ms (i.e., ≤30 % of 500 ms watchdog).
- [ ] Bench test: 3-second `stream_drive` call with firmware at 500 ms produces no safety-stop.

---

## SUC-005: Protocol v2 full command coverage

**Actor**: `NezhaProtocol` (wire layer)

**Preconditions**: `NezhaProtocol` is instantiated with a (real or mock) `SerialConnection`.

**Main Flow**:
1. Caller invokes any `NezhaProtocol` method (drive, timed, distance, go_to, vw, stream_drive, OTOS ops, SET/GET, zero, grip, snap, port, ping, id, ver).
2. Each call produces the correct v2 wire string (space-delimited; no sign-prefix `±`; correct verb from firmware HELP list).
3. Responses are parsed via `parse_response` / `parse_tlm` / `parse_cfg` into typed objects (`ParsedResponse`, `TLMFrame`).

**Postconditions**: Wire-level correctness is fully covered by tests; v1 verbs are absent from the codebase.

**Acceptance Criteria**:
- [ ] `test_protocol_v2.py` covers encoding for every public method and asserts v2 wire format.
- [ ] v1 verbs (`EZ`, `ENC`, `SO`, `SZ`, `SSE`, `SSO`, `SSL`, `SSC`, `TN`, `ROT`, `OO`, `SI`, `K+`) are not present in `host/robot_radio/robot/protocol.py`.
- [ ] `parse_tlm` correctly parses all TLM field combinations, including partial frames.

---

## SUC-006: Sensor TLM parsing and pose tracking

**Actor**: `OdomTracker`, `CamTracker`, any TLM consumer

**Preconditions**: Firmware is emitting v2 `TLM` frames; robot tag is 100.

**Main Flow**:
1. Firmware emits `TLM t=… enc=… pose=…` at each tick.
2. `parse_tlm` parses the frame into a typed `TLMFrame`.
3. `OdomTracker` updates its internal `(x_mm, y_mm, heading_cdeg)` state.
4. `CamTracker` fuses AprilTag 100 observations with pose estimates.

**Postconditions**: Pose and encoder state are up to date in the library after each TLM frame; camera observations with tag 100 are accepted.

**Acceptance Criteria**:
- [ ] Unit tests confirm correct parsing of TLM strings, including partial frames (missing fields).
- [ ] `OdomTracker` state advances correctly across a synthetic drive sequence.
- [ ] `CamTracker` accepts tag-100 observations and rejects other tag IDs if tag filtering is active.

---

## SUC-007: Linear calibration via library

**Actor**: Stakeholder running `tests/calibrate/calibrate_linear.py`

**Preconditions**: Robot is on the bench with power; RADIORELAY is connected; camera (overhead, AprilTag 100) is active; tape measure available.

**Main Flow**:
1. Script instantiates `Nezha` via the library's connect path (preflight: PING/ID).
2. For each trial: laser port 4 is activated; blocking `D` drive issued via `Nezha`; camera (AprilTag 100) and OTOS/encoder distances read via library; stakeholder enters tape measure truth.
3. Script computes updated `mm_per_deg` and `otos_linear_scale`; pushes to robot via `SET ml=…`, `SET mr=…`, `OL …`; writes `data/robots/tovez.json`.
4. `calib_common.py` is absent (deleted).

**Postconditions**: `tovez.json` reflects the calibrated values; no raw serial code remains in the calibration scripts.

**Acceptance Criteria**:
- [ ] `calibrate_linear.py` imports no raw serial module; all robot calls go through `Nezha` or `NezhaProtocol`.
- [ ] `tests/calibrate/calib_common.py` does not exist.
- [ ] Test with mocked `Nezha` + `CamTracker` confirms closed-loop calibration math and JSON write path.

---

## SUC-008: Firmware streaming watchdog at 500 ms

**Actor**: Firmware running on the robot

**Preconditions**: Firmware source is checked out; `source/types/Config.h` is editable.

**Main Flow**:
1. `source/types/Config.h` is updated: `sTimeoutMs = 500`.
2. `source/control/DriveController.cpp` watchdog check uses `_config.sTimeoutMs` (confirmed, not hardcoded).
3. Firmware is rebuilt clean and reflashed.
4. `GET sTimeout` on the connected robot returns `500`.

**Postconditions**: Robot firmware uses a 500 ms streaming watchdog by default; no spurious safety-stops due to relay lag.

**Acceptance Criteria**:
- [ ] `Config.h` diff shows `sTimeoutMs` changed from 200 to 500.
- [ ] Code review of `DriveController.cpp` confirms the watchdog reads `_config.sTimeoutMs` (not a literal `200`).
- [ ] Bench: `GET sTimeout` returns `500` after a clean build and reflash.
