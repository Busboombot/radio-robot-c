---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 028 Use Cases

## SUC-001: Developer runs calibration and results are applied to firmware

- **Actor**: Developer (human or agent)
- **Preconditions**: Robot is connected. One of the three calibration workflows
  (angular, linear, or interactive `rogo calibrate`) is available.
- **Main Flow**:
  1. Developer runs a calibration command (`rogo calibrate distance` or
     `rogo calibrate turns`, or the top-level scripts).
  2. The calibration procedure collects trials, computes new scale values using
     shared helpers (`scale_to_int8`, `mean_stdev`, `_deep_merge`).
  3. Results are pushed to firmware via `_push_calibration` (a single shared
     implementation — the same function regardless of which entry point was
     used).
  4. Updated values are offered for save to the active robot config JSON.
  5. On the next `rogo` invocation, `_make_robot` reads the saved config and
     re-pushes the same values consistently.
- **Postconditions**: Calibration values live in exactly one place; push logic
  is not duplicated between cli.py, robot_mcp.py, and calibrate.py.
- **Acceptance Criteria**:
  - [ ] Single `push_calibration` implementation in `robot_radio/calibration/`
        that both CLI and MCP call.
  - [ ] `_deep_merge`, `scale_to_int8`, and `mean_stdev` helpers exist in one
        module; removed from all other locations.
  - [ ] A CI check (a8 lint, sprint 025) catches any calibration key that is
        pushed but not read in `source/`.

---

## SUC-002: Developer constructs a robot connection consistently from CLI or MCP

- **Actor**: Developer (human using rogo CLI, or agent using MCP server)
- **Preconditions**: A serial relay is connected. Port resolution may be
  automatic (cached session), explicit (--port flag), or probed.
- **Main Flow**:
  1. User invokes any CLI command, or the agent calls an MCP tool that requires
     a robot connection.
  2. Port resolution, HELLO handshake, mode detection, and robot-object
     construction are handled by a shared `make_robot(...)` function.
  3. The session cache is read/written using the same logic regardless of
     entry point.
  4. The calibration freshness check and conditional push run through the shared
     `push_calibration` path.
- **Postconditions**: A connected `Nezha` (or other robot) object is returned
  to the caller. The caller is CLI or MCP — both receive the same object from
  the same construction logic.
- **Acceptance Criteria**:
  - [ ] `_make_robot` logic extracted to a module importable by both cli.py and
        robot_mcp.py.
  - [ ] robot_mcp.py `_connect()` calls the shared constructor rather than
        duplicating port/mode detection.
  - [ ] No `_make_robot` / `_connect` divergence for the handshake or
        calibration push.

---

## SUC-003: Host observes TLM stream without drops or silent gaps

- **Actor**: Host software (agent or test harness)
- **Preconditions**: Robot is connected and STREAM has been issued. Robot may be
  idle, then driving, then idle.
- **Main Flow**:
  1. Host issues `STREAM 50` on the serial channel.
  2. Robot emits periodic TLM frames — each frame carries a `seq=<n>` field.
  3. When the robot transitions to IDLE (motion stops), the stream does not go
     silent; it continues at a low idle rate (`max(tlmPeriodMs, 500) ms`).
  4. A radio command arrives; it does NOT steal the serial TLM stream (channel
     is bound at STREAM time, not retargeted by subsequent commands).
  5. Host-side `TLMFrame` parser surfaces `seq`; a drop-rate counter detects
     gaps and exposes `drop_rate`.
- **Postconditions**: The host can verify stream health (< 2% drop rate during
  a full G run). The stream survives idle→drive→idle without the host needing
  to reconnect.
- **Acceptance Criteria**:
  - [ ] TLM frames include `seq=<n>` (uint16 wrap-around).
  - [ ] `TLMFrame.seq` is populated by `parse_tlm()`.
  - [ ] Drop rate < 2% measured over a 60 s drive.
  - [ ] Stream survives idle→drive→idle without host reconnect.
  - [ ] Radio command does not redirect serial TLM stream.
  - [ ] STREAM handler moves the `tlmPeriodMs < 20` clamp out of
        `telemetryEmit`; `telemetryEmit` no longer mutates config.

---

## SUC-004: Live config change via SET is type-safe and range-validated

- **Actor**: Developer or agent running a tuning session
- **Preconditions**: Robot is running firmware. `SET` is used to tune a live
  control parameter.
- **Main Flow**:
  1. Agent sends `SET tw=0` (invariant violation).
  2. Firmware parses the value with end-pointer validation (not raw `atof`).
  3. A candidate config is constructed; validation checks per-field ranges
     and cross-field invariants (`tw > 0`, `vWheelMax > steerHeadroom`,
     `ctrlPeriod > 0`, `rotationalSlip ∈ [0.5, 1]`, etc.).
  4. Because `tw=0` violates the invariant, the entire SET is rejected; the
     live config is unchanged; the robot replies with the offending key.
  5. A valid multi-key `SET` applies all keys atomically.
- **Postconditions**: The live control config is either unchanged (on any parse
  failure or range violation) or fully updated (all keys valid).
- **Acceptance Criteria**:
  - [ ] Typed parsing with end-pointer validation (non-numeric input rejected
        with ERR).
  - [ ] Per-field range check + cross-field invariant check before applying.
  - [ ] Multi-key SET applies atomically or not at all.
  - [ ] `SET tw=0` rejected with clear ERR; live config unchanged.

---

## SUC-005: SNAP frame reflects live robot state (not stale state)

- **Actor**: Diagnostic tools, field-024 regression
- **Preconditions**: Robot is driving (motion command active).
- **Main Flow**:
  1. Host sends `SNAP` while a G or T command is executing.
  2. Firmware builds the TLM frame from the current `state.inputs` (the same
     struct used by `telemetryEmit`).
  3. Returned frame shows `mode != IDLE` and non-zero `enc` reflecting the
     active motion state.
- **Postconditions**: SNAP and STREAM frames are consistent for the same instant.
  The field-024 SNAP/STREAM discrepancy is closed.
- **Acceptance Criteria**:
  - [ ] SNAP handler confirmed to read `state.inputs` (not `state.target`).
  - [ ] Sim test: SNAP issued during active motion returns `mode != IDLE` and
        `enc != (0,0)`.
  - [ ] If SNAP was reading a stale struct, fix applied; if it required D10,
        documented and cross-referenced.
