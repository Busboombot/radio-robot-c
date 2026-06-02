---
status: final
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 009 Use Cases

---

## SUC-001: Large-Message Round-Trip Verification
Parent: (no prior UC — new capability)

- **Actor**: Developer / tester
- **Preconditions**: Firmware flashed; RadioRelay connected; host has a serial/radio connection open.
- **Main Flow**:
  1. Host sends `ECHO <~200-byte payload>` over the relay.
  2. Firmware tokenizes the verb, preserves the payload exactly (case included), replies `OK echo <payload>`.
  3. Host verifies the returned payload byte-for-byte.
- **Postconditions**: Fragmentation + reassembly in both directions confirmed; no truncation at the old 255-byte ceiling.
- **Acceptance Criteria**:
  - [ ] `ECHO` of a 200-byte ASCII payload returns `OK echo <payload>` intact.
  - [ ] Test works over the radio relay (multi-fragment path), not only serial.
  - [ ] `REASM_MAX`, `_buf`, and TX buffer are all raised to ~512; no buffer overflow.

---

## SUC-002: Liveness and Identity Query
Parent: (no prior UC — new capability)

- **Actor**: Host controller / developer
- **Preconditions**: Firmware running; connection open.
- **Main Flow**:
  1. Host sends `PING`.
  2. Firmware replies `OK pong t=<robot_ms>` with the current system time.
  3. Host records T0 (before send) and T1 (after receive); computes clock offset `(T0+T1)/2 − t_r`.
  4. Host optionally sends `ID` → `ID model=Nezha2 name=… fw=… proto=2 caps=…`.
  5. Host optionally sends `VER` → firmware + protocol version string.
  6. Host optionally sends `HELP` → compact command index.
- **Postconditions**: Host knows the robot is alive, has its identity, and has a clock-offset estimate.
- **Acceptance Criteria**:
  - [ ] `PING` returns `OK pong t=<ms>` with the robot's `uBit.systemTime()`.
  - [ ] `ID` returns `proto=2` and a non-empty `caps=` field.
  - [ ] `VER` and `HELP` return non-empty responses.
  - [ ] Legacy `HELLO` / `DEVICE:` format is absent from firmware.

---

## SUC-003: Named-Key Configuration (SET / GET)
Parent: (no prior UC — replaces K* commands)

- **Actor**: Host controller / calibration tool
- **Preconditions**: Firmware running; connection open.
- **Main Flow**:
  1. Host sends `GET` (no args) → firmware replies `CFG ml=… mr=… tw=… pid.kp=… …` (all params in one line).
  2. Host sends `SET ml=0.487 mr=0.481 tw=120 pid.kp=2.0` → firmware applies valid keys, replies `OK set ml=… mr=… tw=… pid.kp=…`.
  3. Host sends `GET ml pid.kp` → firmware replies `CFG ml=0.487 pid.kp=2.0` (subset).
  4. Host sends `SET badkey=99` → firmware replies `ERR badkey badkey`.
- **Postconditions**: All previously `K*`-settable parameters are reachable by human-readable names; no implicit scaling multipliers; decimals only where fractional.
- **Acceptance Criteria**:
  - [ ] `GET` (no args) returns all config params in a single line that fits in the enlarged buffer.
  - [ ] `SET` of multiple keys at once is reflected by a subsequent `GET`.
  - [ ] Integer params (`tw`, `minSpeed`, `sTimeout`) are formatted as integers; fractional params use decimal.
  - [ ] `SET` with an unrecognized key replies `ERR badkey <key>`.
  - [ ] Legacy `K*` commands are removed.

---

## SUC-004: Unified Telemetry Frame (TLM / STREAM / SNAP)
Parent: (no prior UC — replaces scattered ENC/CS/LS/SO streaming)

- **Actor**: Host controller
- **Preconditions**: Firmware running; sensors attached.
- **Main Flow**:
  1. Host sends `STREAM 40` → firmware emits one `TLM` frame per 40 ms.
  2. Each `TLM` frame: `TLM t=<sample_ms> mode=S enc=1024,1019 pose=350,-12,1780 vel=200,0,15 line=120,340,330,118 color=21,30,18,80` (only present-sensor fields included).
  3. Host sends `SNAP` → firmware emits one immediate `TLM` frame regardless of streaming state.
  4. Host sends `STREAM 0` → telemetry stops.
  5. Host sends `STREAM fields=enc,pose` → only enc and pose fields appear in subsequent `TLM` frames.
- **Postconditions**: All sensor data arrives in a single correlated, timestamped frame; no separate uncorrelated ENC/CS/LS lines.
- **Acceptance Criteria**:
  - [ ] A single `TLM` frame carries enc + pose + available sensors with `t=` at sensor-sample time (not send time).
  - [ ] `STREAM <ms>` controls cadence; `STREAM 0` stops; `SNAP` fires one-shot.
  - [ ] `STREAM fields=…` subset subscription works.
  - [ ] Legacy `ENC`/`CS`/`LS`/`SO` streaming commands (SSE, SSO, SSC, SSL) are removed.
  - [ ] [BENCH] Frames arrive at expected cadence on hardware; `t=` values advance monotonically.

---

## SUC-005: Host-Side Clock Synchronization
Parent: SUC-002 (PING as time probe)

- **Actor**: Host controller
- **Preconditions**: Robot running; PING command available (SUC-002).
- **Main Flow**:
  1. Host fires 5 `PING` requests in rapid succession, records T0/T1 and `t_r` for each.
  2. Host selects the sample with minimum RTT (`T1 − T0`).
  3. Host computes `offset = (T0_best + T1_best)/2 − t_r_best`.
  4. Host translates any subsequent `TLM t=<robot_ms>` to host time via `host_t = t_robot + offset`.
  5. Host re-pings every ~30–60 s to keep offset fresh.
- **Postconditions**: Robot timestamps are mapped into host time; drift stays within ½ minimum RTT over a multi-minute run.
- **Acceptance Criteria**:
  - [ ] After a PING burst, the host's robot→host translation aligns a known robot event (e.g. `EVT done` from a timed drive) with the host clock to within ½ the measured minimum RTT.
  - [ ] Offset remains stable (< 20 ms drift) over a 3-minute run.
  - [ ] Robot clock is never set from the host (monotonic guarantee preserved).
  - [ ] [BENCH] Clock-sync module operates correctly end-to-end over the relay.

---

## SUC-006: V2 Motion Commands (S / T / D / G / STOP / GRIP / ZERO)
Parent: (replaces legacy single-letter packed motion surface)

- **Actor**: Host controller / student programmer
- **Preconditions**: Firmware running; v2 parser active.
- **Main Flow**:
  1. Host sends `S 200 150` → robot enters streaming drive; watchdog resets on each re-send.
  2. Host sends `T 200 150 1000` → timed drive; firmware replies `OK drive l=200 r=150 ms=1000`; `EVT done` on completion.
  3. Host sends `D 200 200 300` → distance drive; `EVT done` on completion.
  4. Host sends `G 300 0 200` → go-to arc; `EVT done` on arrival.
  5. Host sends `STOP` → immediate stop; `OK stop`.
  6. Host sends `GRIP 90` → gripper to 90°; `OK grip deg=90`. `GRIP` (no arg) → `OK grip deg=<current>`.
  7. Host sends `ZERO enc pose` → zeros encoders and odometry; `OK zero enc pose`.
- **Postconditions**: All motion commands use v2 spaced-token format; `G` is unambiguously go-to; `GRIP` is unambiguously gripper; legacy packed format gone.
- **Acceptance Criteria**:
  - [ ] `S`, `T`, `D`, `G`, `STOP` all work with space-separated integer mm args; response uses `OK`/`EVT` taxonomy.
  - [ ] `GRIP <deg>` sets the servo; `GRIP` queries; no ambiguity with `G`.
  - [ ] `ZERO enc` zeros encoders; `ZERO pose` zeros odometry; `ZERO enc pose` zeros both.
  - [ ] `#id` correlation: `T 200 150 1000 #7` → all responses echo `#7`.
  - [ ] `ERR badarg` on out-of-range or malformed args.
  - [ ] [BENCH] Commands drive the physical robot; `EVT done` arrives after completion.

---

## SUC-007: Host Controller Migration to v2
Parent: (depends on SUC-001 through SUC-006)

- **Actor**: Developer / student programmer using the host Python package
- **Preconditions**: `robot_radio` package copied into this repo; firmware speaks v2.
- **Main Flow**:
  1. Developer imports `from robot_radio.robot.nezha import Nezha`.
  2. Nezha connects over serial or radio relay.
  3. Developer calls `robot.ping()` → returns `(robot_ms, offset_ms)`.
  4. Developer calls `robot.drive(200, 150)` (streaming), `robot.stop()`, `robot.go_to(300, 0, 200)`.
  5. Developer calls `robot.get_config()` → returns a dict of all named params.
  6. Developer calls `robot.set_config(ml=0.487)` → applies and confirms.
  7. Developer subscribes to telemetry: `robot.stream(40)` → yields `TLMFrame` objects with host-time-translated `t`.
- **Postconditions**: Host package drives the robot end-to-end over the relay using v2; no legacy protocol code in the package.
- **Acceptance Criteria**:
  - [ ] `robot_radio/` package is present in this repo (copied from scratch location).
  - [ ] `NezhaProtocol` (or equivalent) speaks v2 `OK/ERR/EVT/TLM/CFG/ID` tags.
  - [ ] `SET`/`GET` → `set_config` / `get_config` Python API; no `K*` wire commands.
  - [ ] `TLM` parser produces structured objects; `t` field is translated to host time via clock-sync module.
  - [ ] Streaming drive keepalive uses `S <l> <r>` v2 format.
  - [ ] [BENCH] Host controller drives the robot end-to-end over the relay.

---

## SUC-008: Protocol v2 Specification Document
Parent: (documentation of SUC-001 through SUC-006)

- **Actor**: Developer / future sprint author
- **Preconditions**: Firmware v2 commands are implemented and verified.
- **Main Flow**:
  1. Developer reads `docs/protocol-v2.md` to understand every command, response tag, and error code.
  2. Document mirrors the relay's `radio-relay-protocol.md` style: sections for grammar, response taxonomy, each command group, verification examples.
- **Postconditions**: Single source of truth for the v2 wire protocol.
- **Acceptance Criteria**:
  - [ ] `docs/protocol-v2.md` exists and covers: response taxonomy, liveness/identity, config, telemetry, time-sync, motion, OTOS/port carry-overs, error codes.
  - [ ] All commands match what is implemented in the firmware.
  - [ ] No legacy commands documented.
