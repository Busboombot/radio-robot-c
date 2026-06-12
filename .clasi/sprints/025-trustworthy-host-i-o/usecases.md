---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 025 Use Cases

## SUC-001: Stream survives concurrent command bursts without dropping telemetry or events

- **Actor**: Host Python process (NezhaProtocol / bench tools)
- **Preconditions**: Robot is connected and streaming TLM at a periodic rate; the host
  issues SET/GET/SNAP commands concurrently with the TLM stream.
- **Main Flow**:
  1. Robot emits TLM frames continuously at the configured stream period.
  2. Host sends one or more commands (SET, GET, SNAP) through SerialConnection.
  3. Firmware emits an EVT done line when a motion completes.
  4. Host reads all EVT and TLM lines via wait_for_evt_done() and read_lines().
- **Postconditions**: No TLM frame is silently discarded; every EVT done line that
  firmware emitted is received and acted on by the host.
- **Acceptance Criteria**:
  - [ ] Sending SET/GET/SNAP during a streaming drive does not clear the input buffer.
  - [ ] wait_for_evt_done() never misses an EVT done that was emitted by firmware.
  - [ ] A 60-second stress test (concurrent SNAP + streaming drive) reports zero lost
        TLM frames.
  - [ ] Unit tests confirm the reader thread routes reply/TLM/EVT lines to their
        respective queues independently.

---

## SUC-002: Transport boundary is enforced — _ser is unreachable outside io/serial_conn.py

- **Actor**: Developer adding or modifying host code
- **Preconditions**: The reader-thread design from SUC-001 is in place.
- **Main Flow**:
  1. Developer writes new code that needs to send a raw line or probe the device.
  2. Developer calls a named method on SerialConnection instead of reaching for _ser.
  3. SimConnection exposes the identical interface; no _ser attribute is needed.
  4. CI verifies the boundary on every PR.
- **Postconditions**: All host code above io/ routes through SerialConnection's public
  API; the _ser attribute is private in fact, not just by convention.
- **Acceptance Criteria**:
  - [ ] grep -rn '_ser' host/robot_radio | grep -v io/serial_conn.py returns nothing.
  - [ ] CI step runs that grep and fails the build if any match is found.
  - [ ] sim_conn.py no longer carries the _ser = None stub.
  - [ ] protocol.py, cli.py, and cutebot.py use named SerialConnection methods for
        every operation previously done via _ser.
  - [ ] Existing protocol, CLI, and cutebot behaviour is unchanged (no regression).

---

## SUC-003: Config struct, registry, and firmware usage are always in sync

- **Actor**: Firmware developer adding or removing a config field
- **Preconditions**: Config.h, ConfigRegistry.cpp, and gen_default_config.py exist.
- **Main Flow**:
  1. Developer adds a new field to Config.h.
  2. CI lint script detects the field is not registered in ConfigRegistry.cpp.
  3. CI fails with a clear error listing the offending field.
  4. Developer adds the ConfigRegistry entry (or adds the field to the allowlist with
     a comment), re-pushes, and CI passes.
- **Postconditions**: Every field in Config.h is either registered for GET/SET access or
  explicitly allowlisted; every registered key is either read by firmware source or
  explicitly marked as unused/reserved.
- **Acceptance Criteria**:
  - [ ] Lint script identifies in-struct-not-registered, registered-not-read, and
        registered-not-in-struct mismatches separately.
  - [ ] Current offenders (safetyEnabled, tlmFields, tlmSnapPending unregistered;
        turnScale, distScale registered-but-unread) are resolved or allowlisted.
  - [ ] Adding a bare new field to Config.h without a registry entry breaks CI.
  - [ ] Lint runs in CI on every PR (no manual step).
