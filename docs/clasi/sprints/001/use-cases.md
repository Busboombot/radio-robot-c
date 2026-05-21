---
sprint: "001"
---

# Sprint 001: Use Cases

## SUC-001: Firmware Boots and Emits DEVICE Announcement

- **Actor**: Operator
- **Preconditions**: Firmware flashed to micro:bit V2; serial monitor open at 115200 baud
- **Main Flow**:
  1. Operator powers on or resets the micro:bit
  2. Firmware executes `main()`, constructs `Robot`, calls `robot.run()`
  3. `Robot` initializes subsystems in order: I2C → sensors → motor → serial → radio → announcer
  4. `Announcer` emits `DEVICE:<type>:<name>:<hwName>:<serial>\n` over serial
  5. Robot enters the tick loop: drain serial, drain radio, `uBit.sleep(20)`
- **Postconditions**: The `DEVICE:` string appears on the serial monitor; firmware is running
- **Acceptance Criteria**:
  - [ ] `python build.py` produces a `.hex` with no errors
  - [ ] Serial output contains `DEVICE:` prefix within 3 seconds of power-on
  - [ ] Announcement string contains four colon-separated fields after `DEVICE:`

---

## SUC-002: Host Sends HELLO, Firmware Re-emits Announcement

- **Actor**: Host computer (via serial)
- **Preconditions**: Firmware is running and in its tick loop (SUC-001 complete)
- **Main Flow**:
  1. Host sends `HELLO\n` over serial
  2. `SerialPort.readLine()` returns the line on the next tick
  3. `Announcer` intercepts the line before any other command processing
  4. `Announcer` re-emits the same `DEVICE:<type>:<name>:<hwName>:<serial>\n` string
- **Postconditions**: Host receives the announcement string in response to HELLO
- **Acceptance Criteria**:
  - [ ] Sending `HELLO\n` at any time produces the `DEVICE:` announcement
  - [ ] Response arrives within two tick periods (≤ 40 ms) of the newline being received
  - [ ] The response string is identical to the boot-time announcement

---

## SUC-003: Firmware Stays Stable for 60 Seconds Without Input

- **Actor**: Time / watchdog
- **Preconditions**: Firmware is running in its tick loop; no serial or radio input
- **Main Flow**:
  1. Firmware executes tick loop continuously, sleeping 20 ms each iteration
  2. No commands arrive; no peripheral faults occur
  3. 60 seconds elapse
- **Postconditions**: Firmware is still running; no panic LED pattern visible
- **Acceptance Criteria**:
  - [ ] No micro:bit panic pattern (flashing sad face / error code) appears after 60 s
  - [ ] Serial remains responsive: sending `HELLO\n` after 60 s still produces the announcement
  - [ ] No assertion or hard-fault reset observed
