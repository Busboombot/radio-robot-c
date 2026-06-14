---
id: '007'
title: 'Fix SerialConnection relay handshake: HELLO classify + !GO data-plane'
status: done
use-cases:
- SUC-012
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix SerialConnection relay handshake: HELLO classify + !GO data-plane

## Description

Bench finding after tickets 001–006: `make_robot()` → `SerialConnection.connect()`
fails with "No device found". Hardware validation (team-lead, 2026-06-13) identified
the **real root cause**: announcement-detection failure, not DTR-muting.

**Corrected root-cause analysis (supersedes earlier ticket draft):**

- **`dtr=False` does NOT suppress/mute the relay.** Opening with `dtr=False` and
  sending `HELLO` still gets an immediate `DEVICE:...` banner and plain `PING`/`SNAP`
  work. DTR's only role is **reset**: toggling DTR on open resets any micro:bit
  (via DAPLink), which causes the relay to emit its boot `DEVICE:` announcement.
  Without the reset there is no unsolicited boot banner.

- **The actual failure** was that `connect()` started the reader thread before
  capturing the `DEVICE:` line. The reader loop classified `DEVICE:...` as "other"
  and **dropped it silently** (line 322: "All other lines: drop silently"). So even
  when the relay responded to `HELLO`, the announcement was never captured →
  "No device found".

- **Secondary issue:** the old code used `>PING` (relay prefix) for the readiness
  poll and `>+` for keepalives. After `!GO` the relay is a transparent pipe; those
  `>` prefixes were forwarded verbatim to the robot which does not understand them.

The fix: run `_banner_classify()` **before** the reader thread starts, consuming the
`DEVICE:` line directly from `_ser.readline()`. For `RADIOBRIDGE` devices, run
`_relay_handshake()` (`?` → `!ECHO OFF` → `!MODE RAW250` → `!GO`) also pre-reader.
After `!GO` all traffic is plain (no `>` prefix).

**Authoritative protocol references:**
- Relay protocol spec: `microbit-radio-relay/docs/radio-relay-protocol.md`
  (data plane §2/§5, command plane §3, banner §3.4, defaults §3.6).
- Knowledge note: `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`
  (NOTE: that note contains a now-disproven claim about `dtr=False` — see above).

### Required handshake algorithm

The fix lives entirely in `host/robot_radio/io/serial_conn.py` (connect / handshake /
mode handling) and possibly `host/robot_radio/robot/connection.py` (make_robot's
mode detection):

**Step 1 — Open with DTR asserted.**
Remove the `dtr=False` kwarg at `serial_conn.py:192` (pyserial default is DTR
asserted). DTR-on-open resets the relay and triggers its `DEVICE:` banner.

**Step 2 — HELLO-repeat classify.**
Send `HELLO` repeatedly (with a short inter-send delay and an overall timeout) until
a `DEVICE:<ROLE>:...` banner line arrives. Parse the ROLE field:
- `RADIOBRIDGE` (role `relay`) → relay present; `!GO` data plane required.
- `NEZHA2` (role `robot`) → direct USB connection to robot; no `!GO`.

**Step 3 (relay path) — Pre-flight + enter data plane.**
Before `!GO`, issue:
- `!ECHO OFF` — disable transponder echo.
- `!MODE RAW250` — headerless 250-byte framing (the robot peer runs RAW250).

Then send `!GO`. The relay replies with `# entering data plane`. After `!GO` the relay
is a transparent pipe.

Optionally confirm the robot peer by sending a plain `HELLO` through the data plane
and checking for a `DEVICE:NEZHA2:robot:...` response (useful for unit-test simulation
and belt-and-suspenders bench verification).

**Step 4 (relay path) — Post-`!GO` transparent operation.**
All subsequent commands are **plain** (no `>` prefix): `PING`, `SNAP`, `G`, the `+`
keepalive, etc. Lines beginning with `#` are relay comments/status — the read loop
must silently ignore them (do not treat as protocol errors).

**Step 5 (direct path).**
If ROLE was `NEZHA2`, operate plain as today's "direct" mode — no changes to command
encoding; no `!GO`.

**Net effect:** The obsolete `>`-prefix "relay" mode is replaced by the
`HELLO → classify → !GO → transparent plain` handshake. After the handshake a relay
connection is indistinguishable from the direct path (same plain send, same `+`
keepalive, same `#<id>` corr-id).

### Telemetry caveat (note — do NOT expand scope here)
Async `STREAM` frames may be dropped by the radio bridge; `SNAP` (request/reply) is
reliable through the relay. `Nezha.refresh()` already uses SNAP. The `go_to(on_tick=...)`
callback path relies on streamed TLM, which may be lossy through the relay. This is
noted as a possible follow-up; it is out of scope for this ticket.

## Acceptance Criteria

- [x] `make_robot()` / `SerialConnection.connect()` successfully connects through the
      current `RADIOBRIDGE` relay: HELLO-repeat classifies it as a relay, sets echo
      OFF + RAW250, sends `!GO`, and subsequent plain `PING` / `ID` / `SNAP` reach
      the robot.
- [x] Direct-robot (NEZHA2) connection still works: no `!GO`, plain commands, public
      API unchanged.
- [x] The reader loop ignores lines beginning with `#` (relay comments) and does not
      raise protocol errors on them.
- [x] The keepalive (`+`) and corr-id (`#<id>`) paths work in post-`!GO` transparent
      mode — plain send, no `>` prefix.
- [x] Unit tests pass (see Testing below) — 19 new tests in
      `host/tests/test_serial_relay_handshake.py`, all passing.
- [x] All existing host tests stay green (`host/tests/`); no public API broken
      (577 total = 558 original + 19 new).
- [ ] **LIVE bench verification** (DEFERRED TO TEAM-LEAD): after flashing confirmed
      firmware (`VER` → `fw=0.20260612.28`), `make_robot()` connects, `robot.refresh()`
      returns a populated `RobotState`, and the ticket-006 demo
      (`uv run --group calibrate python host_tests/playfield_tour/playfield_random_tour.py`)
      runs end-to-end. Programmer code + mocked tests are complete; team-lead runs
      the hardware end-to-end verification.

## Implementation Plan

### Files to modify

- `host/robot_radio/io/serial_conn.py`
  - Remove `dtr=False` at the `serial.Serial(...)` construction site.
  - Replace the current relay-mode branch (which sends `>PING` with the `>` prefix)
    with the new `HELLO → classify → !ECHO OFF → !MODE RAW250 → !GO` sequence.
  - Add a `_banner_classify(timeout_s)` private helper that loops sending `HELLO` and
    reads lines until it sees `DEVICE:<ROLE>:...` or times out. Returns `"relay"` or
    `"robot"`.
  - Update the read loop to skip (not error on) lines starting with `#`.
  - Remove or gate the `>`-prefix logic so it is never used on the new path.

- `host/robot_radio/robot/connection.py` (if needed)
  - If `make_robot()` has a hard-coded relay/direct mode switch based on the old
    protocol, update it to rely on the new classify result from `SerialConnection`.

### Files to create

- `host/tests/test_serial_conn_handshake.py`
  - Mock `serial.Serial` at the pyserial boundary.
  - **Relay scenario**: fake relay emits `DEVICE:RADIOBRIDGE:relay:...` banner on
    `HELLO`, ACKs `!ECHO OFF` / `!MODE RAW250` / `!GO` with `# entering data plane`,
    then echoes plain commands transparently. Assert:
    - DTR was asserted (not forced False) on open.
    - Handshake sequence is exactly: HELLO → `!ECHO OFF` → `!MODE RAW250` → `!GO`.
    - Post-`!GO` sends are plain (no `>` prefix).
    - `#`-prefixed relay status lines are ignored by the reader.
  - **Direct-robot scenario**: fake device emits `DEVICE:NEZHA2:robot:...` banner.
    Assert: no `!GO` sent; subsequent commands are plain.
  - Both scenarios use `uv run --with pytest python -m pytest host/tests/ -q`.

### Approach notes

- Do not add a new public constructor parameter to `SerialConnection` or `make_robot`
  for the mode — derive the mode entirely from the `HELLO` classify step. The public
  API is unchanged.
- The `HELLO`-repeat loop should have a configurable (but defaulted) timeout, not a
  fixed retry count, to tolerate relay boot-up delay.
- After `!GO`, the relay relay is a transparent byte pipe — the existing line-reader
  and corr-id machinery requires no changes beyond ignoring `#` lines.

### Testing plan

```
uv run --with pytest python -m pytest host/tests/ -q
```

Run this after every change. The test must cover both the relay and direct-robot
code paths without requiring hardware.

### Documentation updates

- Update the docstring on `SerialConnection.__init__` / `connect()` to describe the
  new HELLO-classify handshake and the `!GO` data-plane transition.
- Note the relay-comment (`#`) filtering in the read-loop docstring or inline comment.
- The ticket-006 demo (`host_tests/playfield_tour/playfield_random_tour.py`) already
  uses `make_robot()` — no demo changes needed; the fix is internal to `SerialConnection`.
