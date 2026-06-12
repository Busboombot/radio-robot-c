---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 026 Use Cases

## SUC-001: Sim exercises the same dispatch path as hardware

**Parent:** (infrastructure / developer tooling)

- **Actor:** Firmware developer / test suite
- **Preconditions:** `host_tests/sim_api.cpp` is built and loaded by pytest. The
  firmware codebase has a wired `CommandQueue` path in `run_blocks()`.
- **Main Flow:**
  1. A test sends a converter command (S, T, D, G, TURN, RT) via `sim.send_command()`.
  2. `sim_api.cpp` enqueues the command via `cmd.process()` (which pushes to
     `CommandQueue`).
  3. `sim_tick()` drains the queue by calling `cmd.dequeueOne(q)` — exactly the same
     call `run_blocks()` makes.
  4. The converter handler runs, pushes a VW `ParsedCommand` onto the queue.
  5. The next `dequeueOne()` call dispatches `handleVW`, which calls `begin*()`.
  6. The test observes the resulting motion state and reply.
- **Postconditions:** Every converter command in sim follows the identical code path
  as on hardware. No hand-mirrored loop body exists in `sim_api.cpp`.
- **Acceptance Criteria:**
  - [ ] `sim_command("TURN 9000")` executes via the queue path in sim, not the direct
    `begin*()` fallback.
  - [ ] `test_vw_converters.py` passes against the queue path.
  - [ ] The "MUST mirror" comment is absent; CI grep-lint blocks its reappearance.

---

## SUC-002: Firmware loop body has a single authoritative form

**Parent:** (infrastructure / CI)

- **Actor:** CI pipeline / firmware developer
- **Preconditions:** `LoopScheduler::run_blocks()` is the firmware main loop.
  `sim_api.cpp::sim_tick()` historically hand-mirrored the same loop body.
- **Main Flow:**
  1. Developer calls `LoopScheduler::tickOnce(now)` from `run_blocks()`.
  2. `sim_tick()` calls the same `tickOnce(now)` instead of its own copy.
  3. Any change to the loop body is made in one place.
  4. CI grep-lint catches the string "MUST mirror" and fails the build if it appears.
- **Postconditions:** One loop body, two callers. Divergence between sim and hardware
  is structurally impossible at the loop level.
- **Acceptance Criteria:**
  - [ ] `LoopScheduler::tickOnce(uint32_t now)` exists and is called by both
    `run_blocks()` and `sim_tick()`.
  - [ ] `sim_tick()` contains no independent copies of watchdog, halt, drive, odometry,
    OTOS, or TLM blocks.
  - [ ] `grep -rn "MUST mirror" source/ host_tests/` returns nothing.
  - [ ] CI build fails if "MUST mirror" is reintroduced.

---

## SUC-003: Protocol dispatch and reply formatting are outside the control layer

**Parent:** (architecture / A2)

- **Actor:** Firmware developer
- **Preconditions:** `source/control/` contains `MotionController.cpp` which currently
  includes `CommandProcessor.h` and `CommandQueue.h` and calls
  `CommandProcessor::replyOK/Err/Evt`.
- **Main Flow:**
  1. A converter command (S, T, D, G, TURN, RT) arrives on the wire.
  2. It is parsed and dispatched in `source/app/` — the protocol/app layer.
  3. The app layer calls the appropriate typed `begin*()` entry point on `MotionController`.
  4. Motion completes; `MotionController` fires an event through a narrow callback
    interface (`MotionEventSink` or equivalent).
  5. The app layer receives the event and formats the `EVT done ...` wire reply.
  6. `source/control/` files include no `CommandProcessor.h` or `CommandQueue.h`.
- **Postconditions:** The control layer is testable with a stub event sink and no
  protocol headers. Protocol changes cannot accidentally break motion behavior and
  vice versa.
- **Acceptance Criteria:**
  - [ ] `grep -rl 'CommandProcessor.h\|CommandQueue.h' source/control/` returns nothing.
  - [ ] Converter/dispatch unit tests live in `source/app/` or `host_tests/`.
  - [ ] Motion state machine tests run with a stub event sink and no protocol headers.

---

## SUC-004: One reply per converter command, by construction

**Parent:** (protocol correctness / D11)

- **Actor:** Host software / `test_protocol_v2.py`
- **Preconditions:** The a2 refactor (SUC-003) has moved reply formatting to `app/`.
  The queue path is wired in sim (SUC-001).
- **Main Flow:**
  1. Host sends `G 400 300 200` (a converter command with a corr-id).
  2. The converter handler in `app/` replies `OK goto x=400 y=300 speed=200 #id` once.
  3. `handleVW` (now in `app/`) calls `beginGoTo()` but does NOT emit a second reply.
  4. Host receives exactly one `OK` for the command.
- **Postconditions:** No duplicate OK replies on any converter command path. The
  mechanism is structural (one reply owner), not a `quiet=true` patch.
- **Acceptance Criteria:**
  - [ ] `host/tests/test_protocol_v2.py` asserts exactly one OK per converter command
    on the queue path.
  - [ ] Direct `VW` commands still receive exactly one `OK vw`.
  - [ ] The test runs in sim (queue wired) and would have caught the original D11 defect.

---

## SUC-005: Hardware smoke ritual passes after a clean flash

**Parent:** (system integration / acceptance)

- **Actor:** Team-lead / programmer agent
- **Preconditions:** The refactored firmware has been built with `--clean` and flashed
  to the robot. Sprint 025's trustworthy serial stream is in place.
- **Main Flow:**
  1. `SAFE` query returns `on`.
  2. Four sequential `TURN 9000` commands complete; robot returns within ~10° of start.
  3. `G` to each corner of a square completes; return-to-start error < 100 mm.
  4. Stream drop-rate during the square run is < 5% (measurable from seq gaps).
- **Postconditions:** The refactored firmware behaves equivalently to pre-refactor on
  all observable motion behaviors. No regressions in field behavior.
- **Acceptance Criteria:**
  - [ ] All four smoke steps pass after a clean flash.
  - [ ] No spurious double-OK lines appear in the host's raw protocol log.
  - [ ] `EVT done` lines arrive for every self-terminating command.
