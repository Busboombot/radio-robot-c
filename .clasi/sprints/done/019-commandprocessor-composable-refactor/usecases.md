---
status: final
sprint: 019
---

# Sprint 019 Use Cases

## SUC-001: Subsystem Registers Its Own Commands

- **Actor**: Firmware developer
- **Preconditions**: A subsystem class exists (e.g., `MotionController`, `Odometry`) and implements the `Commandable` interface.
- **Main Flow**:
  1. Developer calls `getCommands(buf, max)` on the subsystem, or `Robot::buildCommandTable()` aggregates it automatically.
  2. Each entry in the returned `CommandDescriptor[]` array declares a command prefix, a parse function, a handler function, and a handler context pointer.
  3. `CommandProcessor` stores the aggregated table.
  4. Incoming commands are dispatched via longest-prefix linear scan over the table.
- **Postconditions**: Subsystem commands are dispatched through the table without any modification to `CommandProcessor.cpp`.
- **Acceptance Criteria**:
  - [ ] `Commandable` interface is defined in `source/types/CommandTypes.h`.
  - [ ] Each subsystem class (`MotionController`, `Odometry`, `PortController`, `ServoController`, `DebugCommandable`) implements `getCommands()`.
  - [ ] `Robot::buildCommandTable()` aggregates all descriptors into a single static array.

---

## SUC-002: CommandProcessor Dispatches via Table

- **Actor**: Robot firmware (command receive loop)
- **Preconditions**: `CommandProcessor` has been constructed with a `CommandDescriptor[]` table and count.
- **Main Flow**:
  1. A command line arrives on the radio or serial channel.
  2. `CommandProcessor::process()` tokenizes the line.
  3. The dispatcher performs a longest-prefix linear scan over the `CommandDescriptor` table.
  4. The best-matching descriptor is selected (e.g., "DBG LOOP RESET" beats "DBG LOOP" beats "DBG").
  5. If the descriptor has a `parseFn`, it is called with the remaining tokens; on parse failure, `errFmt` is used to reply `ERR <errFmt>`.
  6. On parse success (or no `parseFn`), `handlerFn` is called with the parsed args, reply channel, and `handlerCtx`.
  7. For descriptors with `ForceReply::SERIAL`, the dispatcher substitutes `_serialFn`/`_serialCtx` before calling the handler.
- **Postconditions**: The correct handler is called; the wire response is identical to what the old switch statement produced.
- **Acceptance Criteria**:
  - [ ] `CommandProcessor` accepts a `(const CommandDescriptor*, int count)` constructor.
  - [ ] Longest-prefix matching selects the correct handler for single-word and multi-word prefixes.
  - [ ] `ForceReply::SERIAL` substitution routes DBG/I2CW/I2CR replies to serial regardless of originating channel.
  - [ ] Unknown verb returns `ERR unknown`; parse failure returns `ERR <errFmt>`.

---

## SUC-003: New Command Added Without Touching CommandProcessor

- **Actor**: Firmware developer adding a new subsystem command
- **Preconditions**: Dispatch table is live; `CommandProcessor` uses the new constructor.
- **Main Flow**:
  1. Developer adds a new `CommandDescriptor` entry in the subsystem's `getCommands()` implementation.
  2. `Robot::buildCommandTable()` picks it up automatically.
  3. Developer builds firmware with `python3 build.py`; no changes to `CommandProcessor.cpp` are required.
- **Postconditions**: New command is live on the wire. `CommandProcessor.cpp` is not touched.
- **Acceptance Criteria**:
  - [ ] Adding a command to a subsystem's `getCommands()` is sufficient to make it dispatchable.
  - [ ] `CommandProcessor.cpp` switch statement is entirely removed in the final cutover.
  - [ ] `CommandProcessor.cpp` is under 200 lines after cutover.

---

## SUC-004: Staged Migration Keeps All Commands Live

- **Actor**: Robot firmware (during incremental migration)
- **Preconditions**: Some commands have been migrated to the table; others remain in the old switch.
- **Main Flow**:
  1. `CommandProcessor::process()` checks whether `_cmds == nullptr`.
  2. If `_cmds == nullptr` (old constructor), the existing switch statement runs (all commands work as before).
  3. If `_cmds != nullptr` (new constructor), the table dispatcher runs; commands not yet in the table fall through to an `ERR unknown` response.
- **Postconditions**: At each intermediate migration step, the firmware builds and runs correctly. Commands not yet migrated are handled by the old path.
- **Acceptance Criteria**:
  - [ ] `_cmds == nullptr` guard in `process()` routes to the old switch.
  - [ ] Each migration step compiles and all previously-working commands continue to work.
  - [ ] After the final cutover ticket (T011), `_cmds == nullptr` path is removed.

---

## SUC-005: Config GET/SET Operates via ConfigRegistry

- **Actor**: Robot operator (via `uv run rogo`)
- **Preconditions**: `ConfigRegistry.h/.cpp` extracted; `CfgCtx` wired in `buildCommandTable()`.
- **Main Flow**:
  1. Operator sends `GET vel.kP` or `SET vel.kP 2.5`.
  2. The table dispatcher selects the GET or SET descriptor.
  3. `handleGet` / `handleSet` in `ConfigRegistry.cpp` operate on the `RobotConfig` via offset table.
  4. Response is `OK GET vel.kP 2.500` or `OK SET vel.kP 2.500`.
- **Postconditions**: Config round-trip works correctly; `kRegistry[]` lives in `ConfigRegistry.cpp`, not in `CommandProcessor.cpp`.
- **Acceptance Criteria**:
  - [ ] `source/robot/ConfigRegistry.h/.cpp` contains `kRegistry[]`, `handleGet`, `handleSet`.
  - [ ] GET and SET round-trip a parameter (e.g., `vel.kP`) correctly over the wire.
  - [ ] No config registry code remains in `CommandProcessor.cpp` after extraction.

---

## SUC-006: Debug Commands Always Reply to Serial

- **Actor**: Developer or diagnostic tool
- **Preconditions**: `DebugCommandable` is wired in `main.cpp`; `setSerialReply(fn, ctx)` called on `CommandProcessor`.
- **Main Flow**:
  1. A DBG, I2CW, or I2CR command arrives on any channel (radio or serial).
  2. The dispatcher detects `ForceReply::SERIAL` on the descriptor.
  3. The reply is sent to serial regardless of which channel the command arrived on.
- **Postconditions**: Debug command output is always visible on the serial monitor.
- **Acceptance Criteria**:
  - [ ] `DebugCommandable` handles all DBG subcommands (LOOP, LOOP RESET, I2C, I2CLOG, IRQGUARD, WEDGE) and I2CW, I2CR.
  - [ ] All `DebugCommandable` descriptors use `ForceReply::SERIAL`.
  - [ ] DBG LOOP sent via radio still produces output on serial.

---

## SUC-007: Motion Commands Preserve EVT Async Completions

- **Actor**: Robot host script (via `uv run rogo`)
- **Preconditions**: `MotionController::getCommands()` is implemented; dispatch table is live.
- **Main Flow**:
  1. Host sends `D 200 200 500 #42`.
  2. Table dispatcher calls the D handler; the handler captures `replyFn`, `replyCtx`, `corrId` into `MotionController::beginDistance()` as before.
  3. When the drive completes, `EVT done D #42` fires on the originating channel.
- **Postconditions**: EVT completions for all motion commands (T, D, G, R, TURN) fire correctly on the correct channel.
- **Acceptance Criteria**:
  - [ ] EVT `done T`, `done D`, `done G`, `done R`, `done TURN` fire on correct channel after table-dispatched commands.
  - [ ] `beginStream`, `beginVelocity`, `beginTimed`, `beginDistance`, `beginGoTo`, `beginArc`, `beginTurn` entry points are unchanged.
  - [ ] `beginStream` (S command) continues to bypass MotionCommand entirely.
