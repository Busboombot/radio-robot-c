---
status: in-progress
sprint: 019
tickets:
- 019-001
---

# CommandProcessor Composable Refactor

## Context

The 1742-line `CommandProcessor::process()` switch statement puts all command logic in one place, making it hard to add commands for new subsystems without touching the central dispatcher. The goal is a registration-based system where each subsystem declares its own commands via a `Commandable` interface, and `CommandProcessor` becomes a thin dispatcher over a table of `CommandDescriptor` entries.

Build constraint confirmed: `-std=c++11 -fno-exceptions -fno-rtti` (set in `libraries/codal-microbit-v2/target-locked.json`). No `std::variant`, `std::function`, or heap allocation.

---

## Architecture

### New types — `source/types/CommandTypes.h` (new file)

```cpp
// Tagged argument (replaces std::variant<int, float, std::string>)
enum class ArgType : uint8_t { INT, FLOAT, STR };
struct Argument {
    ArgType type;
    union { int32_t ival; float fval; };
    char sval[32];  // inline string; covers all protocol tokens
};

static constexpr int MAX_ARGS = 10;
struct ArgList { Argument args[MAX_ARGS]; int count; };

struct ParseError { const char* code; const char* detail; };
struct ParseResult { bool ok; union { ArgList args; ParseError err; }; };

// Parse function: raw tokens → ParseResult
typedef ParseResult (*ParseFn)(const char* const* tokens, int ntokens,
                                const KVPair* kvs, int nkv);

// Handler function: parsed args + reply channel + subsystem context
typedef void (*HandlerFn)(const ArgList& args, const char* corrId,
                          ReplyFn replyFn, void* replyCtx,
                          void* handlerCtx);

enum class ForceReply : uint8_t { NONE, SERIAL };

// 24 bytes per entry; ~40 commands = ~960 bytes table (static, not heap)
struct CommandDescriptor {
    const char* prefix;     // "S", "DBG LOOP", "DBG LOOP RESET", etc.
    ParseFn     parseFn;    // nullptr = pass raw tokens to handler
    HandlerFn   handlerFn;
    void*       handlerCtx; // subsystem context pointer (cast inside handler)
    const char* errFmt;     // error code for parse failures; default "badarg"
    ForceReply  forceReply;
};

// Helper to build a descriptor
CommandDescriptor makeCmd(const char* prefix, ParseFn parseFn,
                          HandlerFn handlerFn, void* ctx,
                          const char* errFmt = "badarg",
                          ForceReply forceReply = ForceReply::NONE);

// Commandable interface — subsystems that own commands
class Commandable {
public:
    // Fill buf[0..max-1]; return count written. Avoids std::vector.
    virtual int getCommands(CommandDescriptor* buf, int max) const = 0;
    virtual ~Commandable() {}
};
```

**Note on ackFmt**: Not stored in `CommandDescriptor`. Each handler formats its own ACK body using local format strings. The framework only uses `errFmt` — when `parseFn` returns `ok=false`, the dispatcher calls `replyErr(errFmt, err.detail, corrId, ...)` before invoking the handler.

### Dispatch (replaces the switch statement)

Prefix matching uses a longest-prefix linear scan. "DBG LOOP RESET" (3 tokens) beats "DBG LOOP" (2 tokens) beats "DBG" (1 token). Each DBG subcommand is a full first-class entry — no sub-router.

```cpp
// In CommandProcessor::process():
const CommandDescriptor* best = nullptr; int bestScore = 0;
for (int i = 0; i < _cmdCount; ++i) {
    int score = prefixMatchLen(_cmds[i].prefix, tokens, ntok);
    if (score > bestScore) { best = &_cmds[i]; bestScore = score; }
}
// Remaining tokens after prefix are passed to parseFn
```

For `ForceReply::SERIAL` commands, the dispatcher substitutes `_serialFn`/`_serialCtx` before calling the handler.

---

## Files

### New files
| File | Purpose |
|------|---------|
| `source/types/CommandTypes.h` | All new types: `Argument`, `ArgList`, `ParseResult`, `CommandDescriptor`, `Commandable`, `makeCmd` |
| `source/app/DebugCommandable.h/.cpp` | DBG LOOP, DBG LOOP RESET, DBG I2C, DBG I2CLOG, DBG IRQGUARD, DBG WEDGE, I2CW, I2CR. Holds `LoopScheduler*`, `I2CBus*`, `Robot*`. All use `ForceReply::SERIAL`. |
| `source/robot/ConfigRegistry.h/.cpp` | `kRegistry[]`, `handleGet`, `handleSet` migrated from `CommandProcessor.cpp`. Context: `{RobotConfig*, MotorController*}`. |

### Modified files
| File | Change |
|------|--------|
| `source/types/Protocol.h` | No change; `ReplyFn`/`KVPair` stay here (or `KVPair` moves to `CommandTypes.h`) |
| `source/app/CommandProcessor.h` | Add new constructor `(const CommandDescriptor*, int)`, `setSerialReply(fn, ctx)`, `_cmds`/`_cmdCount` members; keep old `Robot&` constructor during migration |
| `source/app/CommandProcessor.cpp` | Add `dispatchTable()` path in `process()`; keep old switch behind `if (_cmds == nullptr)` guard; strip cases one group at a time |
| `source/control/DriveController.h/.cpp` | **Rename to `MotionController`** (file + class); inherit `Commandable`; add `MotionCtx` struct `{MotionController*, Robot*}`; implement `getCommands()` for S, T, D, G, R, TURN, VW, X, STOP |
| `source/control/Odometry.h/.cpp` | Inherit `Commandable`; add `OdomCtx` struct `{Odometry*, OtosSensor*}`; implement `getCommands()` for OI, OZ, OR, OP, OV, OL, OA |
| `source/robot/Robot.h/.cpp` | Replace `DriveController driveController` with `MotionController motionController`; add `PortController portController` and `ServoController servoController` value members; `buildCommandTable()` aggregates all Commandables + adds PING, ID, HELLO, VER, ECHO, HELP, SNAP, ZERO, GET, SET, STREAM, RF |
| `source/main.cpp` | After migration: `CommandProcessor cmd(cmds, count)` replacing `CommandProcessor cmd(robot)`. Add `DebugCommandable dbgCmd`. Remove `setScheduler`/`setI2CBus` calls after cutover. |

### New files (controllers)
| File | Purpose |
|------|---------|
| `source/control/MotionController.h/.cpp` | Rename of `DriveController`; owns S/T/D/G/R/TURN/VW/X/STOP commands |
| `source/control/PortController.h/.cpp` | Wraps `PortIO&`; inherits `Commandable`; owns P and PA commands |
| `source/control/ServoController.h/.cpp` | Wraps `Servo&`; inherits `Commandable`; owns GRIP command |

### Classes that inherit Commandable
- `MotionController` (renamed from `DriveController`) — motion commands (S/T/D/G/R/TURN/VW/X/STOP)
- `Odometry` — OTOS commands (OI/OZ/OR/OP/OV/OL/OA)
- `PortController` — I/O port commands (P, PA); wraps `PortIO&`
- `ServoController` — gripper command (GRIP); wraps `Servo&`
- `DebugCommandable` — new class for debug commands

### Context structs (static, not heap)
```cpp
// MotionController.h (inner or companion struct)
struct MotionCtx { MotionController* mc; Robot* robot; };

// Odometry.h
struct OdomCtx { Odometry* odo; OtosSensor* otos; };

// ConfigRegistry.h
struct CfgCtx { RobotConfig* cfg; MotorController* mc; };

// DebugCommandable.h
struct DbgCtx { LoopScheduler* sched; I2CBus* bus; Robot* robot; };

// PortController.h — context is just PortIO* (or PortController*)
// ServoController.h — context is just Servo* (or ServoController*)
```

All are declared as statics in `main.cpp` alongside the objects they reference.

---

## Migration order (staged — old switch kept behind null-check)

Each step compiles and runs independently; old dispatch stays live until step 7.

1. **Infrastructure** — Add `CommandTypes.h`. Move `KVPair` there (or add include). Zero behavior change.
2. **ConfigRegistry extraction** — Move `kRegistry[]` + `handleGet`/`handleSet` to `source/robot/ConfigRegistry.cpp/.h`. `CommandProcessor.cpp` includes them. Tests pass unchanged.
3. **New constructor + dispatch** — Add `CommandProcessor(const CommandDescriptor*, int count)` and `dispatchTable()` path. `process()` routes to old switch when `_cmds == nullptr`. Add `setSerialReply()`.
4. **Rename DriveController → MotionController** — Rename header, source, and class; update all include sites and Robot member name (`driveController` → `motionController`). No behavior change; builds clean.
5. **PortController + ServoController** — Implement both new Commandable controller classes with their HAL references. Add as value members to Robot (after gripper/portio refs in declaration order). Register in `buildCommandTable()`.
6. **DebugCommandable** — Implement all DBG/I2CW/I2CR handlers. Wire in `main.cpp` temporarily; verify DBG commands work on new path.
7. **MotionController commands** — Add `getCommands()`. Migrate S/T/D/G/R/TURN/VW/X/STOP. Highest risk: EVT async completion must still fire correctly (the handler captures `replyFn`/`ctx` into `MotionController::begin*()` as before).
8. **Odometry commands** — Add `getCommands()` to `Odometry`. Migrate OI/OZ/OR/OP/OV/OL/OA.
9. **Robot system commands** — Implement `buildCommandTable()`. Migrate PING/ID/HELLO/VER/ECHO/HELP/SNAP/ZERO/GET/SET/STREAM/RF.
10. **Cutover** — In `main.cpp`, switch to `CommandProcessor cmd(cmds, count)`. Remove old `CommandProcessor(Robot&)` constructor. Delete switch path from `.cpp`.

---

## Memory budget

- `CommandDescriptor`: 24 bytes × ~42 commands ≈ **1008 bytes** (static BSS)
- Context structs: ~12 bytes × 7 ≈ 84 bytes (static BSS; includes MotionCtx, OdomCtx, CfgCtx, DbgCtx, PortCtx, ServoCtx)
- `ArgList` on `process()` stack: 10 × 40 bytes = 400 bytes (stack, not heap)
- Total new RAM: ~1070 bytes. No heap allocation anywhere.

---

## Verification

After each migration step:
```
python3 build.py
```
Then connect robot and test representative commands from the migrated group via `uv run rogo`. After full cutover, verify:
- Motion: S, D, T, G, TURN, VW → EVT done fires on the right channel
- Config: GET, SET (round-trip a parameter like `vel.kP`)  
- OTOS: OZ, OL, OA (scalar read/write)
- Debug: DBG LOOP, DBG I2C reply to serial regardless of command source
- Error paths: unknown command → ERR unknown; bad arg count → ERR badarg
