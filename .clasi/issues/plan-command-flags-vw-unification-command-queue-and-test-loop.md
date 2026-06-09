---
status: pending
---

# Plan: Command Flags, VW Unification, Command Queue, and Test Loop

## Context

The robot firmware has a composable dispatch system (sprint 019) where each subsystem registers its own commands. The next step is to annotate commands with behavioral flags so a test loop can execute non-hardware commands and skip hardware-touching ones — enabling command routing and translation to be verified without powering up motors or I2C devices.

The key refactors are:
1. Add an `ACCESS_HARDWARE` flag to `CommandDescriptor` / `makeCmd()`
2. Unify motion commands (S/T/G/R/TURN/D) as VW converters — they compute (v,ω) + stop spec and push a VW command to the front of a command queue; VW is the single hardware-touching leaf
3. Add a parsed-command queue to `CommandProcessor`
4. Add `LoopScheduler::run_test()` — reads serial, dispatches non-hardware commands, reports skips

> **Depends on:** `issue-motion-system-overhaul` must be completed first.
> This issue assumes VW is backed by BVC and accepts stop parameters (established by that sprint).
> The `_VW` raw command introduced by that sprint is also included in the flag table below.

---

## Part 1 — `CmdFlags` in `CommandDescriptor` / `makeCmd()`

**File:** `source/types/CommandTypes.h`

Add a `uint8_t flags` field to `CommandDescriptor`. Define one constant for now:

```cpp
static constexpr uint8_t CMD_NONE            = 0;
static constexpr uint8_t CMD_ACCESS_HARDWARE = 1; // reads or writes any HAL object
```

Update `makeCmd()` — new last param with default 0:
```cpp
inline CommandDescriptor makeCmd(
    const char* prefix, ParseFn parseFn, HandlerFn handlerFn, void* ctx,
    const char* errFmt = "badarg",
    ForceReply forceReply = ForceReply::NONE,
    uint8_t flags = CMD_NONE);
```

All 75 existing callers compile unchanged. The struct grows by 1 byte (24 → 25; padding likely keeps it at 28).

---

## Part 2 — Flag Assignment

Update `makeCmd()` calls in each Commandable's `getCommands()`:

| Commands | ACCESS_HARDWARE | Reason |
|----------|----------------|--------|
| S, T, D, G, R, TURN | **false** | After refactor: only enqueue a VW command |
| VW | **true** | Configures MotionController → eventually writes motors |
| _VW | **true** | Raw seed of BVC current state (ACCESS_HARDWARE like VW) |
| X, STOP | **true** | Immediately writes motor stops (STOP = decelerated stop; distinct from the HALT condition-registration family) |
| OI, OZ, OR, OV, OL, OA | **true** | Write directly to OTOS device |
| OP | **false** | After refactor: reads from cached `state.inputs.otosX/Y/H` |
| P, PA | **true** | GPIO read/write; write path needs HAL |
| GRIP | **true** | Servo write (HAL) |
| I2CW, I2CR | **true** | Raw I2C access |
| DBG WEDGE | **true** | Runs encoder wedge test |
| DBG I2C, DBG I2CLOG | **true** | Touch I2C diagnostic layer |
| DBG LOOP, DBG LOOP RESET, DBG IRQGUARD | **false** | State/config only |
| HELLO, PING, ECHO, ID, VER, HELP, SNAP, ZERO, STREAM, RF, GET, SET, GET VEL | **false** | Cached state / config only |

### OP handler refactor
`handleOP` in `Odometry.cpp` currently calls `otos->getPositionRaw()`. Change it to read from `_odomCtx.odo`'s owning Robot's `state.inputs.otosX/Y/H`. The Odometry context already has access via `OdomCtx`. This removes the only read-only OTOS call from command dispatch.

---

## Part 3 — Parsed Command Queue

### New struct in `source/types/CommandTypes.h`

```cpp
struct ParsedCommand {
    const CommandDescriptor* desc;   // points into CommandProcessor's table
    ArgList  args;
    ReplyFn  replyFn;
    void*    replyCtx;
    char     corrId[8];
};
```

### CommandQueue (new file `source/app/CommandQueue.h`)

Fixed-capacity ring buffer (no heap, no STL):

```cpp
static constexpr int COMMAND_QUEUE_CAPACITY = 16;

class CommandQueue {
public:
    bool push_back(const ParsedCommand& cmd);   // normal enqueue
    bool push_front(const ParsedCommand& cmd);  // head-insert (for VW from S/T/D/G/R/TURN)
    bool pop_front(ParsedCommand& out);
    bool empty() const;
    int  size() const;
private:
    ParsedCommand _buf[COMMAND_QUEUE_CAPACITY];
    int _head = 0, _count = 0;
};
```

### Integration in `CommandProcessor`

Add a `CommandQueue* _queue` pointer to `CommandProcessor`. When not null, `process()` parses and enqueues instead of dispatching immediately. Add a new method:
```cpp
bool dequeueOne(CommandQueue& q);  // dispatch one item from q; return false if empty
```

`LoopScheduler` owns the queue, sets it on the processor at boot.

---

## Part 4 — S/T/D/G/R/TURN → VW Converters

These handlers no longer call any MotionController method. They:
1. Compute (v mm/s, ω rad/s) from their arguments
2. Build a `ParsedCommand` for VW with stop params encoded as key=value args
3. Call `queue.push_front(vwCmd)` (they need the queue; pass via `handlerCtx`)

**VW extended argument encoding** (in ArgList):
- arg[0] = v (float, mm/s)
- arg[1] = ω (float, rad/s)
- Optional key=value pairs: `t=<ms>`, `dist=<mm>`, `x=<mm>`, `y=<mm>`, `h=<rad>`

VW's handler reads stop params and calls the appropriate `beginXxx()` on MotionController.

**Stop type mapping:**
| Source | VW stop type | Key args |
|--------|-------------|----------|
| S | None (stream) | — |
| T | Time | `t=<ms>` |
| D | Distance | `dist=<mm>` |
| G | Position | `x=<mm>`, `y=<mm>`, `h=<rad>` |
| R | Heading | `h=<rad>` (relative) |
| TURN | Heading | `h=<rad>` (absolute) |

**MotionCtx update:** add `CommandQueue* queue` so S/T/D/G/R/TURN handlers can push_front.

---

## Part 5 — `run_test()` in LoopScheduler

New method in `LoopScheduler.h/.cpp`:

```cpp
void run_test();  // never returns; safe mode (no hardware access)
```

Loop structure:
```
Seed queue from run_test() local queue.
while (true) {
    drain serial only (no radio) → cmd.process() → enqueues via _queue

    while (!queue.empty()) {
        dequeue one ParsedCommand
        if (cmd.desc->flags & CMD_ACCESS_HARDWARE) {
            snprintf(msg, ..., "DBG skip %s\n", cmd.desc->prefix);
            serialWrite(msg);
        } else {
            cmd.desc->handlerFn(cmd.args, cmd.corrId, cmd.replyFn, cmd.replyCtx, cmd.desc->handlerCtx);
        }
    }

    uBit.sleep(10);
}
```

**Key behavior:**
- S/T/D/G/R/TURN are NOT skipped (not ACCESS_HARDWARE) — their handlers push VW to the queue. The next inner-loop iteration dequeues the VW and reports it as "DBG skip VW v=X w=Y t=Z".
- This makes the command transformation chain visible in serial output without touching motors.
- Serial-only (no radio); use `cmd.setSerialReply()` and read from `comm.serial()` only.

**Entering test mode:** For now, entered by calling `sched.run_test()` from `main.cpp` directly (swap `run_blocks()` → `run_test()` for a test build). A future `DBG TEST` command could trigger it at runtime.

---

## Files Modified

- `source/types/CommandTypes.h` — add `CMD_NONE`, `CMD_ACCESS_HARDWARE`, `flags` field, `ParsedCommand` struct, update `makeCmd()`
- `source/app/CommandQueue.h` (new) — `CommandQueue` ring buffer
- `source/app/CommandProcessor.h/.cpp` — add `CommandQueue*` member, update `process()` to enqueue when queue set, add `dequeueOne()`
- `source/control/MotionController.h/.cpp` — add `CommandQueue*` to `MotionCtx`; refactor S/T/D/G/R/TURN handlers to push VW; extend VW handler for stop params
- `source/control/Odometry.cpp` — `handleOP` reads cached `state.inputs.otosX/Y/H` instead of OTOS device
- `source/robot/Robot.cpp` — `buildCommandTable()` updated `makeCmd()` calls with flags
- `source/control/PortController.cpp`, `ServoController.cpp`, `DebugCommandable.cpp`, `Odometry.cpp` — `makeCmd()` calls updated with flags
- `source/control/LoopScheduler.h/.cpp` — add `run_test()`; own the `CommandQueue`; wire queue into `CommandProcessor`

## Verification

1. `python3 build.py --clean` — build succeeds with no warnings
2. Swap `run_test()` into `main.cpp` and flash
3. Send `S 100 100` over serial — output should show:
   - No immediate hardware output
   - `DBG skip VW v=100 w=0.0` (or similar) confirming S→VW translation
4. Send `OZ` — output should show `DBG skip OZ` (ACCESS_HARDWARE)
5. Send `GET kp` — output should show `OK get kp=<value>` (cached state, no skip)
6. Send `PING` — output should show `OK ping ms=<time>` (cached state)
