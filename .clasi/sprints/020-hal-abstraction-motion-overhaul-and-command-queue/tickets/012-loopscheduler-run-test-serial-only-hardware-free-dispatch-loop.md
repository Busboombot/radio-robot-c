---
id: '012'
title: LoopScheduler run_test serial-only hardware-free dispatch loop
status: open
use-cases:
  - SUC-012
depends-on:
  - 020-011
github-issue: ''
issue: plan-command-flags-vw-unification-command-queue-and-test-loop.md
completes_issue: true
---

# LoopScheduler run_test() serial-only hardware-free dispatch loop

## Description

Add `LoopScheduler::run_test()` — a serial-only cooperative loop that dispatches
non-hardware commands and reports skips for hardware-touching ones. It is the final
verification that the full command-transformation chain (converter to VW to skip) is
correct without running motors.

Loop structure:
1. Drain serial input only (no radio): `cmd.process(line, serialFn, serialCtx)` enqueues via `_queue`.
2. Inner drain: while `!_queue.empty()`, pop one command.
   - If `desc->flags & CMD_ACCESS_HARDWARE`: snprintf `"DBG skip %s\n"`, write to serial. Discard.
   - Else: dispatch normally (call `handlerFn`).
3. `uBit.sleep(10)` and repeat.

Key behavior: S/T/D/G/R/TURN are NOT flagged so their handlers run and push VW to the
queue. VW IS flagged so the next iteration reports `DBG skip VW`.

Entry: swap `sched.run_blocks()` to `sched.run_test()` in `main.cpp` for a test build.
This requires recompiling; no runtime toggle in this sprint.

## Acceptance Criteria

- [ ] `run_test()` method added to `LoopScheduler.h/.cpp`.
- [ ] Loop reads from serial only; no radio I/O.
- [ ] ACCESS_HARDWARE commands are not dispatched; produce `DBG skip <prefix>\n` on serial.
- [ ] Non-ACCESS_HARDWARE commands are dispatched normally.
- [ ] `S 100 100` over serial produces `DBG skip VW ...` (converter runs, VW is skipped).
- [ ] `GET velKp` produces `OK get velKp=<value>` (not skipped).
- [ ] `OZ` produces `DBG skip OZ` (skipped).
- [ ] `OP` produces `OK op x=<n> y=<n> h=<n>` (not skipped — reads cached state).
- [ ] `PING` produces `OK ping ms=<n>` (not skipped).
- [ ] Firmware compiles with `run_test()` swapped into `main.cpp`; `python3 build.py --clean` passes.
- [ ] Serial behavior verified manually with a test build flashed on the robot.
- [ ] `main.cpp` restored to `run_blocks()` after verification; final build passes.
- [ ] `uv run --with pytest python -m pytest` passes.

## Implementation Plan

### Approach

1. Add `run_test()` declaration to `LoopScheduler.h`.
2. Implement `run_test()` in `LoopScheduler.cpp` — mirror the serial read path from
   `run_blocks()`; add inner queue drain with ACCESS_HARDWARE check.
3. Temporarily swap `run_blocks()` to `run_test()` in `main.cpp`.
4. Build, flash, test via serial.
5. Restore `run_blocks()` in `main.cpp`.

### Files to Modify

- `source/control/LoopScheduler.h` — add `void run_test();` declaration
- `source/control/LoopScheduler.cpp` — implement `run_test()`
- `source/main.cpp` — temporarily swap to `run_test()` for test build (restore after)

### run_test() implementation sketch

```cpp
void LoopScheduler::run_test() {
    // Serial reply helpers (same as run_blocks serial path)
    ReplyFn serialFn  = _serialReplyFn;
    void*   serialCtx = _serialReplyCtx;

    while (true) {
        // 1. Drain serial input
        while (_comm.serial().available()) {
            const char* line = _comm.serial().readLine();
            if (line) _cmd.process(line, serialFn, serialCtx);
        }

        // 2. Drain queue, filtering hardware commands
        ParsedCommand pc;
        while (_queue.pop_front(pc)) {
            if (pc.desc->flags & CMD_ACCESS_HARDWARE) {
                char buf[64];
                snprintf(buf, sizeof(buf), "DBG skip %s\n", pc.desc->prefix);
                serialFn(buf, serialCtx);
            } else {
                pc.desc->handlerFn(pc.args, pc.corrId,
                                   pc.replyFn, pc.replyCtx,
                                   pc.desc->handlerCtx);
            }
        }

        _uBit.sleep(10);
    }
}
```

### Testing Plan

1. Swap `run_test()` into `main.cpp`; `python3 build.py --clean` — zero warnings.
2. Flash via `mbdeploy deploy robot --clean`.
3. Connect serial. Send each command and observe output:
   - `S 100 100` → `DBG skip VW v=100.0 w=0.0` (or similar VW encoding).
   - `OZ` → `DBG skip OZ`.
   - `GET velKp` → `OK get velKp=...`.
   - `PING` → `OK ping ms=...`.
   - `OP` → `OK op x=... y=... h=...`.
4. Restore `run_blocks()` in `main.cpp`; `python3 build.py --clean` — passes.
5. Flash normal build; send `D dist=500` — normal EVT done D behavior.
6. `uv run --with pytest python -m pytest` — no regressions.

### Notes

- The serial read path in `run_test()` mirrors the serial-only portion of `runCommsIn()`
  from `run_blocks()`. Factor out if duplication is excessive.
- In `run_test()`, skip the system watchdog check and HaltController evaluate() —
  no motion is running.
- `uBit.sleep(10)` is a CODAL call; acceptable here since `run_test()` runs on-device.
  The host build's `sim_tick()` in `sim_api.cpp` is the host equivalent.
- After verification, `main.cpp` must be restored to `run_blocks()`. The test build is
  a manual one-off step. Document in the commit message: "Verified run_test() with test
  build; restored run_blocks() for production firmware."
