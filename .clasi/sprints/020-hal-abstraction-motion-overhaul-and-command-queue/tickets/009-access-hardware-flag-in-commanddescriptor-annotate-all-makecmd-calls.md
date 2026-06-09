---
id: '009'
title: ACCESS_HARDWARE flag in CommandDescriptor + annotate all makeCmd calls
status: open
use-cases:
  - SUC-012
depends-on:
  - 020-008
github-issue: ''
issue: plan-command-flags-vw-unification-command-queue-and-test-loop.md
completes_issue: false
---

# ACCESS_HARDWARE flag in CommandDescriptor + annotate all makeCmd calls

## Description

Add a `uint8_t flags` field to `CommandDescriptor` and add two flag constants
(`CMD_NONE = 0`, `CMD_ACCESS_HARDWARE = 1`) to `CommandTypes.h`. Update `makeCmd()`
with a final defaulted parameter `flags = CMD_NONE`.

Annotate all existing `makeCmd()` calls (~75) with the correct flag. This is the
prerequisite for the CommandQueue (ticket 020-010) and `run_test()` (ticket 020-011)
which consume the flag at dispatch time.

Also add the `ParsedCommand` struct to `CommandTypes.h` (needed by ticket 020-010 for
the queue).

## Acceptance Criteria

- [ ] `CommandDescriptor` has `uint8_t flags` field; struct size verified (expected 28 bytes with alignment).
- [ ] `CMD_NONE = 0` and `CMD_ACCESS_HARDWARE = 1` constants defined in `CommandTypes.h`.
- [ ] `makeCmd()` signature updated with `uint8_t flags = CMD_NONE` as last parameter; all existing callers compile unchanged.
- [ ] `ParsedCommand` struct added to `CommandTypes.h`: `{const CommandDescriptor* desc; ArgList args; ReplyFn replyFn; void* replyCtx; char corrId[8]}`.
- [ ] All `makeCmd()` calls annotated per the flag table:
  - S, T, D, G, R, TURN: `CMD_NONE` (will become VW converters in ticket 020-010)
  - VW, `_VW`, X, STOP: `CMD_ACCESS_HARDWARE`
  - OI, OZ, OR, OV, OL, OA: `CMD_ACCESS_HARDWARE`
  - OP: `CMD_NONE` (reads cached state after OP refactor in ticket 020-011)
  - P, PA: `CMD_ACCESS_HARDWARE`
  - GRIP: `CMD_ACCESS_HARDWARE`
  - I2CW, I2CR: `CMD_ACCESS_HARDWARE`
  - DBG WEDGE, DBG I2C, DBG I2CLOG: `CMD_ACCESS_HARDWARE`
  - DBG LOOP, DBG LOOP RESET, DBG IRQGUARD: `CMD_NONE`
  - HELLO, PING, ECHO, ID, VER, HELP, SNAP, ZERO, STREAM, RF, GET, SET, GET VEL: `CMD_NONE`
  - HALT TIME/DIST/POS/COLOR/LINE/CLEAR/INFO/LIST: `CMD_NONE` (registers conditions, no direct hardware write)
  - `+` keepalive: `CMD_NONE`
- [ ] `python3 build.py --clean` passes.
- [ ] `uv run --with pytest python -m pytest` passes.

## Implementation Plan

### Approach

1. Edit `source/types/CommandTypes.h`: add `uint8_t flags` to `CommandDescriptor` struct,
   add constants, update `makeCmd()` signature, add `ParsedCommand` struct.
2. For each file with `makeCmd()` calls, add the `CMD_ACCESS_HARDWARE` flag to the
   appropriate calls; others get the default `CMD_NONE` (no change needed unless
   explicit 0 is desired for clarity).
3. Build and verify zero new warnings.

### Files to Modify

- `source/types/CommandTypes.h` — add `flags` field, constants, update `makeCmd()`, add `ParsedCommand`
- `source/control/MotionController.cpp` — annotate VW, `_VW`, X, STOP, S, T, D, G, R, TURN, `+`
- `source/control/Odometry.cpp` — annotate OI, OZ, OR, OP, OV, OL, OA
- `source/control/PortController.cpp` — annotate P, PA
- `source/control/ServoController.cpp` — annotate GRIP
- `source/app/DebugCommandable.cpp` — annotate DBG subcommands and I2CW, I2CR
- `source/robot/Robot.cpp` — annotate HELLO, PING, ECHO, ID, VER, HELP, SNAP, ZERO, STREAM, RF, GET, SET, GET VEL
- `source/control/HaltController.cpp` — annotate all HALT subcommands

### Flag assignment reference

| Command | Flag |
|---------|------|
| S, T, D, G, R, TURN | CMD_NONE |
| VW, `_VW` | CMD_ACCESS_HARDWARE |
| X, STOP | CMD_ACCESS_HARDWARE |
| `+` | CMD_NONE |
| OI, OZ, OR, OV, OL, OA | CMD_ACCESS_HARDWARE |
| OP | CMD_NONE |
| P, PA | CMD_ACCESS_HARDWARE |
| GRIP | CMD_ACCESS_HARDWARE |
| I2CW, I2CR | CMD_ACCESS_HARDWARE |
| DBG WEDGE, DBG I2C, DBG I2CLOG | CMD_ACCESS_HARDWARE |
| DBG LOOP, DBG LOOP RESET, DBG IRQGUARD | CMD_NONE |
| HELLO, PING, ECHO, ID, VER, HELP, SNAP, ZERO, STREAM, RF, GET, SET, GET VEL | CMD_NONE |
| HALT TIME/DIST/POS/COLOR/LINE/CLEAR/INFO/LIST | CMD_NONE |

### Testing Plan

1. `python3 build.py --clean` — zero warnings; struct size assertion optional.
2. Grep all `makeCmd()` calls in codebase: each must have either 5 or 6 args (6th is
   the flag). Zero 6-arg calls with wrong type.
3. `uv run --with pytest python -m pytest` — no regressions.

### Notes

- The `CommandDescriptor` struct grows from 24 to 28 bytes (uint8_t flags + 3 bytes
  padding to maintain 4-byte alignment of the next `forceReply` field). Verify with
  `static_assert(sizeof(CommandDescriptor) == 28, "size check");` in a .cpp file.
- `ParsedCommand` contains an `ArgList` (400 bytes on stack × 10 args × 40 bytes each).
  On the queue ring buffer with capacity 16, this is 16 × ~450 bytes ≈ 7 KB of BSS.
  This must fit in nRF52 RAM. If the RAM budget is tight, reduce `MAX_ARGS` to 6 or
  `COMMAND_QUEUE_CAPACITY` to 8. Check BSS after this ticket.
- OP is annotated `CMD_NONE` in anticipation of the ticket 020-011 refactor. The
  existing OP handler still calls OTOS device at this point; the refactor happens in
  ticket 020-011. The flag is correct for the post-refactor state.
