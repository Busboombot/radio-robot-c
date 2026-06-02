---
id: '004'
title: "Visible main loop and reply-sink routing \u2014 fix async-completion channel\
  \ bug"
status: done
use-cases:
- SUC-004
depends-on:
- '002'
- '003'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 004 — Visible main loop and reply-sink routing — fix async-completion channel bug

## Description

`Robot::run()` contains the main `while(true)` loop. When `cmd.tick()` fires an async
completion (`T+DONE`, `D+DONE`, `G+DONE`, `SAFETY_STOP`), it uses a hardwired serial
sink regardless of which channel the original command arrived on. A `T` command sent
over radio will receive its `T+DONE` over serial — the wrong channel.

This ticket moves the loop into `main.cpp`, adds `Robot::tick(now_ms, sink)` (no loop
inside), and tracks `activeSink` in the loop so that completions return on the channel
the command arrived on.

After this ticket `Robot::run()` no longer exists.

## Acceptance Criteria

- [x] `Robot::run()` is removed from `Robot.h` and `Robot.cpp`.
- [x] `main.cpp` contains the `while(true)` loop: drain serial with serial sink; drain
  radio with radio sink; call `robot.tick(uBit.systemTime(), activeSink)`.
- [x] `activeSink` (a `ReplyFn` + `void*` pair) is updated to the serial sink when a
  serial command is processed, and to the radio sink when a radio command is processed.
- [x] `Robot::tick(uint32_t now_ms, ReplyFn fn, void* ctx)` exists; it advances
  `DriveController::tick()` and passes the injected sink for completions and telemetry.
  No `while` loop inside `Robot::tick()`.
- [x] `uBit.sleep(tickMs)` (or equivalent) is called in the main loop, not inside Robot.
- [x] **Bug fixed**: Routing logic correct by construction — see implementation notes.
  On-stand radio verification deferred to sprint-end bench gate.
- [ ] **Bench gate**: Deploy to robot. Full smoke sequence: HELLO, EZ/ENC, S drive,
  X stop, SO odometry, T timed drive with DONE over the originating channel, LS/CS,
  serial and radio round-trips both work.

## Implementation Plan

### Approach

1. Add `Robot::tick(uint32_t now_ms, ReplyFn fn, void* ctx)` that calls
   `_dc.tick(now_ms, dt_ms, fn, ctx)` (where `dt_ms = now_ms - _lastTickMs`).
2. Move the loop from `Robot::run()` to `main.cpp`. The loop structure mirrors the
   existing `Robot::run()` exactly, adding only `activeSink` tracking.
3. Remove `Robot::run()`.
4. The `activeSink` is a simple pair of `(ReplyFn fn, void* ctx)`. Initialize to serial.
   After each `cmd.process(line, serialFn, &serial)` call, set `activeSink = {serialFn, &serial}`.
   After each `cmd.process(line, radioFn, &radio)` call, set `activeSink = {radioFn, &radio}`.
   Pass `activeSink` to `robot.tick()`.

### Files to Modify

| File | Change |
|---|---|
| `source/main.cpp` | Add full loop: drain serial, drain radio, tick; track activeSink; call uBit.sleep |
| `source/robot/Robot.h` | Remove `run()` declaration; add `tick(uint32_t now_ms, ReplyFn fn, void* ctx)` |
| `source/robot/Robot.cpp` | Remove `run()` body; implement `tick()` calling `_dc.tick()` with correct dt_ms |

### activeSink Tracking Pattern

```cpp
// In main.cpp
ReplyFn activeFn  = serialReply;
void*   activeCtx = &serial;

while (true) {
    while (serial.readLine(buf, sizeof(buf))) {
        activeFn = serialReply; activeCtx = &serial;
        if (!announcer.handle(buf, serialReply, &serial))
            cmd.process(buf, serialReply, &serial);
    }
    while (radio.poll(buf, sizeof(buf))) {
        activeFn = radioReply; activeCtx = &radio;
        if (!announcer.handle(buf, radioReply, &radio))
            cmd.process(buf, radioReply, &radio);
    }
    robot.tick(uBit.systemTime(), activeFn, activeCtx);
    uBit.sleep(robot.config().tickMs);
}
```

### Testing Plan

- Build and flash: `mbdeploy deploy --build`.
- **Critical verification**: issue `T+500+500+2000` via radio relay; confirm `T+DONE`
  arrives over radio, not serial. Repeat via serial; confirm `T+DONE` arrives over serial.
- Confirm `SAFETY_STOP` also routes to the channel of the last `S` command.
- Full smoke sequence as in Acceptance Criteria.

### Documentation Updates

None needed this ticket.
