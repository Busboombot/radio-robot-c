---
id: '002'
title: "Move MicroBit ownership to main.cpp \u2014 Robot takes peripheral refs"
status: done
use-cases:
- SUC-002
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 002 — Move MicroBit ownership to main.cpp — Robot takes peripheral refs

## Description

`MicroBit uBit` is currently the first member of `Robot`, and `Robot::run()` contains
the main loop. The CODAL singleton should live in `main.cpp` as a file-scope static so
that hardware ownership is explicit and `Robot` is purely an abstraction layer around
peripherals it receives by reference.

This ticket moves `MicroBit` to `main.cpp`, updates `Robot`'s constructor to take
peripheral references, and keeps `Robot::run()` intact (the loop remains in Robot for
now — loop extraction is Ticket 004). The firmware must build and behave identically
after this change.

## Acceptance Criteria

- [x] `source/main.cpp` declares `static MicroBit uBit;`, calls `uBit.init()`, and
  constructs `Robot robot(uBit.i2c, uBit.serial, uBit.radio, uBit.io, uBit.messageBus)`
  (or equivalent peripheral refs Robot needs).
- [x] `Robot.h` has no `MicroBit` member.
- [x] `Robot`'s constructor takes the required CODAL peripheral references and stores
  them for subsystem construction.
- [x] `Announcer` receives `MicroBit&` from `main.cpp` (or the specific `uBit` fields
  it needs — currently `uBit.getName()`, `uBit.getSerial()`).
- [x] Firmware builds via `mbdeploy deploy --build`.
- [ ] **Bench gate**: Deploy to robot.
  - `HELLO` → `DEVICE:Nezha2:<name>:microbit:<serial>` (name and serial correct).
  - `EZ` / `ENC` → `ACK:EZ` then `ENC+0+0`.
  - `S+150+150` → wheels spin, streamed `ENC…` values climb. `X` stops.
  - All sensors respond: `LS`, `CS`, `O` + `OP` if OTOS present.

## Implementation Plan

### Approach

Change `source/main.cpp` from a 6-line stub to an owner of `uBit`. Update `Robot`'s
ctor signature to accept peripheral references. Keep `Robot::run()` calling the same
loop it does now — no loop restructuring yet.

### Files to Modify

| File | Change |
|---|---|
| `source/main.cpp` | Declare `static MicroBit uBit;`; call `uBit.init()`; pass refs to `Robot` ctor |
| `source/robot/Robot.h` | Remove `MicroBit uBit` member; update ctor signature to take CODAL peripheral refs |
| `source/robot/Robot.cpp` | Update init list: remove `uBit()`, pass received refs to subsystems; `uBit.sleep()` in `run()` replaced by a reference to the MicroBit sleep function or a passed sleep ref |
| `source/app/Announcer.h/.cpp` | Confirm ctor receives what it needs (`MicroBit&` or just `uBit.getName/getSerial` equivalents) |

### Notes on uBit.sleep()

`Robot::run()` calls `uBit.sleep(tickMs)`. After the move, `Robot` no longer holds
`uBit`. Options: pass `MicroBit&` to `Robot` ctor stored as a ref for sleep-only use
(simplest); or use a `fiber_sleep()` CODAL free function if available. Choose the
simplest option that compiles.

### Testing Plan

- Build and flash: `mbdeploy deploy --build`.
- Full smoke sequence as in Acceptance Criteria.
- Pay special attention to `HELLO` — `Announcer` calls `uBit.getName()` and
  `uBit.getSerial()`; confirm they return valid values from the new construction path.

### Documentation Updates

None needed this ticket.
