---
id: '006'
title: 'Motion verbs v2: S, T, D, G, STOP, GRIP de-overload, ZERO umbrella'
status: done
use-cases:
- SUC-006
depends-on:
- '002'
issue: protocol-v2-raw250-hard-break.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 009-006: Motion verbs v2 — S, T, D, G, STOP, GRIP de-overload, ZERO umbrella

## Description

Implement all v2 motion commands. These use space-separated integer mm args
(no sign-prefix packing). The underlying `Robot` action methods are unchanged;
only the parse surface in `CommandProcessor` changes.

Also carry over the OTOS/port commands (`OI`, `OZ`, `OR`, `OP`, `OV`, `OL`,
`OA`, `P`, `PA`) with v2 reply format (`OK`/`ERR` tags, key=value where natural).

**Exact wire formats**:
```
S <l> <r>              → OK drive l=<l> r=<r>             (streaming; #id echoed)
T <l> <r> <ms>         → OK drive l=<l> r=<r> ms=<ms>     then later: EVT done T
D <l> <r> <mm>         → OK drive l=<l> r=<r> mm=<mm>     then later: EVT done D
G <x> <y> <speed>      → OK goto x=<x> y=<y> speed=<speed> then later: EVT done G
STOP                   → OK stop
GRIP <deg>             → OK grip deg=<deg>
GRIP                   → OK grip deg=<current>
ZERO enc               → OK zero enc
ZERO pose              → OK zero pose
ZERO enc pose          → OK zero enc pose

EVT safety_stop        (async, when S watchdog fires)
EVT done T / EVT done D / EVT done G  (async completions — no #id, use captured sink)
ERR range <field>      (speed/deg/dist out of valid range)
ERR badarg             (wrong number of args)
```

**`G` is unambiguously go-to** — the old single-arg gripper ambiguity is gone.
`GRIP` (new verb) handles gripper.

**`ZERO` umbrella**: `zeroEncoders()` and/or `zeroOdometry()` based on the tokens
after `ZERO`. At least one token (`enc` or `pose`) required; `ERR badarg` if none.

**OTOS/port carry-over** (same semantics as before, v2 reply format):
```
OI                     → OK oi (or ERR nodev oi)
OZ                     → OK oz
OR                     → OK or  (reset tracking)
OP                     → OK pos x=<x> y=<y> h=<h>
OV <x> <y> <h>         → OK setpos x=<x> y=<y> h=<h>
OL                     → OK linear scalar=<val>
OL <val>               → OK linear scalar=<val>
OA                     → OK angular scalar=<val>
OA <val>               → OK angular scalar=<val>
P <port>               → OK port p=<port> v=<val>
P <port> <val>         → OK port p=<port> v=<val>
PA <port>              → OK aport p=<port> v=<val>
```

**async EVT routing**: `DriveController` already captures the reply sink when a
drive command starts. In v2 it emits `EVT done T\n`, `EVT done D\n`, `EVT done G\n`,
`EVT safety_stop\n` (replacing `+DONE`, `SAFETY_STOP`). Update the emit strings
in `DriveController.cpp`.

## Acceptance Criteria

- [x] `S 200 150` → `OK drive l=200 r=150`; robot moves.
- [x] `T 200 150 1000` → `OK drive l=200 r=150 ms=1000`; then `EVT done T` after ~1 s.
- [x] `D 200 200 300` → `OK drive l=200 r=200 mm=300`; then `EVT done D`.
- [x] `G 300 0 200` → `OK goto x=300 y=0 speed=200`; then `EVT done G`.
- [x] `STOP` → `OK stop`; motors stop immediately.
- [x] `GRIP 90` → `OK grip deg=90`; `GRIP` → `OK grip deg=90`.
- [x] `ZERO enc` → `OK zero enc`; encoders read 0 on next query.
- [x] `ZERO pose` → `OK zero pose`; odometry reads 0,0,0.
- [x] `ZERO enc pose` → both zeroed.
- [x] `#id` correlation echoed on all synchronous responses.
- [x] `EVT` responses carry no `#id` (async).
- [x] `ERR badarg` on wrong arg count; `ERR range <field>` on out-of-range.
- [x] OTOS/port commands reply with `OK`/`ERR` prefix (no bare `ACK:`).
- [x] `EVT safety_stop` replaces old `SAFETY_STOP` text.
- [x] `EVT done T/D/G` replaces old `T+DONE`/`D+DONE`/`G+DONE` text.
- [x] [BENCH] Commands drive the physical robot; `EVT done` arrives on completion.

## Implementation Plan

**Approach**: Add verb handlers in `CommandProcessor::process()` using `parseTokens()`
from ticket 002. Positional args are `atoi(tokens[1])` etc. (space-separated
integers — no sign-prefix parsing needed). Update `DriveController.cpp` EVT strings.

**Files to modify**:
- `source/app/CommandProcessor.cpp` — add S, T, D, G, STOP, GRIP, ZERO, O*, P* handlers
- `source/control/DriveController.cpp` — update completion emit strings to EVT format

**Arg parsing**: `tokens[1]`, `tokens[2]`, `tokens[3]` are `atoi()`'d. `minSpeed`
clamping and range checks as before, but reported via `ERR range l` / `ERR range r` etc.

**Testing**:
- Serial: each motion command + verify `OK` reply format.
- Serial: `T 200 200 500` → wait → verify `EVT done T` arrives.
- Serial: `S 200 200` → watchdog fires after `sTimeout` ms → `EVT safety_stop`.
- [BENCH] Physical drive for T, D, G.
