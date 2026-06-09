---
id: '011'
title: S/T/D/G/R/TURN VW converters + OP cached-state refactor
status: open
use-cases:
  - SUC-012
  - SUC-014
depends-on:
  - 020-010
github-issue: ''
issue: plan-command-flags-vw-unification-command-queue-and-test-loop.md
completes_issue: false
---

# S/T/D/G/R/TURN VW converters + OP cached-state refactor

## Description

Refactor S/T/D/G/R/TURN handlers to become VW converters: instead of calling
`MotionController::begin*()` directly, they compute (v, ω) + stop params, build a
`ParsedCommand` for VW, and call `queue.push_front(vwCmd)`. VW's handler reads the
stop params and calls the appropriate begin method on MotionController.

Also refactor `handleOP` in `Odometry.cpp` to read from `state.inputs.otosX/Y/H`
instead of calling `otos->getPositionRaw()`. This removes the only non-flagged
OTOS device call from command dispatch (resolving Open Question 4 from the
architecture doc: `OdomCtx` must be extended with a `Robot*` or a cached-state pointer).

**Important**: `MotionCtx` currently holds `{MotionController*, Robot*}`. The converter
handlers need access to the `CommandQueue*`. Add `CommandQueue* queue` to `MotionCtx`.

**`OdomCtx` extension**: add `const HardwareState* hwState` pointer to `OdomCtx` so
`handleOP` can read `hwState->otosX/Y/H` without touching the OTOS device.

## Acceptance Criteria

- [ ] `MotionCtx` struct has `CommandQueue* queue` field; wired in `Robot::buildCommandTable()`.
- [ ] `OdomCtx` struct has `const HardwareState* hwState` field; wired in Robot.
- [ ] S handler computes (v, ω) via `BodyKinematics::forward()`, builds VW ParsedCommand with no stop params, calls `queue->push_front()`. Does NOT call `beginStream()` directly.
- [ ] T handler computes (v, ω) via `BodyKinematics::forward()`, builds VW ParsedCommand with `t=<ms>` stop param, calls `queue->push_front()`.
- [ ] D handler computes (v, ω), builds VW ParsedCommand with `dist=<mm>` stop param, calls `push_front`.
- [ ] G handler builds VW ParsedCommand with `x=<mm>`, `y=<mm>`, `h=<rad>` stop params; calls `push_front`.
- [ ] R handler builds VW ParsedCommand encoding (speed, radius as ω = speed/radius); calls `push_front`.
- [ ] TURN handler builds VW ParsedCommand with `h=<rad>` (absolute heading) stop param; calls `push_front`.
- [ ] VW handler extended to read stop params (t, dist, x, y, h) from ArgList and call appropriate `MotionController::begin*()` method.
- [ ] `handleOP` reads `_odomCtx.hwState->otosX`, `otosY`, `otosH` instead of calling OTOS device.
- [ ] All EVT names unchanged: `EVT done T`, `EVT done D`, `EVT done G`, `EVT done TURN`, `EVT done R`.
- [ ] Bench: `T 200 200 2000` drives 2 s; `EVT done T` received — same behavior as before.
- [ ] Bench: `D dist=500` drives 500 mm; `EVT done D` received.
- [ ] Bench: `OP` returns current pose values from cached state (matches TLM pose fields).
- [ ] `python3 build.py --clean` passes.
- [ ] `uv run --with pytest python -m pytest` passes.

## Implementation Plan

### Approach

1. Extend `MotionCtx` with `CommandQueue* queue` (in MotionController.h).
2. Extend `OdomCtx` with `const HardwareState* hwState` (in Odometry.h).
3. Wire both in `Robot::buildCommandTable()` (Robot.cpp).
4. Rewrite each converter handler (S, T, D, G, R, TURN) in MotionController.cpp to
   build a `ParsedCommand` for VW and push_front.
5. Extend VW's handler to branch on stop params present in ArgList (key=value args).
6. Refactor `handleOP` in Odometry.cpp.
7. Verify all EVT names by grepping the existing test suite for EVT strings.

### Files to Modify

- `source/control/MotionController.h` — add `queue` to MotionCtx
- `source/control/MotionController.cpp` — rewrite S/T/D/G/R/TURN handlers; extend VW handler
- `source/control/Odometry.h` — add `hwState` to OdomCtx
- `source/control/Odometry.cpp` — refactor `handleOP`
- `source/robot/Robot.cpp` — wire `queue` and `hwState` into contexts in `buildCommandTable()`

### VW extended arg encoding

VW handler currently accepts two positional args: `v` and `ω`. Extend to also scan
key=value pairs in `ArgList`:
- `t=<ms>` → call `beginTimed(v, omega, ms, ...)`
- `dist=<mm>` → call `beginDistance(v, omega, dist, ...)`
- `x=<mm>; y=<mm>; h=<rad>` → call `beginGoTo(tx, ty, speed, ...)` or `beginTurn(h, ...)`
- No stop params → call `beginVelocity(v, omega, ...)` (current behavior; S mode)

The VW ArgList after parse by the converter handlers will have:
- args[0].fval = v (mm/s)
- args[1].fval = ω (rad/s)
- key=value pairs encoded as subsequent args (requires parse function that reads kv)

Alternative: pass stop params as additional ArgList entries (positional float args
packed after v and ω). Choose the simpler implementation; document the encoding choice.

### OdomCtx extension

```cpp
struct OdomCtx {
    Odometry*           odo;
    IOtosSensor*        otos;       // kept for OI/OZ/OR/OA/OV/OL device commands
    const HardwareState* hwState;   // added for OP cached read
};
```

`handleOP` reads:
```cpp
const OdomCtx* ctx = ...;
float x = ctx->hwState->otosX;
float y = ctx->hwState->otosY;
float h = ctx->hwState->otosH;
// format reply with x, y, h
```

### Testing Plan

1. `python3 build.py --clean` — zero warnings.
2. Flash via `mbdeploy deploy robot --clean`.
3. Bench: `T 200 200 2000` → 2 s drive; `EVT done T` received.
4. Bench: `D dist=500` → ~500 mm; `EVT done D`.
5. Bench: `G x=500 y=0 speed=200` → goto; `EVT done G`.
6. Bench: `TURN 9000` → 90° turn; `EVT done TURN`.
7. Bench: `OP` → reply contains pose values matching recent TLM fields.
8. Bench: `R 200 300` → arc; `EVT done R` on stop.
9. `uv run --with pytest python -m pytest` — no regressions.

### Notes

- EVT names are emitted by `MotionCommand::tick()` at completion, not by the converter
  handlers. The `beginTimed`, `beginDistance`, etc. methods encode the EVT name in the
  MotionCommand EVT prefix. Verify each begin method still emits the correct EVT.
- The `S` converter: since S is stream (no stop), VW's handler path for no-stop-params
  calls `beginVelocity(v, omega, ...)`. This is the Phase B behavior established in
  ticket 020-005. Ensure the converter does not accidentally add stop params.
- `OdomCtx.otos` is kept for the other Odometry commands (OI, OZ, OR, OA, OV, OL)
  that do write the OTOS device — those remain ACCESS_HARDWARE.
- After this ticket, OP is the only Odometry command that is not ACCESS_HARDWARE. This
  is correct per the flag table in ticket 020-009.
