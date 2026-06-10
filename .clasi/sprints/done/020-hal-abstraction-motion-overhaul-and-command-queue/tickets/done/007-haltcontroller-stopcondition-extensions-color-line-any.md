---
id: '007'
title: HaltController + StopCondition extensions (COLOR, LINE_ANY)
status: done
use-cases:
- SUC-009
- SUC-010
- SUC-011
depends-on:
- 020-006
github-issue: ''
issue: issue-motion-system-overhaul.md
completes_issue: false
---

# HaltController + StopCondition extensions (COLOR, LINE_ANY)

## Description

Create `HaltController` — the new class that owns user-registered named stop conditions
and evaluates them each tick. Extend `StopCondition` with `Kind::COLOR` and
`Kind::LINE_ANY`. Wire HaltController into `LoopScheduler` and `Robot`.

`HaltController` evaluates after the system watchdog check (ticket 020-005) and before
the MotionCommand tick. When a halt fires, it calls `cmd.process("X", ...)` or
`cmd.process("X soft", ...)` and emits `EVT halt id=<n>`.

## Acceptance Criteria

- [x] `source/control/HaltController.h/.cpp` created with `StopEntry` array (capacity 8), auto-incrementing ID, `_timerBaselineMs`, `_distBaselineMm`.
- [x] `HaltController::evaluate(const HardwareState& s, uint32_t now_ms)` returns `HaltAction::NONE`, `HaltAction::HARD`, or `HaltAction::SOFT`.
- [x] `evaluate()` emits `EVT halt id=<n>` via a callback before returning the action; clears all conditions when any fires.
- [x] `HaltController::add(StopCondition, StopStyle)` returns assigned ID; `remove(id)`, `clear()`, `info(id)`, `list()` methods present.
- [x] `StopCondition::Kind::COLOR` added: fires when color sensor reading is within HSV distance of target; uses new `float ay` field for hue distance component.
- [x] `StopCondition::Kind::LINE_ANY` added: fires when any of `line[0..3]` satisfies the condition (short-circuit OR).
- [x] `makeColorStop` and `makeLineAnyStop` factory helpers added to `StopCondition.h`.
- [x] `LoopScheduler` calls `_robot.haltController.evaluate()` after watchdog check each tick; result dispatches `cmd.process("X")` or `cmd.process("X soft")`. Design decision: `HaltController` is owned by `Robot` (not LoopScheduler) so it is reachable from both firmware (LoopScheduler calls `_robot.haltController`) and host sim (sim_api calls `robot.haltController`). LoopScheduler.h is CODAL-only and cannot be included in HOST_BUILD, so placing the member on Robot avoids the dependency.
- [x] `Robot` has `HaltController haltController` member (chosen over LoopScheduler — see above).
- [x] `ZERO T` and `ZERO D` command variants wired: `ZERO T` calls `haltController.setTimerBaseline(now_ms)`; `ZERO D` calls `haltController.setDistBaseline(enc_avg)`.
- [ ] Bench: `ZERO T; VW v=300 w=0; HALT TIME 1500` → `EVT halt id=0` fires at ~1500 ms.
- [ ] Bench: `ZERO D; VW v=300 w=0; HALT DIST 400` → `EVT halt id=0` fires at ~400 mm encoder average.
- [ ] Bench: `HALT CLEAR` while driving → conditions cleared, motor continues.
- [ ] Bench: `HALT DIST 500 SOFT` → robot ramps to zero (soft stop); `EVT halt id=<n>` received.
- [ ] Bench: `HALT LINE ANY GE 200` → fires when any line sensor >= 200.
- [x] `python3 build.py --clean` passes.
- [x] `uv run --with pytest python -m pytest` passes (36/36, including 10 new HALT host tests).

## Implementation Plan

### Approach

1. Extend `StopCondition.h/.cpp`: add `Kind::COLOR`, `Kind::LINE_ANY`, `float ay` field,
   factory helpers, update `evaluate()`.
2. Create `HaltController.h/.cpp`: StopEntry struct, fixed array, ID counter, baselines,
   evaluate loop, add/remove/clear/info/list API.
3. Add `HaltController` to `LoopScheduler` (as a member). Wire `evaluate()` into
   `run_blocks()` tick after watchdog check.
4. Wire `ZERO T` and `ZERO D` to `haltController` in the existing `ZERO` command handler
   in `Robot::buildCommandTable()`.
5. Add HALT command registration in a new section of `MotionController::getCommands()`
   or a dedicated `HaltCommandable` helper struct owned by HaltController.

### Files to Create

- `source/control/HaltController.h`
- `source/control/HaltController.cpp`

### Files to Modify

- `source/control/StopCondition.h` — add `Kind::COLOR`, `Kind::LINE_ANY`, `float ay`; add factory helpers
- `source/control/StopCondition.cpp` — implement COLOR and LINE_ANY `evaluate()` branches
- `source/control/LoopScheduler.h` — add `HaltController haltController` member
- `source/control/LoopScheduler.cpp` — wire `evaluate()` in tick body; inject X or X soft
- `source/robot/Robot.h/.cpp` — expose reference to haltController; wire ZERO T/D; add HALT command handlers

### StopEntry struct

```cpp
enum class StopStyle : uint8_t { HARD, SOFT };
struct StopEntry {
    StopCondition cond;
    uint8_t       id;
    StopStyle     style;
    bool          active;
    char          str[40];  // original command string for HALT INFO
};
```

### HaltController evaluate()

```cpp
HaltAction evaluate(const HardwareState& s, uint32_t now_ms,
                    ReplyFn evtFn, void* evtCtx) {
    for (int i = 0; i < _count; ++i) {
        if (!_entries[i].active) continue;
        // Build MotionBaseline from baselines:
        MotionBaseline base { _timerBaselineMs, _distBaselineMm, ... };
        if (_entries[i].cond.evaluate(s, now_ms, base)) {
            uint8_t firedId = _entries[i].id;
            clearAll();
            char msg[32]; snprintf(msg, sizeof(msg), "halt id=%u", firedId);
            CommandProcessor::replyEvt(buf, sizeof(buf), "halt", msg, evtFn, evtCtx);
            return (_entries[i].style == StopStyle::SOFT)
                   ? HaltAction::SOFT : HaltAction::HARD;
        }
    }
    return HaltAction::NONE;
}
```

### COLOR condition evaluate()

HSV conversion utility: `float rgbToH(r, g, b)` returns [0, 360). Hue distance is
wrap-aware: `min(|h1-h2|, 360 - |h1-h2|)`. Fires when
`sqrt(hDist^2 + (s1-s2)^2 + (v1-v2)^2) <= dist`.

### HALT command wire format

The programmer must add HALT TIME/DIST/POS/COLOR/LINE/CLEAR/INFO/LIST as descriptors.
Use longest-prefix matching (consistent with sprint 019 architecture). All HALT variants
share a handler dispatch table based on the first arg token.

### Testing Plan

1. `python3 build.py --clean` — zero warnings.
2. Flash via `mbdeploy deploy robot --clean`.
3. Bench: `ZERO T; VW v=300 w=0; HALT TIME 1500` — `EVT halt id=0` at ~1500 ms.
4. Bench: `ZERO D; VW v=300 w=0; HALT DIST 400` — `EVT halt id=0` at ~400 mm.
5. Bench: `HALT DIST 500 SOFT` — ramp to zero; EVT received.
6. Bench: `HALT CLEAR` while driving — motor continues, no EVT halt.
7. Bench: `HALT LINE ANY GE 200` — EVT when line sensor crosses (manual line test).
8. `uv run --with pytest python -m pytest` — no regressions.

### Notes

- `StopCondition::float ay` addition grows the struct from 28 to 32 bytes. Check that
  kMaxStopConds = 4 in `MotionCommand` still fits in the stack budget.
- `MotionBaseline` is passed to `StopCondition::evaluate()`. For HaltController, the
  baseline `enc0Mm` is `_distBaselineMm` and `t0Ms` is `_timerBaselineMs`. Construct
  a synthetic MotionBaseline with these fields filled and pose fields zeroed.
- `HALT LIST` reply: use repeated `OK HALT id=<n> str="..."` lines, one per active entry.
- The `ZERO` command currently calls `odometry.zero()`. ZERO T and ZERO D are additive
  suffixes — `ZERO` alone keeps its existing behavior.
