---
status: done
sprint: '004'
tickets:
- '001'
- '002'
- '003'
- '004'
---

# Firmware ratio-PID drive controller + G (Go-To) command

## Background

The Nezha motors are dumb DC motors with no onboard velocity controller. The firmware
currently runs a PI+FF loop (velocity → PWM) with weak ratio cross-coupling. This issue
replaces that with a proper tick-accumulation PID that maintains the commanded wheel-speed
ratio, and adds a `G` command that drives the robot to a relative XY position using a
pure-pursuit arc.

All control logic — PID, arc math, G state machine — lives in the firmware (`source/`).
The Python `arc_path.py` is host-side planning only.

*Ported from TypeScript radio-robot sprint 005.*

**Note:** The ratio PID algorithm (originally Part 1 of this issue) is now fully specified
in `nezha-ratio-pid-algorithm.md`, which supersedes the sketch here. This issue covers
only the arc math and G command that build on top of it.

---

## Arc math (`source/app/CommandProcessor.cpp`)

New function `computeArc(tx, ty, trackwidthMm)` — robot always at (0,0,0):

```
cross = -ty                              // heading=0 simplification
R     = (tx²+ty²) / (2*ty)
alpha = atan2(ty, tx + R)                // with CCW/CW correction
leftMm  = (R - W/2)*alpha
rightMm = (R + W/2)*alpha
```

New K parameter: `KTW` (trackwidth mm, default 120).

---

## Part 3: G command — Go-To relative XY (`source/app/CommandProcessor.cpp`)

Format: `G+X+Y+Speed` (mm relative to robot, mm/s).

Two-phase state machine running in `tick()`:

1. **Pre-rotate** (if `|atan2(Y,X)| > KGT`): rotate to face target, then proceed.
2. **Arc drive**: call `computeArc`, issue S at commanded speed, track encoder
   targets. When both encoders reach targets (±`KGD` mm): stop, emit `G+DONE`.

During arc following: do **not** stop and re-rotate. Arc is computed once; ratio PID
handles wheel tracking. (Closed-loop recompute is a future enhancement.)

New K parameters: `KGT` (turn threshold degrees, default 50), `KGD` (done tolerance mm,
default 5).

---

## Files

| File | Change |
|------|--------|
| `source/app/CommandProcessor.h/.cpp` | `computeArc` + G command state machine |
| `source/types/Config.h` | Add KTW, KGT, KGD to CalibParams |

## Acceptance Criteria

- `K` response includes `KTW`, `KGT`, `KGD`
- `G+300+0+200` — drives 300 mm straight forward, emits `G+DONE`
- `G+0+150+200` — pre-rotates ~90°, then drives to position, emits `G+DONE`
- `G+200+50+200` — shallow angle (~14°), uses arc directly without pre-rotating
- `G+300+0+200` with ratio PID active — arc tracks correctly without accumulating heading error
