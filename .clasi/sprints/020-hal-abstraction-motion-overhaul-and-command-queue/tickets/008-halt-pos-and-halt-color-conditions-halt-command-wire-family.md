---
id: '008'
title: HALT POS and HALT COLOR wire commands + complete HALT family registration
status: open
use-cases:
  - SUC-009
  - SUC-010
  - SUC-011
depends-on:
  - 020-007
github-issue: ''
issue: issue-motion-system-overhaul.md
completes_issue: true
---

# HALT POS and HALT COLOR wire commands + complete HALT family registration

## Description

Complete the HALT command wire family. Ticket 020-007 creates HaltController and wires
TIME/DIST/LINE conditions. This ticket adds the HALT POS (position) and HALT COLOR
(HSV sensor) conditions, completes the full `HALT` command handler registration in the
command table (TIME/DIST/POS/COLOR/LINE/CLEAR/INFO/LIST), and adds the `ZERO T` /
`ZERO D` wire verbs.

This is separated from 020-007 to keep each ticket under ~400 lines of change. The
programmer can merge with 020-007 if both fit in one session, but dependency order
must be preserved.

## Acceptance Criteria

- [ ] `HALT TIME <ms>` → registers TIME stop in HaltController; replies `OK HALT id=<n>`.
- [ ] `HALT DIST <mm>` → registers DIST stop; replies `OK HALT id=<n>`.
- [ ] `HALT DIST <mm> SOFT` → registers DIST stop with StopStyle::SOFT; replies `OK HALT id=<n>`.
- [ ] `HALT POS <x_mm> <y_mm> <radius_mm>` → registers POSITION stop using `makePositionStop()`; replies `OK HALT id=<n>`.
- [ ] `HALT COLOR <h> <s> <v> <dist>` → registers COLOR stop using `makeColorStop()`; replies `OK HALT id=<n>`.
- [ ] `HALT LINE <ch|ANY> GE|LE <thresh>` → registers SENSOR or LINE_ANY stop; replies `OK HALT id=<n>`.
- [ ] `HALT CLEAR` → clears all conditions; replies `OK HALT cleared=<count>`.
- [ ] `HALT CLEAR <id>` → clears one condition; replies `OK HALT cleared id=<n>`.
- [ ] `HALT INFO <id>` → replies `OK HALT id=<n> str="<original command>"`.
- [ ] `HALT LIST` → replies `OK HALT count=<n>` followed by one line per active entry.
- [ ] `ZERO T` → calls `haltController.setTimerBaseline(now_ms)`; replies `OK zero T`.
- [ ] `ZERO D` → calls `haltController.setDistBaseline(enc_avg)`; replies `OK zero D`.
- [ ] Bare `ZERO` retains existing behavior (calls `odometry.zero()`).
- [ ] Bench: `HALT POS 500 0 50` while driving — EVT fires when robot within 50 mm of (500, 0).
- [ ] Bench: `HALT COLOR 120 0.8 0.6 0.3` — EVT fires when color matches.
- [ ] `python3 build.py --clean` passes.
- [ ] `uv run --with pytest python -m pytest` passes.

## Implementation Plan

### Approach

All HALT subcommands share the prefix `HALT`. Register each as a separate
`CommandDescriptor` with prefix `HALT TIME`, `HALT DIST`, `HALT POS`, etc. (matches
longest-prefix dispatch from sprint 019).

1. Extend `StopCondition` for POSITION (already exists in 018 as `Kind::POSITION`).
2. Verify COLOR and LINE_ANY evaluate() are correct from ticket 020-007.
3. Write parse and handler functions for each HALT subcommand.
4. Register all HALT descriptors in `HaltController::getCommands()` (implement
   `Commandable` on HaltController, or register from Robot).
5. Extend `ZERO` handler to branch on `T` and `D` tokens before calling `odometry.zero()`.

### Files to Modify

- `source/control/HaltController.h/.cpp` — add `Commandable` interface; `getCommands()` returns all HALT descriptors
- `source/robot/Robot.h/.cpp` — register HaltController descriptors in `buildCommandTable()`; extend ZERO handler
- `source/control/StopCondition.h/.cpp` — verify POSITION, COLOR, LINE_ANY all evaluate correctly

### HALT POS parse

```
HALT POS <x_mm> <y_mm> <radius_mm>
args[0] = x_mm (float), args[1] = y_mm (float), args[2] = radius_mm (float)
```
Handler: `haltController.add(makePositionStop(x, y, r), StopStyle::HARD)`.

### HALT COLOR parse

```
HALT COLOR <h> <s> <v> <dist>
args[0] = h (float, 0-360), args[1] = s (float, 0-1), args[2] = v (float, 0-1), args[3] = dist (float)
```
Handler: `haltController.add(makeColorStop(h, s, v, dist), StopStyle::HARD)`.

### HALT LINE parse

```
HALT LINE <ch|ANY> GE|LE <thresh>
tokens[1] = channel ("0"-"3" or "ANY"), tokens[2] = op ("GE" or "LE"), tokens[3] = threshold (int)
```
If channel == "ANY": `makeLineAnyStop(threshold, Cmp::GE/LE)`.
Else: `makeSensorStop(ch_index, threshold, Cmp::GE/LE)`.

### ZERO extension

The existing `ZERO` handler parses no arguments. Extend it to check `tokens[1]`:
- `tokens[1] == "T"` → reset timer baseline; reply `OK zero T`.
- `tokens[1] == "D"` → reset dist baseline; reply `OK zero D`.
- No second token → existing `odometry.zero()` behavior.

This requires changing ZERO from a no-arg command to one that accepts an optional arg.
Update the `parseZero` function (or remove parseFn and handle in the handler directly).

### Testing Plan

1. `python3 build.py --clean` — zero warnings.
2. Flash via `mbdeploy deploy robot --clean`.
3. Bench: `HALT POS 500 0 50; VW v=300 w=0` → EVT halt when robot reaches x~500.
4. Bench: `HALT COLOR 120 0.8 0.6 0.3` → manual color sensor test.
5. Bench: `HALT INFO 0` after registering a condition → returns original string.
6. Bench: `HALT LIST` → lists active conditions.
7. Bench: `ZERO T; ZERO D` → replies `OK zero T`, `OK zero D`.
8. Bench: bare `ZERO` → odometry still zeroed (regression check).
9. `uv run --with pytest python -m pytest` — no regressions.

### Notes

- POSITION stop already exists in StopCondition (Kind::POSITION) from sprint 018 —
  verify that `makePositionStop()` produces the correct field layout for HaltController's
  synthetic MotionBaseline.
- For HALT POS, the HaltController's synthetic MotionBaseline has `pose0X = 0`,
  `pose0Y = 0` (unused — POSITION stop checks absolute pose from HardwareState, not
  delta from baseline). Confirm this with the StopCondition evaluate() implementation.
- `HALT CLEAR <id>` requires the id as an integer token. Parse it as `atoi(tokens[1])`.
  `HALT CLEAR` with no id clears all. These can be two descriptors or one with optional arg.
