---
status: in-progress
sprint: '020'
tickets:
- 020-005
- 020-006
- 020-007
- 020-008
---

# Issue: Motion system overhaul — BVC-only path, system watchdog, X soft, and first-class STOP conditions

## Problem

The motion system has two parallel, unresolved problems that must be fixed together because they share infrastructure (LoopScheduler, MotionCommand, CommandProcessor):

**Track inconsistency.** Two parallel paths reach `MotorController`: the legacy bypass used by `S` (STREAMING) and `G PRE_ROTATE` writes L/R wheel speeds directly, while the BVC track used by `VW`, `T`, `D`, `R`, `TURN`, `G PURSUE` routes through `BodyVelocityController`. The result is inconsistent ramp/decel behavior and duplicated stop logic.

**Stop condition fragmentation.** Stop conditions are embedded inside `MotionCommand` and bound to the moment a motion starts. The keepalive watchdog is itself duplicated: S-mode has `_lastSMs` in `driveAdvance()`, while VW embeds a TIME stop with keepalive re-arm logic inside `MotionCommand`. There is no way to register stop conditions by ID, query them, or fire them independently of motion start time.

---

## Desired architecture

```
  User / host
      │  VW v ω         ← profiled: BVC ramps toward target
      │  _VW v ω        ← raw: seeds BVC current state, no ramp
      │  +              ← keepalive refresh only (resets watchdog)
      │  X / X soft     ← hard / soft stop
      │  STOP ...       ← register a named stop condition
      │  ZERO T / D     ← reset timer / distance baseline
      ▼
  BodyVelocityController  (trapezoid/S-curve ramp)
      ▼
  BodyKinematics::inverse + saturate
      ▼
  MotorController::setTarget(vL, vR)
      ▼
  Per-wheel VelocityController PIDs

  LoopScheduler (each tick):
      1. system watchdog check  → injects "X" on timeout
      2. haltController.evaluate() → injects "X" or "X soft" on user stop
```

Everything reaches the motor through BVC. No mode bypasses it. Two independent stop-injection points sit in `LoopScheduler`: the system watchdog (replaces all per-command keepalives) and `HaltController` (user-registered stop conditions).

---

## Phase 1 — Motion unification and system watchdog

These changes are the prerequisite foundation. Implement before Phase 2.

### 1. `_VW v omega` — new raw wire command

Calls `bvc.seedCurrent(v, omega)` then `bvc.setTarget(v, omega)`. The profiler thinks it is already at target — no ramp — but wheel PIDs still run. Useful for host-side trajectory planners that manage their own ramp.

### 2. VW refactor — remove embedded keepalive

Remove the embedded TIME stop and keepalive re-arm from VW. VW sets BVC target and runs indefinitely until superseded, X'd, or the system watchdog fires. `MotionCommand::armTime()` and `setDoneEvt()` become dead code and are removed.

**Note:** `MotionCommand` stops used internally by `T`, `D`, `G`, and `TURN` are **not** removed. Only VW's embedded keepalive TIME stop is deleted. The MotionCommand stop array for high-level commands coexists with `HaltController` (see Phase 2).

### 3. System watchdog — single `_watchdogMs` on `LoopScheduler`

Single `_watchdogMs` timestamp on `LoopScheduler`, reset on every inbound radio/serial command in `runCommsIn()`. On timeout: emit `EVT safety_stop`, call `cmd.process("X", ...)`. Replaces both `S`-mode `_lastSMs` in `driveAdvance()` and VW's keepalive TIME stop. There is now exactly one watchdog for all motion modes.

### 4. `+` command — keepalive only

Resets the system watchdog; replies `OK keepalive`. No motion side-effect. Does not touch `HaltController` stop conditions.

### 5. `X soft` variant

`X` = hard stop (existing behavior). `X soft` = sets BVC target to (0, 0), ramps down, emits `EVT done`. Required by Phase 2 (HaltController soft stops).

### 6. `S` command → BVC

`beginStream(vL, vR)` converts (vL, vR) to (v, ω) via `BodyKinematics::forward()`, then calls `bvc.seedCurrent(v, omega)` + `bvc.setTarget(v, omega)`. Removes direct `mc.startDrive()` call. Remove `_lastSMs`.

### 7. `G PRE_ROTATE` → BVC

Replace `mc.startDriveClean(sL, sR)` with `bvc.seedCurrent(0, omega)` + `bvc.setTarget(0, omega)`.

---

## Phase 2 — First-class STOP conditions

Implement after Phase 2 X soft is available.

### HaltController

A new `HaltController` class is the single named owner of all user-registered stop conditions. It sits at the same level as `DriveController` inside `Robot` / `LoopScheduler`, not as a subsystem of `DriveController`.

**Responsibilities:**
- Owns a fixed array of up to 8 `StopEntry` structs: `{StopCondition cond, uint8_t id, StopStyle style, bool active, char str[40]}`
- Owns `_timerBaselineMs` (set by `ZERO T`) and `_distBaselineMm` (set by `ZERO D`)
- Auto-incrementing ID counter (wraps at 255)
- Exposes `add()`, `remove()`, `clear()`, `info()`, `list()` for the STOP command family

**Main-loop interface (LoopScheduler tick):**

```cpp
// Stop injection point 2 (after watchdog check at point 1):
HaltAction action = haltController.evaluate(inputs, now_ms);
if (action == HaltAction::HARD)
    cmd.process("X",      activeFn, activeCtx);
else if (action == HaltAction::SOFT)
    cmd.process("X soft", activeFn, activeCtx);
```

`HaltController` never calls into `MotionController` directly. The `EVT halt id=<n>` is emitted by `HaltController` before the injected command is processed. When a halt condition fires, all registered conditions are cleared.

**Per-condition stop style:**

```
HALT TIME 1000         → hard stop (default)
HALT DIST 500 SOFT     → soft stop (ramp to zero via X soft)
```

### ZERO T and ZERO D — independent baseline resets

Extend the existing `ZERO` command:

```
ZERO T          → resets motion timer baseline to now
ZERO D          → resets distance odometer baseline to current encoder average
```

These are independent of each other and of any motion command. Distinct from the system watchdog — `ZERO T` is a user-controlled elapsed-time origin for stop conditions, not a keepalive.

### HALT command family

> **Note:** The bare `STOP` command (decelerated motor stop) is kept unchanged.
> `HALT` is the new prefix for user-registered stop conditions.

```
HALT TIME <ms>                      → OK HALT id=<n>
HALT DIST <mm>                      → OK HALT id=<n>
HALT POS <x_mm> <y_mm> <radius_mm>  → OK HALT id=<n>
HALT COLOR <h> <s> <v> <dist>       → OK HALT id=<n>
HALT LINE <ch|ANY> GE|LE <thresh>   → OK HALT id=<n>
HALT CLEAR                          → OK HALT cleared=<count>
HALT CLEAR <id>                     → OK HALT cleared id=<n>
HALT INFO <id>                      → OK HALT id=<n> str="<original command>"
HALT LIST                           → OK HALT count=<n> [id=<n> str="..." ...]
```

When a halt condition fires:
```
EVT halt id=<n> [#<corrId>]
```

### Stop condition types

**TIME** — fires when `now_ms - timerBaselineMs >= threshold_ms`. Baseline is set by `ZERO T`. Entirely independent from the system watchdog.

**DIST** — fires when `|(encLMm + encRMm)/2 - distBaselineMm| >= threshold_mm`. Baseline is set by `ZERO D`.

**POS** — fires when Euclidean distance from current pose to (x, y) < radius. Uses `poseX`, `poseY` from `HardwareState`.

**COLOR** — fires when color sensor reading is within `dist` of target HSV. H distance is wrap-aware. Requires adding a 4th float field (`ay`) to `StopCondition` struct.

**LINE** — `STOP LINE <ch> GE|LE <thresh>` fires when `line[ch]` satisfies the condition (channels 0–3). `STOP LINE ANY GE|LE <thresh>` fires when ANY of `line[0..3]` satisfies the condition (short-circuit OR, new evaluation path).

---

## Files affected

- `source/control/BodyVelocityController.h/.cpp` — `seedCurrent(v, omega)` method
- `source/control/MotionController.cpp/.h` — `beginStream` → BVC, `beginVelocity`, `driveAdvance`, new `softStop`; remove `_lastSMs`
- `source/control/MotionCommand.cpp/.h` — remove `armTime`, `setDoneEvt`; remove VW keepalive TIME stop; retain T/D/G/TURN embedded stops
- `source/app/CommandProcessor.cpp` — add `_VW`, add `+`, update `X` for soft variant, remove VW keepalive re-arm; add `HALT` family; extend `ZERO` with `T` and `D` variants
- `source/control/LoopScheduler.cpp/.h` — add `_watchdogMs` (system watchdog, reset in `runCommsIn()`); add watchdog-check task; add `HaltController haltController` member; call `haltController.evaluate()` each tick; inject `"X"` / `"X soft"` at each injection point
- New `source/control/HaltController.h/.cpp` — HaltController class; owns stop registry + baselines
- `source/control/StopCondition.h/.cpp` — add `KIND::COLOR`, `KIND::LINE_ANY`; add `float ay` field; new factory helpers `makeColorStop`, `makeLineAnyStop`; update `evaluate()` for new kinds; add HSV conversion utility
- `source/robot/Robot.h/.cpp` — add `HaltController haltController` member; wire `ZERO T`, `ZERO D` through it

---

## Verification

**Phase 1:**
1. `S` followed by keepalive timeout → `EVT safety_stop`, motors stop; no `_lastSMs` path remains.
2. `VW 300 0; +; +; +` (keepalives arrive) → motor keeps running. Stop keepalives → watchdog fires `EVT safety_stop`.
3. `VW 300 0; X soft` → motor ramps to zero, `EVT done` received.
4. `_VW 300 0` → motor immediately at target speed, no ramp delay.

**Phase 2:**
5. `ZERO T; VW 300 0; HALT TIME 1500` → robot drives ~1.5 s then stops; `EVT halt id=0` received.
6. `ZERO D; VW 300 0; HALT DIST 400` → robot drives ~400 mm then stops; `EVT halt id=0` received.
7. Both halts registered: first to fire wins; `HALT INFO 0` returns original string.
8. `HALT CLEAR` while driving → conditions cleared, motor keeps running.
9. `HALT COLOR 120 0.8 0.6 0.3` → robot stops when color sensor reaches that HSV neighborhood.
10. `HALT LINE ANY GE 200` → robot stops when any line sensor exceeds 200.
11. `HALT DIST 500 SOFT` → robot ramps to zero (soft stop) when distance reached; `EVT halt id=<n>` received.
12. `uv run --with pytest python -m pytest tests/` passes.
