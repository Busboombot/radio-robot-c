---
status: pending
sprint: "004"
---

# Ratio PID startup spike fix

## Status

The fix is confirmed working in TypeScript (`radio-robot/src/nezha.ts`). It may or may
not manifest on the C++ port depending on motor/encoder timing, but the root cause is
structural and applies identically. Incorporate into sprint 004's ratio PID implementation
rather than treating it as a separate sprint.

## Symptom

When a T, D, or G command starts, the wheels stutter for the first ~50–150 ms — one
wheel briefly stops or reverses, then both resume. Steady-state ratio tracking is fine;
only startup is jerky.

## Root Cause

At tick 2 (~20 ms after command start), one wheel may still be at rest (static friction)
while the other has moved a few mm. With the current normalisation denominator of
`max(1, expected)`:

```
fasterDelta = 4 mm   (right wheel moved)
slowerDelta = 0 mm   (left wheel still at rest)
expected    = 0 * cmdRatio = 0
normErr     = (0 - 4) / max(1, 0) = -4.0
correction  = 300 * -4.0 = -1200 PWM%
rightWheel  = 30% FF + (-1200%) = clamped to 0  ← right wheel stops
```

The `max(1, expected)` floor only prevents division-by-zero; even small positive
`expected` values produce huge fractional errors when `fasterDelta` is small.

The S command avoids this via `startDrive()` re-seeding — cumulative deltas appear
as if the wheels have already done ~200 mm at the commanded ratio, so `expected` is
large from tick 1. The T/D/G commands use `startDriveClean()` which does not re-seed,
leaving them exposed.

**Do not fix this by re-seeding in `startDriveClean()`** — it breaks shallow-ratio
arcs (e.g. `G+300+50+200` where cmdRatio ≈ 1.136). The seeded ratio dominates early
travel, creating a phantom error the PID over-corrects, causing 3× encoder overshoot.

## Fix

Replace the normalisation denominator floor with the commanded faster speed:

```cpp
// In MotorController::tick() / driveTick():
float denomFloor = fasterIsRight ? fabsf(tgtRMms) : fabsf(tgtLMms);
if (denomFloor < 1.0f) denomFloor = 1.0f;
float normErr = (expected - fasterDelta) / fmaxf(denomFloor, expected);
```

With `denomFloor = 200` (commanded speed 200 mm/s):
```
Tick 2: normErr = (0 - 4) / max(200, 0) = -0.02
correction = 300 * -0.02 = -6 PWM%   ← right wheel slows slightly, doesn't stall
```

Once real travel exceeds ~200 mm, `expected > denomFloor` and the floor disengages —
PID behaves identically to the original formulation from that point on.

## Properties

- **No bias in deltas.** Encoder snapshot, encoder targets, and cumulative deltas are
  unchanged. Only the dimensionless error is re-scaled during the startup window.
- **Speed-aware.** Faster commands → larger floor → wider damping window. Matched to
  the rate at which `expected` normally grows.
- **Self-disengaging.** Once `expected > denomFloor` (roughly 1 s of travel at the
  commanded speed), the floor has no effect.

## C++ Target

Modify the normalisation line in `source/control/MotorController.cpp` inside `tick()`.
The `tgtRMms` / `tgtLMms` values are already available in that scope.

**Reference implementation:** `radio-robot/src/nezha.ts`, function `driveTick()` —
the TypeScript version is confirmed working with this fix as of 2026-05-21.

## Acceptance Criteria

- `G+300+0+200` — no visible wheel stutter in the first 150 ms; robot drives smoothly
  to target and emits `G+DONE`
- `T+200+200+2000` — no stutter on start; final encoder difference ≤10 mm
- `G+300+50+200` (shallow arc, cmdRatio ≈ 1.136) — encoder targets not overshot; robot
  reaches target position within KGD tolerance
- `S+200+200` — behavior unchanged (S command uses startDrive re-seeding, not affected
  by this fix)
