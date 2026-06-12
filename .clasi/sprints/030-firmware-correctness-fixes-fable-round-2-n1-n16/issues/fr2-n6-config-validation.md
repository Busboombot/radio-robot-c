---
status: in-progress
sprint: '030'
tickets:
- 030-006
---

# FR2-N6 (Med) — Config validation gaps: rate/accel/timeout family unchecked

## Context

Source: `docs/code_review/2026-06-12-Fable-correctness-review/findings.md` §N6.

`validateConfig()` (`ConfigRegistry.cpp:239-258`) checks only tw, ctrlPeriod,
vWheelMax/steerHeadroom, rotSlip. Unchecked foot-guns:

- `SET aDecel=-100`: trapezoid `dv_max` goes negative; `approach()` moves *away* from
  target each tick (`BodyVelocityController.cpp:84-85`) — runaway; decel caps compute
  `sqrtf(negative)` → NaN (NaN comparisons silently disable caps).
- `SET aMax=0` / `yawAccMax=0`: BVC can never leave zero; every verb stalls to its
  TIME net (looks dead).
- `SET sTimeout=0`/negative: watchdog `wdDelta > (int32_t)sTimeoutMs` fires every tick
  once armed — X storm.
- `SET vBodyMax=0`, `yawRateMax=0`: all targets clamp to zero.

Asymmetry: `effectiveSlip()` accepts 0 as "unset → 1.0" but `validateConfig` rejects
`rotSlip=0`, so a host can't restore the documented "unset" state.

## Fix

Add `> 0` checks for the rate/accel family (`aMax`, `aDecel`, `vBodyMax`,
`yawRateMax`, `yawAccMax`) and `sTimeoutMs >= floor` to `validateConfig`. Reconcile
the `rotSlip=0` "unset" semantics so the documented unset state is restorable.

## Acceptance

- `SET aDecel=-100`, `SET aMax=0`, `SET sTimeout=0`, `SET vBodyMax=0`,
  `SET yawRateMax=0` each return `ERR badval` and leave live config unchanged
  (sim tests).
- Valid values still apply atomically (no regression in existing config tests).
