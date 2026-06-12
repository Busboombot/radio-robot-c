---
status: done
sprint: '030'
tickets:
- 030-003
---

# FR2-N3 (High) — TLM emit can call a null or mismatched reply function (crash-grade)

## Context

Source: `docs/code_review/2026-06-12-Fable-correctness-review/findings.md` §N3.

`Robot::telemetryEmit()` calls `fn(tlmBuf, ctx)` with no null check (`Robot.cpp:448`),
invoked whenever `cfg.tlmPeriodMs > 0` (`LoopTickOnce.cpp:130-134`).

1. **Null call:** `_tlmBoundFn` stays nullptr until a STREAM binds the channel, but
   `tlmPeriodMs` is also settable via `SET tlmPeriod=100` (`ConfigRegistry.cpp:81`),
   which does not bind. `SET tlmPeriod=100` with no prior STREAM → null fn-pointer
   call → HardFault. The header comment ("nullptr means TLM is suppressed",
   `Robot.h:148-149`) describes a guard nothing implements.
2. **Fn/ctx mismatch:** D10 binds the *function* from `_tlmBoundCtx`
   (`LoopScheduler.cpp:80-88`) but telemetry is emitted with `ts.activeCtx` — the
   channel of the last command received, not the bound stream channel
   (`LoopTickOnce.cpp:132`). STREAM over serial + a later radio command →
   `serialReplyTlm(msg, &radio)` casts `Radio*` to `SerialPort*` and calls
   `sendReliable` on it. UB; mixed serial+radio is the normal field setup.

## Fix

Pass `robot._tlmBoundCtx` (the bound channel ctx) together with `_tlmBoundFn`, and
guard `fn == nullptr` in `telemetryEmit()` (or refuse `SET tlmPeriod` and funnel
through STREAM).

## Acceptance

- `SET tlmPeriod=100` with no prior STREAM does not crash (no TLM emitted, or ERR).
- STREAM on serial followed by a radio command keeps TLM on the serial channel (no
  fn/ctx mismatch).
