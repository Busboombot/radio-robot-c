---
status: in-progress
sprint: '030'
tickets:
- 030-005
---

# FR2-N7 (Med) — Queue-mode command loss is silent: enqueue failures ignored

## Context

Source: `docs/code_review/2026-06-12-Fable-correctness-review/findings.md` §N7.

`CommandProcessor::dispatchTable()` ignores the `_queue->push_back()` return
(`CommandProcessor.cpp:148`), and all seven converters ignore `pushVW()` failure
(`MotionCommandHandlers.cpp:247` etc.). Queue capacity is 4 (`CommandQueue.h:18`),
drain rate ~1 per 10-25 ms tick. A 5-line host burst loses line 5 with no ERR — the
host just times out. Worse for converters: the converter already replied `OK drive…`,
so a dropped VW means the host believes motion started. Bites the sim and the
post-first-safety-stop firmware today; once N2 is fixed it bites all hardware traffic.

## Fix

Reply `ERR busy`/`ERR full` on enqueue failure in `dispatchTable()`. For converters,
suppress the early `OK` until the `pushVW()` succeeds, or emit a follow-up `ERR` if
it fails.

## Acceptance

- A burst that overflows the queue (capacity 4) produces an `ERR full`/`ERR busy` for
  the dropped command(s) — sim test.
- A converter whose `pushVW` fails does not leave the host with a bare `OK` and no
  motion (either no OK, or a follow-up ERR).
