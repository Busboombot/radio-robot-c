---
status: in-progress
sprint: '030'
tickets:
- 030-004
---

# FR2-N4/N5 (Med-High) — Uniform cancel-if-active across all begin*() entry points

## Context

Source: `docs/code_review/2026-06-12-Fable-correctness-review/findings.md` §N4, §N5.

**N4:** `beginStream()` (`MotionController.cpp:148-172`) and `beginRawVelocity()` do
NOT cancel an active MotionCommand (every other begin* does). An `S` issued while
TURN/G/T/D is active (queue path routes `S` through `handleVW` `stream=1` →
`beginStream`): seeds the BVC mid-motion (instant jump — the "fast spin signature"),
leaves `_activeCmd` running so the old command's stop conditions keep evaluating
against the new stream (firing a stale `EVT done` that silently kills the stream),
and the old command never gets `EVT cancelled`. Same defect class as D6, one layer
down — the origin guard protects plain `VW` keepalives but `S` bypasses it. P1.1's
own verify scenario ("start TURN, inject `S 0 0` → TURN must complete") fails here.

**N5:** `beginTimed()` (`:257`) and `beginDistance()` (`:294`) go straight to
`configure()`, silently resetting the previous command's reply sink. Every other
verb emits `EVT cancelled` for the preempted command; a host awaiting `EVT done G`
that issues a `T` never gets any terminal event for the G.

## Fix

In `beginStream()`, `beginRawVelocity()`, `beginTimed()`, and `beginDistance()`,
cancel any active command first — the same three lines the other begin*() entry
points use (emit `EVT cancelled` for the preempted command). Decide explicitly
whether `S` should instead be rejected/busy-replied while a self-terminating command
runs; document the chosen contract.

## Acceptance

- Start TURN, inject `S 0 0` mid-turn on the queue path → TURN completes at the
  commanded heading (P1.1 verify scenario passes).
- Preempting a G with a T emits `EVT cancelled` for the G's corrId before the T runs.
- No mid-motion BVC seed jump when `S` preempts an active command.
