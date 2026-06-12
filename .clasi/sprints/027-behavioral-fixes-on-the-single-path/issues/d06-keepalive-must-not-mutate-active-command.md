---
status: in-progress
sprint: '027'
tickets:
- 027-003
---

# D6 — Keepalive verbs must not overwrite an active command's target

## Context

`handleVW`'s no-stop-params branch does `if hasActiveCommand() → activeCmd().setTarget(v, ω)`.
Any plain `S l r` or `VW v ω` "keepalive" (which `stream_drive()` emits and the
host docstrings *recommend* as a keepalive) arriving while a TURN or G MotionCommand
is active **overwrites that command's (v, ω) target**:

- **TURN:** ω is stomped (e.g. to 0) → the HEADING stop never fires → the
  2×nominal+2 s TIME net fires → firmware emits `EVT done TURN` **as if it
  succeeded**, at the wrong heading. The host then issues the next G from a false
  heading. Silent navigation corruption.
- **G PURSUE:** stomped for one tick (pursuit hook re-sets next tick) — a
  recoverable jolt.

## Fix (improvement-plan P1.1)

1. Track a `MotionCommand::Origin` enum set at begin time (VW / TURN / G / T / D /
   R / RT). In `handleVW`'s no-stop-params branch, only treat it as a
   keepalive-with-retarget when the active command **is a plain VW session**. For
   any other origin: reset the watchdog, reply `OK vw busy=<origin>`, and do NOT
   `setTarget`.
2. Host story becomes: `+` for everything; `VW` re-send only retargets VW sessions.
   Update `protocol.py` docstrings (`vw()`, `drive()`) that currently recommend the
   destructive pattern.

## Acceptance

- **Sim (queue wired — see sim-runs-real-dispatch issue):** start TURN, inject
  `S 0 0` mid-turn → TURN completes at the **commanded** heading, not the
  stomped one.

## Source
Defect **D6** in the 2026-06-11 sim2real review (+ scenario 4.4); fix P1.1.
Benefits from the sim running the real dispatch path to be testable.
