---
status: in-progress
sprint: '024'
tickets:
- 024-003
---

# D4 — Watchdog: separate link-loss from motion-supervision; re-arm SAFE

## Context

The system watchdog in `LoopScheduler::run_blocks()` fires `EVT safety_stop` + `X`
after `sTimeoutMs` of inbound-command silence whenever motion is active
([source/control/LoopScheduler.cpp:222-231](../../source/control/LoopScheduler.cpp#L222)).
It now (correctly) covers all motion — but that created a trap:

- The host's blocking helpers (`go_to()`/`turn()` + `wait_for_evt_done()`) sent
  nothing while waiting, so every G/TURN > `sTimeoutMs` was killed mid-motion. The
  operator's workarounds — first `SAFE off`, then (2026-06-10) a background daemon
  streaming `+` every 150 ms for the life of the connection — **demoted the
  watchdog from motion-supervisor to dead-process detector**. With keepalives
  always flowing, the unsupervised PRE_ROTATE spin (D5) is unbounded even with
  `SAFE on`. Today `square_run.py` also sets `sTimeout=60000`, a 60 s net.

Once every motion phase has its own TIME net (D5/D7), the watchdog's job reduces
to link-loss detection.

## Fix (improvement-plan P0.1.1 + P0.2)

1. Exempt MotionCommands that carry at least one TIME stop from the keepalive
   requirement (add `_activeCmd.hasTimeStop()` accessor). S/`VW`/R (open-ended)
   stay keepalive-bound. This makes `go_to()`/`turn()` safe even from hosts that
   forget keepalives, without removing the net for streaming modes.
2. `SAFE off` must not be a permanent foot-gun: auto-re-arm `safetyEnabled = true`
   whenever a new motion command begins (one-shot disable), and emit
   `EVT safety re-armed`. Update `tests/dev/safe_cmd_bench.py` to the new semantics.
3. Make `+` quiet on the wire (suppress the `OK keepalive` reply, or honour a `+q`
   variant): at 6.7 Hz the acks are noise competing with TLM for the 250-byte TX
   buffer, and the host filters them anyway.
4. Remove the `sTimeout=60000` overrides in **host code and test fixtures**
   (e.g. `tests/bench/square_run.py`) — the firmware default (500 ms) is already
   sane, so no firmware change is needed here; this is a host/test cleanup that
   stops a 60 s net from masking watchdog behavior once (1)/(2) land.

## Acceptance

- **Sim:** T/D/G/TURN complete with **zero** keepalives, safety on; S without
  keepalives still safety-stops at `sTimeoutMs`. `SAFE off` then a new motion
  command → `EVT safety re-armed`, safety on for that command.
- **Hardware:** full square run with the keepalive daemon OFF completes without
  spurious safety_stops; killing the host process mid-S still stops the robot.

## Source
Defect **D4** in the 2026-06-11 sim2real review (incl. the 3-era timeline); fixes
P0.1.1 + P0.2. **Depends on D5/D7** (per-command TIME nets must exist first).
Re-frames the 2026-06-10 "watchdog covers all motion" change as a link in the
causal chain, not the end of it.
