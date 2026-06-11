---
status: done
sprint: '024'
tickets:
- 024-001
---

# D5 — Bound the GO_TO PRE_ROTATE phase (stop the wild spin)

## Context

This is the primary cause of the "robot goes wild and spins until I power it off"
field failure. Confirmed in current code.

`MotionController::beginGoTo()` ([source/control/MotionController.cpp:387-389](../../source/control/MotionController.cpp#L387)),
for a target beyond `turnInPlaceGate` (35°), seeds the BVC **directly**:
`_bvc.seedCurrent(0, omega); _bvc.setTarget(0, omega)` — `seedCurrent` bypasses the
profiler, so ω jumps straight to ~180°/s. **No MotionCommand is created for this
phase**, so it has no HEADING stop, no TIME stop, no stop conditions at all. Exit
requires the *fused-pose bearing* to fall under the gate in `driveAdvance()`
([MotionController.cpp:694-716](../../source/control/MotionController.cpp#L694)).
If `poseHrad` is wrong/frozen (slip, encoder wedge, OTOS invalid), the bearing
never crosses the gate and the spin is unbounded. TURN/RT got explicit time-bound
nets in a prior review; PRE_ROTATE was missed. G has **no overall TIME net in
either phase** — the only motion verb without one.

## Fix (improvement-plan P0.1.2 + P0.1.3)

1. Replace raw BVC seeding in the PRE_ROTATE branch with a supervised
   `MotionCommand`: `_activeCmd.configure(0, omega, &_bvc)`, then
   `addStop(makeHeadingStop(bearing_delta, gateRad))` and
   `addStop(makeTimeStop(2×nominal + 2000ms))` — the same runaway net TURN/RT have.
2. On completion, transition to PURSUE exactly as the current PRE_ROTATE→PURSUE
   transition does.
3. Drop `seedCurrent(0, omega)`; let the BVC ramp under `yawAccMax` (the instant
   180°/s start is both the "fast spin" signature and a slip generator).
4. PURSUE path: add an overall `makeTimeStop(2 × (distance/speed)·1000 + 4000 ms)`
   so G is bounded end-to-end.

## Acceptance

- **Sim (field profile):** issue `G` to a 135° bearing target with heading frozen
  (mock); command must end via the PRE_ROTATE TIME net and emit `EVT done G`, NOT
  spin forever and NOT via `safety_stop`.
- **Hardware (keepalives flowing / daemon ON, so the watchdog cannot mask the
  result):** `G` to a behind-the-robot target with a frozen/wrong heading → robot
  stops via the PRE_ROTATE TIME stop and emits the timeout `EVT done G`, **not**
  `EVT safety_stop`; robot placed on field + a tour run never produces an unbounded
  spin. (Host-silence → watchdog behavior is d04's test, not this one — with D5 but
  not D4, host silence would trip the 500 ms watchdog before the TIME net and pass
  for the wrong reason.)

## Source
Defect **D5** in `docs/code_review/2026-06-11-Fable-s2p-review/2026-06-11-sim2real-architecture-review.md`;
fix P0.1.2/P0.1.3 in the companion improvement-plan. Independently confirmed in
`docs/code_review/2026-06-11-wild-spin-and-cursing-forensics.md` §1/§3.1.
Sequencing: this must land **before** D4's keepalive-exemption (P0.2.1).
