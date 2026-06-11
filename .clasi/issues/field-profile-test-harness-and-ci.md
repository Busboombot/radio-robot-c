---
status: pending
---

# Field-profile test harness + CI gate (slip, fusion, latency) with incident regressions

## Context

The core reason defects survived to the field is that the sim validates a friendlier
system than reality: OTOS/EKF fusion off by default, `MockMotor` slip = 0, no motor
deadband, watchdog fed every tick. "Passes in sim" has meant almost nothing — and is
the source of the recurring "you didn't actually test it on our code" frustration. A
fix verified only in the default profile is not verified.

## Goal

1. Define a **field profile** sim fixture: OTOS+EKF fusion ON, `MockMotor` slip set
   to the measured values (straight + turn scrub in the real direction — see D2),
   motor deadband (~35 PWM stiction), and ~15 ms command latency.
2. **Every motion-control test runs twice** — exact profile (ground-truth unit
   correctness) AND field profile. A PR that only passes the exact profile is not
   done; wire this as a CI gate.
3. Encode the four incident scenarios from the sim2real review (§4.1–§4.4) as named
   regression tests: G-into-boards, fast-spin-on-placement, TURN-under-rotate,
   keepalive-kills-TURN.

## Acceptance

- CI runs both profiles; the four scenario tests exist and fail against today's code
  (then pass as the Dx fixes land). The field profile reproduces the real failure
  directions (spin, under-rotation, stomped TURN).

## Source
Improvement-plan **P3.1** + the sim/real-split analysis (§4) in the 2026-06-11 review.
Depends on `sim-runs-real-dispatch-path` (tests must run the real dispatch path) and
D2 (correct slip sign in `MockMotor`).
