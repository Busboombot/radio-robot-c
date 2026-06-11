---
status: pending
---

# D8 — GO_TO pursuit law: bound curvature, re-gate, realistic arrival tolerance

## Context

In the PURSUE per-tick hook ([source/control/MotionController.cpp:618-640](../../source/control/MotionController.cpp#L618)),
curvature is `κ = 2·dy/d²`. As the robot passes near/abeam the target (small d,
dy ≠ 0), κ → large and ω = v·κ saturates the wheels into a tight orbit. If a
fused-pose correction lands mid-pursuit (or the target was computed from a stale
pose), the target can end up *behind* the robot — but the bearing gate is only
applied at `beginGoTo` time and in PRE_ROTATE, **never re-checked during PURSUE**.
The POSITION stop uses `arriveTolMm = 5 mm`, which may never be satisfiable on
carpet, leaving the orbit running until the watchdog/boards. This root-causes the
"really terrible… stops and pivots… hunting" field reports.

## Fix (improvement-plan P1.4)

1. Clamp curvature: `|κ| ≤ 2 / max(d_remaining, 2·arriveTolMm)`, or clamp ω to what
   `BodyKinematics::saturate` can express without reversing a wheel — pick one and
   document it.
2. Re-gate: if `|bearing| > 90°` (target behind) for > ~3 consecutive ticks, drop
   back to PRE_ROTATE (safe and supervised after D5) instead of orbiting.
3. Widen `arriveTolMm` for field use (20–25 mm), and make the G POSITION stop radius
   ≥ the worst-case decel distance at commanded speed so the SOFT ramp-down lands
   inside the disc. Values go in `tovez.json` → regenerate DefaultConfig.

## Acceptance

- **Sim (field profile):** targets at 0°, ±90°, 180°, and a 30 mm lateral offset
  all converge; no orbit > 1.5 revolutions in the log.

## Source
Defect **D8** in the 2026-06-11 sim2real review; fix P1.4. Explains the
"stops and pivots / hunting" transcript incident. Depends on D5 (PRE_ROTATE must be
supervised before re-gating back into it).
