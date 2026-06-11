---
status: pending
---

# D2 — Apply `rotationalSlip` where rotation is measured (it is calibrated but dead)

## Context

Confirmed in current code: `rotationalSlip` (default 0.74) exists only in
[source/types/Config.h:116](../../source/types/Config.h#L116) (definition) and the
config registry/DefaultConfig — it is referenced in **zero firmware logic**.
`Odometry::predict()` uses raw `(dR − dL) / trackwidthMm`. Consequence: a TURN's
HEADING stop fires when *encoders* say the delta is reached, but the chassis
physically rotates ~74% of that (a commanded 90° → ~67°). That same error corrupts
`poseHrad` for every subsequent G world-frame transform. `turnScale` / `distScale`
([Config.h:150-151](../../source/types/Config.h#L150)) are likewise registered but
dead. `beginRotation` (RT) computes its encoder-arc target with no slip term either
— so the "bounded" relative-spin primitive is still inaccurate.

## Fix (improvement-plan P0.5)

1. `Odometry::predict()`: `dTheta = ((dR − dL)/trackwidthMm) * cfg.rotationalSlip;`.
   Clamp the configured value to [0.5, 1.0]; treat 0/unset as 1.0 so old configs
   don't break.
2. `beginRotation()` (RT): divide the target arc by `rotationalSlip` (wheels must
   travel *farther* than the no-slip arc to achieve the angle).
3. Decide and document the relationship to D1: with OTOS heading fused, OTOS is the
   heading truth and encoder dθ is the *prediction* — still slip-correct the
   prediction.
4. Fix `MockMotor`'s turn-slip sign so the sim "field profile" reproduces encoder
   **over**-report (scrub), the real failure direction. Resolve the dead
   `turnScale`/`distScale` (wire or remove).

## Acceptance

- **Hardware (isolates D2 — encoder-arc stop, no fusion):** `RT 9000` lands
  90° ± 3° physical (protractor/OTOS), and the dead-reckoned heading between OTOS
  fixes tracks truth after the slip correction.
- **Sim (field profile):** with mock slip on, predicted heading matches mock-body
  truth after the slip correction.
- Note: `TURN 9000` end-point accuracy is validated in **d01**, not here — TURN
  stops on the fused `poseHrad`, so its accuracy is delivered mainly by OTOS heading
  fusion. D2's isolated effect shows up in RT and in dead-reckoning quality between
  fixes.

## Source
Defect **D2** in the 2026-06-11 sim2real review; fix P0.5. Relates to memory
`turn-overshoot-otos-fusion`. Pairs with D1.
