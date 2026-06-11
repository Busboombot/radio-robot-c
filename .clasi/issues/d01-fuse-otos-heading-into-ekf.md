---
status: pending
---

# D1 — Fuse OTOS heading into the EKF (heading is currently open-loop)

## Context

Confirmed in current code. The EKF observes position and velocity but **never
heading**: `EKF::updatePosition()` is x/y only, `EKF::updateVelocity()` is v/ω
only ([source/control/EKF.cpp:194](../../source/control/EKF.cpp#L194),
[:305](../../source/control/EKF.cpp#L305)); there is no `updateHeading()`.
`Robot::otosCorrect()` reads the OTOS heading into `state.inputs.otosH`
([source/robot/Robot.cpp:185](../../source/robot/Robot.cpp#L185)) and passes only
`p.x, p.y` to `correctEKF()` ([Robot.cpp:199](../../source/robot/Robot.cpp#L199)) —
the heading is dropped. Heading is therefore pure encoder integration. Combined
with D2 (slip never applied), heading error accumulates monotonically, and every
G command transforms its target into the world frame through that wrong heading —
the "gets turned around and drives into the boards" failure. The cross-block
covariance P[0..2][3..4] is held at zero, so the ω observation can't correct θ
either; and the position cross-covariance only builds while driving *straight*, so
during in-place turns (when heading error is created) the filter has no way to fix it.

## Fix (improvement-plan P0.3)

1. `EKF.h/.cpp`: add `updateHeading(float theta_meas, float r_theta)` — scalar
   update, H = [0,0,1,0,0], **wrap-safe innovation** `y = wrapPi(theta_meas − _x[2])`,
   χ²(1) gate at 3.84 (with the D3 recovery escape hatch). Follow `updateVelocity`
   as the template for the P row/col update.
2. `Odometry::correctEKF()`: accept `theta_otos_rad`, call `updateHeading` between
   the position and velocity updates; pass `p.h` through from `Robot::otosCorrect()`.
3. New config `ekfROtosTheta` (start ≈ 0.01 rad² ≈ (5.7°)²) in `tovez.json` →
   regenerate `DefaultConfig.cpp`.
4. `EKF::setPose()` must stop zeroing P — set a sane diagonal prior (≈100 mm²,
   100 mm², (5°)², keep velocity variances) so gates aren't strangled after a reset.

## Acceptance

- **Sim (field profile, fusion on + mock slip):** square + figure-eight; fused
  heading tracks mock-OTOS truth within ~2° where today it drifts per-turn.
- **Hardware:** four `TURN 9000` in a row return the robot to its starting
  orientation within a few degrees (today: off by ~90° physically).

## Source
Defect **D1** in the 2026-06-11 sim2real review; fix P0.3. Note: the current sprint
branch is "ekf-velocity-fusion" — it fused v/ω, not heading; this closes that gap.
Relates to existing issue `ekf-velocity-fusion-and-robot-state.md`.
