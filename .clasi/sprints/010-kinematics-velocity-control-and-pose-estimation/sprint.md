---
id: "010"
title: "Kinematics: Velocity Control and Pose Estimation"
status: roadmap
branch: sprint/010-kinematics-velocity-control-and-pose-estimation
use-cases: []
issues:
  - kinematics-velocity-control-layer.md
  - kinematics-pose-estimation-fusion.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 010: Kinematics — Velocity Control + Pose Estimation

## Goals

Deliver the two outer-loop prerequisites for go-to: the robot **holds a
commanded velocity** and **knows where it is**.

**Velocity-control layer** (Layers 1–2 of the kinematics model):

- **Per-wheel velocity PID** (`VelocityController`): setpoint = wheel
  mm/s, feedback = measured wheel velocity (chip velocity preferred);
  PI + feed-forward, anti-windup, low-speed deadband, PWM% clamp ±100.
  Retire the ratio/`kAdj*` cross-coupling as the inner loop.
- **Body kinematics**: single source for `(v,ω)↔(vL,vR)`
  (`vL = v − ω·b/2`, `vR = v + ω·b/2`), `b` = track width from
  `RobotConfig`.
- **Saturation scaling**: when `max(|vL|,|vR|) > vWheelMax`, scale both
  setpoints by `s = vWheelMax/max(|vL|,|vR|)` to **preserve curvature**
  (same arc, slower); keep a `steerHeadroom` reserve.

**Pose-estimation layer** (state-estimation layer of the model):

- **Midpoint (exact-arc) integration**: `θ_mid = θ + dθ/2`, with Odometry
  owning its own previous-encoder delta state (move `_prevOdoEnc*` out of
  the command processor). Removes forward-Euler heading bias on turns.
- **Single authoritative pose** updated by predict (encoders, fast).
- **Complementary OTOS correction** (slow), small α, with **outlier
  gating** (`otosGate`). Structured as a **predict/correct interface** so
  an EKF can drop in later.

## Issues Addressed

- `kinematics-velocity-control-layer.md` — per-wheel PID + body
  kinematics + saturation scaling.
- `kinematics-pose-estimation-fusion.md` — midpoint odometry + OTOS
  complementary fusion with outlier gating.

## Rationale for Grouping

These are the two outer-loop prerequisites for go-to (Sprint 011) and are
naturally co-scheduled: velocity control gives "holds commanded velocity"
and pose estimation gives "knows where it is." Both add `RobotConfig`
tunables and both depend on the same foundation (single authoritative
pose, scheduler fast/slow cadences). Pairing them keeps the kinematics
config and scheduler wiring in one review.

## Dependency Notes

- **Depends on:** 007 (single authoritative pose on Robot/Odometry,
  `RobotConfig` fusion + velocity knobs, the fast/slow scheduler
  cadences); 008 (the chip-velocity signal the PID consumes — encoder
  fallback exists, but chip velocity is the intended feedback source).
- **Blocks:** 011 — pose control consumes the `(v,ω)` body-twist input
  with saturation and steers off the fused authoritative pose.
- Independent of 009 (different layer); the two may proceed in parallel.

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed. (Populated in detail mode.)
