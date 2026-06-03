---
status: in-progress
sprint: '011'
tickets:
- 011-001
- 011-002
- 011-003
- 011-004
- 011-005
- 011-006
---

# Kinematics: Pose Control (pursuit-arc go-to + accel/decel + turn-in-place gate)

Implements **Layers 3–4** of [docs/kinematics-model.md](../../docs/kinematics-model.md)
(§1.4, §1.5, §1.6, §2.5). Third of three kinematics issues; siblings
[[kinematics-velocity-control-layer]] and [[kinematics-pose-estimation-fusion]].

## Depends on

- [[kinematics-velocity-control-layer]] — consumes its `(v,ω)` body-twist input
  and saturation scaler.
- [[kinematics-pose-estimation-fusion]] — steers off the fused authoritative pose.
- [[firmware-architecture-refactor]] — this logic lives in `DriveController`.

## Current state (the gap)

The go-to behavior today is the `G` command's `PRE_ROTATE`/`ARC` state machine
inline in `CommandProcessor` (`computeArc`, `turnThresholdMm`, `doneTolMm`). It
computes a one-shot arc and has **no acceleration/deceleration profile** (no
smooth start, no decel-to-stop) and **no continuous re-steering** off a fused
pose. It also lives in the wrong place (the command processor).

## Scope (target)

1. **Pursuit-arc steering** (§1.5): goal in robot frame `(dx,dy)` → curvature
   `κ = 2·dy/(dx²+dy²)`; set `ω = v·κ`; **recompute every pose update** (receding
   horizon) rather than committing to a fixed arc.
2. **Turn-in-place gate** (§1.5 critique): if the bearing to the target exceeds
   `turnInPlaceGate`, rotate in place to roughly face it before pursuing — handles
   beside/behind targets the bare arc law mishandles.
3. **Accel/decel shaping** (§1.6): slew `v` by `aMax·dt`; cap by
   `v_cap = sqrt(2·aDecel·d_remaining)`; `v = min(v_ramped, v_cap, v_user_max)` —
   an online trapezoidal profile (one `sqrt`, no stored plan).
4. **Arrival** within `arriveTolMm`; emit completion event (routed to the
   originating channel per the refactor).
5. Provide the **velocity command `(v, ω)`** primitive (watchdogged) alongside
   **go-to `(x, y)`** (heading-free). Full pose `(x,y,θ)` regulator is explicitly
   **out of scope / later** (§1.4, §1.8.1).

## Config additions (RobotConfig)

`aMax`, `aDecel`, `turnInPlaceGate`, `arriveTolMm` (plus `v_user_max` per command).

## Verification

- Unit-test curvature, the decel cap, and the turn-in-place gate decision.
- Bench: `goTo(x,y)` from ≥3 starts (including a target behind the robot) lands
  within `arriveTolMm`; motion shows smooth accel and a clean decel-to-stop on the
  point; re-steering keeps it on track as the fused pose updates.
