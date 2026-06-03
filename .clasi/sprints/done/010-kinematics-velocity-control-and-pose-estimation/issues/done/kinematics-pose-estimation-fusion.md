---
status: done
sprint: '010'
tickets:
- '005'
- '006'
---

# Kinematics: Pose Estimation (midpoint odometry + OTOS complementary fusion)

Implements the **state-estimation layer** of
[docs/kinematics-model.md](../../docs/kinematics-model.md) (§1.8.5, §2.3, §2.4).
Second of three kinematics issues; siblings
[[kinematics-velocity-control-layer]] and [[kinematics-pose-control-goto]].

## Depends on

- [[firmware-architecture-refactor]] — the single authoritative pose lives on
  `Robot`/`Odometry`; `RobotConfig` holds the fusion knobs; the scheduler runs the
  fast (encoder) and slow (OTOS) cadences.

## Current state (the gap)

`Odometry::update` (`source/control/Odometry.cpp`) integrates encoder deltas with
**forward-Euler using the start-of-tick heading** (not the midpoint), biasing
heading during turns. There is **no fusion**: the OTOS (`source/hal/OtosSensor`,
which provides `getPositionRaw`/`getVelocityRaw` + linear/angular scalars) is read
on demand for queries but never folded into the pose. Encoder deltas are computed
in the command processor and passed in.

## Scope (target)

1. **Midpoint (exact-arc) integration** (§2.4): `θ_mid = θ + dθ/2`;
   `x += dC·cos(θ_mid)`, `y += dC·sin(θ_mid)`, `θ = wrapπ(θ + dθ)`, with
   `dC=(dL+dR)/2`, `dθ=(dR−dL)/b`. Odometry owns its own previous-encoder delta
   state (move `_prevOdoEnc*` out of the command processor).
2. **Single authoritative pose** (x,y,θ) updated by **predict (encoders, fast)**.
3. **Complementary correction** from OTOS (slow): `x ← x + α_pos·(x_otos − x)`
   (same y), `θ ← θ + α_yaw·wrapπ(θ_otos − θ)` (shortest-arc). Small α to avoid
   jumps. **Outlier gating:** reject an OTOS correction that disagrees with the
   prediction by more than `otosGate` (unless forced).
4. Structure it as a **predict/correct interface** so an EKF can replace the
   fixed-α blend later with no change to layers above.

## Config additions (RobotConfig)

`alphaPos`, `alphaYaw`, `otosGate`; reuse `trackwidthMm` (b) and the OTOS scalars.

## Verification

- Unit-test midpoint integration on a known arc (heading-bias gone vs forward-Euler).
- Bench: drive a known square/loop; overlay encoder-only pose, OTOS-only pose, and
  fused pose against ground truth; tune `α` so fused tracks OTOS without visible
  jumps; confirm a single injected bad OTOS sample is rejected by the gate.
