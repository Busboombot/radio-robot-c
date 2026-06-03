---
id: '006'
title: Add OTOS complementary fusion with outlier gating to Odometry
status: done
use-cases:
- SUC-006
depends-on:
- 010-005
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add OTOS complementary fusion with outlier gating to Odometry

## Description

After Ticket 005, `Odometry` integrates encoder deltas with midpoint
integration but has no external correction. Over a long drive, dead-reckoning
drift accumulates. The OTOS optical-flow odometer provides a correcting signal
with less long-term drift, but it must be blended gently (to avoid pose jumps)
and gated against bad samples.

This ticket adds the `correct()` method to `Odometry`, wires `DriveController`
to call it on each slow-cadence OTOS sample, and adds the three fusion config
keys (`alphaPos`, `alphaYaw`, `otosGate`) to `RobotConfig` and the SET/GET
registry.

## Acceptance Criteria

- [x] `Odometry::correct(float x_otos, float y_otos, float θ_otos_rad,
  float alphaPos, float alphaYaw, float otosGate)` method added:
  - Outlier gate: if `sqrtf((x_otos-_x)^2 + (y_otos-_y)^2) > otosGate`,
    reject the sample; increment `_otosRejected` counter; return without
    modifying pose.
  - If accepted: `_x += alphaPos*(x_otos - _x)`, same for `_y`;
    `_headingRad += alphaYaw * wrapPi(θ_otos_rad - _headingRad)`.
- [x] `Odometry` exposes `uint32_t otosRejectedCount() const` for telemetry.
- [x] `DriveController::tick()` calls `_odo.correct(...)` when an OTOS sample
  is fresh (poll `_otos->getPositionRaw()` on the slow cadence, convert LSB →
  mm and LSB → radians using OtosSensor conversion constants, then call
  `correct()`). If OTOS is null (not connected), skip silently.
- [x] New `RobotConfig` fields: `alphaPos` (default 0.15), `alphaYaw` (default
  0.10), `otosGate` (default 50.0 mm). Added to `Config.h` and
  `defaultRobotConfig()`.
- [x] `kRegistry[]` entries added: `alphaPos`, `alphaYaw`, `otosGate`
  (CFG_FLOAT).
- [x] Unit test for `correct()`: verify that a sample within gate is blended
  with correct α fraction; a sample outside gate leaves pose unchanged and
  increments the rejected counter.
- [ ] [BENCH] Drive a square loop (~1 m sides); compare encoder-only vs
  fused pose vs OTOS-only against ground truth. Fused pose tracks OTOS without
  visible jumps when α is at default.
- [ ] [BENCH] Inject one large out-of-range OTOS value (simulate by driving
  the outlier through `SET otosGate=0` temporarily to pass it, then back);
  observe that a single bad sample does not yank the pose when gate is active.

## Implementation Plan

**Approach**: Add `correct()` to `Odometry`; wire into `DriveController::tick()`
on slow cadence. `DriveController` already has access to `OtosSensor*` via
`Robot` (passed through or stored).

**Files to modify**:
- `source/control/Odometry.h` — add `correct()` declaration; add
  `_otosRejected` counter field.
- `source/control/Odometry.cpp` — implement `correct()` with outlier gate
  and complementary blend; implement `wrapPi()` helper (reuse from Ticket 005
  if it was added there, or add here).
- `source/control/DriveController.h` — add `OtosSensor*` member (or use
  `Robot`'s accessor — check existing ownership); add slow-cadence timer field
  if not already present.
- `source/control/DriveController.cpp` — in `tick()`, on slow-cadence ticks,
  read OTOS, convert units, call `_odo.correct()`.
- `source/types/Config.h` — add `alphaPos`, `alphaYaw`, `otosGate` to
  `RobotConfig` and `defaultRobotConfig()`.
- `source/app/CommandProcessor.cpp` — add `kRegistry[]` entries for the three
  new fusion keys.

**OTOS unit conversion** (from `OtosSensor` comments):
- Position: 1 LSB ≈ 0.305 mm; `x_mm = raw_x * 0.305f`.
- Heading: 1 LSB ≈ 0.00549°; `θ_rad = raw_h * 0.00549f * (π/180)`.

**Testing plan**:
- Unit test `correct()` in host-side tests (no hardware needed).
- Bench tests per ACs above.

**Documentation updates**:
- `Odometry.h` cite §2.4 of `docs/kinematics-model.md` for the predict/correct
  structure and note the EKF upgrade path.
- `DriveController.h` note that it calls `Odometry::correct()` on OTOS slow
  cadence.
