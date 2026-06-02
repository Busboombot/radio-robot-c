---
status: final
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 010 Use Cases

## SUC-001: Chip-Velocity Readout is Correct and Observable

- **Actor**: Developer / firmware system
- **Preconditions**: Robot is connected via serial; motor HAL (sprint 008) is
  running; `Motor::readSpeed` (register 0x47) is implemented with the old
  `lapsToMmScale` conversion path.
- **Main Flow**:
  1. Developer commands a steady motor speed (e.g. S 200 200).
  2. Developer issues a velocity readout command (GET VEL or equivalent v2
     query) to inspect per-wheel chip velocity and source flags.
  3. Firmware reads register 0x47 raw value and converts using the corrected
     formula: `mm/s = (raw / 10) * mmPerDeg * sign`.
  4. Developer compares chip mm/s against encoder-delta mm/s at the same PWM;
     confirms unit factor (x1 vs /10) is correct.
  5. Developer confirms chip velocity tracks encoder velocity within an
     acceptable band across forward and reverse.
- **Postconditions**: `Motor::readSpeed` returns mm/s consistent with
  encoder-delta velocity; `lapsToMmScale` field is deleted from `RobotConfig`;
  source flag (chip vs encoder fallback) is visible in telemetry.
- **Acceptance Criteria**:
  - [ ] `Motor::readSpeed` uses `(raw / 10) * mmPerDeg * sign`; no
    `lapsToMmScale` field or `floor(raw/3.6)*0.01` path remains.
  - [ ] `lapsToMmScale` deleted from `RobotConfig` struct and
    `defaultRobotConfig()`.
  - [ ] GET VEL command returns per-wheel mm/s and a source flag (chip/enc).
  - [ ] [BENCH] At commanded speed (e.g. PWM 50), chip velocity and
    encoder-delta velocity agree within 15%.
  - [ ] [BENCH] The x1-vs-/10 unit factor is bench-confirmed; the correct
    divisor is applied.

---

## SUC-002: Body Kinematics — Single (v,ω) Source

- **Actor**: Firmware control layer
- **Preconditions**: `trackwidthMm` is set in `RobotConfig`; a body-twist
  command `(v, ω)` is requested (e.g. from streaming or go-to).
- **Main Flow**:
  1. DriveController or command layer receives a body-twist `(v, ω)` command.
  2. A single `BodyKinematics` module converts `(v, ω)` to `(vL, vR)` using
     `vL = v - ω*(b/2)`, `vR = v + ω*(b/2)` with `b = trackwidthMm`.
  3. The saturation scaler checks `max(|vL|, |vR|) > vWheelMax`; if so, scales
     both equally: `s = vWheelMax / max(|vL|, |vR|)`, applies `vL*=s, vR*=s`.
  4. The `steerHeadroom` reserve keeps the ceiling below absolute max so the
     inner loop retains steering authority.
  5. Resulting `(vL, vR)` are passed to `VelocityController`.
- **Postconditions**: Wheel speed setpoints are derived from a single canonical
  inverse-kinematics path; curvature is preserved under saturation.
- **Acceptance Criteria**:
  - [ ] A `BodyKinematics` module (or equivalent function group) provides both
    inverse `(v,ω)→(vL,vR)` and forward `(vL,vR)→(v,ω)` maps.
  - [ ] Saturation scaling is applied when `max(|vL|,|vR|) > vWheelMax`; the
    wheel ratio is preserved exactly.
  - [ ] Unit tests: inverse and forward kinematics return correct values for
    known inputs; saturation scales both wheels by the same factor s; curvature
    `κ = (vR - vL) / (b * (vR+vL)/2)` is unchanged after scaling.
  - [ ] `vWheelMax` and `steerHeadroom` are `RobotConfig` fields with
    defaults.

---

## SUC-003: Per-Wheel Velocity PID Replaces Ratio Cross-Coupling

- **Actor**: Firmware control layer / motor system
- **Preconditions**: `BodyKinematics` provides `(vL, vR)` setpoints; chip
  velocity (or encoder-delta fallback) is available as feedback.
- **Main Flow**:
  1. Each tick, `VelocityController` computes a per-wheel PI+FF correction:
     `pwm = kFF*|sp| + kP*err + I`, where `err = sp - measured`.
  2. Anti-windup clamps the integrator when output is saturated.
  3. Low-speed deadband suppresses integrator wind-up below `minWheelMms`.
  4. Output PWM% is clamped to ±100.
  5. Ratio cross-coupling (`RatioPidController`, `kAdj*`) is retired as the
     inner-loop controller; `MotorController.tick()` no longer uses it for
     normal drive.
- **Postconditions**: Each wheel independently tracks its mm/s setpoint;
  under load the slower wheel slows but the ratio (and arc) is preserved by
  the upstream saturation scaler.
- **Acceptance Criteria**:
  - [ ] `VelocityController` class is implemented with PI+FF, anti-windup, and
    deadband; it takes a setpoint and measured velocity and returns PWM%.
  - [ ] `MotorController.tick()` uses `VelocityController` per wheel; the old
    ratio/`kAdj*` path is removed (or bypassed so only velocity PID is active).
  - [ ] New `RobotConfig` keys: `vel.kP`, `vel.kI`, `vel.kFF`, `minWheelMms`.
  - [ ] [BENCH] Command body twist `(v, ω)` at fixed values; measured per-wheel
    velocities track setpoints; robot holds a straight line or arc under load
    without ratio drift.

---

## SUC-004: Velocity Tunables Exposed via SET/GET Registry

- **Actor**: Developer (via host serial)
- **Preconditions**: Sprint 009 SET/GET registry is in place; new
  `RobotConfig` velocity fields exist.
- **Main Flow**:
  1. Developer issues `SET vel.kP=0.3` over serial.
  2. CommandProcessor routes to `kRegistry`, finds the `vel.kP` entry, writes
     the float value into `RobotConfig`.
  3. Developer issues `GET vel.kP` and receives `CFG vel.kP=0.300`.
  4. Robot drives with the updated gain immediately (no restart required).
- **Postconditions**: All new velocity/saturation tunables are live-adjustable
  via SET/GET.
- **Acceptance Criteria**:
  - [ ] `kRegistry[]` entries added for: `vel.kP`, `vel.kI`, `vel.kFF`,
    `minWheelMms`, `vWheelMax`, `steerHeadroom`.
  - [ ] SET and GET round-trip correctly for each new key.
  - [ ] `lapsToMmScale` entry removed from `kRegistry[]` (field deleted).

---

## SUC-005: Midpoint Odometry — Heading Bias Eliminated

- **Actor**: Firmware pose-estimation system
- **Preconditions**: `Odometry` integrates encoder deltas each fast tick;
  previous-encoder state is being held somewhere (currently in
  `DriveController`).
- **Main Flow**:
  1. Each fast tick, the system computes encoder deltas `(dL, dR)`.
  2. `Odometry` owns the previous-encoder positions (`_prevOdoEncL/R`); it
     computes deltas internally.
  3. Integration uses midpoint heading: `θ_mid = θ + dθ/2`;
     `x += dC*cos(θ_mid)`, `y += dC*sin(θ_mid)`, `θ = wrapπ(θ + dθ)`.
  4. On a constant-radius arc, the integrated pose matches the geometric arc
     more closely than forward-Euler.
- **Postconditions**: `Odometry` manages its own encoder delta state; heading
  bias on turns is eliminated.
- **Acceptance Criteria**:
  - [ ] `Odometry::update()` signature accepts raw encoder positions or deltas;
    midpoint integration formula is used.
  - [ ] `_prevOdoEncL/R` state is owned by `Odometry`, not `DriveController`.
  - [ ] `DriveController` passes encoder readings to `Odometry`; it no longer
    stores or computes encoder deltas for odometry itself.
  - [ ] Unit test: drive a known arc (constant dL, dR for N ticks); midpoint
    result is closer to the geometric arc than the old forward-Euler result.

---

## SUC-006: OTOS Complementary Fusion with Outlier Gating

- **Actor**: Firmware pose-estimation system
- **Preconditions**: `Odometry` predict step (SUC-005) is running; `OtosSensor`
  provides position and heading; `alphaPos`, `alphaYaw`, and `otosGate` are
  in `RobotConfig`.
- **Main Flow**:
  1. On each slow cadence tick, firmware reads a fresh OTOS sample
     `(x_otos, y_otos, θ_otos)`.
  2. Outlier gate: if `|x_otos - x_pred| > otosGate` or
     `|y_otos - y_pred| > otosGate`, the sample is rejected; a counter
     increments for telemetry.
  3. If accepted, blended correction is applied:
     `x ← x + αPos*(x_otos - x)`, same for y;
     `θ ← θ + αYaw*wrapπ(θ_otos - θ)`.
  4. Corrected pose is available via `Odometry::getPose()`.
- **Postconditions**: Long-term drift is reduced; a bad OTOS sample cannot yank
  the pose; the predict/correct interface leaves the door open for an EKF.
- **Acceptance Criteria**:
  - [ ] `Odometry` (or a thin `PoseEstimator` wrapper) exposes `predict(dL,
    dR)` and `correct(x_otos, y_otos, θ_otos)` methods.
  - [ ] `correct()` applies outlier gating: samples outside `otosGate` mm of
    the predicted position are rejected.
  - [ ] `correct()` applies complementary blend with `alphaPos` and `alphaYaw`
    from `RobotConfig`.
  - [ ] New `RobotConfig` fields: `alphaPos`, `alphaYaw`, `otosGate`.
  - [ ] [BENCH] Drive a square; compare encoder-only vs fused pose vs OTOS-only
    against ground-truth; tune α so fused tracks OTOS without visible jumps.
  - [ ] [BENCH] Inject a single large OTOS outlier; confirm pose does not jump.
