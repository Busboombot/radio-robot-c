---
status: final
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 011 Use Cases

## SUC-001: Go To a Point in Front of the Robot
Parent: implicit — go-to motion primitive

- **Actor**: Host controller or autonomous behavior
- **Preconditions**: Robot pose is known (odometry initialized). A go-to
  command is issued with a target `(x, y)` forward of the robot and within
  the `turnInPlaceGate` bearing threshold.
- **Main Flow**:
  1. Host sends `G <x> <y> <speed>` (target in front, e.g., `G 300 50 200`).
  2. Firmware acknowledges with `OK goto x=<x> y=<y> speed=<speed>`.
  3. `DriveController::beginGoTo()` initializes the pursuit-arc state machine;
     `_vRamped` starts at zero.
  4. Each tick: odometry predict/correct updates the fused pose; goal is
     expressed in robot frame as `(dx, dy)`.
  5. Curvature `κ = 2·dy/(dx²+dy²)` is recomputed from the fresh robot-frame
     goal. `v` is advanced by the trapezoidal shaper (accel ramp and decel cap).
     `ω = v·κ` is applied via `BodyKinematics::inverse()` → wheel setpoints.
  6. When distance to goal falls within `arriveTolMm`, motors stop and
     `EVT done G [#id]` is emitted on the originating channel.
- **Postconditions**: Robot is within `arriveTolMm` of `(x, y)`. Motion
  showed a smooth accel ramp and a clean decel-to-stop. The completion event
  carries the originating correlation id, if one was supplied.
- **Acceptance Criteria**:
  - [ ] `G 300 0 200` drives straight to 300 mm forward and stops within
    `arriveTolMm` (default 5 mm). [unit + bench]
  - [ ] `G 300 50 200` (target slightly left) curves smoothly to the target;
    robot does not overshoot. [bench]
  - [ ] Accel is visually smooth from rest; decel brings the robot to a stop
    on the target, not past it. [bench]
  - [ ] `EVT done G #id` is emitted on the channel that issued `G … #id`. [bench]

---

## SUC-002: Go To a Point Behind or Beside the Robot
Parent: implicit — turn-in-place gate

- **Actor**: Host controller
- **Preconditions**: Robot pose is initialized. Target bearing to the goal
  exceeds `turnInPlaceGate` (default ≈50°), placing the goal in the
  behind/beside quadrant.
- **Main Flow**:
  1. Host sends `G <x> <y> <speed>` with target behind or beside the robot
     (e.g., `G -300 0 200` for directly behind, `G 0 300 200` for hard left).
  2. DriveController detects bearing exceeds `turnInPlaceGate`; enters
     PRE_ROTATE phase — rotates in place toward the target.
  3. Once bearing drops below `turnInPlaceGate`, transitions to the pursuit-arc
     phase (SUC-001 from that point forward).
  4. Robot arrives within `arriveTolMm` and emits `EVT done G`.
- **Postconditions**: Robot arrived at the goal despite an initially
  unfavorable bearing. The trajectory included a visible in-place rotation
  followed by a pursuit arc.
- **Acceptance Criteria**:
  - [ ] `G -200 0 150` (target directly behind) causes a turn-in-place before
    forward pursuit; robot arrives within `arriveTolMm`. [bench]
  - [ ] `G 0 300 150` (target hard left, 90°) causes turn-in-place; robot
    arrives. [bench]
  - [ ] Bare arc law without the gate does not apply to these targets (the gate
    logic branches at `|bearing| > turnInPlaceGate`). [unit test]

---

## SUC-003: Velocity Primitive — Watchdogged (v, ω) Drive
Parent: implicit — (v, ω) velocity command

- **Actor**: Teleop host or streaming behavior
- **Preconditions**: Firmware is running v2 protocol.
- **Main Flow**:
  1. Host sends `VW <v> <omega> [#id]` repeatedly within `sTimeoutMs` to
     maintain a body-twist velocity: forward speed `v` (mm/s) and yaw rate
     `omega` (mrad/s on the wire, converted to rad/s internally).
  2. Firmware maps `(v, ω)` to wheel setpoints via `BodyKinematics::inverse()`
     + saturation, starts wheels, resets the streaming watchdog.
  3. If no `VW` arrives within `sTimeoutMs`, the watchdog fires: motors stop,
     `EVT safety_stop [#id]` is emitted.
  4. `STOP` clears the mode immediately.
- **Postconditions**: Robot drives at the commanded body twist until preempted
  or watchdogged.
- **Acceptance Criteria**:
  - [ ] `VW 200 0` drives straight forward; `VW 0 500` spins in place. [bench]
  - [ ] `VW 200 300` drives a curved arc. [bench]
  - [ ] Watchdog fires within `sTimeoutMs` ± one tick after the last `VW`. [bench]
  - [ ] `EVT safety_stop #id` carries the originating correlation id. [bench]
  - [ ] `STOP` during `VW` mode halts immediately, no `EVT`. [bench]

---

## SUC-004: Configure Go-To Tuning Parameters
Parent: implicit — SET/GET config surface

- **Actor**: Developer or calibration script
- **Preconditions**: Firmware is running.
- **Main Flow**:
  1. Developer sends `SET aMax=500 aDecel=400 turnGate=45 arriveTol=8`.
  2. Firmware applies the values to `RobotConfig` fields; replies
     `OK set aMax=500 aDecel=400 turnGate=45 arriveTol=8`.
  3. Subsequent `G` commands use the updated parameters.
  4. `GET aMax aDecel turnGate arriveTol` returns the live values.
- **Postconditions**: The four new config keys are writable and readable via
  SET/GET; changes take effect on the next `G` invocation.
- **Acceptance Criteria**:
  - [ ] `SET aMax=300 aDecel=250 turnGate=60 arriveTol=10` succeeds with `OK set`. [unit]
  - [ ] `GET aMax` returns `CFG aMax=300`. [unit]
  - [ ] Unknown key `SET badkey=1` returns `ERR badkey badkey`. [unit]
  - [ ] Tighter `arriveTolMm` produces earlier stop; looser produces later. [bench]
