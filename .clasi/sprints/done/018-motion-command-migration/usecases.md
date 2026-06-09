---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 018 Use Cases

Sprint 018 completes the motion-command migration begun in 016-017. Every sprint
use case is served by the `MotionCommand` + `BodyVelocityController` + `StopCondition`
engine built in 017. The new verbs (R, TURN, sensor-stop modifier) extend that engine
without altering its structure.

---

## SUC-001: Arc drive

**Actor**: Host program (e.g. rogo, calibration script, course-navigation logic)

**Preconditions**:
- Firmware is running with the Sprint 017 MotionCommand core.
- Robot is idle or a previous command has completed/been cancelled.

**Main Flow**:
1. Host sends `R <speed_mms> <radius_mm>` (e.g. `R 300 200` for a left arc).
2. Firmware computes `ω = speed · κ`, where `κ = 1 / radius` (`radius = 0` ⇒ κ = 0, straight).
3. Firmware configures a MotionCommand with target `(speed, ω)` and a soft stop.
4. Robot accelerates under `aMax`/`yawAccMax` limits and follows the arc.
5. Sending `R 0 <r>` or `X` halts: SOFT ramp-down emits `EVT done` (or `X` emits `EVT cancelled`).
6. Host `arc()` wrapper mirrors the `vw()` pattern for fire-and-forget streaming arcs.

**Postconditions**:
- Robot is executing a profiled arc at `(speed, speed/radius)` body twist.
- `EVT done` is emitted after a soft-stop; `EVT cancelled` after hard-stop.

**Acceptance Criteria**:
- [ ] `R 300 0` produces straight-ahead motion (`vL == vR` at steady state).
- [ ] `R 300 200` produces a left arc (positive radius ⇒ CCW/left, `vL < vR`).
- [ ] `R 300 -200` produces a right arc (`vL > vR`).
- [ ] `R 0 200` triggers SOFT ramp-down.
- [ ] `radius = 0` branch does not divide by zero.
- [ ] Sign convention pinned by a Python host test: positive radius ⇒ positive ω ⇒ `vL < vR`.
- [ ] `R` appears in HELP verb list.
- [ ] `arc()` method present in `protocol.py`.

---

## SUC-002: Profiled go-to (G migration)

**Actor**: Host program or autonomous navigation

**Preconditions**:
- OTOS / odometry pose is valid.
- Robot is idle.

**Main Flow**:
1. Host sends `G <x> <y> <speed>` (robot-relative mm).
2. PRE_ROTATE phase (if bearing > gate): raw turn-in-place as before (unchanged).
3. PURSUE phase: each tick a pursuit hook recomputes `(v, ω = v·κ_bearing)` and calls
   `MotionCommand::setTarget`; the decel cap `v_cap = √(2·aDecel·d_remaining)` clamps
   `v` before the hook fires. A `POSITION` stop condition terminates on arrival.
4. On arrival: MotionCommand emits `EVT done G`.
5. `_vRamped` member is removed; the BVC provides all ramping.

**Postconditions**:
- Robot has reached within `arriveTolMm` of the goal, profiler has ramped down.
- `EVT done G` emitted (wire contract preserved).

**Acceptance Criteria**:
- [ ] `test_pursuit_arc_steering.py` continues to pass.
- [ ] `EVT done G` wire format unchanged.
- [ ] `_vRamped` removed from `DriveController.h/.cpp`.
- [ ] Clean build; no stale STREAMING or TIMED branches handle G.

---

## SUC-003: Profiled timed drive (T migration)

**Actor**: Host program or calibration script

**Preconditions**: Robot is idle.

**Main Flow**:
1. Host sends `T <l> <r> <ms>`.
2. Firmware converts `(L, R)` wheel speeds to `(v, ω)` via `BodyKinematics::forward()`.
3. MotionCommand is configured with that twist and a `TIME(ms)` stop condition.
4. Robot accelerates under body limits; after `ms` milliseconds SOFT ramp-down begins.
5. `EVT done T` is emitted after ramp-down completes.

**Postconditions**: Robot has driven for approximately `ms` milliseconds at body-level
velocity `(v, ω)`; profiler has ramped down; `EVT done T` emitted.

**Acceptance Criteria**:
- [ ] `EVT done T` wire format unchanged.
- [ ] `(L, R) → forward() → (v, ω)` conversion correct (no steer bias for equal L=R).
- [ ] T calibration scripts (`calibrate_linear.py`) produce same distance as pre-018.
- [ ] Bespoke `_tEndMs` branch removed from `driveAdvance`.

---

## SUC-004: Profiled distance drive (D migration)

**Actor**: Host program or calibration script

**Preconditions**: Robot is idle; encoder hardware functional.

**Main Flow**:
1. Host sends `D <l> <r> <mm>`.
2. Firmware converts `(L, R)` to `(v, ω)` via `BodyKinematics::forward()`.
3. MotionCommand configured with `DISTANCE(mm)` stop condition (raw encoder sum).
4. Terminal decel cap `v_cap = √(2·aDecel·d_remaining)` applied each tick — robot
   decelerates smoothly as it approaches the target distance.
5. Encoder-reset workaround preserved (call before start).
6. `EVT done D` emitted after SOFT ramp-down.
7. D-timeout heuristic retained as a safety net; re-verified to tolerate ramp-up.

**Postconditions**: Robot has travelled `mm` millimetres; `EVT done D` emitted.

**Acceptance Criteria**:
- [ ] `EVT done D` wire format unchanged.
- [ ] `DISTANCE` stop uses raw (not filtered) encoder sum — same as pre-018 logic.
- [ ] Encoder-reset workaround still in place.
- [ ] D-timeout heuristic still computes 2× nominal + 2 s (proportional, not fixed).
- [ ] Terminal decel cap ensures smooth stop near target.
- [ ] Bespoke `_dEncStartL/R` / `_dTargetMm` branches removed from `driveAdvance`.

---

## SUC-005: Turn-to-heading (TURN verb)

**Actor**: Host program or rotation-calibration script

**Preconditions**: OTOS/odometry heading valid; robot is idle.

**Main Flow**:
1. Host sends `TURN <heading_cdeg>` (heading in centidegrees from current pose zero).
2. Firmware converts to radians; computes `ω` sign from shortest-path direction.
3. MotionCommand configured with `(v=0, ω = ±yawRateMax)` and `HEADING(θ, eps)` stop.
4. Robot rotates; when heading reaches within `eps` of target the command soft-stops.
5. `EVT done TURN` emitted.
6. Host `turn()` wrapper added to `protocol.py`; host `wait_for_evt_done("TURN", ...)`.

**Postconditions**: Robot is facing within `eps` radians of `heading_cdeg`.

**Acceptance Criteria**:
- [ ] `TURN` appears in HELP verb list.
- [ ] `EVT done TURN` emitted (not `EVT done` bare, not `EVT done T`).
- [ ] HEADING stop condition fires correctly (host unit test, `test_stop_condition.py`).
- [ ] Sign convention: positive `heading_cdeg` ⇒ CCW rotation (matches OTOS convention).
- [ ] `turn()` wrapper in `protocol.py`.

---

## SUC-006: Sensor-triggered stop

**Actor**: Host program driving toward a line or colour threshold

**Preconditions**: The relevant sensor (line / colour) is initialized; robot is idle.

**Main Flow**:
1. Host sends a drive verb with a sensor-stop modifier appended:
   `T <l> <r> <ms> sensor=<ch>:<op>:<threshold>`
   where `ch` ∈ {line0, line1, line2, line3, colorR, colorG, colorB, colorC},
   `op` ∈ {ge, le}, `threshold` is an integer sensor value.
   (The modifier is an optional extra token appended to T, D, or TURN commands.)
2. Firmware parses the `sensor=` token and appends a SENSOR stop condition alongside
   the primary stop.
3. If the sensor reading crosses the threshold before the primary stop fires, the
   command soft-stops early; `EVT done` is emitted with the same verb tag.
4. Host `wait_for_evt_done()` and `protocol.drive_until_sensor()` wrapper accept
   channel / threshold / direction arguments.

**Postconditions**: Robot has stopped; `EVT done <verb>` emitted — either because the
primary stop fired or the sensor threshold was crossed (whichever came first).

**Acceptance Criteria**:
- [ ] `sensor=` modifier accepted on `T`, `D`, and `TURN` commands (minimal: T only in scope for this sprint; D and TURN parses are add-ons).
- [ ] SENSOR stop condition fires correctly per `test_stop_condition.py` (pre-existing tests).
- [ ] Firmware emits same `EVT done <verb>` regardless of which stop fired (OR semantics).
- [ ] Host `drive_until_sensor()` wrapper added to `protocol.py`.
- [ ] ERR on unknown channel name or invalid operator.

---

## SUC-007: S-curve profiling (activation)

**Actor**: Developer or operator tuning the motion profile

**Preconditions**: `jMax` / `yawJerkMax` config keys exist (Sprint 017); defaults are 0.

**Main Flow**:
1. Operator sends `SET jMax=<mm/s³>` (and optionally `yawJerkMax=<deg/s³>`).
2. `BodyVelocityController::advance()` detects `jMax > 0` and activates the jerk-limited
   path: slew acceleration toward demand under the jerk bound, integrate.
3. At `jMax = 0` (default) the S-curve code path degenerates to trapezoid — no behaviour
   change for existing users.
4. Host test verifies: jerk-limited ramp reaches target slower than trapezoid under same
   `aMax`, confirming the additional constraint is active.

**Postconditions**: When `jMax > 0`, ramp is S-shaped; when `jMax == 0`, trapezoid.

**Acceptance Criteria**:
- [ ] `SET jMax=<n>` / `GET jMax` round-trip (pre-existing from Sprint 017 registry).
- [ ] S-curve path activated when `jMax > 0`; trapezoid when `jMax == 0`.
- [ ] Host test: jerk-limited ramp takes longer to reach target than trapezoid at same aMax.
- [ ] No behaviour change for any existing user of the default `jMax = 0`.
