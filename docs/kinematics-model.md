# Robot Kinematics

A guide to how this differential-drive robot is modeled and controlled. Two
parts: **(1)** the abstract kinematic model, and **(2)** how we implement it.

Notes marked **[Validation]** / **[Critique]** record where the design was
checked against (or corrects) the original mental model. Related planning issues:
`firmware-architecture-refactor` (where `DriveController` / `VelocityController`
land), `nezha-chip-velocity-readspeed-0x47` (feeds the per-wheel velocity loop),
and `protocol-v2-raw250-hard-break` (the command surface).

## Conventions

- Positions in **mm**, angles in **radians CCW-positive** internally (wire
  protocol may use centidegrees). Body frame: **+x forward**, **+y left**,
  yaw measured from +x.
- Pose **p = (x, y, θ)**. Body twist **(v, ω)**: forward speed `v` (mm/s) and yaw
  rate `ω` (rad/s). Wheel speeds **vL, vR** (mm/s).
- **Track width `b`** (mm) = the lateral distance between the two driven wheels'
  contact patches. This is the *only* geometric length in differential-drive
  kinematics — the robot rotates about the midpoint of the wheel axle, governed
  entirely by `b`.
  - **Not "wheelbase."** Wheelbase is the longitudinal front-to-rear axle
    distance of an Ackermann (car-like) vehicle; a two-wheel differential drive
    has no front/rear axle pair, so there is no wheelbase term. People sometimes
    say "wheelbase" loosely for wheel separation — in this document that quantity
    is always the track width `b`.
  - **Effective vs geometric.** On a 4-wheel skid-steer, tire scrub during turns
    makes the *effective* track width larger than the tape-measured value, so `b`
    is an **empirically-calibrated** parameter, not a pure geometric constant.

---

# Part 1 — The Abstract Kinematic Model

## 1.1 What kind of system this is

The robot is a **differential-drive** platform, which is the classic
**unicycle** model: two independently driven wheels on a common axle. It has

- **3 pose degrees of freedom** (x, y, θ), but
- **2 control inputs** (v, ω) — equivalently (vL, vR).

It is therefore **underactuated**, and **nonholonomic**: it cannot translate
sideways. At any instant it can only move along its current heading and rotate.

**[Validation] "Our only DOF for motion is forward."** Correct as an
*instantaneous* statement: the velocity space is 2-D (v, ω) and there is a hard
constraint `ẏ_body = 0` (no sideways slip). The "3 for pose, 2 for velocity"
framing is exactly right.

## 1.2 The control stack, bottom to top

We build kinematics in layers; each layer exposes a clean command to the one
above.

| Layer | Owns | Command it accepts | Produced from |
|---|---|---|---|
| **0 Motors** | motor-controller chip | PWM per wheel | — |
| **1 Wheel velocity** | per-wheel velocity PID | wheel-speed setpoints `vL,vR` | chip velocity feedback |
| **2 Body kinematics** | (v,ω) ↔ (vL,vR) map | body twist `(v, ω)` | inverse kinematics |
| **3 Pose controller** | pursuit + limits | goal point `(x, y)` | current pose + goal |
| **4 Commanding / sequencing** | accel limits, mode | "go to (x,y)" or "drive (v,ω)" | host / behaviors |

The bottom three are pure kinematics/control; the top is where commands and
trajectory-shaping live.

## 1.3 Differential-drive kinematics (the math)

**Inverse** (body twist → wheel speeds):
```
vL = v − ω·(b/2)
vR = v + ω·(b/2)
```
**Forward** (wheel speeds → body twist):
```
v = (vR + vL) / 2
ω = (vR − vL) / b
```
**Arc geometry.** Holding `(v, ω)` constant traces a circle of
```
radius R = v / ω,   curvature κ = ω / v = (vR − vL) / (b · (vR+vL)/2)
```
A **constant wheel-speed ratio is a constant-radius arc.**

**[Validation] "Arcs are natural for this robot."** Yes. Constant wheel ratio ⇒
constant curvature ⇒ circular arc. Building motion out of arcs is the
physically-natural primitive; you are not fighting the platform.

## 1.4 Two ways we command motion

1. **Velocity command `(v, ω)`** — a direct body twist. Used for teleop /
   streaming control; safety-watchdogged so loss of commands stops the robot.
2. **Position command `(x, y)`** (heading usually free) — the fundamental
   primitive almost every higher behavior is built on.

**[Validation] "Reaching a position with a particular yaw is generally not
possible."** This needs nuance. A differential drive **is controllable** — it can
reach *any* full pose (x, y, θ) — but **not along a single forward arc**. Getting
a specific final yaw requires a maneuver (e.g. drive-then-rotate, or a smooth
pose regulator that may pirouette or reverse). So the right design stance is:

- **Primary primitive: go to (x, y), heading-free.**
- **Full pose (x, y, θ): optional, later**, via a dedicated pose regulator
  (e.g. a Lyapunov/polar controller) — don't force it through the arc primitive.

## 1.5 Go-to-position as a pursuit arc

Express the goal in the robot frame as `(dx, dy)` (dx forward, dy left). The arc
from the robot (pointing +x) through that point has curvature
```
κ = 2·dy / (dx² + dy²)            (pure-pursuit geometry, lookahead² = dx²+dy²)
```
So each update: pick a forward speed `v` (Part 1.6), set **ω = v·κ**, convert to
wheel speeds, and drive. Recompute every time the pose updates — a receding-
horizon pursuit that naturally curves onto the target.

**[Validation] "Pure-pursuit-style arc, not the literal algorithm."** Exactly
the right instinct. We use the pursuit *geometry* (the arc to the target) as the
steering law, recomputed continuously, rather than committing to a fixed planned
path.

**[Critique — a hole to close] Targets beside/behind the robot.** The arc law
gives huge curvature (tiny radius) when `dy` is large relative to `dx`, and is
undefined/ill-behaved for points directly behind. **Fix:** gate on heading error
— if the bearing to the target exceeds a threshold, **rotate in place first**
(spin to roughly face the target), then pursue the arc. This is the
PRE_ROTATE→ARC pattern. Without the gate, "go to a point behind me" misbehaves.

## 1.6 Acceleration limits & (light) trajectory shaping

**[Validation] "Don't put much trajectory planning in the code; just ramp the
commanded velocity up and down, with a max accel/decel."** Endorsed. The
lightweight, principled realization is two cheap pieces:

1. **Slew-limit `v`** toward its target by the max acceleration each tick:
   `v ← v ± a_max·dt` (a ramp / first-order rate limiter). This bounds accel and
   decel during normal driving.
2. **Decel-to-target cap** so it stops *on* the point: cap the speed by
   `v_cap = sqrt(2·a_decel·d_remaining)`, where `d_remaining` is distance to the
   goal. Take `v = min(v_ramped, v_cap, v_user_max)`.

Together these are exactly a **trapezoidal velocity profile**, computed online
with one `sqrt` and no stored plan. No path planner needed in firmware — that's
the right boundary. (Heavier planning, if ever wanted, belongs on the host and
arrives as a sequence of (x,y) waypoints.)

## 1.7 Wheel saturation — preserve the arc, sacrifice speed

This is the subtle one. To hold a commanded **curvature** (wheel-speed ratio)
under uneven load, the wheel controller pushes more power to the loaded wheel. If
that wheel **saturates at 100% PWM**, it can no longer hit its setpoint, the
actual ratio breaks, and the robot **drifts off the arc**.

**Principle: geometry beats speed.** When a wheel would exceed the ceiling,
**scale both wheel setpoints down by the same factor** so the faster wheel sits
exactly at the limit:
```
s = v_wheel_max / max(|vL|, |vR|)        (only when that max > v_wheel_max)
vL ← s·vL ,  vR ← s·vR
```
Because both wheels scale equally, the **ratio — and therefore the radius
R = v/ω — is preserved**: the robot follows the *same arc*, just slower. In
twist terms this is a **velocity multiplier** that drops `v` and `ω` together.

**[Validation] "Lower the total velocity by a multiplier (e.g. 0.9) when a wheel
tops out."** This is the standard **angular-velocity-priority** saturation
strategy, and it's correct. Two flavors:
- **Exact** (above): one-shot scale `s` that puts the limiting wheel right at the
  ceiling — deterministic, stays on the arc.
- **Soft** (the 0.9 idea): multiply `v` by a factor <1 and let it settle over a
  few ticks — smoother, less chattery, but converges to the same place.

Use the exact scale as the hard guarantee; optionally slew it for smoothness.

**[Critique] Straight-line saturation.** If *both* wheels want max (straight at
top speed), there's no curvature authority left — fine for straight-line, but the
pose controller must not expect to steer at absolute top speed. Keep a small
**headroom** below `v_wheel_max` for steering authority.

## 1.8 Model validation summary (holes & recommendations)

1. **Full-pose reachability** — reachable via maneuvers, not single arc. Ship
   (x,y) heading-free as the primitive; add a pose regulator later for (x,y,θ).
2. **Behind/beside targets** — add the **turn-in-place gate** before the arc.
3. **Saturation** — scale both wheels equally (preserve curvature); keep steering
   headroom; the velocity multiplier is the twist-space view of the same thing.
4. **Velocity sensing** — a per-wheel velocity PID needs a clean velocity signal;
   prefer the **chip's velocity read** over differentiating encoder ticks (noisy/
   quantized). See Part 2.2 and the `nezha-chip-velocity-readspeed-0x47` issue.
5. **Odometry integration** — use **midpoint (exact-arc) integration** of heading
   (Part 2.4), not plain forward-Euler, to limit error during turns.

---

# Part 2 — How We Implement It

## 2.1 Modules (post-refactor)

- **VelocityController** (Layer 1) — per-wheel PID on measured wheel velocity →
  PWM. Replaces today's distance-ratio cross-coupling controller as the inner
  loop. (Today `MotorController` has **no per-wheel velocity PID** — only a
  cumulative-distance ratio PID; this is the main inner-loop change.)
- **Body kinematics** (Layer 2) — the (v,ω)↔(vL,vR) maps of Part 1.3, plus the
  saturation scaler of Part 1.7.
- **DriveController** (Layer 3–4) — pose pursuit (Part 1.5), turn-in-place gate,
  accel/decel shaping (Part 1.6), mode/watchdog. Owned by `Robot` per the
  `firmware-architecture-refactor` issue.
- **Odometry + fusion** — the authoritative pose and its update rules (below).
- **RobotConfig** — all calibration/tuning in one object (Part 2.7).

## 2.2 Encoders and the distance calibration

The motor controller reports **wheel angle** at **0.1°/LSB**. We convert angle to
distance:
```
distance_mm = angle_deg × mmPerDeg
```
`mmPerDeg ≈ (π · wheel_diameter) / 360`, but gearing and tire compression make the
effective value differ, so **calibrate empirically per wheel** (`mmPerDegL`,
`mmPerDegR`): drive a measured straight distance, divide by the angle turned.

**Wheel velocity.** Prefer the chip's on-board velocity read (`readSpeed`,
returns laps/s) converted to mm/s by an **empirically-pinned** laps→mm scale
(the chip's "lap" may not equal `360·mmPerDeg`; measure it). Fall back to
encoder-delta/dt only if the chip signal proves unreliable. This is the subject
of the `nezha-chip-velocity-readspeed-0x47` issue.

## 2.3 Two sources of position: encoders vs optical-flow odometer

We have two ways to know where we are:

- **Encoders (dead reckoning).** High rate, low latency, but **accumulate error**
  — wheel slip, uneven tires, and the trackwidth estimate all drift over time.
- **OTOS optical-flow odometer.** Reports an integrated pose and body velocity
  with **less long-term drift / better behavior under slip**, but it is the
  *correcting* source, used less frequently (and reads should be treated as the
  slower-cadence, more-trusted measurement).

**[Validation]** There is **one authoritative robot state (x, y, θ)** held in
`Robot`. We **predict** it frequently from encoder deltas and **correct** it
occasionally from the OTOS. This predict/correct split *is* the structure of a
Kalman filter; we start with a simpler version of it (next).

## 2.4 Maintaining the pose: predict fast, correct gently

**Predict (every fast tick).** Integrate encoder deltas into the pose. Use
**midpoint (exact-arc) integration** so turns don't bias the heading:
```
dC = (dL + dR)/2 ;  dθ = (dR − dL)/b
θ_mid = θ + dθ/2
x += dC·cos(θ_mid) ;  y += dC·sin(θ_mid) ;  θ = wrapπ(θ + dθ)
```
(Today's `Odometry::update` uses the start-of-tick heading, not the midpoint —
upgrade it here.)

**Correct (when an OTOS sample arrives).** Don't snap — blend, to avoid jumps:
```
x ← x + α_pos·(x_otos − x)         (same for y)
θ ← θ + α_yaw·wrapπ(θ_otos − θ)    (shortest-arc)
```
Small gains (`α_pos`, `α_yaw` ≈ 0.1–0.3) pull the estimate toward the trusted
sensor over several updates rather than in one jump. This is a **first-order
complementary filter**: encoders supply the high-frequency motion, OTOS supplies
the low-frequency truth.

**[Validation] "Weighted average so it doesn't jump."** Correct, and the gain
*is* the knob: low α = smooth but slow to trust OTOS; high α = responsive but
jumpy. Add **outlier gating** (reject an OTOS correction that disagrees with the
prediction by more than a threshold, unless forced) so a bad sample can't yank
the pose.

**Later: EKF.** A proper Kalman/EKF replaces the fixed α's with
covariance-derived gains and can model latency, all **behind the same
predict/correct interface** — no change to the layers above. Ship the
complementary filter first.

## 2.5 The control loop, each tick

1. **Predict** pose from encoder deltas (2.4).
2. **Correct** pose from OTOS *if* a fresh sample is available (2.4).
3. **Steer:** goal in robot frame → curvature κ (1.5); choose `v` from the
   accel/decel profile (1.6); set **ω = v·κ**. Recompute the **yaw rate every
   pose update**.
4. **Map** (v, ω) → (vL, vR) (1.3); apply **saturation scaling** (1.7).
5. **Track:** per-wheel **velocity PID** drives each wheel to its setpoint →
   PWM (2.1).
6. **Terminate** when within arrival tolerance of (x, y); emit completion.

## 2.6 Rates & scheduling (why this is multi-rate)

Encoder/velocity reads are **I2C-expensive** (each transfer carries vendor-
mandated ~4 ms delays, so a two-wheel read is several ms); OTOS reads are
cheap. So the loop is **multi-rate**: run the velocity PID and pose prediction at
the fast cadence the I2C budget allows, and fold in OTOS corrections at a slower
cadence. Keep the steering math light enough to run every fast tick. (The
scheduler that enforces this lives in the `firmware-architecture-refactor` issue.)

## 2.7 Calibration & tuning parameters (RobotConfig)

| Param | Meaning |
|---|---|
| `mmPerDegL`, `mmPerDegR` | encoder angle→distance, per wheel (2.2) |
| `trackwidthMm` (b) | track width / wheel separation (1.3, 2.4) |
| `lapsToMm` | chip-velocity laps/s → mm/s scale (2.2) |
| `vel.kP/kI/kFF` (per wheel) | inner velocity-PID gains (2.1) |
| `aMax`, `aDecel` | accel / decel limits (1.6) |
| `vWheelMax`, `steerHeadroom` | wheel-speed ceiling + steering reserve (1.7) |
| `turnInPlaceGate` | bearing error above which we rotate first (1.5) |
| `arriveTolMm` | go-to completion tolerance (2.5) |
| `alphaPos`, `alphaYaw`, `otosGate` | fusion gains + outlier gate (2.4) |
