---
status: in-progress
supersedes:
- velocity-profiler-arc-r-command.md
- body-velocity-controller.md
sprint: '017'
tickets:
- 017-001
- 017-002
- 017-003
- 017-004
- 017-005
---

# MotionCommand + body velocity control — `(v, ω)` twist, profiled ramp, pluggable stop conditions

> **Reconciliation note.** This issue merges and replaces two earlier drafts:
> `velocity-profiler-arc-r-command.md` (profiler inside MotorController, turn as
> `(v, radius)`, scoped to R + G) and `body-velocity-controller.md`
> (`BodyVelocityController` owned by DriveController, turn as `(v, ω)`). Per the
> stakeholder: **we commit to yaw-rate `(v, ω)` control — NOT `(v, radius)` and
> NOT `(v, ratio)`.** An arc is just `ω = v·κ` layered on top, and "ratio" survives
> only as an optional thin parse-time input adapter. On top of that reconciled base
> this issue adds the stakeholder's new requirement: a first-class **`MotionCommand`
> object** carrying a target twist + a small array of **stop conditions**, plus an
> **`X` cancel** verb. (The third pending issue,
> `replace-robot-facade-with-appcontext-struct.md`, is unrelated and untouched.)

## Problem / motivation

Two structural problems, one model fixes both:

1. **No acceleration limiting.** Motion is commanded as raw per-wheel speeds
   `(leftMms, rightMms)` applied to the per-wheel PI loop **instantly** —
   commanding a high speed from a dead stop jerks the chassis. The only velocity
   profiler is an ad-hoc `_vRamped` trapezoid buried inside the go-to (`G`)
   PURSUE loop (`DriveController.cpp:396-410`, member at `DriveController.h:109`).

2. **Termination logic is duplicated per command.** `driveAdvance()` is a hand-
   written if-chain, one branch per `DriveMode`: STREAMING keepalive watchdog
   (`DriveController.cpp:323`), TIMED deadline (`:339`), DISTANCE encoder check
   (`:349`), GO_TO arrival (`:360`). Every new "drive until X" needs another bespoke
   branch, and there is no way to compose conditions ("drive until distance OR
   until the line sensor trips") or to add a sensor-triggered stop at all.

We want motion expressed at the **body level** — average forward velocity + yaw
rate — everywhere, ramped under configurable limits, with **termination expressed
as a small list of composable stop conditions** that the operator constructs per
command. The wheel-space translation already exists (`BodyKinematics::inverse/
forward/saturate`, `source/control/BodyKinematics.h:37,52,73`).

**Outcome:** smoother chassis-friendly starts/stops/turns; one shared unit-testable
motion profiler instead of per-command ad-hoc ramping; one uniform command object
(`MotionCommand`) whose stop conditions replace the per-mode if-chain; new
capability — sensor-triggered and heading-triggered stops — for free.

## Decisions (confirmed with stakeholder)

- **Twist model = `(v, ω)`** internally (forward velocity mm/s, yaw rate rad/s).
  No singularities; yaw-rate and yaw-accel limits map directly and independently.
  `(v, radius)` is rejected (a constant radius couples ω to v and can't express
  independent yaw limits). `(v, ratio)` is rejected as a control representation;
  it survives only as an optional thin **parse-time** adapter (`(v_avg, ratio) → ω`),
  never as controller state. Inner-loop per-wheel `syncGain` coupling in
  MotorController is unchanged.
- **The whole motor-control stack runs off a single commanded `(v, ω)`.** Every
  motion command produces a commanded forward velocity + commanded yaw rate; the
  `BodyVelocityController` ramps the live twist toward that command and pushes the
  wheels. There is exactly one active twist at a time.
- **`MotionCommand` is a real object** — constructed on a target `(v, ω)`, carrying
  a fixed array of stop conditions, holding a pointer to the velocity controller.
  Inbound serial/radio commands *configure* a MotionCommand and *start* it;
  `driveAdvance` *ticks* it; `X` *cancels* it.
- **Stop conditions are a small fixed array of POD objects** (no heap, no virtual
  dispatch — embedded target). Built-in kinds: **time** (also serves the
  safety/keepalive stop), **distance travelled**, **heading / angular position
  reached**, **position reached**, **sensor condition**.
- **Profiler lives in `BodyVelocityController`, owned by `DriveController`**, and
  is *referenced by* the active `MotionCommand`. (Rejected the earlier plan to bury
  it in MotorController.) Trapezoid first, S-curve-ready (jerk config defaults to 0).
- **Incremental rollout, drivable at every commit. `S` stays raw/unramped teleop.**
  Migrate VW → G → T → D onto MotionCommand one verb at a time; `S` (`beginStream`)
  keeps calling `MotorController::setTarget` directly, byte-for-byte unchanged for
  existing host/calibration scripts.

## Architecture — three new pieces

```
   serial / radio verb (VW, R, T, D, G, TURN, …, X)
        │  parse + range-check (CommandProcessor)
        ▼
   DriveController::beginXxx()           ── configures ──▶ MotionCommand  (one owned instance)
        │                                                   ├─ target (v, ω)        (mutable; live keepalive/pursuit updates)
        │                                                   ├─ StopCondition[kMax]   (fixed array + count)
        │                                                   ├─ baseline ctx          (snapshot at start: t0, enc0, heading0, pose0)
        │                                                   ├─ reply sink            (replyFn/replyCtx/corrId  — async EVT done)
        │                                                   └─ BodyVelocityController*  ◀── owned by DriveController
        ▼
   DriveController::driveAdvance(now)  ── per tick ──▶ MotionCommand::tick(inputs, now)
                                                        1. (optional) recompute target  (G pursuit; D/G terminal decel cap)
                                                        2. bvc.setTarget(v, ω); bvc.advance(dt)   ── ramps + pushes wheels
                                                        3. for each StopCondition: evaluate(inputs, now, ctx)
                                                        4. if any fires → soft/hard stop → emit EVT done → IDLE
```

`MotionCommand` and `BodyVelocityController` are **single owned members** of
`DriveController` (one robot, one active motion). `start()` reconfigures the same
storage — no dynamic allocation. The legacy `DriveMode` enum is retained as a
status/telemetry tag during migration but no longer drives termination logic.

---

## Piece 1 — `BodyVelocityController` (`source/control/BodyVelocityController.{h,cpp}`, new)

The `(v, ω)` motion profiler + wheel push. Constructed on `MotorController&` and
`const RobotConfig&`; advanced once per tick. (Named `Body…` because
`VelocityController` is already the per-wheel PI class.)

```cpp
class BodyVelocityController {
public:
    BodyVelocityController(MotorController& mc, const RobotConfig& cfg);

    void  setTarget(float v_mms, float omega_rads);   // commanded twist (may be updated live)
    bool  advance(float dt_s);     // ramp (v,ω) toward target under limits; push wheels; true = still ramping
    void  reset();                 // zero commanded twist + profile derivatives (no brake)
    void  seedCurrent(float v, float omega);          // handoff from raw path without a lurch

    float currentV() const;  float currentOmega() const;
    float targetV()  const;  float targetOmega()  const;  bool atTarget() const;
};
```

Per-tick math (trapezoid; S-curve additive). Limits read live from
`const RobotConfig&` each tick (like `aMax`/`vWheelMax` already are); deg→rad at
the use site.

```
// Linear channel (asymmetric accel / decel)
dv_max = (vTgt >= v ? aMax : aDecel) * dt_s
v      = approach(v, clamp(vTgt, -vBodyMax, +vBodyMax), dv_max)
// Yaw channel (independent rate + accel limits)
omega  = approach(omega, clamp(omegaTgt, -yawRateMax, +yawRateMax), yawAccelMax * dt_s)
// then → wheels:
BodyKinematics::inverse(v, omega, cfg.trackwidthMm, vL, vR);
BodyKinematics::saturate(vL, vR, cfg.vWheelMax, cfg.steerHeadroom, sL, sR);
mc.setTarget(sL, sR);
```

`approach(cur,tgt,step)` = step toward tgt, clamped to ±step. **Ordering invariant:
profile → inverse → saturate → setTarget** (matches the two existing call sites,
`beginVelocity` and the PURSUE phase). `saturate()` stays last as the wheel-space
ceiling guard for the case where in-limit `v` plus in-limit `ω` jointly exceed a
wheel max. **S-curve (later):** when `jMax > 0`, slew *acceleration* toward demand
under the jerk bound and integrate; degenerates to trapezoid at `jMax = 0`.

`advance()` ticks on the **PID `dt`** (the actual measured control-tick elapsed,
clamped 5–50 ms — same dual-clock care as the existing PID, `MotorController.cpp`
`_lastPidMs`). It must NOT be ticked from `driveAdvance`'s own clock as well or the
ramp double-counts.

---

## Piece 2 — `StopCondition` (`source/control/StopCondition.{h,cpp}`, new)

A POD tagged struct — a fixed array of these is the "list you add to." No heap, no
virtuals; evaluated each tick against `HardwareState` + the command's baseline.

```cpp
struct MotionBaseline {            // captured by MotionCommand::start()
    uint32_t t0Ms;
    float    enc0Mm;               // (encLMm + encRMm) * 0.5 at start  → distance travelled
    float    heading0Rad;          // pose heading at start              → angular delta
    float    pose0X, pose0Y;       // pose at start                     → straight-line displacement
};

struct StopCondition {
    enum class Kind : uint8_t { NONE, TIME, DISTANCE, HEADING, POSITION, SENSOR };
    enum class Cmp  : uint8_t { GE, LE };   // sensor / threshold direction

    Kind kind = Kind::NONE;

    // TIME:      ms        (elapsed since t0)            — also the safety/keepalive stop
    // DISTANCE:  mm        (|travelled| target)
    // HEADING:   rad       (target absolute heading; fires when |wrap(target-heading)| < eps)
    // POSITION:  ax,ay mm + radius mm (fires when within radius of (ax,ay))
    // SENSOR:    channel selector + threshold + Cmp
    float    a = 0, b = 0;          // primary / secondary scalar param (per kind, see above)
    uint8_t  sensor = 0;            // SensorSel enum (LINE0..3, COLOR_R/G/B/C, OTOS_H, …) for SENSOR
    Cmp      cmp = Cmp::GE;

    // true ⇒ this condition is satisfied → command terminates
    bool evaluate(const HardwareState& s, uint32_t now_ms, const MotionBaseline& base) const;
};
```

- `TIME` ⇒ `now_ms - base.t0Ms >= a`. A **safety stop** is just a short `TIME`
  condition; the **VW keepalive watchdog** is a `TIME` condition re-armed (baseline
  `t0` bumped) on every keepalive re-send — replacing the bespoke STREAMING watchdog.
- `DISTANCE` ⇒ `fabs((s.encLMm+s.encRMm)*0.5f - base.enc0Mm) >= a`. (Uses raw, not
  filtered, encoder sum — see the D-command finding about filtered encLMm stalling
  the distance check.)
- `HEADING` ⇒ turn-to-heading / "angular position reached"; fires on heading within
  `b` (eps) of target `a`. Drives the rotation-calibration use case directly.
- `POSITION` ⇒ go-to arrival (within radius `b` of `(a, ax)` — store target in
  `a,b` + a third; if two scalars are too few, widen the param block to 4 floats).
- `SENSOR` ⇒ generic "until a sensor reads X": `s.line[ch]`, `s.colorR/G/B/C`,
  `s.otosH`, etc., compared `GE`/`LE` against a threshold.

**Termination is OR across the array:** the command stops when *any* condition fires
(first-wins; the firing condition's index is reported in the EVT for debuggability).
A command with **zero** conditions runs until cancelled (pure streaming, e.g. raw VW
with only the safety stop, or an open-ended `R`).

---

## Piece 3 — `MotionCommand` (`source/control/MotionCommand.{h,cpp}`, new)

The object the stakeholder asked for: a target twist + its stop conditions + a
reference to the velocity controller. One instance owned by `DriveController`,
reconfigured per command.

```cpp
class MotionCommand {
public:
    static constexpr uint8_t kMaxStopConds = 4;
    enum class StopStyle : uint8_t { SOFT, HARD };   // SOFT = profiler ramps to 0; HARD = immediate stop()

    void configure(float v_mms, float omega_rads, BodyVelocityController* bvc);
    bool addStop(const StopCondition& c);            // append to fixed array; false if full
    void setReplySink(ReplyFn fn, void* ctx, const char* corrId);
    void setStopStyle(StopStyle s);                  // default SOFT

    void start(const HardwareState& inputs, uint32_t now_ms);  // snapshot baseline, seed controller, capture sink
    void setTarget(float v_mms, float omega_rads);             // live update (keepalive re-send / G pursuit)

    // Returns true while running. On the tick a stop condition fires it begins the
    // configured stop (soft ⇒ target (0,0), profiler ramps down; hard ⇒ mc.stop()),
    // and once at rest emits "EVT done <tag> #<corrId>" via the reply sink, then idles.
    bool tick(HardwareState& inputs, uint32_t now_ms);

    void cancel(StopStyle s = StopStyle::HARD);      // X / STOP — tear down, emit EVT cancelled
    bool active() const;
};
```

Lifecycle: `configure()` → `addStop()` × N → `setReplySink()` → `start()` (captures
`MotionBaseline`, calls `bvc->seedCurrent` / `reset`, copies the reply sink). Each
tick: optionally recompute target (pursuit / terminal decel cap), `bvc->advance(dt)`,
then evaluate stop conditions. SOFT-stop commands enter a STOPPING sub-phase
(target `(0,0)`, profiler ramps down) with an absolute safety deadline (≈3 s, the
soft-stop teardown guarantee from the R draft) before emitting `EVT done` and going
IDLE; HARD-stop / cancel skip the ramp.

---

## Command → MotionCommand mapping

| Verb | Twist | Stop conditions | Stop style | Notes |
|---|---|---|---|---|
| **VW** `v ω` | `(v, ω)` | one short `TIME` (safety, re-armed per keepalive) | SOFT | replaces STREAMING watchdog; ramps now |
| **R** `speed radius` (arc) | `(speed, speed·κ)`, `κ=1/radius`, `radius=0 ⇒ κ=0` | safety `TIME`, or none (streaming) | SOFT | arc = thin `ω=v·κ` adapter on the `(v,ω)` core; `R 0 r` ⇒ soft stop |
| **T** `L R durMs` | `forward(L,R)→(v,ω)` | `TIME(durMs)` | SOFT | input adapter `(L,R)→(v,ω)` at begin |
| **D** `L R mm` | `forward(L,R)→(v,ω)` | `DISTANCE(mm)` | SOFT | + terminal decel cap `vTgt=min(vTgt, √(2·aDecel·d_remaining))`; keep the encoder-reset workaround |
| **G** `x y speed` | pursuit recomputes `(v, ω=v·κ_bearing)` each tick | `POSITION(x,y,r)` | SOFT | retire `_vRamped`; PRE_ROTATE stays a turn-in-place |
| **TURN** `θ` (new, optional) | `(0, ±yawRate)` | `HEADING(θ, eps)` | SOFT | turn-to-heading; serves rotation calibration |
| **…until sensor** (new) | any twist | add a `SENSOR` condition | SOFT | "drive until line/colour/OTOS reads X" |
| **X** (new) | — | — | HARD | **cancel** active MotionCommand: `cancel(HARD)` → `EVT cancelled` → IDLE |
| **STOP** (existing) | — | — | HARD | alias of X; keep for compatibility |
| **S** `L R` | — (raw) | — | — | **unchanged**: `beginStream` → `MotorController::setTarget` directly |

`X`/`STOP` route through `DriveController::cancel()` → `MotionCommand::cancel(HARD)`
→ `BodyVelocityController::reset()` + `MotorController::stop()` (the hard-stop teardown).

---

## New `Config.h` params (additive) + `kRegistry[]` SET/GET keys

Reuse existing `aMax` (300) / `aDecel` (250) mm/s² for the linear channel. Add:

| Field | Key | Unit | Default |
|---|---|---|---|
| `vBodyMax` | `vBodyMax` | mm/s | 400 |
| `yawRateMaxDeg` | `yawRateMax` | deg/s | 180 |
| `yawAccelMaxDeg` | `yawAccMax` | deg/s² | 720 |
| `jMax` | `jMax` | mm/s³ | 0 (trapezoid) |
| `yawJerkMaxDeg` | `yawJerkMax` | deg/s³ | 0 (trapezoid) |

Defaults chosen so the linear ramp equals today's behaviour and S-curve is off until
enabled. `aMax`/`aDecel` are already in the registry (`CommandProcessor.cpp:101-102`).

---

## Rollout steps (drivable at every commit)

1. **Config + registry** (no behaviour change): add the five fields/defaults +
   `SET`/`GET` keys; smoke-test round-trip.
2. **`BodyVelocityController`, trapezoid only, unwired**: add to the build file list;
   host unit test in isolation (constant target ⇒ `v` ramps at `aMax`, decels at
   `aDecel`; `ω` obeys yaw rate/accel limits; spin-in-place `v=0, ω>0`; straight
   `ω=0`).
3. **`StopCondition` + `MotionCommand`, unwired**: host unit tests — each `Kind`
   fires at the right threshold off a synthetic `HardwareState`/baseline; OR-across-
   array; zero-condition command never self-terminates; SOFT vs HARD teardown.
4. **Wire VW** onto MotionCommand (`(v,ω)` + safety `TIME`); verify it now ramps and
   that keepalive loss still stops the robot (the migrated watchdog).
5. **Add R** (arc) as `(speed, speed·κ)` + soft stop; bench the arcs.
6. **Migrate G**: replace inline `_vRamped` (`DriveController.h:109`,
   `.cpp:396-410`) with a `POSITION`-stop MotionCommand whose pursuit hook updates
   `(v, ω=v·κ)` each tick; keep the `√(2·aDecel·d)` terminal cap. PRE_ROTATE stays
   a raw turn-in-place. Validate `test_pursuit_arc_steering.py` still passes.
7. **Migrate T then D** (separate commits): `(L,R)→forward()→(v,ω)` at begin;
   `TIME`/`DISTANCE` stop conditions; re-verify the **D-timeout heuristic** tolerates
   ramp-up (smoother starts ⇒ less distance in the first ~200 ms).
8. **Add `X` cancel** + `STOP` alias.
9. **New conditions** (after the migration lands): `TURN`/`HEADING`, `SENSOR`-stop
   verbs + host tests.
10. **Leave S raw** throughout.

---

## Critical files

- `source/control/BodyVelocityController.{h,cpp}` — **new** (profiler + wheel push).
- `source/control/StopCondition.{h,cpp}` — **new** (POD condition + `evaluate`).
- `source/control/MotionCommand.{h,cpp}` — **new** (twist + stop array + sink + lifecycle).
- `source/control/DriveController.{h,cpp}` — own `_bvc` + `_activeCommand`; route
  VW/R/T/D/G through MotionCommand; `cancel()`; **remove `_vRamped`**
  (`.h:109`, `.cpp:215,372,396-410`); `driveAdvance` if-chain → `_activeCommand.tick()`.
- `source/types/Config.h` — five new limit fields + defaults (near the existing
  `aMax`/`aDecel`).
- `source/app/CommandProcessor.cpp` — `kRegistry[]` SET/GET entries; new `R`/`X`
  (and later `TURN`) verbs + HELP; optional `(v, ratio)` parse adapter.
- `source/control/RobotState.h` — reference: `HardwareState` (sensor fields read by
  `SensorCondition`), `TargetState` (reply sink + `DriveMode` status tag),
  `MotorCommands.tgtLMms/R`.
- `source/control/BodyKinematics.{h,cpp}` — reference (`inverse`/`forward`/`saturate`).
- `host/robot_radio/robot/protocol.py` — `arc()`, `turn()`, `cancel()` host wrappers
  (mirror `vw`, streaming via `send_fast`; docstrings note keepalive within `sTimeout`).
- Build file list — confirm `source/control/*.cpp` enumeration (glob vs explicit) so
  the three new `.cpp` files compile.

---

## Risks (ranked) — handling baked into the plan above

1. **Profiler `dt` / dual-clock.** `BodyVelocityController::advance` ticks ONLY on
   the PID `dt`; the MotionCommand pushes target/twist but does not advance the ramp
   on `driveAdvance`'s own clock. Wiring both double-counts the ramp.
2. **`tgtLMms/R` must stay live every tick.** They drive the wedge detector, coast
   logic, and the `driving`/`refreshedWheel` encoder gate. The controller writes them
   every tick during ramp; at SOFT-stop completion the profiler reaches exactly 0 so
   the all-zero coast early-return fires correctly.
3. **Soft-stop termination.** The STOPPING sub-phase must not re-arm its own stop
   condition; an absolute ≈3 s deadline guarantees IDLE even if `atRest` never trips.
4. **Distance stop on filtered encoders.** `DISTANCE` must use the raw encoder sum,
   not the filtered `encLMm`, or it stalls/runs away (the prior D finding).
5. **Mode/teardown regressions.** `S`/`beginStream` must keep calling
   `MotorController::setTarget` directly and never construct a MotionCommand; add an
   R-then-S (and VW-then-S) interleave test asserting straight drive with no steer bias.
6. **Removing `_vRamped`.** Safe iff the G MotionCommand seeds the controller from
   rest on both PURSUE entries; verify `test_pursuit_arc_steering.py`.
7. **Sign / divide-by-zero.** `radius=0 ⇒ κ=0` (straight); pin the CCW-positive ω
   convention (positive radius ⇒ left arc, matching `inverse`) with a Python mirror test.
8. **Stop-array capacity / param packing.** `kMaxStopConds=4` and the scalar param
   block must hold the widest condition (POSITION needs target x,y + radius). Widen the
   `StopCondition` float block if two scalars are insufficient; assert on `addStop`
   overflow rather than silently dropping.
9. **D-timeout heuristic vs ramp-up.** Smoother starts cover less distance early;
   re-tune/verify the timeout tolerates the ramp.

---

## Acceptance / verification

1. **Host unit tests** (mirror the pure-Python `tests/dev/test_velocity_controller.py`
   / `test_body_kinematics.py` patterns):
   - `BodyVelocityController`: linear ramp = `aMax`/`aDecel`; yaw obeys
     `yawRateMax`/`yawAccelMax`; spin-in-place and straight non-degenerate.
   - `StopCondition`: each `Kind` (TIME, DISTANCE, HEADING, POSITION, SENSOR) fires at
     the correct threshold; OR-across-array reports the firing index; zero-condition
     command never self-terminates.
   - `(speed, radius) → κ → inverse → saturate → (vL,vR)` mapping incl. `radius=0`
     (straight: `vL==vR`) and signed radius (CCW sign convention).
   - Existing G / kinematics / pursuit tests stay green:
     `uv run --with pytest python -m pytest -q`.
2. **Clean build** (stale incrementals flash broken binaries):
   `python3 build.py --clean <target>`.
3. **Bench (robot on stand, safe to drive)** via `uv run rogo …`, PING/ID liveness
   preflight first; verify flash target is the robot, not the radio relay:
   - `SET`/`GET` the five new keys round-trip.
   - **VW**: step change ⇒ velocity *ramps* (not steps), respects yaw limits;
     keepalive loss ⇒ safety stop fires.
   - **R**: `R 300 0` straight smooth ramp from rest (no jerk); `R 300 200` left arc;
     `R 300 -200` right arc; `R 0 200` soft stop; `X` ⇒ immediate stop.
   - **G**: go-to still arcs in and decelerates cleanly (`_vRamped` removal regression).
   - **D**: fixed distance terminates accurately, no timeout trip, no spasm.
   - **TURN / SENSOR** (once added): turn-to-heading stops at the commanded heading;
     a `SENSOR`-stop drive halts when the line/colour threshold trips.
   - **X / STOP**: cancels any active motion immediately.
   - Confirm `S`/`T`/`D` calibration scripts behave unchanged (no steer bias).
4. Optional: capture a velocity-vs-time telemetry plot to confirm the trapezoidal
   slopes match `aMax`/`aDecel`.
</content>
</invoke>
