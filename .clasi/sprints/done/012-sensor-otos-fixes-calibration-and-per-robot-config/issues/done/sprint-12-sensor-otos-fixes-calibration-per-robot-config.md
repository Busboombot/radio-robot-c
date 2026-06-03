---
status: done
sprint: '012'
tickets:
- '001'
- '002'
- '003'
- '004'
- '005'
- '006'
- '007'
- 008
- 009
- '010'
- '011'
---

# Sprint 12 — Sensor/OTOS Fixes, Calibration & Per-Robot Config

## Context

Sprints 007–011 built the firmware + v2 protocol + radio-relay path, and the
robot now drives over the RADIORELAY. But on-playfield bench testing exposed that
several sensor/telemetry paths are wrong, so the robot is **not student-ready**:

- The encoders are **good** (balanced L/R, ~417 mm for a commanded 400, `ZERO enc`
  works) and `mmPerDegL/R = 0.487/0.481` already **match the prior known-good
  system** — so distance sensing is sound.
- But: the **pose telemetry reports raw OTOS LSB instead of the fused odometry**
  (looked ~5× wrong, broke go-to); **OTOS scalars are never set** at init; the
  **velocity feedback uses the Nezha chip's 0x47 readSpeed**, which is stuck at
  ~30 mm/s regardless of actual speed (feeds the PID junk); **telemetry is stale
  at idle** (control tick gated to non-idle); and several **compiled calibration
  defaults are wrong/missing** (trackwidth 120 vs 126; OTOS scalars unset; no
  per-direction turn gain).

The prior system (`/Volumes/Proj/proj/league-projects/scratch/radio-robot/`) is the
source of truth: per-robot JSON config (`data/robots/<name>.json`), a schema, a
loader, and calibration scripts. A **partial port already exists** in this repo's
`host/robot_radio/` (`config/robot_config.py` loader present), but the
protocol-touching parts speak the **dead pre-v2 protocol** (`KML/KMR/OO/OK`, the
`SO` stream) and must be rewritten to v2.

**Outcome:** robot drives accurate distances, holds straight lines, turns
accurately (both directions), reports a correct fused pose, go-to works, and a
per-robot config loads known-good calibration at connect — verified on the
playfield against OTOS/overhead-camera ground truth.

OTOS hardware is **known-good** (tracks the camera closely elsewhere); every
symptom here is a firmware integration/units bug or a missing calibration value —
**do not "fix" by disabling sensors.**

## Scope (stakeholder chose "everything in Sprint 12")

Firmware fixes + known-good defaults + per-direction turn gain + the full
host-side per-robot config system + the calibration-script rewrite to v2/relay.

## Tickets (dependency-ordered)

Firmware tickets each require a **clean build** (`mbdeploy build --clean`) and
reflash to **robot enum 2** (NOT relay enum 1). Hardware/bench acceptance criteria
are stakeholder-run on the playfield and may be marked deferred.

**T01 — Config fields + SET/GET registry: OTOS scalars + turn asymmetry**
(foundational; no deps). In `source/types/Config.h` add to `RobotConfig`:
`otosLinearScale`, `otosAngularScale`, and per-direction turn calibration
`rotationGainPos`, `rotationGainNeg`, `rotationOffsetDeg`, `rotationOffsetDegNeg`
(+ `rotationalSlip` if used). Add `kRegistry[]` entries in
`source/app/CommandProcessor.cpp` following the existing `CFG_F`/`CFG_FI` pattern.
AC: `SET`/`GET` round-trip for each; GET dump still fits the 512 buffer; clean build.

**T02 — OtosSensor applies linear/angular scalars at init from config**
(deps T01). `source/hal/OtosSensor.cpp init()` + the OTOS bring-up in
`source/robot/Robot.cpp`: after `init()`, convert `otosLinearScale`/`otosAngularScale`
floats → int8 via `round((scale−1)/0.001)` (clamp ±127) and call the existing
`setLinearScalar`/`setAngularScalar`. AC: after boot, `OL`≈+50, `OA`≈−13 with no
host command; runtime `OL`/`OA` still override. Bench: measured run closer to truth.

**T03 — TLM/pose surface reports fused odometry (mm), not raw OTOS LSB**
(no deps). `source/robot/Robot.cpp` tick() pose block (~174-186): always report
`_odo.getPose()` (mm, mm, centidegrees), regardless of `_otosPresent` — the fused
odometry is already maintained in mm by `DriveController::correct()`. Keep `OP` as
raw-LSB **clearly labeled** as raw; add an mm form (`OP` mm or a flag). AC: drive 1 m
→ `pose=` x≈1000 (mm) not ≈3279 (LSB); `test_tlm_stream.py`/`test_otos_fusion.py`
assert mm-scale pose.

**T04 — Fix C++ readSpeed (chip 0x47 works per vendor); chip velocity feedback + encoder-delta fallback**
(no deps; pairs with T05). The chip's readSpeed is NOT broken — the vendor MakeCode
`nezhaV2.readSpeed(M1)` returns sensible, increasing values when run in isolation
(`start(M1, pwm)` → `pause 500ms` → `readSpeed`). Our C++ `Motor::readSpeedRaw`
(`source/hal/Motor.cpp`) uses the SAME frame/delays/parse, yet returns a stuck
~30-33 mm/s regardless of speed — so the bug is in OUR read CONTEXT, not the chip.
Prime suspect: tight-loop I2C interleaving — `MotorController::tick()` hammers the
Nezha (0x10) every 20 ms with 0x46 enc ×2, 0x47 speed ×2, 0x60 write ×2 (plus OTOS/
line/color on the bus), so 0x47 is read in a bad window (stale / before the chip's
speed estimate settles). Debug against the vendor program as the reference oracle
on hardware: replicate start→pause→readSpeed to confirm a good read, then find what
in the loop breaks it (read timing/order, settle delay, post-write window, parse).
FIX the read so chip velocity tracks actual speed; use **chip velocity as the
feedback source with encoder-delta as fallback** (the implausibility gate should
reject a stuck/implausible chip value — both too-high AND too-low — not be removed).
Keep `getActualVelocity()`/`getVelocitySourceFlags()` API.
AC: `tests/test_readspeed_and_get_vel.py` updated; bench: `vel=`/`GET VEL` chip
source scales with commanded speed (matches the vendor readSpeed curve), idle ≈ 0,
and falls back to encoder-delta only on a genuine bad read.

**T05 — Refresh encoders + odometry predict every tick (fix idle staleness)**
(deps T04 recommended). `source/control/DriveController.cpp` tick() (~219-226):
move `_mc.tick()` + `getEncoderPositions` + `_odo.predict()` out of the
`if (_mode != IDLE)` guard so caches refresh at rest (motor commands stay gated to
non-idle; OTOS `correct()` already runs at idle). AC: `SNAP` at rest after motion is
current; hand-push at idle updates `enc=`/`pose=`; no motor twitch at idle.

**T06 — Known-good compiled defaults + per-direction turn gain applied**
(deps T01; land late). `source/types/Config.h defaultRobotConfig()`: `trackwidthMm`
120→**126**; `otosLinearScale`=1.05, `otosAngularScale`=0.987; turn gains
(CCW 1.0 / CW 1.17, offsets 0) and `rotationalSlip` 0.74. Do **not** change
`mmPerDegL/R`. Apply the per-direction gain/offset in the turn/rotate path
(turn-in-place gate + go-to pre-rotate in `DriveController`). Update tests that
hard-code trackwidth 120. AC: `GET tw`=126, scalars at defaults; turns symmetric
within tolerance on bench.

**T07 — OTOS mounting offset support** (deps T01). Prior system applies a
mounting offset+yaw+upside-down transform (`poseRobotFrame`) that this firmware
lacks. Add config fields (`odomOffX/Y`, `odomYawDeg`, `odomUpsideDown`) + apply the
transform when reading OTOS into odometry (`DriveController::correct()` / OtosSensor),
plus an `OO`-equivalent SET path. AC: with a nonzero offset, fused pose reflects the
robot-center frame. (Scope check: for nezha bots offsets are ~0 — keep minimal/correct.)

**T08 — Per-robot config schema + data dir + loader finalize** (host; no firmware
dep). Port `robot_config.schema.json` and create `data/robots/<robot>.json`
(e.g. `tovez.json`) seeded from known-good values (trackwidth 126, otos scalars,
mmPerDeg 0.487/0.481, wheel geometry). Finalize `host/robot_radio/config/robot_config.py`
(`get_robot_config()` via `ROBOT_CONFIG` env or `data/robots/active_robot.json`).
Add robot **matching by v2 `ID`** (device announcement name / serial) over the relay.
AC: host loads the active robot config and matches the connected robot.

**T09 — Connect-time apply rewritten to v2** (deps T08, and T01 for new keys).
`host/robot_radio/io/cli.py::_push_calibration()`: replace dead verbs —
`KML/KMR`→`SET ml/mr`, add `SET tw`, keep `OL`/`OA`/`OI`, route mounting offset to the
T07 `OO`-equivalent (or drop if not needed), remove `OK`. Use the ack-gated blocking
send (relay drops back-to-back writes). AC: host unit test asserts v2 verbs emitted
(no `KML`/`OO`/`OK`); bench: post-connect `GET` matches the robot JSON.

**T10 — Calibration scripts rewritten to v2 + relay** (deps T03,T04,T05,T08,T09).
Port/adapt `calibrate_linear.py` + `calibrate_angular.py` into `host/` against the v2
protocol and the relay data plane, using **OTOS (and overhead camera) as ground
truth**: drive measured distances / spins, compute & write back `otos_linear_scale`,
`otos_angular_scale`, per-direction turn gains, and verify `mmPerDeg`. Replace
`sensors/odom_tracker.py` `parse_so` with TLM parsing; update `sensors/calibration.py`
docs. AC: scripts run end-to-end over the relay and emit recommended config values;
update `data/robots/<robot>.json`.

**T11 — End-to-end bench verification + record calibration** (deps all; stakeholder-run).
Run the verification plan below; record the measured calibration into the per-robot
JSON; confirm ship-shape.

## Key files

- Firmware: `source/types/Config.h`, `source/app/CommandProcessor.cpp`,
  `source/robot/Robot.cpp`, `source/control/MotorController.cpp`,
  `source/control/DriveController.cpp`, `source/hal/OtosSensor.cpp`,
  `source/control/Odometry.*`.
- Host: `host/robot_radio/config/robot_config.py` (present),
  `host/robot_radio/io/cli.py` (`_push_calibration`), `host/robot_radio/io/calibrate.py`,
  `host/robot_radio/sensors/{odom_tracker,calibration}.py`, new
  `data/robots/<robot>.json` + `robot_config.schema.json` + `active_robot.json`.
- Prior source of truth to port from:
  `/Volumes/Proj/proj/league-projects/scratch/radio-robot/` (`src/otos.ts`,
  `robot_radio/config/robot_config.py`, `test/calibrate/calibrate_{linear,angular}.py`,
  `data/robots/nezha-1.json`, `data/robots/robot_config.schema.json`).

## Decisions (locked)

- **Velocity:** FIX the C++ readSpeed (the chip 0x47 works per vendor MakeCode); use
  chip velocity as feedback with encoder-delta fallback. Do NOT demote the chip — our
  read context (tight-loop I2C interleaving) is the bug. Vendor program is the oracle. (T04)
- **Scope:** the full system in Sprint 12 (firmware fixes + defaults + turn gain +
  config-system port + calibrate-script rewrite).
- `mmPerDegL/R` (0.487/0.481) unchanged — already known-good.
- OTOS LSB constants (0.305 mm, 0.005493°) are correct — not the bug.

## Verification (end-to-end, stakeholder, on playfield)

After a **clean build** + reflash to **robot enum 2**, driving over the relay (RAW250):
1. **Connect-time apply:** `GET tw/ml/mr`, `OL`, `OA` match the active robot JSON
   (tw=126, OL≈+50, OA≈−13) with no manual commands.
2. **Distance:** `D 1000`; tape-measure actual within a few %; `pose=` x≈1000 mm.
3. **Straight:** forward over a marked line; minimal lateral drift (correct velocity PID).
4. **Turns:** in-place 90°/180° CCW & CW vs OTOS/camera; symmetric within tolerance.
5. **Pose tracking:** square / out-and-back; fused `pose=` vs camera ground truth;
   `OP` (raw OTOS) cross-check.
6. **Idle freshness:** stop, `SNAP` repeatedly, hand-push → `enc=`/`pose=` update.
7. **Velocity:** speed sweep → `vel=`/`GET VEL` scales with command (not pinned ~30).
8. **go-to:** `G x y` to targets; arrival within `arriveTol`, final pose matches camera.

## Risks

- **Stale host port:** the partially-ported host calibration/apply code speaks the
  dead pre-v2 protocol — T09/T10 must rewrite, not reuse, the protocol-touching parts.
- **Default-change test breakage:** trackwidth 120→126 may break tests with hard-coded
  120 (`test_odometry_midpoint.py`, `test_otos_fusion.py`) — T06 updates them.
- **Per-direction turn gain** is a new firmware feature on a closed-loop (OTOS-corrected)
  turn path, not the prior open-loop model — verify it composes with go-to pre-rotate.
- **Build hygiene:** all firmware tickets need `--clean` (stale incremental builds have
  produced false bench results before).
- **OTOS mounting offset (T07):** confirm whether nezha offsets are nonzero; keep minimal
  if ~0.

## Process note

This is a CLASI sprint: on approval I'll `create_sprint("012", …)`, `detail_sprint`,
dispatch the sprint-planner to author architecture-update/usecases + create these
tickets, acquire the lock, and execute ticket-by-ticket with clean builds + reflash,
then the stakeholder runs T11 on the playfield before close.
