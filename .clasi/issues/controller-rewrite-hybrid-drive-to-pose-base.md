---
status: pending
---

# Controller Rewrite: Hybrid Drive-to-Pose Base

## Context

Today the **host** closes every control loop with the camera in the loop: Python
`Navigator` (`robot_radio/nav/navigator.py`) reads an AprilTag pose each frame,
computes wheel speeds, and streams `S<L><R>` commands to a largely-passive robot.
Wheel control on the robot is a **position/ratio cross-coupling PID** (`src/nezha.ts`
`ratioPid`/`driveTick`) — it equalizes *distance* between wheels, never controls
*velocity*. The Nezha's `readSpeed` velocity command (I2C `0x47`) is implemented but
stubbed to return 0 and unused. There is no unified robot-state object and no
dead-reckoning pose estimate on the robot.

We are inverting this. The robot becomes a **self-contained drive-to-pose base**:
it controls wheel *velocity* in closed loop, runs its own kinematic odometry, fuses
a synthesized world pose, and drives itself to a commanded pose. The host becomes a
pure **executive** that issues pose goals and periodically corrects the robot's pose
estimate from the camera (camera *never* in the fast loop — localization/correction
only, per standing guidance). This is a substantial, multi-phase change.

**Decisions locked with the stakeholder:**
- **Hybrid locus** — wheel-velocity PID, kinematic odometry, fusion, *and a simple
  pose controller* live in firmware; the heavier, genuinely-pluggable trajectory
  *planners* live on the host and emit pose-waypoint sequences.
- **Complementary fusion now, EKF later** — ship a weighted encoder+OTOS blend first;
  swap an EKF in behind the same interface later.
- **Pluggable = compile-in + serial select** on the robot (a fixed menu of pose
  controllers chosen at runtime by a config command, no reflash); **real Python
  plugins** on the host for path planners.
- This plan covers **both** the greenfield target design and the migration.

---

## Hardware realities that shape the design (read first)

1. **I2C read penalty.** Every Nezha `readAngle` (0x46) and `readSpeed` (0x47) call
   does `delayMs(4)` before and after → **~8 ms each**, so a 2-wheel encoder read or
   speed read is **~16 ms**. OTOS pose/velocity bursts are cheap (~2 ms). A single
   20 ms tick cannot do all reads. **Firmware must become a multi-rate cooperative
   scheduler.**
2. **"50–100 Hz odometry" is not achievable from encoders.** The 16 ms encoder read
   caps encoder odometry at ~12.5–25 Hz when sharing the bus. The *fast* pose signal
   comes from cheap OTOS velocity reads (25 Hz). The fused pose updates at the
   encoder/fusion rate (~12.5 Hz) with velocity refreshed at 25 Hz. **Flagged so
   expectations are set.**
3. **`readSpeed` (0x47) is a stub.** The entire velocity-PID rests on it. **Phase 0
   validates it empirically before any PID is written**; fallback is to derive wheel
   velocity from encoder deltas (drops the 16 ms speed read entirely).
4. **Speed quantization.** `readSpeed` returns `floor(raw/3.6)*0.01` laps/s ≈
   **2.54 mm/s per LSB**. Expect steady-state ripple; do not promise smooth control
   below ~30 mm/s. Mitigate with FF-dominant tuning + a low-speed deadband.
5. **Heading-sign seam (must fix once).** Firmware/OTOS/camera are **CCW-positive**;
   host `robot_radio/kinematics/differential_drive.py` documents **CW-positive rad**.
   Standardize the new stack on **CCW-positive radians internally** and fix the host
   negation. Add a `BV+0+1000` "spin CCW" sanity check before trusting any controller.
6. **3 poses don't fit one 19-char radio packet.** State queries reply on multiple
   short lines; the fast path streams only the fused pose.

---

# PART 1 — Target (greenfield) design

## Firmware module layout (Static TypeScript, namespaces only)

| File | Namespace | Responsibility |
|---|---|---|
| `src/nezha-ext.ts` | `nezhaV2` | unchanged vendor I2C shim (0x46/0x47/0x60) |
| `src/otos.ts` | `otos` | unchanged OTOS driver |
| `src/pid.ts` | `PidController` | unchanged PID class (reused by velocity PID) |
| `src/nezha.ts` → thin HAL | `robot` | keep `motorsPwm`, `stopMotors`, raw enc/speed reads, geometry consts, gripper/ports/sensors. **Remove** `ratioPid`, `driveTick`, `computeArc`, `startDrive*` |
| `src/state.ts` (new) | `rstate` | unified robot-state object + accessors (no logic) |
| `src/velpid.ts` (new) | `velctl` | per-wheel velocity PID + body (v,ω) inverse kinematics |
| `src/odom.ts` (new) | `odom` | encoder forward kinematics + OTOS read + complementary fusion + world-correction |
| `src/posectl.ts` (new) | `posectl` | pose-controller plugin registry + selection |
| `src/sched.ts` (new) | `sched` | multi-rate cooperative scheduler (new master loop body) |
| `src/command.ts` | `command` | keep parser; rewrite handlers; `tick()` → `sched.tick()` |
| `src/main.ts` | — | keep radio/serial wiring; while-loop calls `sched.tick()` |

**Every new file must be added to `pxt.json` `files`** or PXT silently ignores it.

### Unified robot-state object (`rstate`)
Single module-level object. All angles **rad CCW+**, positions **mm**, vel **mm/s**,
yaw rate **rad/s**.
- `synth {x,y,yaw,v,omega}` — fused, authoritative
- `enc {x,y,yaw,v,omega}` — encoder-only (diagnostic)
- `otos {x,y,yaw,v,omega}` — OTOS-only (diagnostic)
- `wheel {vL,vR (measured), spL,spR (setpoints), pwmL,pwmR}`
- `cmd {mode: IDLE|VEL|POSE, v,omega, tx,ty,tyaw, termV,termOmega,headingFree, ctrlId, done}`
- `health {lastSpeedReadMs, lastEncReadMs, lastOtosReadMs, i2cErrCount}`

## Multi-rate scheduler

`main.ts` free-runs `while(true){ sched.tick(); }` (no top-level pause). Each task
owns `nextDueMs` + period and self-gates on `input.runningTime()`. **At most one
heavy (16 ms) read per tick** (a `heavyBudgetUsed` flag) so two can never stack.

| Loop | Period | I2C cost | Notes |
|---|---|---|---|
| Velocity PID | 40 ms (25 Hz) | speed 16 + PWM 2 = 18 ms | rate capped by the 16 ms speed read |
| Encoder odometry | 80 ms (12.5 Hz) | 16 ms | runs on opposite phase from velocity PID |
| OTOS pose+vel | 40 ms (25 Hz) | 4 ms | cheap; fast velocity source |
| Fusion | 80 ms | 0 | runs right after each encoder update |
| Pose controller | 40 ms | 0 | consumes `synth`, writes velocity setpoints |
| Streaming reports | ≥120 ms | 0 | each radio reply costs ~5 ms settle |

Phasing: velocity-PID at t≡0 (mod 40), encoder at t≡40 (mod 80) → at most one 16 ms
read per 40 ms window. Macro-cycle (80 ms) ≈ 60 ms I2C duty, fits with headroom.

## Wheel velocity PID
- **Setpoint** `sp{L,R}_mms`; **feedback** `readSpeed → laps/s × wheelCircMm` where
  `wheelCircMm = π·80.77 ≈ 253.7`; apply per-wheel forward sign.
- **Output** PWM% via 0x60, clamp ±100.
- **PI, no D** (speed feedback too quantized for derivative — same lesson as TN).
  FF-dominant: `pwm_ff = kFF·|sp| + sign(sp)·deadband` (deadband = config
  `motor_deadband` = 12); `pwm = clamp(pwm_ff + kP·err + I)`, anti-windup on saturation.
- Low-speed deadband: `|sp| < minWheelMms` (~20 mm/s) → command 0.
- Tunables: `K+VP/VI/VF/VD`.

## Body (v,ω) kinematics  (b = trackwidth = 126 mm)
- **Inverse:** `spL = v − ω·b/2`, `spR = v + ω·b/2` (verify CCW+ → right faster).
- **Forward odometry** per encoder tick (use raw encoder degrees, not rounded mm):
  `dL=ΔencL·mmPerDegL`, `dR=ΔencR·mmPerDegR`, `dC=(dL+dR)/2`, `dθ=(dR−dL)/b`;
  midpoint integrate: `θmid=yaw+dθ/2; x+=dC·cosθmid; y+=dC·sinθmid; yaw=wrapPi(yaw+dθ)`;
  `v=dC/dt; omega=dθ/dt`.

## Complementary fusion
- `synth.x = wPos·otos.x + (1−wPos)·enc.x` (same for y); `synth.yaw =` shortest-arc
  blend of otos/enc by `wYaw`. Defaults `wPos=0.7, wYaw=0.8` (tunable `K+WP/WY`).
- `synth.v/omega` from OTOS velocity (fast, cheap); fall back to encoder if OTOS down.
- **Camera world-correction** (`WC` command): hard-snap `enc`, `otos`
  (`otos.setPositionRaw`), and `synth` to the corrected pose. Rate-limited; reject
  corrections > N mm from current synth unless forced.
- **Deferred to EKF phase:** covariance, slip detection, camera-latency replay.
  EKF later replaces the blend behind `odom.fuse()` — no protocol change.

## Pose-controller plugin mechanism
Interface (array of function pointers indexed by int):
`PoseCtrlFn(s: SynthPose, tgt: Target, dt, out: VW) -> done: boolean`. Registry +
`NAMES` in `posectl.ts`; `posectl.select(id)` (validated), `posectl.step(dt)`. Ship:
1. **`propPose` (id 0)** — proportional: spin-in-place if `|headingErr|>gate`, else
   `v=clamp(kV·dist), omega=kOmega·headErr`; done on `dist<posTol` & (headingFree or
   `yawErr<yawTol`). Direct successor to `ChaseController`+`TN`.
2. **`lyapunov` (id 1)** — polar unicycle regulator (ρ,α,β); converges to full pose
   smoothly, no separate final-turn phase. General-purpose default.
3. **`trapezoid` (id 2)** — accel-limited profile; ends at `termV` for waypoint
   chaining (the "assume terminal velocity state" variant).

Selection: `PC<id>`. Tunables `K+PV/PO/PT/PY`.

## Command protocol additions (≤19 chars, sign-prefix ints; ω as milli-rad/s)

| Command | Format | Meaning |
|---|---|---|
| Set body velocity | `BV<v_mms><w_mrps>` | primary live-drive primitive; `mode=VEL`, watchdog like `S` |
| Drive to pose | `DP<x><y><yaw_deg>` | `mode=POSE`, headingFree=false |
| Drive to pose + terminal vel | `DT<x><y><yaw><tv_mms>` | ends at terminal velocity (chaining) |
| Drive to location | `DL<x><y>` | headingFree=true |
| Select pose controller | `PC<id>` | choose compiled-in controller |
| Inject world-correction | `WC<x><y><yaw_deg>` | hard-snap fusion (augments/replaces `SI`) |
| Query state | `QF/QE/QO` → reply per pose, `QV` → synth vel | three short lines + vel; fast path streams `QF` only |
| Tunables | `K+VP/VI/VF/VD`, `K+WP/WY`, `K+PV/PO/PT/PY` | extend existing `K` handler |

`BV` supersedes raw `S<L><R>` as the live primitive (`S` kept as a thin legacy wrapper).
`X`/`R`/`O*`/`K`/`P*` retained.

## Host executive
- New `robot_radio/planners/` mirroring the existing `controllers/` registry pattern:
  `base.py` (`Planner.plan(start, goal, ctx) -> list[PoseWaypoint]`), `__init__.py`
  `PLANNERS` dict. `PoseWaypoint = (x,y,yaw|None, terminal_v|None)`. Concrete:
  `straight_line`, `bezier` (reuse `path/builder.py`), `arc/dubins`, `via_tags`.
- Executive loop streams `DT` per intermediate waypoint (non-stop chaining), `DP` for
  the final; polls `QF` at low rate to advance/re-plan. **Not a fast loop.**
- Camera-correction thread: read aprilcam daemon pose at ~2–5 Hz, push `WC` on fresh
  robot-tag fixes (reuse `Odometry` staleness `age<0.3`). Never gates motion.
- MCP tools rebuilt on drive-to-pose: `goto`→`DP`; `navigate_to`→`DL`/planner→`DT…`;
  `visit_tags`→per-tag `DP/DL`; `follow_path`/`follow_pose_path`→planner→`DT…,DP`.
  `Navigator.navigate/approach/adaptive_turn` demoted to planner helpers (no
  frame-by-frame wheel driving).

---

# PART 2 — Migration (phased; robot stays drivable; each phase field-tested)

Legacy `S/T/D/G/TN` path stays alive until its replacement is validated.

- **Phase 0 — De-risk `readSpeed` 0x47 (BLOCKING).** Implement `robot.readSpeedLaps{L,R}`,
  add temporary `QSPD` stream, drive at known PWM, log `(pwm, speed→mm/s,
  encoder-derived mm/s)`. Verify monotonicity/sign/latency/quantization. **If
  unreliable, switch to encoder-derived velocity** (drop the 16 ms speed read; derive
  velocity in the odometry loop). *Riskiest single assumption.*
- **Phase 1 — Scaffolding, no behavior change.** Add `rstate` + `sched`; move existing
  `command.tick()` body into one 20 ms scheduler task. Regression test: all legacy
  commands still work.
- **Phase 2 — Multi-rate odometry + 3-pose state (read-only).** Add encoder/OTOS/fusion
  loops populating `enc/otos/synth`; add `QF/QE/QO/QV`. Drive with *existing* ratio-PID;
  compare three poses to camera on a known square/line; tune `wPos/wYaw`.
- **Phase 3 — Velocity PID + body (v,ω) behind `BV`.** Add `velctl`; old `S`/ratio-PID
  untouched. Test `BV` straight lines + constant-ω arcs; tune `KVP/KVI/KVF`.
- **Phase 4 — Pose controller (`DP/DL/DT/PC`).** Add `posectl` (propPose first, then
  lyapunov). Resolve heading-sign seam *before* this; run `BV+0+1000` spin sanity check.
  Test `DP` to a marked pose from several starts; compare controllers via `PC`. Legacy
  `G/TN` remain as fallback.
- **Phase 5 — Camera world-correction (`WC`).** Add host corrector thread. Test long
  loop with/without `WC`; measure drift reduction.
- **Phase 6 — Host executive + planners.** Build `planners/`, rewire MCP tools onto
  `DP/DT/DL`, demote `Navigator`, fix `differential_drive.py` sign. Test `follow_path`
  + `visit_tags` end-to-end with correction active.
- **Phase 7 — Retire legacy.** Delete ratio-PID/`computeArc`/`G`/`TN`/host
  `ChaseController`/crawl; re-point `S/goto/turn_to` to `BV/DP` wrappers; default
  trackwidth 120→126.
- **Phase 8 (LATER) — EKF.** Replace the complementary blend inside `odom.fuse()` with
  an EKF + camera-latency replay. No protocol change.

### Riskiest steps & mitigations
1. `readSpeed` sanity (Phase 0) — validate empirically; encoder-derived-velocity fallback ready.
2. I2C over-subscription — staggered phasing + one-heavy-read-per-tick; fallback drops the speed read (frees 16 ms).
3. Heading-sign seam — pin CCW+ rad everywhere, fix host negation, spin sanity check before Phase 4.
4. 3-pose packet overflow — multi-line `QF/QE/QO/QV`; stream fused pose only on fast path.
5. Low-speed ripple from 2.54 mm/s quantization — FF-dominant PID + `minWheelMms` deadband.

---

## Critical files
- Firmware: `src/main.ts`, `src/command.ts` (parser ~250–282, tick dispatch ~1087–1241,
  TN turn), `src/nezha.ts` (`ratioPid` 253, `driveTick` 562–592, enc/mm conv 100–120,
  `computeArc` 269–292, geometry 243/252), `src/nezha-ext.ts` (`readAngle` 208–240,
  `readSpeed`), `src/otos.ts`, `src/pid.ts`. New: `state.ts`, `velpid.ts`, `odom.ts`,
  `posectl.ts`, `sched.ts` (+ register all in `pxt.json`).
- Host: `robot_radio/robot/protocol.py`, `robot_radio/robot/nezha.py`,
  `robot_radio/nav/navigator.py`, `robot_radio/sensors/odometry.py`,
  `robot_radio/controllers/__init__.py` (registry pattern to mirror),
  `robot_radio/kinematics/differential_drive.py` (sign fix),
  `robot_radio/config/robot_config.py` + `data/robots/nezha-1.json` (trackwidth 126,
  wheel_diameter 80.77, scalars). New: `robot_radio/planners/`.

## Verification (end-to-end on the field — required; stakeholder sets pass criterion)
Each phase has a concrete field test above. Common rig: robot on field, camera + OTOS
available, commands via relay (per memory: relay is reliable; robot direct-USB only at
the bench). Per phase:
- **Phase 0:** log + plot `readSpeed` vs PWM vs encoder-derived velocity; confirm sane.
- **Phase 2:** drive a known square; overlay `enc/otos/synth` vs camera; tune weights.
- **Phase 3:** `BV` lines/arcs; commanded vs measured `synth.v/omega` within tolerance.
- **Phase 4:** `DP` to a marked pose from ≥3 starts; lands within `posTol/yawTol`;
  compare `propPose` vs `lyapunov` via `PC`.
- **Phase 5:** long loop, drift vs camera with/without `WC`.
- **Phase 6:** MCP `follow_path`/`visit_tags` complete with correction active.

This is a CLASI project — execute via the SE process (sprint planning → tickets →
programmer dispatch), not as one monolithic change. The phases above are the natural
ticket/sprint boundaries.
