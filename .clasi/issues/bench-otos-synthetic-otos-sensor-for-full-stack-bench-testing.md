---
status: pending
---

# Bench OTOS — synthetic OTOS sensor for full-stack bench testing

## Context

We need to validate the full firmware stack (motors, encoders, color/line sensors,
EKF pose fusion, motion control, navigation, telemetry) **on the bench** — robot on
a stand with the wheels spinning freely. The problem: the OTOS is an optical-flow
sensor that tracks the ground, so on a stand it sees **no motion** and reports a
frozen pose. That frozen pose poisons the EKF and makes every motion command that
depends on pose behave wrongly, so the bench can't exercise the navigation/fusion
stack at all.

A **Bench OTOS** solves this: a debug `IOtosSensor` implementation that runs on real
hardware and *synthesizes* the robot's pose by integrating the **commanded** wheel
velocities into an ideal "this is where we'd be if everything worked perfectly" pose,
then returns that pose with a small injected error. With it active, the whole stack
believes the robot is driving as commanded, so we can validate fusion/nav on the bench.

This is also the tool that unblocks the hardware validation deferred from the 025–028
sprint run (smoke ritual, EKF behavior on the field).

**Key prior art:** this exact integrator already exists in the **sim** —
`MockOtosSensor`'s "sim model" (`source/hal/mock/MockOtosSensor.cpp:59-89`) integrates
wheel velocity + Gaussian noise into a pose via midpoint-arc kinematics, and
`ExactPoseTracker` (`source/hal/mock/MockHAL.h:19-35`) is the noiseless oracle. But
`source/hal/mock/` is **excluded from the firmware build**
(`CMakeLists.txt` — `list(FILTER SOURCE_FILES EXCLUDE REGEX ".*/hal/mock/.*")`), so
none of it compiles to the device. This feature re-homes that proven math into a
firmware-compiled class.

## Decisions (confirmed with stakeholder)

- **Selection:** swappable live via a `DBG OTOS BENCH 0|1` command (volatile, defaults
  off, lost on reboot — can never persist into a fielded robot).
- **Truth source:** integrate the **commanded** wheel velocity
  (`state.commands.tgtLMms/tgtRMms`) — the ideal noise-free pose.
- **Error model:** per-tick Gaussian noise **plus** a slow yaw-drift/bias term, to
  mimic the real OTOS's characteristic drift (makes EKF correction visibly matter).
  Sigmas + drift passed as DBG args (volatile, bench-scoped).
- **Observability:** a `DBG OTOS` query that reports **ideal vs errored-OTOS vs
  EKF-fused** pose in one reply (no TLM wire-format change; the errored OTOS pose and
  fused pose already ride existing `otos=`/`pose=` TLM fields).

## Approach

A new concrete `BenchOtosSensor : public IOtosSensor`, fed the commanded velocity once
per control tick, swapped in front of the real `OtosSensor` by a pointer in `NezhaHAL`.
The `IOtosSensor` interface and the production `OtosSensor` path are **untouched**.

### Files to create
- `source/hal/BenchOtosSensor.h` / `.cpp` — concrete `IOtosSensor`. Lives directly
  under `source/hal/` (auto-globbed into the firmware build), **not** under `hal/mock/`.
  - Implements all `IOtosSensor` virtuals. `readTransformed` returns the errored pose;
    `readVelocityTransformed`/`readAccelTransformed` return values derived from the same
    noisy segment (consistent position/velocity channels, as the mock does).
  - `begin()`/`is_initialized()`/`readStatus(out=0)`/`lastReadOk()` hardcode **valid**
    so `Robot::otosCorrect`'s D9 validity gate fuses the bench pose.
  - Concrete-only (not on the interface): `tick(velL, velR, trackwidthMm, dt_ms)`,
    `enable(bool)`, `enabled()`, `setNoise(linSigma, yawSigma)`, `setDrift(yawRate)`,
    noiseless-oracle accessors `idealX/Y/H()`, `reset()`.
  - Integrator math ported from `MockOtosSensor::tick` + `ExactPoseTracker::update`;
    integrates **two** accumulators each tick — `_ideal*` (noiseless oracle) and
    `_odom*` (= ideal + Gaussian noise + accumulated yaw drift).
  - PRNG: use CODAL's `microbit_random()`/`microbit_seed_random()` (no `<random>`/
    `std::mt19937` — too heavy for nRF52); approximate-Gaussian via sum-of-uniforms.
    Under `#ifdef HOST_BUILD`, deterministic LCG fallback for reproducible sim tests.
  - Reuse `source/control/BodyKinematics.h::forward()` for the wheel→body map to keep
    one source of truth with `Odometry::predict`.

### Files to modify
- `source/hal/NezhaHAL.h` / `.cpp` — add `BenchOtosSensor _benchOtos;` value member and
  an `IOtosSensor* _activeOtos = &_otos;`. `otos()` returns `*_activeOtos`. Add
  `setOtosBench(bool)` (repoint the active pointer) and `benchOtos()` accessor.
  `begin()` still probes the real `_otos` exactly as today; `_benchOtos.begin()` is a
  pure no-op (no I2C). The boot-time real-OTOS detection/retry in `main.cpp` is
  unaffected — bench mode only diverts `otos()` afterward.
- `source/robot/Robot.h` / `.cpp` — add `benchOtosTick(now_ms)` (computes dt via signed
  delta, reads `state.commands.tgt*` + `config.trackwidthMm`, calls the bench sensor's
  `tick`). Route `otosCorrect`'s OTOS reads through `hal.otos()` (the active pointer)
  rather than the ctor-cached `otos` ref, so the runtime swap takes effect live
  (see Implementation note). Add the bench accessor for the DBG query.
- `source/control/LoopTickOnce.cpp` — call `robot.benchOtosTick(now)` immediately
  **before** the OTOS block (`~line 100`, before `robot.otosCorrect`). Runs in both
  firmware (`LoopScheduler::run_blocks`) and sim (`sim_api.cpp` calls `loopTickOnce`)
  automatically. `controlCollectSplitPhase` (which refreshes `state.commands.tgt*`)
  already runs earlier in the tick, so integrate-then-read ordering holds.
- `source/app/DebugCommandable.h` / `.cpp` — add a `DBG OTOS BENCH` command modeled on
  `DBG WEDGE` (`parseDbgWedge`/`handleDbgWedge`, optional integer-milli args; register
  longest-prefix-first in `getCommands()`):
  - `DBG OTOS BENCH 1 [linSigma] [yawSigma] [driftDegPerSec]` → enable + set error params.
  - `DBG OTOS BENCH 0` → disable (back to real OTOS).
  - `DBG OTOS` (no args) → reply `OK dbg ideal=x,y,h otos=x,y,h fused=x,y,h err=dx,dy,dh`
    (ideal from the bench sensor, errored from `state.inputs.otos*`, fused from the EKF
    pose). `DbgCtx` already carries `Robot* robot` — no struct change needed.

### Not touched
`IOtosSensor.h` (interface unchanged), `OtosSensor.*` (production path unchanged),
`CMakeLists.txt` (`source/hal/` auto-globbed), `Config.h`/`ConfigRegistry`/
`gen_default_config.py` (no persisted config — error params are volatile DBG args),
the `STREAM`/TLM field bitmask (DBG query instead of a new TLM field).

## Reuse references
- Integrator + noise math: `source/hal/mock/MockOtosSensor.cpp:59-89` (port, don't include)
- Noiseless oracle: `source/hal/mock/MockHAL.h:19-35` (`ExactPoseTracker::update`)
- Forward kinematics: `source/control/BodyKinematics.h::forward`; midpoint integration in `source/control/Odometry.cpp::predict`
- Validity stubs: `source/hal/mock/MockOtosSensor.h:32-33` (always-valid `readStatus`/`lastReadOk`)
- DBG command pattern + optional-int args: `source/app/DebugCommandable.cpp` (`parseDbgWedge`/`handleDbgWedge`, registration ~`:449-458`)
- On-hardware DBG bench-harness precedent: `source/app/WedgeTest.{h,cpp}`
- Commanded velocity source: `source/control/RobotState.h` (`MotorCommands::tgtLMms/tgtRMms`)
- PRNG: CODAL `microbit_random()` / `microbit_seed_random()`

## Implementation note (reference reseating)
`Robot` binds `IOtosSensor& otos` once in its constructor (`Robot.cpp` `otos(hal.otos())`),
so a C++ reference can't be re-seated by the runtime pointer-swap. Resolution: have
`otosCorrect` and `benchOtosTick` read through `hal.otos()` (the live active pointer)
instead of the cached `otos` ref — a tiny indirection isolated to those two methods;
all other read sites keep the cached ref. (Alternative considered and rejected: a single
dual-mode sensor object — re-merges the two classes and pollutes the production class
with debug state.)

## Verification

**Host sim (automated):** add a focused `host_tests` unit test driving a known commanded
profile and asserting the ported `BenchOtosSensor` integrator matches `ExactPoseTracker`
to within the injected noise band, and matches the oracle exactly with sigma=drift=0.
Run the full `host_tests/` + `host/tests/` suite — the production OTOS path is untouched,
so existing OTOS tests must pass unchanged.

**Bench (hardware, stakeholder-run):**
1. Robot on a stand, wheels free. Flash, boot (real OTOS detected as today). Send
   `DBG OTOS BENCH 1`.
2. `STREAM 100 fields=enc,pose,otos,twist,vel`, issue a drive (`D 500` / `G x y`).
   Confirm `otos=` (bench errored pose) and `pose=` (EKF-fused) both advance and track,
   and `vel=`/`twist=` are non-zero — the stack believes it is moving though wheels spin free.
3. Poll `DBG OTOS`: confirm `ideal` ≈ commanded integral, `err` small/bounded, `fused`
   sits between ideal and errored (EKF doing its job); `ekf_rej` stays low.
4. Drive a closed square; confirm fused pose closes the loop within tolerance.
5. `DBG OTOS BENCH 0` → immediate return to real-OTOS behavior (otos pose stops advancing
   on the stand). Reboot → confirm boots in real mode (volatile toggle).
6. Safety check: an open-ended `VW` without keepalives still fires `EVT safety_stop`
   (watchdog unaffected); self-terminating `D`/`T` complete without keepalives. (Operator
   guidance: on the bench, prefer self-terminating commands, stream `+` keepalives, or
   `SAFE off`.)

## Out of scope
- No change to the production OTOS path, the `IOtosSensor` interface, safety/watchdog
  behavior, or persisted config.
- No streamed TLM field for the oracle (DBG query only) — can be added later if live
  plotting is wanted.
