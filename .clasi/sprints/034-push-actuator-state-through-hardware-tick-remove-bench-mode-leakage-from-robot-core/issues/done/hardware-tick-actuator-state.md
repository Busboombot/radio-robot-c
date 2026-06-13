---
status: done
sprint: '034'
tickets:
- 034-001
- 034-002
- 034-003
- 034-004
- 034-005
- 034-006
---

# Refactor — `Hardware::tick` carries actuator command state; kill `benchOtosTick` and robot-visible bench mode

Stakeholder design direction (Eric, 2026-06-12).

## Problem

The Bench OTOS integration leaks implementation into the firmware core:

- `Robot::benchOtosTick()` / `Robot::isBenchOtosActive()` downcast `hal` to
  `NezhaHAL*`, reach into `BenchOtosSensor` concretely, and carry
  `#ifndef HOST_BUILD` guards (`Robot.cpp:381-426`).
- `handleDbgOtosBench` / `handleDbgOtos` do the same downcast
  (`DebugCommandable.cpp:423-507`).
- `loopTickOnce` calls `robot.benchOtosTick(now)` (`LoopTickOnce.cpp:99`).

To be precise about the goal: the codebase DOES contain a bench mode — it is
selected by build defines. What must not happen is the robot core *code*
referencing it. After the defines are resolved, `Robot`, `Odometry`,
`MotionController`, etc. compile to the same source either way, with zero
mention of bench/sim anywhere in them; the variation lives entirely in the
defines at the two seams listed below (hardware creation, the one `hal.tick`
call). Today that's violated: Robot.cpp itself names `NezhaHAL`,
`BenchOtosSensor`, and carries the build guards. A second `IOtosSensor`
*implementation* is fine — mode flags, downcasts, and build guards inside
the robot core are not. The synthetic plant needs exactly two inputs per tick:
the time and the commanded actuator state. Both already exist
(`RobotState.h:23-32` — `MotorCommands` is the declarative actuator state),
and the seam already exists (`Hardware::tick` is in the interface;
`NezhaHAL::tick` is currently a no-op).

## Design

Three build configurations:

| Config             | Hardware created                  | Loop calls            |
|--------------------|-----------------------------------|-----------------------|
| `production`       | NezhaHAL (real only)              | `tick(now)` — no cmd state |
| `bench debug`      | NezhaHAL (+ bench OTOS available) | `tick(now, cmds)`     |
| `simulation debug` | MockHAL (host build)              | `tick(now, cmds)`     |

Simulation debug needs the command state MOST: MockHAL integrates the entire
plant — motor model → encoders, and OTOS from wheel velocities. Today it
scavenges that input by capturing `setSpeed` writes inside `MockMotor` and
having the harness call `hal.tick(now)` directly (`sim_api.cpp:183, 492`)
plus an explicit bench-sensor poke (`sim_api.cpp:685`) — workarounds for the
same missing parameter. All of those collapse into the one loop call.

Encapsulation rule: passing the command state to hardware is a deliberate,
debug-only break of encapsulation — the test seam a synthetic plant needs
(it has no physics to receive the commands through). Production does not
pass command state to hardware; the interface stays clean there.

Exactly TWO variation points, each a single build-define switch:

1. **Hardware creation** — which HAL set is instantiated (this seam already
   exists: `main.cpp` vs `sim_api.cpp`). The bench/test doubles
   (`BenchOtosSensor`) are not compiled into production.
2. **The `hal.tick` call in the main loop** (`loopTickOnce`, where
   `robot.benchOtosTick(now)` is today): debug builds (bench AND sim) pass
   `state.commands`; production calls the plain tick.

No other site varies. Verified against current source: firmware has no other
`hal.tick` callers; the sim harness's direct `hal.tick` / bench-poke calls
are removed by this change (they move into the loop call).

## Changes

1. Add the debug-build `Hardware::tick(uint32_t now_ms, const MotorCommands&
   cmds)` overload (const ref — hardware reads desired state, never writes
   it). `NezhaHAL` implements it: when the bench OTOS is the active sensor,
   integrate `_benchOtos.tick(cmds.tgtLMms, cmds.tgtRMms, trackwidth, dt)`
   (cfg from construction, dt from now_ms); no-op otherwise.
2. Replace `robot.benchOtosTick(now)` in `loopTickOnce` with the `hal.tick`
   call per the table above (the one build-define switch in the loop).
3. Delete `Robot::benchOtosTick`, `Robot::isBenchOtosActive`,
   `Robot::_lastBenchTickMs`, both `static_cast<NezhaHAL*>` sites in
   Robot.cpp, and their `#ifndef HOST_BUILD` blocks.
4. Rework the `DBG OTOS BENCH` / `DBG OTOS` handlers to reach the sensor
   swap without downcasting Robot's `hal` (interface method on `Hardware`,
   or handler ctx carries the NezhaHAL pointer directly — DebugCommandable
   is firmware-side and may know the concrete type; Robot may not).
5. `MockHAL` implements `tick(now, cmds)` as its plant-integration step
   (motor model → encoders, OTOS from wheel velocities); remove the sim
   harness's direct `hal.tick` calls (`sim_api.cpp:183, 492`) and the
   explicit bench-sensor poke (`sim_api.cpp:685`) — the loop call replaces
   them all. Whether MockHAL integrates from `cmds.tgtLMms/R` or keeps its
   internal PWM-response model is its own implementation choice; the input
   arrives through the same port either way.
6. Exclude `BenchOtosSensor` (and the DBG OTOS commands) from the
   production build.

Out of scope (separate discussion, not yet agreed): runtime hardware-set
selection (`HW SET` + reboot), `MotorCommands` → `ActuatorCommands` rename,
gripper servo joining the command state.

## Acceptance

- `grep -r "benchOtosTick\|isBenchOtosActive" source/` → no hits.
- No `static_cast<NezhaHAL*>` outside hal/ and (if chosen) DebugCommandable.
- `#ifndef HOST_BUILD` count in Robot.cpp reduced to zero for this feature.
- Build-define variation exists in exactly two places: hardware creation and
  the `loopTickOnce` `hal.tick` call.
- Sim suite green unchanged; bench OTOS behavior on hardware identical
  (enable via DBG, drive, synthetic pose advances).
- Production build compiles with no bench/test-double code linked.
