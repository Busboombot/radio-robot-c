---
sprint: '034'
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 034 Use Cases

## SUC-034-001: Firmware core compiles identically in production and bench/sim

**Actor:** Firmware build system

**Preconditions:** Sprint 033 merged; `source/robot/Robot.cpp`, `source/control/LoopTickOnce.cpp` currently contain `#ifndef HOST_BUILD` blocks and NezhaHAL downcasts.

**Main Flow:**
1. Developer runs `python3 build.py` (firmware) and `uv run --with pytest python -m pytest host_tests/ host/tests/` (sim).
2. Both exit clean; no compilation errors.
3. Developer greps `source/robot/` and `source/control/` for `NezhaHAL`, `BenchOtosSensor`, `benchOtosTick`, `isBenchOtosActive`.
4. No hits in those directories.

**Postconditions:** Robot, Odometry, and MotionController contain zero bench/sim-specific mentions. All variation is in hal/ and main.cpp / sim_api.cpp.

**Acceptance Criteria:**
- [ ] `grep -r "benchOtosTick\|isBenchOtosActive" source/` returns no hits.
- [ ] `grep -rn "NezhaHAL\|BenchOtosSensor" source/robot/ source/control/` returns no hits.
- [ ] `python3 build.py` exits clean.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` exits green (no regression from the 719-passing baseline).

---

## SUC-034-002: Commanded actuator state reaches the synthetic plant via Hardware::tick

**Actor:** Control loop (loopTickOnce)

**Preconditions:** `Hardware::tick(now, cmds)` overload exists on the interface; NezhaHAL and MockHAL implement it.

**Main Flow:**
1. `loopTickOnce` calls `hal.tick(now, state.commands)` in bench-debug or sim-debug builds (replacing the old `robot.benchOtosTick(now)` call).
2. In bench-debug: NezhaHAL's overload sees `cmds.tgtLMms/tgtRMms`; if bench mode is active, it calls `_benchOtos.tick(...)`.
3. In sim-debug: MockHAL's overload receives `cmds` and uses it as the plant integration input (motors → encoders → OTOS).
4. sim_api.cpp no longer calls `hal.tick(now)` directly or pokes `benchOtos` directly.

**Postconditions:** A single loopTickOnce call drives both the control path and the HAL integration path consistently. No direct `hal.tick` outside `loopTickOnce` in the hot path.

**Acceptance Criteria:**
- [ ] `loopTickOnce` contains the one `hal.tick` call that varies by build define.
- [ ] `sim_api.cpp` has no direct `s->hal.tick(now)` calls outside of any explicit sim_tick helper that itself calls loopTickOnce.
- [ ] `sim_api.cpp` has no direct `benchOtos.tick(...)` poke calls in the main sim_tick / sim_tick_collect_tlm loop paths.
- [ ] Sim suite 719+ tests green.

---

## SUC-034-003: DBG OTOS query returns readable integer-formatted pose on hardware

**Actor:** Bench operator

**Preconditions:** Bench mode enabled with `DBG OTOS BENCH 1`; robot on stand with wheels spinning.

**Main Flow:**
1. Operator sends `DBG OTOS` via serial.
2. Firmware formats ideal, otos, fused, and err pose values as scaled integers (mm for x/y, cdeg for heading).
3. Reply line contains non-blank integer fields, e.g. `ideal=0,0,0 otos=2,-1,45 fused=150,20,1745 err=-2,1,-45`.
4. Final `OK dbg otos` reply line follows.

**Postconditions:** Bench operator can read the synthetic vs fused pose comparison on hardware.

**Acceptance Criteria:**
- [ ] `DBG OTOS` reply on hardware contains no blank comma-separated fields (no `ideal=,,` symptom).
- [ ] Fields use integer format consistent with SNAP (mm for position, cdeg for heading).
- [ ] Host sim test asserts the integer-format path produces non-empty, in-range values for a known pose.
- [ ] Existing sim test for `DBG OTOS` reply structure passes unchanged (or is updated to expect integer format).

---

## SUC-034-004: BenchOtosSensor and DBG OTOS commands absent from production build

**Actor:** Production firmware build

**Preconditions:** Production build define is set (no bench/debug/sim define); `BenchOtosSensor.cpp` is under `source/hal/` (auto-globbed).

**Main Flow:**
1. `python3 build.py` in production configuration compiles the firmware.
2. `BenchOtosSensor.cpp` is excluded via a compile-time guard or CMake filter.
3. `DBG OTOS` and `DBG OTOS BENCH` command handlers are guarded out of the production command table.

**Postconditions:** Production binary contains no bench-testing code. Binary size does not regress.

**Acceptance Criteria:**
- [ ] Production build links without `BenchOtosSensor` symbols.
- [ ] `DBG OTOS` and `DBG OTOS BENCH` commands are absent from the production command table.
- [ ] `python3 build.py` exits clean.

---

## SUC-034-005: Bench OTOS end-to-end behavior preserved on hardware after refactor

**Actor:** Bench operator

**Preconditions:** Sprint 034 firmware flashed; robot on stand, real OTOS detected on boot.

**Main Flow:**
1. Operator sends `DBG OTOS BENCH 1`. Firmware replies `bench=1`.
2. Operator sends `D 500` (or `T 150 150`). Wheels spin freely on stand.
3. Operator issues `SNAP` or `STREAM`. `otos=` and `pose=` fields advance (synthetic pose feeds EKF).
4. Operator sends `DBG OTOS BENCH 0`. Firmware replies `bench=0`. `otos=` in subsequent SNAP is frozen (real OTOS on stand sees no motion).
5. Operator power-cycles. Robot boots with bench mode off.

**Postconditions:** Bench-OTOS tooling fully functional; refactor has not broken any observable behavior.

**Acceptance Criteria:**
- [ ] `DBG OTOS BENCH 1` activates synthetic OTOS (reply `bench=1`).
- [ ] Drive command with bench mode active shows non-zero `otos=` and `pose=` advancing in telemetry.
- [ ] `DBG OTOS` returns non-empty integer pose fields (SUC-034-003 satisfied).
- [ ] `DBG OTOS BENCH 0` restores real OTOS (reply `bench=0`; pose frozen on stand).
- [ ] Reboot resets bench mode to off.
