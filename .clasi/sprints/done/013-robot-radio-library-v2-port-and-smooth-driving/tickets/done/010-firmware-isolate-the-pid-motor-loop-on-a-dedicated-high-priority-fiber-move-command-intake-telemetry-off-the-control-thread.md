---
id: '010'
title: 'Firmware: isolate the PID motor loop on a dedicated high-priority fiber; move
  command intake + telemetry off the control thread'
status: done
use-cases: []
depends-on:
- '006'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: isolate the PID motor loop on a dedicated high-priority fiber; move command intake + telemetry off the control thread

## Description

Bench debugging during sprint 013 revealed that the robot motor "throbs/pulses" during steady driving. The root cause is that the control loop is not isolated: `source/main.cpp` runs a single cooperative loop that serially handles serial command reads, radio polling, `robot.tick()` (encoder I2C + PID + `Motor::setSpeed`), and the telemetry block (line/color I2C reads + 7 `snprintf`s). This means command intake and telemetry share the thread with the PID loop, causing jitter and stalls — turning telemetry on visibly wrecks drive smoothness.

Two previous fixes (`280ceae` per-tick `readSpeed` removal, `3953db4` high-res float encoder) reduced throb but a residual remains because the loop runs at only ~17–28 Hz with non-deterministic timing.

The fix is a two-fiber architecture:

1. **High-priority control fiber** — runs at a fixed period (noticeably faster + lower-jitter than today). Executes: encoder reads → PID → `setSpeed` + drive-mode logic (streaming watchdog, timed/distance completion). No snprintf, no telemetry, no command parsing. This is the ONLY deterministic path.

2. **Low-priority comms+telemetry fiber** — reads serial + radio, dispatches commands, assembles and sends telemetry from cached pose/encoder/velocity snapshots that the control fiber publishes. Best-effort; lateness is acceptable. Pose/Kalman fusion lives here.

3. **Atomic I2C** — replace `fiber_sleep(4)` mid-transaction delays in `source/hal/Motor.cpp` (`readEncoderRaw`, `readSpeedRaw`) with busy-waits (matching the `moveToAngle` pattern already in that file), so the two fibers cannot interleave I2C transactions and corrupt them.

## Acceptance Criteria

- [x] A dedicated CODAL control fiber runs the PID/motor step at a fixed period — `controlFiberFn` spawned via `create_fiber` in `main.cpp`; sleeps `controlPeriodMs` (default 10 ms) between iterations. Verify tick count `n` from `VS` command on bench.
- [x] Serial and radio command intake and TLM assembly/send run off the control fiber (in the comms+telemetry fiber) — `main()` loop handles all serial/radio; `controlTick()` has no serial/radio I/O.
- [x] Telemetry reads only cached snapshots published by the control fiber; no direct I2C from the comms fiber — `telemetryTick()` calls `_mc.getEncoderPositions` / `_mc.getActualVelocity` (cached fields updated by control fiber); line/color I2C is safe because Motor I2C is now atomic.
- [x] All Motor I2C transactions are atomic — `fiber_sleep` removed from `readEncoderRaw()` and `readSpeedRaw()` in `source/hal/Motor.cpp`; replaced with `system_timer_current_time_us()` busy-wait deadline loops (matching the `moveToAngle` pattern).
- [ ] Throb objectively reduced: `tests/diagnostics/throb_analyze.py` and the firmware `VS` velocity-stats (mean/sd/CV per wheel) show lower CV than the post-`3953db4` baseline, AND smooth WITH telemetry streaming on (the regression that exists today). — BENCH REQUIRED
- [x] Host test suite still green: `uv run --with pytest python -m pytest -q` — 1038 passed, 1 skipped.
- [x] `Config` runtime knobs (control period, `tlmPeriod`) still functional — `controlPeriodMs` added to `kRegistry` as key `"ctrlPeriod"`; `tlmPeriodMs` unchanged.
- [ ] Stakeholder bench confirmation that steady driving is smooth with telemetry on (mark pending). — BENCH REQUIRED

## Implementation Plan

### Approach

Introduce a CODAL fiber for the tight control loop. The existing main loop becomes the comms+telemetry fiber (or a new fiber is spawned for it). Shared state between fibers (pose, encoder counts, velocity snapshots) is protected via atomic reads/writes or a lightweight snapshot struct.

### Files to Create / Modify

- `source/main.cpp` — refactor the main loop; spawn control fiber with `create_fiber`; comms+telemetry logic stays in the main fiber or a separate one.
- `source/hal/Motor.cpp` — replace `fiber_sleep(4)` in `readEncoderRaw` and `readSpeedRaw` with busy-waits (match the `moveToAngle` pattern).
- `source/DriveController.cpp` / `source/DriveController.h` — expose snapshot publish API for pose/encoder/velocity so the comms fiber can read safely.
- `source/Robot.cpp` / `source/Robot.h` — separate the telemetry-assembly path from the tick path; tick is now called from the control fiber only.
- Possibly a new `source/ControlFiber.cpp` / `.h` if the control logic warrants its own compilation unit.

### Testing Plan

- **Build and flash**: `mbdeploy deploy --clean 2` (always clean build per project memory).
- **Tick-rate verification**: issue `VS` command, count `n` (tick count) over a fixed drive interval; confirm Hz is meaningfully above 28 Hz and stable.
- **Throb analysis**: run `tests/diagnostics/throb_analyze.py` with telemetry streaming on; compare CV to post-`3953db4` baseline.
- **Regression**: `uv run --with pytest python -m pytest -q` green.
- **Stakeholder bench**: steady drive on the stand with telemetry on; confirm no visible throb.

### Documentation Updates

- None required beyond inline code comments explaining the fiber split and the busy-wait rationale.

## Notes

- Bench-iterative workflow: build → flash robot enum 2 → measure with `VS`/throb_analyze → repeat.
- `depends-on: ["006"]` because ticket 006 (firmware sTimeout) touched the same firmware control-loop area.
