---
id: "003"
title: "Migrate T timed-drive onto MotionCommand TIME stop"
status: open
use-cases:
- SUC-003
depends-on:
- "002"
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Migrate T timed-drive onto MotionCommand TIME stop

## Description

Replace `beginTimed`'s bespoke `_tEndMs` deadline + the `if (_mode == TIMED)` branch in
`driveAdvance` with a MotionCommand configured with a `TIME(durationMs)` stop condition.
Convert the `(L, R)` wheel-speed inputs to body twist `(v, ω)` via
`BodyKinematics::forward()` at begin, giving T profiled acceleration under body limits.
`EVT done T` wire contract preserved.

**Key changes:**
- `beginTimed`: call `BodyKinematics::forward(L, R, trackwidthMm, v, omega)` to get the
  body twist; configure `_activeCmd` with target `(v, omega)` and `makeTimeStop(durationMs)`;
  set `setDoneEvt("EVT done T")`; set SOFT style; capture reply sink + corr_id; call
  `_activeCmd.start(*_hwState, now_ms)`.
- Remove `startDriveClean`, direct `mc.setTarget`, `_tgtL/R` write-back, and `_tEndMs`
  assignment from `beginTimed`.
- Remove the `if (_mode == TIMED) { fullStop; emitEvt; }` block from `driveAdvance`.
- Remove `_tEndMs` member from `DriveController.h`.
- `_mode` set to `DriveMode::VELOCITY` (same as VW/R) so STREAMING watchdog does not fire.

**`BodyKinematics::forward(L, R, b, v, omega)`:**
Verify this method exists and has the correct signature. From source: `BodyKinematics.h:52`
documents `forward`. If the signature differs, adapt accordingly.

**EVT done T:** grep all tests (`test_motion_verbs_v2.py`, calibration scripts) for
`done T` before touching any emission path.

**`target.mode` / `target.corrId`:** After migration, `beginTimed` no longer writes
`target.deadlineMs`. The MotionCommand captures corrId; `target.corrId` can be cleared.
Ensure `target.mode = DriveMode::VELOCITY` is set so TLM shows mode=V for T drives
(mode character for VELOCITY is 'V' per AppContext).

Note: T calibration scripts (`calibrate_linear.py`, `calibrate_bench.py`) drive in a
straight line via T commands. After migration, T uses `forward()` to compute `(v, ω)`.
For equal L=R (straight drive), `forward` gives `v = (L+R)/2` and `omega = 0` —
no steer bias. Verify this is correct.

## Acceptance Criteria

- [ ] `_tEndMs` member removed from `DriveController.h`/`.cpp`.
- [ ] `EVT done T` wire format preserved (grep all test files for `done T` before editing).
- [ ] Equal L=R inputs → no steer bias: `omega = 0` from `forward(L, L, ...)`.
- [ ] T branch removed from `driveAdvance`.
- [ ] `uv run --with pytest python -m pytest -q` passes at 1179/8 baseline.
- [ ] Clean build: `python3 build.py --clean` succeeds.
- [ ] Existing T-related tests in `test_motion_verbs_v2.py` pass unchanged.

## Implementation Plan

### Files to modify
- `source/control/DriveController.h` — remove `uint32_t _tEndMs`
- `source/control/DriveController.cpp`:
  - Constructor: remove `_tEndMs(0)` initialiser
  - `beginTimed`: rewrite to call `forward()`, configure MotionCommand, start it;
    remove old `startDriveClean`/`setTarget`/`_tEndMs` lines; set `target.mode = VELOCITY`
  - `driveAdvance`: remove the `if (_mode == TIMED)` block entirely
- `source/app/CommandProcessor.cpp`: no change to T handler (wire format unchanged)

### Testing plan
- Grep `done T` in all test files and calibration scripts to locate all EVT assertions.
- Run `tests/dev/test_motion_verbs_v2.py` — must pass unchanged.
- Run full pytest suite.
- Bench (stakeholder-deferred): T 200 200 1000 drives for ~1 s and emits EVT done T;
  calibration scripts (`calibrate_linear.py`) produce same distance as pre-018.
