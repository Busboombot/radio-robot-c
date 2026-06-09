---
id: '001'
title: "R arc command \u2014 firmware verb + host wrapper"
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# R arc command — firmware verb + host wrapper

## Description

Add the `R <speed_mms> <radius_mm>` firmware verb and supporting host wrapper. This is
the first new command in sprint 018 — it is self-contained and drivable immediately,
making it a safe first ticket.

The arc is a thin `ω = speed / radius` adapter on the existing `BodyVelocityController`:
- Parse `(speed, radius)` from wire.
- Compute `κ = 1/radius`; `radius = 0 ⇒ κ = 0` (straight, no divide-by-zero).
- Sign convention: **positive radius ⇒ positive ω ⇒ CCW (left arc)**. Matches
  `BodyKinematics::inverse` where CCW-positive ω gives `vL < vR`.
- Configure a MotionCommand with target `(speed, speed·κ)`, stop style SOFT, no initial
  stop condition (open-ended arc; host sends `X` or `R 0 r` to stop).
- `R 0 <r>` with speed=0: `ω=0` too (0/r=0); SOFT ramp-down triggers immediately;
  emits `EVT done R`.
- Add `DriveController::beginArc(speed, radius, ...)` entry point.
- Add `R` to HELP verb list.
- Add `arc(speed_mms, radius_mm)` to `protocol.py` (fire-and-forget `send_fast`, mirrors `vw()`).

**Wire format:** `R <speed_mms> <radius_mm> [#id]`
  - `speed_mms`: integer mm/s, range −1000 … +1000
  - `radius_mm`: integer mm, range −10000 … +10000 (0 allowed — means straight)
  - Reply: `OK arc speed=<v> radius=<r> [#id]`
  - Completion: `EVT done R [#id]` (on SOFT ramp-down; open-ended arcs cancelled via X)

Note: Using `EVT done R` until stakeholder confirms; grep existing tests for this string
before finalising to catch any conflicts.

## Acceptance Criteria

- [x] `R 300 0` → `vL == vR` at steady state (straight, no curvature).
- [x] `R 300 200` → positive ω, `vL < vR` (left arc). Verified by Python unit test.
- [x] `R 300 -200` → negative ω, `vL > vR` (right arc). Verified by Python unit test.
- [x] `R 0 200` → SOFT ramp-down begins; `EVT done R` emitted. (EVT format verified by host test; on-robot bench deferred per ticket.)
- [x] `radius_mm = 0` does not divide by zero in firmware.
- [x] `R` appears in `HELP` verb list.
- [x] `arc()` method present in `protocol.py` with correct docstring (CCW-positive convention).
- [x] New Python unit tests: `(speed, radius) → expected (vL, vR)` for straight/left/right
  arcs; explicit sign-convention assertion. Lives in `tests/dev/test_arc_command.py`.
- [x] `uv run --with pytest python -m pytest -q` passes at 1226/8 (1179 + 47 new tests, 8 known pre-existing failures).
- [x] Clean build: `python3 build.py --clean` succeeds.

**Deferred (stakeholder-approved):** On-robot bench verification (straight, left/right arcs, R 0 r soft-stop, X cancel).
**Note for ticket 002 (G migration):** G's PURSUE phase already computes `ω = v * κ` with `κ = 2*dy/d²`. The same `BodyKinematics::inverse` path used by R's BVC will apply. The main difference is G uses a per-tick `setTarget()` call (pursuit hook) rather than a fixed target at begin — same pattern this ticket establishes for R's open-ended operation.

## Implementation Plan

### Files to modify
- `source/control/DriveController.h` — add `beginArc(float speedMms, float radiusMm, uint32_t now_ms, TargetState& target, ReplyFn fn, void* ctx, const char* corr_id = nullptr)` declaration
- `source/control/DriveController.cpp` — implement `beginArc`: compute `omega = (radiusMm != 0) ? speedMms / radiusMm : 0`; configure `_activeCmd` with target `(speedMms, omega)`; `setDoneEvt("EVT done R")`; `setStopStyle(SOFT)`; call `_activeCmd.start(*_hwState, now_ms)`; set `_mode = DriveMode::VELOCITY`
- `source/app/CommandProcessor.cpp` — add `R` verb handler between G and VW; parse `speed = atoi(tokens[1])`, `radius = atoi(tokens[2])`; range-check speed ∈ [-1000, 1000] and radius ∈ [-10000, 10000]; call `_robot.driveController.beginArc(...)`; reply `OK arc speed=<v> radius=<r>`; add `R` to HELP string
- `host/robot_radio/robot/protocol.py` — add `arc(self, speed_mms: int, radius_mm: int) -> None` using `send_fast`
- `tests/dev/` — new or updated test file with arc kinematics unit tests

### Key implementation notes
- Guard `radiusMm != 0` for division; zero radius ⇒ κ = 0, ω = 0.
- `DriveMode::VELOCITY` reused (no new enum value needed). The STREAMING watchdog does
  not fire because the early-return on `_activeCmd.active()` prevents it.
- No stop conditions attached — the command runs until `X`/`STOP` cancel it or the
  host sends `R 0 r` (speed=0 triggers SOFT ramp-down because target `(0,0)` → BVC
  ramps to zero → atTarget() fires).

### Testing plan
- Python unit test: `(speed, radius) → κ → inverse → saturate → (vL, vR)` covering
  straight (radius=0), left (radius=200), right (radius=-200), sign convention assertion.
- Grep `EVT done R` and `OK arc` in existing tests before writing to detect conflicts.
- Full pytest suite: `uv run --with pytest python -m pytest -q`.
- Bench (stakeholder-deferred): straight, left arc, right arc, `R 0 r` soft-stop, `X` cancel.
