---
id: "005"
title: "TURN verb — turn-to-heading with HEADING stop condition"
status: open
use-cases:
- SUC-005
depends-on:
- "004"
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# TURN verb — turn-to-heading with HEADING stop condition

## Description

Add the `TURN <heading_cdeg>` firmware verb. Rotates the robot to an absolute heading
using `(v=0, ω=±yawRateMax)` body twist and a `HEADING(θ, eps)` stop condition.

**Wire format:** `TURN <heading_cdeg> [eps=<cdeg>] [#id]`
- `heading_cdeg`: integer, target heading in centidegrees (same units as TLM pose field).
  Range: −18000 … +18000 (±180°).
- `eps=<cdeg>`: optional tolerance in centidegrees. Default: 300 cdeg (3°). Range 10–1800 cdeg.
- Reply: `OK turn heading=<cdeg> eps=<cdeg> [#id]`
- Completion: `EVT done TURN [#id]`

**`DriveController::beginTurn(heading_cdeg, eps_cdeg, ...)`:**
1. Convert: `theta_rad = heading_cdeg * PI / 18000.0f`; `eps_rad = eps_cdeg * PI / 18000.0f`.
2. Compute shortest-path ω sign: `delta = wrap_angle(theta_rad - currentHeadingRad)`;
   `omega_sign = (delta >= 0) ? +1.0f : -1.0f`.
3. `omega = omega_sign * _cfg.yawRateMax * PI / 180.0f` (yawRateMax is in deg/s).
4. Configure `_activeCmd` with target `(0.0f, omega)`; add `makeHeadingStop(theta_rad, eps_rad)`;
   set `setDoneEvt("EVT done TURN")`; SOFT style; capture reply sink + corr_id.
5. Call `_activeCmd.start(*_hwState, now_ms)`.
6. `_mode = DriveMode::VELOCITY`.

**HEADING stop semantics (from `StopCondition.cpp`):**
`a` = target heading delta in rad; `b` = eps in rad. Fires when
`|wrap(currentHeading - heading0 - a)| < b`. Since `a` is a *delta* from baseline,
and the baseline captures `heading0Rad` at start, the stop fires when the robot has
rotated by `theta_rad` from its heading at start — matching the absolute target if pose
heading equals `theta_rad` at start. This is correct for a turn-to-absolute-heading when
the baseline is captured at `start()` call time.

**Implementation note on HEADING stop absolute vs. delta:** The HEADING condition uses
`heading0Rad` (baseline heading at start) and `a` (delta). So `a = theta_rad - heading0Rad`
is needed — NOT just `theta_rad`. The `beginTurn` implementation must compute the delta:
`delta_rad = wrap_angle(theta_rad - currentHeadingRad)`, then use `makeHeadingStop(delta_rad, eps_rad)`.
This is what the stop condition evaluates against.

**Host wrapper:** `turn(heading_cdeg, eps_cdeg=300)` in `protocol.py` using `self._conn.send(...)`.
Add TURN to `wait_for_evt_done` examples in docstring.

## Acceptance Criteria

- [ ] `TURN` appears in HELP verb list.
- [ ] `EVT done TURN` emitted on arrival (grep tests for `done TURN` to confirm no prior assumptions).
- [ ] HEADING stop fires at correct heading within eps (Python unit test verifying delta-rad computation).
- [ ] Positive `heading_cdeg` produces CCW rotation (positive ω — matches OTOS CCW convention).
- [ ] `eps=` optional parameter parsed; default 300 cdeg if absent.
- [ ] `turn()` wrapper in `protocol.py`.
- [ ] `uv run --with pytest python -m pytest -q` passes at 1179/8 baseline.
- [ ] Clean build: `python3 build.py --clean` succeeds.

## Implementation Plan

### Files to modify
- `source/control/DriveController.h` — add `beginTurn(float headingCdeg, float epsCdeg, uint32_t now_ms, TargetState& target, ReplyFn fn, void* ctx, const char* corr_id = nullptr)`
- `source/control/DriveController.cpp` — implement `beginTurn` per description
- `source/app/CommandProcessor.cpp`:
  - Add TURN handler: parse `heading_cdeg = atoi(tokens[1])`; parse optional `eps=` via `parseKV`; range-check heading ∈ [-18000, 18000]; call `beginTurn`; reply `OK turn heading=<h> eps=<e>`
  - Add `TURN` to HELP string
- `host/robot_radio/robot/protocol.py` — add `turn(heading_cdeg, eps_cdeg=300)` method

### wrap_angle helper
The `wrap_angle(x)` function is already defined in `StopCondition.cpp` (static). Either
replicate the `atan2f(sinf(x), cosf(x))` pattern in `beginTurn`, or expose it as a
header-level inline. Simplest: replicate inline (one line).

### Testing plan
- Python unit test: verify delta_rad computation for heading targets in all quadrants;
  verify eps conversion; verify ω sign (positive heading → positive ω).
- Grep `done TURN` in all test files before writing EVT emission.
- Full pytest suite.
- Bench (stakeholder-deferred): TURN 9000 rotates ~90° CCW; TURN -9000 rotates ~90° CW.
