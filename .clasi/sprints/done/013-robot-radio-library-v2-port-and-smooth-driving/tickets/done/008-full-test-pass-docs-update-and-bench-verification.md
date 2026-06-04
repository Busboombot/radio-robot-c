---
id: 008
title: Full test pass, docs update, and bench verification
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-007
- SUC-008
depends-on:
- '005'
- '006'
- '007'
github-issue: ''
issue: plan-port-v2-update-the-robot-radio-package-new-sprint.md
completes_issue: true
---

# Full test pass, docs update, and bench verification

## Description

This is the integration and verification ticket. All library code and firmware changes from T001–T007 are complete. This ticket:

1. Runs the full test suite; fixes any remaining failures.
2. Updates `host/robot_radio/README.md` (or `host/README.md` if that's where project docs live) with the unified library run path, test command, and calibration script invocation.
3. Performs bench verification: flash firmware (T006), confirm `GET sTimeout=500`, drive a blocking `D` leg, drive a `stream_drive` leg, confirm smooth (no `EVT safety_stop`).
4. Runs `calibrate_linear.py` end-to-end on hardware to confirm it works with the library.

The bench verification steps (3 and 4) require the stakeholder to be at the bench with the robot powered and RADIORELAY connected.

## Acceptance Criteria

**Automated (CI-runnable)**:
- [x] `uv run --with pytest python -m pytest host/tests` — all tests green, no skips. (409 passed in 0.99s)
- [x] `uv run python -c "import sys; sys.path.insert(0, 'host'); from robot_radio.robot import Nezha, NezhaProtocol; from robot_radio import nav, path, controllers, kinematics"` exits 0. (Note: `host/` must be on sys.path since robot_radio is a host sub-package.)
- [x] `host/robot_radio/README.md` created; documents: import path, test command, calibration run (`uv run python tests/calibrate/calibrate_linear.py`), smooth-driving guidance, pytest scope guardrail, and note that `calib_common.py` has been removed. Root `README.md` and `tests/calibrate/README.md` also updated.

**Bench (stakeholder-run)**:
- [ ] Firmware flashed with `sTimeoutMs=500`: `GET sTimeout` returns `500`.

> STAKEHOLDER BENCH STEP — pending

- [ ] Blocking drive: `nezha.speed_for_distance(200, 500)` completes smoothly (no `EVT safety_stop` in log).

> STAKEHOLDER BENCH STEP — pending

- [ ] Stream drive: `stream_drive(200, 200)` for 3 seconds completes smoothly (no `EVT safety_stop`).

> STAKEHOLDER BENCH STEP — pending

- [ ] `uv run python tests/calibrate/calibrate_linear.py` connects to robot, activates laser, drives 900 mm blocking `D`, reads camera + encoder + OTOS, accepts tape input, writes `tovez.json` — no crashes or raw serial errors.

> STAKEHOLDER BENCH STEP — pending

## Implementation Plan

**Approach**: Fix any remaining test failures found after merging all prior tickets. Update README. Document bench steps.

**Files to modify**:
- `host/robot_radio/README.md` or `host/README.md` — add library overview, test command, calibration run path, note on `calib_common.py` removal.
- Any test files with lingering failures from T001–T007 integration.

**Files to create**: None expected — all test files created in prior tickets.

**Testing plan**:
- Run `uv run --with pytest python -m pytest host/tests -v` — should be all green.
- If failures exist, trace to the relevant ticket and fix in this ticket's scope.
- Bench: follow the bench verification steps above with the robot powered on the stand.

**Documentation updates**:
- `host/robot_radio/README.md`: library architecture overview (NezhaProtocol → Nezha → sensors/nav/path), v2 protocol note, test command, calibration script invocation, deferred nav/path note.
- `tests/calibrate/README.md`: update to remove any reference to `calib_common.py`; add note that `calibrate_linear.py` now uses the library.
