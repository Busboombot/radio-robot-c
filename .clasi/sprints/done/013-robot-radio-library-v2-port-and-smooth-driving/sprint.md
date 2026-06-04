---
id: '013'
title: 'robot_radio Library: v2 Port and Smooth Driving'
status: done
branch: sprint/013-robot-radio-library-v2-port-and-smooth-driving
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
issues:
- plan-port-v2-update-the-robot-radio-package-new-sprint.md
---

# Sprint 013: robot_radio Library: v2 Port and Smooth Driving

## Goals

1. Establish `robot_radio` (at `host/robot_radio/`) as the single, fully tested robot-control library for this project.
2. Complete the v1-to-v2 protocol migration: protocol object rewritten from v1 verbs to v2 wire format; higher-level `Nezha` driver, sensors, nav/path stack adapted.
3. Make driving smooth: raise the firmware streaming watchdog default (`sTimeoutMs` 200 → 500 ms); use blocking `T`/`D` for calibration/bench moves; use `stream_drive` keepalive at 30 % of the watchdog for continuous driving.
4. Rebase all calibration tooling onto the library; remove hand-rolled serial code.
5. Cover all robot interactions with tests; henceforth no robot-interaction change ships without tests.

## Problem

Bench driving has been herky-jerky due to two compounding issues:

- **Firmware**: the `S`/`VW` streaming watchdog fires at 200 ms, which is too short for the laggy RADIORELAY. Motors cut (`EVT safety_stop`) → jerk → restart.
- **Host**: calibration and bench scripts hand-roll raw serial (`calib_common.py`) instead of routing through a tested library. There is no unified, tested abstraction for talking to the robot.

The prior library (`scratch/radio-robot/robot_radio`) has the right architecture (single `NezhaProtocol` wire owner + higher-level objects) but speaks v1. This repo already has a correct, tested v2 `NezhaProtocol` and 44 passing tests. The sprint merges these two into a single canonical library.

## Solution

The work proceeds in eight stages (one ticket each):

1. Bring in the prior library's richer module structure (nav, path, controllers, kinematics, extra sensors) without overwriting the existing v2 protocol/config/tests.
2. Consolidate the protocol object around the existing v2 `NezhaProtocol` (harvest encodings and helpers; confirm full v1→v2 verb mapping; tests extended).
3. Rewrite the `Nezha` high-level driver for v2 (blocking `T`/`D` via `wait_for_evt_done`, `stream_drive` keepalive, TLM/EVT consumption); tests added.
4. Adapt sensors, odom tracker, and cam tracker to v2 `parse_tlm` + pydantic config; confirm robot tag 100; tests added.
5. Smoke-validate nav/path/controllers/kinematics import chain; defer deep v2 validation.
6. Raise `sTimeoutMs` 200 → 500 in firmware `Config.h`; verify watchdog path in `DriveController.cpp`; note clean-build + reflash requirement.
7. Rebase `tests/calibrate/calibrate_linear.py` on the library; remove `calib_common.py`.
8. Full test pass, README/docs update, bench verification.

## Success Criteria

- `uv run --with pytest python -m pytest host/tests` — all green, including new tests for `Nezha` drive, `stream_drive` keepalive, and calibration math.
- `from robot_radio.robot import Nezha, NezhaProtocol` imports cleanly.
- Robot preflight (`PING`/`ID`) round-trip confirmed on powered hardware.
- Bench: firmware flashed with `sTimeout=500`; blocking `T`/`D` leg completes smoothly with no `EVT safety_stop`; `stream_drive` leg smooth.
- `tests/calibrate/calibrate_linear.py` runs end-to-end on hardware; `calib_common.py` is deleted.

## Scope

### In Scope

- Bringing nav/path/controllers/kinematics modules into `host/robot_radio/` (smoke-level; deep validation deferred).
- Protocol object consolidation: full v1→v2 verb mapping, `parse_response`/`parse_tlm`/`parse_cfg`, EVT/TLM/CFG, OTOS, SET/GET, ping/id/ver, liveness preflight.
- `Nezha` high-level driver for v2 with blocking `T`/`D` and `stream_drive` keepalive.
- Sensors: `odom_tracker`, `cam_tracker`, `otos.py` aligned to v2 TLM + pydantic config + tag 100.
- Firmware `sTimeoutMs` 200 → 500; clean build + reflash.
- Calibration script rebased on library; `calib_common.py` removed.
- Test expansion: `Nezha` drive mock, keepalive cadence, calibration math.

### Out of Scope

- Deep v2 validation of nav/path/controllers path planning (smoke only; deferred to a follow-up sprint).
- Firmware changes beyond `sTimeoutMs` (no new firmware features this sprint).
- `rogo` CLI changes beyond import compatibility.
- Angular calibration script (`calibrate_angular.py`) — not rebased in this sprint.

## Test Strategy

Tests run with:

```
uv run --with pytest python -m pytest host/tests
```

Existing 44 tests must remain green throughout. New tests added per-ticket:

- **T002**: extend `test_protocol_v2.py` with v1→v2 encoding assertions; test `parse_response`, `parse_tlm`, `wait_for_evt_done`, ping/id/ver.
- **T003**: `test_nezha_drive.py` — mock `SerialConnection`; test `speed_for_time`, `speed_for_distance`, `stop`, `go_to`, `wait_for_evt_done`; assert no `drive()` call goes without a v2-format `S` or `T`/`D` command.
- **T003**: `test_stream_keepalive.py` — assert `stream_drive` resends at ≤30 % of watchdog (i.e., within 150 ms for 500 ms watchdog).
- **T004**: sensors tests — odom_tracker and cam_tracker parse v2 TLM; confirm tag 100 passthrough.
- **T007**: `test_calibrate_linear.py` — mock drive + mock sensor readout; confirm closed-loop math; no raw serial calls.

## Architecture Notes

See `architecture-update.md`. Key constraints:

- Single `SerialConnection` owner (library is single-threaded; no shared serial state).
- Robot liveness preflight (`PING`/`ID`) is mandatory on every connect path.
- `NezhaProtocol` is the only code that touches the serial port; all higher-level objects delegate to it.
- `host/robot_radio/config/robot_config.py` (pydantic `RobotConfig`) + `data/robots/tovez.json` + JSON schema are the config source of truth.
- v2 wire format: space-delimited tokens; `EVT done <verb>` completion; unified `TLM` frame; no v1 sign-prefix or per-stream verbs.

## GitHub Issues

(None linked — driven by internal CLASI issue.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Copy prior robot_radio modules into host/robot_radio | — |
| 002 | Consolidate NezhaProtocol to v2 + extend tests | 001 |
| 003 | Rewrite Nezha high-level driver for v2 + tests | 002 |
| 004 | Adapt sensors, odom/cam trackers to v2 + pydantic config | 003 |
| 005 | Smoke-validate nav/path/controllers/kinematics import chain | 004 |
| 006 | Firmware: raise sTimeoutMs 200 to 500 | — |
| 007 | Rebase calibrate_linear.py on the library; remove calib_common.py | 003, 004 |
| 008 | Full test pass, docs update, bench verification | 005, 006, 007 |

Tickets execute serially in the order listed.
