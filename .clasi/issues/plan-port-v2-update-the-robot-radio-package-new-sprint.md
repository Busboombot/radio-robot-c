---
status: pending
---

# Plan: Port & v2-update the `robot_radio` package (new sprint)

## Context

Bench driving has been **herky-jerky** and prone to safety-stops. Two root causes:

1. **Firmware:** the `S`/`VW` streaming watchdog default is `sTimeoutMs = 200`
   ([source/types/Config.h](source/types/Config.h)). Over the laggy RADIORELAY,
   gaps > 200 ms make the firmware cut motors (`EVT safety_stop`) → jerk →
   restart. Blocking `T`/`D` moves have **no watchdog** and are inherently smooth.
2. **Host:** every bench script (incl. the new `tests/calibrate/calib_common.py`)
   **hand-rolls raw serial** instead of going through a real robot library. There
   is no single, tested abstraction for talking to the robot.

The stakeholder wants the proven **prior-repo library**
(`/Volumes/Proj/proj/league-projects/scratch/radio-robot/robot_radio`) copied in
and **fully updated to protocol v2**, then tested — and made the package we
always test against henceforth. Its architecture is good (a single **protocol
object** that owns the wire + **higher-level objects** that delegate to it); the
bulk of the change is the protocol object.

This runs as a **dedicated sprint**. Sprint 012 is closed out first.

## Current state (what exists)

- **Prior lib (source to copy):** `…/scratch/radio-robot/robot_radio` — 49 files,
  rich: `io/serial_conn.py`, `robot/protocol.py` (`NezhaProtocol`, **v1 wire**),
  `robot/nezha.py` (`Nezha` high-level driver), `robot/robot.py`, `sensors/`
  (otos, odom_tracker, cam_tracker, color, motion_monitor), `nav/`, `path/`,
  `controllers/`, `kinematics/`, `config/`, `io/cli.py` (`rogo`), `io/calibrate.py`.
  Speaks **v1**: sign-prefixed no-space tokens (`S+100-50`), per-stream
  `SSE/SSO/SSL/SSC` emitting `ENC`/`SO`/`LS`/`CS`, ad-hoc replies
  (`ACK:OL`, `TN+DONE`), verbs `EZ/ENC/SO/SZ/OR/OI/OK/OO/SI/K+SS/K+TW/ROT/TN`.
- **This repo already has a PARTIAL v2 port:** [host/robot_radio/](host/robot_radio/)
  with a **correct, tested v2 `NezhaProtocol`** (`robot/protocol.py`:
  `parse_response`/`parse_tlm`/`parse_cfg`, `drive`/`timed`/`distance`/`go_to`/
  `vw`/`stream_drive`/`wait_for_evt_done`, OTOS `OP/OZ/OR/OI/OL/OA`, `SET/GET`),
  `io/serial_conn.py` (v2 relay handshake), `sensors/odom_tracker.py`,
  `sensors/cam_tracker.py`, `config/robot_config.py` (pydantic `RobotConfig`,
  `tovez.json`, schema), `io/cli.py` (`rogo`), `io/calibrate.py`, `clock_sync.py`,
  and **44 passing tests** in [host/tests/](host/tests/). This is the **reference
  + harvest source** for the v2 protocol layer — not to be discarded.
- **Firmware v2 verbs** (from `HELP`): `PING ECHO ID VER HELP SET GET "GET VEL"
  STREAM SNAP S T D G VW STOP GRIP ZERO OI OZ OR OP OV OL OA P PA`. Note: **no**
  `EZ/ENC/SO/LS/CS/SI/OO/TN/ROT/K+…` — those v1 verbs are gone.

So the work is **not** a fresh v1→v2 reverse-engineering: the prior lib provides
the rich architecture; the existing `host/robot_radio` provides the proven v2
protocol + config + tests to graft in.

## Goal / Issue statement

> **Issue:** Establish `robot_radio` as the single, tested robot-control library
> for this project: bring over the prior repo's richer architecture, convert it
> fully to protocol v2 (protocol object rewrite + higher-level objects adapted),
> make driving smooth (blocking `T`/`D` + correct `S` keepalive; firmware
> watchdog default raised), reconcile with this repo's pydantic config, port the
> calibration tools and `tests/calibrate` onto the library, and cover it with
> tests. Henceforth all robot interaction goes through this package and is tested.

## Approach

### Process (CLASI, team-lead)
1. **Close Sprint 012.** It is in a messy state (project `uninitialized`; 012
   `executing` with 11 tickets in `unknown` state; T011 not done). Resolve T011
   (the playfield-calibration/bench work) by marking it done or splitting the
   unfinished bench-verification into a deferred issue, then run the `close-sprint`
   skill (`close_sprint`, `main_branch="master"`, `test_command="uv run --with
   pytest python -m pytest host/tests"` or `"true"` to skip). Expect to use
   close-sprint self-repair / `move_ticket_to_done` for the `unknown`-state tickets.
2. **Create Sprint 013** from the issue above (sprint-planner writes architecture
   + sequenced tickets). This plan file is the issue source.

### Canonical location & layout
- The library lives at **`host/robot_radio/`** (replacing the current partial one;
  tests already live at `host/tests/`). Keep the prior architecture's module
  layout, adding the prior lib's richer packages (`nav/`, `path/`, `controllers/`,
  `kinematics/`, `media/`, extra `sensors/`).
- Public API preserved from the prior lib: **`NezhaProtocol`** (protocol object)
  and **`Nezha`** (high-level driver) plus `SerialConnection`, sensors/nav/path.

### The core change — protocol object (v1 → v2)
Rewrite `robot/protocol.py` so every method speaks v2. **Harvest the already-correct
implementation from the current `host/robot_radio/robot/protocol.py`** rather than
re-deriving. Concrete mapping:

| Concern | v1 (prior lib) | v2 (target) |
|---|---|---|
| Number format | sign-prefix, no spaces `S+100-50` | space-separated `S 100 -50` |
| Relay framing | `>`-prefix / strip `<#` | v2 `SerialConnection` mode prefix (already handled) |
| Stream drive | `S` + `SSE/SSO/...`; resend at 30% watchdog | `S`/`VW` keepalive + unified `STREAM <ms>`; resend at 30% |
| Wheel keepalive | `drive()` → `S±l±r` | `drive()` → `S l r`; add `vw()` (`VW v omega`) |
| Timed/Distance | `T±l±r±ms` / `D±l±r±mm`, poll `ENC` | `T l r ms` / `D l r mm`, `wait_for_evt_done("T"/"D")` |
| Go-to | `G±x±y±spd` | `G x y spd` + `EVT done G` |
| Completion | poll `ENC`/`SAFETY_STOP`/`LOG:X` | `EVT done <verb>` / `EVT safety_stop` (+`#corr_id`) |
| Telemetry | per-stream `ENC`/`SO`/`LS`/`CS` | unified `TLM …` frame (`parse_tlm`), `SNAP` |
| Encoders zero | `EZ` | `ZERO enc` |
| Pose/odo zero | `SZ` | `ZERO pose` |
| OTOS read pose | `SO` (fused, robot frame) | `TLM pose=` (fused) + `OP` (raw LSB ×0.305176 mm) |
| OTOS init | `OI`+`OK` | `OI` (IMU-cal folded in) |
| OTOS reset/zeroraw | `OR` / — | `OR` (reset tracking) / `OZ` (zero raw) / `OV` (set raw) |
| OTOS scalars | `OL±n`/`OA±n` → `ACK:OL` | `OL n`/`OA n` → `OK linear scalar=n` |
| Config get/set | `K+SS`, `K+TW`, `OO`, `SI` | `SET k=v` / `GET k…` (`sTimeout`, `ml`, `mr`, `tw`, …); **no `OO`/`SI`** |
| Ports | `P+port+val` / `PA+port+val` | `P port val` / `PA port val` |
| Turn-in-place | `TN±deg` (`TN+DONE`) | **not in v2** — use `G`/`VW`/`T`; drop `turn_closed_loop`/`rotate_motor` or reimplement host-side |
| Identity/liveness | (n/a) | add `ping()`/`get_id()`/`get_ver()` (already in v2 ref) — **mandatory preflight** (see [[robot-liveness-preflight]]) |

Verbs with **no v2 equivalent** (`TN`, `ROT`, `OO`, `SI`, `ENCM`, separate `LS`/`CS`)
are removed or reimplemented host-side; document each in the architecture.

### Higher-level objects (smaller changes)
- `robot/nezha.py` (`Nezha`): keep the public surface (`speed`, `speed_for_time`,
  `speed_for_distance`, `go_to`, `drive` generator, `stop`, OTOS/port helpers,
  live `encoders`/`otos_pose`/`line_sensor`/`color` state) but route through the
  v2 protocol and consume `TLM`/`EVT`. Heading stays **radians** at this layer.
  Preserve the `speed_for_distance` **hop strategy** but rebase it on `D`
  + `wait_for_evt_done` (v2 `D` already self-terminates at distance with a 5 s cap).
- `sensors/odom_tracker.py`, `cam_tracker.py`: adopt v2 `parse_tlm` (reuse the
  current host versions) — robot tag = **100**, cm→mm ×10.
- `nav/`, `path/`, `controllers/`, `kinematics/`: copy over; they sit above the
  protocol and need only the `Nezha`/pose API to be stable. Defer deep validation.

### Smooth driving (the actual fix)
- **Calibration/bench moves use blocking `T`/`D`** via `Nezha` (no watchdog).
- **Continuous/teleop** uses `stream_drive()` keepalive at 30% of watchdog.
- **Firmware:** raise `sTimeoutMs` default `200 → 500` in
  [source/types/Config.h](source/types/Config.h) (and confirm the watchdog check
  path in `source/control/DriveController.cpp`); requires a clean build + reflash
  (`mbdeploy build --clean` then mass-storage flash — see [[clean-build-before-bench]]).

### Config reconciliation
Keep **this repo's** pydantic `RobotConfig` + `data/robots/tovez.json` + schema as
source of truth (already richer: `peripherals.laser_port`, calibration block).
Point the ported lib at it; drop the prior lib's own config module if it conflicts.

### Calibration tools onto the library
Rewrite `tests/calibrate/calibrate_linear.py` (and retire the hand-rolled
`tests/calibrate/calib_common.py`) to use `Nezha`/`NezhaProtocol` + the library's
camera/odom trackers. Drive with blocking `D`. Keep the tape=truth /
camera+enc+OTOS-tracked / closed-loop behavior already designed.

### Tests (standing rule)
- Start from the 44 passing tests in `host/tests/`; expand to cover the ported
  high-level `Nezha` driving (mock `SerialConnection`), the v1→v2 protocol
  encodings, `wait_for_evt_done`, `stream_drive` keepalive cadence, and the
  calibration math. **Henceforth every robot-interaction change ships with tests
  against `robot_radio`.**

## Representative files to create/modify
- Copy in (ticket #1, verbatim): the prior `robot_radio/**` (nav, path,
  controllers, kinematics, media, sensors, robot, io) into `host/robot_radio/`.
- Rewrite: `host/robot_radio/robot/protocol.py` (v2), `robot/nezha.py`,
  `sensors/odom_tracker.py`, `sensors/cam_tracker.py`, `io/serial_conn.py`
  (v2 relay), `io/calibrate.py`, `io/cli.py`.
- Keep: `host/robot_radio/config/robot_config.py`, `data/robots/*`, schema.
- Firmware: `source/types/Config.h` (`sTimeoutMs` 200→500).
- Tools: `tests/calibrate/calibrate_linear.py` (rebased on the lib); remove
  `tests/calibrate/calib_common.py`.
- Tests: `host/tests/test_protocol_v2.py` (extend), new `test_nezha_drive.py`,
  `test_stream_keepalive.py`, `test_calibrate_linear.py`.

## Risks / decisions
- **Sprint 012 close may need repair** (tickets in `unknown` state; project
  `uninitialized`). Handle via close-sprint self-repair / explicit
  `move_ticket_to_done`; if T011 bench-verification isn't done, split it to a
  deferred issue so 012 can close.
- **Scope of nav/path/controllers**: copied over but only smoke-tested this sprint;
  deep v2 validation can be a follow-up sprint. Flag in architecture.
- **Dropped v1 verbs** (`TN`, `OO`, `SI`, `ROT`): confirm nothing critical depends
  on them; reimplement turn-in-place host-side if needed.
- **Single serial owner**: keep the library single-threaded (one `SerialConnection`
  owner) as both repos do.

## Verification (end-to-end)
1. `uv run --with pytest python -m pytest host/tests` — all green (incl. new tests).
2. Import smoke: `uv run python -c "from robot_radio.robot import Nezha, NezhaProtocol"`.
3. Robot preflight on a powered robot: `PING`/`ID` round-trip via the lib.
4. Bench: flash firmware with `sTimeout=500`; drive a multi-second leg with
   blocking `T`/`D` and confirm **smooth, no safety-stop**; then a `stream_drive`
   leg and confirm smooth keepalive.
5. Run the rebased `tests/calibrate/calibrate_linear.py`: laser on (port 4),
   blocking drive, camera+tape+enc+OTOS distances, closed-loop convergence,
   writes `tovez.json`.

## High-level sprint/ticket outline
1. Copy prior `robot_radio` verbatim into `host/robot_radio/` (additive).
2. Rewrite the **protocol object** to v2 (harvest from current host version) + tests.
3. Adapt `Nezha` high-level driver (blocking T/D, stream keepalive, TLM/EVT) + tests.
4. Adapt sensors/odom/cam trackers + config wiring to v2 + tests.
5. Bring nav/path/controllers across (smoke-level) ; mark deep validation deferred.
6. Firmware: `sTimeoutMs` 200→500; clean build + reflash; bench smoothness check.
7. Rebase `tests/calibrate/calibrate_linear.py` on the library; remove `calib_common.py`.
8. Full test pass + bench verification; update docs/README; close sprint.
