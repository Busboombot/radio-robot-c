# Architecture & Modularity Review — Major Issues Only

**Date:** 2026-06-11
**Scope:** layering and placement across firmware (`source/`) and host (`host/robot_radio/`).
**Companion:** `2026-06-11-Fable-s2p-review/` (behavioral defects D1–D12). This review is structural;
where a structural issue *caused* a behavioral defect, the D-number is cited.

Criterion: only issues that are unambiguously wrong or will predictably cause future failures.
Debatable style/taste points are excluded.

---

## A1. Three independent implementations of go-to-point, three pose estimators, no designated owner

The same closed-loop "drive to a world point" capability exists in three places that do not share
code, parameters, or pose state:

1. **Firmware** — `MotionController::beginGoTo` + PURSUE law (encoder/OTOS EKF pose).
2. **Host library** — `nav/navigator.py` (1349 lines) + `controllers/` (pure_pursuit, stanley, ltv,
   pid) driving from camera pose via `sensors/odometry.py`.
3. **CLI inline** — `io/cli.py::cmd_goto` (~165 lines of pure-pursuit with its own gains: TICK_S,
   AIM_GATE, STEER_KP, SLOW_RADIUS…), plus `_spin_to_world_yaw`, `_daemon_spin_to_yaw`,
   `_crawl_drive_distance` — a fourth-ish controller family living inside argument-parsing code.

Likewise three pose estimators: firmware EKF, host `sensors/odom_tracker.py` (TLM integration),
host `sensors/odometry.py` (camera + OTOS fallback). Nothing defines which is authoritative when,
or reconciles them. Every navigation bug must be hunted in three stacks; fixes to one (e.g. the
P0/P1 firmware work) don't help an agent that happens to invoke `cmd_goto` or `navigator`.

**Direction:** pick the owner per regime (suggestion: firmware owns short-horizon motion + pose
fusion; host owns route planning and camera *corrections* sent as pose resets), then delete or
demote the redundant controllers. At minimum, `cmd_goto`'s inline controller should be the first
casualty — fold it into `nav/`.

## A2. Firmware control layer depends upward on the protocol/app layer

`source/control/` includes `CommandProcessor.h` from six .cpp files and several headers
(MotionController, LoopScheduler, Odometry, HaltController, PortController, ServoController,
MotionCommand.h, MotorController.h, RobotState.h). Concretely:

- Control code formats wire replies itself (`CommandProcessor::replyOK/Err/Evt`, inline `snprintf`
  of `EVT …` in `MotionController::emitEvt`, `LoopScheduler` safety_stop, `HaltController`).
- `MotionController` contains the protocol command handlers themselves (`handleS/T/D/G/TURN/RT/VW/
  X/STOP`), builds `ParsedCommand` objects, and pushes to the app-layer `CommandQueue`
  (`MotionCtx` holds `CommandQueue*` + `CommandDescriptor vwDesc`).

This inversion is not cosmetic — it already produced field defects: the converter→queue→handleVW
double-dispatch is the direct mechanism of the duplicate-OK bug (D11) and of the sim/hardware
dispatch split (sim never wires the queue). When the motion state machine and the protocol
front-end are the same class, every protocol change risks motion behavior and vice versa.

**Direction:** command parsing/conversion and reply formatting move to `app/`; control exposes
typed begin*/cancel/advance APIs and reports completion through a narrow callback or event struct
that app/ turns into `EVT` lines. One dispatch path, one reply per command, sim and hardware
identical by construction.

## A3. Two god objects hold most of the firmware

- `MotionController.cpp` — **1953 lines**: 8+ begin* state machines, the pursuit law, stop
  conditions wiring, command handlers/converters, queue manipulation, reply emission.
- `Robot.cpp` — **1490 lines**: facade, sensor orchestration, `buildTlmFrame()` telemetry
  formatting (10+ snprintf fields), the entire command table (`buildCommandTable` registering
  HELLO/PING/GET/SET/STREAM/ZERO/…), config plumbing.

Mostly the same root cause as A2; listed separately because even after the layering fix these
files need splitting (telemetry formatter; command table; motion laws vs. mode machinery) before
any agent can modify one subsystem without context-window-sized diffs and accidental coupling.

## A4. The simulator duplicates the production loop by hand (re-flag)

`host_tests/sim_api.cpp` re-implements `LoopScheduler::run_blocks()` ("MUST mirror
LoopScheduler.cpp exactly" — the comment is the bug report), with its own watchdog, halt, odometry
and telemetry sequencing, plus the unwired-queue dispatch divergence. Already P1.3 in the
improvement plan; repeated here because it is *the* structural reason sim validation keeps failing
to predict field behavior. The `tickOnce()` extraction should be treated as an architecture item,
not a nice-to-have.

## A5. No single owner of the host serial stream; the transport boundary is leaky

Multiple call sites read/write the pyserial object directly, bypassing `SerialConnection`:

- `robot/protocol.py:269` — `ser = self._conn._ser` (reads the raw port inside the protocol layer).
- `io/cli.py:287–289` — raw `conn._ser.reset_input_buffer()` / `_ser.write(b"HELLO\n")`.
- `robot/cutebot.py:93–94` — raw `_conn._ser.write(...)`.
- `SerialConnection.send()` itself clears the input buffer per write (D11a).
- `io/sim_conn.py:69` must fake `_ser = None` because callers reach for the private member.

Result: several uncoordinated readers/writers compete for one input buffer, so TLM/EVT lines are
randomly consumed or destroyed depending on which code path is active. This is the structural form
of D11a; the fix (single reader thread demuxing replies/TLM/EVT, already specified in P2.2.0) is
also the modularity fix — afterward, `_ser` should be unreachable from outside `io/`.

## A6. CLI is a 2262-line application layer with library logic trapped inside

`io/cli.py` contains, beyond arg parsing: closed-loop controllers (A1), calibration push logic
(`_push_calibration`, `_scale_to_int8` — duplicating `calibrate.py`), session/port caching, TLM
snapshot parsing, robot construction policy (`_make_robot`). `io/robot_mcp.py` (1016 lines) is a
second front-end that needs the same behaviors and cannot import them cleanly, so logic drifts
between the two. Anything two front-ends need belongs in `robot/`, `nav/`, or `config/`.

## A7. Calibration logic exists in four places

`host/calibrate_angular.py` (718), `host/calibrate_linear.py` (555), `host/calibrate_verify.py`,
and `robot_radio/io/calibrate.py` (1101) — with literal duplicates (`_deep_merge`, `mean_stdev`,
`scale_to_int8` also re-duplicated in cli.py). Worse than the duplication itself: calibration
*outputs* are not reliably consumed — `rotationalSlip`, `turnScale`, `distScale` are calibrated,
stored, registered in `ConfigRegistry`, and read by nothing in firmware (D2). A calibration
pipeline whose values silently go nowhere is the most expensive kind of dead code: it consumes
bench time and produces false confidence.

**Direction:** one calibration package under `robot_radio/`, thin top-level entry scripts, and a
CI-able check that every config key registered in `ConfigRegistry` is referenced somewhere in
`source/` (would have caught D2 mechanically).

## A8. Config registry and config struct are out of sync (smaller, but mechanical to fix)

`safetyEnabled`, `tlmFields`, `tlmSnapPending` exist in `types/Config.h` + `DefaultConfig.cpp` but
have no `ConfigRegistry` entries (unreachable via GET/SET); conversely several registered keys are
unread (A7/D2). Same root fix as A7's check: generate or lint the registry against both the struct
and actual usage.

---

## Suggested priority

| Rank | Issue | Why first |
|---|---|---|
| 1 | A5 single serial-stream owner | Cheap, host-only, unblocks trusting every other test |
| 2 | A2 (+A3) protocol out of control layer | Eliminates double-OK class, makes sim path = hw path possible |
| 3 | A4 tickOnce() extraction | Makes sim evidence meaningful for everything after |
| 4 | A1 navigation ownership decision | Biggest conceptual issue; needs a decision before code moves |
| 5 | A7/A8 calibration + registry lint | Mechanical; stops silent dead-config recurrence |
| 6 | A6 CLI extraction | Do opportunistically as A1/A7 pull logic out |
