---
id: "010"
title: "N12+N13: Correctness cleanup B — GET serial chunking and dead code removal"
status: open
use-cases:
  - SUC-009
depends-on:
  - '009'
github-issue: ""
issue: ""
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# N12+N13: Correctness cleanup B — GET serial chunking and dead code removal

## Description

**N12 (Low-Med, bench-gated):** The `GET` / `CFG` dump builds into a 768-byte buffer
(`ConfigRegistry.cpp:165`) and realistically produces 600-800 bytes. CODAL's serial TX
buffer is 255 bytes (`SerialPort.cpp:17`, with a comment that bursts must fit).
`sendReliable`'s wait cannot make room for a line longer than the buffer — it spins
5 ms then hands the whole string to ASYNC, which drops the overflow. Bare `GET` over
serial may be truncated mid-keys.

**This ticket is bench-gated**: confirm truncation on hardware before implementing
chunking. If the bench test shows the full config is received correctly (e.g. ASYNC
drains before the next line), the chunking is still a defensive improvement but the
acceptance criterion is met either way.

**N13 (Low):** Residual dead/vestigial code:
- `RatioPidController` — constructed, reset, SET-tunable via `pid.*` keys, but its
  `update()` never runs in `controlTick` (sync-gain coupling replaced it).
- `PID_BYPASS` macro (`MotorController.cpp:12`) — unused.
- `Odometry::update()` — deprecated, no callers.
- `DriveMode::TIMED` — unreachable (T runs as VELOCITY); TLM `mode=` can never
  read `T`. Check host parsers don't expect it before removing.

Depends on ticket 009 to complete the cleanup cluster together.

## Acceptance Criteria

- [ ] N12 bench step: run `GET` over serial and capture full output. Confirm whether
      truncation occurs. (This is a hardware step — team-lead / stakeholder verifies.)
- [ ] N12 implementation: if bench confirms truncation, chunk the `CFG` dump into
      multiple serial writes of <= 200 bytes each. If no truncation is observed,
      add a comment in `ConfigRegistry.cpp` documenting the buffer-size risk and
      confirming bench result.
- [ ] N13: `RatioPidController` construction, reset, and `pid.*` SET wiring removed
      from `MotorController.cpp`.
- [ ] N13: `PID_BYPASS` macro removed.
- [ ] N13: `Odometry::update()` removed (function definition and declaration).
- [ ] N13: `DriveMode::TIMED` removed; grep confirms no host parser expects `mode=T`
      in TLM (or a comment is added if a parser reference is found).
- [ ] `python3 build.py` clean build passes with no new warnings.
- [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.

## Implementation Plan

### Approach

N12: Measure first (bench), then chunk if needed. Chunking approach: iterate over the
config registry key-value pairs and flush a new `CFG` line every ~180 chars rather
than building the entire dump into one buffer.

N13: Delete the identified dead code one piece at a time and verify the build after
each deletion to catch any surprising dependents.

### Files to modify

- `source/config/ConfigRegistry.cpp`
  - N12: chunk the `CFG` dump output if bench confirms truncation.
- `source/motor/MotorController.cpp` (and `.h`)
  - N13: remove `RatioPidController` member, construction, reset, and all `pid.*`
    SET wiring; remove `PID_BYPASS`.
- `source/odometry/Odometry.cpp` (and `.h`)
  - N13: remove `update()` (declaration + definition).
- `source/control/MotionController.cpp` (or wherever `DriveMode::TIMED` appears)
  - N13: remove `TIMED` case and enum value; confirm no switch case handles it.
- `source/control/` (or wherever `DriveMode` is defined)
  - N13: remove `TIMED` from the `DriveMode` enum.

### Bench verification step (N12)

Use `uv run rogo get` or a direct serial session to issue `GET` and capture the full
response. Compare the number of keys returned against the expected count. The
team-lead / stakeholder runs this step with the robot on the bench.

### Testing plan

Run: `uv run --with pytest python -m pytest host_tests/ host/tests/ -v`

Build: `python3 build.py` (clean).

### Notes

- `pid.*` SET keys will return an unknown-key ERR after removal. No known host
  scripts use them (OQ-1 in architecture-update.md). Programmer should grep
  `host/` for `pid.` SET usage before deleting.
- N12 is only a chunking change if the bench confirms truncation. The ticket
  completes regardless — the bench step is explicit and documented here.
- `completes_issue` for `fr2-n11-16-cleanup.md` is handled by ticket 009; this
  ticket has no linked issue (N12/N13 were included in that issue file but this
  ticket is the execution unit for them).
