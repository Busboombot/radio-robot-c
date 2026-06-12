---
id: '002'
title: Bench runaway safety wrapper and bench program hardening
status: open
use-cases:
  - SUC-007
  - SUC-008
depends-on: []
github-issue: ''
issue: bench-programs-runaway-auto-abort.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 027-002: Bench runaway safety wrapper and bench program hardening

## Description

A recurring field failure pattern: a bench program declared "reached" off a
wrong fused pose and exited without sending `X`. The firmware `G` kept running
autonomously. This is one of the two open leads from `field-024-full-speed-
spin-unresolved.md` (the host-abandons-G lead).

This ticket creates `tests/bench/bench_safety.py` providing a `BenchRun`
context manager that wraps every bench drive program, and wraps all existing
programs in `tests/bench/` and `tests/dev/` with it.

No firmware changes; purely host-side.

## Acceptance Criteria

- [ ] `tests/bench/bench_safety.py` exists and exports `BenchRun`.
- [ ] `BenchRun` is a context manager with:
  - Constructor parameters: `robot` (the active `Robot` / serial connection),
    `max_seconds` (default 60), optional `progress_fn` callable.
  - `__enter__`: runs preflight liveness check (PING or SNAP; raises
    `RobotSilentError` if no reply within 2 s); registers a `SIGINT` handler
    that calls `send_stop()`.
  - `__exit__`: always calls `send_stop()` (sends `X` + `STREAM 0`), even on
    exception.
  - Wall-clock cap: if `max_seconds` elapsed without explicit stop, calls
    `send_stop()` and raises `RunawayAbortError("wall clock cap")`.
  - Runaway detection (checked on every telemetry frame if a `telem_check`
    is registered, or polled at ~1 Hz via TLM stream):
    - Full-tilt PWM (commanded speed > 50% max) with encoder delta < 5 mm/s
      for 3 consecutive frames → `send_stop()` + raise `RunawayAbortError`.
    - Zero encoder motion for > 5 s while commanding motion → same.
- [ ] All programs in `tests/bench/` and `tests/dev/` that command robot
      motion are wrapped in `with BenchRun(robot) as bench:` or equivalent.
      Specifically: `square_run.py`, `goto_tag.py`, `four_corners.py`,
      `drive2.py`, `drive_measure.py`, `tour_goto.py`, `validate_motion.py`,
      `world_goto_chart.py`.
- [ ] Interrupting a wrapped program with Ctrl-C leaves the robot stopped
      (verified manually by the programmer: run a drive program and Ctrl-C it;
      robot must stop within 1 s).
- [ ] A docstring in `bench_safety.py` documents the `BenchRun` API and
      the error types.

## Implementation Plan

### Approach

`bench_safety.py` is a standalone module (no robot-radio imports except the
serial/robot object). `BenchRun` uses Python's context manager protocol.
The wall-clock cap is implemented as a thread that polls `time.monotonic()`
and calls `send_stop()` when exceeded, then sets a flag to raise in
`__exit__`. Simpler alternative: check elapsed time inline in the calling
program's loop — but `BenchRun` should be passive and not require the caller
to poll it. Use a daemon thread for the wall-clock enforcer.

Runaway detection requires access to the TLM stream. The context manager can
accept a `telem_iter` argument (an iterable of TLM frames if the program is
streaming) or operate without it (just wall-clock cap + SIGINT). For programs
that don't stream, the wall-clock cap alone is sufficient.

`send_stop()` sends `X` using the robot's send method, then `STREAM 0`.

Preflight: send `SNAP` and wait for a reply with a 2 s timeout; if nothing,
raise `RobotSilentError`. (Reuses the existing liveness pattern from
`robot-liveness-preflight` memory.)

### Files to create/modify

- `tests/bench/bench_safety.py` — new module.
- `tests/bench/square_run.py` — wrap motion section in `BenchRun`.
- `tests/bench/goto_tag.py` — wrap motion section.
- `tests/bench/four_corners.py` — wrap motion section.
- `tests/bench/drive2.py` — wrap motion section.
- `tests/dev/drive_measure.py` — wrap motion section.
- `tests/dev/tour_goto.py` — wrap motion section.
- `tests/dev/validate_motion.py` — wrap motion section.
- `tests/bench/world_goto_chart.py` — wrap motion section.
- Other scripts in `tests/dev/` that command motion.

### Testing plan

No automated test exists for `BenchRun` itself (it requires a real serial
connection). Verification is:
1. Run `uv run python tests/bench/square_run.py --boxes black-N` and Ctrl-C
   mid-run — robot stops.
2. Read `bench_safety.py` and verify the `finally` block sends `X`.

Existing `host_tests/` suite still passes (no firmware changes):
```
uv run pytest host_tests/ -v
```

### Documentation updates

Docstring in `bench_safety.py`. Update `tests/bench/README.md` if it exists,
or note the wrapper requirement in comments.

## Notes

- `rogo` commands use `uv run rogo ...`; check whether `bench_safety.py`
  can intercept the rogo-based `square_run.py` or whether it only wraps
  direct serial API programs. If rogo wraps the connection itself, the
  always-X `finally` may need to be in the rogo caller layer. Use
  `rogo ... ; rogo X` shell pattern as a fallback if needed.
- This ticket has no 026-churn exposure (no firmware changes).
