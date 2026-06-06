---
id: '001'
title: Headless wedge reproduction harness (tests/bench/wedge_repro.py)
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: residual-motor-encoder-wedge-after-stop.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 015-001: Headless wedge reproduction harness (tests/bench/wedge_repro.py)

## Description

Create `tests/bench/wedge_repro.py`: a pure-pyserial (NO matplotlib) bench
harness that drives deterministic drive->stop->drive cycles and auto-detects
the encoder wedge condition (commanded to move but enc delta ~= 0). The script
produces a numeric wedge rate (X wedged / N total) for two stop-trigger modes:
clean serial STOP and watchdog-fired stop (keepalive lapse).

This is Phase 0 of the issue plan. Without a deterministic repro we cannot
trust any fix. The script is host Python only -- no firmware change.

Reference for connection pattern: `tests/bench/drive_raw.py`.

## Acceptance Criteria

- [x] `uv run python tests/bench/wedge_repro.py --help` runs without error.
- [x] Script connects without DTR reset (`dtr=False`, `dsrdtr=False`), PINGs
  for liveness, and hard-fails with a clear error if the robot is silent.
- [x] `SET sTimeout=2000` is sent before streaming begins.
- [x] `--clean-stop` mode: each drive phase ends with an explicit `STOP` sent
  over serial (3x with 50 ms gaps), then the script pauses 200 ms and restarts.
- [x] `--watchdog-stop` mode: each drive phase ends by letting the S keepalive
  lapse for at least `sTimeout + 500` ms so the firmware S-watchdog fires; then
  the script resumes.
- [x] `--cycles N` controls the number of drive->stop->drive cycles (default 50).
- [x] `--speed V` sets the wheel speed passed to `S V V` (default 200).
- [x] Wedge detection: after each stop->restart, reads streaming lines for
  >= 1.5 s and classifies the cycle as "wedged" if both `enc=L,R` values show
  |delta| < 5 mm over the observation window, or "clean" if either encoder
  accumulates movement.
- [x] Script prints per-cycle result (wedged/clean) and a final summary line:
  `RESULT: X/N wedged  mode=<mode>`.
- [ ] A documented bench run of `--watchdog-stop --cycles 20` produces at least
  1 observed wedge on the hardware bench -- or the output explicitly states
  "0 wedges in 20 cycles" documenting the absence.

## Implementation Plan

### Approach

Model on `tests/bench/drive_raw.py`. Key differences from drive_raw.py:
- Loop N cycles instead of a fixed-duration drive.
- Two stop paths controlled by `--watchdog-stop` / `--clean-stop` flag.
- Post-stop encoder observation window (1.5 s) with delta classification.
- Wedge detection and per-cycle + summary reporting.

### Files to Create

- `tests/bench/wedge_repro.py` -- new file

### Structure

```
wedge_repro.py
  parse_args()             -- argparse: port, speed, cycles, watchdog-stop/clean-stop
  connect(port)            -- serial dtr=False dsrdtr=False; PING liveness; fail if silent
  setup_stream(s)          -- SET sTimeout=2000; STREAM 40
  drive_phase(s, speed, duration_s)   -- S speed speed; keepalive every 150 ms
  stop_clean(s)            -- STOP x3 at 50 ms gaps; 200 ms settle
  stop_watchdog(s, sTimeout_ms)       -- let keepalive lapse; sleep sTimeout+500 ms
  observe_encoders(s, window_s)       -- collect enc= lines for window_s; return (L_delta, R_delta)
  classify_wedge(l_delta, r_delta)    -- True if both < WEDGE_THRESHOLD_MM (5 mm)
  main()                   -- cycle loop, per-cycle print, final RESULT line
```

S keepalive interval: 150 ms (same as drive_raw.py).
Drive phase duration per cycle: 1.5 s (enough to accumulate measurable enc at speed 200).

### Testing Plan

- Run against live robot on the bench: `--clean-stop --cycles 10` and
  `--watchdog-stop --cycles 20`.
- Confirm wedge-rate prints and script exits cleanly.
- No automated pytest test added: bench run IS the acceptance test.

### Documentation Updates

Module docstring and `--help` output serve as documentation. No separate
documentation file.
