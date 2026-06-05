---
id: 009
title: 'Hardware bench verification: single-loop firmware end-to-end (closes sprint
  013 deferred gate)'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-004
- SUC-005
- SUC-006
- SUC-007
depends-on:
- 008
github-issue: ''
issue: plan-single-cooperative-main-loop-abandon-fibers.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Hardware bench verification: single-loop firmware end-to-end (closes sprint 013 deferred gate)

## Description

Deploy the completed single-loop firmware to the robot on the hardware bench
and run the full verification gate described in `docs/hardware-bench-testing.md`
and the design issue. This ticket closes the sprint 013 bench verification
that was deferred into this sprint.

The robot is on a stand with wheels off the ground — safe to drive motors
freely during verification.

**Note on pytest command**: The correct invocation is
`uv run --with pytest python -m pytest` (NOT `uv run pytest`, which resolves
the wrong Python and fails on missing `pyserial`).

## Files Modified

None — this is a verification-only ticket. If bugs are found during bench
testing, they are fixed in the relevant source ticket before this one is
marked done.

## Acceptance Criteria

All criteria are **stakeholder-bench** (require the robot on the stand) unless
marked (CI).

### Pytest suite (CI)
- [ ] (CI) `uv run --with pytest python -m pytest` passes clean with no
  failures or errors. Run from the repo root with the robot NOT connected
  (the suite mocks hardware).

### Deploy gate
- [ ] Clean build (`mbdeploy deploy --build --clean`) succeeds with no warnings
  about removed symbols (`controlFiberFn`, `gRobot`, `create_fiber`,
  `controlTick`, `telemetryTick`).
- [ ] HELLO banner appears after boot; boot icon displays on LED matrix.

### Basic drive and sensors
- [ ] `S 200 200` → both wheels spin; `GET VEL` returns plausible mm/s values
  (approximately 200 mm/s per wheel); encoders advance.
- [ ] `GET ENC` returns non-zero values after driving.
- [ ] `X` (stop) → wheels stop; encoders hold.

### Control rate
- [ ] Effective control rate ≥ 40 Hz confirmed. Method: add a temporary
  tick-counter field to the TLM frame or use `GET VEL` rapid-fire polling
  to measure update frequency. The rate should be visibly better than the
  prior two-fiber ~40 Hz (i.e., ≥ 50 Hz is expected given the ~20 ms
  per-wheel sample period and < 1 ms control task cost).

### Streaming watchdog (inline EVT)
- [ ] Send `S 200 200` then stop sending for `sTimeoutMs` → `EVT safety_stop`
  arrives on the originating channel; motors stop.
- [ ] `EVT safety_stop` is delivered promptly (within one drive-advance task
  iteration after the deadline — no ring-buffer drain delay).

### Timed and distance drives
- [ ] `T 200 200 1000` → wheels drive for ~1 s; `EVT done T` arrives on the
  originating channel after completion.
- [ ] `D 200 200 500` → wheels drive ~500 mm; `EVT done D` arrives.

### GO_TO command
- [ ] `G <x> <y> <speed>` → robot pursues the relative target; `EVT done G`
  arrives on completion. Pose converges under OTOS correction (if OTOS present).

### Lag tuning (SUC-004, SUC-006)
- [ ] `SET lag.line 0` → `GET LS` (or TLM line fields) updates every loop
  iteration (fresh per-loop reads).
- [ ] `SET lag.line 500` → line sensor updates ~2×/second while control rate
  is unaffected (confirm with `GET VEL` still plausible).
- [ ] Same test with `lag.color`.
- [ ] `GET lag.otos` returns `100` (default); `SET lag.otos 200; GET lag.otos`
  returns `200`.

### I2C ordering stress test (Open Question 1)
- [ ] With sensor tasks running (line, color, OTOS at default lags), drive
  `S 200 200` for 5 seconds. Observe `GET ENC` / `GET VEL` for corruption
  (sudden large jumps, sign flips, implausible zeros). No corruption confirms
  the ordering rule keeps sensor I2C outside the motor's pending-read window.

### Radio stress test (SUC-004)
- [ ] Stream drive commands rapidly over radio while `S 200 200` is active.
  Control rate holds steady (no observable pulsing or velocity anomalies);
  commands are serviced (confirm `GET VEL` plausible throughout).

### No motor throb
- [ ] Steady `S 200 200` for 5 seconds: no visible pulsing of wheel speed.
  `GET VEL` should read approximately constant (within ±20 mm/s) across
  repeated polls.

## Verification Procedure

```
# 1. Pytest
uv run --with pytest python -m pytest

# 2. Clean build + flash
mbdeploy probe
mbdeploy deploy --build --clean

# 3. Open serial terminal at 115200 baud and run the command sequence above.
#    After each step, observe the response and confirm the acceptance criterion.
```

## Testing Plan

- All acceptance criteria above are the test plan for this ticket.
- If any criterion fails, the programmer investigates, fixes the relevant
  source ticket (001–008), re-runs pytest, re-flashes, and re-verifies.
- The ticket is not marked done until every criterion passes.
