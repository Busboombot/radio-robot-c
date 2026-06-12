---
id: '001'
title: "Hardware bench validation via Bench OTOS \u2014 turns, square, velocity profiles"
status: in-progress
use-cases:
- SUC-001
- SUC-002
- SUC-003
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Hardware bench validation via Bench OTOS — turns, square, velocity profiles

## Description

Validate the full firmware stack (v0.20260612.17 — sprint 030 N1–N16 correctness fixes
+ sprint 031 Bench OTOS) on the real robot. Firmware is already flashed. Robot is on
the bench stand; use `DBG OTOS BENCH 1` so the optical-odometry path runs in bench
mode. Drive three sequences, collect raw TLM logs, and write a validation verdict.

**This ticket is executed by the team-lead directly via rogo — do NOT dispatch to a
programmer agent.**

Port: `/dev/cu.usbmodem2121402` (relay). All rogo commands via
`rogo --port /dev/cu.usbmodem2121402`.

## Acceptance Criteria

- [ ] Robot responds to PING / ID and reports firmware v0.20260612.17 (or later)
- [ ] `DBG OTOS BENCH 1` accepted; OTOS data appears in STREAM TLM (`otos=` field present)
- [ ] **Sequence 1 — TURN closure**: Four TURN 9000 commands complete; TLM captured;
      total heading change ≤ 720 deg; clean stop after each turn (residual |v| ≤ 30 mm/s)
- [ ] **Sequence 2 — 300 mm square**: Four (D 300 + TURN 9000) cycles complete; TLM
      captured; tick-to-tick |dv| ≤ 120 mm/s; ekf_rej climb ≤ 20; clean stops
- [ ] **Sequence 3 — Velocity profiles**: D at 150/300/500 mm/s + T at 300 mm/s complete;
      TLM captured; no instant-max-speed start (v at tick 1 ≤ 60% of peak); |dv| ≤ 120 mm/s;
      heading drift ≤ 25 deg on straight runs; clean stops
- [ ] Raw TLM logs saved to `docs/bench-validation-032/` (one file per sequence)
- [ ] Written validation verdict in `docs/bench-validation-032/verdict.md`:
      "PASS" if all criteria met, otherwise list each pathology and file a new issue for each

## Implementation Plan

### Setup

```
rogo --port /dev/cu.usbmodem2121402 cmd PING
rogo --port /dev/cu.usbmodem2121402 cmd ID
rogo --port /dev/cu.usbmodem2121402 cmd "DBG OTOS BENCH 1"
rogo --port /dev/cu.usbmodem2121402 cmd "STREAM 30 fields=mode,pose,twist,enc,ekf_rej,otos"
```

Verify `otos=` field appears in TLM before proceeding. If OTOS field absent, stop
and investigate (possible: firmware not Bench-OTOS capable, or DBG command rejected).

### Sequence 1: TURN closure (4 × 90 deg CCW)

Zero state, then drive four TURN 9000 commands in sequence, waiting for `EVT done`
after each. Capture all STREAM output to `docs/bench-validation-032/turn_closure.log`.

Check after all four turns:
- Heading field from `pose=` (centidegrees): total change across all turns / 100 → degrees
- Omega from `twist=` (mrad/s): divide by 1000 → rad/s; should never exceed yaw rate cap
- Velocity `vel=` or `twist` v at the last frame of each turn ≤ 30 mm/s (clean stop)

### Sequence 2: 300 mm square

Zero state, then four cycles of `D 300 250 250` + `TURN 9000`. Wait for `EVT done`
after each segment. Capture to `docs/bench-validation-032/square.log`.

Check:
- Tick-to-tick |dv| (velocity v from `twist=` first field, mm/s): max jump ≤ 120
- `ekf_rej=` field: difference between first and last frames ≤ 20
- Clean stop (residual v ≤ 30 mm/s) after each segment

### Sequence 3: Velocity profiles

Four separate runs; zero state before each; capture individually:

| Label | Command | Log file |
|---|---|---|
| slow | `D 250 150 150` | `vel_slow.log` |
| medium | `D 400 300 300` | `vel_medium.log` |
| fast | `D 500 500 500` | `vel_fast.log` |
| timed | `T 1500 300 300` | `vel_timed.log` |

Check each:
- First TLM frame v ≤ 60% of peak v (no instant-max start)
- |dv| per tick ≤ 120 mm/s
- Heading drift (pose heading change / 100 → degrees) ≤ 25 deg for straight runs
- Clean stop (v ≤ 30 mm/s after EVT done)

### Verdict

Write `docs/bench-validation-032/verdict.md` with:
- Firmware version confirmed
- Pass/fail for each criterion above
- Overall: PASS or list of pathologies
- For each pathology: open a new issue in `.clasi/issues/`

## Testing

- **Existing tests to run**: N/A (hardware validation ticket)
- **New tests to write**: None (hardware only; sim harness is T002)
- **Verification command**: Manual — review TLM logs and verdict doc
