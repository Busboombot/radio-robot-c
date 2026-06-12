---
id: 008
title: 'N8+N9: Sensor freshness gate in TLM and same-tick OTOS fusion skip'
status: done
use-cases:
- SUC-007
depends-on: []
github-issue: ''
issue: fr2-n8-n9-sensor-validity.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# N8+N9: Sensor freshness gate in TLM and same-tick OTOS fusion skip

## Description

N8: `lineRead()` / `colorRead()` set `valid = true` on first success and never clear
or age it (`Robot.cpp:263-286`). `buildTlmFrame` gates on the bit alone
(`Robot.cpp:339-342`). A line or color sensor that wedges after boot keeps publishing
its last-good values indefinitely. Freshness fields (`lastUpdMs`, `lagMs`) exist and
are maintained but are never consulted. Same problem for the raw `otos=` TLM field —
it keeps emitting the last-good OTOS pose while `otos.valid` is false.

N9: `Robot::otosCorrect()` checks `otos.lastReadOk()` *before* this tick's reads
(`Robot.cpp:209`), but `_lastReadOk` is updated by `readXYH()` *during*
`readTransformed()` (`OtosSensor.cpp:268`). If this tick's I2C transaction fails,
`raw[6]={0}` decodes to pose (0,0,0) / velocity (0,0) and is passed to
`correctEKF()` this tick; the failure is only caught on the next call. Near (0,0) a
zero-filled read is accepted by the Mahalanobis gate, and a zero velocity update
drags `fusedV` down (the D9 symptom).

## Acceptance Criteria

- [x] `buildTlmFrame` gates line/color sensor fields on freshness:
      `now - lastUpdMs <= 2 * lagMs` (fields already present — no new fields needed).
- [x] `buildTlmFrame` gates the raw `otos=` field on freshness similarly.
- [x] `readTransformed()` and `readVelocityTransformed()` return a success bool.
- [x] `Robot::otosCorrect()` skips fusion when the same-tick read fails (checks the
      return value of `readTransformed`, not the stale `lastReadOk()`).
- [x] New sim test: stalled line/color sensor (frozen mock) stops appearing in TLM
      after ~2 * lagMs.
- [x] New sim test: same-tick OTOS read failure does not fuse (0,0,0)/(0,0) into
      the EKF (assert EKF state unchanged).
- [x] `python3 build.py` clean build passes.
- [x] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes.

## Implementation Plan

### Approach

N8 is a two-line change in `buildTlmFrame`: replace the `valid` bit check with a
freshness expression. N9 requires modifying `readTransformed` / `readVelocityTransformed`
signatures and updating the call site in `Robot::otosCorrect`.

### Files to modify

- `source/robot/Robot.cpp`
  - `buildTlmFrame()`: replace `sensor.valid` gates for line/color/otos with
    freshness expression (`now - sensor.lastUpdMs <= 2 * sensor.lagMs`).
  - `otosCorrect()`: use the return value of `readTransformed()` (or an explicit
    read-and-check) to gate fusion for this tick.
- `source/sensors/OtosSensor.h` — change `readTransformed()` /
  `readVelocityTransformed()` signatures to return `bool`.
- `source/sensors/OtosSensor.cpp`
  - `readTransformed()`: return true on success, false if the underlying I2C
    read fails (before or while updating `_lastReadOk`).
  - `readVelocityTransformed()`: same pattern.
- `host_tests/` or `host/tests/` — add stale-sensor TLM test and same-tick OTOS
  failure test.

### Testing plan

Run: `uv run --with pytest python -m pytest host_tests/ host/tests/ -v`

Build: `python3 build.py` (clean).

### Notes

- Independent of tickets 001-007 (only sensor/Robot.cpp and OtosSensor.cpp).
- OTOS is mandatory (per project knowledge); this ticket makes the failure path
  safe, not optional.
- Do not change the `lastReadOk()` API — it remains valid for historical/TLM use.
  The new return value from `readTransformed` is for the same-tick gate only.
