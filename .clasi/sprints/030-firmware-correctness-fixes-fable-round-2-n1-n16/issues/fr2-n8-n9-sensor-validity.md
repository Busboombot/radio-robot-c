---
status: in-progress
sprint: '030'
tickets:
- 030-008
---

# FR2-N8/N9 (Med) — Sensor validity: sticky line/color + TLM staleness, and one-tick-stale OTOS gate

## Context

Source: `docs/code_review/2026-06-12-Fable-correctness-review/findings.md` §N8, §N9.

**N8:** `lineRead()`/`colorRead()` set `valid = true` on first success and never clear
or age it (`Robot.cpp:263-286`); `buildTlmFrame` gates on the bit alone
(`Robot.cpp:339-342`). A sensor that wedges after boot publishes its last values
forever. Freshness fields (`lastUpdMs`, `lagMs`) exist but are never consulted. Same
for the raw `otos=` TLM field, which keeps emitting the last-good pose while
`otos.valid` is false.

**N9:** `Robot::otosCorrect()` checks `otos.lastReadOk()` *before* this tick's reads
(`Robot.cpp:209`), but `_lastReadOk` is updated by `readXYH()` *during*
`readTransformed()` (`OtosSensor.cpp:268`). If this tick's I2C fails, `raw[6]={0}`
decodes to pose (0,0,0)/velocity (0,0) and is passed to `correctEKF()` this tick; the
failure is only caught next call. Near (0,0) a zero-filled read is accepted and a
zero velocity update drags `fusedV` down (the D9 symptom, one tick at a time).

## Fix

- N8: gate the line/color and raw `otos=` TLM fields on freshness:
  `now − lastUpdMs <= 2×lagMs` (fields already exist).
- N9: have `readTransformed`/`readVelocityTransformed` return a success flag; skip
  fusion on same-tick failure (don't fuse the zero-filled decode).

## Acceptance

- A stalled line/color sensor stops being published in TLM after ~2×lag (sim test
  with a frozen mock).
- A same-tick OTOS read failure does not fuse a (0,0,0)/(0,0) sample (sim test).
