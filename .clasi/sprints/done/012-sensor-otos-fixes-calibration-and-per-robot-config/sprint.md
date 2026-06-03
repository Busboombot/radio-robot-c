---
id: '012'
title: Sensor/OTOS Fixes, Calibration and Per-Robot Config
status: done
branch: sprint/012-sensor-otos-fixes-calibration-and-per-robot-config
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
- sprint-12-sensor-otos-fixes-calibration-per-robot-config.md
---

# Sprint 012: Sensor/OTOS Fixes, Calibration and Per-Robot Config

## Goals

Make the robot student-ready by fixing the sensor/telemetry/velocity feedback
stack, baking in correct per-robot calibration defaults, and building a
host-side per-robot config system that loads known-good calibration at connect
time. Verified on the playfield with OTOS and overhead camera as ground truth.

## Problem

After Sprints 007-011 brought up firmware, v2 protocol, and radio-relay, bench
testing revealed the robot is not yet student-ready:

- TLM `pose=` reports raw OTOS LSB (looks ~5x wrong), not the fused odometry
  in mm — breaking go-to.
- OTOS linear and angular scalars are never set at init, so the OTOS tracks
  with uncorrected scaling errors.
- Motor chip velocity (register 0x47) is stuck at ~30-33 mm/s regardless of
  actual speed due to tight-loop I2C interleaving on the Nezha (0x10), feeding
  junk to the velocity PID.
- Encoder and odometry caches are only refreshed while non-IDLE, so SNAP/TLM
  at rest is stale.
- Compiled defaults are wrong: trackwidth 120 (should be 126), no OTOS scalars,
  no per-direction turn gain.
- The host-side `_push_calibration()` speaks the dead pre-v2 protocol
  (KML/KMR/OO/OK verbs) and must be rewritten to v2.

The prior system (`/Volumes/Proj/proj/league-projects/scratch/radio-robot/`) is
the source of truth for per-robot JSON config and calibration scripts.

## Solution

Fix the firmware integration bugs and defaults (T01-T07), then build the full
host-side per-robot config system and calibration scripts (T08-T10), and do
a stakeholder-run end-to-end bench verification (T11).

The chip velocity fix (T04) requires debugging the I2C read context so that
`readSpeed` tracks actual speed rather than being stuck. The chip is known good
per vendor MakeCode; our tight-loop interleaving is the bug.

## Success Criteria

1. `pose=` in TLM reports fused odometry in mm (x~1000 after 1 m drive).
2. At boot, `OL`/`OA` reflect the config scalars without a host command.
3. `vel=` scales with commanded speed (not stuck at ~30); chip source 'C' shown.
4. `SNAP` at rest and hand-push update `enc=`/`pose=` immediately.
5. `GET tw` = 126; OTOS scalars at known-good defaults.
6. Post-connect `GET ml/mr/tw/OL/OA` matches active robot JSON.
7. Calibration scripts run end-to-end over relay and emit recommended values.
8. T11 bench verification passes (stakeholder-run on playfield).

## Scope

### In Scope

- Firmware: Config fields + SET/GET registry (T01)
- Firmware: OTOS scalars applied at init from config (T02)
- Firmware: TLM pose = fused odometry in mm (T03)
- Firmware: Fix chip readSpeed I2C context + encoder-delta fallback (T04)
- Firmware: Refresh encoders + odometry every tick including IDLE (T05)
- Firmware: Known-good compiled defaults + per-direction turn gain (T06)
- Firmware: OTOS mounting offset support (T07)
- Host: Per-robot config schema + data dir + loader finalize (T08)
- Host: Connect-time calibration push rewritten to v2 (T09)
- Host: Calibration scripts rewritten to v2 + relay (T10)
- Stakeholder end-to-end bench verification (T11)

### Out of Scope

- Navigation algorithm changes (go-to path following logic itself)
- New robot models beyond the nezha/tovez platform
- PID re-tuning (gains are correct; this sprint fixes the feedback sources)

## Test Strategy

- All firmware tickets: clean build (`mbdeploy build --clean`) + reflash to
  robot enum 2 (not relay enum 1).
- Unit tests: updated for trackwidth 120→126 default change (T06).
- Protocol tests: host unit test asserts v2 verbs emitted, no dead verbs (T09).
- Velocity test: `tests/test_readspeed_and_get_vel.py` updated (T04).
- Pose test: `test_tlm_stream.py`/`test_otos_fusion.py` assert mm-scale pose (T03).
- Hardware ACs: stakeholder-run on playfield during T11. All bench ACs in T01-T10
  that require the robot to be live are marked deferred to T11.

## Architecture Notes

See `architecture-update.md` for the full architecture analysis. Key decisions:

- Chip velocity is the primary feedback source; encoder-delta is the fallback.
  The chip 0x47 register works per vendor MakeCode; our read context is the bug.
- TLM `pose=` always reports fused odometry (mm, mm, centidegrees).
  `OP` retains raw OTOS LSB (clearly labeled).
- `mmPerDegL/R` (0.487/0.481) and OTOS LSB constants (0.305 mm, 0.005493 deg)
  are correct — do not change them.
- New RobotConfig fields added for OTOS scalars, per-direction turn gain/offset,
  OTOS mounting offset. All wired to SET/GET registry.
- RAM impact from new Config fields must be confirmed at first build (CODAL heap
  ceiling is tight).

## GitHub Issues

None.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Config fields + SET/GET registry: OTOS scalars + turn asymmetry | — |
| 002 | OtosSensor applies linear/angular scalars at init from config | 001 |
| 003 | TLM pose reports fused odometry (mm), not raw OTOS LSB | — |
| 004 | Fix chip readSpeed I2C context + encoder-delta fallback | — |
| 005 | Refresh encoders + odometry every tick (fix idle staleness) | 004 |
| 006 | Known-good compiled defaults + per-direction turn gain applied | 001 |
| 007 | OTOS mounting offset support | 001 |
| 008 | Per-robot config schema + data dir + loader finalize | — |
| 009 | Connect-time calibration push rewritten to v2 | 008, 001 |
| 010 | Calibration scripts rewritten to v2 + relay | 003, 004, 005, 008, 009 |
| 011 | End-to-end bench verification + record calibration | all |

Tickets execute serially in the order listed.
