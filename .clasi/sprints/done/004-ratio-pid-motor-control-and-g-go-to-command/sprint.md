---
id: '004'
title: Ratio PID Motor Control and G Go-To Command
status: done
branch: sprint/004-ratio-pid-motor-control-and-g-go-to-command
use-cases: []
issues:
- nezha-ratio-pid-algorithm
- firmware-ratio-pid-and-g-command
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 004: Ratio PID Motor Control and G Go-To Command

## Goals

Improve straight-line accuracy to <0.3% error via cumulative-distance ratio PID, and add
autonomous XY navigation via the `G` go-to command. After this sprint, `T+200+200+2000`
produces a final encoder difference of ≤10 mm, and `G+300+0+200` drives the robot to the
target position and emits `G+DONE`.

## Problem

Sprint 2's MotorController uses simple PI + FF. The Nezha motors are dumb DC motors with no
onboard velocity loop, so the two wheels drift relative to each other — particularly under
varying load. Confirmed TypeScript results show 0.3% straight-line error with ratio PID
(340/339 mm over 2 s), which is unachievable with the PI + FF approach. Additionally, the
`G` go-to command requires arc math that only makes sense once the ratio PID is reliable
enough to track a computed arc without accumulating heading error.

## Solution

**Ratio PID (Issue B: nezha-ratio-pid-algorithm):**
Create `RatioPidController` class (port of `src/pid.ts`). Replace MotorController's tick body
with the cumulative-distance ratio PID algorithm specified in the issue. Add `startDrive()` /
`startDriveClean()` startup methods. Add 10 new K calibration parameters. The public interface
of MotorController is unchanged from sprint 2 — only the tick body changes.

**G command / arc math (Issue C: firmware-ratio-pid-and-g-command):**
Add `computeArc(tx, ty, trackwidthMm)` to CommandProcessor. Add the two-phase G state machine
(pre-rotate phase + arc-drive phase) to CommandProcessor::tick(). Add 3 new K parameters
(KTW, KGT, KGD). This sprint's G command replaces sprint 3's single-arg gripper G handler —
the go-to G has three arguments (`G+X+Y+Speed`).

## Success Criteria

- `K` response includes all 10 new ratio PID params: `KLF`, `KLB`, `KRF`, `KRB`, `KCP`, `KCI`, `KCD`, `KCC`, `KAT`, `KAG`
- `K` response also includes `KTW`, `KGT`, `KGD`
- `T+200+200+2000` — 2-second timed run; final encoder difference ≤10 mm (matching confirmed 340/339 mm TypeScript result)
- `S+200+200` — both encoders finish within 5 mm of each other after a 1 m straight run
- `S+100+200` — right encoder travels ~2× left distance; ratio holds within 3% over 1 m
- `S+200+200` with one wheel impeded by hand mid-run — robot recovers to straight tracking within ~0.5 s of releasing
- `S+0+0` (stop) resets integrators; next S command starts clean
- `G+300+0+200` — drives 300 mm straight forward, emits `G+DONE`
- `G+0+150+200` — pre-rotates ~90°, then drives to position, emits `G+DONE`
- `G+200+50+200` — shallow angle (~14°), uses arc directly without pre-rotating
- `G+300+0+200` with ratio PID active — arc tracks correctly without accumulating heading error

## Scope

### In Scope

**Control (`source/control/`)**
- `RatioPidController.h/.cpp` — standard discrete PID with anti-windup integral clamp:
  - `integral += kI * error * dtS; integral = clamp(integral, -iClamp, +iClamp)`
  - `deriv = (error - prevError) / dtS` (0 on first call)
  - `output = kP * error + integral + kD * deriv`
  - Public `integral` field (slower-wheel adjustment reads it directly)
  - `reset()` method
- `MotorController.h/.cpp` — replace tick body with driveTick algorithm:
  - Add `startDriveClean(leftMms, rightMms)` — used by T, D, G commands; resets PID state, snapshots encoders, computes ratio
  - Add `startDrive(leftMms, rightMms)` — used by S command only; re-seeds cmdEncStart to preserve accumulated ratio history on keepalive re-sends
  - Per-tick algorithm: read encoders → cumulative deltas → normalized error → PID update → FF base PWM with per-direction scale factors (kScaleLF/LB/RF/RB) → slower-wheel adjustment → clamp and apply PWM
  - New fields in `MotorGains` or `CalibParams`: kScaleLF=1.0, kScaleLB=1.0, kScaleRF=1.0, kScaleRB=1.0, ratioPid.kP=300.0, ratioPid.kI=0.0, ratioPid.kD=0.0, ratioPid.iClamp=30, kAdjThreshold=0.5, kAdjGain=0.05

**Types (`source/types/Config.h`)**
- Add `KLF`, `KLB`, `KRF`, `KRB`, `KCP`, `KCI`, `KCD`, `KCC`, `KAT`, `KAG` to CalibParams
- Add `KTW` (trackwidth mm, default 120), `KGT` (turn threshold degrees, default 50), `KGD` (done tolerance mm, default 5)

**App (`source/app/CommandProcessor.h/.cpp`)**
- `computeArc(tx, ty, trackwidthMm)` — robot always at (0,0,0); math: `cross = -ty; R = (tx²+ty²)/(2*ty); alpha = atan2(ty, tx+R); leftMm = (R-W/2)*alpha; rightMm = (R+W/2)*alpha`
- G command (replaces sprint 3's single-arg gripper G): `G+X+Y+Speed` — two-phase state machine in tick():
  - Phase 1 (pre-rotate): if `|atan2(Y,X)| > KGT`, rotate to face target then advance to phase 2
  - Phase 2 (arc drive): call `computeArc`, issue `startDriveClean` at commanded speed, track encoder targets; when both encoders within ±KGD mm of targets: stop, emit `G+DONE`
- New K setters in command table: `KLF`, `KLB`, `KRF`, `KRB`, `KCP`, `KCI`, `KCD`, `KCC`, `KAT`, `KAG`, `KTW`, `KGT`, `KGD`
- `startDrive()` called by S command (keepalive re-send path); `startDriveClean()` called by T, D, G commands

**Important algorithm constraints (from confirmed TypeScript behavior):**
- Track cumulative distance since command start, not instantaneous velocity
- Normalize error as fraction of expected distance (dimensionless): `normErr = (expected - fasterDelta) / max(1, expected)`
- Always correct the faster wheel only; slower wheel runs at pure FF (plus optional slow-down adj)
- FF is the primary drive signal: `basePwm = kFF * |commandedSpeed|`. PID is correction only
- Do NOT reset cmdEncStart on every S re-send (keepalive); use startDrive() re-seeding instead
- `min(kFF * speed, 22)%` stall threshold — minimum reliable speed is ~50 mm/s at kFF=0.15

### Out of Scope

- Navigation layer changes (sprint 5 PathFollower/PoseProvider unchanged)
- ExternalCameraPoseProvider
- Closed-loop arc recompute during G command (compute arc once; ratio PID handles tracking)

## Test Strategy

Hardware-in-the-loop:

1. Build and deploy
2. Straight-line accuracy: `T+200+200+2000` — read `ENC` immediately after; left-right difference ≤10 mm
3. Ratio test: `S+100+200` 10 s run — right encoder should be ~2× left; check ratio within 3%
4. Hand-impede test: `S+200+200`, push one wheel sideways briefly, release — both wheels resume synchronized tracking
5. Stop-start clean: `S+200+200`; `S+0+0`; `S+200+200` again — no jerk on second start
6. K dump: `K` response includes all 13 new params with defaults
7. G straight: `G+300+0+200` — robot drives 300 mm forward, emits `G+DONE`; measure actual distance
8. G turn-then-drive: `G+0+150+200` — robot pre-rotates ~90° then drives, emits `G+DONE`
9. G shallow: `G+200+50+200` — no pre-rotate (angle < KGT); drives arc, emits `G+DONE`
10. K tune: `KCP 150` sets ratio kP to 150; `K` confirms; re-run straight test with degraded gains

## Architecture Notes

**MotorController public interface is unchanged.** Sprint 2 callers (`CommandProcessor::tick`,
`CommandProcessor::process`) call `setTarget()` and `stop()` identically. Sprint 5 adds
`startDrive()` and `startDriveClean()` which CommandProcessor calls at command-start time
(not every tick). The tick body replacement is an internal change with no visible API change.

**`startDrive()` re-seeding prevents keepalive jerk.** On S re-sends, instead of resetting
deltas to zero (which makes the PID think wheels are perfectly synced), the algorithm re-seeds
`cmdEncStart` so existing encoder deltas represent the correct ratio for the new command. The
re-seeding logic: `seedFaster = max(prevDeltaFaster, cmdFasterAbs); seedSlower = seedFaster / newRatio;
cmdEncStartFaster = curFaster - sign(newFasterSpeed) * seedFaster`.

**G command replaces gripper G from sprint 3.** Sprint 5 removes the single-arg gripper handler
and installs the three-arg go-to handler. The gripper is only controllable via the gripper
servo directly (Robot member) or via a future dedicated command if needed. Note: this is a
deliberate scope trade-off — the gripper G command is sacrificed for the go-to G command since
the protocol uses the same prefix. If gripper control is needed post-sprint-5, a different
prefix (e.g. `GR`) would be added.

**Arc math is stateless.** `computeArc()` is a pure function — no side effects, no state.
It is called once at G command start. The ratio PID handles wheel tracking during the arc;
no closed-loop arc recomputation occurs.

**Issue references:**
- `.clasi/issues/nezha-ratio-pid-algorithm.md` — authoritative ratio PID algorithm spec (all tick math, startDrive/startDriveClean logic, calibration defaults)
- `.clasi/issues/firmware-ratio-pid-and-g-command.md` — arc math and G state machine spec
- `radio-robot/src/pid.ts` — TypeScript PidController to port
- `radio-robot/src/nezha.ts` — TypeScript driveTick and startDrive implementations (confirmed working 2026-05-21)

## GitHub Issues

None linked yet.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Add ratio PID params to Config.h and create RatioPidController | — |
| 002 | Replace MotorController tick body with ratio PID algorithm | 001 |
| 003 | Add computeArc and G go-to command to CommandProcessor | 001, 002 |
| 004 | Build verification and deploy | 003 |

Tickets execute serially in the order listed.
