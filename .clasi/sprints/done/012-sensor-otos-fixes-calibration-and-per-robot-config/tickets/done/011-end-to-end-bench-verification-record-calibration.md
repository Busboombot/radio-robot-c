---
id: '011'
title: End-to-end bench verification + record calibration
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
depends-on:
- '001'
- '002'
- '003'
- '004'
- '005'
- '006'
- '007'
- 008
- 009
- '010'
github-issue: ''
issue: sprint-12-sensor-otos-fixes-calibration-per-robot-config.md
completes_issue: true
---

# End-to-end bench verification + record calibration

## Description

STAKEHOLDER-RUN TICKET. No code changes. This ticket is a structured
verification checklist run on the playfield by the stakeholder after all T01-T10
firmware and host changes are complete.

Prerequisites before starting:
- All T01-T10 tickets are in `done`.
- Fresh clean build: `mbdeploy build --clean`.
- Robot flashed to **robot enum 2** (NOT relay enum 1).
- Relay connected (RAW250 mode).
- `data/robots/active_robot.json` points to the correct robot JSON.

## Verification Plan

Work through each check in order. Record the measured values. If any check fails,
identify which ticket introduced the regression and reopen that ticket.

### 1. Connect-time apply

Connect via `rogo connect` (relay mode).

- [ ] `GET tw` returns 126.
- [ ] `GET ml` returns approximately 0.487.
- [ ] `GET mr` returns approximately 0.481.
- [ ] `OL` (no arg) returns scalar approximately +50 (for linear scale 1.05).
- [ ] `OA` (no arg) returns scalar approximately -13 (for angular scale 0.987).
- [ ] No `KML`/`KMR`/`OO`/`OK` errors in the connect log.

### 2. Distance accuracy

- [ ] Issue `ZERO enc pose`, then `D 200 200 1000` (1 m forward).
- [ ] Tape-measure actual distance traveled: within a few percent of 1000 mm.
- [ ] `pose=` x in TLM is approximately 1000 mm (not ~3279).

### 3. Straight-line driving

- [ ] `S 200 200` for 3-5 seconds over a marked line.
- [ ] Minimal lateral drift (velocity PID operating correctly).
- [ ] `GET VEL` returns source 'C' (chip) for both wheels; values approximately 200.

### 4. Velocity scaling

- [ ] At `S 100 100`: `GET VEL` values approximately 100, source 'C'.
- [ ] At `S 200 200`: `GET VEL` values approximately 200, source 'C'.
- [ ] At `S 300 300`: `GET VEL` values approximately 300, source 'C'.
- [ ] Values are NOT stuck at ~30-33 mm/s.
- [ ] At idle (STOP): `GET VEL` approximately 0.

### 5. In-place turns — symmetry

- [ ] `ZERO enc pose`, then rotate 90° CCW. Measure heading vs. OTOS/camera. Record error.
- [ ] `ZERO enc pose`, then rotate 90° CW. Measure heading vs. OTOS/camera. Record error.
- [ ] CCW and CW errors are symmetric within tolerance (e.g. <5°).
- [ ] `ZERO enc pose`, then rotate 180° CCW and 180° CW. Both within tolerance.

### 6. Pose tracking and fused pose cross-check

- [ ] Drive a 0.5 m square (D 200 200 500, rotate 90°, repeat x4).
- [ ] Final `pose=` should be near (0, 0, 0) after returning to start.
- [ ] `OP` (raw OTOS LSB) cross-check: compare with tape/camera at several points.

### 7. Idle telemetry freshness

- [ ] Drive, then stop (IDLE mode).
- [ ] Issue `SNAP`: `enc=` and `pose=` reflect current position (not stale).
- [ ] Hand-push robot slightly; issue `SNAP` again: values update.
- [ ] No motor twitch or unintended movement during SNAP.

### 8. Go-to

- [ ] `ZERO enc pose`.
- [ ] `G 500 0 200` (go to x=500 mm, y=0).
- [ ] Robot arrives within `arriveTol`. `EVT done G` received.
- [ ] Final `pose=` x approximately 500.
- [ ] Camera ground truth matches final pose.

### 9. Calibration scripts (if time permits)

- [ ] Run `python host/calibrate_linear.py` over relay. Script completes without error.
- [ ] Recommended `otos_linear_scale` written to robot JSON.
- [ ] Run `python host/calibrate_angular.py` over relay. Script completes without error.
- [ ] Recommended `otos_angular_scale` and turn gains written to robot JSON.

## Calibration Recording

After verification, record the final calibrated values here:

| Parameter | Value | Method |
|-----------|-------|--------|
| `otos_linear_scale` | | calibrate_linear.py |
| `otos_angular_scale` | | calibrate_angular.py |
| `rotation_gain_neg` (CW) | | calibrate_angular.py |
| trackwidth | 126 | confirmed |
| `mm_per_wheel_deg_left` | 0.487 | confirmed (unchanged) |
| `mm_per_wheel_deg_right` | 0.481 | confirmed (unchanged) |

## Acceptance Criteria

- [ ] All 9 verification sections above pass.
- [ ] Calibrated values recorded in the table above.
- [ ] `data/robots/<robot>.json` updated with measured calibration values.
- [ ] Robot is student-ready: drives accurate distances, holds straight lines, turns symmetrically, go-to works.

## Testing

- **This ticket is stakeholder-run on the playfield.** No automated tests.
- **Prerequisite**: all T01-T10 must be in `done` before this ticket begins.
