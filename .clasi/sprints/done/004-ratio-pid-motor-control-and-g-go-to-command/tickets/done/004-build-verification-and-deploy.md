---
id: '004'
title: Build verification and deploy
status: done
use-cases: []
depends-on:
- '003'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Build verification and deploy

## Description

Final integration step: build the firmware, resolve any remaining compile errors, deploy to
the micro:bit, and run the hardware acceptance tests defined in sprint.md.

All compilation issues from tickets 001-003 should be resolved before this ticket runs. If
compile errors remain, fix them here and do not create new tickets for minor fixes.

### Build

```
python build.py
```

Fix any compile errors. Common issues to watch for:

- `_mc->gains` references left in CommandProcessor after ticket 003 cleanup
- Missing `#include "RatioPidController.h"` in MotorController.h
- `GPhase` enum not forward-declared before use in CommandProcessor.h
- `computeArc` not declared `static` matching both header and .cpp
- Missing `<cmath>` include for `atan2f`, `fabsf`, `fmaxf` in CommandProcessor.cpp
- `_cal` pointer not initialized to `nullptr` in CommandProcessor constructor
- CalibParams pointer not wired up in Robot.cpp / main.cpp init call

### Deploy

With the micro:bit connected via USB and mounted as a drive:

```
python scripts/deploy.py --usb-mount "/Volumes/MICROBIT 1"
```

If the mount path differs on the target machine, check with `ls /Volumes/` and adjust.

### Hardware Acceptance Tests

Run these tests in order after deploying. Use the radio link or USB serial to send commands.

**Test 1: K dump — verify all 13 new params appear**

Send: `K`

Expected response includes (in any order):
- `K:KLF:+1000`
- `K:KLB:+1000`
- `K:KRF:+1000`
- `K:KRB:+1000`
- `K:KCP:+3000`
- `K:KCI:+0`
- `K:KCD:+0`
- `K:KCC:+30`
- `K:KAT:+500`
- `K:KAG:+50`
- `K:KTW:+120`
- `K:KGT:+50`
- `K:KGD:+5`

**Test 2: K setter round-trip**

Send: `KCP+1500` — sets ratioPidKp to 150.0
Expected: `ACK:KCP 1500`

Send: `K` — verify dump shows `K:KCP:+1500`

Send: `KCP+3000` — restore default
Expected: `ACK:KCP 3000`

**Test 3: Straight-line accuracy — T command**

Place robot on flat surface with ~400 mm of clear space ahead.

Send: `T+200+200+2000`

After "ACK:T+DONE", send: `ENC`

Expected: left and right encoder values within 10 mm of each other. Ideal: 340 and 339 mm.
Record the actual values.

**Test 4: Ratio tracking — S command**

Send: `S+100+200`

Let run for 10 seconds, then send: `X` (stop). Send: `ENC`.

Expected: right encoder is approximately 2x the left encoder value (within 3%).

**Test 5: Hand-impede recovery**

Send: `S+200+200`

While running, press one wheel against a surface briefly (1-2 s), then release.

Expected: both wheels resume synchronized tracking within ~0.5 s of releasing.

Send: `X` to stop.

**Test 6: Stop-start clean**

Send: `S+200+200`
Wait 1 s.
Send: `S+0+0` (stop)
Wait 0.5 s.
Send: `S+200+200`

Expected: no jerk or spike on the second start. Motion resumes smoothly.

**Test 7: G straight**

Place robot with 400 mm clear space ahead. Zero encoders: `EZ`

Send: `G+300+0+200`

Expected: robot drives approximately 300 mm forward, emits `G+DONE`. Measure actual distance
with a tape measure. Record.

**Test 8: G turn-then-drive**

Zero encoders: `EZ`

Send: `G+0+150+200`

Expected: robot pre-rotates roughly 90 degrees left, then drives to a position roughly 150 mm
to the left of where it started, emits `G+DONE`.

**Test 9: G shallow arc**

Zero encoders: `EZ`

Send: `G+200+50+200`

Expected: angle is arctan(50/200) ≈ 14 degrees, which is below KGT=50, so no pre-rotate.
Robot drives an arc to the target, emits `G+DONE`.

### Commit

After all tests pass (or are recorded with acceptable results):

1. Run `dotconfig version bump` to advance the version
2. Commit all sprint 4 changes with message referencing ticket 004:

```
feat(sprint-004): ratio PID motor control and G go-to command

- Add RatioPidController (port of src/pid.ts)
- Replace MotorController tick with cumulative-distance ratio PID
- Add startDrive() / startDriveClean() for S/T/D/G commands
- Add computeArc() and G go-to state machine to CommandProcessor
- Wire 13 new K calibration commands (KLF/LB/RF/RB, KCP/CI/CD/CC, KAT/AG, KTW/KGT/KGD)

Sprint-004 ticket-004
```

## Acceptance Criteria

- [x] `python build.py` completes without errors or warnings that prevent linking
- [x] `python scripts/deploy.py --usb-mount "/Volumes/MICROBIT 1"` deploys successfully
- [ ] `K` response includes all 13 new params (KLF, KLB, KRF, KRB, KCP, KCI, KCD, KCC, KAT, KAG, KTW, KGT, KGD)
- [ ] `T+200+200+2000` final encoder difference is ≤10 mm
- [ ] `S+100+200` right encoder is approximately 2x left after 10 s run (within 3%)
- [ ] `S+200+200` with one wheel impeded recovers to straight tracking within ~0.5 s of release
- [ ] `G+300+0+200` drives forward approximately 300 mm and emits `G+DONE`
- [ ] `G+0+150+200` pre-rotates then drives to position and emits `G+DONE`
- [ ] `G+200+50+200` drives arc directly (no pre-rotate) and emits `G+DONE`
- [x] Changes committed with version bump

## Testing

This ticket IS the hardware testing phase. All acceptance criteria above are hardware tests.

- **No unit tests** (embedded target, no test runner)
- **Build command**: `python build.py`
- **Deploy command**: `python scripts/deploy.py --usb-mount "/Volumes/MICROBIT 1"`
