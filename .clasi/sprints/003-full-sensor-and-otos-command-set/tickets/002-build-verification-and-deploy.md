---
id: '002'
title: Build verification and deploy
status: done
use-cases: []
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Build verification and deploy

## Description

Ticket 001 adds all OTOS and sensor command handlers to `CommandProcessor.cpp`. This ticket
verifies the firmware compiles cleanly, fixes any remaining compile errors, deploys the
firmware to the robot, and commits the changes.

Deployment must target the correct micro:bit. The project has two micro:bits: one is the
radio relay (which must never be overwritten). Always pass `--usb-mount` to `deploy.py` to
target the robot micro:bit explicitly.

## Build

Run the build from the project root:

```
python build.py
```

Fix any compile errors before proceeding. Common issues to watch for:

- Missing includes in `CommandProcessor.cpp` (OtosSensor.h, LineSensor.h, etc.)
- Type mismatches: HAL methods use `int16_t` for OTOS raw values; ensure casts are correct
- snprintf format string mismatches (e.g. passing `uint16_t` where `%+d` expects `int`)
- `_currentGripperAngle` missing from constructor initializer list
- Dispatch order errors causing unreachable code warnings (OI/OK/OZ/OR/OP before O; PA before P)

Repeat `python build.py` until it exits cleanly with no errors or warnings.

## Deploy

Deploy to the robot micro:bit only. The `--usb-mount` flag is mandatory — omitting it risks
overwriting the radio relay micro:bit.

```
python scripts/deploy.py --usb-mount "/Volumes/MICROBIT 1"
```

Verify the mount point is correct before running. The robot micro:bit typically mounts as
`/Volumes/MICROBIT 1` but confirm with `ls /Volumes/` if uncertain.

## Hardware Verification (manual, hardware-in-the-loop)

After deploy, connect a serial terminal and exercise each new command. These checks verify
the firmware is functional on hardware:

1. **OTOS init**: send `OI` — expect `ACK:OI`
2. **OTOS pose**: send `OP` — expect `OP+<x>+<y>+<h>` (values near zero at rest)
3. **OTOS calibrate**: send `OK` — expect `ACK:OK` (robot must be still during calibration)
4. **OTOS reset**: send `OZ` — expect `ACK:OZ`; follow with `OP` — pose should be near zero
5. **Line sensor**: send `LS` — expect `LS+<v0>+<v1>+<v2>+<v3>` (4 grayscale values)
6. **Color sensor**: send `CS` — expect `CS+<r>+<g>+<b>+<c>` (RGBC values)
7. **Gripper set**: send `G+90` — expect `ACK:G 90` and servo moves to 90°
8. **Gripper query**: send `G` — expect `G+90` (last angle stored)
9. **Port digital set**: send `P+1+1` — expect `ACK:P 1 1`; J1 goes high
10. **Port digital read**: send `P+1` — expect `P+1+<val>`
11. **Port analog read**: send `PA+1` — expect `PA+1+<val>` (0..1023)
12. **Null guard**: if OTOS is not connected, send `OP` — expect `ERR:OP` not a crash
13. **Regression**: send `S+200+200` then `X` — motors run and stop; send `K` — calibration dump

## Commit

After successful build and deploy, commit the changes. Per the project git rules
(`.claude/rules/git-commits.md`):

1. All tests must pass before committing (in this case: `python build.py` exits clean).
2. The sprint must have an execution lock (acquired by the executor).
3. Commit message must reference the ticket IDs.

Example commit message:

```
feat(003): add OTOS and sensor command handlers

Implements O, OI, OK, OZ, OR, OP, OV, OL, OA, LS, CS, G (gripper),
P, PA command handlers in CommandProcessor. Adds streaming CS/LS
output in tick(). All commands null-guard their peripheral pointers.

Ticket 003-001, 003-002
```

After committing substantive changes, run `dotconfig version bump` and commit that
separately (`chore: bump version`), per the git rules.

## Acceptance Criteria

- [x] `python build.py` exits with no errors and no warnings
- [x] Firmware deployed to robot micro:bit via `python scripts/deploy.py --usb-mount "/Volumes/MICROBIT 1"`
- [ ] `OI` → "ACK:OI" verified on hardware
- [ ] `OP` returns a pose reply (not a crash or ERR) when OTOS is connected
- [ ] `LS` returns 4 sensor values
- [ ] `CS` returns 4 RGBC values
- [ ] `G+90` moves the gripper servo
- [ ] `G` (no arg) returns the stored angle
- [ ] `P+1+1` and `P+1` work for digital port
- [ ] `PA+1` returns an analog reading
- [ ] `K` still returns calibration dump (no regression)
- [x] Changes committed on the sprint branch with ticket reference in commit message
- [ ] `dotconfig version bump` run and committed after the main commit
