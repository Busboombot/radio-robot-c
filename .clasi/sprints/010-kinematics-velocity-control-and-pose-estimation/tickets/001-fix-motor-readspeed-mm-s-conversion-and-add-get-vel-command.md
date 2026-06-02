---
id: '001'
title: Fix Motor::readSpeed mm/s conversion and add GET VEL command
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: kinematics-velocity-control-layer.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Fix Motor::readSpeed mm/s conversion and add GET VEL command

## Description

`Motor::readSpeed` currently converts the 0x47 register raw value using
`floor(raw/3.6)*0.01 * lapsToMmScale`, which produces values ~11× too high
and depends on a provisional `lapsToMmScale` config field. The correct
conversion mirrors `readEncoder`: `mm/s = (raw / 10.0) * mmPerDeg * sign`,
since register 0x47 uses the same angular-velocity unit (tenths of degrees/s)
as register 0x46 uses for angle.

This ticket corrects the conversion, deletes `lapsToMmScale` from
`RobotConfig`, and adds a `GET VEL` command (v2 protocol) so the host can
observe per-wheel velocity and source flags for bench verification and PID
tuning.

## Acceptance Criteria

- [ ] `Motor::readSpeed()` uses `mm/s = (raw / 10.0f) * mmPerDeg * sign`
  where `mmPerDeg` = `cfg.mmPerDegL` (M2/left) or `cfg.mmPerDegR` (M1/right),
  and `sign` = `_lastDir`. The old `floor(raw/3.6f)*0.01f * lapsToMmScale`
  path is deleted.
- [ ] `lapsToMmScale` field is deleted from the `RobotConfig` struct in
  `source/types/Config.h` and from `defaultRobotConfig()`.
- [ ] The `kRegistry[]` entry for `lapsToMmScale` is removed from
  `CommandProcessor.cpp`.
- [ ] `GET VEL` command (or `vel=` TLM sub-field) returns per-wheel mm/s
  (two values) and a source flag indicating chip (`C`) vs encoder-delta (`E`)
  for each wheel: e.g. `OK GET vel=vL:C,vR:E`.
- [ ] [BENCH] At a steady commanded speed (e.g. `S 200 200`), chip velocity
  and encoder-delta velocity agree within 15% in magnitude and match in sign.
- [ ] [BENCH] Confirm the unit factor: if `raw * mmPerDeg` is ~10× higher than
  encoder-delta velocity, apply `/10` (tenths); if it matches, use `raw *
  mmPerDeg` directly (no division). Document the confirmed factor in a code
  comment in `Motor.cpp`.

## Implementation Plan

**Approach**: Surgical fix to `Motor.cpp` and `Config.h`; no control-loop
changes in this ticket.

**Files to modify**:
- `source/hal/Motor.cpp` — rewrite `readSpeed()` body; update header comment.
- `source/hal/Motor.h` — update `readSpeed` doc comment; remove mention of
  `lapsToMmScale`.
- `source/types/Config.h` — delete `lapsToMmScale` field from `RobotConfig`
  struct and `defaultRobotConfig()`.
- `source/app/CommandProcessor.cpp` — remove `lapsToMmScale` from `kRegistry[]`;
  add `GET VEL` handler that calls `_robot.motorController().getActualVelocity()`
  and `getVelocitySourceFlags()`.

**Testing plan**:
- Unit tests: not straightforward for I2C reads; verify the conversion formula
  manually with known raw values in a comment or a standalone test if the test
  harness supports mocking.
- Bench: run `S 200 200`, issue `GET VEL`; compare chip vs encoder values.
  Confirm sign correctness in both forward and reverse.

**Documentation updates**:
- Add a comment in `Motor.cpp::readSpeed` citing the bench-confirmed unit
  factor and the rationale for reusing `mmPerDeg`.
