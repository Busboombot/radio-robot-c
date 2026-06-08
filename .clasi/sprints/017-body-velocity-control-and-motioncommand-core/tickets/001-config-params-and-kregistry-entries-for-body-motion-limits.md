---
id: '001'
title: Config params and kRegistry entries for body motion limits
status: done
use-cases:
- SUC-005
depends-on: []
github-issue: ''
issue: motion-command-body-velocity-control.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 017-001: Config params and kRegistry entries for body motion limits

## Description

Add five new fields to `RobotConfig` in `source/types/Config.h` and their corresponding
`kRegistry[]` entries in `source/app/CommandProcessor.cpp`. This is a pure additive
config change — no behaviour changes, no new classes, no callers yet. It unblocks all
subsequent tickets that read these fields from `const RobotConfig&`.

The issue spec (`motion-command-body-velocity-control.md`) requires these fields for the
`BodyVelocityController`. This ticket covers the config and registry only.

## Files to Create / Modify

- **Modify** `source/types/Config.h` — add five fields and their defaults.
- **Modify** `source/app/CommandProcessor.cpp` — add five `kRegistry[]` entries.

## Acceptance Criteria

- [x] Five new fields present in `RobotConfig` struct:
  - `float vBodyMax`      — body forward speed ceiling, mm/s (default 400.0)
  - `float yawRateMax`    — yaw rate ceiling, deg/s (default 180.0)
  - `float yawAccMax`     — yaw acceleration limit, deg/s² (default 720.0)
  - `float jMax`          — linear jerk limit, mm/s³ (default 0.0, trapezoid)
  - `float yawJerkMax`    — yaw jerk limit, deg/s³ (default 0.0, trapezoid)
- [x] Defaults set in `defaultRobotConfig()`.
- [x] Five new `CFG_F` entries in `kRegistry[]` with keys `vBodyMax`, `yawRateMax`,
  `yawAccMax`, `jMax`, `yawJerkMax`.
- [x] Host unit test (new file `tests/dev/test_body_motion_config.py`) verifies:
  - Each of the five keys is present in the registry with the correct default value
    (pure-Python mock of the registry table using the same key names).
  - Round-trip: encode float → key=val string, decode → same float.
- [x] Clean build: `python3 build.py --clean` completes without errors or warnings.
- [x] Host test suite at baseline: `uv run --with pytest python -m pytest -q` shows
  1064 pass / 8 fail (29 new tests added, no new failures introduced).

## Bench Verification (stakeholder-deferred)

- SET/GET round-trip for all five keys deferred to on-robot bench session.
  Expected: `SET vBodyMax=300` → `GET vBodyMax` → `CFG vBodyMax=300.000`; etc.

## Implementation Plan

1. In `Config.h`, append the five fields to `RobotConfig` immediately after the existing
   `arriveTolMm` field (near the `aMax`/`aDecel` group) and set defaults in
   `defaultRobotConfig()`.
2. In `CommandProcessor.cpp`, add five `CFG_F` macro entries to `kRegistry[]` after the
   existing `aMax`/`aDecel` entries.
3. Write `tests/dev/test_body_motion_config.py` as a pure-Python test (no serial
   connection) that constructs a dict of `{key: default_value}` mirroring the new entries
   and asserts each default matches the spec. Follow the pattern of
   `tests/dev/test_config_registry.py`.
4. Run `python3 build.py --clean` and confirm clean build.
5. Run `uv run --with pytest python -m pytest -q` and confirm baseline.

## Bench Verification (stakeholder-deferred)

- `SET vBodyMax=300` → `GET vBodyMax` returns `CFG vBodyMax=300.000`.
- Repeat for all five keys.
- `SET jMax=0` → no change in robot motion (S-curve still off).
