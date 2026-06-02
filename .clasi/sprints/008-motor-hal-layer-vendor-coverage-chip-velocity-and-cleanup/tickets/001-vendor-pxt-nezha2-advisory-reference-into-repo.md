---
id: '001'
title: Vendor pxt-nezha2 advisory reference into repo
status: open
use-cases: [SUC-001]
depends-on: []
github-issue: ''
issue: nezha-full-vendor-i2c-coverage.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Vendor pxt-nezha2 advisory reference into repo

## Description

Copy the authoritative PlanetX `pxt-nezha2` TypeScript driver into
`vendor/pxt-nezha2/` so the full Nezha2 I2C command surface is auditable
in-repo without requiring access to the scratch repo. The files are
advisory only — they are never compiled into firmware.

This is the foundation ticket for the full vendor command coverage work.
Having the reference in-repo allows reviewers to verify that each new HAL
wrapper matches the vendor implementation.

Source:
`/Volumes/Proj/proj/league-projects/scratch/radio-robot/vendor/pxt-nezha2/`

## Acceptance Criteria

- [ ] `vendor/pxt-nezha2/main.ts` is present in `radio-robot-c`.
- [ ] `vendor/pxt-nezha2/test.ts` is present (if it exists in the source).
- [ ] `vendor/pxt-nezha2/README` (or `README.md`) explains advisory-only
  status and identifies the authoritative source path.
- [ ] `vendor/pxt-nezha2/` is not referenced in `codal.json`,
  `CMakeLists.txt`, or any build file — the TypeScript must never be
  compiled.
- [ ] `python3 build.py` produces `MICROBIT.hex` without errors; report the
  RAM line from build output as a baseline for subsequent tickets.

## Implementation Plan

### Approach

Pure file copy plus a short README. No code changes.

### Files to Create

- `vendor/pxt-nezha2/main.ts` — copy from source above.
- `vendor/pxt-nezha2/test.ts` — copy if present; omit if absent.
- `vendor/pxt-nezha2/README` — one paragraph: advisory status, not
  compiled, authoritative source path, date copied.

### Files to Modify

None. Verify that no existing build file references `vendor/`.

### Testing Plan

- `python3 build.py` must succeed; firmware behavior unchanged.
- `grep -r "pxt-nezha2" codal.json` (and any other build config files)
  must return nothing.

### Documentation Updates

None beyond the `README` in `vendor/pxt-nezha2/`.
