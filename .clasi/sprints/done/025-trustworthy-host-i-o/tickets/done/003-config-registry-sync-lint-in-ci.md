---
id: '003'
title: Config registry sync lint in CI
status: done
use-cases:
- SUC-003
depends-on: []
github-issue: ''
issue: a8-config-registry-sync-lint.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Config registry sync lint in CI

## Description

The config system is out of sync in both directions with no mechanical check:

**In-struct, not registered** (unreachable via GET/SET):
- `safetyEnabled` (line 162 of `source/types/Config.h`)
- `tlmFields` (line 176)
- `tlmSnapPending` (line 179)

**Registered, not read** (SET "works" but changes nothing):
- `turnScale` (in `ConfigRegistry.cpp`)
- `distScale` (in `ConfigRegistry.cpp`)

The pattern — not the individual mismatches — is the problem. There is no
mechanical check, so drift recurs silently every time a field is added or a
consumer is removed.

This ticket:
1. Writes `scripts/check_config_sync.py` — a lint script that cross-checks
   three sets and reports mismatches.
2. Resolves the current offenders (register, remove, or explicitly allowlist
   each one with a comment).
3. Wires the lint into CI so every PR is checked.

## Scope

### `scripts/check_config_sync.py` (new)

The script parses three sources:

**Set A — struct fields**: parse `source/types/Config.h` for field declarations
inside the `struct RobotConfig { ... }` block. Extract the field names (the
identifier before the `;`). Skip comments, typedefs, and enum constants.

**Set B — registered keys**: parse `source/robot/ConfigRegistry.cpp` for
`CFG_F(`, `CFG_I(`, `CFG_B(`, or similar macro calls. Extract the first
string argument (the key name).

**Set C — firmware usage**: grep `source/` (excluding `ConfigRegistry.cpp` and
`DefaultConfig.cpp`) for identifiers matching each registered key in Set B.
A key is "used" if it appears in any source file outside those two.

**Allowlist**: an optional `scripts/config_sync_allowlist.json` file mapping
field/key names to a required justification comment string. Entries in the
allowlist are exempt from the lint report.

**Output**: report three categories separately:
- `in-struct-not-registered`: fields in Set A with no matching key in Set B.
- `registered-not-in-struct`: keys in Set B with no matching field in Set A.
- `registered-not-used`: keys in Set B with no usage in Set C.

Exit 0 if all three categories are empty (or all non-empty entries are in the
allowlist). Exit 1 otherwise, printing the offending names.

### Resolve current offenders

Examine each offender and choose the appropriate resolution:

- `safetyEnabled` — used internally by firmware to gate safety logic; not a
  tunable field. Add to allowlist with comment: `"used internally, not
  externally tunable"`.
- `tlmFields` — the TLM field bitmask; its value is set by the STREAM command
  handler directly, not via ConfigRegistry. Add to allowlist with comment:
  `"set by STREAM command, not via ConfigRegistry SET"`.
- `tlmSnapPending` — ephemeral flag set/cleared within the tick loop; not a
  persistent config value. Add to allowlist with comment: `"ephemeral tick
  flag, not a persistent config field"`.
- `turnScale` — confirm whether it is read anywhere in `source/`. If truly
  unread, remove the ConfigRegistry entry. If it maps to a field that was
  renamed, fix the registry key. (Verify before removing; do not remove a
  key that is used.)
- `distScale` — same process as `turnScale`.

If `turnScale` or `distScale` turn out to be legitimately unread (dead config),
remove their `ConfigRegistry.cpp` entries and add a comment in the commit
message explaining why they were removed.

### CI integration

Add a job to `.github/workflows/build.yml` (or `host-lint.yml`) that runs:

```bash
python scripts/check_config_sync.py
```

This job should run on every PR. It does not require firmware compilation; it
is pure Python parsing and can run on `ubuntu-latest` without the ARM toolchain.

## Acceptance Criteria

- [x] `scripts/check_config_sync.py` exists and is runnable via
      `python scripts/check_config_sync.py` from the repo root.
- [x] Script reports `in-struct-not-registered`, `registered-not-in-struct`,
      and `registered-not-used` categories separately.
- [x] Script exits 0 when all offenders are resolved or allowlisted.
- [x] Script exits 1 when a new unregistered field is added to `Config.h`
      (verified by a local dry-run adding a test field and running the script).
- [x] `safetyEnabled`, `tlmFields`, `tlmSnapPending` are either registered or
      in the allowlist with a justification comment.
- [x] `turnScale` and `distScale` are either removed from `ConfigRegistry.cpp`
      (if confirmed unread) or their usage is confirmed and documented.
      (Confirmed absent from ConfigRegistry.cpp; test_config_registry.py
      confirms they were removed in sprint 024-006 as dead keys. No source/
      references found outside Config.h comments.)
- [x] CI job runs the lint on every PR and fails on drift.
- [x] No existing tests broken: `uv run --with pytest python -m pytest -q tests/dev/`

## Implementation Plan

### Approach

Write the script incrementally: parse Config.h first (simplest), then
ConfigRegistry.cpp, then the usage grep. Verify output against the known
offenders before wiring CI.

### Files to create

- `scripts/check_config_sync.py`
- `scripts/config_sync_allowlist.json` (may be empty initially if all
  offenders are removed rather than allowlisted)

### Files to modify

- `source/robot/ConfigRegistry.cpp` — remove dead entries for `turnScale`
  and/or `distScale` if confirmed unread (or add usage if they should be read).
- `.github/workflows/build.yml` (or new `host-lint.yml`) — add lint job.

### Testing plan

No pytest tests for the lint script itself in this sprint. Manual verification:

1. Run `python scripts/check_config_sync.py` from repo root; confirm exit 0.
2. Add a test field `float testDriftField;` to `Config.h` temporarily; run
   the script; confirm exit 1 with `testDriftField` listed under
   `in-struct-not-registered`. Remove the test field.
3. `uv run --with pytest python -m pytest -q tests/dev/` — no regressions.

Note: `tests/dev/test_config_registry.py` already exists; confirm it still
passes after any ConfigRegistry.cpp changes.

### Documentation updates

Add a comment block at the top of `scripts/check_config_sync.py` explaining
the three-set model and how to add allowlist entries. Update `scripts/README`
or the repo-level docs if such a file exists.
