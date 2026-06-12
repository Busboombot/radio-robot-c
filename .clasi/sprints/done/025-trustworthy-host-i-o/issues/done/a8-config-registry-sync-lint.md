---
status: done
sprint: '025'
tickets:
- 025-003
---

# A8 — Config struct, ConfigRegistry, and actual usage must be kept in sync (lint)

## Context

The config system is out of sync in both directions:

- `safetyEnabled`, `tlmFields`, `tlmSnapPending` exist in `types/Config.h` and are
  initialized in `DefaultConfig.cpp`, but have **no `ConfigRegistry` entries** —
  unreachable via GET/SET (registry has ~61 entries; the struct has more).
- Conversely, several registered keys are read by nothing in firmware
  (`turnScale`, `distScale`; `rotationalSlip` until P0.5 lands) — see A7/D2.

Each individual mismatch is small; the pattern is the problem: there is no
mechanical check, so the drift recurs every time a field is added or a consumer is
removed, and it fails silently (a SET that "works" but changes nothing, or a field
that can't be tuned in the field).

## Fix

1. Script (CI + `just` target) that cross-checks three sets: fields in
   `types/Config.h`, entries in `ConfigRegistry.cpp`, and references in `source/`
   outside DefaultConfig/ConfigRegistry. Report: in-struct-not-registered,
   registered-not-read, registered-not-in-struct. Allowlist for deliberate
   exceptions, with a required comment.
2. Resolve the current offenders: register or remove the three unregistered
   fields; A7 item 3 handles the unread keys.
3. Consider generating `ConfigRegistry.cpp` from the same source as
   `DefaultConfig.cpp` (`scripts/gen_default_config.py` already exists) so one
   direction of drift becomes impossible.

## Acceptance

- Lint runs in CI and currently passes (offenders resolved or allowlisted with
  justification); adding an unregistered config field breaks the build.

## Priority suggestion

**Low-medium effort, schedule early anyway** — it's a half-day mechanical task that
permanently closes the defect class behind D2, and A7 depends on it. Good filler
item for any sprint.

## Source
Finding **A8** in `docs/code_review/2026-06-11-architecture-modularity-review.md`.
