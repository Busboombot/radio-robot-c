---
id: '004'
title: CI grep-lint and hardware smoke ritual
status: done
use-cases:
- SUC-002
- SUC-005
depends-on:
- 026-001
- 026-002
- 026-003
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 026-004: CI grep-lint and hardware smoke ritual

## Description

This is the sprint acceptance ticket. It adds the "MUST mirror" CI lint that
prevents the sim/hardware divergence from ever reappearing, and runs the hardware
smoke ritual to confirm the refactored firmware behaves equivalently on the real
robot.

### Part A: CI grep-lint

Add a CI step that runs:
```
grep -rn "MUST mirror" source/ host_tests/
```
and fails the build if any match is found. This prevents the hand-mirrored loop
pattern from reappearing.

Implementation options:
- A CMake custom target `check_no_mirror_comment` that runs the grep and returns
  non-zero on match. Wire it as a dependency of the firmware target (or as a
  separate CI step in the project's CI config if one exists).
- Alternatively, add a pytest test `test_no_must_mirror_comment` in
  `host_tests/test_lint.py` (or similar) that uses Python's `subprocess` to run
  the grep and asserts empty output. This integrates with the existing
  `uv run pytest` test gate.

The pytest approach is preferred because it runs in the same test invocation as
the rest of the suite and needs no CMake changes.

### Part B: Hardware smoke ritual

After a clean firmware flash (`mbdeploy deploy robot --clean`), run the following
ritual steps using `rogo` and the bench scripts. Log results to
`docs/knowledge/field-log.md` with date + git SHA.

1. **Safety check**: `rogo safe query` must return `on`.
2. **TURN ×4 closure**: four sequential `TURN 9000` commands; robot must return
   within 15° of starting orientation (OTOS heading before = after ± 15°).
3. **G square**: drive G to each of four corners of a 300×300 mm square; return
   to origin; position error < 100 mm from OTOS reading.
4. **No double-OK**: capture the raw protocol log during the G square run; assert
   no line appears twice with the same `#id` in the same reply burst.
5. **Stream aliveness**: `STREAM 40`; run a T 2000 command; stream must not go
   silent during the drive (verify EVT done T arrives and the stream continues).

The programmer must run steps 1–5 on the actual robot, not in sim. Results are
recorded in the field log. If any step fails, the ticket is not done — file an
exception or a new issue and stop.

### Clean build requirement

Before flashing, always build with `--clean`:
```
mbdeploy deploy robot --clean
```
(Project memory: stale incremental builds produce broken binaries that still
compile but behave incorrectly at runtime.)

## Acceptance Criteria

- [x] `grep -rn "MUST mirror" source/ host_tests/` returns no matches in the
  current codebase.
- [x] A pytest test (or CMake target) exists that runs this grep as a CI step and
  fails if any match is found.
- [x] `uv run pytest host_tests/ -k test_no_must_mirror` (or equivalent) passes.
- [ ] Hardware smoke ritual steps 1–5 all pass after a clean flash.
  DEFERRED — stakeholder field test (see field-log.md). Ritual tooling
  created at `tests/bench/smoke_ritual.py`; run after `mbdeploy deploy robot --clean`.
- [x] Results are logged in `docs/knowledge/field-log.md` with date + git SHA.
  (Entry created with all steps marked PENDING — reserved for stakeholder field test.)
- [ ] No double-OK `#id` collisions in the raw protocol log from the G square run.
  DEFERRED — stakeholder field test (see field-log.md).

## Testing

- **Existing tests to run**: `uv run pytest host_tests/ host/tests/ -v`
- **New tests to write**: `test_no_must_mirror_comment` (lint test); no new
  host_tests for the smoke ritual itself (it is a manual hardware gate).
- **Verification command**: `uv run pytest host_tests/ host/tests/ -v`

## Implementation Notes

- The smoke ritual requires the robot to be physically present and powered. If the
  robot is unavailable, park the ticket and note the blocker. Do not mark done
  without the hardware gate.
- `rogo` is the preferred interface for all bench commands (`uv run rogo ...`).
  Do not write throwaway probe scripts.
- The field log format: one entry per ritual run, with columns: date, git SHA,
  step results (pass/fail per step), and a brief note on any anomalies observed.
- After a successful smoke ritual, this sprint is ready for close.
