---
id: '006'
title: 'field-024 diagnosis closure: SNAP TLM discrepancy + issue resolution'
status: open
use-cases:
  - SUC-008
depends-on:
  - "027-002"
  - "027-005"
github-issue: ''
issue: field-024-full-speed-spin-unresolved.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 027-006: field-024 diagnosis closure

## Description

Two open leads from `field-024-full-speed-spin-unresolved.md`:

**Lead A — SNAP TLM discrepancy:** SNAP frames showed `enc=0` and `mode=IDLE`
while the robot was physically spinning at full speed. The STREAM/TLM encoder
path reads fine (`enc_watch.py` verified). The 024-005 commit changed
`buildTlmFrame`; investigate whether SNAP uses a different code path or a
stale snapshot. This is a diagnostic investigation; if the fix is a one-liner
(stale pointer, wrong field), fix it here. If it requires D10 firmware TLM
restructuring (seq numbers, idle rate), defer to sprint 028 and document the
finding.

**Lead B — host abandons G without X:** Closed by 027-002 (`BenchRun` wrapper
ensures every bench program sends `X` on exit). This ticket verifies that
`square_run.py` (the program that triggered the field-024 failure) is covered
by the wrapper.

Once both leads are resolved (fixed or explicitly deferred), close the issue.

## Acceptance Criteria

### Lead A — SNAP TLM investigation

- [ ] Read `Robot::buildTlmFrame()` and the SNAP handler to determine whether
      SNAP constructs its frame differently from STREAM (different source
      struct, stale pointer, or a copy vs. reference issue introduced in
      024-005).
- [ ] If the cause is a one-line bug (e.g. SNAP passes a local copy of
      `HardwareState` that was captured before `driveAdvance` updated mode/enc):
      - Fix it in `Robot.cpp` or the SNAP handler.
      - Add a test or note that verifies the SNAP frame `mode` matches the
        STREAM frame `mode` in the sim after a motion command starts.
- [ ] If the cause requires D10 firmware changes (seq numbers, frame
      demux, idle-rate changes):
      - Document the finding in a comment in the issue file.
      - Add a cross-reference: "requires sprint 028 D10 work".
      - Do NOT attempt the D10 fix here.
- [ ] The field-024 issue file is updated with resolution notes:
      - Lead B: "closed by 027-002 (bench runaway wrapper)".
      - Lead A: either "fixed in 027-006" or "deferred to 028 — see
        [D10 cross-reference]".
- [ ] Issue is moved to done if both leads are either fixed or explicitly
      deferred with sprint references.

### Lead B — host-abandon-without-X

- [ ] `square_run.py` in `tests/bench/` confirmed to use `BenchRun` (from
      027-002). If 027-002 already wrapped it, verify; if not, wrap it here.
- [ ] Manual test: run `square_run.py` and Ctrl-C mid-run; robot stops.

### All existing tests pass

- [ ] `uv run pytest host_tests/ -v` green.

## Implementation Plan

### Approach

**Lead A investigation:** Read `source/robot/Robot.cpp` starting at
`buildTlmFrame`. Identify where SNAP calls it vs. where STREAM calls it.
Specifically check:
- Does SNAP call `buildTlmFrame(&state.inputs)` at the time of the SNAP
  request, or does it use a cached/queued frame?
- In 024-005, were any fields moved from `state.inputs` to a separate struct,
  leaving a stale pointer?
- Does the SNAP handler run in the same cooperative-loop tick as
  `driveAdvance`, or can it fire between the motor command write and the
  odometry update (where `mode` and `enc` might not yet reflect the new
  state)?

If the root cause is a single wrong field or a stale reference, fix it
in-place. Write a sim test that issues SNAP immediately after starting
motion and asserts `mode != IDLE` and `enc != 0`.

If it requires D10 (multi-frame mux, seq numbers), open the `d10-*` issue
or add a note to the field-024 issue and stop.

**Lead B verification:** If `square_run.py` already uses `BenchRun` (from
027-002), this ticket only verifies; no code change needed.

### Files to potentially modify

- `source/robot/Robot.cpp` — SNAP handler or `buildTlmFrame` fix (if Lead A
  has a one-liner fix).
- `tests/bench/square_run.py` — wrap in BenchRun if not already done.
- `.clasi/issues/field-024-full-speed-spin-unresolved.md` — update with
  resolution notes and sprint references.

### Testing plan

```
python3 build.py
uv run pytest host_tests/ -v
```

For Lead A fix (if applicable): add a sim test that issues SNAP during active
motion and asserts `mode` and `enc` reflect the live state.

### Documentation updates

Update `.clasi/issues/field-024-full-speed-spin-unresolved.md` with
resolution notes before moving to done.

## Notes

- This ticket is primarily diagnostic. If Lead A is a complex firmware issue,
  the output is a well-documented finding + sprint 028 reference, not a code
  change. Do not over-engineer.
- The SNAP vs STREAM discrepancy (`mode=IDLE` while spinning) is the key
  smell. The first thing to check: does `SNAP` read `state.inputs` directly,
  or does it read `state.target` (which shows the *commanded* mode, not the
  running mode)? A wrong struct reference would explain the `mode=IDLE` while
  motors are running.
- If 027-002 already wrapped `square_run.py`, this ticket's Lead B work is
  just a one-line verification and documentation update.
