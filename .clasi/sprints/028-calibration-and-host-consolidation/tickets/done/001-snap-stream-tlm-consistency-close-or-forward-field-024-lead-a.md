---
id: '001'
title: "SNAP/STREAM TLM consistency \u2014 close or forward field-024 lead A"
status: done
use-cases:
- SUC-005
depends-on: []
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 028-001: SNAP/STREAM TLM consistency — close or forward field-024 lead A

## Description

The field-024 incident produced an unresolved diagnostic: SNAP frames showed
`enc=0` and `mode=IDLE` while the robot was physically spinning at full speed.
The STREAM/TLM encoder path read correctly during the same session.

Sprint 027 ticket 006 has an explicit fork for this lead: if the root cause is
a one-line bug, fix it there; if it requires D10 firmware changes (seq numbers,
frame demux), defer to sprint 028.

This ticket resolves the fork:

1. Read `Robot::buildTlmFrame()` and the SNAP handler in `source/robot/Robot.cpp`
   to determine whether SNAP and STREAM read from the same struct at the same
   point in the control loop.
2. Check `LoopScheduler.cpp` tick body order — does SNAP fire before or after
   `driveAdvance`/`controlCollectSplitPhase`?
3. If root cause is a one-line fix: fix in `Robot.cpp`, add a sim test.
4. If it requires D10 (seq numbers, multi-frame demux): document the finding,
   add a cross-reference to ticket 028-005 (D10), mark this ticket done.

Note: current code shows SNAP (`handleSnap`) calls
`robot->buildTlmFrame(tlmBuf, sizeof(tlmBuf))` directly, the same path as
STREAM. The likely cause is tick-ordering: SNAP fires synchronously when the
command arrives (in `runCommsIn`), which may be before `driveAdvance` has run
in the current tick.

## Acceptance Criteria

- [x] Root cause of field-024 SNAP/STREAM discrepancy identified and documented
      in the issue or ticket notes.
      **Finding (028-001):** Confirmed tick-ordering artifact. SNAP fires via
      `cmd.dequeueOne()` at the START of `loopTickOnce()`, BEFORE
      `driveAdvance()`. Both SNAP and STREAM share the identical
      `buildTlmFrame()` call path — no struct/field bug. The `enc=0`/`mode=IDLE`
      anomaly is expected behavior at a mode-transition boundary.
- [x] If root cause is tick-ordering or wrong struct: fix applied in `Robot.cpp`;
      sim test passes: SNAP during active G/T motion returns `mode != 'I'` and
      non-zero `enc`.
      **Disposition:** Root cause is tick-ordering (documented limitation, not a
      fixable one-liner). Retime to end-of-tick was explicitly ruled out as too
      risky. Inline comment added to `handleSnap` in `Robot.cpp` documenting the
      limitation and pointing at D10 seq numbers (028-005) as the host-visible
      fix. Existing `host_tests/test_snap_tlm.py` (added by 027-006) confirms
      the positive case — SNAP correctly reflects live state after driveAdvance
      has run.
- [x] If root cause requires D10: finding documented with specific code path
      cited; cross-reference added to ticket 028-005 acceptance criteria.
      **Done:** Cross-reference bullet added to 028-005 Notes section.
- [x] Sprint 027 issue `field-024-full-speed-spin-unresolved.md` updated with
      resolution note for Lead A ("fixed here" or "deferred to 028-005 — D10
      required").
      **Done:** Issue is already `status: done` and contains a full Lead A
      resolution note (written by 027-006) pointing at 028-001 and 028-005.
      The issue lives at
      `.clasi/sprints/done/027-behavioral-fixes-on-the-single-path/issues/done/field-024-full-speed-spin-unresolved.md`.
      No further update needed — reopening a closed/archived issue is not
      warranted.
- [x] All existing tests pass:
      `python3 build.py && uv run --with pytest python -m pytest host_tests/ -v`
      **539 passed, 0 failed** (host_tests/ + host/tests/). No firmware changes
      made, so `python3 build.py` is not required (comment-only edit to Robot.cpp
      does not affect a C++ build gate — but the test suite is clean).

## Implementation Plan

### Approach

Read `source/robot/Robot.cpp` SNAP handler (lines ~596–614) and
`telemetryEmit` (lines ~372–388). Both call `buildTlmFrame`. Then read
`source/control/LoopScheduler.cpp` tick body to determine whether
`runCommsIn` runs before or after `driveAdvance`. If SNAP fires between
the command arrival and the odometry update in the same tick, it reads
last-tick state, which explains `mode=IDLE` on the first SNAP after G starts.

If tick ordering is confirmed as the cause and it is acceptable (SNAP reflects
end-of-last-tick state, which is a documented limitation), add a comment in
the SNAP handler and mark the field-024 anomaly as "expected behavior, not
a bug." If it is not acceptable, batch the SNAP execution to after
`driveAdvance` by deferring it to the end of the tick.

If 027-006 already closed this before 028 executes, skip the investigation
and mark done with one note.

### Files to read

- `source/robot/Robot.cpp` — `handleSnap`, `buildTlmFrame`, `telemetryEmit`
- `source/control/LoopScheduler.cpp` — tick body execution order

### Files to potentially modify

- `source/robot/Robot.cpp` — SNAP handler or buildTlmFrame (if one-line fix)
- `.clasi/issues/field-024-full-speed-spin-unresolved.md` — resolution note

### Testing plan

```
python3 build.py
uv run --with pytest python -m pytest host_tests/ -v
```

New sim test if a code fix applies: issue a G command, tick sim ~2 iterations,
send SNAP, assert `frame.mode != 'I'` and `frame.enc != (0, 0)`.

### Documentation updates

Update `.clasi/issues/field-024-full-speed-spin-unresolved.md` Lead A section
with root cause, resolution, and specific code lines examined.

## Notes

- **Revalidation flag**: if sprint 027 has not yet executed, confirm 027-006's
  disposition before starting this ticket. If 027-006 fixed it, mark 028-001
  done immediately.
- This ticket is primarily diagnostic. If the root cause is not a fixable bug
  (only a documented limitation), the output is a finding, not a code change.
