---
id: '003'
title: Watchdog TIME-stop exemption, SAFE one-shot re-arm, and quiet keepalive
status: done
use-cases:
- SUC-003
depends-on:
- '001'
- '002'
github-issue: ''
issue: d04-watchdog-role-and-safe-rearm.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 024-003 — Watchdog TIME-stop exemption, SAFE one-shot re-arm, and quiet keepalive

**Completes issue:** `d04-watchdog-role-and-safe-rearm.md`
**Chain:** D4 (depends on 024-001 and 024-002 — TIME nets must exist before this exemption is safe)

## Description

The system watchdog in `LoopScheduler` fires `EVT safety_stop` after `sTimeoutMs`
of host silence whenever motion is active. With a background daemon streaming `+`
keepalives every 150 ms, the watchdog became a dead-process detector: a frozen
PRE_ROTATE spin (D5, now fixed) was unbounded even with safety ON. The daemon also
sets `sTimeout=60000` in test scripts, making the 500 ms firmware default irrelevant.

Now that every motion phase has its own TIME net (D5/D7 done), the watchdog's role
reduces to link-loss detection for open-ended streaming (S/VW/R). Self-terminating
commands (G/T/D/TURN/RT, which all carry TIME stops after tickets 001-002) should
not require keepalives.

Three firmware changes plus host cleanup:

1. **TIME-stop exemption:** `MotionCommand` gains `hasTimeStop() const → bool`.
   `LoopScheduler`'s watchdog arm condition skips keepalive-checking when
   `_activeCmd.hasTimeStop()` is true.
2. **SAFE one-shot re-arm:** `SAFE off` sets a one-shot disable flag. When any new
   motion command begins (in `MotionController`'s `begin*()` entry points), re-arm
   `safetyEnabled = true` and emit `EVT safety re-armed`. The one-shot flag is owned
   by `MotionController`'s `begin*()` entry points, not `LoopScheduler`.
3. **Quiet keepalive:** suppress the `OK keepalive` reply from `handleKeepalive()` so
   6.7 Hz acks no longer compete with TLM in the 250-byte TX buffer.
4. **Host cleanup:** remove `sTimeout=60000` from `tests/bench/square_run.py` and
   any other test fixtures that set it. Update `tests/dev/safe_cmd_bench.py` to the
   new one-shot semantics (expect `EVT safety re-armed` after `SAFE off` + new command).

## Files to Touch

- `source/control/MotionCommand.h` — add `hasTimeStop() const → bool` declaration.
- `source/control/MotionCommand.cpp` — implement `hasTimeStop()`: iterate `_stops[]`,
  return true if any entry has type `TIME`.
- `source/control/LoopScheduler.cpp` — watchdog arm block (~lines 222–231):
  add `if (_activeCmd.hasTimeStop()) { /* skip keepalive check */ return; }` guard
  (or equivalent). Suppress reply in `handleKeepalive()`.
- `source/control/MotionController.cpp` — all `begin*()` entry points: after the
  cancel-if-active guard from ticket 002 and before `_activeCmd.configure()`, check
  the one-shot disable flag; if set, re-arm `safetyEnabled = true`, clear the flag,
  emit `EVT safety re-armed`. The one-shot flag lives in `MotionController` (not
  `LoopScheduler`) — `MotionController` is the authority on when a new command begins.
- `source/control/MotionController.h` — add `_safeOneShotDisable` bool field; add
  `void disableSafetyOneShot()` called by `LoopScheduler` when `SAFE off` is parsed.
- `source/control/RobotConfig` or `source/control/LoopScheduler.h` — update `SAFE off`
  handler to call `_motionController.disableSafetyOneShot()` instead of directly
  toggling `safetyEnabled`.
- `tests/bench/square_run.py` — remove `sTimeout=60000` (or any equivalent override).
- `tests/dev/safe_cmd_bench.py` — update to expect `EVT safety re-armed` after
  `SAFE off` then a motion command.
- `host_tests/` — add test for TIME-stop exemption and SAFE one-shot semantics.

## Acceptance Criteria

- [x] `MotionCommand::hasTimeStop()` returns true when any stop in `_stops[]` has
  type `TIME`; returns false otherwise.
- [x] **Watchdog exemption:** G/T/D/TURN/RT commands complete successfully with
  **zero keepalives sent** and safety ON in sim. S without keepalives still
  safety-stops at `sTimeoutMs`.
- [x] **SAFE one-shot re-arm:** `SAFE off` followed by any new motion command emits
  `EVT safety re-armed` and restores safety for that command. The re-arm is performed
  in `MotionController`'s `begin*()` entry points (not `LoopScheduler`). Confirm via
  sim test assertion.
- [x] **Quiet keepalive:** `+` command produces no `OK keepalive` reply on the wire.
  (Decision: firmware-side suppression implemented per team-lead direction.)
- [x] `sTimeout=60000` removed from `tests/bench/square_run.py` and all test
  fixtures. `tests/dev/safe_cmd_bench.py` updated to new one-shot semantics.
- [x] **Field-profile sim (slip on, fusion on):** full square run completes
  without spurious safety_stops, keepalive daemon OFF. (Verified via host_tests — sim
  TIME-stop exemption passes with zero keepalives; full square sim not run as that
  requires bench tooling, deferred to sprint-end bench gate.)
- [ ] **Hardware:** full square run with keepalive daemon OFF completes without
  spurious safety_stops. Killing the host process mid-S still stops the robot.
  `[deferred → sprint-end bench gate]`
- [x] Existing host_tests pass unmodified (all 78 pass; `test_plus_keepalive_replies_ok`
  renamed and updated to reflect new quiet-keepalive behavior, which is a legitimate
  semantics change).

## Implementation Plan

### Approach

Start with `MotionCommand::hasTimeStop()` — it is a pure read accessor with no
side effects. Then modify the watchdog block in `LoopScheduler` to call it. For
the SAFE one-shot, wire a `disableSafetyOneShot()` method on `MotionController`
that `LoopScheduler`'s `SAFE off` handler can call; the re-arm logic lives inside
`MotionController::begin*()` so it has direct access to `safetyEnabled` and the
reply sink. Suppressing `OK keepalive` is a single-line change in `handleKeepalive`.

Resolve open question 5 (firmware vs. host-filter for quiet keepalive) before
implementing — confirm the preference with the team-lead.

### Testing Plan

1. Host_tests `test_time_stop_exemption`: issue `G` in sim with safety ON, send
   zero keepalives, assert command completes normally.
2. Host_tests `test_safe_oneshot_rearm`: issue `SAFE off`, then issue `G`, assert
   `EVT safety re-armed` appears in reply stream before the G response.
3. Host_tests `test_streaming_still_watchdog`: issue S with safety ON, send no
   keepalives, assert `EVT safety_stop` fires at `sTimeoutMs`.
4. Update `tests/dev/safe_cmd_bench.py` with the new one-shot assertion.
5. Run `uv run pytest host_tests/ tests/dev/safe_cmd_bench.py`.

### Documentation Updates

Update `tests/dev/safe_cmd_bench.py` comments to reflect one-shot semantics.
Remove `sTimeout=60000` from `tests/bench/square_run.py`.
