---
id: '005'
title: X cancel verb and host cancel() wrapper
status: open
use-cases:
  - SUC-003
depends-on:
  - '004'
github-issue: ''
issue: motion-command-body-velocity-control.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 017-005: X cancel verb and host cancel() wrapper

## Description

Add the `X` cancel verb to the firmware (`CommandProcessor.cpp`) and update `STOP` to use
the same teardown path. Add a `cancel()` method to the host protocol wrapper
(`host/robot_radio/robot/protocol.py`). Update the HELP verb list to include `X`.

After this ticket, any active `MotionCommand` (currently only VW) can be hard-stopped by
sending `X` or `STOP`. The `X` verb is the canonical cancel; `STOP` is an alias for
backward compatibility.

## Files to Modify

- `source/app/CommandProcessor.cpp` — add `X` verb handler; update `STOP` handler to call
  `DriveController::cancel()`.
- `host/robot_radio/robot/protocol.py` — add `cancel()` method.
- `tests/dev/test_motion_verbs_v2.py` — add `X` and updated `STOP` test cases.

## Acceptance Criteria

### Firmware

- [ ] New `X` verb in `CommandProcessor::process`:
  - Calls `_robot.driveController.cancel(now_ms, replyFn, ctx)`.
  - Replies `OK x` (or `OK cancel` — pick one consistent name; `OK x` is simpler).
  - If no command is active, cancels cleanly (no-op beyond `_mc.stop()` idempotency).
- [ ] `STOP` verb updated to call `_robot.driveController.cancel(now_ms, replyFn, ctx)`
  instead of `driveController.stop(...)`. Still replies `OK stop`.
- [ ] `HELP` verb response updated to include `X` in the verb list.
- [ ] `EVT cancelled` emitted by `MotionCommand::cancel(HARD)` on the cancel tick.
- [ ] If no MotionCommand is active when `X`/`STOP` arrives: `_mc.stop()` is still called
  (motors halt); no `EVT cancelled` emitted (no active command to cancel).
- [ ] `S` command is not affected: STOP still stops it; X stops it via `_mc.stop()`.

### Host wrapper

- [ ] `NezhaProtocol.cancel()` method in `protocol.py`:
  - Sends `X\n` via `send_fast` (fire-and-forget, like `vw`).
  - Docstring: "Cancel the active motion command. Sends X (hard stop)."

### Tests

- [ ] `test_motion_verbs_v2.py` or new test file: `X` sends `X\n` and receives `OK x`
  (or `OK cancel`; match the firmware implementation).
- [ ] `STOP` test: still receives `OK stop` and halts motion (existing test passes).
- [ ] `cancel()` host method test: mock conn asserts `X\n` is sent via `send_fast`.
- [ ] All existing tests: `uv run --with pytest python -m pytest -q` at 1035/8.

### Build
- [ ] Clean build: `python3 build.py --clean` completes without errors.

## Implementation Plan

1. In `CommandProcessor.cpp`:
   - Add `X` verb handler (immediately before or after the STOP handler):
     ```cpp
     if (strcmp(verb, "X") == 0) {
         _robot.driveController.cancel(_robot.systemTime(), replyFn, ctx);
         replyOK(rbuf, sizeof(rbuf), "x", nullptr, corr_id, replyFn, ctx);
         return;
     }
     ```
   - Update `STOP` handler to call `_robot.driveController.cancel(...)` instead of
     `_robot.driveController.stop(...)`.
   - Update `HELP` response string to include `X`.
2. In `host/robot_radio/robot/protocol.py`, add `cancel()` method after `vw()`:
   ```python
   def cancel(self) -> None:
       """Cancel the active motion command (hard stop). Sends X."""
       self.send_fast("X")
   ```
3. Update `tests/dev/test_motion_verbs_v2.py` to cover `X`.
4. Run clean build and verify baseline.

## Notes

- `DriveController::cancel()` (implemented in ticket 004) is the single teardown entry
  point: calls `_activeCmd.cancel(HARD)` then `_mc.stop()`. Both `X` and `STOP` route
  through it.
- If `X` is sent when IDLE (`_activeCmd.active()` is false), `cancel()` should still call
  `_mc.stop()` for motor safety but skip emitting `EVT cancelled` (no active command).
  This is handled inside `DriveController::cancel()` by checking `_activeCmd.active()`
  before calling `_activeCmd.cancel(HARD)`.

## Bench Verification (stakeholder-deferred)

- Start `VW 200 0`, wait 200 ms, send `X` → robot stops immediately; `EVT cancelled`
  received; `OK x` also received.
- Start `VW 200 0`, send `STOP` → same result; `OK stop` received.
- Send `X` when idle → `OK x` received; no crash; no `EVT cancelled`.
- `S 200 200` then `X` → motors stop; `S` is no longer active.
