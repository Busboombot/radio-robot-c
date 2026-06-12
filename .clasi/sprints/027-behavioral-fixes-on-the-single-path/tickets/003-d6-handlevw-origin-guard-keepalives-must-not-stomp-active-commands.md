---
id: '003'
title: "D6: handleVW Origin guard — keepalives must not stomp active commands"
status: open
use-cases:
  - SUC-003
depends-on:
  - "027-001"
github-issue: ''
issue: d06-keepalive-must-not-mutate-active-command.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 027-003: D6 — handleVW Origin guard

## Description

`handleVW`'s no-stop-params branch at line 1819–1821 of
`source/control/MotionController.cpp` (pre-026) calls
`activeCmd().setTarget(v, ω)` on any active MotionCommand. A `VW 0 0` or
`S 0 0` keepalive arriving during a TURN overwrites the TURN's ω to 0.
The HEADING stop can no longer fire; the TIME net fires 2×nominal+2 s later
and emits `EVT done TURN` at the wrong heading. Silent navigation corruption.

Fix: add `MotionCommand::Origin` enum. In the no-stop-params branch, only call
`setTarget` when `origin == Origin::VW`; for any other origin, reset the
watchdog and reply `OK vw busy=<origin>`, do NOT call `setTarget`.

**REVALIDATE-AFTER-026:** This ticket targets the pre-026 code location
(`MotionController.cpp` line 1819). If sprint 026 has landed when this ticket
executes, the programmer MUST edit `source/app/MotionCommandHandlers.cpp`
instead. Verify the file location before starting implementation.

## Acceptance Criteria

- [ ] `MotionCommand::Origin` enum added to `source/control/MotionCommand.h`
      with values: `VW`, `TURN`, `G`, `T`, `D`, `R`, `RT`.
- [ ] `setOrigin(Origin)` and `origin()` accessors added to `MotionCommand`.
- [ ] Each `begin*()` method in `MotionController` calls `_activeCmd.setOrigin(X)`:
  - `beginVelocity` sets `Origin::VW`
  - `beginTurn` sets `Origin::TURN`
  - `beginGoTo` sets `Origin::G` (both PRE_ROTATE and PURSUE configure paths)
  - `beginTimed` sets `Origin::T`
  - `beginDistance` sets `Origin::D`
  - `beginArc` sets `Origin::R`
  - `beginRotation` sets `Origin::RT`
- [ ] In `handleVW` no-stop-params branch: only call `setTarget(v, ω)` when
      `activeCmd().origin() == Origin::VW`; for any other origin, reply
      `OK vw busy=<origin_name>` and do NOT call `setTarget`.
- [ ] `protocol.py` docstrings for `vw()` and `drive()` updated to remove the
      recommendation to use `VW`/`S` as keepalives during non-VW commands.
- [ ] `test_d6_cannot_stomp_turn` in `host_tests/test_vw_converters.py`
      promoted from xfail to passing (remove the xfail mark; test passes).
- [ ] `test_scenario_keepalive_kills_turn` in `host_tests/test_incident_scenarios.py`
      promoted from xfail to passing (remove the xfail mark).
- [ ] All `host_tests/` and `host/tests/` pass.
- [ ] Firmware builds clean; `--clean` build before bench.

## Implementation Plan

### Approach

Step 1: Add `Origin` enum to `MotionCommand.h` and trivial accessors
`setOrigin(Origin)` / `Origin origin() const`. No behavioral logic in
`MotionCommand.cpp` beyond the getter/setter.

Step 2: Add one `setOrigin(Origin::X)` call at the top of each `begin*()`
method in `MotionController.cpp`. Trivial per-method one-liner.

Step 3: In `handleVW` no-stop-params branch — check the origin and guard the
`setTarget` call. The watchdog is already reset by inbound command receipt in
`runCommsIn()`; no additional watchdog-reset call is needed inside the handler.
Format the busy reply as: `OK vw busy=TURN` (or `busy=G`, etc.) using the
existing `CommandProcessor::replyOK` call pattern.

Step 4: Update `protocol.py` docstrings. Grep for `vw()` and `drive()`;
update the keepalive guidance section.

Step 5: Promote the two xfail tests.

### Files to modify

- `source/control/MotionCommand.h` — Origin enum + accessors.
- `source/control/MotionCommand.cpp` — implement accessors if not inline.
- `source/control/MotionController.cpp` — `setOrigin` calls in `begin*` and
  `handleVW` guard (pre-026). Post-026: `MotionCommandHandlers.cpp` for the
  guard; `MotionController.cpp` still for the `begin*` `setOrigin` calls.
- `host/robot_radio/robot/protocol.py` — docstring update.
- `host_tests/test_vw_converters.py` — remove xfail from
  `test_d6_cannot_stomp_turn`.
- `host_tests/test_incident_scenarios.py` — remove xfail from
  `test_scenario_keepalive_kills_turn`.

### Testing plan

```
python3 build.py
uv run pytest host_tests/ -v
```

Confirm: `test_d6_cannot_stomp_turn` and `test_scenario_keepalive_kills_turn`
both show as PASSED (not xfailed).

### Documentation updates

`protocol.py` docstring only. No protocol-v2.md change (no wire-protocol
change; `OK vw busy=X` is a new reply format on a non-standard branch but
does not affect documented behavior for conforming callers).

## Notes

- **026-churn flag:** The `handleVW` handler location changes when 026 lands.
  If `source/app/MotionCommandHandlers.cpp` exists, use it for the guard.
  The `begin*` `setOrigin` calls always live in `MotionController.cpp`.
- Origin enum overhead: 4 bytes per MotionCommand instance. Negligible.
