---
id: '006'
title: "sensor= modifier \u2014 attach SENSOR stop to T, D, TURN commands"
status: done
use-cases:
- SUC-006
depends-on:
- '005'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# sensor= modifier — attach SENSOR stop to T, D, TURN commands

## Description

Add an optional `sensor=<ch>:<op>:<thr>` token to the T, D, and TURN command handlers.
When present, the firmware parses it and appends a `SENSOR` stop condition to the
MotionCommand alongside its primary stop. The EVT name is unchanged (e.g. `EVT done T`
fires whether the TIME or the SENSOR stop won).

**Wire format (additive — backward compatible):**
```
T  <l> <r> <ms>          [sensor=<ch>:<op>:<thr>] [#id]
D  <l> <r> <mm>          [sensor=<ch>:<op>:<thr>] [#id]
TURN <heading_cdeg>       [sensor=<ch>:<op>:<thr>] [eps=<cdeg>] [#id]
```

Channel names → channel indices (matching `StopCondition.cpp::getSensorValue`):
| Name | Channel index |
|---|---|
| line0 | 0 |
| line1 | 1 |
| line2 | 2 |
| line3 | 3 |
| colorR | 4 |
| colorG | 5 |
| colorB | 6 |
| colorC | 7 |

Op: `ge` → `StopCondition::Cmp::GE`; `le` → `StopCondition::Cmp::LE`.
`thr`: integer; sensor values are `uint16_t` raw ADC counts.

**Parsing:** After the primary positional args are consumed, scan `kvs[]` (already
parsed by `parseKV`) for key `"sensor"`. If found, split the value on `:` to extract
`ch_name`, `op_str`, `thr_str`. Resolve `ch_name` to an index (ERR if unknown);
resolve `op_str` to Cmp (ERR if not `ge`/`le`); parse `thr_str` as int. Call
`_activeCmd.addStop(makeSensorStop(ch_idx, (float)thr, cmp))`.

**`DriveController` API change:** The `beginTimed`, `beginDistance`, and `beginTurn`
signatures accept an optional `StopCondition* sensorStop` pointer (or a bool + condition
inline). Simplest: pass the SENSOR condition separately and call `addStop` inside `begin*`
after the primary stop. CommandProcessor parses `sensor=`, builds the condition, and
passes it through. Alternative: parse inside CommandProcessor and call `activeCmd().addStop()`
AFTER calling `begin*`. The latter avoids changing `begin*` signatures — prefer this approach
since `begin*` already exposes `_activeCmd` via `activeCmd()` and the sensor condition is
additive. Sequence: call `begin*(...)` first (which calls `start()`), then
`_robot.driveController.activeCmd().addStop(sensorCond)`. BUT: `start()` captures the
baseline — adding a stop after `start()` is safe since `evaluate()` reads the baseline
from `MotionCommand` not from the stop condition itself. Confirm this is safe by reading
`MotionCommand::start()` (it captures baseline but does not copy stops; `addStop` operates
on the array directly). This is safe.

**Host wrapper:** `drive_until_sensor(channel, threshold, direction, ...)` in `protocol.py`
as a helper that wraps T with a `sensor=` modifier. Also update `timed()`, `distance()`,
`turn()` to accept optional `sensor` argument that appends the modifier.

## Acceptance Criteria

- [x] `T 200 200 5000 sensor=line0:ge:512` stops early when line[0] ≥ 512, emits `EVT done T`.
- [x] `T 200 200 5000 sensor=line0:ge:512` also stops at 5 s if sensor never trips.
- [x] `sensor=` absent → same behaviour as before (backward compatible).
- [x] Unknown channel name → `ERR badarg sensor`.
- [x] Unknown operator → `ERR badarg sensor`.
- [x] `addStop` after `start()` is verified safe (check against MotionCommand implementation;
  note in code comment).
- [x] SENSOR conditions for all 8 named channels fire correctly (Python unit test using
  `test_stop_condition.py` pattern — SENSOR kind was tested in 017 but channel-name
  resolution is new here).
- [x] `drive_until_sensor()` wrapper in `protocol.py`.
- [x] `uv run --with pytest python -m pytest -q` passes at 1292/8 baseline (1324/8 after new tests).
- [x] Clean build: `python3 build.py --clean` succeeds.
- NOTE: On-robot bench (`T 200 200 10000 sensor=line0:ge:512` — robot stops on line crossing) is stakeholder-deferred.

## Implementation Plan

### Channel-name resolution helper
Add a static helper function in `CommandProcessor.cpp`:
```cpp
static bool parseSensorToken(const char* value, uint8_t& ch, float& thr, StopCondition::Cmp& cmp);
```
Splits `value` (e.g. "line0:ge:512") on `:` using `strchr`; resolves channel name to
index; resolves op string; parses threshold. Returns false on any error.

### Files to modify
- `source/app/CommandProcessor.cpp`:
  - Add `parseSensorToken` static helper
  - In T handler: after `beginTimed(...)`, check kvs for `"sensor"` key; parse and call
    `_robot.driveController.activeCmd().addStop(makeSensorStop(...))`
  - In D handler: same pattern
  - In TURN handler: same pattern (after `beginTurn(...)`)
- `host/robot_radio/robot/protocol.py`:
  - Add `drive_until_sensor(channel, threshold, direction, duration_ms, left_mms, right_mms)` wrapper
  - Update `timed()`, `distance()`, `turn()` to accept optional `sensor` kwarg that appends the token

### Testing plan
- Python unit test: verify `sensor=` parsing produces correct `ch`, `op`, `thr`;
  verify OR-stop semantics (sensor fires before TIME → still EVT done T).
- Existing SENSOR stop-condition tests (`test_stop_condition.py`) must still pass.
- Full pytest suite: `uv run --with pytest python -m pytest -q`.
- Bench (stakeholder-deferred): `T 200 200 10000 sensor=line0:ge:512` — robot stops on
  line crossing; confirm EVT done T arrives.
