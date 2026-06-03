---
id: 009
title: Connect-time calibration push rewritten to v2
status: done
use-cases:
- SUC-008
depends-on:
- 008
- '001'
github-issue: ''
issue: ''
completes_issue: false
---

# Connect-time calibration push rewritten to v2

## Description

`host/robot_radio/io/cli.py::_push_calibration()` speaks the dead pre-v2
protocol: `KML`, `KMR`, `OO`, `OK` verbs are not recognized by the v2 firmware.
This function is called automatically at connect and via `rogo sync`. It must be
completely rewritten to use v2 verbs.

The relay drops back-to-back writes without an ack; each command must be
sent with ack-gated blocking (wait for `OK` or `ERR` before sending the next).

Also wire in `match_robot_by_id()` from T08 so the host identifies the robot
from the v2 `ID` response and loads the matching config before pushing.

## Files to Modify

- **`host/robot_radio/io/cli.py`** — `_push_calibration(conn)`:
  Complete replacement. New sequence:
  ```
  1. Send HELLO to get ID response
  2. Call match_robot_by_id(id_response) to load config
  3. SET ml=<cfg.calibration.mm_per_wheel_deg_left>
  4. SET mr=<cfg.calibration.mm_per_wheel_deg_right>
  5. SET tw=<cfg.geometry.trackwidth>  (integer mm)
  6. OI                                 (OTOS init — must precede scalar writes)
  7. OL <cfg.calibration.otos_linear_scalar>   (int8)
  8. OA <cfg.calibration.otos_angular_scalar>  (int8)
  # If otos mounting offsets are nonzero in config, add SET odomOffX/odomOffY/odomYaw
  # OK removed — no v2 equivalent; OTOS IMU bias is handled by OI
  ```
  Each send must use ack-gated blocking (wait for response before next send).
  Use `conn.send(cmd, read_ms=200)` or equivalent.

- **`host/robot_radio/io/cli.py`** — `show calibration` command display:
  Update to print v2 verbs (SET ml/mr/tw, OL, OA) instead of KML/KMR/OO/OK.

- **`host/robot_radio/config/robot_config.py`** — import and call
  `match_robot_by_id()` from `_push_calibration()`.

## Approach

1. Read the full `_push_calibration()` implementation and the connect flow
   (lines ~340-440 of cli.py).
2. Replace the function body entirely. Preserve the docstring updated for v2.
3. Update the `show calibration` display function.
4. Write a unit test that captures the sequence of `conn.send()` calls and
   asserts: no `KML`, `KMR`, `OO`, `OK` verbs; correct `SET ml/mr/tw`, `OL`, `OA` verbs.
5. Run `uv run pytest` to confirm existing tests are not broken.

## Acceptance Criteria

- [x] `_push_calibration()` emits only v2 verbs: `SET ml`, `SET mr`, `SET tw`, `OI`, `OL`, `OA`.
- [x] No `KML`, `KMR`, `OO`, `OK`, `KML/KMR` verbs in the emitted sequence.
- [x] Each command is sent with ack-gated blocking (not fire-and-forget).
- [x] Host unit test asserts the correct verb sequence (mocking conn.send).
- [x] `show calibration` prints v2 verbs only.
- [x] `uv run pytest` passes.
- [ ] (Bench deferred to T11) Post-connect `GET tw/ml/mr` matches active robot JSON.

## Testing

- **New tests**: mock `conn.send()`, call `_push_calibration()`, assert v2 verb sequence.
- **Existing tests to run**: full `uv run pytest`
- **Verification command**: `uv run pytest tests/test_push_calibration.py` (new file)
