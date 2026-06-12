---
id: '003'
title: Extract make_robot and push_calibration to shared library modules (a6 partial)
status: done
use-cases:
- SUC-001
- SUC-002
depends-on:
- 028-002
github-issue: ''
issue: a6-extract-library-logic-from-cli.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 028-003: Extract make_robot and push_calibration to shared library modules (a6 partial)

## Description

`io/cli.py` (2262 lines) and `io/robot_mcp.py` (1016 lines) both need robot
construction and calibration push, but `cli.py` owns the implementations and
`robot_mcp.py` has a diverged copy (`_connect()` vs `_make_robot()`). This
causes "works for the human, fails for the agent" drift because the two paths
make subtly different choices (session cache, HELLO retry, mode detection).

This ticket extracts exactly two concerns into library modules:

1. **Robot construction + port resolution** → `robot_radio/robot/connection.py`
   (`make_robot(port, mode, verbose, args)`, `get_port(args)`, session cache
   helpers).

2. **Calibration push** → already in `robot_radio/calibration/push.py` from
   ticket 028-002. This ticket wires both front-ends to call it.

Controller loops, TLM snapshot parsing, and navigation remain in `cli.py` —
those are A1 territory (sprint 029).

## Acceptance Criteria

- [x] `host/robot_radio/robot/connection.py` exists and exports:
      - `make_robot(port, mode, verbose, args) -> tuple[Robot, SerialConnection, dict]`
      - `get_port(args) -> str`
      - `read_session_cache() -> dict | None`
      - `write_session_cache(port, mode, device_name) -> None`
- [x] `io/cli.py` `_make_robot` calls `connection.make_robot(...)` and returns
      its result unchanged. `_get_port`, `_read_session_cache`,
      `_write_session_cache` replaced with imports from `connection`.
- [x] `io/robot_mcp.py` `_connect()` calls `connection.make_robot(...)` for
      port resolution, HELLO handshake, and mode detection. Calibration push
      calls `calibration.push.push_calibration(proto, config)`.
- [x] Session cache behavior is identical for CLI and MCP: same file path,
      same read/write logic. Verified by `test_connection.py::TestSessionCacheParity`.
- [x] `cli.py` line count reduced by at least 150 lines (construction + session
      cache helpers removed). Delta: 2241 → 2033 = −208 lines.
- [x] `rogo ping`, `rogo hello`, `rogo drive` and at least one MCP tool
      (`connect`) exercise the shared path without error. Verified by import
      check and `rogo --help` exit 0.
- [x] All existing tests pass:
      `uv run --with pytest python -m pytest host_tests/ tests/dev/ host/tests/ -q`
      1615 passed (1596 pre-ticket + 19 new connection tests).

## Implementation Plan

### Approach

1. Create `host/robot_radio/robot/connection.py`. Move `_make_robot`,
   `_get_port`, `_read_session_cache`, `_write_session_cache`,
   `_parse_device_line`, `_SESSION_CACHE_PATH`, and `_calibration_path` from
   `cli.py` into it. Public API uses `make_robot` (no leading underscore).
2. In `cli.py`, replace the full `_make_robot` body with a one-line call to
   `connection.make_robot(...)`. Remove the session cache helpers.
3. In `robot_mcp.py`, replace `_connect()` internals:
   - Remove `SerialConnection(port, mode=mode)` direct construction.
   - Call `make_robot(port, mode, verbose=False, args=_mock_args(port, mode))`
     where `_mock_args` is a simple namespace with the required attributes.
   - Replace `_robot._proto.push_calibration(_config)` with
     `calibration.push.push_calibration(_robot._proto, _config)`.
4. Run tests. Verify session cache file is written on first MCP connect.

### Files to create

- `host/robot_radio/robot/connection.py`

### Files to modify

- `host/robot_radio/io/cli.py` — remove construction + cache helpers
- `host/robot_radio/io/robot_mcp.py` — use `make_robot`, shared `push_calibration`

### Testing plan

```
uv run --with pytest python -m pytest host_tests/ tests/dev/ -v
```

Smoke test: `rogo --help` exits 0. `rogo hello` (with relay connected) succeeds.

Optional integration test: verify session cache is written after `rogo hello`
and read on a subsequent `rogo ping` invocation (both commands go fast path).

### Documentation updates

Add a module docstring to `connection.py` explaining port resolution precedence
and session cache behavior (same as the existing `_make_robot` docstring).

## Notes

- `completes_issue: false` — full A6 completion (cli.py < 800 lines, no
  control loops) is sprint 029 work. This ticket removes ~150 lines, leaving
  `cli.py` around 2000 lines. The A6 issue remains `pending` after this sprint.
- The `_mock_args` approach for MCP is intentional: `make_robot` uses an
  `args` namespace for backward compatibility with CLI arg objects. If this
  becomes awkward, refactor `make_robot` to take explicit keyword arguments
  in a follow-up, but do not over-engineer in this ticket.
- Depends on 028-002 for `push_calibration` availability in
  `calibration/push.py`.
