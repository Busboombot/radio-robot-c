---
id: 009
title: Convert rogo CLI (cli.py) to protocol v2 + add drive --resend keepalive knob
status: done
use-cases:
- SUC-001
- SUC-002
depends-on:
- '002'
- '003'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Convert rogo CLI (cli.py) to protocol v2 + add drive --resend keepalive knob

## Description

Bench testing revealed that `host/robot_radio/io/cli.py` (the installed `rogo`
command) was never converted to protocol v2. It connects to the relay, but
multiple commands still use v1 verbs and parsing, causing crashes or garbage
output against the v2 firmware. This ticket performs a full v2 conversion of
`cli.py` and adds a `--resend MS` flag to the `drive stream` subcommand so the
stakeholder can tune the keepalive cadence to diagnose motor "throbbing".

Note: `tests/rogo.py` is a separate old standalone direct-USB harness and is
OUT OF SCOPE. The installed `rogo` entrypoint is `robot_radio.io.cli:main`.

## Acceptance Criteria

### 1. Full v2 conversion of `host/robot_radio/io/cli.py`

- [x] Audit every command and helper in `cli.py` for v1 verb strings. Grep for:
  `EZ`, `SO`, `SSE`, `SSO`, `SSL`, `SSC`, `TN`, `ROT`, `OO`, `SI`, `K+`,
  `_sign`, `set_watchdog`, `query_ol`, `set_world_pose`. No v1 verbs may remain
  in any executed code path.
- [x] `proto.query_ol()` has been replaced with `otos_get_linear_scalar()`.
  Verify this is already done and leave it; convert if it is not.
- [x] `proto.set_watchdog(ms)` replaced with `set_config(sTimeout=ms)`. v2 has
  no `set_watchdog` verb. Fixed in `_spin_to_world_yaw`, `_daemon_spin_to_yaw`,
  and `cmd_goto`.
- [x] `proto.set_world_pose(...)` / the `SI` verb: v2 has no `SI` verb.
  Decision: replaced with `robot.set_world_pose()` which delegates to
  `proto.otos_set_position()` (OV command). The heading is converted from
  degrees to centi-degrees for the OV command.
- [x] `robot.rotate` / `robot.angle` (`cmd_rotate` / `cmd_angle`): these are
  Cutebot-only PR/PA servo verbs not implemented in v2 Nezha firmware. Marked
  unsupported with clear error messages. Use `rogo turn` or `rogo drive --ms`
  as alternatives.
- [x] `rogo enc` reads encoder values via v2 `SNAP` → `TLM` (`parse_tlm`),
  NOT the v1 `ENC` response line. Must return real values after a drive.
- [x] `rogo opos` (new command) reads fused pose via v2 `SNAP` → `TLM`
  (`parse_tlm`). The old `rogo pose` is camera-based (AprilTag); the new
  `rogo opos` reads robot OTOS pose. NOT the v1 `SO` response line.
- [x] `rogo line` reads line sensor via v2 `TLM` (SNAP → TLM line= field),
  NOT v1 `LS`.
- [x] `rogo color` reads color sensor via v2 `TLM` (SNAP → TLM color= field),
  NOT v1 `CS`.
- [x] `rogo stop` sends the v2 `STOP` verb, NOT v1 `X`. Output changed to
  print "STOP" not "X".
- [x] All remaining commands (`ping`, `id`, `config`, `drive`, `servo`, etc.)
  verified to use the v2 `NezhaProtocol` / `Nezha` API exclusively.

### 2. Add `--resend MS` to `drive stream`

- [x] `rogo drive <L> <R> stream [--resend MS]` accepts an integer `--resend`
  argument (milliseconds). When provided, the streaming `S` keepalive is sent
  every MS milliseconds on the host side.
- [x] Default value is 150 ms (30 % of the 500 ms firmware `sTimeout`).
- [x] The `--resend` flag is documented in the command's `--help` output with a
  description explaining its purpose (keepalive cadence, diagnose throbbing).
- [x] Values <= 0 are rejected with a clear error message.

### 3. Tests in `host/tests/` (mock `SerialConnection`)

- [x] Tests cover the v2 wire encoding for at minimum: `enc`, `pose`, `stop`,
  and `set_config` (formerly `set_watchdog`) commands.
- [x] Tests cover `TLM` read-back parsing for `enc` and `pose` commands.
- [x] Tests cover the `--resend` flag: verify the correct cadence argument is
  forwarded and that invalid values (<=0) raise an error.
- [x] CRITICAL: mock read loops must NOT return an empty `[]` — they must
  return actual data or a finite `side_effect` ending with a terminal line.
  An empty mock spins forever and causes an OOM. This rule must be followed
  in every new test.
- [x] All new tests are sub-second individually.

### 4. Full suite stays green

- [x] `uv run --with pytest python -m pytest -q` passes all ~1012+ tests in
  ~1 second (no new hangs or failures). Final: 1038 passed, 1 skipped in 0.73s.

### 5. Bench verification (stakeholder-run, mark pending)

- [ ] `uv run rogo --port <relay> enc` returns real encoder values.
- [ ] `uv run rogo --port <relay> pose` returns real fused pose.
- [ ] `uv run rogo --port <relay> drive 0 0` and `stop` work without error.
- [ ] `uv run rogo --port <relay> drive 150 150 stream --resend 150` runs
  without crashing; vary `--resend` (e.g. 100, 200, 300) to assess throbbing.

## Implementation Plan

### Approach

1. Read `host/robot_radio/io/cli.py` in full.
2. Grep for all v1 verb strings listed in criterion 1.
3. Convert or remove each offender using the v2 `NezhaProtocol` / `Nezha`
   API (already ported in tickets 002 and 003).
4. Add the `--resend` argument to the `drive stream` subparser and thread it
   through to the keepalive loop.
5. Write mock-based tests for each converted command and for `--resend`.

### Files to modify

- `host/robot_radio/io/cli.py` — primary change target
- `host/tests/test_cli.py` (create if it does not exist) — new tests

### Files to read for context

- `host/robot_radio/io/cli.py` — current state
- `host/robot_radio/protocol_v2.py` — `NezhaProtocol` API (ticket 002)
- `host/robot_radio/nezha.py` — `Nezha` high-level driver (ticket 003)
- `host/tests/test_protocol_v2.py` — existing v2 test patterns and mock style

### Testing plan

Use `unittest.mock.MagicMock` for `SerialConnection`. Follow the existing
pattern in `test_protocol_v2.py`. For read-loop mocks, always provide a
`side_effect` list that terminates with a `TLM` or `OK` line — never an
empty list or infinite mock.

Run `uv run --with pytest python -m pytest -q` after every batch of changes.

### Documentation updates

- Update `--help` strings in the affected `cli.py` subparsers to remove any
  references to removed v1 commands and add the `--resend` description.
- No README changes required unless a top-level usage section references
  removed commands.
