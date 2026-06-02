---
id: 008
title: 'Host controller migration: copy robot_radio into repo, rewrite protocol layer
  for v2'
status: done
use-cases:
- SUC-007
depends-on:
- '003'
- '004'
- '005'
- '006'
issue: protocol-v2-raw250-hard-break.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 009-008: Host controller migration — copy robot_radio into repo, rewrite protocol layer for v2

## Description

Copy the `robot_radio` package from
`/Volumes/Proj/proj/league-projects/scratch/radio-robot/robot_radio`
into this repo at `host/robot_radio/`, then rewrite its protocol layer
(`robot/protocol.py` and `robot/nezha.py`) to speak v2.

**What to copy verbatim** (no changes yet):
- `config/`, `controllers/`, `nav/`, `path/`, `kinematics/`, `sensors/`, `io/`
- `robot/robot.py`, `robot/nezha_state.py`, `robot/nezha_kinematic.py`, `robot/__init__.py`

**What to rewrite for v2**:
- `robot/protocol.py` → `NezhaProtocol` class: replace all v1 encode/parse logic
  with v2 counterparts.
- `robot/nezha.py` → `Nezha` class: update public API to use v2 commands.

**New `NezhaProtocol` wire encoding**:

| Method | v1 command | v2 command |
|---|---|---|
| `drive(l, r)` | `S{+l}{+r}` | `S {l} {r}` |
| `timed(l, r, ms)` | `T{+l}{+r}{+ms}` | `T {l} {r} {ms}` |
| `distance(l, r, mm)` | `D{+l}{+r}{+mm}` | `D {l} {r} {mm}` |
| `go_to(x, y, s)` | `G{+x}{+y}{+s}` | `G {x} {y} {s}` |
| `stop()` | `X` | `STOP` |
| `grip(deg)` | `G{+deg}` | `GRIP {deg}` |
| `zero_encoders()` | `EZ` | `ZERO enc` |
| `zero_otos()` | `SZ` | `ZERO pose` |
| `read_encoders()` → parse `ENC` | parse `TLM enc=` | |
| `set_config(...)` | `K{key}{+val}` x N | `SET key=val …` |
| `get_config()` | `K` (24 lines) | `GET` (1 line) |
| `ping()` | (none) | `PING` → `OK pong t=<ms>` |
| `echo(payload)` | (none) | `ECHO <payload>` → `OK echo <payload>` |
| `get_id()` | `HELLO` | `ID` → `ID …` |
| `stream(ms)` | `SSE+1` / `SSO+1` etc. | `STREAM {ms}` |
| `snap()` | (none) | `SNAP` |

**Response parsing**:
- All responses start with `OK`, `ERR`, `EVT`, `TLM`, `CFG`, or `ID`.
- `EVT done T/D/G` replaces `T+DONE`/`D+DONE`/`G+DONE`.
- `EVT safety_stop` replaces `SAFETY_STOP`.
- `CFG key=val …` is the `GET` response.
- `TLMFrame` dataclass: `t`, `mode`, `enc`, `pose`, `line`, `color` (all optional).

**`_sign()` helper**: delete. Space-separated integers need no special encoding.

**Streaming keepalive**: `S {l} {r}` with spaces; no sign prefix.

**Package installability**: create `host/pyproject.toml` with `[project]` metadata
and `[tool.uv.workspace]` or similar so `uv pip install -e host/` works.

## Acceptance Criteria

- [x] `host/robot_radio/` exists in this repo; `host/pyproject.toml` (or `setup.py`) makes it installable.
- [x] `NezhaProtocol.drive(200, 150)` sends `S 200 150\n` (no sign prefix, no packing).
- [x] `NezhaProtocol.stop()` sends `STOP\n`.
- [x] `NezhaProtocol.get_config()` sends `GET\n` and parses the `CFG …` response into a dict.
- [x] `NezhaProtocol.set_config(ml=0.487)` sends `SET ml=0.487\n` and parses `OK set …`.
- [x] `NezhaProtocol.ping()` sends `PING\n` and parses `OK pong t=<n>` → returns `(t_robot_ms, rtt_ms)`.
- [x] `TLMFrame` dataclass parses `TLM t=… enc=… pose=…` correctly.
- [x] `EVT done T/D/G` parsed correctly by blocking drive wrappers.
- [x] `EVT safety_stop` parsed and surfaces appropriately.
- [x] All v1 `_sign()` calls and sign-prefix parsing removed.
- [ ] [BENCH] Host controller drives robot end-to-end over the relay using v2 (full workflow: connect, `ID`, `SET` a config key, `STREAM 40`, observe TLM frames, drive `T 200 200 1000`, observe `EVT done T`).

## Implementation Plan

**Approach**: Copy the package; do a targeted rewrite of `protocol.py` and `nezha.py`.
Do not change `nav/`, `path/`, `kinematics/`, `sensors/` — they are protocol-neutral.
`io/serial_conn.py` is unchanged (still sends/receives raw text lines).

**Files to create**:
- `host/robot_radio/` (entire package tree, copied)
- `host/pyproject.toml`

**Files to rewrite**:
- `host/robot_radio/robot/protocol.py` — full v2 rewrite
- `host/robot_radio/robot/nezha.py` — update to v2 API

**Testing**:
- Unit tests (no hardware): mock `SerialConnection` to inject canned v2 response lines;
  verify `NezhaProtocol` methods produce correct commands and parse responses correctly.
- [BENCH] End-to-end drive test over relay.
