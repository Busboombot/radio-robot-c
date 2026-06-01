---
id: '006'
title: "mbdeploy Package \u2014 Build, Deploy, List, Probe"
status: done
branch: sprint/006-mbdeploy-package-build-deploy-list-probe
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
issues:
- mbdeploy-a-standalone-micro-bit-deploy-package-build-deploy-list-probe.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 006: mbdeploy Package — Build, Deploy, List, Probe

## Goals

Consolidate the loose deploy scripts (`scripts/deploy.py`, `scripts/build_and_deploy.py`,
`scripts/build.py`, `scripts/lib/device_link.py`) into a standalone, pipx-installable
Python package `mbdeploy/` located as a sub-directory of this repo. The package provides
a single console command `mbdeploy` with four subcommands: `build`, `deploy`, `list`, and
`probe`. It replaces all existing script-based tooling and introduces a persistent device
registry at `config/devices.json` that identifies each connected micro:bit by its pyOCD
Unique ID, assigns it a stable enumeration number, and prevents accidental flashing of
the radio relay board.

## Problem

Deploy tooling is fragmented: three loose scripts, an ad-hoc device registry
(`scripts/lib/known_devices.json`), and no reliable way to target a specific board by
name or number. Flashing the radio relay by accident is a real hazard. The tooling is
not installable, not portable, and not testable in isolation.

## Solution

Build `mbdeploy/` as a proper Python distribution (hatchling, src layout) installable
via `pipx install --editable ./mbdeploy`. Port the proven device-link primitives from
`scripts/lib/device_link.py` into `mbdeploy/src/mbdeploy/devices.py`, extend them with
a `config/devices.json` registry layer (stable enum numbers, persistent across probe
runs), and wire everything into a four-subcommand CLI in `cli.py`. Delete the old
scripts. Update `justfile`, root `pyproject.toml`, and `README.md`.

Firmware is not touched. All project paths are CWD-relative so the package is portable
when extracted and published later.

## Success Criteria

- `pipx install --editable ./mbdeploy` then `mbdeploy --help` lists all four subcommands.
- `mbdeploy list` shows connected board UID + `/dev/cu.*` port.
- `mbdeploy probe` writes `config/devices.json` with `enum: 1` for the board; re-running
  keeps enum=1 and refreshes port/announcement.
- `mbdeploy build` produces `MICROBIT.hex` via the root `build.py`.
- `mbdeploy deploy 1`, `deploy gutov`, `deploy /dev/cu.usbmodem...` all resolve to the
  same UID; `deploy --build 1` builds then flashes; a relay board is refused without
  `--force-relay`; an unknown enum errors "device not connected".
- Unit tests pass: relay refusal, unique-device auto-pick, all four target-resolution
  paths (enum/name/path/uid), and "device not connected" error.
- `just probe`, `just list`, `just deploy 1`, `just build-deploy -- 1` smoke-test the
  justfile recipes against the installed command.
- Old `scripts/` directory is fully removed.

## Scope

### In Scope

- `mbdeploy/` package scaffold: `pyproject.toml` (hatchling, entry point, deps
  pyocd+pyserial), `src/mbdeploy/{__init__,cli,devices,builder}.py`, `README.md`.
- `devices.py`: port `flashable_probes`, `port_serial_map`, `probe_type`, `is_relay`
  from `scripts/lib/device_link.py`; add registry layer (`load_devices`, `save_devices`,
  `assign_enum`, `probe_all`, `resolve_target`).
- `cli.py` + `builder.py`: argparse with four subcommands; `build` shells to project
  build; `deploy` resolves target, guards relay, re-confirms live UID, flashes via pyocd.
- Unit tests for relay refusal, auto-pick, target resolution (all four paths), and
  "device not connected".
- Cutover: delete `scripts/deploy.py`, `scripts/build_and_deploy.py`, `scripts/build.py`,
  `scripts/lib/`, `scripts/` (empty); update `justfile` (add `mbd-install`, update
  recipes), root `pyproject.toml` (drop pyocd, keep pyserial), `README.md`.

### Out of Scope

- Extracting/publishing `mbdeploy` to PyPI (future).
- Windows or Linux port of the UID-to-serial-port join (macOS `ioreg` only; off-macOS
  degrades gracefully to empty map).
- Firmware changes of any kind.
- Sprint 005 (Navigation Layer) — left untouched.
- `Dockerfile` — unchanged.

## Test Strategy

Unit tests use monkeypatching to mock `flashable_probes()`, `port_serial_map()`, and
the `config/devices.json` registry. Tests do not require connected hardware.

Tests cover:
- Relay refusal (with and without `--force-relay`)
- Unique-device auto-pick when exactly one non-relay device is present
- Ambiguous auto-pick error when multiple non-relay devices are present
- Target resolution for all four input types: enum digit, `/dev/` path, 40-52 hex UID,
  5-char name via `common_name` and via `device_name` (distinct firmware fields)
- "Device not connected" error when resolved UID is not in the live probe list
- Bench rig may be silent to HELLO; tests must not depend on live announcement

Smoke tests via `just` recipes confirm end-to-end CLI invocation against installed
command on a machine with a connected board.

## Architecture Notes

- `mbdeploy` is dev/build tooling, not firmware. It lives in `mbdeploy/` outside the
  `source/` tree and does not affect the robot firmware architecture.
- The `config/devices.json` registry is a new project-level artifact committed to git.
  It is CWD-relative (with `--config` override) so the package works across project roots.
- Root `build.py` stays at repo root (Dockerfile/upstream dependency). `mbdeploy build`
  shells out to it as `python3 build.py` from CWD (configurable via `--build-cmd`).
- pyOCD Unique ID == USB serial that `ioreg` ties to `/dev/cu.*`. This is the verified
  identity chain — no guessing by port enumeration order.
- The 5-char micro:bit codename appears in different announcement fields for robot
  (`DEVICE:Nezha2:<name>:microbit:...`) vs relay (`DEVICE:RADIOBRIDGE:relay:<name>:...`),
  so target resolution by name must check both `common_name` and `device_name`.
- `is_relay` is generic (matches `RELAY`/`BRIDGE` case-insensitively); `is_robot` is
  dropped — auto-pick selects the unique non-relay device instead.

## GitHub Issues

None.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Package Scaffold | — |
| 002 | devices.py — Primitives and Registry Layer | 001 |
| 003 | cli.py, builder.py, and Unit Tests | 002 |
| 004 | Cutover — Remove scripts, Update justfile and pyproject | 003 |

Tickets execute serially in the order listed.
