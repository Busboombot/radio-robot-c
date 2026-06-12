---
id: 028
title: Calibration and host consolidation
status: done
branch: sprint/028-calibration-and-host-consolidation
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- a7-consolidate-calibration
- a6-extract-library-logic-from-cli
- d10-trustworthy-telemetry-stream
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 028: Calibration and host consolidation

## Goals

One calibration pipeline whose outputs all land somewhere. CLI and MCP
front-ends call the same library functions for everything they share.
Telemetry drop rate is measurable.

## Problem

Three independent problem clusters converge here:

**Calibration (a7):** Calibration logic exists in four places:
`host/calibrate_angular.py` (718 lines), `host/calibrate_linear.py` (555),
`host/calibrate_verify.py`, and `robot_radio/io/calibrate.py` (1101) —
with literal duplicates (`_deep_merge`, `mean_stdev`, `scale_to_int8`, plus
re-duplication in cli.py). Worse: calibration outputs (`rotationalSlip`,
`turnScale`, `distScale`) were calibrated, stored, and registered — but read
by nothing in firmware (D2; `rotationalSlip` was wired in sprint 024, others
remain dead). The a8 lint (sprint 025) enforces the registry check, so
landing a7 after a8 means the CI gate catches any newly dead calibration key
immediately.

**CLI/MCP logic extraction (a6, partial):** `io/cli.py` (2262 lines) contains
library logic that `io/robot_mcp.py` (1016 lines) also needs but cannot import
cleanly, so the two front-ends drift. The MCP path the main agent uses and the
CLI path a human uses are not the same code. The minimum extraction for this
sprint: calibration push → the consolidated calibration package (a7), robot
construction/port resolution (`_make_robot`) → `robot/` or `config/`. Full a6
completion (controllers, TLM snapshot parsing) can trail into sprint 029 as
a1 pulls those pieces out.

**Telemetry firmware items (d10):** The host-side foundation was sprint 025
(reader thread, d11a). The remaining firmware-side items are: seq numbers
(uint16 wrap) in TLM frames so the host can measure drop rate; low idle rate
(emit at `max(tlmPeriodMs, 500)` when IDLE > grace instead of silence); TLM
channel binding (`STREAM` captures its reply channel; other channels don't
steal it); move the `tlmPeriodMs` clamp from `telemetryEmit` to the
STREAM/SET handler.

## Solution

**a7:** Consolidate into one calibration package under `robot_radio/`
(suggested: `robot_radio/calibration/`). Reduce top-level scripts to thin
entry points. De-duplicate helpers. For every calibrated value: wire to a
consumer or remove from the pipeline. a8's lint (sprint 025) enforces going
forward.

**a6 (partial):** Extract calibration push + `_make_robot`/port-resolution
into library modules. cli.py and robot_mcp.py become thin adapters over the
same calls for these functions. The controller loops (A1 territory) stay in
cli.py for now.

**d10:** In `source/robot/Robot.cpp` / `source/app/` — add `seq=<n>` to TLM
frame; replace idle-silence with idle at `max(tlmPeriodMs, 500)`; bind TLM
channel in STREAM handler; move clamp. Update `host/robot_radio/` TLMFrame
parsing to surface `seq`; expose a drop-rate metric.

## Success Criteria

- One calibration package under `robot_radio/`; zero duplicated helpers; every
  calibration-output key read somewhere in `source/` (a7 lint enforcement).
- CLI and MCP call the same library functions for calibration push and robot
  construction; no duplicated logic between the two front-ends for these
  functions.
- Host-side drop rate from TLM seq gaps < 2% during a full G run over the
  relay; stream survives idle→drive→idle without reconnecting; a radio command
  does not kill the serial stream.

## Scope

### In Scope

- New `robot_radio/calibration/` package; thin top-level calibrate scripts.
- `robot_radio/io/cli.py`, `robot_radio/io/robot_mcp.py` — calibration push
  and `_make_robot` extracted.
- `robot_radio/robot/` or `config/` — port-resolution/robot-construction module.
- `source/robot/Robot.cpp` (or `source/app/` post-026) — d10 seq, idle rate,
  channel binding, clamp relocation.
- `host/robot_radio/` TLMFrame parsing — seq field, drop-rate metric.
- `docs/protocol-v2.md` — idle-rate and channel-binding documentation.

### Out of Scope

- CLI controller loops (A1 territory, sprint 029).
- nav/ navigator.py / controllers/ (sprint 029).
- Full a6 completion — controllers and TLM snapshot parsing trail into sprint
  029 as a1 pulls those pieces.
- D12 numerical/timing hygiene items (anytime filler; see below).

## Test Strategy

- Calibration package: unit tests for each helper with a single test target
  (no duplication to maintain); integration test that calibrate_angular and
  calibrate_linear produce identical config output as before.
- CLI/MCP: integration test that both front-ends call the same underlying
  function for calibration push and robot construction.
- D10: host-side drop-rate test during a 60 s drive over the relay.

## Architecture Notes

The a8 lint (sprint 025) is a hard prerequisite for a7: the lint enforces that
calibration outputs land somewhere, so a7 must ship after a8 is in CI or
alongside a fix to register/consume the newly-consolidated keys.

A6 partial scope (calibration push + robot construction only) is chosen because
the controller loops in cli.py belong to the A1 ownership decision (sprint 029)
and should not be moved until that decision is made. Pulling construction/port
resolution forward is warranted because the CLI/MCP drift directly causes
"works for the human, fails for the agent" reports.

**Anytime filler items** — these are independent and can be slotted into this
sprint if capacity allows, or executed as standalone commits at any point:

- `d12-numerical-and-timing-hygiene` (EKF Q loop-rate coupling, dispatch
  latency, EVT truncation, reset ordering fragility). None is an active motion
  runaway; each erodes estimator trust. Good filler alongside firmware work.
- `set-config-validation` (if not landed in sprint 027). SET raw atof/atoi
  with no range checks can break live control config; see the issue for the
  minimal fix.
- `cmd_goto → nav/` fold-in — the cheap first step of A1 (fold cli.py's
  inline pure-pursuit into nav/) is independent of the A1 ownership decision
  and can be pulled forward at any time.

## Why Fourth

The a7 consolidation depends on a8's lint (sprint 025) already enforcing
every-key-is-read. The d10 firmware items fit here because the host-side
foundation (d11a reader thread, sprint 025) is already in place, and the
behavioral sprint (027) has proven the single dispatch path on the field.

## Sizing

Medium — approximately 1–2 focused sessions.

## GitHub Issues

(None yet — link when created.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | SNAP/STREAM TLM consistency — close or forward field-024 lead A | — |
| 002 | Consolidate calibration into robot_radio/calibration/ package (a7) | — |
| 003 | Extract make_robot and push_calibration to shared library modules (a6 partial) | 028-002 |
| 004 | SET validation — typed parse, range checks, atomic apply | — |
| 005 | D10 firmware telemetry — seq numbers, idle rate, channel binding, clamp relocation | 028-001 |

Tickets execute serially in the order listed.
001 and 002 are independent and can start immediately.
003 depends on 002 (needs the calibration package).
004 is firmware-only and independent of all host tickets.
005 depends on 001 (SNAP investigation informs the seq/channel-binding work).
