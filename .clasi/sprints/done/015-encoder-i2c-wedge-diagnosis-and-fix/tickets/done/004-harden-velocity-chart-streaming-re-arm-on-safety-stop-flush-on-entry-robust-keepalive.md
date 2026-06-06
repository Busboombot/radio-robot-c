---
id: '004'
title: "Harden velocity_chart streaming \u2014 re-arm on safety_stop, flush on entry,\
  \ robust keepalive"
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: residual-motor-encoder-wedge-after-stop.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Harden velocity_chart streaming — re-arm on safety_stop, flush on entry, robust keepalive

## Description

`tests/bench/velocity_chart.py` froze after a few seconds because its worker
thread called `NezhaProtocol.stream_drive(...)`, which exits on the **first**
`EVT safety_stop` it sees. A single stale `safety_stop` left in the serial
buffer from a prior run — or a real watchdog blip — permanently terminated the
generator with no recovery, leaving the chart frozen.

The fix replaces the `stream_drive` generator in `_stream_worker` with a custom
robust streaming loop that mirrors the proven raw-serial path established during
diagnosis (see `tests/bench/velchart_repro.py`). The new loop:

1. Flushes on entry: sends `STOP` + `STREAM 0`, waits 50 ms, and calls
   `reset_input_buffer()` to discard any stale `EVT safety_stop` accumulated
   since the last session.
2. Sets the firmware S-watchdog to 10 s (`SET sTimeout=10000`).
3. Enables TLM streaming at 10 Hz (`STREAM 100`), keeping serial load low.
4. Sends `S <speed> <speed>` immediately and re-sends every 150 ms measured by
   `time.monotonic()`, independent of line-read activity.
5. On `EVT safety_stop`: logs the event via `status_queue`, immediately
   re-sends `S` (re-arming the wheels), and **continues** — does not exit.
6. Exits only on `stop_event.is_set()` (SPACE/disconnect) or a real
   `OSError` from pyserial.
7. On exit (finally): sends `STOP` × 3 + `STREAM 0` before disconnecting.

Only `tests/bench/velocity_chart.py` was changed. `protocol.py` is untouched;
other callers retain the original `stream_drive` semantics.

## Acceptance Criteria

- [x] `uv run python tests/bench/velocity_chart.py --help` parses cleanly and prints expected usage.
- [x] `python3 -c "import ast; ast.parse(open('tests/bench/velocity_chart.py').read())"` reports no syntax errors.
- [x] `_stream_worker` flushes the serial buffer on entry (STOP + STREAM 0 + `reset_input_buffer()`) before starting the streaming loop.
- [x] `_stream_worker` sets `sTimeout=10000` and enables `STREAM 100`.
- [x] `_stream_worker` sends `S <speed> <speed>` keepalive every 150 ms via `time.monotonic()`, independent of line-read rate.
- [x] On `EVT safety_stop`, the worker logs the event and re-sends `S` without exiting the loop (re-arm behaviour).
- [x] `protocol.py` (shared library) is unchanged — no regression for other callers.
- [x] Full test suite passes with 1042 tests (no host regressions; `tests/bench` is in norecursedirs).
- [ ] **Bench-validated by team-lead**: chart streams continuously for ≥ 30 s on hardware with no freezes (requires robot connection — not validated by programmer).

## Testing

- **Existing tests to run**: `uv run --with pytest python -m pytest -q` — 1042 passed, 1 skipped.
- **New tests to write**: N/A — `tests/bench` is excluded from pytest (`norecursedirs`); validation is manual hardware bench testing.
- **Verification command**: `uv run --with pytest python -m pytest -q`
