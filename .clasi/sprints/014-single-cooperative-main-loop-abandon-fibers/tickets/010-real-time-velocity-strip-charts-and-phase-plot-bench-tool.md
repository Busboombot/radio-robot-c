---
id: '010'
title: Real-time velocity strip charts and phase plot bench tool
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: velocity-charting-tool-real-time-strip-charts-phase-plot.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Real-time velocity strip charts and phase plot bench tool

## Description

(What needs to be done and why.)

## Acceptance Criteria

- [x] `tests/bench/velocity_chart.py` exists and is a standalone runnable script
- [x] Connects to the robot using the same `SerialConnection` + `NezhaProtocol` + `Nezha` pattern as `calibrate_linear.py`
- [x] Streams telemetry via `nezha.stream_drive(speeds, period_ms=40, watchdog_ms=500)` in a daemon thread
- [x] Renders three matplotlib panels: left-wheel strip chart, right-wheel strip chart, vR vs vL phase plot
- [x] Strip charts show scrolling time axis (0 = oldest in window), auto-anchored Y ±350 mm/s
- [x] Commanded speed shown as dashed horizontal reference line on both strip charts
- [x] Phase plot has reference line (slope=1 for equal commands), grey history trace, red current dot
- [x] CLI accepts `--port`, `--speed`, and `--window` arguments with documented defaults
- [x] Auto-detects port via `list_serial_ports()` when `--port` is omitted
- [x] Graceful shutdown on Ctrl-C: sends STREAM 0 + STOP before closing
- [x] `matplotlib>=3.8` added to the `calibrate` dependency group in `pyproject.toml`
- [x] `tests/bench` added to `norecursedirs` in `pyproject.toml` so pytest ignores it
- [x] No matplotlib import anywhere in `host/robot_radio/`
- [x] Full test suite (1042 tests) passes unchanged

## Testing

- **Existing tests to run**: `uv run --with pytest python -m pytest -q`
- **New tests to write**: None — this is a visual bench tool; hardware-interactive by design
- **Verification command**: `uv run --with pytest python -m pytest -q`
