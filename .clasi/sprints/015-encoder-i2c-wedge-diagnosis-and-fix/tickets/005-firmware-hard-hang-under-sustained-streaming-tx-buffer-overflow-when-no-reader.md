---
id: "005"
title: "Firmware hard-hang under sustained streaming — TX-buffer overflow when no reader"
status: open
use-cases: []
depends-on: []
github-issue: ""
issue: ""
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware hard-hang under sustained streaming — TX-buffer overflow when no reader

## Description

During sprint-015 wedge debugging, the firmware repeatedly went **hard-unresponsive**
(no `PING`/pong, no boot banner) after many back-to-back streaming runs, requiring a
**full power cycle** to recover — a DTR-pulse micro:bit reset did NOT reliably recover it.
This is **distinct from** the velocity_chart `safety_stop` freeze (fixed in 015-004, which
is host-side) and from the encoder wedge (host-side static, fixed in sprint 014).

**Leading hypothesis:** the firmware's serial **TX buffer is 255 bytes** (`uint8_t`
size cap in CODAL — see the sprint-014 note where `setTxBufferSize(1024)` wrapped to 0).
When `STREAM` is left enabled and the host stops draining the port (e.g. during a tool's
reconnect / PING-retry window, or a crashed reader), the firmware keeps emitting TLM with
no consumer, the TX buffer fills, and a blocking/faulting write in the cooperative loop
**stalls the whole loop** (or faults), hanging the firmware. The encoder-read and watchdog
tasks stop running; nothing recovers it short of power.

This is why the robot needed repeated power cycles throughout sprint-015 debugging.

## Acceptance Criteria

- [ ] Confirm the mechanism with instrumentation: reproduce by enabling `STREAM` then
      ceasing to read the port; capture whether the loop stalls in the serial write
      (e.g. `DBG LOOP` stops advancing, or a fault) and whether the TX path blocks.
- [ ] Firmware no longer hard-hangs when the host stops reading mid-stream: TLM emission
      must be **non-blocking / drop-on-full** (never block the cooperative loop on a full
      TX buffer), OR streaming auto-disables after N missed reads / a TX-stall is detected.
- [ ] After the host stops reading for ≥30 s with `STREAM` on, the robot still answers
      `PING` immediately when the host resumes — no power cycle required.
- [ ] The cooperative control loop (encoder→PID→PWM) keeps running at its normal rate
      even when the serial consumer is absent.

## Testing

- **Existing tests to run**: `uv run --with pytest python -m pytest` (host suite, 1042).
- **New tests to write**: a host-side repro (extend `tests/bench/velchart_repro.py` or
  add a `tests/bench/tx_starve.py`) that enables `STREAM`, stops reading for a while, then
  re-reads and asserts the robot still responds to `PING`.
- **Verification command**: bench — `uv run python tests/bench/<repro>.py`; confirm no
  power cycle needed to recover.
- **Firmware files likely involved**: `source/hal/SerialPort.cpp` (TX path / buffer size),
  `source/robot/Robot.cpp` (`telemetryEmit`), `source/control/LoopScheduler.cpp`
  (telemetry-emit task), `source/types/Config.h` (`tlmPeriodMs`).
