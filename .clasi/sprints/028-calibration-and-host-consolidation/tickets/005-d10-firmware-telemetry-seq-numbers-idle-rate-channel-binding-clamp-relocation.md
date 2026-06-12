---
id: '005'
title: "D10 firmware telemetry — seq numbers, idle rate, channel binding, clamp relocation"
status: open
use-cases:
  - SUC-003
  - SUC-005
depends-on:
  - "028-001"
github-issue: ''
issue: d10-trustworthy-telemetry-stream.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 028-005: D10 firmware telemetry — seq numbers, idle rate, channel binding, clamp relocation

## Description

The TLM stream design has four firmware-side problems that make the host unable
to trust or measure it:

1. **No sequence number**: frames drop under relay backpressure with no way for
   the host to detect loss.
2. **Idle silence**: when the robot stops (IDLE > 400 ms), the stream goes
   silent by design. The host cannot distinguish "robot idle" from "serial
   dropped."
3. **Channel theft**: `activeTlmFn` in `LoopScheduler` is updated to whichever
   channel sent the last command (serial or radio). A single radio command
   silently redirects the serial TLM stream to radio.
4. **Clamp in emit path**: `telemetryEmit` mutates `config.tlmPeriodMs` — a
   config write hidden inside a telemetry emit function.

The host-side foundation (reader thread, sprint 025) is in place. This ticket
adds the firmware-side fixes. If sprint 027 ticket 006 / 028-001 concluded
that the field-024 SNAP/STREAM discrepancy requires D10 changes, this ticket
also closes that lead.

## Acceptance Criteria

### Sequence number

- [ ] `buildTlmFrame` emits `seq=<n>` as the first field after `t=` and `mode=`.
      `n` is a `uint16_t` counter on `Robot` (`_tlmSeq`) incremented on every
      call to `buildTlmFrame` (both STREAM and SNAP paths share the same
      counter).
- [ ] `TLMFrame` in `host/robot_radio/robot/protocol.py` gains
      `seq: int | None = None`. `parse_tlm()` populates it when present.
- [ ] A `tlm_drop_rate(frames: list[TLMFrame]) -> float` helper is added (in
      `protocol.py` or `robot_radio/sensors/tlm_health.py`). Returns the
      fraction of expected seq numbers absent. Returns 0.0 for fewer than 2
      frames or when all frames have `seq=None`.
- [ ] Drop rate < 2% verified over a 60 s drive with `STREAM 50` over the relay.

### Idle rate

- [ ] `telemetryEmit` no longer returns early when `stopped == true`. When
      stopped, emit continues at `max(config.tlmPeriodMs, 500)` ms interval.
- [ ] Stream survives idle-drive-idle (min 5 s each phase) without host reconnect
      or observable gap > 600 ms.

### Channel binding

- [ ] A `_tlmBoundFn` / `_tlmBoundCtx` pair is added to `Robot` (or stored
      in `LoopScheduler`). These are set only when a `STREAM` command arrives
      (in `handleStream`), not on every command arrival.
- [ ] The `activeTlmFn = serialReplyTlm` and `activeTlmFn = radioReply` lines
      in `runCommsIn` (LoopScheduler.cpp lines ~52–63) are replaced: after
      processing each command, `activeTlmFn` reads from `_tlmBoundFn` (the
      last STREAM-bound channel).
- [ ] A radio command arriving after STREAM was issued on serial does NOT
      redirect the TLM stream. Verified in sim: STREAM on serial, G on radio,
      TLM still emitted on serial path.

### Clamp relocation

- [ ] The `if (config.tlmPeriodMs < 20) config.tlmPeriodMs = 20;` line is
      removed from `telemetryEmit`.
- [ ] The clamp is applied in `handleStream` before writing
      `config.tlmPeriodMs`. The STREAM reply includes the clamped value:
      `OK stream period=<clamped>`.
- [ ] `telemetryEmit` no longer writes to `config`.

### Documentation

- [ ] `docs/protocol-v2.md` updated: `seq=<n>` field documented; idle-rate
      behavior documented; channel-binding behavior documented.

### Regression

- [ ] All existing tests pass:
      `python3 build.py && uv run --with pytest python -m pytest host_tests/ -v`

### field-024 SNAP lead (if deferred from 028-001)

- [ ] If 028-001 concluded the SNAP/STREAM discrepancy requires D10 changes:
      confirm that the shared `_tlmSeq` counter between SNAP and STREAM makes
      them consistent. Document closure in the field-024 issue.
- [ ] If 028-001 fixed the discrepancy without D10: this criterion is N/A.

## Implementation Plan

### Approach

**Firmware changes:**

1. Add `uint16_t _tlmSeq = 0;` to `Robot` (in `Robot.h` private section).
2. In `buildTlmFrame` (Robot.cpp ~line 306): after the `TLM t=... mode=...`
   snprintf, add `n = snprintf(buf+pos, rem, " seq=%u", (unsigned)_tlmSeq++);`.
3. In `telemetryEmit` (Robot.cpp ~line 372): replace the
   `bool stopped` + early-return block so that when stopped the effective
   period is `max(config.tlmPeriodMs, 500)` instead of suppression:
   ```cpp
   uint32_t effectivePeriod = stopped
       ? (uint32_t)(config.tlmPeriodMs > 500 ? config.tlmPeriodMs : 500)
       : (uint32_t)config.tlmPeriodMs;
   bool periodic = (config.tlmPeriodMs > 0)
                   && ((now_ms - _lastTlmMs) >= effectivePeriod);
   if (!periodic) return;
   ```
   Remove the `config.tlmPeriodMs < 20` clamp line (Robot.cpp ~line 382).
4. In `handleStream` (Robot.cpp ~line 811): before writing
   `robot->config.tlmPeriodMs = ms;`, add `if (ms < 20) ms = 20;`. Update the
   OK body to include the clamped value. Store the caller's `replyFn` and
   `replyCtx` as `robot->_tlmBoundFn` and `robot->_tlmBoundCtx`.
5. In `LoopScheduler::runCommsIn` (LoopScheduler.cpp ~lines 52–63): remove the
   `activeTlmFn = serialReplyTlm;` and `activeTlmFn = radioReply;` lines.
   Replace with `sched.activeTlmFn = sched._robot._tlmBoundFn;` etc. (or
   read from the Robot accessor). `activeTlmFn` is now sticky until STREAM
   changes it.

**Host changes:**

6. In `host/robot_radio/robot/protocol.py` `TLMFrame`: add
   `seq: int | None = None`.
7. In `parse_tlm`: add `if "seq" in kv: try: frame.seq = int(kv["seq"])`.
8. Add `tlm_drop_rate(frames)` helper function.

### Files to modify

- `source/robot/Robot.h` — add `_tlmSeq`, `_tlmBoundFn`, `_tlmBoundCtx`
- `source/robot/Robot.cpp` — `buildTlmFrame` (seq), `telemetryEmit` (idle
  rate, remove clamp), `handleStream` (clamp, channel binding)
- `source/control/LoopScheduler.cpp` — `activeTlmFn` binding change
- `host/robot_radio/robot/protocol.py` — `TLMFrame.seq`, `parse_tlm`,
  `tlm_drop_rate`
- `docs/protocol-v2.md` — seq, idle-rate, channel-binding

### Testing plan

```
python3 build.py
uv run --with pytest python -m pytest host_tests/ -v
```

New sim tests:
- Issue STREAM, run 50 ticks, collect TLM frames, assert `seq` increments
  monotonically.
- STREAM → transition to IDLE → tick 600 ms → assert TLM frame still arrives
  (idle-rate test).
- STREAM on serial, G command on radio, assert TLM still emitted on serial
  channel (channel-binding test).
- STREAM 10 → assert reply period=20 (clamp applied); STREAM 100 → reply
  period=100 (no clamp).

Drop-rate bench test: run `uv run python tests/bench/square_run.py` with
STREAM 50 active; collect frames for 60 s; assert
`tlm_drop_rate(frames) < 0.02`.

### Documentation updates

Update `docs/protocol-v2.md`:
- TLM frame format: add `seq=<n>` field (uint16 wrapping at 65535; absent on
  old firmware).
- Idle behavior: "stream continues at max(period, 500 ms) when IDLE."
- Channel binding: "TLM stream bound to the channel that last issued STREAM;
  commands on other channels do not redirect it."

## Notes

- Depends on 028-001 to know whether the SNAP/STREAM field-024 lead is closed
  here or was already closed by 028-001. The shared `_tlmSeq` counter is the
  natural resolution if D10 is required.
- The channel-binding change is a behavior change: sessions that use radio
  commands without first issuing STREAM on serial will no longer get serial
  TLM. Document this and note it is intentional.
- `_tlmBoundFn = nullptr` on init: TLM is suppressed until STREAM is issued
  — the same behavior as today (tlmPeriodMs starts at 0).
