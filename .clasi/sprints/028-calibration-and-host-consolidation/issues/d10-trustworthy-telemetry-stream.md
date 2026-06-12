---
status: in-progress
sprint: 028
tickets:
- 028-005
---

# D10 — Telemetry stream the host can trust (seq, idle rate, channel binding)

## Context

The TLM stream design fights its consumers:

- `tlmPeriodMs = 0` by default (off until `STREAM <ms>`), and `telemetryEmit()` goes
  **silent by design** whenever IDLE > 400 ms — so an agent watching the stream sees
  it "die" at every stop and starts debugging the serial layer.
- TLM frames use async drop-on-full `send()` into CODAL's ~250-byte TX buffer; a
  100+ byte frame at the 50 Hz clamp floor over the relay drops frames under
  backpressure — with **no sequence number**, so the host can't detect loss.
- `activeTlmFn` retargets to whichever channel sent the **last** command: a single
  radio command silently steals the serial telemetry stream.
- `telemetryEmit` mutates `config.tlmPeriodMs` (the clamp) — a config write hidden in
  the telemetry path.

## Fix (improvement-plan P2.2)

1. Add `seq=<n>` (uint16 wrap) to the TLM frame; surface it in `TLMFrame` parsing so
   the host can measure drop rate.
2. Replace idle-silence with a low idle rate: when IDLE > grace, emit at
   `max(tlmPeriodMs, 500)` instead of nothing. Document in `protocol-v2.md`.
3. Bind the TLM channel explicitly: `STREAM` captures its reply channel as the
   stream sink; commands on other channels no longer steal it (`activeTlmFn` updates
   only on `STREAM`).
4. Move the `tlmPeriodMs < 20` clamp out of `telemetryEmit` into the `STREAM`/SET
   handler; reject periods the TX buffer can't sustain at the current field set and
   reply `OK stream ms=<clamped>` so the host knows what it got.

## Acceptance

- Host-side drop-rate from seq gaps < 2% during a full G run over the relay; stream
  survives idle→drive→idle without the host reconnecting; a radio command does not
  kill the serial stream.

## Source
Defect **D10** in the 2026-06-11 sim2real review (+ scenario 4.5); fix P2.2.
Pairs with D11a (host side) and D11 (double-OK pollution).
