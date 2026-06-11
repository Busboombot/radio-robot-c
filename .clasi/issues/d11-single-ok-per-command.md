---
status: pending
---

# D11 — One reply per command (kill the duplicate OK on the hardware path)

## Context

On hardware, a converter command produces **two** OK replies with the same corr-id:
the converter handler emits `OK goto x=.. #id`, then `handleVW` emits `OK vw x=.. #id`
for the same command. Sim (no queue wired) emits only one — so this asymmetry exists
*only* on real hardware. Any host logic that correlates by `#id` or counts replies
sees phantom responses, and the extra line pollutes the same stream the host parses
for TLM/EVT.

## Fix (improvement-plan P1.2)

- Pick one owner: the converter keeps the user-facing reply (`OK goto …`); `pushVW`
  marks the ParsedCommand `quiet=true` so `handleVW` skips its `replyOK` for
  converted commands. Direct `VW` commands still get `OK vw`.
- Update `docs/protocol-v2.md` accordingly.

## Acceptance

- `host/tests/test_protocol_v2.py`: exactly one OK per command on the queue path;
  direct VW still replies once.

## Source
Defect **D11** in the 2026-06-11 sim2real review; fix P1.2. Best verified after the
sim runs the real dispatch path (so the double-OK is reproducible in test).
