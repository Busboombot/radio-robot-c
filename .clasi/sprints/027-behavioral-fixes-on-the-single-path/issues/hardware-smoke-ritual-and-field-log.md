---
status: in-progress
sprint: '027'
tickets:
- 027-005
---

# Hardware smoke ritual + field log (run before/after every flash)

## Context

Field regressions repeatedly went unnoticed until the robot was already misbehaving
on the playfield, and the same failures recurred across flashes because there was no
fast, fixed acceptance check. A scripted 5-minute ritual catches the known failure
modes immediately and creates a dated record tied to the firmware SHA.

## Goal

A scripted bench check in `tests/bench/` (≈5 min) that runs and prints pass/fail for:

1. **SAFE query** — must report `on`.
2. **`TURN 9000` × 4** — orientation closure (should return to start within a few °;
   exercises D1/D2).
3. **G square** — return-to-start error < 50 mm (exercises D5/D8 + heading truth).
4. **Lift test** — lift mid-motion → expect `EVT otos lost`, no spin on placement
   (exercises D9).
5. **Stream drop-rate** print from TLM `seq` gaps (exercises D10).

Run it **before and after every firmware flash**; append results to
`docs/knowledge/field-log.md` with date + git SHA. Also captures the "measure the
heading convention once and pin it" discipline (the convention used by the smoke
test is the canonical one; bench programs must not re-guess it).

## Acceptance

- The script exists, runs end-to-end against the robot, prints a clear pass/fail per
  check, and writes a dated SHA-stamped entry to the field log.

## Source
Improvement-plan **P3.2** in the 2026-06-11 review, plus the "measure conventions,
don't guess" and "trust hardware priors" process patterns from
`docs/code_review/2026-06-11-wild-spin-and-cursing-forensics.md` §4. Relates to
memory `camera-heading-convention`.
