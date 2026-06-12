---
status: done
sprint: 028
tickets:
- 028-002
---

# A7 — One calibration package; calibration outputs must be consumed

## Context

Calibration logic exists in four places: `host/calibrate_angular.py` (718 lines),
`host/calibrate_linear.py` (555), `host/calibrate_verify.py`, and
`robot_radio/io/calibrate.py` (1101) — with literal duplicates (`_deep_merge`,
`mean_stdev`, `scale_to_int8`, the last re-duplicated again in cli.py).

Worse than the duplication: calibration **outputs are not reliably consumed**.
`rotationalSlip`, `turnScale`, `distScale` are calibrated, stored in
`tovez.json`, registered in `ConfigRegistry` — and read by **nothing** in
`source/` (defect D2; `rotationalSlip` is being wired in by P0.5, the others
remain dead). A calibration pipeline whose values silently go nowhere consumes
bench time and produces false confidence; this already cost real field-debugging
sessions.

## Fix

1. Consolidate into one calibration package under `robot_radio/` (suggested:
   `robot_radio/calibration/`), with the three top-level scripts reduced to thin
   entry points.
2. De-duplicate the helpers; single implementation of config merge/save.
3. For every calibrated value: either wire it to a consumer in firmware or delete
   it from the pipeline and config. No third state.
4. Add the registry-vs-usage lint from A8 so a calibrated-but-unread key fails CI
   (would have caught D2 mechanically).

## Acceptance

- One calibration package; zero duplicated helpers; every key the pipeline writes
  is read somewhere in `source/` (lint-enforced) or removed.

## Priority suggestion

**Medium-high.** The consolidation itself is medium, but item 3 + the lint protect
the P0.5 calibration work currently in flight — land the lint early (it pairs with
A8 and is a small standalone task), let the package consolidation ride a later
sprint.

## Source
Finding **A7** in `docs/code_review/2026-06-11-architecture-modularity-review.md`;
defect D2 in the sim2real review.
