# tests/ — layout & conventions

Keep the **root of `tests/` clean** — no loose scripts. Every test or script
lives in a subdirectory chosen by purpose:

- `dev/` — **development, exploratory, and one-off scripts.** Anything you
  create ad-hoc to probe behavior, a throwaway, or an older/superseded test goes
  HERE — not in the root.
- `bench/` — hardware bench scripts and their shared helpers (e.g.
  `bench_safety.py`).
- `calibrate/` — calibration routines.
- `diagnostics/` — diagnostic / self-check tools.
- `playfield_tour/` — the real-robot, camera-localized playfield tour.

## RULE

Do **not** add files to the root of `tests/`. One-off / development tests go in
`dev/`. If a script is better classified as a smoke test, system test,
calibration, or diagnostic, put it in the matching subdirectory (create one if
none fits). The root stays empty of files.
