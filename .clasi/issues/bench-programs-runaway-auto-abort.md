---
status: pending
---

# Bench programs: short, self-terminating runs with runaway/stall auto-abort

## Context

From the frustration forensics: a recurring trigger was a bench program that ran for
minutes while the robot misbehaved — full-tilt with no encoder motion, or driving off
course — and did not stop itself ("your program is supposed to detect problems… run
it for a little bit and stop"). The robot is on a stand or a small field; an
unsupervised multi-minute run is never acceptable and forces the operator to lunge
for the power switch.

## Goal

A small shared safety wrapper for `tests/bench/` and `tests/dev/` drive programs:

1. **Bounded duration by default** — every run has a short wall-clock cap; long runs
   are opt-in and chunked.
2. **Runaway detection → immediate `X` + abort:** full-tilt-with-no-encoder-motion,
   no-progress-toward-target for N seconds, frozen/zero encoders while commanding
   motion, or telemetry/`seq` silence beyond a grace window.
3. **Always stop on exit** — finally-block sends `X` (and clears any stream) on
   normal end, exception, or Ctrl-C, so interrupting the host program never leaves
   the robot driving.
4. Preflight liveness/tag check before any motion (reuses the existing
   robot-liveness-preflight pattern).

## Acceptance

- A drive program with an induced runaway (e.g. frozen encoders) sends `X` and exits
  within the detection window instead of running on; interrupting any bench program
  mid-drive leaves the robot stopped.

## Source
Frustration forensics in `docs/code_review/2026-06-11-wild-spin-and-cursing-forensics.md`
§2 (bucket #3) and §4. Complements the firmware-side bounds (D4/D5) — this is the
host-side discipline. Relates to memories `robot-liveness-preflight`,
`dont-defer-doable-hardware-actions`.
