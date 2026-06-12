# Field Log

Hardware smoke-ritual results logged here after each sprint acceptance run.
One entry per run, with date, git SHA, step results, and anomaly notes.

---

## Sprint 026 — one-dispatch-path

**Date:** 2026-06-11
**Git SHA:** deb3c29156c7139a3dd9d0fd35544d4d5fd07254
**Ritual script:** `tests/bench/smoke_ritual.py`
**Flash command:** `mbdeploy deploy robot --clean`

| Step | Name               | Result                                | Notes |
|------|--------------------|---------------------------------------|-------|
| 1    | Safety check       | PENDING — stakeholder field test      |       |
| 2    | TURN ×4 closure    | PENDING — stakeholder field test      |       |
| 3    | G square           | PENDING — stakeholder field test      |       |
| 4    | No double-OK       | PENDING — stakeholder field test      |       |
| 5    | Stream aliveness   | PENDING — stakeholder field test      |       |

**Overall:** PENDING — reserved for stakeholder field test.

To run the ritual:

```
mbdeploy deploy robot --clean   # flash first
uv run python tests/bench/smoke_ritual.py --port /dev/cu.usbmodem<N>
```

Ticket 026-004 will be updated to PASS once all five steps are confirmed on
the real robot. See ticket for acceptance criteria.
