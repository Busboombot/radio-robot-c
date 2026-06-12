---
id: '003'
title: Add D11 double-OK gate test in sim
status: open
use-cases:
  - SUC-004
depends-on:
  - "026-001"
  - "026-002"
github-issue: ''
issue: d11-single-ok-per-command.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 026-003: Add D11 double-OK gate test in sim

## Description

D11 is the defect where a converter command (G, TURN, T, D, R, RT, S) produces
two `OK` replies on the hardware path: one from the converter handler and one
from `handleVW`. In sim (no queue wired) only one reply was emitted, making the
defect invisible to the test suite.

Tickets 001 and 002 fix the structural root causes: the queue is now wired in
sim, and `handleVW`'s stop-param branches no longer emit a reply (the converter
already did). This ticket adds the regression test that gates the D11 fix and
would have caught the original defect.

### What to add

In `host/tests/test_protocol_v2.py`, add a new parameterized test
`test_single_ok_per_converter_command` that:

1. For each converter command — `S 200 200`, `T 500 200 200`, `D 300 200 200`,
   `G 400 300 200`, `TURN 9000`, `RT 9000`, `R 200 200` — sends the command
   with a unique `#id` correlation token via the sim.
2. Collects both the synchronous reply from `sim_command()` and async EVTs from
   `sim_get_async_evts()` over a sufficient number of ticks (enough for the
   command's completion EVT to fire, e.g., 500 ticks at 10 ms = 5 s simulated).
3. Counts the number of reply lines that start with `OK` and contain `#<id>`.
4. Asserts the count is exactly 1.

Also add `test_direct_vw_replies_once`: send `VW 200 0 #99` directly (no
stop params) and assert exactly one `OK` line with `#99`.

The test must use the sim's `SimConnection` or the existing fixture pattern in
`host/tests/conftest.py` that provides a `sim` fixture. If `test_protocol_v2.py`
does not currently use the sim, adapt the test to use the same fixture pattern
as `host_tests/*.py` (the C-ABI sim) — check which fixture provides `sim` and
use that.

### Why this test is the acceptance gate

Before tickets 001+002: the test would fail for all converter commands (two OKs).
After tickets 001+002: it must pass. This test is the mechanical proof that:
(a) the queue is wired (otherwise the converter takes the fallback path and there
would be no second reply, so the test would trivially pass without actually
verifying anything meaningful), AND (b) `handleVW` does not emit a second reply
on the converter push path.

### Also add: `test_d6_cannot_stomp_turn` (diagnostic only, not a fix)

Add a test that starts a `TURN 9000` command, injects a `VW 0 0` keepalive
mid-turn, and asserts that the TURN still completes at the correct heading
(or at least that the `EVT done TURN` fires). This test will FAIL in sprint 026
(D6 is not fixed here), but its presence documents the defect and gives sprint
027 a ready-to-fix regression test. Mark it with `@pytest.mark.xfail` so it
does not block CI.

## Acceptance Criteria

- [ ] `test_single_ok_per_converter_command` exists in `host/tests/test_protocol_v2.py`
  (or a new `test_d11_gate.py` in `host_tests/`).
- [ ] The test sends each of S, T, D, G, TURN, RT, R with a corr-id and asserts
  exactly one `OK #<id>` reply is received across the sync reply and all async
  EVT accumulation.
- [ ] `test_direct_vw_replies_once` passes: direct `VW` also gets exactly one OK.
- [ ] `test_d6_cannot_stomp_turn` exists and is marked `@pytest.mark.xfail`
  (documents the D6 defect, expected to fail until sprint 027).
- [ ] All new tests pass (`uv run pytest host/tests/ -v` or equivalent path).
- [ ] All existing tests still pass.

## Testing

- **Existing tests to run**: `uv run pytest host_tests/ host/tests/ -v`
- **New tests to write**: `test_single_ok_per_converter_command`,
  `test_direct_vw_replies_once`, `test_d6_cannot_stomp_turn` (xfail).
- **Verification command**: `uv run pytest host_tests/ host/tests/ -v`

## Implementation Notes

- If the host test suite (`host/tests/`) does not currently have access to the
  `SimHandle` C-ABI sim, check `host/tests/conftest.py` for the fixture. If the
  sim fixture is only in `host_tests/conftest.py`, either add a bridge or place
  the new test in `host_tests/test_d11_gate.py` instead of `host/tests/`.
- The existing `test_protocol_v2.py` tests the Python-layer protocol parsing.
  If it does not use the C-ABI sim, a new `host_tests/test_d11_gate.py` is the
  better location for the sim-backed assertion. The D11 issue references
  `host/tests/test_protocol_v2.py` by name — add there if possible, but use the
  correct sim fixture regardless of file location.
- For async EVT collection: after sending the command, call `sim_tick()` in a
  loop for enough ticks to cover the command's maximum duration (TURN 9000 at
  default yawRateMax ≈ 60 deg/s → ~1.5 s → 150 ticks), then call
  `sim_get_async_evts()`. Count `OK ... #<id>` lines across both the sync reply
  and the accumulated EVTs.
