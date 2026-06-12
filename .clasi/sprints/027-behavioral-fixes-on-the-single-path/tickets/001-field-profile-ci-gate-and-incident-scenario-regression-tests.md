---
id: '001'
title: Field-profile CI gate and incident scenario regression tests
status: open
use-cases:
  - SUC-001
  - SUC-002
  - SUC-003
  - SUC-004
  - SUC-005
depends-on: []
github-issue: ''
issue: field-profile-test-harness-and-ci.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 027-001: Field-profile CI gate and incident scenario regression tests

## Description

The exact-profile sim (OTOS fusion off, slip 0) validates a system that does
not exist on the field. A PR that passes only the exact profile is not done.

This ticket:
1. Adds a `sim_field_profile` fixture to `host_tests/conftest.py` that tests
   can use to run in the field profile (turn slip 0.26, OTOS fusion ON).
2. Adds `host_tests/test_incident_scenarios.py` with the four §4 scenario
   regression tests from the sim2real review.

The field-profile fixture `set_field_profile()` already exists in
`host_tests/firmware.py` and is used by `test_goto_bounds.py` and
`test_cancel_on_begin.py`. This ticket makes it available as a shared fixture
and adds the named scenario tests.

## Acceptance Criteria

- [ ] `host_tests/conftest.py` exposes a `sim_field_profile` fixture: creates
      a `Sim`, applies `SET sTimeout=60000`, then calls
      `s.set_field_profile(slip_turn_extra=0.26, fuse_otos=True)`.
- [ ] `host_tests/test_incident_scenarios.py` exists with four test functions:
  - `test_scenario_g_into_boards` — G to a target requiring PURSUE in the
    field profile; orbit count must be < 1.5 revolutions before arriving or
    timing out. (Regression guard for D8; mark `xfail(strict=False)` if it
    fails before 027-004 lands.)
  - `test_scenario_turn_under_rotate` — TURN 9000 in field profile; OTOS
    heading at completion must be ≥ 85° (regression guard for 024 heading
    fusion; should pass against current code).
  - `test_scenario_keepalive_kills_turn` — TURN 9000 + mid-flight `VW 0 0`
    keepalive injection; TURN must complete at commanded heading. Mark
    `@pytest.mark.xfail(strict=True)` until 027-003 (D6) lands; then remove.
  - `test_scenario_spin_on_placement` — OTOS pose frozen at (0,0,0) mid-PRE_ROTATE
    in field profile; command must exit via TIME net, not spin forever. Should
    pass against current code (D5 already bounds it).
- [ ] `test_d6_cannot_stomp_turn` from 026-003 confirmed present in
      `host_tests/test_vw_converters.py` and still `xfail` (promoted to
      passing in 027-003).
- [ ] All existing `host_tests/` tests pass.

## Implementation Plan

### Approach

Add `sim_field_profile` fixture to `host_tests/conftest.py` (5 lines, mirrors
the `sim` fixture but applies `set_field_profile` after creation).

Create `host_tests/test_incident_scenarios.py`. Each test uses `sim` or
`sim_field_profile` fixture as appropriate. Test structure follows the
existing patterns in `test_goto_bounds.py` — tick loop with keepalives,
drain EVTs, assert terminal EVT content.

For `test_scenario_turn_under_rotate`: after TURN completes, read OTOS heading
via `sim.send_command("GET otosH")` and assert the returned value ≥ 85° in
degrees. (The `GET` command returns the current HardwareState field.)

For `test_scenario_keepalive_kills_turn`: start TURN 9000, tick 500 ms, inject
`VW 0 0` (the stomping keepalive), continue ticking to completion, assert
`EVT done TURN` and that the OTOS heading at completion is ≥ 85°.

### Files to create/modify

- `host_tests/conftest.py` — add `sim_field_profile` fixture.
- `host_tests/test_incident_scenarios.py` — new file.

### Testing plan

```
uv run pytest host_tests/test_incident_scenarios.py -v
uv run pytest host_tests/ -v
```

Verify: two new tests pass (scenarios 1+2), two are xfail (scenarios 3+4 for
the not-yet-fixed defects). All existing tests still pass.

### Documentation updates

None. Test docstrings are the documentation.

## Notes

- If `test_scenario_g_into_boards` fails before D8 (027-004) lands, the xfail
  mark is appropriate — this is the test D8 is written to pass.
- Revalidate this ticket after sprint 026 merges if the sim's queue-wiring
  changes any tick semantics (unlikely — `set_field_profile` is independent
  of dispatch path). Low churn exposure.
