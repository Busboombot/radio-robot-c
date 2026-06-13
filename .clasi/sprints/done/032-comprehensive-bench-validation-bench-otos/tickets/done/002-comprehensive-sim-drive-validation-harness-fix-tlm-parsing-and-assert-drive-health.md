---
id: '002'
title: "Comprehensive sim drive-validation harness \u2014 fix TLM parsing and assert\
  \ drive health"
status: done
use-cases:
- SUC-004
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Comprehensive sim drive-validation harness — fix TLM parsing and assert drive health

## Description

The sim harness at `host_tests/test_zz_comprehensive_bench_validation.py` drives the
firmware simulator through turns/square/velocity profiles and checks for failure
signatures. It is currently broken due to wrong TLM unit assumptions:

- `pose=x,y,h` — heading `h` is in **centidegrees** (integer), but the harness passes
  it directly to `math.degrees()` as if it were radians, producing headings in the
  millions of degrees.
- `twist=v,omega` — omega is in **mrad/s** (integer), but assertions compare it against
  `12.0` (rad/s) without converting, so the check is off by 1000×.

The `_frames()` parser must be fixed to convert these values into natural units before
they reach `_analyze()` and the assertion checks.

Exact firmware encoding (confirmed from `source/robot/Robot.cpp` `buildTlmFrame`):
- `pose` heading: `(int)(heading_rad * 5729.5779513f)` — i.e., centidegrees (18000/π cdeg/rad)
- `twist` omega: `(int)(fusedOmega * 1000.0f)` — i.e., mrad/s

The host `TLMFrame` dataclass in `host/robot_radio/robot/protocol.py` already documents
these units correctly; the harness just wasn't converting.

## Acceptance Criteria

- [ ] `uv run --with pytest python -m pytest host_tests/test_zz_comprehensive_bench_validation.py -s` passes (exit 0)
- [ ] `_frames()` converts `pose` heading from centidegrees to degrees (divide by 100) before storing in `f["h"]`
- [ ] `_frames()` converts `twist` omega from mrad/s to rad/s (divide by 1000) before storing in `f["omega"]`
- [ ] `_analyze()` `heading_total_change_deg` and `heading_final_deg` report sensible values (e.g., ~360 deg for 4×90 turns, not millions)
- [ ] `omega_max` assertion threshold of `12.0 rad/s` is correct and meaningful after the conversion fix
- [ ] All existing assertion thresholds in the test remain unchanged (they were written for correct units; only the parsing was wrong)
- [ ] Printed report shows meaningful metrics across all sequences

## Implementation Plan

### Files to modify

`host_tests/test_zz_comprehensive_bench_validation.py` — fix `_frames()` only.

### Changes

In `_frames()`, the pose and twist parsing blocks currently do:

```python
f["x"], f["y"], f["h"] = float(p[0]), float(p[1]), float(p[2])
```
and:
```python
f["v"], f["omega"] = float(t[0]), float(t[1])
```

Fix to:

```python
# pose heading: firmware emits centidegrees (18000/π cdeg/rad)
f["x"], f["y"] = float(p[0]), float(p[1])
f["h"] = float(p[2]) / 100.0          # centidegrees → degrees
```

and:

```python
# twist: v is mm/s (integer), omega is mrad/s → convert to rad/s
f["v"] = float(t[0])
f["omega"] = float(t[1]) / 1000.0     # mrad/s → rad/s
```

### Verify no other unit assumptions are broken

After the fix, scan `_analyze()` and all assertion blocks for any remaining
direct use of `f["h"]` or `f["omega"]` that might assume different units.
The `heading_total_change_deg` metric already calls `math.degrees()` on
summed heading deltas — after the fix `f["h"]` will be in degrees, so that
`math.degrees()` call will be wrong (degrees-of-degrees). Remove the
`math.degrees()` call from the heading accumulator in `_analyze()` since
the values are already in degrees.

Specifically in `_analyze()`:

```python
# BEFORE (broken — applies degrees() to centidegree-sourced values):
metrics["heading_total_change_deg"] = math.degrees(
    sum(abs(hs[i] - hs[i - 1]) for i in range(1, len(hs)))
)
metrics["heading_final_deg"] = math.degrees(hs[-1])

# AFTER (correct — h values are already degrees after _frames() fix):
metrics["heading_total_change_deg"] = sum(abs(hs[i] - hs[i - 1]) for i in range(1, len(hs)))
metrics["heading_final_deg"] = hs[-1]
```

### No other files to change

Do not modify `host/robot_radio/robot/protocol.py` or any firmware source.
Do not change assertion thresholds — they were already written for correct units.

## Testing

- **Existing tests to run**: `uv run --with pytest python -m pytest host_tests/test_zz_comprehensive_bench_validation.py -s`
- **New tests to write**: None — this ticket fixes the existing harness
- **Verification command**: `uv run --with pytest python -m pytest host_tests/test_zz_comprehensive_bench_validation.py -s`

Expected printed report after fix: turns sequence shows `heading_total_change_deg` ≈ 360,
omega values ≈ 0–3 rad/s (not thousands), no assertion failures.
