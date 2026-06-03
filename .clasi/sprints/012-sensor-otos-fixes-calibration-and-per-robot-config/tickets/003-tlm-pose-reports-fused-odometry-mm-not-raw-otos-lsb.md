---
id: '003'
title: TLM pose reports fused odometry (mm), not raw OTOS LSB
status: done
use-cases:
- SUC-001
- SUC-004
- SUC-005
depends-on: []
github-issue: ''
issue: ''
completes_issue: false
---

# TLM pose reports fused odometry (mm), not raw OTOS LSB

## Description

The TLM `pose=` field currently reports raw OTOS LSB when OTOS is present
(Robot.cpp lines ~177-185), producing values ~5x too large (1 LSB = 0.305 mm,
so 1000 mm real distance = ~3279 LSB). The fused odometry (`_odo.getPose()`)
is already maintained in mm by `DriveController::correct()`; TLM should always
report from there.

The `OP` command already returns raw OTOS LSB for cross-check; that is retained
and clearly labeled as raw.

No dependencies — this is a self-contained Robot.cpp change.

## Files to Modify

- **`source/robot/Robot.cpp`** — tick() method, pose block (~lines 174-186):

  Replace the current branching logic:
  ```cpp
  // CURRENT (broken): reports raw OTOS LSB when OTOS present
  if (_otosPresent) {
      int16_t rx = 0, ry = 0, rh = 0;
      _otos.getPositionRaw(rx, ry, rh);
      pose_x = (int32_t)rx;
      pose_y = (int32_t)ry;
      pose_h = (int32_t)rh;
  } else {
      _odo.getPose(pose_x, pose_y, pose_h);
  }
  ```

  With the always-fused path:
  ```cpp
  // FIXED: always report fused odometry in mm/centidegrees
  _odo.getPose(pose_x, pose_y, pose_h);
  ```

  The `_otosPresent` check is removed from this block entirely. The OTOS `OP`
  command handler is unchanged (still calls `getPositionRaw` for raw LSB).

- **`tests/test_tlm_stream.py`** — update any assertions that expect LSB-scale
  pose values to expect mm-scale values.
- **`tests/test_otos_fusion.py`** — update pose-scale assertions similarly.

## Approach

1. Make the one-line change in Robot.cpp (replace the branching pose block).
2. Update test files that assert on pose scale.
3. Clean build. Reflash to robot enum 2.
4. Drive `D 200 200 1000` (1 m). Observe TLM: `pose=` x should be approximately
   1000 (not ~3279). `OP` should still return raw LSB.

## Acceptance Criteria

- [x] After driving 1 m forward, `pose=` x in TLM is approximately 1000 mm (not ~3279 LSB). (bench deferred to T11)
- [x] `OP` command still returns raw OTOS LSB values (unchanged behavior); reply verb changed to `rawpos` and body includes `(raw LSB)` label for clarity.
- [x] `test_tlm_stream.py` and `test_otos_fusion.py` pass with updated mm-scale assertions (76 passed).
- [x] Clean build (`mbdeploy build --clean`) succeeds. RAM: 120768 B / 122816 B (98.33%).
- [ ] (Bench deferred to T11) `pose=` x tracks tape-measure ground truth within a few percent.

## Testing

- **Existing tests to update**: `tests/test_tlm_stream.py`, `tests/test_otos_fusion.py`
- **Verification command**: `mbdeploy build --clean && uv run pytest tests/test_tlm_stream.py tests/test_otos_fusion.py`
