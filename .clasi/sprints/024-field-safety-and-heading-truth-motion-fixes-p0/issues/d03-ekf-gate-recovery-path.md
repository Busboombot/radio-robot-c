---
status: in-progress
sprint: '024'
tickets:
- 024-005
---

# D3 — EKF Mahalanobis gate has no recovery path (divergence trap)

## Context

`EKF::updatePosition` rejects any OTOS fix whose Mahalanobis distance exceeds the
χ² threshold (`d2 > 5.99 → ++_rejected; return;`). With `r_otos_xy = 50` and a
small P (steady state, or zeroed by `setPose`), innovations ≳ 17 mm are rejected.
Once heading drift (D1/D2) makes dead-reckoned position diverge past the gate,
**every** subsequent OTOS fix is rejected and the filter free-runs on encoders
forever — "confidently wrong, forever." `_rejected` is counted but never
telemetered, alarmed, or acted on. `EKF::setPose()` zeroing P entirely (false
perfect certainty) makes post-reset re-acquisition slower and the gate tighter.

## Fix (improvement-plan P0.4)

1. In `updatePosition` (and the new `updateHeading` from D1): count **consecutive**
   rejections; after N = 10 consecutive, either inflate S (scale R by ~10× for that
   update) or accept unconditionally once, then reset the streak. Converts
   "permanently lost" into "recovers within ~1 s".
2. Telemeter `ekf_rej` (cumulative reject count) in the TLM frame so divergence is
   visible from the host before it becomes a robot in the boards.
3. Pairs with D1.4: `setPose()` sets a sane P prior instead of zero.

## Acceptance

- **Sim:** teleport the mock-OTOS pose 200 mm mid-run (fusion on) → fused pose
  converges to the new OTOS truth in < 2 s instead of free-running forever.
- **Host:** `ekf_rej` appears in TLM and rises during induced divergence, falls
  after recovery.

## Source
Defect **D3** in the 2026-06-11 sim2real review; fix P0.4. Pairs with D1.
