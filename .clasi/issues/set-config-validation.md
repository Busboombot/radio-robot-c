---
status: pending
---

# SET can write invalid live control config (no parse/range/invariant checks)

## Context

Unique finding from the 2026-06-08 review — **re-verified still present** in current
code (it was only relocated). `handleSet` now lives at
[source/robot/ConfigRegistry.cpp:232](source/robot/ConfigRegistry.cpp#L232) and
parses with raw `atof()`/`atoi()` ([:288](source/robot/ConfigRegistry.cpp#L288),
[:293](source/robot/ConfigRegistry.cpp#L293), [:298](source/robot/ConfigRegistry.cpp#L298))
writing directly into `RobotConfig` with **no parse-failure detection, no range
checks, no cross-field invariant checks, and no atomic all-or-nothing application**.

Risk: a malformed or out-of-range `SET` can break the active control model live —
e.g. `tw=0` divides by zero in odometry/kinematics; `vWheelMax < steerHeadroom`
makes the saturation ceiling negative; a negative `ctrlPeriod` cast to `uint32_t`
in the scheduler. Direct drive commands have range checks; the live config path
bypasses equivalent constraints. This is also a foot-gun the agent has used heavily
during tuning (`SET alphaYaw=…`, `yawRateMax=…`), so bad values land silently.

## Fix (2026-06-08 finding, minimal correction)

1. Typed parsing with end-pointer validation (reject non-numeric / trailing-garbage).
2. Central `RobotConfig` validation: per-field ranges + cross-field invariants
   (`tw > 0`, `vWheelMax > steerHeadroom`, `ctrlPeriod > 0`, `rotationalSlip ∈ [0.5,1]`,
   etc.).
3. Apply multi-key `SET` only after the whole candidate config validates; reply with
   the offending key on failure. Keep dependent controller updates tied to a
   successful commit.

## Acceptance

- `SET tw=0` (and other invariant violations) is rejected with a clear error and the
  live config is unchanged; a valid multi-key `SET` applies atomically; a partially
  invalid multi-key `SET` applies nothing.

## Source
"High: `SET` can write invalid live control configuration" in
`docs/code_review/2026-06-11-Fable-s2p-review/source-code-review-findings.md`
(2026-06-08). Not covered by the June-11 D1–D12 inventory or the wild-spin forensic —
re-mapped from `CommandProcessor.cpp` to its current home in `ConfigRegistry.cpp`.
