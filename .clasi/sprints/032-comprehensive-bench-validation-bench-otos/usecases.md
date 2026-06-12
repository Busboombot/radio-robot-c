---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 032 Use Cases

## SUC-001: Turn closure validation on bench hardware

- **Actor**: Team lead (engineer)
- **Preconditions**: Robot powered on, firmware v0.20260612.17 flashed, relay connected, Bench OTOS enabled via `DBG OTOS BENCH 1`, STREAM active
- **Main Flow**:
  1. Zero encoders and pose
  2. Issue four TURN 9000 commands (4 × 90.00 deg CCW)
  3. Collect TLM for each turn
  4. Check heading closure (~360 deg total), omega bounded, clean stop after each turn
- **Postconditions**: Raw TLM log captured; heading closure measured; any runaway spin or bad-stop finding recorded
- **Acceptance Criteria**:
  - [ ] Four turns complete without runaway spin (total heading change ≤ 720 deg)
  - [ ] omega bounded (no spike beyond yaw rate cap)
  - [ ] Clean stop after each turn (residual |v| ≤ 30 mm/s)

## SUC-002: Square drive validation on bench hardware

- **Actor**: Team lead (engineer)
- **Preconditions**: Same as SUC-001
- **Main Flow**:
  1. Zero encoders and pose
  2. Drive 4 × (D 300 mm + TURN 90 deg) to trace a square
  3. Collect TLM for each segment
  4. Check for velocity jumps (bad start/mid-motion), EKF rejection count, clean stops
- **Postconditions**: Raw TLM log captured; velocity jump and EKF health data available
- **Acceptance Criteria**:
  - [ ] Tick-to-tick |dv| ≤ 120 mm/s throughout (no bad velocity jumps)
  - [ ] ekf_rej not climbing unreasonably (≤ 20 count over the square)
  - [ ] Clean stops after each segment

## SUC-003: Velocity profile validation on bench hardware

- **Actor**: Team lead (engineer)
- **Preconditions**: Same as SUC-001
- **Main Flow**:
  1. For each speed (slow 150 mm/s, medium 300 mm/s, fast 500 mm/s) issue D command + collect TLM
  2. Issue T (timed velocity) command at 300 mm/s + collect TLM
  3. Check for bad starts (instant full-speed jump), velocity jumps, clean stops, no spurious heading drift on straight runs
- **Postconditions**: TLM logs captured for 4 drive profiles; start/stop/jump metrics recorded
- **Acceptance Criteria**:
  - [ ] No instant-max-speed start (v at tick 1 ≤ 60% of peak on any run)
  - [ ] Tick-to-tick |dv| ≤ 120 mm/s
  - [ ] Clean stop (residual |v| ≤ 30 mm/s)
  - [ ] Straight drive heading drift ≤ 25 deg

## SUC-004: Sim harness correctly validates drive health in CI

- **Actor**: CI / developer
- **Preconditions**: `host_tests/test_zz_comprehensive_bench_validation.py` exists in repo
- **Main Flow**:
  1. Developer runs `uv run --with pytest python -m pytest host_tests/test_zz_comprehensive_bench_validation.py -s`
  2. Harness drives sim through turns/square/velocity profiles (field profile, OTOS fusion ON)
  3. TLM pose heading (centidegrees) and twist omega (mrad/s) are correctly converted before assertions
  4. Printed report shows meaningful metrics; pytest passes
- **Postconditions**: Test passes with no absurd metric values
- **Acceptance Criteria**:
  - [ ] `pytest` passes (exit 0)
  - [ ] Heading metrics in report are in degrees (not millions of degrees)
  - [ ] omega/v assertions use correct units (mrad/s → rad/s conversion applied)
  - [ ] No TLM parsing regressions
