---
status: draft
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 024 Use Cases

## SUC-001: Drive to a playfield square without runaway spin

- **Actor**: Host program (rogo / square_run.py) issuing `G x y speed`
- **Preconditions**: Robot is placed on the playfield; pose is SI-aligned to the
  camera world frame; keepalive daemon is running; safety is ON.
- **Main Flow**:
  1. Host sends `G <tx> <ty> <speed>`; firmware enters PRE_ROTATE if bearing > 35°.
  2. Firmware spins in place under a supervised MotionCommand (HEADING + TIME stops).
  3. When bearing closes to within the gate, firmware transitions to PURSUE and drives
     toward the target under an overall TIME net.
  4. Firmware emits `EVT done G` on arrival or on TIME stop; host receives it.
- **Postconditions**: Robot rests near target or stops cleanly; no unbounded spin has
  occurred; no `EVT safety_stop` was emitted from a missing stop condition.
- **Acceptance Criteria**:
  - [ ] With heading frozen (mock) and keepalives flowing, a G to a 135° bearing target
        ends via the PRE_ROTATE TIME net and emits `EVT done G`.
  - [ ] A full tour (four playfield squares) never produces an unbounded spin.
  - [ ] G emits `EVT done G`, not `EVT safety_stop`, in all simulated failure modes.

---

## SUC-002: Accurate in-place turn via OTOS heading fusion

- **Actor**: Host issuing `TURN <heading_cdeg>` or `G` to a heading-offset target
- **Preconditions**: OTOS is valid and has been calibrated; EKF fusion is enabled
  (`fuseOtos = true`).
- **Main Flow**:
  1. Host sends `TURN 9000` (90°).
  2. EKF fuses OTOS heading observations at every OTOS correction cycle.
  3. Fused `poseHrad` tracks OTOS-measured heading as the robot spins.
  4. HEADING stop fires when fused heading delta reaches 90° ± tolerance.
  5. Robot stops; firmware emits `EVT done TURN`.
- **Postconditions**: Physical heading matches commanded delta within ~3°; `poseHrad`
  accurately represents the robot's true orientation.
- **Acceptance Criteria**:
  - [ ] Four consecutive `TURN 9000` commands return robot to starting orientation
        within a few degrees (hardware check).
  - [ ] In sim (field profile, fusion + mock slip enabled): fused heading tracks
        mock-OTOS truth within ~2° across a square + figure-eight run.

---

## SUC-003: Watchdog stops open-ended streaming; self-terminating commands run uninterrupted

- **Actor**: Host program (with or without keepalive daemon)
- **Preconditions**: Safety is ON; firmware default sTimeout (500 ms) is in effect.
- **Main Flow (streaming)**:
  1. Host starts `S l r`; robot drives.
  2. Host goes silent; 500 ms later firmware fires `EVT safety_stop`.
- **Main Flow (self-terminating)**:
  1. Host sends `G`, `TURN`, `T`, or `D` command; keepalive daemon is OFF.
  2. Command runs to completion via its own TIME/HEADING/POSITION stop.
  3. Firmware emits `EVT done <verb>`; no safety_stop fires.
- **Postconditions**: Link-loss is detected for streaming commands; self-terminating
  commands are never killed by the watchdog.
- **Acceptance Criteria**:
  - [ ] `G`, `TURN`, `T`, `D` complete with zero keepalives sent, safety ON.
  - [ ] `S` without keepalives triggers `EVT safety_stop` at sTimeoutMs.
  - [ ] `SAFE off` then new motion command → `EVT safety re-armed`, safety restored.
  - [ ] `sTimeout=60000` override removed from `tests/bench/square_run.py` and
        host test fixtures.

---

## SUC-004: EKF recovers from pose divergence within one second

- **Actor**: EKF on firmware (automatic, every OTOS correction cycle)
- **Preconditions**: EKF has diverged (consecutive gate rejections); OTOS is valid.
- **Main Flow**:
  1. Heading drift or position jump causes > 10 consecutive Mahalanobis gate
     rejections in `updatePosition` or `updateHeading`.
  2. Recovery path triggers: R_eff is inflated for that update (or update accepted
     unconditionally); rejection streak resets.
  3. Filter converges toward OTOS truth within ~1 s.
  4. `ekf_rej` count appears in TLM stream; host can observe the recovery.
- **Postconditions**: EKF is no longer in "confidently wrong, forever" state; `ekf_rej`
  telemetry reflects the event.
- **Acceptance Criteria**:
  - [ ] Sim: teleporting mock-OTOS pose 200 mm mid-run → fused pose converges in < 2 s.
  - [ ] `ekf_rej` appears in TLM and rises during induced divergence, falls after recovery.

---

## SUC-005: Encoder heading prediction corrected for rotational slip

- **Actor**: Firmware odometry (automatic, every predict step)
- **Preconditions**: `rotationalSlip` is configured in `tovez.json` (default 0.74).
- **Main Flow**:
  1. Robot executes `RT 9000` (relative 90° rotation).
  2. `Odometry::predict()` applies `rotationalSlip` to the raw encoder delta-theta.
  3. EKF prediction step uses corrected dθ; subsequent OTOS heading fuses from a
     better prior.
  4. Physical rotation matches commanded angle within tolerance.
- **Postconditions**: Encoder-derived heading prediction matches physical reality more
  closely; dead-reckoning quality between OTOS fixes is improved.
- **Acceptance Criteria**:
  - [ ] `RT 9000` lands 90° ± 3° physical (protractor or OTOS readout) on hardware.
  - [ ] Sim (field profile): predicted heading matches mock-body truth after correction.
  - [ ] `beginRotation()` (RT) compensates the wheel-arc target for slip correctly.
