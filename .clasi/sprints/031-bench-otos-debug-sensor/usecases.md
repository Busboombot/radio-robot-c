---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 031 Use Cases

## SUC-001: Enable bench OTOS simulation mode
Parent: UC-debug

- **Actor**: Developer (bench session via rogo or serial terminal)
- **Preconditions**: Robot is flashed with sprint 031 firmware; robot sits
  on a stand with wheels free-spinning; real OTOS sees no optical motion.
- **Main Flow**:
  1. Developer sends `DBG OTOS BENCH 1` to the robot.
  2. Robot replies `OK dbg otos bench=1`.
  3. BenchOtosSensor is now the active OTOS source; the real OtosSensor
     is bypassed for all pose/velocity reads.
  4. Developer optionally tunes noise/drift via `DBG OTOS BENCH 1 noiseXY=<f>
     noiseH=<f> drift=<f>` arguments.
- **Postconditions**: `hal.otos()` returns synthesized pose data from
  integrated commanded-velocity; the EKF fuses it on every otosCorrect tick.
- **Acceptance Criteria**:
  - [ ] `DBG OTOS BENCH 1` reply is `OK dbg otos bench=1`
  - [ ] `DBG OTOS BENCH 0` returns to real OTOS; reply is `OK dbg otos bench=0`
  - [ ] Noise/drift params accepted and reflected in subsequent `DBG OTOS` query

## SUC-002: Drive a bench session with synthesized OTOS pose
Parent: UC-debug

- **Actor**: Developer running full-stack bench validation
- **Preconditions**: BenchOtosSensor enabled (SUC-001); robot on stand.
- **Main Flow**:
  1. Developer issues a drive command (e.g., `D 500` or `S 200 200`).
  2. Firmware runs: MotionController sets tgtLMms/tgtRMms in
     `state.commands`; BenchOtosSensor integrates those each control tick;
     `otosCorrect` fuses the synthesized pose into the EKF.
  3. EKF fused pose advances as if the robot were driving on the floor.
- **Postconditions**: OTOS-fused pose in `state.inputs` reflects a
  plausible trajectory; motion stop conditions (distance, time) fire
  normally; TLM streams pose data.
- **Acceptance Criteria**:
  - [ ] A `D 500` command on the bench terminates with `EVT done` (distance
    stop fires) rather than timing out
  - [ ] TLM `otos_x`/`otos_y`/`otos_h` advance monotonically during drive

## SUC-003: Inspect ideal vs errored vs EKF-fused pose
Parent: UC-debug

- **Actor**: Developer diagnosing EKF or noise model behavior
- **Preconditions**: BenchOtosSensor enabled; robot has been driven.
- **Main Flow**:
  1. Developer sends `DBG OTOS`.
  2. Robot replies with a single line containing ideal pose (noiseless
     integrator), errored OTOS pose (noise + drift applied), and current
     EKF-fused pose.
- **Postconditions**: Developer can compare all three poses to validate
  the noise model and EKF correction.
- **Acceptance Criteria**:
  - [ ] `DBG OTOS` reply contains `ideal=`, `otos=`, and `fused=` fields
  - [ ] When noise=0 and drift=0, ideal == otos within float precision
  - [ ] When bench mode is off, `DBG OTOS` still reports real OTOS vs EKF

## SUC-004: Host-sim validates BenchOtosSensor integrator correctness
Parent: UC-test

- **Actor**: CI / automated test suite
- **Preconditions**: HOST_BUILD; zero-noise BenchOtosSensor constructed.
- **Main Flow**:
  1. Test drives the integrator with known tgtL/tgtR values over N ticks.
  2. Compares noiseless accumulated pose against analytic ground truth.
  3. Enables noise; asserts accumulated pose stays within the expected
     statistical band.
- **Postconditions**: BenchOtosSensor is verified correct before hardware flash.
- **Acceptance Criteria**:
  - [ ] Zero-noise oracle test passes: accumulated pose matches analytic
    formula within 0.1 mm / 0.001 rad after 100 ticks
  - [ ] Nonzero-noise test passes: accumulated pose stays within 3-sigma
    of the noise-free oracle after 100 ticks
  - [ ] `python3 build.py` clean build passes
  - [ ] `uv run --with pytest python -m pytest host_tests/ host/tests/` passes
