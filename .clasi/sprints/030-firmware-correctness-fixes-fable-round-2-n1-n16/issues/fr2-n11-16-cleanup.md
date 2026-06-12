---
status: in-progress
sprint: '030'
tickets:
- 030-009
---

# FR2-N11..N16 (Low) — Correctness cleanup cluster

## Context

Source: `docs/code_review/2026-06-12-Fable-correctness-review/findings.md` §N11–§N16.
(N15 also covers `d12` item #1; EVT truncation overlaps `d12` item #3.)

- **N11 (Low-Med):** PURSUE backtrack re-gate cancels the PURSUE MotionCommand with
  HARD (`MotionController.cpp:698`); `cancel()` emits `EVT cancelled #<corrId>` for the
  G's id, so a host sees a spurious cancelled then later `EVT done G #<same id>`.
  Suppress the EVT for internal phase transitions (`cancelQuiet()` / clear the sink
  before cancel, as `_startPreRotate` already does).
- **N12 (Low-Med):** full `GET` dump (~50 keys, 600-800 bytes) exceeds CODAL's 255-byte
  TX buffer (`SerialPort.cpp:17`) — bare `GET` over serial is truncated. Verify on the
  bench; if confirmed, chunk into multiple ≤200-byte `CFG` lines.
- **N13 (Low):** residual dead/vestigial code — `RatioPidController` (constructed,
  SET-tunable via `pid.*`, never run), `PID_BYPASS` (`MotorController.cpp:12`),
  `Odometry::update()` (no callers), unreachable `DriveMode::TIMED` (T runs as
  VELOCITY, so TLM `mode=` can never read `T` — check host parsers). Remove/retire.
- **N14 (Low):** queued-path `corrId` truncated to 7 chars — `ParsedCommand::corrId`
  is 8 bytes (`CommandTypes.h:158`) while tokenizer/MotionCommand/TargetState carry 16.
  >7-digit ids (ms timestamps) silently truncated on every queued reply/EVT. Make
  sizes uniform (16) or reject long ids loudly.
- **N15 (Low):** EKF process noise loop-rate-coupled — `EKF::predict()` adds full `Q`
  per call ignoring `dt_s` (`EKF.cpp:149`); effective Q varies ~2.5x with bus traffic.
  Scale Q by dt (or gate predict to controlPeriod). (= d12 #1.)
- **N16 (Low):** invalid `sensor=` stop silently ignored on the queue path
  (`MotionCommandHandlers.cpp:784-793`) — parse failure skips the stop after the host
  already got `OK`. Validate in the converter before replying OK (mirror the direct
  path's ERR+cancel).

Optional: widen/bound-check the 48-byte EVT buffer (`emitEvt`) — d12 #3.

## Fix

Address each per its note above. Each is small; group as cleanup tickets. Prioritize
N15 (estimator trust) and N16 (silent behavioral surprise); N12 needs bench
confirmation first.

## Acceptance

- N11: PURSUE re-gate emits no `EVT cancelled` for the G's corrId (sim test).
- N14: a 16-char corrId round-trips intact on the queue path (sim test).
- N15: EKF Q effect is invariant to loop rate (predict gated/scaled) — sim test.
- N16: invalid `sensor=` on the queue path returns ERR before OK (sim test).
- N13: dead code removed; host TLM parsers confirmed not to expect `mode=T`.
- N12: GET-over-serial truncation confirmed on the bench and chunked if real.
