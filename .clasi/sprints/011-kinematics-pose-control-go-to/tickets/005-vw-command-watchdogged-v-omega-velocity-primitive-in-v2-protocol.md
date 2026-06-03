---
id: '005'
title: 'VW command: watchdogged (v,omega) velocity primitive in v2 protocol'
status: open
use-cases:
- SUC-003
depends-on:
- "011-001"
github-issue: ''
issue: kinematics-pose-control-goto.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 011-005: VW command — watchdogged (v, ω) velocity primitive in v2 protocol

## Description

Add the `VW` verb to the v2 command surface. `VW` is a keepalive body-twist
command (forward speed `v` + yaw rate `ω`) that uses `BodyKinematics::inverse()`
to convert to wheel setpoints and then reuses the existing `STREAMING` watchdog
path — no new `DriveMode` needed.

This ticket can be implemented in parallel with tickets 002–004 since it only
depends on the config additions from ticket 001. Sequenced after 004 in the
ticket table to keep the dependency graph serial for the programmer, but a
parallel execution is valid.

**Wire format decision** (open question from architecture): the proposal uses
**mrad/s** for omega on the wire (integer, thousandths of rad/s). Confirm with
stakeholder before implementation. Alternative is cdeg/s. The default is
mrad/s per the architecture document.

### Wire specification

```
VW <v> <omega_mrads> [#id]
→ OK vw v=<v> omega=<omega_mrads> [#id]
```

- `v` — forward speed, mm/s. Range: −1000 … +1000.
- `omega_mrads` — yaw rate in milli-radians per second. Range: −3142 … +3142
  (approximately ±π rad/s). Positive = CCW (left turn).
- Watchdog resets on each `VW` command (same `_lastSMs` as `S`).
- `EVT safety_stop [#id]` fired when no `VW` arrives within `sTimeoutMs`.

### DriveController — `beginVelocity()`

New entry point that maps `(v, ω)` to wheel setpoints and delegates to the
STREAMING path:

```cpp
void DriveController::beginVelocity(float v_mms, float omega_rads, uint32_t now_ms,
                                     ReplyFn fn, void* ctx)
{
    float vL, vR;
    BodyKinematics::inverse(v_mms, omega_rads, _cfg.trackwidthMm, vL, vR);
    float sL, sR;
    BodyKinematics::saturate(vL, vR, _cfg.vWheelMax, _cfg.steerHeadroom, sL, sR);
    // Delegate to the existing stream path (keeps watchdog logic in one place)
    beginStream(sL, sR, now_ms, fn, ctx);
}
```

`beginStream()` already: calls `_mc.startDrive()`, sets `_mode = STREAMING`,
updates `_lastSMs`, captures `_driveFn`/`_driveCtx`. No changes to
`beginStream()` itself.

### Robot — `velocityDrive()`

```cpp
void Robot::velocityDrive(float v_mms, float omega_rads,
                           ReplyFn fn, void* ctx)
{
    _driveController.beginVelocity(v_mms, omega_rads, systemTime(), fn, ctx);
}
```

Follows the exact pattern of `streamDrive()`.

### CommandProcessor — VW verb

```cpp
// ── VW — velocity body-twist (watchdogged) ──────────────────────────────
// VW <v> <omega_mrads>  → OK vw v=<v> omega=<omega_mrads>
if (strcmp(verb, "VW") == 0) {
    if (ntok < 3) {
        replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
        return;
    }
    int v     = atoi(tokens[1]);
    int omega = atoi(tokens[2]);
    if (v < -1000 || v > 1000) {
        replyErr(rbuf, sizeof(rbuf), "range", "v", corr_id, replyFn, ctx);
        return;
    }
    if (omega < -3142 || omega > 3142) {
        replyErr(rbuf, sizeof(rbuf), "range", "omega", corr_id, replyFn, ctx);
        return;
    }
    float omega_rads = (float)omega / 1000.0f;  // mrad/s → rad/s
    _robot.velocityDrive((float)v, omega_rads, replyFn, ctx);
    char body[32];
    snprintf(body, sizeof(body), "v=%d omega=%d", v, omega);
    replyOK(rbuf, sizeof(rbuf), "vw", body, corr_id, replyFn, ctx);
    return;
}
```

Add VW before the STOP handler (after G) to maintain alphabetical ordering.

### TLM mode character

The existing TLM `mode=` field uses `S` for `STREAMING`. `VW` reuses
`DriveMode::STREAMING`, so the TLM mode character remains `S`. Document this
in protocol-v2.md: "S = STREAMING (set by either S or VW command)".

## Acceptance Criteria

- [ ] `VW 200 0` → `OK vw v=200 omega=0`; robot drives straight forward. [bench]
- [ ] `VW 0 500` → `OK vw v=0 omega=500`; robot spins in place CCW. [bench]
- [ ] `VW 200 300` → `OK vw v=200 omega=300`; robot drives a curved arc (left
  turn). [bench]
- [ ] Watchdog fires within `sTimeoutMs` ± one tick when VW stream stops;
  `EVT safety_stop` emitted. [bench]
- [ ] `VW 200 0 #7` → `OK vw v=200 omega=0 #7`; watchdog stop → `EVT safety_stop #7`. [bench]
- [ ] `VW 1001 0` → `ERR range v`. [unit]
- [ ] `VW 200 3200 0` (too many args parsed as range error) — verify `VW 200 3143` → `ERR range omega`. [unit]
- [ ] `STOP` during VW halts immediately; no `EVT`. [bench]
- [ ] `GET` full dump still within 512-byte buffer (no new keys added in this ticket). [unit]
- [ ] All existing tests pass.

## Implementation Plan

### Approach

Three files, one new method each, no new drive modes. Can be executed in parallel
with tickets 002–004 by a second programmer if available.

### Files to modify

- `source/control/DriveController.h` — add `beginVelocity()` declaration
- `source/control/DriveController.cpp` — add `beginVelocity()` definition
- `source/app/Robot.h` — add `velocityDrive()` declaration
- `source/app/Robot.cpp` — add `velocityDrive()` definition
- `source/app/CommandProcessor.cpp` — add `VW` verb handler in `process()`
- `docs/protocol-v2.md` — add VW section under Motion Commands; add VW to
  HELP verb list; update Named Key Table with four new keys (aMax, aDecel,
  turnGate, arriveTol) from ticket 001; note that TLM mode=S covers both S and VW

### Testing plan

Unit tests: range validation (`v`, `omega`), `badarg` on missing args.
Bench tests: straight, spin, arc, watchdog, corr-id echoing on watchdog stop.

### Documentation updates

`docs/protocol-v2.md`:
- Add `### VW — Body-Twist Velocity Drive` section under §10 Motion Commands
- Update HELP response string example to include `VW`
- Add four new keys to Named Key Table (§7 `SET`/`GET`): `aMax`, `aDecel`,
  `turnGate`, `arriveTol`
- Note that `mode=S` in TLM covers both `S` and `VW` streaming modes
