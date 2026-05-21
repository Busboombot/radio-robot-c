---
id: '003'
title: Add computeArc and G go-to command to CommandProcessor
status: done
use-cases: []
depends-on:
- '001'
- '002'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add computeArc and G go-to command to CommandProcessor

## Description

Add the `computeArc()` pure function and the two-phase G go-to state machine to
CommandProcessor. Also wire up all 13 new K-command setters (ratio PID params + arc/G
params) in the K dump and K setter blocks.

This ticket replaces the single-arg gripper G handler (sprint 3) with a three-arg
go-to G handler. When `G+X+Y+Speed` is received (3 args), it starts the go-to state
machine. The single-arg gripper G (1 arg) may coexist or be removed — see the sprint
notes. Because the protocol uses the same `G` prefix, the simplest approach is:
parse the arg count; if 3 args, go-to G; if 1 arg, gripper G (backward compat); if 0 args,
query gripper angle. If backward compat with gripper is not required, remove the gripper path.

### New K Parameters

The existing `_mc->gains` struct (sprint 2) is being replaced by ticket 002's ratio PID
state (stored in `CalibParams` and `_pid` on MotorController). The old K commands
(KFF, KSP, KSI, KIC, KSR) still exist in CommandProcessor but now point at removed fields.
This ticket must also clean up those stale K setters that reference `_mc->gains`.

After ticket 002, MotorController no longer has a public `gains` struct. The new
calibration home is `CalibParams` fields accessible via a `CalibParams& cal` reference
(or equivalent). Determine the correct way to update CalibParams fields at runtime — if
CommandProcessor holds a reference to CalibParams, update it directly; if not, add a setter
method. Check main.cpp or Robot.cpp for how CalibParams is wired to MotorController and
CommandProcessor.

New K commands to add (in addition to cleaning up stale ones):

| K command | CalibParams field | Wire encoding | Default |
|-----------|-------------------|---------------|---------|
| KLF       | kScaleLF          | * 1000        | 1000    |
| KLB       | kScaleLB          | * 1000        | 1000    |
| KRF       | kScaleRF          | * 1000        | 1000    |
| KRB       | kScaleRB          | * 1000        | 1000    |
| KCP       | ratioPidKp        | * 10          | 3000    |
| KCI       | ratioPidKi        | * 1000        | 0       |
| KCD       | ratioPidKd        | * 1000        | 0       |
| KCC       | ratioPidMax       | integer       | 30      |
| KAT       | kAdjThreshold     | * 1000        | 500     |
| KAG       | kAdjGain          | * 1000        | 50      |
| KTW       | trackwidthMm      | integer mm    | 120     |
| KGT       | turnThresholdMm   | integer deg   | 50      |
| KGD       | doneTolMm         | integer mm    | 5       |

Wire encoding: all K protocol values are integers (no floats on wire). Use the
same pattern as existing K commands:
- Scale factor params (kScaleLF etc.): encode as `value * 1000` (e.g. 1.0 -> +1000)
- Gain params (ratioPidKp): ratioPidKp=300.0 encoded as +3000 (multiply by 10)
- Integer params (trackwidthMm, doneTolMm, turnThresholdMm): encode directly as integers

Match the existing K dump format: `"K:KLF:%+d"` with the integer wire value.

### Architecture: Where CalibParams Lives at Runtime

Read `source/app/Robot.cpp` (or equivalent main setup file) to understand how CalibParams
is constructed and passed to MotorController and CommandProcessor. The K setter needs a
pointer or reference to the live CalibParams. If CommandProcessor already holds `params`
(its own Params struct) for encoder/tick params, add a pointer `CalibParams* _cal` to
CommandProcessor that is set in `init()`. Then K setters for ratio PID params write
directly to `*_cal`.

Also: when a K setter changes ratioPidKp/Ki/Kd/Max at runtime, the live `_pid` object in
MotorController needs to be updated too. Add a method to MotorController (e.g.
`updatePidGains(float kP, float kI, float kD, float iClamp)`) so CommandProcessor can
push the new value into the running PID without reconstructing it.

### G Command State Machine

#### Private state to add to CommandProcessor.h

```cpp
enum class GPhase { IDLE, PRE_ROTATE, ARC };

GPhase  _gPhase;
float   _gTargetX;      // commanded X in mm
float   _gTargetY;      // commanded Y in mm
float   _gSpeed;        // commanded speed in mm/s (positive)
float   _gArcLeftMm;    // left wheel arc distance in mm (from computeArc)
float   _gArcRightMm;   // right wheel arc distance in mm
float   _gArcStartL;    // encoder position when arc phase started (mm)
float   _gArcStartR;
```

Initialize `_gPhase = GPhase::IDLE` in CommandProcessor constructor.

#### `computeArc()` — pure function

Add as a static private method to CommandProcessor:

```cpp
/**
 * Compute differential arc wheel distances for a relative XY target.
 * Robot starts at (0,0,0). Heading=0 is forward (+X direction).
 *
 * @param tx          Target X in mm (forward from robot)
 * @param ty          Target Y in mm (left from robot)
 * @param trackwidthMm  Distance between wheel contact patches in mm
 * @param leftMm      Output: left wheel distance in mm (signed)
 * @param rightMm     Output: right wheel distance in mm (signed)
 */
static void computeArc(float tx, float ty, float trackwidthMm,
                        float& leftMm, float& rightMm);
```

Implementation (from `.clasi/issues/firmware-ratio-pid-and-g-command.md`):

```cpp
void CommandProcessor::computeArc(float tx, float ty, float trackwidthMm,
                                   float& leftMm, float& rightMm)
{
    float W = trackwidthMm;
    // Special case: ty == 0 means straight ahead
    if (fabsf(ty) < 0.001f) {
        leftMm  = tx;
        rightMm = tx;
        return;
    }
    float R     = (tx * tx + ty * ty) / (2.0f * ty);
    float alpha = atan2f(ty, tx + R);
    leftMm  = (R - W / 2.0f) * alpha;
    rightMm = (R + W / 2.0f) * alpha;
}
```

#### G command handler in `process()`

Replace the existing gripper-G handler block with the following. The current gripper handler
starts with `if (buf[0] == 'G' && (len == 1 || buf[1] == '+' || buf[1] == '-'))`.
Replace that entire block:

```cpp
// ── G — go-to XY or gripper ─────────────────────────────────────────────
if (buf[0] == 'G' && (len == 1 || buf[1] == '+' || buf[1] == '-')) {
    int32_t args[3] = {0, 0, 0};
    int n = (len > 1) ? parseSignedArgs(buf + 1, args, 3) : 0;

    if (n == 3) {
        // G+X+Y+Speed — go-to command
        float tx    = (float)args[0];
        float ty    = (float)args[1];
        float speed = fabsf((float)args[2]);
        if (speed < 1.0f) speed = 1.0f;

        _gTargetX = tx;
        _gTargetY = ty;
        _gSpeed   = speed;

        float angleRad = atan2f(ty, tx);
        float kgt = _cal ? _cal->turnThresholdMm : 50.0f;  // degrees threshold
        float angleDeg = angleRad * 57.2957795f;            // radians to degrees

        if (fabsf(angleDeg) > kgt) {
            // Pre-rotate phase: rotate in place to face target
            // Rotation direction: positive ty = turn left (right wheel faster)
            // turnScale / trackwidthMm are used to compute rotation arc
            // For simplicity: call startDriveClean with turn speeds
            // Turn left (CCW): left wheel negative, right wheel positive
            float turnSign = (ty >= 0.0f) ? 1.0f : -1.0f;
            _mc->startDriveClean(-turnSign * speed, turnSign * speed);
            _mc->setTarget(-turnSign * speed, turnSign * speed);
            _tgtL = -turnSign * speed;
            _tgtR =  turnSign * speed;
            // Compute how far to turn: arc length = (trackwidth/2) * |angleRad|
            float tw = _cal ? _cal->trackwidthMm : 120.0f;
            _gArcLeftMm  = -turnSign * (tw / 2.0f) * fabsf(angleRad);
            _gArcRightMm =  turnSign * (tw / 2.0f) * fabsf(angleRad);
            int32_t el, er;
            _mc->getEncoderPositions(el, er);
            _gArcStartL = (float)el;
            _gArcStartR = (float)er;
            _gPhase = GPhase::PRE_ROTATE;
            _mode = DriveMode::GO_TO;
        } else {
            // Arc phase directly (shallow angle)
            float tw = _cal ? _cal->trackwidthMm : 120.0f;
            computeArc(tx, ty, tw, _gArcLeftMm, _gArcRightMm);
            // Scale arc distances to speed ratio
            float maxArc = fmaxf(fabsf(_gArcLeftMm), fabsf(_gArcRightMm));
            float leftSpd  = (maxArc > 0.001f) ? (speed * _gArcLeftMm  / maxArc) : speed;
            float rightSpd = (maxArc > 0.001f) ? (speed * _gArcRightMm / maxArc) : speed;
            _mc->startDriveClean(leftSpd, rightSpd);
            _mc->setTarget(leftSpd, rightSpd);
            _tgtL = leftSpd;
            _tgtR = rightSpd;
            int32_t el, er;
            _mc->getEncoderPositions(el, er);
            _gArcStartL = (float)el;
            _gArcStartR = (float)er;
            _gPhase = GPhase::ARC;
            _mode = DriveMode::GO_TO;
        }

        char reply[48];
        snprintf(reply, sizeof(reply), "ACK:G %d %d %d",
                 (int)tx, (int)ty, (int)speed);
        replyFn(reply, ctx);
        return;
    }

    if (n == 1 || n == 0) {
        // Gripper backward-compat path
        if (n == 0) {
            if (!_gripper) { replyFn("ERR:G", ctx); return; }
            char r[16];
            snprintf(r, sizeof(r), "G%+d", (int)_currentGripperAngle);
            replyFn(r, ctx);
        } else {
            if (!_gripper) { replyFn("ERR:G", ctx); return; }
            int deg = clampInt((int)args[0], 0, 180);
            _gripper->setAngle((uint8_t)deg);
            _currentGripperAngle = deg;
            char r[24];
            snprintf(r, sizeof(r), "ACK:G %d", deg);
            replyFn(r, ctx);
        }
        return;
    }

    // n == 2: unrecognized
    char errbuf[140];
    snprintf(errbuf, sizeof(errbuf), "ERR:%s", buf);
    replyFn(errbuf, ctx);
    return;
}
```

#### G state machine in `tick()`

Add a GO_TO branch in tick() after the DISTANCE mode block:

```cpp
// G-mode: advance go-to state machine
if (_mode == DriveMode::GO_TO) {
    int32_t el, er;
    _mc->getEncoderPositions(el, er);
    float kgd = _cal ? _cal->doneTolMm : 5.0f;

    if (_gPhase == GPhase::PRE_ROTATE) {
        // Check if pre-rotation is complete
        float dL = fabsf((float)el - _gArcStartL);
        float dR = fabsf((float)er - _gArcStartR);
        float targetL = fabsf(_gArcLeftMm);
        float targetR = fabsf(_gArcRightMm);
        bool doneL = dL >= targetL - kgd;
        bool doneR = dR >= targetR - kgd;
        if (doneL && doneR) {
            // Advance to arc phase
            float tw = _cal ? _cal->trackwidthMm : 120.0f;
            computeArc(_gTargetX, _gTargetY, tw, _gArcLeftMm, _gArcRightMm);
            float maxArc = fmaxf(fabsf(_gArcLeftMm), fabsf(_gArcRightMm));
            float leftSpd  = (maxArc > 0.001f) ? (_gSpeed * _gArcLeftMm  / maxArc) : _gSpeed;
            float rightSpd = (maxArc > 0.001f) ? (_gSpeed * _gArcRightMm / maxArc) : _gSpeed;
            _mc->startDriveClean(leftSpd, rightSpd);
            _mc->setTarget(leftSpd, rightSpd);
            _tgtL = leftSpd;
            _tgtR = rightSpd;
            _gArcStartL = (float)el;
            _gArcStartR = (float)er;
            _gPhase = GPhase::ARC;
        }
    } else if (_gPhase == GPhase::ARC) {
        // Check if arc drive is complete
        float dL = (float)el - _gArcStartL;
        float dR = (float)er - _gArcStartR;
        bool doneL = fabsf(dL - _gArcLeftMm)  <= kgd;
        bool doneR = fabsf(dR - _gArcRightMm) <= kgd;
        if (doneL && doneR) {
            fullStop(replyFn, ctx);
            _gPhase = GPhase::IDLE;
            replyFn("G+DONE", ctx);
        }
    }
}
```

Note: the GO_TO mode must also run `_mc->tick(dt_s)` — confirm the existing `if (_mode != IDLE)` guard at the top of tick() already covers GO_TO, or add GO_TO to the condition.

#### CalibParams access in CommandProcessor

Add to CommandProcessor.h private section:

```cpp
CalibParams* _cal;  // pointer to live calibration params (set in init)
```

Add `_cal(nullptr)` to the constructor initializer list.

Update `init()` to accept a `CalibParams*` as an additional parameter, or add a separate
`setCalib(CalibParams* cal)` method. Pass it from Robot/main when creating CommandProcessor.

#### New K dump entries

In the `if (len == 1)` K dump block, after the existing entries, add:

```cpp
// Ratio PID params (read from _cal)
if (_cal) {
    snprintf(kbuf, sizeof(kbuf), "K:KLF:%+d", (int)(_cal->kScaleLF * 1000.0f + 0.5f));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KLB:%+d", (int)(_cal->kScaleLB * 1000.0f + 0.5f));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KRF:%+d", (int)(_cal->kScaleRF * 1000.0f + 0.5f));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KRB:%+d", (int)(_cal->kScaleRB * 1000.0f + 0.5f));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KCP:%+d", (int)(_cal->ratioPidKp * 10.0f + 0.5f));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KCI:%+d", (int)(_cal->ratioPidKi * 1000.0f + 0.5f));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KCD:%+d", (int)(_cal->ratioPidKd * 1000.0f + 0.5f));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KCC:%+d", (int)(_cal->ratioPidMax));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KAT:%+d", (int)(_cal->kAdjThreshold * 1000.0f + 0.5f));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KAG:%+d", (int)(_cal->kAdjGain * 1000.0f + 0.5f));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KTW:%+d", (int)(_cal->trackwidthMm));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KGT:%+d", (int)(_cal->turnThresholdMm));
    replyFn(kbuf, ctx);
    snprintf(kbuf, sizeof(kbuf), "K:KGD:%+d", (int)(_cal->doneTolMm));
    replyFn(kbuf, ctx);
}
```

#### New K setter cases

In the K setter block (`if (len >= 4)`), after the existing cases, add:

```cpp
if (_cal) {
    if (memcmp(key, "LF", 2) == 0) {
        _cal->kScaleLF = v / 1000.0f;
        snprintf(reply, sizeof(reply), "ACK:KLF %d", (int)(_cal->kScaleLF * 1000.0f + 0.5f));
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "LB", 2) == 0) {
        _cal->kScaleLB = v / 1000.0f;
        snprintf(reply, sizeof(reply), "ACK:KLB %d", (int)(_cal->kScaleLB * 1000.0f + 0.5f));
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "RF", 2) == 0) {
        _cal->kScaleRF = v / 1000.0f;
        snprintf(reply, sizeof(reply), "ACK:KRF %d", (int)(_cal->kScaleRF * 1000.0f + 0.5f));
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "RB", 2) == 0) {
        _cal->kScaleRB = v / 1000.0f;
        snprintf(reply, sizeof(reply), "ACK:KRB %d", (int)(_cal->kScaleRB * 1000.0f + 0.5f));
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "CP", 2) == 0) {
        _cal->ratioPidKp = v / 10.0f;
        if (_mc) _mc->updatePidGains(_cal->ratioPidKp, _cal->ratioPidKi, _cal->ratioPidKd, _cal->ratioPidMax);
        snprintf(reply, sizeof(reply), "ACK:KCP %d", (int)(_cal->ratioPidKp * 10.0f + 0.5f));
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "CI", 2) == 0) {
        _cal->ratioPidKi = v / 1000.0f;
        if (_mc) _mc->updatePidGains(_cal->ratioPidKp, _cal->ratioPidKi, _cal->ratioPidKd, _cal->ratioPidMax);
        snprintf(reply, sizeof(reply), "ACK:KCI %d", (int)(_cal->ratioPidKi * 1000.0f + 0.5f));
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "CD", 2) == 0) {
        _cal->ratioPidKd = v / 1000.0f;
        if (_mc) _mc->updatePidGains(_cal->ratioPidKp, _cal->ratioPidKi, _cal->ratioPidKd, _cal->ratioPidMax);
        snprintf(reply, sizeof(reply), "ACK:KCD %d", (int)(_cal->ratioPidKd * 1000.0f + 0.5f));
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "CC", 2) == 0) {
        _cal->ratioPidMax = (float)v;
        if (_mc) _mc->updatePidGains(_cal->ratioPidKp, _cal->ratioPidKi, _cal->ratioPidKd, _cal->ratioPidMax);
        snprintf(reply, sizeof(reply), "ACK:KCC %d", (int)_cal->ratioPidMax);
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "AT", 2) == 0) {
        _cal->kAdjThreshold = v / 1000.0f;
        snprintf(reply, sizeof(reply), "ACK:KAT %d", (int)(_cal->kAdjThreshold * 1000.0f + 0.5f));
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "AG", 2) == 0) {
        _cal->kAdjGain = v / 1000.0f;
        snprintf(reply, sizeof(reply), "ACK:KAG %d", (int)(_cal->kAdjGain * 1000.0f + 0.5f));
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "TW", 2) == 0) {
        _cal->trackwidthMm = (float)v;
        snprintf(reply, sizeof(reply), "ACK:KTW %d", (int)_cal->trackwidthMm);
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "GT", 2) == 0) {
        _cal->turnThresholdMm = (float)v;
        snprintf(reply, sizeof(reply), "ACK:KGT %d", (int)_cal->turnThresholdMm);
        replyFn(reply, ctx); return;
    }
    if (memcmp(key, "GD", 2) == 0) {
        _cal->doneTolMm = (float)v;
        snprintf(reply, sizeof(reply), "ACK:KGD %d", (int)_cal->doneTolMm);
        replyFn(reply, ctx); return;
    }
}
```

#### Update S command to call startDrive()

The existing S command handler (sprint 2) calls `_mc->resetIntegrators()` on mode change
then `_mc->setTarget()`. With the new ratio PID, the S command must call
`_mc->startDrive(leftMms, rightMms)` instead of `resetIntegrators()` + `setTarget()`.

Replace in the S handler:

```cpp
// OLD:
if (_mode != DriveMode::STREAMING) {
    _mc->resetIntegrators();
}
_mc->setTarget((float)leftMms, (float)rightMms);

// NEW:
_mc->startDrive((float)leftMms, (float)rightMms);
_mc->setTarget((float)leftMms, (float)rightMms);
```

Note: `startDrive()` handles the first-call vs keepalive distinction internally (via the
re-seeding logic). It is safe to call on every S receive.

#### Update T command to call startDriveClean()

Replace in the T handler:

```cpp
// OLD:
_mc->resetIntegrators();
_mc->setTarget((float)leftMms, (float)rightMms);

// NEW:
_mc->startDriveClean((float)leftMms, (float)rightMms);
_mc->setTarget((float)leftMms, (float)rightMms);
```

#### Update D command to call startDriveClean()

Replace in the D handler:

```cpp
// OLD:
_mc->resetIntegrators();
_mc->setTarget((float)leftMms, (float)rightMms);
_mc->resetEncoderAccumulators();

// NEW:
_mc->startDriveClean((float)leftMms, (float)rightMms);
_mc->setTarget((float)leftMms, (float)rightMms);
// resetEncoderAccumulators() is still needed for D-mode tracking — keep it
_mc->resetEncoderAccumulators();
```

#### Remove stale MotorController.gains references

The old K setters for KFF, KSP, KSI, KIC, KSR reference `_mc->gains` which no longer
exists after ticket 002. Remove or redirect those K setter cases:
- KFF: now maps to `_cal->kFF` (encode as * 1000)
- KSP, KSI, KIC, KSR: remove entirely (ratio PID replaces them; KCP/KCI/KCD/KCC take over)

Also remove the stale `_mc->gains.*` references in the K dump block.

### `updatePidGains()` in MotorController

Add this method to MotorController (declared in `.h`, implemented in `.cpp`):

```cpp
/**
 * Update PID gains at runtime (called by K-command setters).
 * Also updates the iClamp on the running _pid instance.
 */
void updatePidGains(float kP, float kI, float kD, float iClamp) {
    _pid._kP     = kP;
    _pid._kI     = kI;
    _pid._kD     = kD;
    _pid._iClamp = iClamp;
}
```

Since `_kP`, `_kI`, `_kD`, `_iClamp` are private on `RatioPidController`, either make them
package-accessible (e.g. make MotorController a friend, or add setters to RatioPidController),
or add an `updateGains()` method to `RatioPidController`:

```cpp
// In RatioPidController:
void updateGains(float kP, float kI, float kD, float iClamp) {
    _kP = kP; _kI = kI; _kD = kD; _iClamp = iClamp;
}
```

Then `MotorController::updatePidGains()` calls `_pid.updateGains(kP, kI, kD, iClamp)`.

---

## Acceptance Criteria

- [x] `computeArc()` is a static private method on CommandProcessor; `computeArc(300, 0, 120, l, r)` sets l=r=300
- [x] `computeArc(0, 150, 120, l, r)` returns non-zero left and right wheel distances with correct signs
- [x] G command with 3 args (`G+300+0+200`) sets `_mode = DriveMode::GO_TO` and `_gPhase = GPhase::ARC`
- [x] G command with `|angle| > KGT` sets `_gPhase = GPhase::PRE_ROTATE` first
- [x] G tick emits `G+DONE` when both encoder targets are within KGD mm
- [x] `K` response includes all 13 new params: KLF, KLB, KRF, KRB, KCP, KCI, KCD, KCC, KAT, KAG, KTW, KGT, KGD
- [x] `KCP+1500` sets ratioPidKp to 150.0 and `K` dump shows `K:KCP:+1500`
- [x] `KTW+130` sets trackwidthMm to 130 and `K` dump shows `K:KTW:+130`
- [x] S command calls `_mc->startDrive()` instead of `resetIntegrators()`
- [x] T command calls `_mc->startDriveClean()` instead of `resetIntegrators()`
- [x] D command calls `_mc->startDriveClean()` instead of `resetIntegrators()`
- [x] Stale `_mc->gains` references are removed; no compile error
- [x] `RatioPidController::updateGains()` exists and `MotorController::updatePidGains()` calls it
- [x] CommandProcessor holds a `CalibParams* _cal` that is set during init
- [x] Project builds without errors: `python build.py`

## Testing

Build verification only at this stage — hardware tests in ticket 004.

- **Build verification**: `python build.py` must complete without errors
- **Inspection**: K dump block produces entries for all 13 new params
- **Hardware tests in ticket 004**: K dump live on device, G command end-to-end
