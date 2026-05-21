---
status: pending
sprint: '005'
---

# Firmware: Nezha ratio-PID drive controller

## What to implement

Replace the simple velocity PI + ratio cross-coupling in `MotorController` with the
confirmed-working cumulative-distance ratio PID described below. This is the authoritative
algorithm spec for the S/T/D drive controller; the G-command arc math is covered separately
in `firmware-ratio-pid-and-g-command.md`.

**C++ targets:**
- `source/control/RatioPidController.h/.cpp` — `RatioPidController` class (port of `src/pid.ts`)
- `source/control/MotorController.h/.cpp` — replace tick loop with `driveTick` logic below
- `source/types/Config.h` — add `CalibParams` fields: KLF, KLB, KRF, KRB, KCP, KCI, KCD, KCC, KAT, KAG
- `source/app/CommandProcessor.h/.cpp` — add `startDrive` / `startDriveClean` + new K commands

**Reference source (TypeScript, confirmed working 2026-05-21):**
- `src/pid.ts` — `PidController` class
- `src/nezha.ts` — `robot` namespace, encoder + motor helpers
- Confirmed 340/339 mm final encoder over 2-second run (0.3% error)

---

## Conditions

- Robot: DFRobot Nezha V2 (micro:bit V2 extension board)
- Motors: M1 (right), M2 (left) — dumb DC motors driven by I2C PWM controller, no onboard velocity loop
- Encoders: Nezha V2 relative angle encoders, read via `nezhaV2.readRelAngle(motor)` → degrees
- Firmware: micro:bit V2 MakeCode/PXT, Static TypeScript, 20 ms tick
- Confirmed working 2026-05-21: 340/339 mm final encoder over 2-second run (0.3% error)
- Source: `src/nezha.ts` (robot namespace), `src/pid.ts` (PidController namespace)

---

## Problem Statement

The Nezha motors are dumb DC motors — the micro:bit drives them by setting a PWM percentage
(-100 to +100) via I2C. There is no onboard velocity controller. The relationship between PWM%
and actual speed is noisy and load-dependent. When commanded to drive straight (equal PWM to
both wheels), the robot drifts because the two motors have slightly different characteristics.

The goal is to keep the ratio of cumulative encoder distances equal to the ratio of commanded
speeds, regardless of load, friction, or motor asymmetry.

---

## Key Design Choices

### 1. Track cumulative distance, not instantaneous velocity

Most wheel synchronization schemes compare instantaneous velocities. This one compares
**cumulative encoder distances since the command started**. This makes the error integral over
time rather than noisy point-in-time, which suits the 20 ms tick rate and I2C encoder reads.

### 2. Normalize the error

The error is expressed as a **fraction of expected distance** (dimensionless), not raw mm.
This makes the PID gains speed-independent: the same kP works at 100 mm/s and 400 mm/s.

### 3. Always correct the faster wheel, not the slower

The commanded faster wheel gets a PID correction added to its base feed-forward PWM. The slower
wheel runs at pure feed-forward. When the PID integral accumulates beyond a threshold, a
separate "slower-wheel adjustment" begins to back off the slower wheel to create headroom.

### 4. Feed-forward is the primary drive signal

PWM is computed as `kFF * |commandedSpeed|`. This is the only term that makes the motors move.
The PID is purely a correction on top of feed-forward — it cannot drive the robot by itself.
If kFF is wrong (too low), the motors stall. If too high, the PID cannot compensate for the
excess.

---

## Calibration Parameters

All parameters live as `export let` variables on the `robot` namespace in `src/nezha.ts` and
are settable at runtime via K commands (no reflash needed).

| Variable | K command | Default | Meaning |
|---|---|---|---|
| `mmPerDegL` | KML | 0.487 | Left encoder: mm of travel per degree of rotation |
| `mmPerDegR` | KMR | 0.481 | Right encoder: mm per degree (different due to wheel radius variance) |
| `kFF` | KFF | 0.150 | Feed-forward gain: PWM% per mm/s commanded speed |
| `kScaleLF` | KLF | 1.0 | Left-forward PWM scale factor (corrects motor asymmetry) |
| `kScaleLB` | KLB | 1.0 | Left-backward PWM scale factor |
| `kScaleRF` | KRF | 1.0 | Right-forward PWM scale factor |
| `kScaleRB` | KRB | 1.0 | Right-backward PWM scale factor |
| `ratioPid.kP` | KCP | 300.0 | PID proportional gain (PWM% per unit of normalized error) |
| `ratioPid.kI` | KCI | 0.0 | PID integral gain (currently unused; set to 0) |
| `ratioPid.kD` | KCD | 0.0 | PID derivative gain (currently unused) |
| `ratioPid.iClamp` | KCC | 30 | PID integral anti-windup clamp (PWM%) |
| `kAdjThreshold` | KAT | 0.5 | Integral threshold (seconds) before slower-wheel adj activates |
| `kAdjGain` | KAG | 0.05 | Slower-wheel adj gain |

---

## PID Controller (`src/pid.ts`)

Standard discrete PID with anti-windup integral clamp:

```
integral += kI * error * dtS
integral = clamp(integral, -iClamp, +iClamp)
deriv = (error - prevError) / dtS   (0 on first call)
output = kP * error + integral + kD * deriv
```

The `integral` field is public — the slower-wheel adjustment reads it directly.

---

## Per-Command Setup

Before any drive command runs ticks, the ratio state must be initialized.

### Variables set at command start

```
cmdEncStartL   — encoder mm snapshot for left wheel when command began
cmdEncStartR   — encoder mm snapshot for right wheel when command began
driveFasterIsRight — true if |rightSpeed| >= |leftSpeed|
driveCmdRatio  — |fasterSpeed| / |slowerSpeed|, always >= 1.0
```

### `startDriveClean(leftMms, rightMms)` — used by T, D, G arc

Clean start with no history. Snapshot encoders now. Compute ratio from commanded speeds.
Reset PID state.

```
driveFasterIsRight = (|rightMms| >= |leftMms|)
driveCmdRatio = |fasterSpeed| / |slowerSpeed|   (1.0 if speeds are equal or both zero)
cmdEncStartL = encoderMm(true)    // current left encoder position in mm
cmdEncStartR = encoderMm(false)   // current right encoder position in mm
ratioPid.reset()
```

### `startDrive(leftMms, rightMms)` — used by the S (streaming) command only

The S command is continuously re-sent by the host (keepalive). On the first send, this runs.
On re-sends (keepalive) it is skipped so the PID state accumulates continuously.

The key insight: instead of resetting deltas to zero (which would make the PID think the wheels
are perfectly synchronized at t=0 regardless of their actual state), we **re-seed** cmdEncStart
so that the existing encoder deltas already represent the correct ratio for the new command.

```
// Compute new ratio from the new command's speeds
newFasterIsRight = (|rightMms| >= |leftMms|)
newRatio = |newFasterSpeed| / |newSlowerSpeed|

// How far has each wheel actually traveled since the last command started?
curL = encoderMm(true)
curR = encoderMm(false)
prevDeltaFaster = |curFaster - cmdEncStartFaster|   // faster wheel's actual distance
cmdFasterAbs = |newFasterSpeed|                      // new faster speed magnitude

// The seed for the faster wheel: whichever is larger — the actual travel so far,
// or the new commanded speed. This ensures the PID doesn't get a false "I'm behind"
// spike at startup.
seedFaster = max(prevDeltaFaster, cmdFasterAbs)
seedSlower = seedFaster / newRatio

// Place cmdEncStart such that the current encoder position corresponds to these seeds:
//   delta = current - start, so start = current - delta * sign
cmdEncStartFaster = curFaster - sign(newFasterSpeed) * seedFaster
cmdEncStartSlower = curSlower - sign(newSlowerSpeed) * seedSlower

// Reset PID only if the faster/slower assignment changed (motor role swap)
if (newFasterIsRight != driveFasterIsRight) ratioPid.reset()
```

---

## Per-Tick Drive (`driveTick`, called every 20 ms)

### Step 1: Read encoder positions

```
encLMm = round(readEncoder(left)  * mmPerDegL)
encRMm = round(readEncoder(right) * mmPerDegR)
```

`readEncoder` returns raw degrees from the Nezha hardware. Multiplying by mmPerDeg converts to
millimetres. The sign convention is already corrected in `readEncoder` so that forward motion
always returns a positive value on both wheels.

### Step 2: Compute cumulative deltas since command start

```
fDL = encLMm - cmdEncStartL    // total mm left wheel has traveled (signed)
fDR = encRMm - cmdEncStartR    // total mm right wheel has traveled (signed)

fasterDelta = |fDR| if driveFasterIsRight else |fDL|
slowerDelta  = |fDL| if driveFasterIsRight else |fDR|
```

Both are taken as absolute values because we track magnitude of travel, not direction.
Direction is encoded separately in the commanded speeds' signs.

### Step 3: Compute normalized error

```
expected = slowerDelta * driveCmdRatio
```

This is the distance the faster wheel *should have traveled* given how far the slower wheel
actually went and the commanded ratio. If the robot is tracking perfectly, `fasterDelta ==
expected`.

```
normErr = (expected - fasterDelta) / max(1, expected)
```

- `normErr > 0`: faster wheel is lagging — it hasn't traveled as far as it should have
- `normErr < 0`: faster wheel is ahead — it has traveled farther than it should have
- Dividing by `expected` normalizes to a dimensionless fraction (0 to 1 range roughly)
- `max(1, expected)` prevents division by zero at command start when both wheels are at rest

### Step 4: PID update

```
correction = ratioPid.update(normErr, dtS)
```

With kP=300 and kI=kD=0, this is just `300 * normErr`. The output is in PWM% — it is added
directly to the faster wheel's base feed-forward PWM.

### Step 5: Feed-forward base PWM for each wheel

```
scaleL = kScaleLF if tgtLMms >= 0 else kScaleLB    // pick per-direction scale
scaleR = kScaleRF if tgtRMms >= 0 else kScaleRB

baseFaster = kFF * |tgtFasterSpeed| * scaleFaster
baseSlower = kFF * |tgtSlowerSpeed| * scaleSlower
```

At the default kFF=0.150: a 200 mm/s command → base PWM = 30%. The per-direction scale
factors (kScaleLF etc.) allow trimming individual motors for persistent asymmetry.

### Step 6: Slower-wheel adjustment

When the faster wheel has been consistently lagging (large positive PID integral), the PID
correction alone may not be enough — the slower wheel also needs to back off to create
headroom. This activates only once the integral exceeds `kAdjThreshold` (0.5 s):

```
excess = ratioPid.integral - kAdjThreshold
if excess <= 0:
    adj = 0
else:
    adj = -kAdjGain * excess * baseFaster
```

`adj` is negative (backing off the slower wheel) and proportional to `baseFaster` so it
scales with speed. At kAdjGain=0.05 and threshold=0.5 s, the adjustment is gentle and only
activates under sustained load.

### Step 7: Compute and clamp final PWM

```
uFaster = clamp(baseFaster + correction, 0, 100)
uSlower = clamp(baseSlower + adj,        0, 100)
```

Both are clamped to 0..100 (positive) because direction is applied separately:

```
uL = driveFasterIsRight ? (tgtLMms >= 0 ? uSlower : -uSlower)
                        : (tgtLMms >= 0 ? uFaster : -uFaster)
uR = driveFasterIsRight ? (tgtRMms >= 0 ? uFaster : -uFaster)
                        : (tgtRMms >= 0 ? uSlower : -uSlower)
motorsPwm(round(uL), round(uR))
```

---

## Encoder Convention

`robot.readEncoder(leftWheel)` returns signed degrees where forward motion is positive on
both wheels. This is important: the Nezha M1/M2 motor wiring means one motor physically runs
"backwards" for forward motion. The sign correction is baked into `readEncoder` using the
`LEFT_FWD_SIGN` / `RIGHT_FWD_SIGN` constants (-1 or +1), so all callers see a consistent
"positive = forward" convention.

---

## What Not To Do

- **Do not track velocity.** Instantaneous encoder delta per tick is too noisy. Track cumulative
  distance since command start.
- **Do not reset cmdEncStart to the current encoder value on every S re-send.** The host re-sends
  S every ~150 ms as a keepalive. Resetting on each re-send throws away all accumulated ratio
  history and creates a startup spike on every keepalive cycle.
- **Do not apply correction to the slower wheel.** The slower wheel runs at pure feed-forward
  (plus the optional slow-down adjustment). Applying gain to both wheels creates a coupled
  control system that oscillates. Pick one wheel (the faster/commanded-higher one) and correct
  only it.
- **Do not make kFF too low.** If `kFF * speed < motor stall threshold (~22% PWM for the Nezha)`,
  the motor won't move at all and the PID will wind up helplessly. The minimum reliable speed
  is ~50 mm/s at kFF=0.150.

---

## Acceptance Criteria

- `K` response includes `KLF`, `KLB`, `KRF`, `KRB`, `KCP`, `KCI`, `KCD`, `KCC`, `KAT`, `KAG`
- `S+200+200` — both encoders finish within 5 mm of each other after a 1 m straight run
- `S+100+200` — right encoder travels ~2× left distance; ratio holds within 3% over 1 m
- `S+200+200` with one wheel impeded by hand mid-run — robot recovers to straight tracking within ~0.5 s of releasing
- `T+200+200+2000` — 2-second timed run; final encoder difference ≤ 10 mm (matching confirmed 340/339 mm result)
- `S+0+0` (stop) resets integrators; next S command starts clean
- New K commands accepted and echoed correctly in `K` dump
