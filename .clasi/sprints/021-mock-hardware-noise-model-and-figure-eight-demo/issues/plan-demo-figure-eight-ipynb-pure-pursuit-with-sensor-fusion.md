---
status: in-progress
sprint: '021'
tickets:
- 021-001
---

# Plan: Mock Hardware Noise Model + `demo_figure_eight.ipynb`

## Context

This issue covers two related deliverables:

1. **C++ mock hardware error model** — add realistic noise/error to `MockMotor`,
   `MockOtosSensor`, and `MockHAL` so the sim produces encoder drift and OTOS drift
   comparable to real hardware. An `ExactPoseTracker` inside MockHAL provides an oracle
   ground-truth pose (the "camera") separate from the noisy dead-reckoning.

2. **Notebook** (`host_tests/demo_figure_eight.ipynb`) — drives the sim along a
   figure-eight path using Pure Pursuit and compares three position-estimation regimes
   (dead reckoning → OTOS + camera → Kalman filter fusion). Noise comes from the C++ layer,
   not from Python wrappers.

**Why errors belong in C++:** In real life the error originates in the hardware — wheel slip
at the ground contact, encoder quantisation, OTOS drift. The sim should model that. Python
code querying the sim gets the same "lies" the firmware gets.

---

## Part 1 — C++ Mock Hardware Changes

### 1a. `MockMotor` — slip + encoder noise

Add to `source/hal/mock/MockMotor.h`:
```cpp
// Slip model (set by MockHAL before tick()).
float _turnRate        = 0.0f;  // [0..1]: 0=straight, 1=point-turn; set each tick
float _slipStraight    = 0.0f;  // baseline encoder under-report fraction (e.g. 0.01)
float _slipTurnExtra   = 0.0f;  // additional under-report at full point-turn (e.g. 0.04)
float _encoderNoiseSigma = 0.0f; // Gaussian noise std-dev (mm) added per tick

// Setters
void setSlip(float straight, float turnExtra);
void setEncoderNoise(float sigmaMm);
void setTurnRate(float r) { _turnRate = r; }  // called by MockHAL

// True (no-slip) velocity for ExactPoseTracker
float trueVelocityMms() const { return _trueVelMms; }
```

Modified `tick()` in `MockMotor.cpp`:
```cpp
void MockMotor::tick(uint32_t dt_ms) {
    float dt_s   = dt_ms / 1000.0f;
    float vel    = (_cmdSpeed / 100.0f) * kNominalMaxMms * _offsetFactor;
    _trueVelMms  = vel;                                  // exact; ExactPoseTracker uses this
    float slip   = _slipStraight + _slipTurnExtra * _turnRate;
    float noisy  = vel * (1.0f - slip) + gaussianNoise(_rng, _encoderNoiseSigma);
    _encoderMm  += noisy * dt_s;
}
```

`gaussianNoise()` — Box-Muller with a per-object `std::mt19937` (host-only code, `<random>`
is fine).

**Turn-rate coupling:** `_turnRate` is set by `MockHAL::tick()` from the two motors'
commands before calling `motor.tick()`. This keeps MockMotor itself simple (no cross-motor
dependency):
```
turnRate = |cmdL - cmdR| / (|cmdL| + |cmdR| + ε)   ∈ [0, 1]
```
- Straight (cmdL=cmdR): turnRate=0, slip=slipStraight
- Point turn (cmdL=-cmdR): turnRate=1, slip=slipStraight+slipTurnExtra

High-acceleration slip can be added later as a separate param; leave it out for now.

---

### 1b. `MockHAL` — ExactPoseTracker + coordination

Add inline struct to `source/hal/mock/MockHAL.h`:
```cpp
struct ExactPoseTracker {
    float x = 0, y = 0, h = 0;   // mm, mm, rad
    void reset() { x = y = h = 0; }
    void update(float velLMms, float velRMms, float trackwidthMm, uint32_t dt_ms);
    // midpoint integration identical to Odometry::predict
};
```

`ExactPoseTracker::update()` uses the motors' `trueVelocityMms()` — no slip, no noise.

Modified `MockHAL::tick()`:
```cpp
void MockHAL::tick(uint32_t now_ms) {
    int32_t dt = (int32_t)(now_ms - _lastTickMs);
    if (dt > 0) {
        uint32_t udt = (uint32_t)dt;

        // Compute turn rate from current commands.
        float aL  = fabsf(_motorL.cmdSpeed()), aR = fabsf(_motorR.cmdSpeed());
        float mag = aL + aR;
        float turnRate = (mag > 0.5f) ? fabsf(_motorR.cmdSpeed() - _motorL.cmdSpeed()) / mag : 0.0f;

        // Feed turn rate so motors apply correct slip.
        _motorL.setTurnRate(turnRate);
        _motorR.setTurnRate(turnRate);

        // Tick motors — encoder noise/slip applied here.
        _motorL.tick(udt);
        _motorR.tick(udt);

        // Update oracle pose from true (pre-slip) velocities.
        _exactPose.update(_motorL.trueVelocityMms(), _motorR.trueVelocityMms(),
                          _trackwidthMm, udt);

        // Update OTOS sim model from true velocities + its own noise.
        _otos.tick(_motorL.trueVelocityMms(), _motorR.trueVelocityMms(),
                   _trackwidthMm, udt);

        _line.tick(udt);
        _color.tick(udt);
    }
    _lastTickMs = now_ms;
}
```

`_trackwidthMm` is populated from `RobotConfig` at MockHAL construction (needs a ctor arg
or a `setTrackwidth()` call from SimHandle after constructing cfg).

Add accessors:
```cpp
ExactPoseTracker& exactPoseMock() { return _exactPose; }
void setTrackwidth(float mm) { _trackwidthMm = mm; }
```

---

### 1c. `MockOtosSensor` — drift sim model

Add to `source/hal/mock/MockOtosSensor.h`:
```cpp
// Sim-driven integration (disabled by default; existing injection still works).
void enableSimModel(bool on) { _useSimModel = on; }
void setLinearNoise(float sigma) { _linearNoiseSigma = sigma; }
void setYawNoise(float sigma)    { _yawNoiseSigma    = sigma; }

// Called by MockHAL::tick() with true wheel velocities.
void tick(float velLMms, float velRMms, float trackwidthMm, uint32_t dt_ms);
```

`tick()` in `MockOtosSensor.cpp`:
```cpp
void MockOtosSensor::tick(float velL, float velR, float tw, uint32_t dt_ms) {
    if (!_useSimModel) return;
    float dt_s = dt_ms / 1000.0f;
    float dL = velL * dt_s, dR = velR * dt_s;
    float dC  = (dL + dR) * 0.5f;
    float dTh = (dR - dL) / tw;

    // Linear noise: fractional error on displacement
    float nL  = gaussianNoise(_rng, _linearNoiseSigma);
    float nTh = gaussianNoise(_rng, _yawNoiseSigma);
    float noisyDC  = dC  * (1.0f + nL);
    float noisyDTh = dTh * (1.0f + nTh);

    float hMid = _odomH + noisyDTh * 0.5f;
    _odomX += noisyDC * cosf(hMid);
    _odomY += noisyDC * sinf(hMid);
    _odomH = wrapPi(_odomH + noisyDTh);
}
```

`readTransformed()` returns `_odomX/Y/H` when `_useSimModel`, else the injected pose
(unchanged from today). `setInjectedPose()` also resets `_odomX/Y/H` so external resets
still work.

**Noise magnitudes:**
- Encoder: `slipStraight=0.005`, `slipTurnExtra=0.03`, `noiseSigma=0.05 mm/tick` (~1% total)
- OTOS: `linearNoise=0.01`, `yawNoise=0.025` (1% linear, 2.5% yaw per step)

These are the defaults for the notebook demo; all are settable at runtime.

---

### 1d. `sim_api.cpp` — new C functions

```c
// Oracle ground-truth pose (ExactPoseTracker, not dead-reckoning odometry).
float sim_get_exact_pose_x(void* h);
float sim_get_exact_pose_y(void* h);
float sim_get_exact_pose_h(void* h);

// Encoder slip model (side: 0=left, 1=right, 2=both).
void sim_set_motor_slip(void* h, int side, float straight, float turn_extra);

// Encoder Gaussian noise (mm std-dev per tick).
void sim_set_encoder_noise(void* h, int side, float sigma_mm);

// OTOS sim model toggle + noise.
void sim_enable_otos_model(void* h);
void sim_set_otos_linear_noise(void* h, float sigma_fraction);
void sim_set_otos_yaw_noise(void* h, float sigma_fraction);

// OTOS accumulated pose read (what the firmware's otosCorrect sees).
float sim_get_otos_x(void* h);
float sim_get_otos_y(void* h);
float sim_get_otos_h(void* h);
```

SimHandle ctor also calls `hal.setTrackwidth(cfg.trackwidthMm)` after building cfg.

---

### 1e. `sim_conn.py` — Python bindings

New methods on `SimConnection`:

```python
def get_exact_pose(self) -> dict[str, float]:
    """Oracle ground-truth pose from ExactPoseTracker (mm, rad)."""

def set_slip(self, straight: float = 0.005, turn_extra: float = 0.03) -> None:
    """Both wheels: straight-driving slip + extra slip at full point turn."""

def set_encoder_noise(self, sigma_mm: float = 0.05) -> None:
    """Both wheels: Gaussian encoder noise std-dev per tick (mm)."""

def enable_otos_model(self) -> None:
    """Switch MockOtosSensor to integrate from motor commands + noise."""

def set_otos_noise(self, linear: float = 0.01, yaw: float = 0.025) -> None:
    """OTOS noise fractions (applied per integration step)."""

def get_otos_pose(self) -> dict[str, float]:
    """Read OTOS accumulated noisy pose (what otosCorrect sees)."""
```

Also extend `_snapshot()` and `_setup_types()` to include `exact_pose_x/y/h` and
`otos_x/y/h` in the state log (optional; adds clarity).

---

## Part 2 — Notebook Structure (revised)

The notebook is unchanged at a high level but drops all Python noise classes. Noise
is configured via C++ API; Python just reads the results.

### Cell 1 — Build & Imports
Same boilerplate. `make_sim()` now also enables the noise model:
```python
def make_sim(slip=(0.005, 0.03), enc_noise=0.05,
             otos_linear=0.01, otos_yaw=0.025):
    conn = SimConnection()
    conn.connect()
    proto = NezhaProtocol(conn)
    proto.set_config(sTimeout=60000)
    conn.set_slip(*slip)
    conn.set_encoder_noise(enc_noise)
    conn.enable_otos_model()
    conn.set_otos_noise(otos_linear, otos_yaw)
    return conn, proto
```

### Cell 2 — Figure-Eight Path
Unchanged.

### Cell 3 — Pure Pursuit + EKF helpers (Python only)

**`pure_pursuit_vw(path, pos_mm, yaw_rad, lookahead_mm, base_speed_mms)`** — same as
original plan: computes curvature κ and returns `(v_mms, omega_mrads)`.

**`EKF`** — same as original plan: 3-state EKF for [x, y, theta], predict from VW
control input, update from OTOS or camera.

No `EncoderOdometer` or `OTOSOdometer` classes — the C++ sim provides those.

### Cell 4 — Experiment 1: Dead Reckoning Only

```python
conn, proto = make_sim()
# In this experiment, position estimate = firmware dead-reckoning from noisy encoders
# (conn.get_state()["pose_x/y/h"])  <- noisy because encoders have slip+noise
# Ground truth = conn.get_exact_pose()   <- oracle from ExactPoseTracker

old_enc = conn.get_state()
while not lap_done:
    state = conn.get_state()
    pos_est = (state["pose_x"], state["pose_y"])   # noisy
    yaw_est = state["pose_h"]
    
    v, omega = pure_pursuit_vw(path, pos_est, yaw_est, ...)
    conn.send(f"VW {int(v)} {int(omega)}")
    conn.send("+")
    conn.tick(50)
    
    truth_log.append(conn.get_exact_pose())
    est_log.append(conn.get_state())
```

### Cell 5 — Experiment 2: OTOS + Delayed Camera Fixes

```python
# Position estimate = OTOS accumulated noisy pose (conn.get_otos_pose())
# Every 30 cycles: inject exact pose as camera fix (5-cycle delay)
# Hard-reset OTOS pose via conn.set_otos_pose(x, y, h) (existing API)
```

### Cell 6 — Experiment 3: Kalman Filter Fusion

```python
# predict() from VW command
# update_otos() from conn.get_otos_pose() every cycle
# update_camera() from conn.get_exact_pose() with 5-cycle delay every 30 cycles
```

### Cell 7 — Comparison Plot
Unchanged from original plan.

---

## Files Changed

| File | Change |
|------|--------|
| `source/hal/mock/MockMotor.h` | Add slip/noise fields + setters |
| `source/hal/mock/MockMotor.cpp` | Enhanced `tick()` with slip + Box-Muller noise |
| `source/hal/mock/MockHAL.h` | Add `ExactPoseTracker` struct + `_exactPose`, `_trackwidthMm` |
| `source/hal/mock/MockHAL.cpp` | Turn-rate computation, exact pose update, OTOS tick |
| `source/hal/mock/MockOtosSensor.h` | Add `tick()`, noise fields, sim model flag |
| `source/hal/mock/MockOtosSensor.cpp` | Implement noisy `tick()`, update `readTransformed()` |
| `host_tests/sim_api.cpp` | New `sim_get_exact_pose_*`, `sim_set_motor_slip`, etc. |
| `host/robot_radio/io/sim_conn.py` | New Python bindings + snapshot fields |
| `host_tests/demo_figure_eight.ipynb` | New notebook (all cells) |

All changes are in the host-side sim layer. Zero impact on real firmware.

---

## Verification

1. Build `libfirmware_host` — no errors.
2. In a Python REPL: `conn.set_slip(0.005, 0.03)` → drive straight → `conn.get_state()
   ["pose_x"]` diverges from `conn.get_exact_pose()["x"]` over distance.
3. Drive a circle → divergence is larger than straight (slip model working).
4. `conn.enable_otos_model()` → OTOS pose drifts independently.
5. Notebook Cell 4: encoder dead-reckoning visibly drifts from reference path.
6. Notebook Cell 6: EKF trajectory stays closest to reference across all three.
