---
status: done
sprint: 001-004
---

# Plan: C++ Port of radio-robot Firmware

## Context

The existing robot firmware lives in the TypeScript/micro:bit project at `/Volumes/Proj/proj/league-projects/scratch/radio-robot`. This project (`radio-robot-c`) will replace that with clean, modular C++ running on the CODAL framework for micro:bit V2. The Python host stack at `radio_robot/` remains unchanged — wire protocol compatibility is required.

The key motivations beyond a straight port:
- Path following must be pluggable (pure virtual `PathFollower` interface) to experiment with different algorithms
- Pose source must be pluggable (pure virtual `PoseProvider`) to accommodate external camera input from the Python host
- Every peripheral and subsystem gets its own class (SRP, OOP modularity)

---

## File Structure (all under `source/`)

```
source/
├── main.cpp                            # replaces existing; 15 lines, constructs Robot
├── app/
│   ├── Robot.h / Robot.cpp             # top-level composer + main loop
│   ├── CommandProcessor.h / .cpp       # drive state machine, command dispatch
│   └── Announcer.h / Announcer.cpp     # DEVICE: startup announcement + HELLO echo
├── control/
│   ├── MotorController.h / .cpp        # PI + feed-forward + ratio cross-coupling
│   └── Odometry.h / Odometry.cpp       # dead-reckoning pose integration
├── hal/
│   ├── SerialPort.h / SerialPort.cpp   # line-buffered 115200 serial
│   ├── Radio.h / Radio.cpp             # radio group 10, 4-slot ring buffer
│   ├── NezhaV2.h / NezhaV2.cpp         # motor driver + encoder HAL
│   ├── OtosSensor.h / OtosSensor.cpp   # SparkFun OTOS I2C driver (0x17)
│   ├── LineSensor.h / LineSensor.cpp   # line sensor I2C (0x1A)
│   ├── ColorSensor.h / ColorSensor.cpp # color sensor I2C (0x39/0x43)
│   ├── GripperServo.h / GripperServo.cpp # servo on P1
│   └── PortIO.h / PortIO.cpp           # J1-J4 digital/analog GPIO
├── nav/
│   ├── PoseProvider.h                  # pure virtual interface + Pose struct
│   ├── OtosPoseProvider.h / .cpp       # PoseProvider backed by OtosSensor
│   ├── DeadReckoningPoseProvider.h / .cpp  # PoseProvider backed by Odometry
│   ├── PathFollower.h                  # pure virtual interface + Waypoint struct
│   ├── PurePursuitFollower.h / .cpp    # lookahead-distance path tracking
│   └── StanleyFollower.h / .cpp        # cross-track + heading error steering
└── types/
    ├── Config.h                        # CalibParams, MotorGains, DriveMode enum
    └── Protocol.h                      # command prefixes, reply format constants
```

**Build system:** `CMakeLists.txt` already uses `RECURSIVE_FIND_FILE` on `source/`, so no CMake edits needed — just add the files.

---

## Class Design

### HAL Layer (`hal/`)

**`NezhaV2`** — motor driver + encoders
- `setPwm(int8_t leftPct, int8_t rightPct)` — raw PWM via I2C
- `readEncoder(bool isLeft)` → int32_t mm (applies FWD sign: LEFT=+1, RIGHT=-1)
- `resetEncoders()`
- Constants: `LEFT_MOTOR=M2`, `RIGHT_MOTOR=M1`, `LEFT_FWD_SIGN=+1`, `RIGHT_FWD_SIGN=-1`

**`OtosSensor`** — SparkFun OTOS at I2C 0x17
- `begin()` → bool (verify product ID 0x5F)
- `init()` — enable all signal processing (lookup table, accel, rotation, Kalman)
- `calibrateImu(samples)` — blocking, ~2.4ms/sample
- `resetTracking()` — clear Kalman, preserve position
- `getPositionRaw(x,y,h)` / `setPositionRaw(x,y,h)` — burst read/write at REG 0x20
- `getVelocityRaw(x,y,h)` — burst read at REG 0x26
- `getLinearScalar()` / `setLinearScalar(int8_t)` — 0.1% per LSB
- `getAngularScalar()` / `setAngularScalar(int8_t)`
- LSB conversions: 1 pos LSB ≈ 0.305 mm; 1 heading LSB ≈ 0.00549°

**`SerialPort`** — line-buffered serial
- `readLine(char* buf, len)` → bool (true when `\n` received)
- `send(const char*)`, `sendf(fmt, ...)` — snprintf into stack-local buffer

**`Radio`** — group 10, channel 0, power 7
- 4-slot ring buffer for burst arrival between ticks
- `poll(buf, len, isRelayed&)` → bool
- `send(msg, relay)` — relay adds `<` prefix

**`LineSensor`**, **`ColorSensor`**, **`GripperServo`**, **`PortIO`** — thin I2C/GPIO wrappers

### Control Layer (`control/`)

**`MotorController`**
- Public `MotorGains gains` — kFF=0.15, kP=0.05, kI=0.20, iClamp=60, kRatio=0.01
- `setTarget(leftMms, rightMms)`, `stop()`, `resetIntegrators()`
- `tick(dt_s)` — reads encoders from `NezhaV2`, runs PI + FF + ratio cross-coupling, applies co-clamp, calls `setPwm()`
- `getActualVelocity(l, r)`, `getEncoderPositions(l, r)`, `resetEncoderAccumulators()`
- **Integrators reset only on mode change, not on watchdog refresh** (matches TypeScript behavior)

**`Odometry`**
- Float internal state; int32_t protocol output (mm, centidegrees)
- `update(dL_mm, dR_mm, trackwidth_mm)` — differential drive integration
- `getPose(x,y,h_cdeg)`, `setPose(...)`, `zero()`

### Application Layer (`app/`)

**`CommandProcessor`**
- Public `CalibParams calib` (mmPerDegL, mmPerDegR, distScale, turnScale, minSpeedMms, tickMs=20, sTimeoutMs=200, encReportEvery=2)
- Drive mode enum: `IDLE | STREAMING | TIMED | DISTANCE`
- Command dispatch via fixed `CmdEntry { prefix, handler* }` table (linear scan, ~30 entries)
- `process(line, replyFn, ctx)` — parse + dispatch
- `tick(now_ms)` — PI update, mode timeout checks, streaming output
- Injected via `init()`: `OtosSensor*`, `LineSensor*`, `ColorSensor*`, `GripperServo*`, `PortIO*` (nullable — robot works without optional peripherals)
- S-mode watchdog: 200ms; on timeout calls `fullStop()` and emits `LOG:SAFETY_STOP`
- Streaming reports: ENC / SO / CS / LS every `encReportEvery` ticks

All 30+ commands from the TypeScript are ported:
- Motion: `X`, `STOP`, `S`, `T`, `D`
- Odometry: `SO`, `SZ`, `SI`, `ENC`, `EZ`, `K`
- OTOS: `O`, `OI`, `OK`, `OZ`, `OR`, `OP`, `OV`, `OL`, `OA`
- Sensors: `LS`, `CS`, `G`, `P`, `PA`
- Calibration: `KML`, `KMR`, `KSD`, `KST`, `KFF`, `KSP`, `KSI`, `KIC`, `KSR`, `KSM`, `KSS`, `KTR`, `KER`

**`Robot`**
- Owns static instances of all subsystems; controls init order (I2C → sensors → motor → cmd → serial → radio → announcer)
- `run()` — drain serial, drain radio, call `cmd.tick()`, `uBit.sleep(tickMs)`
- ReplyFn differentiated by context struct (serial vs. radio, relay vs. direct)

**`Announcer`**
- Emits `DEVICE:<type>:<name>:<hwName>:<serial>` on startup
- Intercepts `HELLO` before CommandProcessor; re-emits announcement

### Navigation Layer (`nav/`)

**`PoseProvider`** — pure virtual interface
```cpp
virtual void        update()  = 0;
virtual bool        getPose(Pose& out) = 0;
virtual const char* sourceName() const = 0;
```
`Pose` struct: `{int32_t x_mm, y_mm, h_cdeg; bool valid}`

Concrete implementations:
- `OtosPoseProvider` — converts raw LSB → mm/centidegrees; tracks validity
- `DeadReckoningPoseProvider` — wraps `Odometry`; always valid
- *(Future)* `ExternalCameraPoseProvider` — receives pose via `SI` command with staleness timeout; enables camera-driven navigation from Python host with no new protocol commands

**`PathFollower`** — pure virtual interface
```cpp
virtual void setPath(const Waypoint* wps, uint8_t count) = 0;
virtual bool compute(const Pose& pose, int16_t& leftMms, int16_t& rightMms) = 0;
virtual void reset() = 0;
virtual bool isFinished() const = 0;
virtual const char* name() const = 0;
```
`Waypoint` struct: `{int32_t x_mm, y_mm}`

Concrete implementations:
- `PurePursuitFollower` — lookahead κ = 2×d_lateral/Lf²; tunable lookahead_mm, trackwidth_mm, base_speed_mms, stop_dist_mm; MAX_WAYPOINTS=32 (256 bytes, static)
- `StanleyFollower` — δ = θ_e + atan2(k×e, v_soft+v); tunable k, omega_gain, goal_tol_mm

---

## Key Design Decisions

1. **Static instances in `Robot`** — all subsystems are members of the single static `Robot` object in `main.cpp`. No heap; explicit init ordering eliminates static-init-order fiasco.

2. **ReplyFn as `void(*)(const char*, void*)` not `std::function`** — avoids heap. The `void* ctx` carries serial-vs-radio and relay-vs-direct state.

3. **Command dispatch table over switch/if-else** — `CmdEntry[]` with prefix + handler pointer. Longer prefixes sort first (e.g., `OI` before `O`). O(N) scan is fast enough at 20ms ticks.

4. **CommandProcessor owns drive mode; MotorController knows only targets and gains** — clean SRP separation. Motor layer has no protocol knowledge.

5. **Integrators survive watchdog refresh** — `resetIntegrators()` only called on mode change. Direct port of the TypeScript behavior that avoids step response on each S re-send.

6. **No virtual dispatch in hot path** — `MotorController::tick()` calls `NezhaV2` through a reference, not a virtual. PathFollower/PoseProvider use virtual only in navigation code, which runs less frequently.

7. **OTOS injected as nullable pointer** — robot functions without OTOS. Raw pointer + null check beats `optional<reference_wrapper>` in C++14.

8. **Odometry: float internally, int32_t in protocol** — avoids fixed-point math complexity; protocol output clamped to ±9999 mm and ±18000 centidegrees.

9. **PathFollower copies waypoints** — each follower has `Waypoint _path[32]` (static, 256 bytes). Eliminates lifetime dependency on the caller's buffer after `setPath()`.

10. **`uBit.sleep(tickMs)` not busy-wait** — yields fiber so CODAL radio event handler can run between ticks.

---

## Wire Protocol Compatibility

The sign-prefixed integer protocol is unchanged:
- Arguments: mandatory sign prefix, no spaces (`S+200-150`, `T+200-150+500`)
- Relay mode: `>` prefix inbound, `<` prefix outbound
- Responses: `ACK:S +200 +150`, `SO+1234-0567+090`, `ERR:command`, `LOG:SAFETY_STOP`
- Baud: 115200; Radio: group 10, channel 0, power 7

All existing Python host code (`robot_radio/`) works without modification.

---

## Implementation Sprints (suggested ordering)

**Sprint 1: HAL Layer**
Implement and test all `hal/` classes: NezhaV2, OtosSensor, LineSensor, ColorSensor, GripperServo, PortIO, SerialPort, Radio.

**Sprint 2: Control Layer + main loop skeleton**
Implement MotorController, Odometry. Wire up Robot::init()/run() and Announcer. Verify serial announcement and HELLO response.

**Sprint 3: CommandProcessor — motion and calibration commands**
Implement drive mode state machine (S/T/D/X), watchdog, PI tick, and all K* calibration commands. Verify motor control and encoder streaming match TypeScript behavior.

**Sprint 4: CommandProcessor — sensor and odometry commands**
Implement SO/SZ/SI/ENC/EZ, OTOS commands (O/OI/OK/OZ/OR/OP/OV/OL/OA), LS/CS/G/P/PA.

**Sprint 5: Navigation layer**
Implement PoseProvider hierarchy and PathFollower hierarchy (PurePursuit, Stanley).

---

## Verification

- Build: `python build.py` (Docker-based CODAL build, produces `.hex`)
- Deploy: `python scripts/deploy.py` (copies `.hex` to micro:bit mount)
- Protocol test: use existing `robot_radio/qbot_pro.py` to send commands and verify responses
- Motor test: `S+100-100` → robot drives forward; `S+100-100` to `X` → stops
- Encoder streaming: verify ENC output at correct cadence
- OTOS: `OI` → init, `OP` → pose (x/y/h should be ~0 at start)
- Path following: use existing `robot_radio/navigator.py` or manual waypoint injection

---

## Critical Files for Reference During Implementation

- TypeScript source: `/Volumes/Proj/proj/league-projects/scratch/radio-robot/src/command.ts` — command handlers (893 lines, exact logic to port)
- TypeScript source: `/Volumes/Proj/proj/league-projects/scratch/radio-robot/src/nezha.ts` — motor/encoder calibration constants and sign conventions
- TypeScript source: `/Volumes/Proj/proj/league-projects/scratch/radio-robot/src/otos.ts` — I2C register map, LSB conversion factors
- Python nav: `/Volumes/Proj/proj/league-projects/scratch/radio-robot/robot_radio/pure_pursuit.py`
- Python nav: `/Volumes/Proj/proj/league-projects/scratch/radio-robot/robot_radio/stanley.py`
- Python nav: `/Volumes/Proj/proj/league-projects/scratch/radio-robot/robot_radio/controllers.py`
- Existing entry point to replace: `/Volumes/Proj/proj/RobotProjects/radio-robot-c/source/main.cpp`
