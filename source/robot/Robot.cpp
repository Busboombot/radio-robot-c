#include "Robot.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "MotionController.h"
#include "MicroBit.h"
#include <cstdio>
#include <cmath>

// ---------------------------------------------------------------------------
// Constructor — initializer list must match member declaration order.
//
// Declaration order (from Robot.h):
//   config, state, motorL, motorR, otos, line, colorSensor, gripper, portio,
//   motorController, odometry, motionController, portController, servoController
//
// Two post-construction binds:
//   motionController.setHardwareState(&state.inputs)  — MotionController reads pose
//   motorController.setCommandsRef(&state.commands)  — MotorController writes tgt*/pwm*
// ---------------------------------------------------------------------------

Robot::Robot(Motor& mL, Motor& mR, OtosSensor& o, LineSensor& l,
                       ColorSensor& c, Servo& g, PortIO& p,
                       const RobotConfig& cfg)
    : config(cfg),
      state(defaultInputs(cfg)),
      motorL(mL), motorR(mR),
      otos(o), line(l), colorSensor(c), gripper(g), portio(p),
      motorController(motorL, motorR, config),
      odometry(),
      motionController(motorController, odometry, config),
      portController(portio),
      servoController(gripper)
{
    motionController.setHardwareState(&state.inputs);
    motorController.setCommandsRef(&state.commands);
}

// ---------------------------------------------------------------------------
// systemTime — robot system time in milliseconds since boot.
// ---------------------------------------------------------------------------

uint32_t Robot::systemTime() const
{
    return (uint32_t)system_timer_current_time();
}

// ---------------------------------------------------------------------------
// controlCollectSplitPhase — split-phase COLLECT for the cooperative loop.
//
// Reads both encoders, applies the speed-scaled outlier filter, writes
// state.inputs.enc{L,R}Mm, then calls motorController.controlTick() for PID+PWM.
//
// Migrated from the original Robot controlCollectSplitPhase with mechanical
// member-name substitutions (_state → state, _mc → motorController,
// _motorL → motorL, _motorR → motorR, _config → config).
// ---------------------------------------------------------------------------

void Robot::controlCollectSplitPhase(uint32_t now_ms, int /*pendingWheel*/)
{
    // WedgeTest-proven pattern (sprint 015): read BOTH encoders every tick,
    // right motor (M1) first, then left (M2). Write-on-change is already
    // handled by Motor::setSpeed(). Single re-read on implausible delta.
    //
    // Cost: ~8 ms (2 × 4 ms post-write settle). controlPeriodMs must be ≥ 10 ms.
    //
    // Previous alternating-one-per-tick design (~5 Hz per wheel) wedged within
    // ~165 ticks: each wedge caused the velocity PID to saturate and jerk.
    // WedgeTest ran 10 min / 165 cycles with ZERO wedges using this pattern.
    bool driving = (state.commands.tgtLMms != 0.0f ||
                    state.commands.tgtRMms != 0.0f);
    if (driving) {
        // Outlier threshold SCALES with commanded speed. A legit tick can't move
        // much more than (target speed × a worst-case ~200 ms scheduler tick), so
        // the gate is max(40 mm floor, |target mm/s| × 0.2). A bad read triggers up
        // to kRetries re-reads; if any is sane → use it; if ALL fail → hold the old
        // stored value so the outlier baseline stays correct next tick.
        //
        // Why scaled, not a fixed 150 mm: at slow calibration speeds (~80 mm/s) a
        // legit tick is <10 mm, but the chip still occasionally returns ~149 mm
        // garbage reads — which slipped UNDER a fixed 150 mm gate, fed the velocity
        // loop a huge spurious velocity, and spasmed the motor. Scaling keeps the
        // gate tight when slow (rejects those) and wide when fast (~80 mm at
        // 400 mm/s) so normal fast driving isn't tripped.
        const float kMaxDeltaMm = fmaxf(40.0f,
            fmaxf(fabsf((float)state.commands.tgtLMms),
                  fabsf((float)state.commands.tgtRMms)) * 0.2f);
        static constexpr int kRetries = 2;

        // Right (M1) first — proven ordering from WedgeTest.
        {
            float newR = motorR.readEncoderMmFSettle(config);
            float dR   = newR - state.inputs.encRMm;
            if (dR > kMaxDeltaMm || dR < -kMaxDeltaMm) {
                newR = state.inputs.encRMm;             // default: hold old
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = motorR.readEncoderMmFSettle(config);
                    float dr2 = r2 - state.inputs.encRMm;
                    if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newR = r2; break; }
                }
            }
            state.inputs.encRMm = newR;
        }

        // Left (M2) second.
        {
            float newL = motorL.readEncoderMmFSettle(config);
            float dL   = newL - state.inputs.encLMm;
            if (dL > kMaxDeltaMm || dL < -kMaxDeltaMm) {
                newL = state.inputs.encLMm;             // default: hold old
                for (int k = 0; k < kRetries; ++k) {
                    float r2  = motorL.readEncoderMmFSettle(config);
                    float dr2 = r2 - state.inputs.encLMm;
                    if (dr2 <= kMaxDeltaMm && dr2 >= -kMaxDeltaMm) { newL = r2; break; }
                }
            }
            state.inputs.encLMm = newL;
        }
    }
    _prevDriving = driving;
    _lastControlMs = now_ms;
    // refreshedWheel=3: both wheels updated; 0: idle, no velocity update.
    motorController.controlTick(state.inputs, state.commands, now_ms, driving ? 3 : 0);
}

// ---------------------------------------------------------------------------
// otosCorrect — OTOS complementary correction task entry point.
//
// Uses otos.readTransformed(config) from T001 instead of inlined LSB math.
// The kOtosSlowMs cadence gate is dropped — run_blocks handles that cadence
// via the task table's periodMs (lagOtosMs).
// ---------------------------------------------------------------------------

void Robot::otosCorrect(uint32_t now_ms)
{
    if (!otos.is_initialized()) return;
    OtosPose p = otos.readTransformed(config);
    state.inputs.otosX = p.x;
    state.inputs.otosY = p.y;
    state.inputs.otosH = p.h;
    state.inputs.otos.lastUpdMs = now_ms;
    state.inputs.otos.valid     = true;
    odometry.correct(state.inputs, p.x, p.y, p.h,
                     config.alphaPos, config.alphaYaw, config.otosGate);
}

// ---------------------------------------------------------------------------
// lineRead — read 4-channel line sensor into HardwareState.
// ---------------------------------------------------------------------------

void Robot::lineRead()
{
    if (!line.is_initialized()) return;
    if (line.readValues(state.inputs.line)) {
        state.inputs.lineVS.lastUpdMs = systemTime();
        state.inputs.lineVS.valid     = true;
    }
}

// ---------------------------------------------------------------------------
// colorRead — non-blocking RGBC poll into HardwareState.
// ---------------------------------------------------------------------------

void Robot::colorRead()
{
    if (!colorSensor.is_initialized()) return;
    if (colorSensor.pollRGBC(state.inputs.colorR,
                              state.inputs.colorG,
                              state.inputs.colorB,
                              state.inputs.colorC)) {
        state.inputs.colorVS.lastUpdMs = systemTime();
        state.inputs.colorVS.valid     = true;
    }
}

// ---------------------------------------------------------------------------
// portsRead — read digital and analogue GPIO ports into HardwareState.
// ---------------------------------------------------------------------------

void Robot::portsRead()
{
    for (uint8_t i = 0; i < 4; ++i) {
        state.inputs.digitalIn[i] = (portio.readDigital(i) != 0);
        state.inputs.analogIn[i]  = (int16_t)portio.readAnalog(i);
    }
    state.inputs.portsVS.lastUpdMs = systemTime();
    state.inputs.portsVS.valid     = true;
}

// ---------------------------------------------------------------------------
// distanceDrive — begin a distance drive and reset encoder outlier baseline.
//
// The encoder-reset workaround: beginDistance resets the MotionController's
// accumulator to 0, but state.inputs.encLMm/R still hold the previous
// drive's final value. The outlier filter compares new reads to those stale
// values; the ~target→0 jump looks like a huge backward outlier and gets
// REJECTED, freezing encLMm/R and corrupting the velocity loop. Zeroing
// them here aligns the filter baseline with the fresh accumulator.
// ---------------------------------------------------------------------------

void Robot::distanceDrive(int32_t l, int32_t r, int32_t targetMm,
                                ReplyFn fn, void* ctx, const char* corr_id)
{
    motionController.beginDistance((float)l, (float)r, targetMm,
                                   systemTime(), state.target, fn, ctx, corr_id);
    state.inputs.encLMm = 0.0f;
    state.inputs.encRMm = 0.0f;
}

// ---------------------------------------------------------------------------
// buildTlmFrame — assemble the unified TLM frame; returns length.
//
// Reads state.inputs, config, motionController.mode(). Shared by the periodic
// STREAM (telemetryEmit) and the synchronous SNAP command.
// ---------------------------------------------------------------------------

int Robot::buildTlmFrame(char* buf, int len)
{
    uint32_t t_sample = systemTime();
    int32_t encL = static_cast<int32_t>(state.inputs.encLMm);
    int32_t encR = static_cast<int32_t>(state.inputs.encRMm);

    int32_t pose_x = 0, pose_y = 0, pose_h = 0;
    if (config.tlmFields & TLM_FIELD_POSE) {
        Odometry::getPose(state.inputs, pose_x, pose_y, pose_h);
    }
    bool haveLine = line.is_initialized() && state.inputs.lineVS.valid &&
                    (config.tlmFields & TLM_FIELD_LINE);
    bool haveColor = colorSensor.is_initialized() && state.inputs.colorVS.valid &&
                     (config.tlmFields & TLM_FIELD_COLOR);
    bool haveVel = (config.tlmFields & TLM_FIELD_VEL) != 0;
    float velL = haveVel ? state.inputs.velLMms : 0.0f;
    float velR = haveVel ? state.inputs.velRMms : 0.0f;

    char modeChar = 'I';
    switch (motionController.mode()) {
        case DriveMode::STREAMING: modeChar = 'S'; break;
        case DriveMode::TIMED:     modeChar = 'T'; break;
        case DriveMode::DISTANCE:  modeChar = 'D'; break;
        case DriveMode::GO_TO:     modeChar = 'G'; break;
        case DriveMode::VELOCITY:  modeChar = 'V'; break;
        default:                   modeChar = 'I'; break;
    }

    int pos = 0, rem = len;
    int n = snprintf(buf + pos, (size_t)rem, "TLM t=%lu mode=%c",
                     (unsigned long)t_sample, modeChar);
    if (n > 0 && n < rem) { pos += n; rem -= n; }
    if (config.tlmFields & TLM_FIELD_ENC) {
        n = snprintf(buf + pos, (size_t)rem, " enc=%d,%d", (int)encL, (int)encR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (config.tlmFields & TLM_FIELD_POSE) {
        n = snprintf(buf + pos, (size_t)rem, " pose=%d,%d,%d",
                     (int)pose_x, (int)pose_y, (int)pose_h);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveVel) {
        n = snprintf(buf + pos, (size_t)rem, " vel=%d,%d", (int)velL, (int)velR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveLine) {
        n = snprintf(buf + pos, (size_t)rem, " line=%u,%u,%u,%u",
                     (unsigned)state.inputs.line[0], (unsigned)state.inputs.line[1],
                     (unsigned)state.inputs.line[2], (unsigned)state.inputs.line[3]);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveColor) {
        n = snprintf(buf + pos, (size_t)rem, " color=%u,%u,%u,%u",
                     (unsigned)state.inputs.colorR, (unsigned)state.inputs.colorG,
                     (unsigned)state.inputs.colorB, (unsigned)state.inputs.colorC);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    buf[pos] = '\0';
    return pos;
}

// ---------------------------------------------------------------------------
// telemetryEmit — gate and emit the periodic TLM frame.
//
// Emits only while driving (+ a short grace period). When idle, the stream
// goes silent so the radio link is clear for commands. SNAP handles the
// synchronous request path.
// ---------------------------------------------------------------------------

void Robot::telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    static constexpr uint32_t kGraceMs = 400;
    if (motionController.mode() != DriveMode::IDLE) _lastActiveMs = now_ms;
    bool stopped = (now_ms - _lastActiveMs) > kGraceMs;

    bool periodic = (config.tlmPeriodMs > 0) && !stopped &&
                    ((now_ms - _lastTlmMs) >= (uint32_t)config.tlmPeriodMs);
    if (!periodic) return;

    if (config.tlmPeriodMs < 20) config.tlmPeriodMs = 20;  // clamp to 50 Hz max

    char tlmBuf[128];
    buildTlmFrame(tlmBuf, sizeof(tlmBuf));
    fn(tlmBuf, ctx);
    _lastTlmMs = now_ms;
}
