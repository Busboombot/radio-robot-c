#include "Robot.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "MotionController.h"
#include "MicroBit.h"
#include "MicroBitDevice.h"
#include "Odometry.h"
#include "DebugCommandable.h"
#include "LoopScheduler.h"
#include "Communicator.h"
#include "Radio.h"
#include "RadioChannel.h"
#include "CommandProcessor.h"
#include "ConfigRegistry.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cassert>

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
    motionController.setCtx(this);
    odometry.setCtx(&otos);
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

// ===========================================================================
// buildCommandTable — system command handlers + aggregation
//
// All system command handlers are static functions defined here.
// handlerCtx is always RobotSysCtx* (cast inside each handler).
// ===========================================================================

// ---------------------------------------------------------------------------
// Internal accessor — cast handlerCtx to RobotSysCtx*.
// ---------------------------------------------------------------------------
namespace {

static RobotSysCtx& ctxFrom(void* p)
{
    return *reinterpret_cast<RobotSysCtx*>(p);
}

// ---------------------------------------------------------------------------
// HELLO — raw DEVICE banner (no OK wrapper).
//   prefix "HELLO"; parseFn nullptr; no args.
//   Output: DEVICE:NEZHA2:robot:<name>:<serial>
// ---------------------------------------------------------------------------

static ParseResult parseHello(const char* const* /*tokens*/, int /*ntokens*/,
                               const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleHello(const ArgList& /*args*/, const char* /*corrId*/,
                         ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    (void)handlerCtx;
    const char* name   = microbit_friendly_name();
    uint32_t    serial = microbit_serial_number();
    char banner[64];
    snprintf(banner, sizeof(banner),
             "DEVICE:NEZHA2:robot:%s:%lu", name, (unsigned long)serial);
    replyFn(banner, replyCtx);
}

// ---------------------------------------------------------------------------
// PING — clock-sync probe.
//   prefix "PING"; parseFn nullptr.
//   Reply: OK pong t=<ms>
// ---------------------------------------------------------------------------

static ParseResult parsePing(const char* const* /*tokens*/, int /*ntokens*/,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handlePing(const ArgList& /*args*/, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    uint32_t t = robot->systemTime();
    char rbuf[64];
    char body[32];
    snprintf(body, sizeof(body), "t=%lu", (unsigned long)t);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "pong", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// ECHO — echo payload tokens back.
//   prefix "ECHO"; parseFn stores tokens as STR args.
//   Reply: OK echo <joined tokens>
// ---------------------------------------------------------------------------

static ParseResult parseEcho(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    // Store each token as a STR arg; handler reassembles them.
    ParseResult r;
    r.ok = true;
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.args.count = n;
    for (int i = 0; i < n; ++i) {
        r.args.args[i].type = ArgType::STR;
        r.args.args[i].ival = 0;
        r.args.args[i].fval = 0.0f;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(r.args.args[i].sval) - 1; ++j)
            r.args.args[i].sval[j] = tokens[i][j];
        r.args.args[i].sval[j] = '\0';
    }
    return r;
}

static void handleEcho(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    (void)handlerCtx;
    // Reassemble tokens into a single space-joined payload.
    char payload[512];
    int pos = 0;
    for (int i = 0; i < args.count && pos < (int)sizeof(payload) - 2; ++i) {
        if (i > 0) payload[pos++] = ' ';
        for (const char* c = args.args[i].sval;
             *c != '\0' && pos < (int)sizeof(payload) - 1; ++c)
            payload[pos++] = *c;
    }
    payload[pos] = '\0';

    char rbuf[520];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "echo", payload, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// ID — full identification response.
//   prefix "ID"; parseFn nullptr.
//   Reply: ID model=Nezha2 name=<n> serial=<s> fw=<ver> proto=2 caps=<c>
// ---------------------------------------------------------------------------

static ParseResult parseId(const char* const* /*tokens*/, int /*ntokens*/,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleId(const ArgList& /*args*/, const char* corrId,
                      ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot  = ctxFrom(handlerCtx).robot;
    const char* name   = microbit_friendly_name();
    uint32_t    serial = microbit_serial_number();

    char caps[64];
    caps[0] = '\0';
    bool first = true;
    auto addCap = [&](const char* cap) {
        if (!first) {
            int n = (int)strlen(caps);
            caps[n] = ','; caps[n+1] = '\0';
        }
        int rem = (int)(sizeof(caps) - strlen(caps) - 1);
        if (rem > 0) strncat(caps, cap, (size_t)rem);
        first = false;
    };
    if (robot->otos.is_initialized())        addCap("otos");
    if (robot->line.is_initialized())        addCap("line");
    if (robot->colorSensor.is_initialized()) addCap("color");
    addCap("portio");

    char rbuf[520];
    if (corrId && corrId[0] != '\0') {
        snprintf(rbuf, sizeof(rbuf),
                 "ID model=Nezha2 name=%s serial=%lu fw=%s proto=%d caps=%s #%s",
                 name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION,
                 caps, corrId);
    } else {
        snprintf(rbuf, sizeof(rbuf),
                 "ID model=Nezha2 name=%s serial=%lu fw=%s proto=%d caps=%s",
                 name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION,
                 caps);
    }
    replyFn(rbuf, replyCtx);
}

// ---------------------------------------------------------------------------
// VER — firmware/protocol version query.
//   prefix "VER"; parseFn nullptr.
//   Reply: OK ver fw=<ver> proto=2
// ---------------------------------------------------------------------------

static ParseResult parseVer(const char* const* /*tokens*/, int /*ntokens*/,
                             const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleVer(const ArgList& /*args*/, const char* corrId,
                       ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/)
{
    char rbuf[64];
    char body[64];
    snprintf(body, sizeof(body), "fw=%s proto=%d", FIRMWARE_VERSION, PROTO_VERSION);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "ver", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// HELP — list all verbs.
//   prefix "HELP"; parseFn nullptr.
//   Reply: OK help <verb list>
// ---------------------------------------------------------------------------

static ParseResult parseHelp(const char* const* /*tokens*/, int /*ntokens*/,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleHelp(const ArgList& /*args*/, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/)
{
    char rbuf[520];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "help",
        "PING ECHO ID VER HELP SET GET GET VEL STREAM SNAP "
        "S T D G R TURN VW RF X STOP GRIP ZERO "
        "OI OZ OR OP OV OL OA P PA "
        "[sensor=<ch>:<op>:<thr>]",
        corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// SNAP — synchronous telemetry frame.
//   prefix "SNAP"; parseFn nullptr.
//   Reply: TLM ... (raw frame, not OK-wrapped)
// ---------------------------------------------------------------------------

static ParseResult parseSnap(const char* const* /*tokens*/, int /*ntokens*/,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleSnap(const ArgList& /*args*/, const char* /*corrId*/,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    char tlmBuf[128];
    robot->buildTlmFrame(tlmBuf, sizeof(tlmBuf));
    replyFn(tlmBuf, replyCtx);
}

// ---------------------------------------------------------------------------
// ZERO — zero encoders and/or odometry.
//   prefix "ZERO"; parseFn passes "enc"/"pose" token args.
//   Reply: OK zero <enc|pose|enc pose>
// ---------------------------------------------------------------------------

static ParseResult parseZero(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    if (ntokens < 1) {
        r.ok = false;
        r.err = { "badarg", nullptr };
        return r;
    }
    // Check that at least one valid arg is present.
    bool hasEnc  = false;
    bool hasPose = false;
    for (int i = 0; i < ntokens; ++i) {
        if (strcmp(tokens[i], "enc")  == 0) hasEnc  = true;
        if (strcmp(tokens[i], "pose") == 0) hasPose = true;
    }
    if (!hasEnc && !hasPose) {
        r.ok = false;
        r.err = { "badarg", nullptr };
        return r;
    }
    // Pass tokens as STR args.
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.ok = true;
    r.args.count = n;
    for (int i = 0; i < n; ++i) {
        r.args.args[i].type = ArgType::STR;
        r.args.args[i].ival = 0;
        r.args.args[i].fval = 0.0f;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(r.args.args[i].sval) - 1; ++j)
            r.args.args[i].sval[j] = tokens[i][j];
        r.args.args[i].sval[j] = '\0';
    }
    return r;
}

static void handleZero(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    bool doEnc  = false;
    bool doPose = false;
    for (int i = 0; i < args.count; ++i) {
        if (strcmp(args.args[i].sval, "enc")  == 0) doEnc  = true;
        if (strcmp(args.args[i].sval, "pose") == 0) doPose = true;
    }
    if (doEnc)  robot->motorController.resetEncoderAccumulators();
    if (doPose) robot->odometry.zero(robot->state.inputs);

    char rbuf[64];
    char body[16];
    if (doEnc && doPose)       snprintf(body, sizeof(body), "enc pose");
    else if (doEnc)            snprintf(body, sizeof(body), "enc");
    else                       snprintf(body, sizeof(body), "pose");
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "zero", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// STREAM — configure telemetry stream period and/or field mask.
//   prefix "STREAM"; parseFn passes period int or fields= string.
//   Reply: OK stream period=<ms> | OK stream fields=<csv>
// ---------------------------------------------------------------------------

static ParseResult parseStream(const char* const* tokens, int ntokens,
                                const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    r.ok = true;
    // Pass tokens as raw STR args.
    // "STREAM <ms>" → args[0].sval = "<ms>"  (parsed as int by handler)
    // "STREAM fields=enc,pose" → args[0].sval = "fields=enc,pose"  (handler checks prefix)
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.args.count = n;
    for (int i = 0; i < n; ++i) {
        r.args.args[i].type = ArgType::STR;
        r.args.args[i].ival = 0;
        r.args.args[i].fval = 0.0f;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(r.args.args[i].sval) - 1; ++j)
            r.args.args[i].sval[j] = tokens[i][j];
        r.args.args[i].sval[j] = '\0';
    }
    return r;
}

static void handleStream(const ArgList& args, const char* corrId,
                          ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    char rbuf[520];

    // Scan args for a "fields=..." entry.
    for (int i = 0; i < args.count; ++i) {
        const char* sv = args.args[i].sval;
        if (strncmp(sv, "fields=", 7) == 0) {
            const char* fp = sv + 7;
            uint8_t mask = 0;
            char fbuf[64];
            int flen = 0;
            for (const char* c = fp; ; ++c) {
                bool end = (*c == '\0' || *c == ',');
                if (!end && flen < (int)(sizeof(fbuf) - 1))
                    fbuf[flen++] = *c;
                if (end) {
                    fbuf[flen] = '\0';
                    if (strcmp(fbuf, "enc")   == 0) mask |= TLM_FIELD_ENC;
                    if (strcmp(fbuf, "pose")  == 0) mask |= TLM_FIELD_POSE;
                    if (strcmp(fbuf, "vel")   == 0) mask |= TLM_FIELD_VEL;
                    if (strcmp(fbuf, "line")  == 0) mask |= TLM_FIELD_LINE;
                    if (strcmp(fbuf, "color") == 0) mask |= TLM_FIELD_COLOR;
                    flen = 0;
                    if (*c == '\0') break;
                }
            }
            robot->config.tlmFields = mask ? mask : TLM_FIELD_ALL;

            // Reconstruct the fields string for the response body.
            char body[80];
            int bpos = 0;
            bool needComma = false;
            const struct { uint8_t bit; const char* name; } kFieldNames[] = {
                { TLM_FIELD_ENC,   "enc"   },
                { TLM_FIELD_POSE,  "pose"  },
                { TLM_FIELD_VEL,   "vel"   },
                { TLM_FIELD_LINE,  "line"  },
                { TLM_FIELD_COLOR, "color" },
            };
            int brem = (int)sizeof(body);
            int bw = snprintf(body + bpos, (size_t)brem, "fields=");
            if (bw > 0 && bw < brem) { bpos += bw; brem -= bw; }
            for (int fi = 0; fi < 5 && brem > 1; ++fi) {
                if (robot->config.tlmFields & kFieldNames[fi].bit) {
                    if (needComma) { body[bpos++] = ','; --brem; }
                    bw = snprintf(body + bpos, (size_t)brem, "%s", kFieldNames[fi].name);
                    if (bw > 0 && bw < brem) { bpos += bw; brem -= bw; }
                    needComma = true;
                }
            }
            body[bpos] = '\0';
            CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stream", body,
                                      corrId, replyFn, replyCtx);
            return;
        }
    }

    // No fields= — expect a positional period arg.
    if (args.count < 1) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "usage: STREAM <ms>",
                                   corrId, replyFn, replyCtx);
        return;
    }
    int32_t ms = (int32_t)atoi(args.args[0].sval);
    if (ms < 0) ms = 0;
    if (ms > 0 && ms < 20) ms = 20;
    robot->config.tlmPeriodMs = ms;
    char body[32];
    snprintf(body, sizeof(body), "period=%d", (int)ms);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stream", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// RF — radio channel get/set.
//   prefix "RF"; parseFn passes optional channel as INT arg.
//   Reply: OK rf chan=<n> group=10
// ---------------------------------------------------------------------------

static ParseResult parseRf(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    r.ok = true;
    if (ntokens >= 1) {
        r.args.count = 1;
        r.args.args[0].type = ArgType::INT;
        r.args.args[0].ival = atoi(tokens[0]);
        r.args.args[0].fval = 0.0f;
        r.args.args[0].sval[0] = '\0';
    } else {
        r.args.count = 0;
    }
    return r;
}

static void handleRf(const ArgList& args, const char* corrId,
                      ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    LoopScheduler* sched = ctxFrom(handlerCtx).sched;
    char rbuf[64];
    if (sched == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noradio", nullptr,
                                   corrId, replyFn, replyCtx);
        return;
    }
    Radio& radio = sched->comm().radio();

    if (args.count < 1) {
        // Query.
        char body[32];
        snprintf(body, sizeof(body), "chan=%d group=%d",
                 radio.channel(), radiochan::kGroup);
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rf", body,
                                  corrId, replyFn, replyCtx);
        return;
    }

    int ch = args.args[0].ival;
    if (ch < radiochan::kMin || ch > radiochan::kMax) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "range", "chan",
                                   corrId, replyFn, replyCtx);
        return;
    }
    // Persist first, then reply on the OLD channel, then re-tune.
    radiochan::save(sched->uBit().storage, ch);
    char body[32];
    snprintf(body, sizeof(body), "chan=%d group=%d", ch, radiochan::kGroup);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rf", body,
                              corrId, replyFn, replyCtx);
    radio.setChannel(ch);
}

// ---------------------------------------------------------------------------
// GET VEL — per-wheel velocity readout (separate descriptor from GET).
//   prefix "GET VEL"; parseFn nullptr.
//   Reply: OK get vel=<vL>:E,<vR>:E
// ---------------------------------------------------------------------------

static ParseResult parseGetVel(const char* const* /*tokens*/, int /*ntokens*/,
                                const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r; r.ok = true; r.args.count = 0; return r;
}

static void handleGetVel(const ArgList& /*args*/, const char* corrId,
                          ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    Robot* robot = ctxFrom(handlerCtx).robot;
    float vL = robot->state.inputs.velLMms;
    float vR = robot->state.inputs.velRMms;
    char rbuf[64];
    char body[48];
    snprintf(body, sizeof(body), "vel=%d:E,%d:E", (int)vL, (int)vR);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "get", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// parseGet — convert positional key-name tokens into STR args for handleGet.
//   Each token becomes args[i].sval = key name.
// ---------------------------------------------------------------------------

static ParseResult parseGet(const char* const* tokens, int ntokens,
                             const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult r;
    r.ok = true;
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    r.args.count = n;
    for (int i = 0; i < n; ++i) {
        r.args.args[i].type = ArgType::STR;
        r.args.args[i].ival = 0;
        r.args.args[i].fval = 0.0f;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(r.args.args[i].sval) - 1; ++j)
            r.args.args[i].sval[j] = tokens[i][j];
        r.args.args[i].sval[j] = '\0';
    }
    return r;
}

// ---------------------------------------------------------------------------
// parseSet — convert kv pairs into "key=value" STR args for handleSet.
// ---------------------------------------------------------------------------

static ParseResult parseSet(const char* const* /*tokens*/, int /*ntokens*/,
                             const KVPair* kvs, int nkv)
{
    ParseResult r;
    if (nkv == 0) {
        r.ok = false;
        r.err = { "badarg", "no key=value pairs" };
        return r;
    }
    r.ok = true;
    int n = (nkv > MAX_ARGS) ? MAX_ARGS : nkv;
    r.args.count = 0;
    for (int i = 0; i < n; ++i) {
        if (!kvs[i].key) continue;
        char* dst = r.args.args[r.args.count].sval;
        int cap = (int)(sizeof(r.args.args[0].sval) - 1);
        int written = snprintf(dst, (size_t)(cap + 1), "%s=%s",
                               kvs[i].key, kvs[i].value);
        if (written > cap) dst[cap] = '\0';
        r.args.args[r.args.count].type = ArgType::STR;
        r.args.args[r.args.count].ival = 0;
        r.args.args[r.args.count].fval = 0.0f;
        ++r.args.count;
    }
    return r;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Robot::buildCommandTable — aggregate all Commandables + system commands.
// ---------------------------------------------------------------------------

int Robot::buildCommandTable(CommandDescriptor* buf, int max,
                             DebugCommandable* dbg,
                             LoopScheduler*    sched) const
{
    // Populate stable context structs (members, so pointers are valid for the
    // lifetime of this Robot).
    _cfgCtx       = { const_cast<RobotConfig*>(&config),
                      const_cast<MotorController*>(&motorController) };
    _sysCtx.robot = const_cast<Robot*>(this);
    _sysCtx.sched = sched;

    void* sysCtxPtr = &_sysCtx;

    int n = 0;

    // ---- Commandable members ----
    n += motionController.getCommands(buf + n, max - n);
    n += odometry.getCommands(buf + n, max - n);
    n += portController.getCommands(buf + n, max - n);
    n += servoController.getCommands(buf + n, max - n);
    if (dbg) n += dbg->getCommands(buf + n, max - n);

    // ---- System commands ----
    // Order: most-specific prefix first (GET VEL before GET).
    if (n < max) buf[n++] = makeCmd("HELLO",   parseHello,  handleHello,  sysCtxPtr, "badarg");
    if (n < max) buf[n++] = makeCmd("PING",    parsePing,   handlePing,   sysCtxPtr, "badarg");
    if (n < max) buf[n++] = makeCmd("ECHO",    parseEcho,   handleEcho,   sysCtxPtr, "badarg");
    if (n < max) buf[n++] = makeCmd("ID",      parseId,     handleId,     sysCtxPtr, "badarg");
    if (n < max) buf[n++] = makeCmd("VER",     parseVer,    handleVer,    sysCtxPtr, "badarg");
    if (n < max) buf[n++] = makeCmd("HELP",    parseHelp,   handleHelp,   sysCtxPtr, "badarg");
    if (n < max) buf[n++] = makeCmd("SNAP",    parseSnap,   handleSnap,   sysCtxPtr, "badarg");
    if (n < max) buf[n++] = makeCmd("ZERO",    parseZero,   handleZero,   sysCtxPtr, "badarg");
    if (n < max) buf[n++] = makeCmd("STREAM",  parseStream, handleStream, sysCtxPtr, "badarg");
    if (n < max) buf[n++] = makeCmd("RF",      parseRf,     handleRf,     sysCtxPtr, "badarg");
    if (n < max) buf[n++] = makeCmd("GET VEL", parseGetVel, handleGetVel, sysCtxPtr, "badarg");
    if (n < max) buf[n++] = makeCmd("GET",     parseGet,    handleGet,    &_cfgCtx,  "badkey");
    if (n < max) buf[n++] = makeCmd("SET",     parseSet,    handleSet,    &_cfgCtx,  "badkey");

    // Safety assertion: buffer must not be exceeded.
    assert(n <= max);

    return n;
}
