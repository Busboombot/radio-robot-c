// CommandProcessor.cpp — protocol v2 wire-protocol parser and dispatcher.
//
// Sprint 009, Ticket 002: v2 tokenizer, verb-only uppercasing, #id
// correlation, OK/ERR/EVT/TLM/CFG/ID response taxonomy.
// Legacy packed parsing (parseSignedArgs, K*, S+/T+/D+, etc.) removed.
//
// OOP change: HELLO restored as a raw DEVICE: identification banner
// (NEZHA2 role, not RADIOBRIDGE/RELAY) so mbdeploy probe_type() can
// identify this robot. Banner is also emitted once at boot via main.cpp.
//
// Sprint 009, Ticket 004: SET/GET named-key config registry.
// Static kRegistry[] maps friendly key names to RobotConfig fields.

#include "CommandProcessor.h"
#include "Robot.h"
#include "MicroBitDevice.h"
#include "OtosSensor.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "Servo.h"
#include "PortIO.h"
#include "MotorController.h"
#include "Odometry.h"
#include "WedgeTest.h"
#include "DriveController.h"
#include "LoopScheduler.h"
#include "Communicator.h"
#include "RadioChannel.h"
#include "LoopScheduler.h"
#include "I2CBus.h"
#include "Config.h"
#include "StopCondition.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>

// ---------------------------------------------------------------------------
// Config registry — maps friendly key names to RobotConfig field offsets.
// ---------------------------------------------------------------------------

enum ConfigFieldType {
    CFG_FLOAT,        // float field, wire format: %.3f
    CFG_INT,          // int32_t field, wire format: %d
    CFG_FLOAT_AS_INT  // float field stored as integer magnitude, wire format: %d
};

struct ConfigEntry {
    const char*    key;
    ConfigFieldType type;
    size_t         offset;  // offsetof(RobotConfig, field)
};

// Helper macros so the table stays readable.
#define CFG_F(k, field)  { k, CFG_FLOAT,        offsetof(RobotConfig, field) }
#define CFG_I(k, field)  { k, CFG_INT,           offsetof(RobotConfig, field) }
#define CFG_FI(k, field) { k, CFG_FLOAT_AS_INT,  offsetof(RobotConfig, field) }

static const ConfigEntry kRegistry[] = {
    // Encoder calibration (mm per degree of motor rotation)
    CFG_F("ml",           mmPerDegL),
    CFG_F("mr",           mmPerDegR),
    // Feed-forward and motor scale factors
    CFG_F("kff",          kFF),
    CFG_F("klf",          kScaleLF),
    CFG_F("klb",          kScaleLB),
    CFG_F("krf",          kScaleRF),
    CFG_F("krb",          kScaleRB),
    // Slower-wheel adjustment
    CFG_F("adjThr",       kAdjThreshold),
    CFG_F("adjGain",      kAdjGain),
    // Geometry — stored as float, displayed as integer (mm)
    CFG_FI("tw",          trackwidthMm),
    // Ratio PID gains
    CFG_F("pid.kp",       ratioPidKp),
    CFG_F("pid.ki",       ratioPidKi),
    CFG_F("pid.kd",       ratioPidKd),
    CFG_F("pid.max",      ratioPidMax),
    // Velocity loop gains (Sprint 010).
    // C++ field names use flat camel-case; SET/GET key strings use dotted form.
    //   velKp  ↔ "vel.kP"   velKi  ↔ "vel.kI"   velKff ↔ "vel.kFF"
    CFG_F("vel.kP",       velKp),
    CFG_F("vel.kI",       velKi),
    CFG_F("vel.kFF",      velKff),
    CFG_F("vel.iMax",     velIMax),        // integrator clamp (PWM%)
    CFG_F("vel.kAw",      velKaw),         // back-calc anti-windup gain (1/s)
    CFG_F("vel.filt",     velFiltAlpha),   // velocity EMA weight (smoothing)
    CFG_F("sync",         syncGain),       // cross-wheel ratio coupling gain
    // Velocity deadband and wheel speed ceiling (Sprint 010)
    CFG_F("minWheelMms",  minWheelMms),
    CFG_F("vWheelMax",    vWheelMax),
    CFG_F("steerHeadroom",steerHeadroom),
    // OTOS complementary fusion (Sprint 010, Ticket 006)
    CFG_F("alphaPos",     alphaPos),
    CFG_F("alphaYaw",     alphaYaw),
    CFG_F("otosGate",     otosGate),
    // Go-to tolerances — stored as float, displayed as integer (mm)
    // Legacy keys retained for backward compatibility.
    CFG_FI("turnThr",     turnThresholdMm),
    CFG_FI("doneTol",     doneTolMm),
    // Pose-control tunables (Sprint 011)
    CFG_F ("aMax",        aMax),
    CFG_F ("aDecel",      aDecel),
    CFG_FI("turnGate",    turnInPlaceGate),   // wire: integer degrees; DriveController converts to radians at use-site
    CFG_FI("arriveTol",   arriveTolMm),       // wire: integer mm
    // Body motion limits (Sprint 017 — BodyVelocityController)
    CFG_F("vBodyMax",    vBodyMax),           // body forward speed ceiling, mm/s
    CFG_F("yawRateMax",  yawRateMax),         // yaw rate ceiling, deg/s
    CFG_F("yawAccMax",   yawAccMax),          // yaw acceleration limit, deg/s²
    CFG_F("jMax",        jMax),               // linear jerk limit, mm/s³ (0=trapezoid)
    CFG_F("yawJerkMax",  yawJerkMax),         // yaw jerk limit, deg/s³   (0=trapezoid)
    // Command scaling
    CFG_F("distScale",    distScale),
    CFG_F("turnScale",    turnScale),
    // Timing and speed (int32_t fields)
    CFG_I("minSpeed",     minSpeedMms),
    CFG_I("sTimeout",     sTimeoutMs),
    CFG_I("tick",         tickMs),
    CFG_I("ctrlPeriod",   controlPeriodMs),
    CFG_I("tlmPeriod",    tlmPeriodMs),
    // Sensor lag budgets (ms) for the cooperative scheduler's low-priority tasks.
    // SET lag.* N updates cfg.lag*Ms; LoopScheduler syncs task periodMs live.
    CFG_I("lag.otos",     lagOtosMs),
    CFG_I("lag.line",     lagLineMs),
    CFG_I("lag.color",    lagColorMs),
    CFG_I("lag.ports",    lagPortsMs),
    // OTOS calibration and turn asymmetry (Sprint 012)
    CFG_F("otosLinSc",    otosLinearScale),
    CFG_F("otosAngSc",    otosAngularScale),
    CFG_F("rotGainPos",   rotationGainPos),
    CFG_F("rotGainNeg",   rotationGainNeg),
    CFG_F("rotOffPos",    rotationOffsetDeg),
    CFG_F("rotOffNeg",    rotationOffsetDegNeg),
    CFG_F("rotSlip",      rotationalSlip),
    CFG_F("odomOffX",     odomOffX),
    CFG_F("odomOffY",     odomOffY),
    CFG_F("odomYaw",      odomYawDeg),
};

#undef CFG_F
#undef CFG_I
#undef CFG_FI

static constexpr int kRegistryCount = (int)(sizeof(kRegistry) / sizeof(kRegistry[0]));

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommandProcessor::CommandProcessor(Robot& robot)
    : _robot(robot)
{
}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

int CommandProcessor::clampInt(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---------------------------------------------------------------------------
// parseTokens
// ---------------------------------------------------------------------------

int CommandProcessor::parseTokens(const char* line, char* workBuf, int workBufSize,
                                  char** tokens, int maxTokens,
                                  char* corr_id, int corrIdSize)
{
    // Copy line into workBuf, trimming leading/trailing whitespace.
    int srcLen = 0;
    const char* p = line;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;

    for (const char* q = p; *q != '\0' && srcLen < workBufSize - 1; ++q, ++srcLen) {
        workBuf[srcLen] = *q;
    }
    // Trim trailing whitespace.
    while (srcLen > 0 &&
           (workBuf[srcLen - 1] == ' ' || workBuf[srcLen - 1] == '\t' ||
            workBuf[srcLen - 1] == '\r' || workBuf[srcLen - 1] == '\n')) {
        --srcLen;
    }
    workBuf[srcLen] = '\0';

    if (corr_id && corrIdSize > 0) corr_id[0] = '\0';

    if (srcLen == 0) return 0;

    // Tokenize by splitting on whitespace in place.
    int count = 0;
    char* cur  = workBuf;
    char* end  = workBuf + srcLen;

    while (cur < end && count < maxTokens) {
        // Skip leading whitespace.
        while (cur < end && (*cur == ' ' || *cur == '\t')) ++cur;
        if (cur >= end) break;

        tokens[count++] = cur;

        // Advance to next whitespace or end.
        while (cur < end && *cur != ' ' && *cur != '\t') ++cur;
        if (cur < end) {
            *cur = '\0';
            ++cur;
        }
    }

    // Upper-case the verb (tokens[0]).
    if (count > 0) {
        for (char* c = tokens[0]; *c != '\0'; ++c) {
            *c = (char)toupper((unsigned char)*c);
        }
    }

    // Check if the last token is a correlation id: '#' followed by digits only.
    if (count > 0 && corr_id && corrIdSize > 1) {
        const char* last = tokens[count - 1];
        if (last[0] == '#') {
            const char* d = last + 1;
            bool allDigits = (*d != '\0');  // must have at least one digit
            while (*d != '\0' && allDigits) {
                if (*d < '0' || *d > '9') allDigits = false;
                ++d;
            }
            if (allDigits) {
                // Extract digits into corr_id (without the '#').
                int len = (int)(d - (last + 1));
                if (len >= corrIdSize) len = corrIdSize - 1;
                memcpy(corr_id, last + 1, (size_t)len);
                corr_id[len] = '\0';
                --count;  // remove the #id token from the list
            }
        }
    }

    return count;
}

// ---------------------------------------------------------------------------
// parseKV
// ---------------------------------------------------------------------------

int CommandProcessor::parseKV(char** tokens, int ntokens, KVPair* kvs, int maxKV)
{
    int kvCount = 0;
    // Skip tokens[0] (verb) and start from index 1.
    for (int i = 1; i < ntokens && kvCount < maxKV; ++i) {
        char* eq = strchr(tokens[i], '=');
        if (!eq) continue;  // positional arg, not kv

        KVPair kv;
        if (eq == tokens[i]) {
            // '=' at the start: no key.
            kv.key   = nullptr;
            kv.value = eq + 1;
        } else {
            *eq      = '\0';
            kv.key   = tokens[i];
            kv.value = eq + 1;
        }
        kvs[kvCount++] = kv;
    }
    return kvCount;
}

// ---------------------------------------------------------------------------
// Reply builders
// ---------------------------------------------------------------------------

void CommandProcessor::replyOK(char* buf, int size,
                               const char* verb, const char* body, const char* id,
                               ReplyFn fn, void* ctx)
{
    if (body && body[0] != '\0') {
        if (id && id[0] != '\0') {
            snprintf(buf, (size_t)size, "OK %s %s #%s", verb, body, id);
        } else {
            snprintf(buf, (size_t)size, "OK %s %s", verb, body);
        }
    } else {
        if (id && id[0] != '\0') {
            snprintf(buf, (size_t)size, "OK %s #%s", verb, id);
        } else {
            snprintf(buf, (size_t)size, "OK %s", verb);
        }
    }
    fn(buf, ctx);
}

void CommandProcessor::replyErr(char* buf, int size,
                                const char* code, const char* detail, const char* id,
                                ReplyFn fn, void* ctx)
{
    if (detail && detail[0] != '\0') {
        if (id && id[0] != '\0') {
            snprintf(buf, (size_t)size, "ERR %s %s #%s", code, detail, id);
        } else {
            snprintf(buf, (size_t)size, "ERR %s %s", code, detail);
        }
    } else {
        if (id && id[0] != '\0') {
            snprintf(buf, (size_t)size, "ERR %s #%s", code, id);
        } else {
            snprintf(buf, (size_t)size, "ERR %s", code);
        }
    }
    fn(buf, ctx);
}

void CommandProcessor::replyEvt(char* buf, int size,
                                const char* name, const char* body,
                                ReplyFn fn, void* ctx)
{
    if (body && body[0] != '\0') {
        snprintf(buf, (size_t)size, "EVT %s %s", name, body);
    } else {
        snprintf(buf, (size_t)size, "EVT %s", name);
    }
    fn(buf, ctx);
}

// ---------------------------------------------------------------------------
// Registry helpers — append one key=value pair to a string buffer.
// Returns the number of characters written (not counting the NUL).
// ---------------------------------------------------------------------------

static int appendKeyValue(char* buf, int remaining, const ConfigEntry& entry,
                          const RobotConfig& cfg)
{
    if (remaining <= 1) return 0;

    const char* base = reinterpret_cast<const char*>(&cfg);
    int written = 0;

    switch (entry.type) {
    case CFG_FLOAT: {
        const float v = *reinterpret_cast<const float*>(base + entry.offset);
        written = snprintf(buf, (size_t)remaining, "%s=%.3f", entry.key, (double)v);
        break;
    }
    case CFG_INT: {
        const int32_t v = *reinterpret_cast<const int32_t*>(base + entry.offset);
        written = snprintf(buf, (size_t)remaining, "%s=%d", entry.key, (int)v);
        break;
    }
    case CFG_FLOAT_AS_INT: {
        const float v = *reinterpret_cast<const float*>(base + entry.offset);
        written = snprintf(buf, (size_t)remaining, "%s=%d", entry.key, (int)v);
        break;
    }
    }

    if (written < 0 || written >= remaining) return remaining - 1;
    return written;
}

// ---------------------------------------------------------------------------
// handleGet — build CFG response line from named keys (or all keys).
// tokens[1..ntok-1] are the requested keys; empty = all keys.
// ---------------------------------------------------------------------------

static void handleGet(char** tokens, int ntok, const RobotConfig& cfg,
                      char* rbuf, int rbufSize, const char* corr_id,
                      ReplyFn replyFn, void* ctx)
{
    // Build: "CFG key=val key=val ... [#id]"
    // Sprint 012: buffer expanded from 512 to 768 to accommodate 10 new config
    // keys (otosLinSc, otosAngSc, rotGainPos/Neg, rotOffPos/Neg, rotSlip,
    // odomOffX/Y, odomYaw) which add ~156 bytes to the full GET dump.
    // Stack-local buffer; no heap impact.
    char line[768];
    int pos = 0;
    int rem = (int)sizeof(line);

    // Write the "CFG " prefix.
    int n = snprintf(line + pos, (size_t)rem, "CFG");
    if (n > 0 && n < rem) { pos += n; rem -= n; }

    bool anyKey = (ntok <= 1);  // no args → dump all

    if (anyKey) {
        // Dump all registry entries.
        for (int i = 0; i < kRegistryCount && rem > 2; ++i) {
            line[pos++] = ' '; --rem;
            int w = appendKeyValue(line + pos, rem, kRegistry[i], cfg);
            pos += w; rem -= w;
        }
    } else {
        // Dump only the requested keys.
        for (int t = 1; t < ntok && rem > 2; ++t) {
            const char* reqKey = tokens[t];
            bool found = false;
            for (int i = 0; i < kRegistryCount; ++i) {
                if (strcmp(kRegistry[i].key, reqKey) == 0) {
                    line[pos++] = ' '; --rem;
                    int w = appendKeyValue(line + pos, rem, kRegistry[i], cfg);
                    pos += w; rem -= w;
                    found = true;
                    break;
                }
            }
            if (!found) {
                // Unknown key in GET — reply with ERR for that key but continue.
                CommandProcessor::replyErr(rbuf, rbufSize,
                                           "badkey", reqKey, corr_id,
                                           replyFn, ctx);
            }
        }
    }

    // Append correlation id if present.
    if (corr_id && corr_id[0] != '\0' && rem > 3) {
        int w = snprintf(line + pos, (size_t)rem, " #%s", corr_id);
        if (w > 0 && w < rem) { pos += w; rem -= w; }
    }

    line[pos] = '\0';
    replyFn(line, ctx);
}

// ---------------------------------------------------------------------------
// handleSet — apply key=value pairs to RobotConfig.
// Emits "OK set <applied>" and "ERR badkey <key>" per unknown key.
// Calls updatePidGains() if any PID param was applied.
// ---------------------------------------------------------------------------

static void handleSet(KVPair* kvs, int nkv, RobotConfig& cfg,
                      MotorController& mc,
                      char* rbuf, int rbufSize, const char* corr_id,
                      ReplyFn replyFn, void* ctx)
{
    // Build "OK set <applied keys>" body.
    char applied[480];
    int apos = 0;
    int arem = (int)sizeof(applied);

    bool pidChanged = false;
    bool velChanged = false;

    for (int i = 0; i < nkv; ++i) {
        if (!kvs[i].key) continue;  // already rejected by pre-scan

        const char* k = kvs[i].key;
        const char* v = kvs[i].value;

        // Find in registry.
        const ConfigEntry* entry = nullptr;
        for (int r = 0; r < kRegistryCount; ++r) {
            if (strcmp(kRegistry[r].key, k) == 0) {
                entry = &kRegistry[r];
                break;
            }
        }

        if (!entry) {
            // Unknown key — emit ERR and continue processing remaining keys.
            CommandProcessor::replyErr(rbuf, rbufSize,
                                       "badkey", k, corr_id, replyFn, ctx);
            continue;
        }

        // Write through to RobotConfig.
        char* base = reinterpret_cast<char*>(&cfg);
        switch (entry->type) {
        case CFG_FLOAT: {
            float fv = (float)atof(v);
            memcpy(base + entry->offset, &fv, sizeof(float));
            break;
        }
        case CFG_INT: {
            int32_t iv = (int32_t)atoi(v);
            memcpy(base + entry->offset, &iv, sizeof(int32_t));
            break;
        }
        case CFG_FLOAT_AS_INT: {
            float fv = (float)atoi(v);
            memcpy(base + entry->offset, &fv, sizeof(float));
            break;
        }
        }

        // Track PID changes so we can call updatePidGains() once at the end.
        if (strcmp(k, "pid.kp") == 0 || strcmp(k, "pid.ki") == 0 ||
            strcmp(k, "pid.kd") == 0 || strcmp(k, "pid.max") == 0) {
            pidChanged = true;
        }

        // Per-wheel velocity gains must be pushed into the live controllers
        // (they hold copies made at construction). filt/sync are read per-tick.
        if (strcmp(k, "vel.kP") == 0 || strcmp(k, "vel.kI") == 0 ||
            strcmp(k, "vel.kFF") == 0 || strcmp(k, "vel.iMax") == 0 ||
            strcmp(k, "vel.kAw") == 0 || strcmp(k, "minWheelMms") == 0) {
            velChanged = true;
        }

        // Append to applied list.
        if (apos > 0 && arem > 1) { applied[apos++] = ' '; --arem; }
        int w = snprintf(applied + apos, (size_t)arem, "%s=%s", k, v);
        if (w > 0 && w < arem) { apos += w; arem -= w; }
    }

    // Update PID gains in MotorController if any PID param changed.
    if (pidChanged) {
        mc.updatePidGains(cfg.ratioPidKp, cfg.ratioPidKi,
                          cfg.ratioPidKd, cfg.ratioPidMax);
    }
    if (velChanged) {
        mc.updateVelGains(cfg);
    }

    // Emit OK set only if at least one key was applied.
    if (apos > 0) {
        applied[apos] = '\0';
        CommandProcessor::replyOK(rbuf, rbufSize, "set", applied, corr_id,
                                  replyFn, ctx);
    }
}

// ---------------------------------------------------------------------------
// parseSensorToken — parse "sensor=<ch>:<op>:<thr>" into channel, cmp, threshold.
//
// Called by T, D, and TURN handlers after finding the "sensor" key in kvs[].
// The value string is of the form "<ch_name>:<op>:<thr_int>".
//
// Channel name → index mapping (must match StopCondition.cpp::getSensorValue):
//   line0–line3 → 0–3
//   colorR → 4, colorG → 5, colorB → 6, colorC → 7
//
// Op: "ge" → StopCondition::Cmp::GE; "le" → StopCondition::Cmp::LE.
// thr: parsed as integer (sensor values are uint16_t raw ADC counts).
//
// Returns true on success (out-params set); false on any parse/lookup failure.
// ---------------------------------------------------------------------------

static bool parseSensorToken(const char* value,
                              uint8_t& ch_out, float& thr_out,
                              StopCondition::Cmp& cmp_out)
{
    // Expected format: "<ch_name>:<op>:<thr>"
    // Split on ':' in a local copy of value.
    char buf[32];
    int vlen = 0;
    for (const char* p = value; *p && vlen < (int)sizeof(buf) - 1; ++p, ++vlen) {
        buf[vlen] = *p;
    }
    buf[vlen] = '\0';

    // Find first ':' to separate channel name.
    char* colon1 = strchr(buf, ':');
    if (!colon1) return false;
    *colon1 = '\0';
    const char* ch_name = buf;
    const char* rest    = colon1 + 1;

    // Find second ':' to separate op from threshold.
    char* colon2 = strchr(rest, ':');
    if (!colon2) return false;
    *colon2 = '\0';
    const char* op_str  = rest;
    const char* thr_str = colon2 + 1;

    // Resolve channel name to index.
    uint8_t ch = 0;
    bool found = false;
    struct { const char* name; uint8_t idx; } chMap[] = {
        { "line0",  0 }, { "line1",  1 }, { "line2",  2 }, { "line3",  3 },
        { "colorR", 4 }, { "colorG", 5 }, { "colorB", 6 }, { "colorC", 7 },
    };
    for (int i = 0; i < 8; ++i) {
        if (strcmp(ch_name, chMap[i].name) == 0) {
            ch    = chMap[i].idx;
            found = true;
            break;
        }
    }
    if (!found) return false;

    // Resolve operator.
    StopCondition::Cmp cmp;
    if (strcmp(op_str, "ge") == 0) {
        cmp = StopCondition::Cmp::GE;
    } else if (strcmp(op_str, "le") == 0) {
        cmp = StopCondition::Cmp::LE;
    } else {
        return false;
    }

    // Parse threshold.
    int thr = atoi(thr_str);

    ch_out  = ch;
    thr_out = (float)thr;
    cmp_out = cmp;
    return true;
}

// ---------------------------------------------------------------------------
// process — v2 command dispatch
// ---------------------------------------------------------------------------

void CommandProcessor::process(const char* line, ReplyFn replyFn, void* ctx)
{
    // Telemetry streaming is gated purely on motors-running (see Robot::
    // telemetryEmit): stream while driving, silent when stopped. To read while
    // stopped, the host REQUESTS a frame (SNAP) — a synchronous command-response,
    // so commands don't need to keep the stream alive.

    // Working buffer for tokenization. parseTokens() copies into this.
    char workBuf[512];
    char* tokens[MAX_TOKENS];
    char  corr_id[16];

    int ntok = parseTokens(line, workBuf, sizeof(workBuf),
                           tokens, MAX_TOKENS,
                           corr_id, sizeof(corr_id));
    if (ntok == 0) return;

    const char* verb = tokens[0];

    // Reply buffer used by all handlers.
    char rbuf[520];

    // ── Check for any bad kv token (key missing) before dispatching ──────────
    // Scan for kv tokens with missing key; these are always badarg.
    KVPair kvs[MAX_KV];
    int    nkv = parseKV(tokens, ntok, kvs, MAX_KV);
    for (int i = 0; i < nkv; ++i) {
        if (kvs[i].key == nullptr) {
            replyErr(rbuf, sizeof(rbuf), "badarg", "missing key", corr_id, replyFn, ctx);
            return;
        }
    }

    // ── Dispatch on verb ─────────────────────────────────────────────────────

    // ── HELLO ────────────────────────────────────────────────────────────────
    // Reply: DEVICE:NEZHA2:robot:<friendly_name>:<serial>
    // Raw identification banner — NOT wrapped in OK taxonomy.
    // Emitted at boot and on demand so mbdeploy probe_type() can identify us.
    // Format matches announce.md: DEVICE:<role>:<common_name>:<device_name>:<serial>
    // serial printed as decimal uint32 (%lu), same source as ID command.
    if (strcmp(verb, "HELLO") == 0) {
        const char* name   = microbit_friendly_name();
        uint32_t    serial = microbit_serial_number();
        char banner[64];
        snprintf(banner, sizeof(banner),
                 "DEVICE:NEZHA2:robot:%s:%lu", name, (unsigned long)serial);
        replyFn(banner, ctx);
        return;
    }

    // ── PING ─────────────────────────────────────────────────────────────────
    // Reply: OK pong t=<robot_ms>  (clock-sync probe; t MUST be robot clock)
    if (strcmp(verb, "PING") == 0) {
        uint32_t t = _robot.systemTime();
        char body[32];
        snprintf(body, sizeof(body), "t=%lu", (unsigned long)t);
        replyOK(rbuf, sizeof(rbuf), "pong", body, corr_id, replyFn, ctx);
        return;
    }

    // ── ECHO ─────────────────────────────────────────────────────────────────
    // Reply: OK echo <payload>  — payload is everything after the verb token,
    // preserving case and spacing exactly (extracted from raw line, not tokens).
    if (strcmp(verb, "ECHO") == 0) {
        // Find the payload by skipping past the verb in the original line.
        // line is the original (immutable) parameter; workBuf is our copy.
        // We need to locate the content after "ECHO" in the raw line.
        const char* p = line;
        // Skip leading whitespace.
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
        // Skip the verb token (any run of non-whitespace).
        while (*p != '\0' && *p != ' ' && *p != '\t') ++p;
        // Skip exactly one whitespace separator (if present).
        if (*p == ' ' || *p == '\t') ++p;
        // p now points at the payload (may be empty).

        // If a corr_id was found, we must strip the trailing "#<id>" from
        // the payload. The parseTokens() pass already removed it from tokens
        // but the raw line still has it. Strip it now.
        char payload[512];
        int plen = 0;
        const char* q = p;
        while (*q != '\0' && *q != '\r' && *q != '\n') {
            payload[plen++] = *q++;
            if (plen >= (int)(sizeof(payload) - 1)) break;
        }
        payload[plen] = '\0';

        // Trim trailing whitespace.
        while (plen > 0 && (payload[plen-1] == ' ' || payload[plen-1] == '\t')) {
            payload[--plen] = '\0';
        }

        // Strip trailing corr_id token ("#<digits>") if present.
        if (corr_id[0] != '\0') {
            // The corr_id token is "#" + corr_id digits at the very end.
            // Look for " #<corr_id>" suffix and remove it.
            char suffix[20];
            snprintf(suffix, sizeof(suffix), " #%s", corr_id);
            int slen = (int)strlen(suffix);
            if (plen >= slen && strcmp(payload + plen - slen, suffix) == 0) {
                plen -= slen;
                payload[plen] = '\0';
            }
        }

        replyOK(rbuf, sizeof(rbuf), "echo", payload, corr_id, replyFn, ctx);
        return;
    }

    // ── ID ───────────────────────────────────────────────────────────────────
    // Reply: ID model=Nezha2 name=<name> serial=<serial> fw=<ver> proto=2
    //        caps=<present subsystems>
    // Tag is "ID", not "OK" — this uses a custom snprintf path.
    if (strcmp(verb, "ID") == 0) {
        // Robot friendly name and serial number from CODAL.
        // microbit_friendly_name() returns a pointer to a static buffer (5 chars).
        // microbit_serial_number() returns a uint32_t hardware ID.
        const char* name   = microbit_friendly_name();
        uint32_t    serial = microbit_serial_number();

        // Build caps= string from runtime-present subsystems.
        char caps[64];
        caps[0] = '\0';
        bool first = true;
        auto addCap = [&](const char* cap) {
            if (!first) {
                int n = (int)strlen(caps);
                caps[n] = ','; caps[n+1] = '\0';
            }
            // strncat safe: caps is 64 bytes, max total caps length is ~50.
            int rem = (int)(sizeof(caps) - strlen(caps) - 1);
            if (rem > 0) strncat(caps, cap, (size_t)rem);
            first = false;
        };
        if (_robot.otos.is_initialized())        addCap("otos");
        if (_robot.line.is_initialized())        addCap("line");
        if (_robot.colorSensor.is_initialized()) addCap("color");
        // gripper: omitted from caps — no gripper hardware (re-enable when added)
        // if (_robot.gripper.is_initialized()) addCap("gripper");
        // portio is always present.
        addCap("portio");

        if (corr_id[0] != '\0') {
            snprintf(rbuf, sizeof(rbuf),
                     "ID model=Nezha2 name=%s serial=%lu fw=%s proto=%d caps=%s #%s",
                     name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION,
                     caps, corr_id);
        } else {
            snprintf(rbuf, sizeof(rbuf),
                     "ID model=Nezha2 name=%s serial=%lu fw=%s proto=%d caps=%s",
                     name, (unsigned long)serial, FIRMWARE_VERSION, PROTO_VERSION,
                     caps);
        }
        replyFn(rbuf, ctx);
        return;
    }

    // ── VER ──────────────────────────────────────────────────────────────────
    // Reply: OK ver fw=<ver> proto=2
    if (strcmp(verb, "VER") == 0) {
        char body[64];
        snprintf(body, sizeof(body), "fw=%s proto=%d", FIRMWARE_VERSION, PROTO_VERSION);
        replyOK(rbuf, sizeof(rbuf), "ver", body, corr_id, replyFn, ctx);
        return;
    }

    // ── HELP ─────────────────────────────────────────────────────────────────
    // Reply: OK help <verb list>
    if (strcmp(verb, "HELP") == 0) {
        replyOK(rbuf, sizeof(rbuf), "help",
                "PING ECHO ID VER HELP SET GET GET VEL STREAM SNAP S T D G R TURN VW RF X STOP GRIP ZERO OI OZ OR OP OV OL OA P PA [sensor=<ch>:<op>:<thr>]",
                corr_id, replyFn, ctx);
        return;
    }

    // ── GET ──────────────────────────────────────────────────────────────────
    // GET                → CFG <all key=value pairs>
    // GET ml pid.kp      → CFG ml=0.487 pid.kp=2.0
    // GET ml #9          → CFG ml=0.487 #9
    // GET VEL            → OK get vel=<vL>:<src>,<vR>:<src>
    //                      Reports per-wheel measured velocity (mm/s) and source
    //                      flag: 'C' = chip (register 0x47), 'E' = encoder-delta.
    //                      Used for bench confirmation and PID tuning.
    if (strcmp(verb, "GET") == 0) {
        // Special case: GET VEL — velocity readout (not a config key).
        if (ntok >= 2 && strcmp(tokens[1], "VEL") == 0) {
            // Read velocities from HardwareState (written by controlCollect each tick).
            // Chip readSpeed (0x47) is disabled (sprint 013 throb fix), so source
            // is always encoder-delta ('E') for both wheels.
            float vL = _robot.state.inputs.velLMms;
            float vR = _robot.state.inputs.velRMms;
            char body[48];
            snprintf(body, sizeof(body), "vel=%d:E,%d:E",
                     (int)vL, (int)vR);
            replyOK(rbuf, sizeof(rbuf), "get", body, corr_id, replyFn, ctx);
            return;
        }
        // Positional args (tokens[1..]) are the requested keys.
        // parseKV() would consume tokens that contain '='; GET only uses plain
        // key names, so pass the raw token list directly.
        handleGet(tokens, ntok, _robot.config, rbuf, sizeof(rbuf),
                  corr_id, replyFn, ctx);
        return;
    }

    // ── SET ──────────────────────────────────────────────────────────────────
    // SET ml=0.487 pid.kp=2.0  → OK set ml=0.487 pid.kp=2.0
    // SET badkey=99             → ERR badkey badkey
    // SET ml=0.487 bad=1        → OK set ml=0.487   (+ ERR badkey bad)
    if (strcmp(verb, "SET") == 0) {
        if (nkv == 0) {
            replyErr(rbuf, sizeof(rbuf), "badarg", "no key=value pairs", corr_id,
                     replyFn, ctx);
            return;
        }
        handleSet(kvs, nkv, _robot.config, _robot.motorController,
                  rbuf, sizeof(rbuf), corr_id, replyFn, ctx);
        return;
    }

    // ── STREAM ───────────────────────────────────────────────────────────────
    // STREAM <ms>              → OK stream period=<ms>  (0 = off)
    // STREAM fields=enc,pose   → OK stream fields=enc,pose  (subset)
    if (strcmp(verb, "STREAM") == 0) {
        // Check for fields= kv pair first.
        bool hasFields = false;
        for (int i = 0; i < nkv; ++i) {
            if (kvs[i].key && strcmp(kvs[i].key, "fields") == 0) {
                hasFields = true;
                // Parse comma-separated field names and build bitmask.
                uint8_t mask = 0;
                const char* fp = kvs[i].value;
                // Tokenize the value string on commas.
                char fbuf[64];
                int flen = 0;
                for (const char* c = fp; ; ++c) {
                    bool end = (*c == '\0' || *c == ',');
                    if (!end && flen < (int)(sizeof(fbuf) - 1)) {
                        fbuf[flen++] = *c;
                    }
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
                _robot.config.tlmFields = mask ? mask : TLM_FIELD_ALL;
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
                    if (_robot.config.tlmFields & kFieldNames[fi].bit) {
                        if (needComma) { body[bpos++] = ','; --brem; }
                        bw = snprintf(body + bpos, (size_t)brem, "%s", kFieldNames[fi].name);
                        if (bw > 0 && bw < brem) { bpos += bw; brem -= bw; }
                        needComma = true;
                    }
                }
                body[bpos] = '\0';
                replyOK(rbuf, sizeof(rbuf), "stream", body, corr_id, replyFn, ctx);
                break;
            }
        }
        if (hasFields) return;

        // No fields= — expect a positional period argument.
        if (ntok < 2) {
            replyErr(rbuf, sizeof(rbuf), "badarg", "usage: STREAM <ms>", corr_id,
                     replyFn, ctx);
            return;
        }
        int32_t ms = (int32_t)atoi(tokens[1]);
        if (ms < 0) ms = 0;
        if (ms > 0 && ms < 20) ms = 20;  // clamp to 20 ms minimum
        _robot.config.tlmPeriodMs = ms;
        char body[32];
        snprintf(body, sizeof(body), "period=%d", (int)ms);
        replyOK(rbuf, sizeof(rbuf), "stream", body, corr_id, replyFn, ctx);
        return;
    }

    // ── SNAP ─────────────────────────────────────────────────────────────────
    // SNAP → one telemetry frame returned SYNCHRONOUSLY as the reply (a request/
    // response read, not the async stream). This is the radio-safe way to read
    // telemetry while stopped: it's an ordinary command-response, so the relay
    // delivers it (unlike an async stream frame, which the radio drops).
    if (strcmp(verb, "SNAP") == 0) {
        char tlmBuf[128];
        _robot.buildTlmFrame(tlmBuf, sizeof(tlmBuf));
        replyFn(tlmBuf, ctx);
        return;
    }

    // ── DBG — debug/testing controls ──────────────────────────────────────────
    // DBG LOOP <x> <state>  → set scheduler task x run flag (0/1).
    //                         Takes effect in LoopScheduler::run_all().
    // DBG LOOP              → list tasks: "LOOP <idx> <name> run=<0|1> n=<runs> avgUs=<us>"
    if (strcmp(verb, "DBG") == 0) {
        if (ntok < 2) {
            replyErr(rbuf, sizeof(rbuf), "badarg", "dbg subcommand", corr_id, replyFn, ctx);
            return;
        }
        // Only the verb is auto-uppercased; normalise the subcommand here.
        for (char* c = tokens[1]; *c != '\0'; ++c) {
            *c = (char)toupper((unsigned char)*c);
        }
        if (strcmp(tokens[1], "LOOP") == 0) {
            if (_sched == nullptr) {
                replyErr(rbuf, sizeof(rbuf), "noimpl", "no scheduler", corr_id, replyFn, ctx);
                return;
            }
            // DBG LOOP RESET — zero all timing stats for a fresh window.
            if (ntok >= 3 && strcmp(tokens[2], "RESET") == 0) {
                _sched->resetStats();
                replyOK(rbuf, sizeof(rbuf), "dbg", "loop reset", corr_id, replyFn, ctx);
                return;
            }
            // DBG LOOP <x> <state> — set task x run flag.
            if (ntok >= 4) {
                int x = atoi(tokens[2]);
                int s = atoi(tokens[3]);
                if (!_sched->setTaskRun(x, s != 0)) {
                    replyErr(rbuf, sizeof(rbuf), "badarg", "task index", corr_id, replyFn, ctx);
                    return;
                }
                char body[40];
                snprintf(body, sizeof(body), "loop %d run=%d", x, (s != 0) ? 1 : 0);
                replyOK(rbuf, sizeof(rbuf), "dbg", body, corr_id, replyFn, ctx);
                return;
            }
            // DBG LOOP — emit ALL timing as ONE compact line (sending ~10
            // separate lines overflows the 255-byte serial TX buffer and faults
            // the firmware). avgUs are per-iteration: ctl=control/PID task;
            // cyc=full loop period incl. idle sleep; wrk=work excl. sleep;
            // t0..t7 = the 8 low-priority tasks in table order (comms-in,
            // drive-advance, odometry-predict, otos-correct, line-read,
            // color-read, ports-read, telemetry-emit).
            {
                unsigned long cr = (unsigned long)_sched->controlRuns();
                unsigned long lr = (unsigned long)_sched->loopRuns();
                auto avgUs = [](const Task* t) -> unsigned long {
                    return (t && t->runs > 0)
                         ? (unsigned long)(t->totalTimeUs / t->runs) : 0UL;
                };
                char buf[200];
                // Single snprintf, fixed args — no incremental offset math, no
                // loop, one send. Output ~110 chars, well under buf and the
                // 255-byte TX buffer.
                snprintf(buf, sizeof(buf),
                    "LOOP ctl=%lu cyc=%lu wrk=%lu loops=%lu "
                    "t0=%lu t1=%lu t2=%lu t3=%lu t4=%lu t5=%lu t6=%lu t7=%lu",
                    cr ? (unsigned long)(_sched->controlTotalUs() / cr) : 0UL,
                    lr ? (unsigned long)(_sched->loopTotalUs() / lr) : 0UL,
                    lr ? (unsigned long)(_sched->loopWorkTotalUs() / lr) : 0UL,
                    lr,
                    avgUs(_sched->taskAt(0)), avgUs(_sched->taskAt(1)),
                    avgUs(_sched->taskAt(2)), avgUs(_sched->taskAt(3)),
                    avgUs(_sched->taskAt(4)), avgUs(_sched->taskAt(5)),
                    avgUs(_sched->taskAt(6)), avgUs(_sched->taskAt(7)));
                replyFn(buf, ctx);
            }
            replyOK(rbuf, sizeof(rbuf), "dbg", "loop", corr_id, replyFn, ctx);
            return;
        }
        // ── DBG I2C [RESET] ───────────────────────────────────────────────────
        // DBG I2C        → one line: per-device txn/err/last-err, reentry count,
        //                  stuck encoder counters. All on ONE serial send (≤255 B).
        // DBG I2C RESET  → zero all I2CBus stats + stuck counters; reply OK.
        if (strcmp(tokens[1], "I2C") == 0) {
            if (_i2cBus == nullptr) {
                replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus", corr_id, replyFn, ctx);
                return;
            }
            if (ntok >= 3 && strcmp(tokens[2], "RESET") == 0) {
                _i2cBus->resetStats();
                _robot.motorController.resetStuckCounters();
                replyOK(rbuf, sizeof(rbuf), "dbg", "i2c reset", corr_id, replyFn, ctx);
                return;
            }
            // Build a single compact dump line — must fit in ≤200 chars so the
            // 255-byte serial TX buffer is not exceeded.
            // Format:
            //   I2C 0x10:txn=N err=N last=N 0x17:txn=N err=N last=N
            //       0x1A:txn=N err=N last=N 0x43:txn=N err=N last=N
            //       reentry=N stuck=L:N,R:N
            // (single line, all on one snprintf)
            uint32_t rV  = _i2cBus->reentryViolations();
            uint8_t  sL  = _robot.motorController.stuckCountL();
            uint8_t  sR  = _robot.motorController.stuckCountR();
            // Single snprintf into ≤200-byte buffer (well under 255-byte serial TX
            // buffer). snprintf truncates + NUL-terminates safely on overflow.
            // Format: I2C 0x10:txn=N err=N last=N 0x17:... reentry=N stuck=L:N,R:N
            // Worst-case length with max uint32 counts: ~200 chars (truncated safely).
            char buf[200];
            int n = snprintf(buf, sizeof(buf),
                "I2C 0x10:txn=%lu err=%lu last=%d "
                "0x17:txn=%lu err=%lu last=%d "
                "0x1A:txn=%lu err=%lu last=%d "
                "0x43:txn=%lu err=%lu last=%d "
                "reentry=%lu stuck=L:%u,R:%u",
                (unsigned long)_i2cBus->txnCount(0x10),
                (unsigned long)_i2cBus->errCount(0x10),
                _i2cBus->lastErr(0x10),
                (unsigned long)_i2cBus->txnCount(0x17),
                (unsigned long)_i2cBus->errCount(0x17),
                _i2cBus->lastErr(0x17),
                (unsigned long)_i2cBus->txnCount(0x1A),
                (unsigned long)_i2cBus->errCount(0x1A),
                _i2cBus->lastErr(0x1A),
                (unsigned long)_i2cBus->txnCount(0x43),
                (unsigned long)_i2cBus->errCount(0x43),
                _i2cBus->lastErr(0x43),
                (unsigned long)rV,
                (unsigned)sL,
                (unsigned)sR);
            // Safety: snprintf returns bytes-that-would-have-been-written.
            // If it exceeded sizeof(buf)-1, the string was truncated and NUL-
            // terminated by snprintf — safe to send as-is.
            (void)n;
            replyFn(buf, ctx);
            replyOK(rbuf, sizeof(rbuf), "dbg", "i2c", corr_id, replyFn, ctx);
            return;
        }
        // ── DBG I2CLOG [ARM] — dump the I2C transaction ring buffer ───────────
        // Diagnostic tool (OFF by default — zero overhead unless armed). Usage:
        //   DBG I2CLOG ARM   → reset + start logging every transaction
        //   <reproduce>      → drive / exercise the bus
        //   DBG I2CLOG       → dump the recent ring (one line, addr/RW/byte/dt)
        // Dump when telemetry is quiet so it doesn't garble the async serial TX.
        if (strcmp(tokens[1], "I2CLOG") == 0) {
            if (_i2cBus == nullptr) {
                replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus", corr_id, replyFn, ctx);
                return;
            }
            if (ntok >= 3 && strcmp(tokens[2], "ARM") == 0) {
                _i2cBus->resetStats();
                _i2cBus->setLogging(true);
            } else {
                _i2cBus->dumpRecent(replyFn, ctx);
            }
            replyOK(rbuf, sizeof(rbuf), "dbg", "i2clog", corr_id, replyFn, ctx);
            return;
        }
        // ── DBG IRQGUARD [0|1] — mask IRQs for the FULL I2C transaction ────────
        // nRF52 TWIM errata fix: with no arg, reports state; with 0/1, toggles.
        if (strcmp(tokens[1], "IRQGUARD") == 0) {
            if (_i2cBus == nullptr) {
                replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus", corr_id, replyFn, ctx);
                return;
            }
            if (ntok >= 3) _i2cBus->setIrqGuard(atoi(tokens[2]) != 0);
            char msg[24];
            snprintf(msg, sizeof(msg), "irqguard=%d", _i2cBus->irqGuard() ? 1 : 0);
            replyOK(rbuf, sizeof(rbuf), "dbg", msg, corr_id, replyFn, ctx);
            return;
        }
        // ── DBG WEDGE — run the self-contained encoder-wedge harness ──────────
        // Hands raw i2c + serial to runWedgeTest(), which takes over the robot
        // and NEVER returns until the bug trips or a serial byte arrives. The
        // cooperative loop is suspended for the duration (intended).
        if (strcmp(tokens[1], "WEDGE") == 0) {
            if (_sched == nullptr) {
                replyErr(rbuf, sizeof(rbuf), "noimpl", "no scheduler", corr_id, replyFn, ctx);
                return;
            }
            // DBG WEDGE [rateHz] [writeMs] [busKHz] [dither] [reg] [sensors] [realCtrl]
            int wrate  = (ntok >= 3) ? atoi(tokens[2]) : 50;    // loop/read rate (Hz)
            int wwrite = (ntok >= 4) ? atoi(tokens[3]) : 40;    // motor write min interval (ms)
            int wbus   = (ntok >= 5) ? atoi(tokens[4]) : 400;   // I2C bus speed (kHz)
            int wdith  = (ntok >= 6) ? atoi(tokens[5]) : 3;     // per-tick pwm dither (+/- units)
            int wreg   = (ntok >= 7) ? atoi(tokens[6]) : 0x46;  // encoder read reg (46/47)
            int wsens  = (ntok >= 8) ? atoi(tokens[7]) : 0;     // 1 = sensor bus traffic
            int wreal  = (ntok >= 9) ? atoi(tokens[8]) : 0;     // 1 = drive via real PID/Motor path
            replyOK(rbuf, sizeof(rbuf), "dbg", "wedge start", corr_id, replyFn, ctx);
            runWedgeTest(_sched->uBit(), wrate, wwrite, wbus, wdith, wreg, wsens,
                         wreal, &_sched->robot());  // blocks until wedge/stop
            replyOK(rbuf, sizeof(rbuf), "dbg", "wedge end", corr_id, replyFn, ctx);
            return;
        }
        replyErr(rbuf, sizeof(rbuf), "badarg", "dbg subcommand", corr_id, replyFn, ctx);
        return;
    }

    // ── I2CW — raw I2C write (diagnostic) ─────────────────────────────────────
    // I2CW <addr7-hex> <byte-hex> [byte-hex ...]
    //   Writes the given bytes to the 7-bit device address (hex). Up to 24 bytes.
    //   Reply: OK i2cw addr=0xNN n=<len> status=<codal>
    // Lets the bus be poked live from the serial port (e.g. replay the Nezha
    // controller wake sequence to recover a wedged encoder) without a reflash.
    if (strcmp(verb, "I2CW") == 0) {
        if (_i2cBus == nullptr) {
            replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus", corr_id, replyFn, ctx);
            return;
        }
        if (ntok < 3) {
            replyErr(rbuf, sizeof(rbuf), "badarg", "usage: I2CW <addr> <byte>...",
                     corr_id, replyFn, ctx);
            return;
        }
        uint8_t addr7 = (uint8_t)strtol(tokens[1], nullptr, 16);
        uint8_t data[24];
        int len = 0;
        for (int i = 2; i < ntok && len < (int)sizeof(data); ++i) {
            data[len++] = (uint8_t)strtol(tokens[i], nullptr, 16);
        }
        int status = _i2cBus->write((uint16_t)(addr7 << 1), data, len);
        char body[48];
        snprintf(body, sizeof(body), "addr=0x%02X n=%d status=%d", addr7, len, status);
        replyOK(rbuf, sizeof(rbuf), "i2cw", body, corr_id, replyFn, ctx);
        return;
    }

    // ── I2CR — raw I2C read (diagnostic) ──────────────────────────────────────
    // I2CR <addr7-hex> <count> [reg-hex]
    //   Reads <count> bytes (1..16) from the 7-bit device address (hex). If a
    //   register byte is given, it is written first with a repeated-start.
    //   Reply: OK i2cr addr=0xNN n=<count> wstatus=<s> status=<s> data=AA,BB,...
    if (strcmp(verb, "I2CR") == 0) {
        if (_i2cBus == nullptr) {
            replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus", corr_id, replyFn, ctx);
            return;
        }
        if (ntok < 3) {
            replyErr(rbuf, sizeof(rbuf), "badarg", "usage: I2CR <addr> <count> [reg]",
                     corr_id, replyFn, ctx);
            return;
        }
        uint8_t addr7 = (uint8_t)strtol(tokens[1], nullptr, 16);
        int count = atoi(tokens[2]);
        if (count < 1 || count > 16) {
            replyErr(rbuf, sizeof(rbuf), "range", "count", corr_id, replyFn, ctx);
            return;
        }
        int wstatus = 0;
        if (ntok >= 4) {
            uint8_t reg = (uint8_t)strtol(tokens[3], nullptr, 16);
            wstatus = _i2cBus->write((uint16_t)(addr7 << 1), &reg, 1, true);
        }
        uint8_t buf[16];
        int status = _i2cBus->read((uint16_t)(addr7 << 1), buf, count);
        char body[120];
        int pos = snprintf(body, sizeof(body),
                           "addr=0x%02X n=%d wstatus=%d status=%d data=",
                           addr7, count, wstatus, status);
        for (int i = 0; i < count && pos < (int)sizeof(body) - 4; ++i) {
            pos += snprintf(body + pos, (size_t)((int)sizeof(body) - pos),
                            "%s%02X", i ? "," : "", buf[i]);
        }
        replyOK(rbuf, sizeof(rbuf), "i2cr", body, corr_id, replyFn, ctx);
        return;
    }

    // ── S — streaming velocity ────────────────────────────────────────────────
    // S <l> <r>  → OK drive l=<l> r=<r>
    // Watchdog reset is implicit: DriveController::beginStream() updates _lastSMs.
    if (strcmp(verb, "S") == 0) {
        if (ntok < 3) {
            replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
            return;
        }
        int l = atoi(tokens[1]);
        int r = atoi(tokens[2]);
        // Range check: cap at ±1000 mm/s (hardware limit)
        if (l < -1000 || l > 1000) {
            replyErr(rbuf, sizeof(rbuf), "range", "l", corr_id, replyFn, ctx);
            return;
        }
        if (r < -1000 || r > 1000) {
            replyErr(rbuf, sizeof(rbuf), "range", "r", corr_id, replyFn, ctx);
            return;
        }
        _robot.driveController.beginStream((float)l, (float)r, _robot.systemTime(), _robot.state.target, replyFn, ctx);
        char body[32];
        snprintf(body, sizeof(body), "l=%d r=%d", l, r);
        replyOK(rbuf, sizeof(rbuf), "drive", body, corr_id, replyFn, ctx);
        return;
    }

    // ── T — timed drive ───────────────────────────────────────────────────────
    // T <l> <r> <ms>  → OK drive l=<l> r=<r> ms=<ms>; later EVT done T
    if (strcmp(verb, "T") == 0) {
        if (ntok < 4) {
            replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
            return;
        }
        int l  = atoi(tokens[1]);
        int r  = atoi(tokens[2]);
        int ms = atoi(tokens[3]);
        if (l < -1000 || l > 1000) {
            replyErr(rbuf, sizeof(rbuf), "range", "l", corr_id, replyFn, ctx);
            return;
        }
        if (r < -1000 || r > 1000) {
            replyErr(rbuf, sizeof(rbuf), "range", "r", corr_id, replyFn, ctx);
            return;
        }
        if (ms < 1 || ms > 30000) {
            replyErr(rbuf, sizeof(rbuf), "range", "ms", corr_id, replyFn, ctx);
            return;
        }
        _robot.driveController.beginTimed((float)l, (float)r, (uint32_t)ms, _robot.systemTime(), _robot.state.target, replyFn, ctx, corr_id);

        // Optional sensor= modifier: appended after begin*() returns.
        // Safe: addStop() after start() is allowed — start() snapshots the
        // baseline (enc/heading) but does NOT copy the stops array; tick()
        // reads _stops[0.._nStops-1] directly, so a stop added here is
        // evaluated on the very next tick().
        for (int i = 0; i < nkv; ++i) {
            if (kvs[i].key && strcmp(kvs[i].key, "sensor") == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (!parseSensorToken(kvs[i].value, ch, thr, cmp)) {
                    replyErr(rbuf, sizeof(rbuf), "badarg", "sensor", corr_id, replyFn, ctx);
                    _robot.driveController.cancel(_robot.systemTime(), replyFn, ctx);
                    return;
                }
                _robot.driveController.activeCmd().addStop(makeSensorStop(ch, thr, cmp));
                break;
            }
        }

        char body[48];
        snprintf(body, sizeof(body), "l=%d r=%d ms=%d", l, r, ms);
        replyOK(rbuf, sizeof(rbuf), "drive", body, corr_id, replyFn, ctx);
        return;
    }

    // ── D — distance drive ────────────────────────────────────────────────────
    // D <l> <r> <mm>  → OK drive l=<l> r=<r> mm=<mm>; later EVT done D
    if (strcmp(verb, "D") == 0) {
        if (ntok < 4) {
            replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
            return;
        }
        int l  = atoi(tokens[1]);
        int r  = atoi(tokens[2]);
        int mm = atoi(tokens[3]);
        if (l < -1000 || l > 1000) {
            replyErr(rbuf, sizeof(rbuf), "range", "l", corr_id, replyFn, ctx);
            return;
        }
        if (r < -1000 || r > 1000) {
            replyErr(rbuf, sizeof(rbuf), "range", "r", corr_id, replyFn, ctx);
            return;
        }
        if (mm < 1 || mm > 10000) {
            replyErr(rbuf, sizeof(rbuf), "range", "mm", corr_id, replyFn, ctx);
            return;
        }
        _robot.distanceDrive((int32_t)l, (int32_t)r, (int32_t)mm, replyFn, ctx, corr_id);

        // Optional sensor= modifier: appended after begin*() returns.
        // Safe: addStop() after start() — see T handler comment above.
        for (int i = 0; i < nkv; ++i) {
            if (kvs[i].key && strcmp(kvs[i].key, "sensor") == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (!parseSensorToken(kvs[i].value, ch, thr, cmp)) {
                    replyErr(rbuf, sizeof(rbuf), "badarg", "sensor", corr_id, replyFn, ctx);
                    _robot.driveController.cancel(_robot.systemTime(), replyFn, ctx);
                    return;
                }
                _robot.driveController.activeCmd().addStop(makeSensorStop(ch, thr, cmp));
                break;
            }
        }

        char body[48];
        snprintf(body, sizeof(body), "l=%d r=%d mm=%d", l, r, mm);
        replyOK(rbuf, sizeof(rbuf), "drive", body, corr_id, replyFn, ctx);
        return;
    }

    // ── G — go-to XY ─────────────────────────────────────────────────────────
    // G <x> <y> <speed>  → OK goto x=<x> y=<y> speed=<speed>; later EVT done G
    if (strcmp(verb, "G") == 0) {
        if (ntok < 4) {
            replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
            return;
        }
        int x     = atoi(tokens[1]);
        int y     = atoi(tokens[2]);
        int speed = atoi(tokens[3]);
        if (x < -10000 || x > 10000) {
            replyErr(rbuf, sizeof(rbuf), "range", "x", corr_id, replyFn, ctx);
            return;
        }
        if (y < -10000 || y > 10000) {
            replyErr(rbuf, sizeof(rbuf), "range", "y", corr_id, replyFn, ctx);
            return;
        }
        if (speed < 1 || speed > 1000) {
            replyErr(rbuf, sizeof(rbuf), "range", "speed", corr_id, replyFn, ctx);
            return;
        }
        _robot.driveController.beginGoTo((float)x, (float)y, (float)speed, _robot.systemTime(), _robot.state.target, replyFn, ctx, corr_id);
        char body[64];
        snprintf(body, sizeof(body), "x=%d y=%d speed=%d", x, y, speed);
        replyOK(rbuf, sizeof(rbuf), "goto", body, corr_id, replyFn, ctx);
        return;
    }

    // ── R — arc drive (open-ended, MotionCommand-based) ──────────────────────
    // R <speed_mms> <radius_mm> [#id]  → OK arc speed=<v> radius=<r> [#id]
    // Computes κ = 1/radius; radius=0 ⇒ straight (κ=0).
    // Sign convention: positive radius ⇒ CCW/left arc (matches BodyKinematics::inverse).
    // speed=0 ⇒ SOFT ramp-down, emits EVT done R.
    // Open-ended: host cancels with X or sends R 0 <r> to soft-stop.
    if (strcmp(verb, "R") == 0) {
        if (ntok < 3) {
            replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
            return;
        }
        int speed  = atoi(tokens[1]);
        int radius = atoi(tokens[2]);
        if (speed < -1000 || speed > 1000) {
            replyErr(rbuf, sizeof(rbuf), "range", "speed", corr_id, replyFn, ctx);
            return;
        }
        if (radius < -10000 || radius > 10000) {
            replyErr(rbuf, sizeof(rbuf), "range", "radius", corr_id, replyFn, ctx);
            return;
        }
        uint32_t now = _robot.systemTime();
        _robot.driveController.beginArc((float)speed, (float)radius, now,
                                        _robot.state.target, replyFn, ctx, corr_id);
        char body[48];
        snprintf(body, sizeof(body), "speed=%d radius=%d", speed, radius);
        replyOK(rbuf, sizeof(rbuf), "arc", body, corr_id, replyFn, ctx);
        return;
    }

    // ── TURN — rotate to absolute heading (MotionCommand-based, HEADING stop) ──
    // TURN <heading_cdeg> [eps=<cdeg>] [#id]
    //   heading_cdeg: integer, target heading in centidegrees (range ±18000 = ±180°).
    //   eps=<cdeg>: optional tolerance in centidegrees (default 300 = 3°; range 10–1800).
    //   Reply: OK turn heading=<cdeg> eps=<cdeg> [#id]
    //   Async completion: EVT done TURN [#id]
    // Positive heading ⇒ CCW rotation (positive ω, matches OTOS CCW convention).
    // Shortest-path sign computed at begin; SOFT stop (BVC ramps ω to zero on arrival).
    if (strcmp(verb, "TURN") == 0) {
        if (ntok < 2) {
            replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
            return;
        }
        int heading_cdeg = atoi(tokens[1]);
        if (heading_cdeg < -18000 || heading_cdeg > 18000) {
            replyErr(rbuf, sizeof(rbuf), "range", "heading", corr_id, replyFn, ctx);
            return;
        }

        // Parse optional eps=<cdeg> keyword argument; default 300 cdeg (3°).
        int eps_cdeg = 300;
        for (int i = 0; i < nkv; ++i) {
            if (kvs[i].key && strcmp(kvs[i].key, "eps") == 0) {
                eps_cdeg = atoi(kvs[i].value);
                if (eps_cdeg < 10 || eps_cdeg > 1800) {
                    replyErr(rbuf, sizeof(rbuf), "range", "eps", corr_id, replyFn, ctx);
                    return;
                }
                break;
            }
        }

        uint32_t now = _robot.systemTime();
        _robot.driveController.beginTurn((float)heading_cdeg, (float)eps_cdeg, now,
                                         _robot.state.target, replyFn, ctx, corr_id);

        // Optional sensor= modifier: appended after begin*() returns.
        // Safe: addStop() after start() — see T handler comment above.
        for (int i = 0; i < nkv; ++i) {
            if (kvs[i].key && strcmp(kvs[i].key, "sensor") == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (!parseSensorToken(kvs[i].value, ch, thr, cmp)) {
                    replyErr(rbuf, sizeof(rbuf), "badarg", "sensor", corr_id, replyFn, ctx);
                    _robot.driveController.cancel(_robot.systemTime(), replyFn, ctx);
                    return;
                }
                _robot.driveController.activeCmd().addStop(makeSensorStop(ch, thr, cmp));
                break;
            }
        }

        char body[48];
        snprintf(body, sizeof(body), "heading=%d eps=%d", heading_cdeg, eps_cdeg);
        replyOK(rbuf, sizeof(rbuf), "turn", body, corr_id, replyFn, ctx);
        return;
    }

    // ── VW — body-twist velocity drive (watchdogged, MotionCommand-based) ────
    // VW <v> <omega_mrads> [#id]  → OK vw v=<v> omega=<omega_mrads> [#id]
    // Converts (v mm/s, omega mrad/s) to body twist; configures a MotionCommand
    // with a TIME stop condition at sTimeoutMs (keepalive watchdog).
    // omega on wire: milli-radians/s (integer); converted to rad/s at boundary.
    //
    // Re-sent VW packets act as keepalives:
    //   - If a VW MotionCommand is already active, call setTarget to update
    //     the target and re-arm the TIME baseline without restarting the ramp.
    //   - Otherwise, start a fresh MotionCommand via beginVelocity.
    //
    // On keepalive loss: TIME condition fires → SOFT ramp to zero → EVT safety_stop.
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
        uint32_t now = _robot.systemTime();

        if (_robot.driveController.hasActiveCommand()) {
            // Keepalive re-send: update target and re-arm TIME baseline.
            // This avoids resetting the profiler ramp on every VW packet.
            _robot.driveController.activeCmd().setTarget((float)v, omega_rads);
        } else {
            // New VW command: configure MotionCommand from scratch.
            _robot.driveController.beginVelocity((float)v, omega_rads, now,
                                                 _robot.state.target,
                                                 replyFn, ctx, corr_id);
        }

        char body[32];
        snprintf(body, sizeof(body), "v=%d omega=%d", v, omega);
        replyOK(rbuf, sizeof(rbuf), "vw", body, corr_id, replyFn, ctx);
        return;
    }

    // ── RF — radio channel (frequency band); group is always 10 ──────────────
    // RF        → OK rf chan=<n> group=10        (query)
    // RF <n>    → OK rf chan=<n> group=10        (set + persist, 0..83)
    //
    // The channel persists in flash (uBit.storage) and is applied immediately.
    // WARNING: re-tuning over the radio drops the link the instant it takes
    // effect (the relay stays on the old channel) — send `RF <n>` over USB
    // serial, or set the channel with the on-board buttons at boot. The OK
    // reply is sent BEFORE re-tuning so it still reaches the host on the old
    // channel; the new channel takes effect right after.
    if (strcmp(verb, "RF") == 0) {
        if (_sched == nullptr) {
            replyErr(rbuf, sizeof(rbuf), "noradio", nullptr, corr_id, replyFn, ctx);
            return;
        }
        Radio& radio = _sched->comm().radio();

        if (ntok < 2) {
            // Query.
            char body[32];
            snprintf(body, sizeof(body), "chan=%d group=%d",
                     radio.channel(), radiochan::kGroup);
            replyOK(rbuf, sizeof(rbuf), "rf", body, corr_id, replyFn, ctx);
            return;
        }

        int ch = atoi(tokens[1]);
        if (ch < radiochan::kMin || ch > radiochan::kMax) {
            replyErr(rbuf, sizeof(rbuf), "range", "chan", corr_id, replyFn, ctx);
            return;
        }

        // Persist first, then reply on the OLD channel, then re-tune.
        radiochan::save(_sched->uBit().storage, ch);
        char body[32];
        snprintf(body, sizeof(body), "chan=%d group=%d", ch, radiochan::kGroup);
        replyOK(rbuf, sizeof(rbuf), "rf", body, corr_id, replyFn, ctx);
        radio.setChannel(ch);
        return;
    }

    // ── X — cancel active MotionCommand (hard stop) ──────────────────────────
    // X  → OK x
    // If a MotionCommand (e.g. VW) is active, HARD-cancels it: ramp zeroed,
    // MotionCommand emits EVT cancelled, mode goes IDLE. If no command is
    // active, _mc.stop() is still called (motor safety) but no EVT is emitted.
    if (strcmp(verb, "X") == 0) {
        { uint32_t now = _robot.systemTime(); _robot.driveController.cancel(now, replyFn, ctx); }
        replyOK(rbuf, sizeof(rbuf), "x", nullptr, corr_id, replyFn, ctx);
        return;
    }

    // ── STOP — stop motors immediately (alias for X; preserved for backward compat) ─
    // STOP  → OK stop
    // Routes through DriveController::cancel() just like X so any active
    // MotionCommand is torn down cleanly. Replies OK stop (not OK x).
    if (strcmp(verb, "STOP") == 0) {
        { uint32_t now = _robot.systemTime(); _robot.driveController.cancel(now, replyFn, ctx); }
        replyOK(rbuf, sizeof(rbuf), "stop", nullptr, corr_id, replyFn, ctx);
        return;
    }

    // ── GRIP — gripper control ────────────────────────────────────────────────
    // GRIP <deg>  → OK grip deg=<deg>
    // GRIP        → OK grip deg=<current>
    if (strcmp(verb, "GRIP") == 0) {
        int32_t deg;
        if (ntok >= 2) {
            deg = (int32_t)atoi(tokens[1]);
            if (deg < 0 || deg > 180) {
                replyErr(rbuf, sizeof(rbuf), "range", "deg", corr_id, replyFn, ctx);
                return;
            }
            { uint8_t clamped = (deg < 0) ? 0 : (deg > 180) ? 180 : (uint8_t)deg; _robot.gripper.setAngle(clamped); }
        } else {
            deg = _robot.gripper.currentAngle();
        }
        char body[24];
        snprintf(body, sizeof(body), "deg=%d", (int)deg);
        replyOK(rbuf, sizeof(rbuf), "grip", body, corr_id, replyFn, ctx);
        return;
    }

    // ── ZERO — zero encoders and/or odometry ─────────────────────────────────
    // ZERO enc         → OK zero enc
    // ZERO pose        → OK zero pose
    // ZERO enc pose    → OK zero enc pose
    if (strcmp(verb, "ZERO") == 0) {
        if (ntok < 2) {
            replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
            return;
        }
        bool doEnc  = false;
        bool doPose = false;
        for (int i = 1; i < ntok; ++i) {
            if (strcmp(tokens[i], "enc") == 0)  doEnc  = true;
            if (strcmp(tokens[i], "pose") == 0) doPose = true;
        }
        if (!doEnc && !doPose) {
            replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
            return;
        }
        if (doEnc)  _robot.motorController.resetEncoderAccumulators();
        if (doPose) _robot.odometry.zero(_robot.state.inputs);
        // Build body: "enc", "pose", or "enc pose"
        char body[16];
        if (doEnc && doPose)       snprintf(body, sizeof(body), "enc pose");
        else if (doEnc)            snprintf(body, sizeof(body), "enc");
        else                       snprintf(body, sizeof(body), "pose");
        replyOK(rbuf, sizeof(rbuf), "zero", body, corr_id, replyFn, ctx);
        return;
    }

    // ── OI — OTOS init ────────────────────────────────────────────────────────
    // OI  → OK oi
    if (strcmp(verb, "OI") == 0) {
        if (!_robot.otos.is_initialized()) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "oi", corr_id, replyFn, ctx);
            return;
        }
        _robot.otos.init();
        replyOK(rbuf, sizeof(rbuf), "oi", nullptr, corr_id, replyFn, ctx);
        return;
    }

    // ── OZ — OTOS zero (reset tracking) ──────────────────────────────────────
    // OZ  → OK oz
    if (strcmp(verb, "OZ") == 0) {
        if (!_robot.otos.is_initialized()) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "oz", corr_id, replyFn, ctx);
            return;
        }
        _robot.otos.setPositionRaw(0, 0, 0);
        replyOK(rbuf, sizeof(rbuf), "oz", nullptr, corr_id, replyFn, ctx);
        return;
    }

    // ── OR — OTOS reset tracking ──────────────────────────────────────────────
    // OR  → OK or
    if (strcmp(verb, "OR") == 0) {
        if (!_robot.otos.is_initialized()) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "or", corr_id, replyFn, ctx);
            return;
        }
        _robot.otos.resetTracking();
        replyOK(rbuf, sizeof(rbuf), "or", nullptr, corr_id, replyFn, ctx);
        return;
    }

    // ── OP — OTOS read raw position (LSB units, debug cross-check) ───────────
    // OP  → OK rawpos x=<x> y=<y> h=<h>  (values are raw OTOS LSB, not mm)
    // 1 LSB ≈ 0.305 mm for position; TLM pose= is fused odometry in mm/cdeg.
    if (strcmp(verb, "OP") == 0) {
        if (!_robot.otos.is_initialized()) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "op", corr_id, replyFn, ctx);
            return;
        }
        int16_t ox = 0, oy = 0, oh = 0;
        _robot.otos.getPositionRaw(ox, oy, oh);
        char body[64];
        snprintf(body, sizeof(body), "x=%d y=%d h=%d (raw LSB)", (int)ox, (int)oy, (int)oh);
        replyOK(rbuf, sizeof(rbuf), "rawpos", body, corr_id, replyFn, ctx);
        return;
    }

    // ── OV — OTOS set position ────────────────────────────────────────────────
    // OV <x> <y> <h>  → OK setpos x=<x> y=<y> h=<h>
    if (strcmp(verb, "OV") == 0) {
        if (!_robot.otos.is_initialized()) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "ov", corr_id, replyFn, ctx);
            return;
        }
        if (ntok < 4) {
            replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
            return;
        }
        int16_t ox = (int16_t)atoi(tokens[1]);
        int16_t oy = (int16_t)atoi(tokens[2]);
        int16_t oh = (int16_t)atoi(tokens[3]);
        _robot.otos.setPositionRaw(ox, oy, oh);
        char body[48];
        snprintf(body, sizeof(body), "x=%d y=%d h=%d", (int)ox, (int)oy, (int)oh);
        replyOK(rbuf, sizeof(rbuf), "setpos", body, corr_id, replyFn, ctx);
        return;
    }

    // ── OL — OTOS linear scalar ───────────────────────────────────────────────
    // OL        → OK linear scalar=<val>
    // OL <val>  → OK linear scalar=<val>
    if (strcmp(verb, "OL") == 0) {
        if (!_robot.otos.is_initialized()) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "ol", corr_id, replyFn, ctx);
            return;
        }
        if (ntok >= 2) {
            int8_t val = (int8_t)atoi(tokens[1]);
            _robot.otos.setLinearScalar(val);
        }
        int8_t val = _robot.otos.getLinearScalar();
        char body[24];
        snprintf(body, sizeof(body), "scalar=%d", (int)val);
        replyOK(rbuf, sizeof(rbuf), "linear", body, corr_id, replyFn, ctx);
        return;
    }

    // ── OA — OTOS angular scalar ──────────────────────────────────────────────
    // OA        → OK angular scalar=<val>
    // OA <val>  → OK angular scalar=<val>
    if (strcmp(verb, "OA") == 0) {
        if (!_robot.otos.is_initialized()) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "oa", corr_id, replyFn, ctx);
            return;
        }
        if (ntok >= 2) {
            int8_t val = (int8_t)atoi(tokens[1]);
            _robot.otos.setAngularScalar(val);
        }
        int8_t val = _robot.otos.getAngularScalar();
        char body[24];
        snprintf(body, sizeof(body), "scalar=%d", (int)val);
        replyOK(rbuf, sizeof(rbuf), "angular", body, corr_id, replyFn, ctx);
        return;
    }

    // ── P — digital port read/write ───────────────────────────────────────────
    // P <port>        → OK port p=<port> v=<val>  (read)
    // P <port> <val>  → OK port p=<port> v=<val>  (write)
    if (strcmp(verb, "P") == 0) {
        if (ntok < 2) {
            replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
            return;
        }
        int port = atoi(tokens[1]);
        if (port < 1 || port > 4) {
            replyErr(rbuf, sizeof(rbuf), "range", "port", corr_id, replyFn, ctx);
            return;
        }
        int val;
        if (ntok >= 3) {
            val = atoi(tokens[2]);
            _robot.portio.setDigital((uint8_t)port, val != 0);
        } else {
            val = _robot.portio.readDigital((uint8_t)port);
        }
        char body[24];
        snprintf(body, sizeof(body), "p=%d v=%d", port, val);
        replyOK(rbuf, sizeof(rbuf), "port", body, corr_id, replyFn, ctx);
        return;
    }

    // ── PA — analog port read/write ───────────────────────────────────────────
    // PA <port>        → OK aport p=<port> v=<val>  (read)
    // PA <port> <val>  → OK aport p=<port> v=<val>  (write)
    if (strcmp(verb, "PA") == 0) {
        if (ntok < 2) {
            replyErr(rbuf, sizeof(rbuf), "badarg", nullptr, corr_id, replyFn, ctx);
            return;
        }
        int port = atoi(tokens[1]);
        if (port < 1 || port > 4) {
            replyErr(rbuf, sizeof(rbuf), "range", "port", corr_id, replyFn, ctx);
            return;
        }
        int val;
        if (ntok >= 3) {
            val = atoi(tokens[2]);
            if (val < 0 || val > 1023) {
                replyErr(rbuf, sizeof(rbuf), "range", "val", corr_id, replyFn, ctx);
                return;
            }
            _robot.portio.setAnalog((uint8_t)port, (uint16_t)val);
        } else {
            val = _robot.portio.readAnalog((uint8_t)port);
        }
        char body[24];
        snprintf(body, sizeof(body), "p=%d v=%d", port, val);
        replyOK(rbuf, sizeof(rbuf), "aport", body, corr_id, replyFn, ctx);
        return;
    }

    // ── Fallback — unrecognized verb ─────────────────────────────────────────
    replyErr(rbuf, sizeof(rbuf), "unknown", verb, corr_id, replyFn, ctx);
}
