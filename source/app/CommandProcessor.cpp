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
#include "DriveController.h"
#include "Config.h"
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

    // Emit OK set only if at least one key was applied.
    if (apos > 0) {
        applied[apos] = '\0';
        CommandProcessor::replyOK(rbuf, rbufSize, "set", applied, corr_id,
                                  replyFn, ctx);
    }
}

// ---------------------------------------------------------------------------
// process — v2 command dispatch
// ---------------------------------------------------------------------------

void CommandProcessor::process(const char* line, ReplyFn replyFn, void* ctx)
{
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
        if (_robot.otos())        addCap("otos");
        if (_robot.lineSensor())  addCap("line");
        if (_robot.colorSensor()) addCap("color");
        if (_robot.servo())       addCap("gripper");
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
                "PING ECHO ID VER HELP SET GET GET VEL STREAM SNAP S T D G VW STOP GRIP ZERO OI OZ OR OP OV OL OA P PA",
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
            float vL = _robot.state().inputs.velLMms;
            float vR = _robot.state().inputs.velRMms;
            char body[48];
            snprintf(body, sizeof(body), "vel=%d:E,%d:E",
                     (int)vL, (int)vR);
            replyOK(rbuf, sizeof(rbuf), "get", body, corr_id, replyFn, ctx);
            return;
        }
        // Positional args (tokens[1..]) are the requested keys.
        // parseKV() would consume tokens that contain '='; GET only uses plain
        // key names, so pass the raw token list directly.
        handleGet(tokens, ntok, _robot.config(), rbuf, sizeof(rbuf),
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
        handleSet(kvs, nkv, _robot.config(), _robot.motor(),
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
                _robot.config().tlmFields = mask ? mask : TLM_FIELD_ALL;
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
                    if (_robot.config().tlmFields & kFieldNames[fi].bit) {
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
        _robot.config().tlmPeriodMs = ms;
        char body[32];
        snprintf(body, sizeof(body), "period=%d", (int)ms);
        replyOK(rbuf, sizeof(rbuf), "stream", body, corr_id, replyFn, ctx);
        return;
    }

    // ── SNAP ─────────────────────────────────────────────────────────────────
    // SNAP → (emits one immediate TLM frame on next tick, then OK snap)
    // The TLM frame is emitted by Robot::tick() when tlmSnapPending is set.
    if (strcmp(verb, "SNAP") == 0) {
        _robot.config().tlmSnapPending = true;
        replyOK(rbuf, sizeof(rbuf), "snap", nullptr, corr_id, replyFn, ctx);
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
        _robot.streamDrive((int32_t)l, (int32_t)r, replyFn, ctx);
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
        _robot.timedDrive((int32_t)l, (int32_t)r, (uint32_t)ms, replyFn, ctx, corr_id);
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
        _robot.goTo((float)x, (float)y, (float)speed, replyFn, ctx, corr_id);
        char body[64];
        snprintf(body, sizeof(body), "x=%d y=%d speed=%d", x, y, speed);
        replyOK(rbuf, sizeof(rbuf), "goto", body, corr_id, replyFn, ctx);
        return;
    }

    // ── VW — body-twist velocity drive (watchdogged) ─────────────────────────
    // VW <v> <omega_mrads> [#id]  → OK vw v=<v> omega=<omega_mrads> [#id]
    // Converts (v mm/s, omega mrad/s) → wheel setpoints via BodyKinematics::inverse()
    // then enters STREAMING mode — same watchdog/safety_stop as S command.
    // omega on wire: milli-radians/s (integer); converted to rad/s at firmware boundary.
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
        _robot.velocityDrive((float)v, omega_rads, replyFn, ctx, corr_id);
        char body[32];
        snprintf(body, sizeof(body), "v=%d omega=%d", v, omega);
        replyOK(rbuf, sizeof(rbuf), "vw", body, corr_id, replyFn, ctx);
        return;
    }

    // ── STOP — stop motors immediately ───────────────────────────────────────
    // STOP  → OK stop
    if (strcmp(verb, "STOP") == 0) {
        _robot.stop();
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
            _robot.setGripperAngle(deg);
        } else {
            deg = _robot.gripperAngle();
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
        if (doEnc)  _robot.zeroEncoders();
        if (doPose) _robot.zeroOdometry();
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
        OtosSensor* otos = _robot.otos();
        if (!otos) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "oi", corr_id, replyFn, ctx);
            return;
        }
        otos->init();
        replyOK(rbuf, sizeof(rbuf), "oi", nullptr, corr_id, replyFn, ctx);
        return;
    }

    // ── OZ — OTOS zero (reset tracking) ──────────────────────────────────────
    // OZ  → OK oz
    if (strcmp(verb, "OZ") == 0) {
        OtosSensor* otos = _robot.otos();
        if (!otos) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "oz", corr_id, replyFn, ctx);
            return;
        }
        otos->setPositionRaw(0, 0, 0);
        replyOK(rbuf, sizeof(rbuf), "oz", nullptr, corr_id, replyFn, ctx);
        return;
    }

    // ── OR — OTOS reset tracking ──────────────────────────────────────────────
    // OR  → OK or
    if (strcmp(verb, "OR") == 0) {
        OtosSensor* otos = _robot.otos();
        if (!otos) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "or", corr_id, replyFn, ctx);
            return;
        }
        otos->resetTracking();
        replyOK(rbuf, sizeof(rbuf), "or", nullptr, corr_id, replyFn, ctx);
        return;
    }

    // ── OP — OTOS read raw position (LSB units, debug cross-check) ───────────
    // OP  → OK rawpos x=<x> y=<y> h=<h>  (values are raw OTOS LSB, not mm)
    // 1 LSB ≈ 0.305 mm for position; TLM pose= is fused odometry in mm/cdeg.
    if (strcmp(verb, "OP") == 0) {
        OtosSensor* otos = _robot.otos();
        if (!otos) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "op", corr_id, replyFn, ctx);
            return;
        }
        int16_t ox = 0, oy = 0, oh = 0;
        otos->getPositionRaw(ox, oy, oh);
        char body[64];
        snprintf(body, sizeof(body), "x=%d y=%d h=%d (raw LSB)", (int)ox, (int)oy, (int)oh);
        replyOK(rbuf, sizeof(rbuf), "rawpos", body, corr_id, replyFn, ctx);
        return;
    }

    // ── OV — OTOS set position ────────────────────────────────────────────────
    // OV <x> <y> <h>  → OK setpos x=<x> y=<y> h=<h>
    if (strcmp(verb, "OV") == 0) {
        OtosSensor* otos = _robot.otos();
        if (!otos) {
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
        otos->setPositionRaw(ox, oy, oh);
        char body[48];
        snprintf(body, sizeof(body), "x=%d y=%d h=%d", (int)ox, (int)oy, (int)oh);
        replyOK(rbuf, sizeof(rbuf), "setpos", body, corr_id, replyFn, ctx);
        return;
    }

    // ── OL — OTOS linear scalar ───────────────────────────────────────────────
    // OL        → OK linear scalar=<val>
    // OL <val>  → OK linear scalar=<val>
    if (strcmp(verb, "OL") == 0) {
        OtosSensor* otos = _robot.otos();
        if (!otos) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "ol", corr_id, replyFn, ctx);
            return;
        }
        if (ntok >= 2) {
            int8_t val = (int8_t)atoi(tokens[1]);
            otos->setLinearScalar(val);
        }
        int8_t val = otos->getLinearScalar();
        char body[24];
        snprintf(body, sizeof(body), "scalar=%d", (int)val);
        replyOK(rbuf, sizeof(rbuf), "linear", body, corr_id, replyFn, ctx);
        return;
    }

    // ── OA — OTOS angular scalar ──────────────────────────────────────────────
    // OA        → OK angular scalar=<val>
    // OA <val>  → OK angular scalar=<val>
    if (strcmp(verb, "OA") == 0) {
        OtosSensor* otos = _robot.otos();
        if (!otos) {
            replyErr(rbuf, sizeof(rbuf), "nodev", "oa", corr_id, replyFn, ctx);
            return;
        }
        if (ntok >= 2) {
            int8_t val = (int8_t)atoi(tokens[1]);
            otos->setAngularScalar(val);
        }
        int8_t val = otos->getAngularScalar();
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
            _robot.portIO().setDigital((uint8_t)port, val != 0);
        } else {
            val = _robot.portIO().readDigital((uint8_t)port);
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
            _robot.portIO().setAnalog((uint8_t)port, (uint16_t)val);
        } else {
            val = _robot.portIO().readAnalog((uint8_t)port);
        }
        char body[24];
        snprintf(body, sizeof(body), "p=%d v=%d", port, val);
        replyOK(rbuf, sizeof(rbuf), "aport", body, corr_id, replyFn, ctx);
        return;
    }

    // ── Fallback — unrecognized verb ─────────────────────────────────────────
    replyErr(rbuf, sizeof(rbuf), "unknown", verb, corr_id, replyFn, ctx);
}
