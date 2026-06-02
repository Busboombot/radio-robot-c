// CommandProcessor.cpp — protocol v2 wire-protocol parser and dispatcher.
//
// Sprint 009, Ticket 002: v2 tokenizer, verb-only uppercasing, #id
// correlation, OK/ERR/EVT/TLM/CFG/ID response taxonomy.
// Legacy packed parsing (parseSignedArgs, K*, S+/T+/D+, etc.) removed.
// Announcer removed; HELLO returns ERR unknown.
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
    CFG_F("ml",         mmPerDegL),
    CFG_F("mr",         mmPerDegR),
    // Feed-forward and motor scale factors
    CFG_F("kff",        kFF),
    CFG_F("klf",        kScaleLF),
    CFG_F("klb",        kScaleLB),
    CFG_F("krf",        kScaleRF),
    CFG_F("krb",        kScaleRB),
    // Slower-wheel adjustment
    CFG_F("adjThr",     kAdjThreshold),
    CFG_F("adjGain",    kAdjGain),
    // Geometry — stored as float, displayed as integer (mm)
    CFG_FI("tw",        trackwidthMm),
    // Ratio PID gains
    CFG_F("pid.kp",     ratioPidKp),
    CFG_F("pid.ki",     ratioPidKi),
    CFG_F("pid.kd",     ratioPidKd),
    CFG_F("pid.max",    ratioPidMax),
    // Go-to tolerances — stored as float, displayed as integer (mm)
    CFG_FI("turnThr",   turnThresholdMm),
    CFG_FI("doneTol",   doneTolMm),
    // Command scaling
    CFG_F("distScale",  distScale),
    CFG_F("turnScale",  turnScale),
    // Timing and speed (int32_t fields)
    CFG_I("minSpeed",   minSpeedMms),
    CFG_I("sTimeout",   sTimeoutMs),
    CFG_I("tick",       tickMs),
    CFG_I("tlmPeriod",  tlmPeriodMs),
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
    char line[512];
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
                "PING ECHO ID VER HELP SET GET STREAM SNAP S T D G STOP GRIP ZERO",
                corr_id, replyFn, ctx);
        return;
    }

    // ── GET ──────────────────────────────────────────────────────────────────
    // GET                → CFG <all key=value pairs>
    // GET ml pid.kp      → CFG ml=0.487 pid.kp=2.0
    // GET ml #9          → CFG ml=0.487 #9
    if (strcmp(verb, "GET") == 0) {
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

    // ── Fallback — unrecognized verb ─────────────────────────────────────────
    replyErr(rbuf, sizeof(rbuf), "unknown", verb, corr_id, replyFn, ctx);
}
