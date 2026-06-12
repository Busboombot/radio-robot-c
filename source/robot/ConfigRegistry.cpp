// ConfigRegistry.cpp — config key-to-field registry for the robot.
//
// Moved from CommandProcessor.cpp (Sprint 019, Ticket 002).
// kRegistry[] maps friendly key names to RobotConfig fields by byte offset.
// handleGet and handleSet implement the HandlerFn-compatible GET/SET wire
// protocol handlers; they are called from the CommandProcessor switch until
// the composable dispatch table is in place (T010/T011).

#include "ConfigRegistry.h"
#include "../app/CommandProcessor.h"
#include "../control/MotorController.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

// ---------------------------------------------------------------------------
// Helper macros so the table stays readable.
// ---------------------------------------------------------------------------
#define CFG_F(k, field)  { k, CFG_FLOAT,        offsetof(RobotConfig, field) }
#define CFG_I(k, field)  { k, CFG_INT,           offsetof(RobotConfig, field) }
#define CFG_FI(k, field) { k, CFG_FLOAT_AS_INT,  offsetof(RobotConfig, field) }

const ConfigEntry kRegistry[] = {
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
    //   velKp  <-> "vel.kP"   velKi  <-> "vel.kI"   velKff <-> "vel.kFF"
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
    CFG_FI("turnGate",    turnInPlaceGate),   // wire: integer degrees; MotionController converts to radians at use-site
    CFG_FI("arriveTol",   arriveTolMm),       // wire: integer mm
    // Body motion limits (Sprint 017 -- BodyVelocityController)
    CFG_F("vBodyMax",    vBodyMax),           // body forward speed ceiling, mm/s
    CFG_F("yawRateMax",  yawRateMax),         // yaw rate ceiling, deg/s
    CFG_F("yawAccMax",   yawAccMax),          // yaw acceleration limit, deg/s^2
    CFG_F("jMax",        jMax),               // linear jerk limit, mm/s^3 (0=trapezoid)
    CFG_F("yawJerkMax",  yawJerkMax),         // yaw jerk limit, deg/s^3   (0=trapezoid)
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
    // EKF heading fusion noise (sprint 024-004)
    CFG_F("ekfRHead",     ekfROtosTheta),
};

#undef CFG_F
#undef CFG_I
#undef CFG_FI

const int kRegistryCount = (int)(sizeof(kRegistry) / sizeof(kRegistry[0]));

// ---------------------------------------------------------------------------
// appendKeyValue — append one key=value pair to a string buffer.
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
// handleGet — HandlerFn-compatible GET handler.
//
// args.args[0..args.count-1].sval carries the requested key names.
// An empty args list (count == 0) means dump all keys.
// handlerCtx is cast to CfgCtx*.
//
// Emits one CFG response line. For each unknown key, also emits ERR badkey.
// ---------------------------------------------------------------------------

void handleGet(const ArgList& args, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    const CfgCtx* ctx = reinterpret_cast<const CfgCtx*>(handlerCtx);
    const RobotConfig& cfg = *ctx->cfg;

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

    // replyCtx is the opaque context forwarded to replyFn (used for ERR replies).
    // For ERR replies we need a temporary buffer.
    char rbuf[128];

    bool anyKey = (args.count == 0);  // no args -> dump all

    if (anyKey) {
        // Dump all registry entries.
        for (int i = 0; i < kRegistryCount && rem > 2; ++i) {
            line[pos++] = ' '; --rem;
            int w = appendKeyValue(line + pos, rem, kRegistry[i], cfg);
            pos += w; rem -= w;
        }
    } else {
        // Dump only the requested keys.
        for (int t = 0; t < args.count && rem > 2; ++t) {
            const char* reqKey = args.args[t].sval;
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
                // Unknown key in GET -- reply with ERR for that key but continue.
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badkey", reqKey, corrId,
                                           replyFn, replyCtx);
            }
        }
    }

    // Append correlation id if present.
    if (corrId && corrId[0] != '\0' && rem > 3) {
        int w = snprintf(line + pos, (size_t)rem, " #%s", corrId);
        if (w > 0 && w < rem) { pos += w; rem -= w; }
    }

    line[pos] = '\0';
    replyFn(line, replyCtx);
}

// ---------------------------------------------------------------------------
// validateConfig — check RobotConfig invariants before atomic commit.
//
// Returns true if the candidate config is valid.  On failure, sets *badKey to
// a short description of the first failing invariant (key name or "key=value"
// form) and returns false.  Only checks invariants whose violation causes a
// known runtime failure:
//
//   tw > 0          — trackwidthMm divides in odometry arc/heading; zero →
//                     division by zero.
//   ctrlPeriod > 0  — controlPeriodMs is cast to uint32 sleep; zero or
//                     negative wraps to a huge value, starving the control
//                     fiber.
//   vWheelMax > steerHeadroom  — effective ceiling = vWheelMax-steerHeadroom;
//                     when ≤ 0 the saturation ceiling goes negative, clamping
//                     output to a negative value and inverting the wheel.
//   rotationalSlip in [0.5, 1.0] — values outside this range produce
//                     nonsensical arc estimates that break odometry.
// ---------------------------------------------------------------------------

static bool validateConfig(const RobotConfig& c, const char** badKey)
{
    if (c.trackwidthMm <= 0.0f) {
        *badKey = "tw";
        return false;
    }
    if (c.controlPeriodMs <= 0) {
        *badKey = "ctrlPeriod";
        return false;
    }
    if (c.vWheelMax <= c.steerHeadroom) {
        *badKey = "vWheelMax";
        return false;
    }
    if (c.rotationalSlip < 0.5f || c.rotationalSlip > 1.0f) {
        *badKey = "rotSlip";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// handleSet — HandlerFn-compatible SET handler.
//
// args.args[0..args.count-1].sval carries "key=value" strings (one per pair).
// handlerCtx is cast to CfgCtx*.
//
// Parse/validation strategy (028-004):
//   1. Replace atof/atoi with strtof/strtol end-pointer checks.  A value is
//      rejected (ERR badval <key>) if the end-pointer is not at the end of the
//      string, or the string is empty.
//   2. Build a candidate RobotConfig copy (candidate = cfg) and apply all valid
//      keys to the candidate, NOT to cfg.
//   3. After processing all keys, call validateConfig(candidate).  If it
//      fails, emit ERR badval <key>=<value> and return — cfg is unchanged.
//   4. Only if validateConfig passes: cfg = candidate; emit OK set <applied>.
//
// Emits ERR badval <key> per parse failure (non-numeric / empty value).
// Emits ERR badkey <key> per unknown key.
// Calls MotorController::updatePidGains / updateVelGains when relevant
// params change (only after successful commit).
// ---------------------------------------------------------------------------

void handleSet(const ArgList& args, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    CfgCtx* ctx = reinterpret_cast<CfgCtx*>(handlerCtx);
    RobotConfig& cfg = *ctx->cfg;
    MotorController& mc = *ctx->mc;

    // Candidate config — all valid key writes go here; committed atomically.
    RobotConfig candidate = cfg;

    // Build "OK set <applied keys>" body.
    char applied[480];
    int apos = 0;
    int arem = (int)sizeof(applied);

    bool pidChanged = false;
    bool velChanged = false;
    bool anyParseErr = false;   // set on strtof/strtol end-pointer failure

    // Each arg's sval holds "key=value"; split on '='.
    for (int i = 0; i < args.count; ++i) {
        // Copy sval so we can split in place.
        char kvbuf[64];
        int kvlen = 0;
        for (const char* p = args.args[i].sval;
             *p && kvlen < (int)sizeof(kvbuf) - 1; ++p, ++kvlen) {
            kvbuf[kvlen] = *p;
        }
        kvbuf[kvlen] = '\0';

        char* eq = strchr(kvbuf, '=');
        if (!eq) continue;  // malformed: no '='; skip

        *eq = '\0';
        const char* k = kvbuf;
        const char* v = eq + 1;

        if (!k || k[0] == '\0') continue;  // no key

        // Find in registry.
        const ConfigEntry* entry = nullptr;
        for (int r = 0; r < kRegistryCount; ++r) {
            if (strcmp(kRegistry[r].key, k) == 0) {
                entry = &kRegistry[r];
                break;
            }
        }

        if (!entry) {
            // Unknown key -- emit ERR and continue processing remaining keys.
            char rbuf[128];
            CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                       "badkey", k, corrId, replyFn, replyCtx);
            anyParseErr = true;
            continue;
        }

        // Typed parse with end-pointer validation.
        // Reject empty string or any non-numeric suffix.
        char* base = reinterpret_cast<char*>(&candidate);
        switch (entry->type) {
        case CFG_FLOAT: {
            if (v[0] == '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            char* endp = nullptr;
            float fv = strtof(v, &endp);
            if (endp == v || *endp != '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            memcpy(base + entry->offset, &fv, sizeof(float));
            break;
        }
        case CFG_INT: {
            if (v[0] == '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            char* endp = nullptr;
            long lv = strtol(v, &endp, 10);
            if (endp == v || *endp != '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            int32_t iv = (int32_t)lv;
            memcpy(base + entry->offset, &iv, sizeof(int32_t));
            break;
        }
        case CFG_FLOAT_AS_INT: {
            if (v[0] == '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            char* endp = nullptr;
            long lv = strtol(v, &endp, 10);
            if (endp == v || *endp != '\0') {
                char rbuf[128];
                CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                           "badval", k, corrId, replyFn, replyCtx);
                anyParseErr = true;
                continue;
            }
            float fv = (float)lv;
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

    // If any parse / badkey error occurred, do not commit — cfg is unchanged.
    if (anyParseErr) {
        return;
    }

    // Validate the candidate config before committing.
    if (apos > 0) {
        const char* badKey = nullptr;
        if (!validateConfig(candidate, &badKey)) {
            // Find the failing key's current value in the candidate for the
            // ERR badval key=value detail.
            char detail[64];
            // Look up the failing key in the registry to format its candidate value.
            bool found = false;
            for (int r = 0; r < kRegistryCount && !found; ++r) {
                if (strcmp(kRegistry[r].key, badKey) == 0) {
                    const char* cbase = reinterpret_cast<const char*>(&candidate);
                    switch (kRegistry[r].type) {
                    case CFG_FLOAT: {
                        float fv;
                        memcpy(&fv, cbase + kRegistry[r].offset, sizeof(float));
                        snprintf(detail, sizeof(detail), "%s=%.3f", badKey, (double)fv);
                        break;
                    }
                    case CFG_INT: {
                        int32_t iv;
                        memcpy(&iv, cbase + kRegistry[r].offset, sizeof(int32_t));
                        snprintf(detail, sizeof(detail), "%s=%d", badKey, (int)iv);
                        break;
                    }
                    case CFG_FLOAT_AS_INT: {
                        float fv;
                        memcpy(&fv, cbase + kRegistry[r].offset, sizeof(float));
                        snprintf(detail, sizeof(detail), "%s=%d", badKey, (int)fv);
                        break;
                    }
                    }
                    found = true;
                }
            }
            if (!found) {
                // Fallback: key name only (should not happen with invariant keys).
                snprintf(detail, sizeof(detail), "%s", badKey);
            }
            char rbuf[128];
            CommandProcessor::replyErr(rbuf, (int)sizeof(rbuf),
                                       "badval", detail, corrId, replyFn, replyCtx);
            return;  // cfg unchanged
        }

        // Validation passed — commit atomically.
        cfg = candidate;

        // Update PID gains in MotorController if any PID param changed.
        if (pidChanged) {
            mc.updatePidGains(cfg.ratioPidKp, cfg.ratioPidKi,
                              cfg.ratioPidKd, cfg.ratioPidMax);
        }
        if (velChanged) {
            mc.updateVelGains(cfg);
        }

        applied[apos] = '\0';
        char rbuf[520];
        CommandProcessor::replyOK(rbuf, (int)sizeof(rbuf), "set", applied, corrId,
                                  replyFn, replyCtx);
    }
}
