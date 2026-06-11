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
// handleSet — HandlerFn-compatible SET handler.
//
// args.args[0..args.count-1].sval carries "key=value" strings (one per pair).
// handlerCtx is cast to CfgCtx*.
//
// Emits OK set <applied> once (if at least one key was applied).
// Emits ERR badkey <key> per unknown key.
// Calls MotorController::updatePidGains / updateVelGains when relevant
// params change.
// ---------------------------------------------------------------------------

void handleSet(const ArgList& args, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    CfgCtx* ctx = reinterpret_cast<CfgCtx*>(handlerCtx);
    RobotConfig& cfg = *ctx->cfg;
    MotorController& mc = *ctx->mc;

    // Build "OK set <applied keys>" body.
    char applied[480];
    int apos = 0;
    int arem = (int)sizeof(applied);

    bool pidChanged = false;
    bool velChanged = false;

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
        char rbuf[520];
        CommandProcessor::replyOK(rbuf, (int)sizeof(rbuf), "set", applied, corrId,
                                  replyFn, replyCtx);
    }
}
