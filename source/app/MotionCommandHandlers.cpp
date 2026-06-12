// MotionCommandHandlers.cpp — motion command parsing, conversion, and reply
// formatting, extracted from MotionController.cpp (sprint 026, ticket 002).
//
// Dependency direction: app/ → control/.  All CommandProcessor reply calls
// live here.  MotionController.cpp no longer includes CommandProcessor.h or
// CommandQueue.h after this extraction.
//
// D11 suppression rule (sprint 026-002):
//   Each converter handler (S, T, D, G, R, TURN, RT) calls replyOK ONCE
//   and then pushes a VW ParsedCommand onto the queue.  When handleVW is
//   later dispatched from the queue for a converter push, it MUST NOT call
//   replyOK again — the converter already replied.
//   Only the open-ended VW branch (no stop params) in handleVW emits replyOK.
//   All stop-param branches in handleVW call begin*() and return WITHOUT
//   calling replyOK.

#include "MotionCommandHandlers.h"
#include "MotionController.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "CommandQueue.h"
#include "BodyKinematics.h"
#include "StopCondition.h"
#include "CommandTypes.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
// Forward declarations for static handler/parser functions used by
// getMotionCommands() (defined later in this file).
// ---------------------------------------------------------------------------
static ParseResult parseVW(const char* const* tokens, int ntokens,
                           const KVPair* kvs, int nkv);
static void handleVW(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx);

// ---------------------------------------------------------------------------
// mc_parseSensorToken — parse "sensor=<ch>:<op>:<thr>" into channel, cmp,
// threshold.
//
// Returns true on success; false on any parse/lookup failure.
// ---------------------------------------------------------------------------
static bool mc_parseSensorToken(const char* value,
                                uint8_t& ch_out, float& thr_out,
                                StopCondition::Cmp& cmp_out)
{
    char buf[32];
    int vlen = 0;
    for (const char* p = value; *p && vlen < (int)sizeof(buf) - 1; ++p, ++vlen)
        buf[vlen] = *p;
    buf[vlen] = '\0';

    char* colon1 = strchr(buf, ':');
    if (!colon1) return false;
    *colon1 = '\0';
    const char* ch_name = buf;
    const char* rest    = colon1 + 1;

    char* colon2 = strchr(const_cast<char*>(rest), ':');
    if (!colon2) return false;
    *colon2 = '\0';
    const char* op_str  = rest;
    const char* thr_str = colon2 + 1;

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

    StopCondition::Cmp cmp;
    if (strcmp(op_str, "ge") == 0) {
        cmp = StopCondition::Cmp::GE;
    } else if (strcmp(op_str, "le") == 0) {
        cmp = StopCondition::Cmp::LE;
    } else {
        return false;
    }

    int thr = atoi(thr_str);
    ch_out  = ch;
    thr_out = (float)thr;
    cmp_out = cmp;
    return true;
}

// ── Helper macro: set one INT arg ───────────────────────────────────────────
static inline void setIntArg(Argument& a, int v)
{
    a.type    = ArgType::INT;
    a.ival    = v;
    a.sval[0] = '\0';
}

// ── Helper: copy a sensor= KV value string into args[idx] as STR ───────────
static int packSensorArg(ArgList& out, int nextIdx,
                         const KVPair* kvs, int nkv)
{
    for (int i = 0; i < nkv; ++i) {
        if (kvs[i].key && strcmp(kvs[i].key, "sensor") == 0) {
            out.args[nextIdx].type = ArgType::STR;
            out.args[nextIdx].ival = 0;
            out.args[nextIdx].fval = 0.0f;
            int slen = 0;
            const char* src = kvs[i].value;
            while (*src && slen < (int)(sizeof(out.args[nextIdx].sval) - 1))
                out.args[nextIdx].sval[slen++] = *src++;
            out.args[nextIdx].sval[slen] = '\0';
            return nextIdx + 1;
        }
    }
    return nextIdx;  // no sensor= found
}

// ---------------------------------------------------------------------------
// pushVW — build a VW ParsedCommand and push_front it onto the queue.
//
// args must already contain args[0].ival=v (mm/s), args[1].ival=omega (mrad/s),
// and any stop params packed in args[2..] as ArgType::STR "key=value" strings.
//
// Returns false if the queue is null (caller falls back to direct begin*() call).
// ---------------------------------------------------------------------------
static bool pushVW(MotionCtx* ctx, const ArgList& args,
                   const char* corrId, ReplyFn replyFn, void* replyCtx)
{
    if (ctx->queue == nullptr) return false;
    ParsedCommand pc;
    pc.desc    = &ctx->vwDesc;
    pc.args    = args;
    pc.replyFn = replyFn;
    pc.replyCtx = replyCtx;
    int cidLen = 0;
    while (corrId && corrId[cidLen] != '\0' && cidLen < (int)(sizeof(pc.corrId) - 1)) {
        pc.corrId[cidLen] = corrId[cidLen];
        ++cidLen;
    }
    pc.corrId[cidLen] = '\0';
    return ctx->queue->push_front(pc);
}

// ── Helper: pack a "key=value" STR arg ─────────────────────────────────────
static int packKVArg(ArgList& out, int idx, const char* key, int ival)
{
    out.args[idx].type = ArgType::STR;
    out.args[idx].ival = 0;
    out.args[idx].fval = 0.0f;
    snprintf(out.args[idx].sval, sizeof(out.args[idx].sval), "%s=%d", key, ival);
    return idx + 1;
}

// ── Helper: scan args[2..] for a "key=value" string and return int value. ──
static int vwScanKV(const ArgList& args, const char* key, int defVal)
{
    int keyLen = 0;
    while (key[keyLen]) ++keyLen;
    for (int i = 2; i < args.count; ++i) {
        if (args.args[i].type != ArgType::STR) continue;
        const char* s = args.args[i].sval;
        int j = 0;
        while (j < keyLen && s[j] == key[j]) ++j;
        if (j == keyLen && s[j] == '=') {
            return atoi(s + j + 1);
        }
    }
    return defVal;
}

// ── Helper: check if a key is present in args[2..] ──────────────────────────
static bool vwHasKey(const ArgList& args, const char* key)
{
    int keyLen = 0;
    while (key[keyLen]) ++keyLen;
    for (int i = 2; i < args.count; ++i) {
        if (args.args[i].type != ArgType::STR) continue;
        const char* s = args.args[i].sval;
        int j = 0;
        while (j < keyLen && s[j] == key[j]) ++j;
        if (j == keyLen && s[j] == '=') return true;
    }
    return false;
}

// ── S ────────────────────────────────────────────────────────────────────────

static ParseResult parseS(const char* const* tokens, int ntokens,
                           const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int l = atoi(tokens[0]);
    int r = atoi(tokens[1]);
    if (l < -1000 || l > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "l"; return res;
    }
    if (r < -1000 || r > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "r"; return res;
    }
    res.ok = true;
    res.args.count = 2;
    setIntArg(res.args.args[0], l);
    setIntArg(res.args.args[1], r);
    return res;
}

// handleS — VW converter (no stop params → S/streaming mode via beginStream fallback).
//
// Computes (v, ω) from (l, r) via BodyKinematics::forward(), encodes as
// VW args[0]=v_mms, args[1]=omega_mrads (no stop params), and pushes to the
// queue. Falls back to beginStream() when queue is null (sim / unit test).
static void handleS(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int l = args.args[0].ival;
    int r = args.args[1].ival;

    if (ctx->queue != nullptr) {
        // Compute body twist via forward kinematics; encode as VW mrad/s integers.
        float v_mms, omega_rads;
        BodyKinematics::forward((float)l, (float)r, ctx->robot->config.trackwidthMm,
                                v_mms, omega_rads);
        int v_int     = (int)v_mms;
        int omega_int = (int)(omega_rads * 1000.0f);  // rad/s → mrad/s

        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], v_int);
        setIntArg(vwArgs.args[1], omega_int);
        // Pack a "stream=1" marker so handleVW routes via beginStream (seed
        // BVC immediately, no trapezoid ramp).  This preserves the original
        // S-command semantics on the queue path.
        vwArgs.count = packKVArg(vwArgs, 2, "stream", 1);

        char body[32];
        snprintf(body, sizeof(body), "l=%d r=%d", l, r);
        char rbuf[64];
        if (!pushVW(ctx, vwArgs, corrId, replyFn, replyCtx)) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full", nullptr, corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
    } else {
        // Queue not available (sim fallback, should not be reached since sim
        // now wires the queue): use direct beginStream().
        ctx->mc->beginStream((float)l, (float)r,
                             ctx->robot->systemTime(),
                             ctx->robot->state.target,
                             replyFn, replyCtx);
        char body[32];
        snprintf(body, sizeof(body), "l=%d r=%d", l, r);
        char rbuf[64];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
    }
}

// ── T ────────────────────────────────────────────────────────────────────────

static ParseResult parseT(const char* const* tokens, int ntokens,
                           const KVPair* kvs, int nkv)
{
    ParseResult res;
    if (ntokens < 3) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int l  = atoi(tokens[0]);
    int r  = atoi(tokens[1]);
    int ms = atoi(tokens[2]);
    if (l < -1000 || l > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "l"; return res;
    }
    if (r < -1000 || r > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "r"; return res;
    }
    if (ms < 1 || ms > 30000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "ms"; return res;
    }
    res.ok = true;
    res.args.count = 3;
    setIntArg(res.args.args[0], l);
    setIntArg(res.args.args[1], r);
    setIntArg(res.args.args[2], ms);
    // Pack optional sensor= into args[3].
    res.args.count = packSensorArg(res.args, 3, kvs, nkv);
    return res;
}

// handleT — VW converter with t=<ms> stop param.
//
// Computes (v, ω) from (l, r) via forward kinematics, builds VW args with
// "t=<ms>" stop param, and pushes to the queue.  Falls back to direct
// beginTimed() when queue is null.
static void handleT(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int l  = args.args[0].ival;
    int r  = args.args[1].ival;
    int ms = args.args[2].ival;

    if (ctx->queue != nullptr) {
        // N16 fix (030-009): validate sensor= BEFORE replying OK on the queue
        // path.  On the direct path, parse failure replies ERR and cancels.  The
        // queue path previously packed the raw token and forwarded it to handleVW,
        // which silently skipped the stop on parse failure after OK was already
        // sent.  Validate here so both paths reply ERR consistently.
        if (args.count >= 4) {
            uint8_t ch; float thr; StopCondition::Cmp cmp;
            if (!mc_parseSensorToken(args.args[3].sval, ch, thr, cmp)) {
                char rbuf[80];
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                           corrId, replyFn, replyCtx);
                return;
            }
        }

        float v_mms, omega_rads;
        BodyKinematics::forward((float)l, (float)r, ctx->robot->config.trackwidthMm,
                                v_mms, omega_rads);
        int v_int     = (int)v_mms;
        int omega_int = (int)(omega_rads * 1000.0f);

        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], v_int);
        setIntArg(vwArgs.args[1], omega_int);
        vwArgs.count = packKVArg(vwArgs, 2, "t", ms);

        // sensor= forwarding: pack into the VW args if present.
        if (args.count >= 4) {
            vwArgs.args[vwArgs.count] = args.args[3];  // copy STR sensor arg
            ++vwArgs.count;
        }

        char body[48];
        snprintf(body, sizeof(body), "l=%d r=%d ms=%d", l, r, ms);
        char rbuf[80];
        if (!pushVW(ctx, vwArgs, corrId, replyFn, replyCtx)) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full", nullptr, corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct beginTimed().
        ctx->mc->beginTimed((float)l, (float)r, (uint32_t)ms,
                            ctx->robot->systemTime(),
                            ctx->robot->state.target,
                            replyFn, replyCtx, corrId);
        // Optional sensor= stop condition (packed into args[3] by parseT).
        if (args.count >= 4) {
            uint8_t ch; float thr; StopCondition::Cmp cmp;
            if (!mc_parseSensorToken(args.args[3].sval, ch, thr, cmp)) {
                char rbuf[64];
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                           corrId, replyFn, replyCtx);
                ctx->mc->cancel(ctx->robot->systemTime(), replyFn, replyCtx);
                return;
            }
            ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
        }
        char body[48];
        snprintf(body, sizeof(body), "l=%d r=%d ms=%d", l, r, ms);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
    }
}

// ── D ────────────────────────────────────────────────────────────────────────

static ParseResult parseD(const char* const* tokens, int ntokens,
                           const KVPair* kvs, int nkv)
{
    ParseResult res;
    if (ntokens < 3) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int l  = atoi(tokens[0]);
    int r  = atoi(tokens[1]);
    int mm = atoi(tokens[2]);
    if (l < -1000 || l > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "l"; return res;
    }
    if (r < -1000 || r > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "r"; return res;
    }
    if (mm < 1 || mm > 10000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "mm"; return res;
    }
    res.ok = true;
    res.args.count = 3;
    setIntArg(res.args.args[0], l);
    setIntArg(res.args.args[1], r);
    setIntArg(res.args.args[2], mm);
    // Pack optional sensor= into args[3].
    res.args.count = packSensorArg(res.args, 3, kvs, nkv);
    return res;
}

// handleD — VW converter with dist=<mm> stop param.
//
// Computes (v, ω) from (l, r), builds VW args with "dist=<mm>" stop param,
// and pushes to the queue.  Falls back to direct distanceDrive() when queue is null.
static void handleD(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int l  = args.args[0].ival;
    int r  = args.args[1].ival;
    int mm = args.args[2].ival;

    if (ctx->queue != nullptr) {
        // N16 fix (030-009): validate sensor= BEFORE replying OK on the queue path.
        if (args.count >= 4) {
            uint8_t ch; float thr; StopCondition::Cmp cmp;
            if (!mc_parseSensorToken(args.args[3].sval, ch, thr, cmp)) {
                char rbuf[80];
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                           corrId, replyFn, replyCtx);
                return;
            }
        }

        float v_mms, omega_rads;
        BodyKinematics::forward((float)l, (float)r, ctx->robot->config.trackwidthMm,
                                v_mms, omega_rads);
        int v_int     = (int)v_mms;
        int omega_int = (int)(omega_rads * 1000.0f);

        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], v_int);
        setIntArg(vwArgs.args[1], omega_int);
        vwArgs.count = packKVArg(vwArgs, 2, "dist", mm);

        // sensor= forwarding: pack into the VW args if present.
        if (args.count >= 4) {
            vwArgs.args[vwArgs.count] = args.args[3];  // copy STR sensor arg
            ++vwArgs.count;
        }

        char body[48];
        snprintf(body, sizeof(body), "l=%d r=%d mm=%d", l, r, mm);
        char rbuf[80];
        if (!pushVW(ctx, vwArgs, corrId, replyFn, replyCtx)) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full", nullptr, corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct distanceDrive() (resets enc baseline).
        ctx->robot->distanceDrive((int32_t)l, (int32_t)r, (int32_t)mm,
                                   replyFn, replyCtx, corrId);
        // Optional sensor= stop condition (packed into args[3] by parseD).
        if (args.count >= 4) {
            uint8_t ch; float thr; StopCondition::Cmp cmp;
            if (!mc_parseSensorToken(args.args[3].sval, ch, thr, cmp)) {
                char rbuf[64];
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                           corrId, replyFn, replyCtx);
                ctx->mc->cancel(ctx->robot->systemTime(), replyFn, replyCtx);
                return;
            }
            ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
        }
        char body[48];
        snprintf(body, sizeof(body), "l=%d r=%d mm=%d", l, r, mm);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
    }
}

// ── G ────────────────────────────────────────────────────────────────────────

static ParseResult parseG(const char* const* tokens, int ntokens,
                           const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 3) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int x     = atoi(tokens[0]);
    int y     = atoi(tokens[1]);
    int speed = atoi(tokens[2]);
    if (x < -10000 || x > 10000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "x"; return res;
    }
    if (y < -10000 || y > 10000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "y"; return res;
    }
    if (speed < 1 || speed > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "speed"; return res;
    }
    res.ok = true;
    res.args.count = 3;
    setIntArg(res.args.args[0], x);
    setIntArg(res.args.args[1], y);
    setIntArg(res.args.args[2], speed);
    return res;
}

// handleG — VW converter with x=<mm>, y=<mm>, speed=<mm/s> stop params.
//
// G is a go-to command: VW args use speed as v, 0 as omega (G's own logic
// computes steering), with "x=<mm>", "y=<mm>", "speed=<mm/s>" stop params.
// Falls back to direct beginGoTo() when queue is null.
static void handleG(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int x     = args.args[0].ival;
    int y     = args.args[1].ival;
    int speed = args.args[2].ival;

    if (ctx->queue != nullptr) {
        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], speed);   // v = speed
        setIntArg(vwArgs.args[1], 0);       // omega = 0 (G's steering computed by VW handler)
        vwArgs.count = packKVArg(vwArgs, 2, "x", x);
        vwArgs.count = packKVArg(vwArgs, vwArgs.count, "y", y);
        vwArgs.count = packKVArg(vwArgs, vwArgs.count, "speed", speed);

        char body[64];
        snprintf(body, sizeof(body), "x=%d y=%d speed=%d", x, y, speed);
        char rbuf[96];
        if (!pushVW(ctx, vwArgs, corrId, replyFn, replyCtx)) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full", nullptr, corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "goto", body, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct beginGoTo().
        ctx->mc->beginGoTo((float)x, (float)y, (float)speed,
                           ctx->robot->systemTime(),
                           ctx->robot->state.target,
                           replyFn, replyCtx, corrId);
        char body[64];
        snprintf(body, sizeof(body), "x=%d y=%d speed=%d", x, y, speed);
        char rbuf[96];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "goto", body, corrId, replyFn, replyCtx);
    }
}

// ── R ────────────────────────────────────────────────────────────────────────

static ParseResult parseR(const char* const* tokens, int ntokens,
                           const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int speed  = atoi(tokens[0]);
    int radius = atoi(tokens[1]);
    if (speed < -1000 || speed > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "speed"; return res;
    }
    if (radius < -10000 || radius > 10000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "radius"; return res;
    }
    res.ok = true;
    res.args.count = 2;
    setIntArg(res.args.args[0], speed);
    setIntArg(res.args.args[1], radius);
    return res;
}

// handleR — VW converter with "speed=<mm/s>", "radius=<mm>" stop params.
//
// R (arc) is open-ended: v = speed, omega = speed/radius (κ = 1/radius).
// No stop condition; stop params encode raw speed + radius for VW handler.
// Falls back to direct beginArc() when queue is null.
static void handleR(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int speed  = args.args[0].ival;
    int radius = args.args[1].ival;

    if (ctx->queue != nullptr) {
        // Compute omega = speed * kappa = speed / radius (kappa = 1/radius).
        float omega_rads = (radius != 0) ? ((float)speed / (float)radius) : 0.0f;
        int v_int     = speed;
        int omega_int = (int)(omega_rads * 1000.0f);  // rad/s → mrad/s

        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], v_int);
        setIntArg(vwArgs.args[1], omega_int);
        vwArgs.count = packKVArg(vwArgs, 2, "speed", speed);
        vwArgs.count = packKVArg(vwArgs, vwArgs.count, "radius", radius);

        char body[48];
        snprintf(body, sizeof(body), "speed=%d radius=%d", speed, radius);
        char rbuf[80];
        if (!pushVW(ctx, vwArgs, corrId, replyFn, replyCtx)) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full", nullptr, corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "arc", body, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct beginArc().
        uint32_t now = ctx->robot->systemTime();
        ctx->mc->beginArc((float)speed, (float)radius, now,
                          ctx->robot->state.target,
                          replyFn, replyCtx, corrId);
        char body[48];
        snprintf(body, sizeof(body), "speed=%d radius=%d", speed, radius);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "arc", body, corrId, replyFn, replyCtx);
    }
}

// ── TURN ─────────────────────────────────────────────────────────────────────

static ParseResult parseTURN(const char* const* tokens, int ntokens,
                              const KVPair* kvs, int nkv)
{
    ParseResult res;
    if (ntokens < 1) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int heading_cdeg = atoi(tokens[0]);
    if (heading_cdeg < -18000 || heading_cdeg > 18000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "heading"; return res;
    }
    // Parse optional eps=<cdeg> kv; default 300.
    int eps_cdeg = 300;
    for (int i = 0; i < nkv; ++i) {
        if (kvs[i].key && strcmp(kvs[i].key, "eps") == 0) {
            eps_cdeg = atoi(kvs[i].value);
            if (eps_cdeg < 10 || eps_cdeg > 1800) {
                res.ok = false; res.err.code = "range"; res.err.detail = "eps"; return res;
            }
            break;
        }
    }
    res.ok = true;
    res.args.count = 2;
    setIntArg(res.args.args[0], heading_cdeg);
    setIntArg(res.args.args[1], eps_cdeg);
    // Pack optional sensor= into args[2].
    res.args.count = packSensorArg(res.args, 2, kvs, nkv);
    return res;
}

// handleTURN — VW converter with "h=<cdeg>", "eps=<cdeg>" stop params.
//
// TURN is an absolute-heading rotation: v = 0 (spin-in-place), omega is
// computed by VW handler from heading_cdeg. Stop params "h=<cdeg>" and
// "eps=<cdeg>" tell VW handler to call beginTurn().
// Falls back to direct beginTurn() when queue is null.
static void handleTURN(const ArgList& args, const char* corrId,
                       ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int heading_cdeg = args.args[0].ival;
    int eps_cdeg     = args.args[1].ival;

    if (ctx->queue != nullptr) {
        // N16 fix (030-009): validate sensor= BEFORE replying OK on the queue path.
        if (args.count >= 3) {
            uint8_t ch; float thr; StopCondition::Cmp cmp;
            if (!mc_parseSensorToken(args.args[2].sval, ch, thr, cmp)) {
                char rbuf[80];
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                           corrId, replyFn, replyCtx);
                return;
            }
        }

        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], 0);   // v = 0 (spin-in-place; omega computed by VW handler)
        setIntArg(vwArgs.args[1], 0);   // omega placeholder; VW handler uses "h" param
        vwArgs.count = packKVArg(vwArgs, 2, "h", heading_cdeg);
        vwArgs.count = packKVArg(vwArgs, vwArgs.count, "eps", eps_cdeg);

        // sensor= forwarding: pack into the VW args if present.
        if (args.count >= 3) {
            vwArgs.args[vwArgs.count] = args.args[2];  // copy STR sensor arg
            ++vwArgs.count;
        }

        char body[48];
        snprintf(body, sizeof(body), "heading=%d eps=%d", heading_cdeg, eps_cdeg);
        char rbuf[80];
        if (!pushVW(ctx, vwArgs, corrId, replyFn, replyCtx)) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full", nullptr, corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "turn", body, corrId, replyFn, replyCtx);
    } else {
        // Queue not available: fall back to direct beginTurn().
        uint32_t now = ctx->robot->systemTime();
        ctx->mc->beginTurn((float)heading_cdeg, (float)eps_cdeg, now,
                           ctx->robot->state.target,
                           replyFn, replyCtx, corrId);
        // Optional sensor= stop condition (packed into args[2] by parseTURN).
        if (args.count >= 3) {
            uint8_t ch; float thr; StopCondition::Cmp cmp;
            if (!mc_parseSensorToken(args.args[2].sval, ch, thr, cmp)) {
                char rbuf[64];
                CommandProcessor::replyErr(rbuf, sizeof(rbuf), "badarg", "sensor",
                                           corrId, replyFn, replyCtx);
                ctx->mc->cancel(ctx->robot->systemTime(), replyFn, replyCtx);
                return;
            }
            ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
        }
        char body[48];
        snprintf(body, sizeof(body), "heading=%d eps=%d", heading_cdeg, eps_cdeg);
        char rbuf[80];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "turn", body, corrId, replyFn, replyCtx);
    }
}

// ── RT (relative turn, encoder-arc stop) ───────────────────────────────────────

static ParseResult parseRT(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 1) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int rel_cdeg = atoi(tokens[0]);
    if (rel_cdeg < -180000 || rel_cdeg > 180000) {   // ±1800° relative
        res.ok = false; res.err.code = "range"; res.err.detail = "deg"; return res;
    }
    res.ok = true;
    res.args.count = 1;
    setIntArg(res.args.args[0], rel_cdeg);
    return res;
}

// handleRT — RELATIVE spin-in-place by rel_cdeg, stopped on encoder arc.
// Enqueues a VW with "rot=<cdeg>" so the VW handler (loop context) calls
// beginRotation(). Falls back to a direct call when the queue is null.
static void handleRT(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int rel_cdeg = args.args[0].ival;

    if (ctx->queue != nullptr) {
        ArgList vwArgs;
        vwArgs.count = 2;
        setIntArg(vwArgs.args[0], 0);   // v = 0 (spin in place)
        setIntArg(vwArgs.args[1], 0);   // omega placeholder (computed by beginRotation)
        vwArgs.count = packKVArg(vwArgs, 2, "rot", rel_cdeg);
        char body[32];
        snprintf(body, sizeof(body), "rot=%d", rel_cdeg);
        char rbuf[64];
        if (!pushVW(ctx, vwArgs, corrId, replyFn, replyCtx)) {
            CommandProcessor::replyErr(rbuf, sizeof(rbuf), "full", nullptr, corrId, replyFn, replyCtx);
            return;
        }
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rt", body, corrId, replyFn, replyCtx);
    } else {
        uint32_t now = ctx->robot->systemTime();
        ctx->mc->beginRotation((float)rel_cdeg, now, ctx->robot->state.target,
                               replyFn, replyCtx, corrId);
        char body[32];
        snprintf(body, sizeof(body), "rot=%d", rel_cdeg);
        char rbuf[64];
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "rt", body, corrId, replyFn, replyCtx);
    }
}

// ── VW ───────────────────────────────────────────────────────────────────────
//
// VW — body-twist velocity command (open-ended, keepalive watchdog).
//
// Wire format: VW <v_mms> <omega_mrads> [key=value ...]
//   args[0].ival = v in mm/s
//   args[1].ival = omega in mrad/s (wire units; converted to rad/s here)
//   args[2..] = optional stop params (ArgType::STR "key=value"):
//     "t=<ms>"        → call beginTimed(v, omega, ms, ...)
//     "dist=<mm>"     → call beginDistance(vL, vR equivalent, mm, ...)
//     "x=<mm>"+"y=<mm>"+"speed=<mm/s>" → call beginGoTo(x, y, speed, ...)
//     "h=<cdeg>"+"eps=<cdeg>"           → call beginTurn(h_cdeg, eps_cdeg, ...)
//     "speed=<mm/s>"+"radius=<mm>"      → call beginArc(speed, radius, ...)
//     "stream=1"      → call beginStream (S-command semantics, seeded BVC)
//     (no stop params) → open-ended velocity (beginVelocity or keepalive re-arm)
//
// D11 suppression: when dispatched from a converter push (stop-param branches),
// handleVW does NOT call replyOK — the converter handler already replied.
// Only the open-ended branch (no stop params) emits OK vw.

static ParseResult parseVW(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int v     = atoi(tokens[0]);
    int omega = atoi(tokens[1]);
    if (v < -1000 || v > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "v"; return res;
    }
    if (omega < -3142 || omega > 3142) {
        res.ok = false; res.err.code = "range"; res.err.detail = "omega"; return res;
    }
    res.ok = true;
    res.args.count = 2;
    setIntArg(res.args.args[0], v);
    setIntArg(res.args.args[1], omega);
    return res;
}

static void handleVW(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int v     = args.args[0].ival;
    int omega = args.args[1].ival;
    float omega_rads = (float)omega / 1000.0f;  // mrad/s → rad/s
    uint32_t now = ctx->robot->systemTime();

    // ── Stop-param dispatch ─────────────────────────────────────────────────
    // Converter handlers (S, T, D, G, R, TURN, RT) pack stop params as STR
    // args at args[2..].  Scan them here to select the appropriate begin*() call.
    //
    // D11 suppression: all stop-param branches call begin*() and return WITHOUT
    // calling replyOK.  The converter handler already emitted the reply before
    // pushing this VW ParsedCommand onto the queue.
    // Only the open-ended path (no stop params) emits "OK vw ...".

    // Check for RT (relative rotation): "rot=<cdeg>" present.
    if (vwHasKey(args, "rot")) {
        int rot_cdeg = vwScanKV(args, "rot", 0);
        ctx->mc->beginRotation((float)rot_cdeg, now,
                               ctx->robot->state.target,
                               replyFn, replyCtx, corrId);
        // D11: no replyOK here — handleRT already replied.
        return;
    }

    // Check for TURN: "h=<cdeg>" present (and no "x" key).
    if (vwHasKey(args, "h") && !vwHasKey(args, "x")) {
        int h_cdeg  = vwScanKV(args, "h",   0);
        int eps     = vwScanKV(args, "eps", 300);

        ctx->mc->beginTurn((float)h_cdeg, (float)eps, now,
                           ctx->robot->state.target,
                           replyFn, replyCtx, corrId);

        // Optional sensor= forwarding.
        for (int i = 2; i < args.count; ++i) {
            if (args.args[i].type == ArgType::STR &&
                strncmp(args.args[i].sval, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (mc_parseSensorToken(args.args[i].sval + 7, ch, thr, cmp)) {
                    ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
                }
                break;
            }
        }

        // D11: no replyOK here — handleTURN already replied.
        return;
    }

    // Check for G (go-to): "x=<mm>" and "y=<mm>" present.
    if (vwHasKey(args, "x") && vwHasKey(args, "y")) {
        int x_mm    = vwScanKV(args, "x",     0);
        int y_mm    = vwScanKV(args, "y",     0);
        int speed   = vwScanKV(args, "speed", v);  // fallback to v

        ctx->mc->beginGoTo((float)x_mm, (float)y_mm, (float)speed, now,
                           ctx->robot->state.target,
                           replyFn, replyCtx, corrId);

        // D11: no replyOK here — handleG already replied.
        return;
    }

    // Check for R (arc): "radius=<mm>" present (speed=<mm/s> also present).
    if (vwHasKey(args, "radius")) {
        int speed   = vwScanKV(args, "speed",  v);
        int radius  = vwScanKV(args, "radius", 0);

        ctx->mc->beginArc((float)speed, (float)radius, now,
                          ctx->robot->state.target,
                          replyFn, replyCtx, corrId);

        // D11: no replyOK here — handleR already replied.
        return;
    }

    // Check for T (timed): "t=<ms>" present.
    if (vwHasKey(args, "t")) {
        int ms = vwScanKV(args, "t", 0);

        // Convert (v, omega) back to (vL, vR) for beginTimed (which takes wheel speeds).
        float b = ctx->robot->config.trackwidthMm;
        float vL = (float)v - omega_rads * (b * 0.5f);
        float vR = (float)v + omega_rads * (b * 0.5f);

        ctx->mc->beginTimed(vL, vR, (uint32_t)ms, now,
                            ctx->robot->state.target,
                            replyFn, replyCtx, corrId);

        // Optional sensor= forwarding.
        for (int i = 2; i < args.count; ++i) {
            if (args.args[i].type == ArgType::STR &&
                strncmp(args.args[i].sval, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (mc_parseSensorToken(args.args[i].sval + 7, ch, thr, cmp)) {
                    ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
                }
                break;
            }
        }

        // D11: no replyOK here — handleT already replied.
        return;
    }

    // Check for D (distance): "dist=<mm>" present.
    if (vwHasKey(args, "dist")) {
        int mm = vwScanKV(args, "dist", 0);

        // Convert (v, omega) back to (vL, vR) for distanceDrive (which takes wheel speeds).
        float b = ctx->robot->config.trackwidthMm;
        float vL = (float)v - omega_rads * (b * 0.5f);
        float vR = (float)v + omega_rads * (b * 0.5f);

        ctx->robot->distanceDrive((int32_t)vL, (int32_t)vR, (int32_t)mm,
                                   replyFn, replyCtx, corrId);

        // Optional sensor= forwarding.
        for (int i = 2; i < args.count; ++i) {
            if (args.args[i].type == ArgType::STR &&
                strncmp(args.args[i].sval, "sensor=", 7) == 0) {
                uint8_t ch; float thr; StopCondition::Cmp cmp;
                if (mc_parseSensorToken(args.args[i].sval + 7, ch, thr, cmp)) {
                    ctx->mc->activeCmd().addStop(makeSensorStop(ch, thr, cmp));
                }
                break;
            }
        }

        // D11: no replyOK here — handleD already replied.
        return;
    }

    // ── No stop params: open-ended velocity (VW / S mode) ──────────────────
    //
    // If a "stream=1" marker is present (packed by handleS on the queue path),
    // route via beginStream so the BVC is seeded immediately at the target speed
    // (no trapezoid ramp-up).  This preserves the original S-command semantics.
    // Direct VW commands with no stop params use beginVelocity (MotionCommand
    // with ramp), which supports X soft + EVT done on completion.
    //
    // D11 note: this is the ONLY branch that emits replyOK.  The "stream=1"
    // path is dispatched from handleS (which already called replyOK), but
    // beginStream doesn't reply — the S converter's replyOK covers it.
    // The open-ended VW path (no stream, no stop params) is the direct VW
    // command; it emits exactly one replyOK here.
    if (vwHasKey(args, "stream")) {
        // S streaming path — seed BVC at target and go to STREAMING mode.
        // Convert (v, omega) back to (vL, vR) for beginStream.
        float b  = ctx->robot->config.trackwidthMm;
        float vL = (float)v - omega_rads * (b * 0.5f);
        float vR = (float)v + omega_rads * (b * 0.5f);
        ctx->mc->beginStream(vL, vR, now,
                             ctx->robot->state.target,
                             replyFn, replyCtx);
        // D11: no replyOK here — handleS already replied before pushing this VW.
        return;
    } else if (ctx->mc->hasActiveCommand()) {
        // D6 origin guard: only update the target when the active command is a
        // VW-origin command.  Any other origin (TURN, G, T, D, R, RT) means a
        // non-VW command is running; calling setTarget(0,0) here would corrupt
        // its target (e.g. zero omega on an active TURN stops the rotation
        // prematurely and silently corrupts navigation).
        //
        // For non-VW origins: reset the system watchdog by returning a busy
        // reply and do NOT call setTarget.
        if (ctx->mc->activeCmd().origin() == MotionCommand::Origin::VW) {
            // VW keepalive: update target and re-arm.
            ctx->mc->activeCmd().setTarget((float)v, omega_rads);
        } else {
            // Non-VW command active: reply busy, do not stomp target.
            static const char* kOriginNames[] = {
                "VW", "TURN", "G", "T", "D", "R", "RT"
            };
            int originIdx = static_cast<int>(ctx->mc->activeCmd().origin());
            const char* originName = (originIdx >= 0 && originIdx < 7)
                                     ? kOriginNames[originIdx] : "?";
            char rbuf[64];
            char body[32];
            snprintf(body, sizeof(body), "busy=%s", originName);
            CommandProcessor::replyOK(rbuf, sizeof(rbuf), "vw", body, corrId, replyFn, replyCtx);
            return;
        }
    } else {
        // New VW command: configure MotionCommand from scratch.
        ctx->mc->beginVelocity((float)v, omega_rads, now,
                               ctx->robot->state.target,
                               replyFn, replyCtx, corrId);
    }

    // Open-ended direct VW: emit exactly one OK reply.
    char body[32];
    snprintf(body, sizeof(body), "v=%d omega=%d", v, omega);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "vw", body, corrId, replyFn, replyCtx);
}

// ── _VW ──────────────────────────────────────────────────────────────────────
// Raw velocity command: seeds BVC current state immediately (no ramp).
// Fire-and-forget — no MotionCommand, system watchdog handles keepalive.

static ParseResult parse_VW(const char* const* tokens, int ntokens,
                             const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false; res.err.code = "badarg"; res.err.detail = nullptr; return res;
    }
    int v     = atoi(tokens[0]);
    int omega = atoi(tokens[1]);
    if (v < -1000 || v > 1000) {
        res.ok = false; res.err.code = "range"; res.err.detail = "v"; return res;
    }
    if (omega < -3142 || omega > 3142) {
        res.ok = false; res.err.code = "range"; res.err.detail = "omega"; return res;
    }
    res.ok = true;
    res.args.count = 2;
    setIntArg(res.args.args[0], v);
    setIntArg(res.args.args[1], omega);
    return res;
}

static void handle_VW(const ArgList& args, const char* corrId,
                      ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    int v     = args.args[0].ival;
    int omega = args.args[1].ival;
    float omega_rads = (float)omega / 1000.0f;  // mrad/s → rad/s
    ctx->mc->beginRawVelocity((float)v, omega_rads);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "_VW", nullptr, corrId, replyFn, replyCtx);
}

// ── X and STOP ───────────────────────────────────────────────────────────────

static ParseResult parseNoArgs(const char* const* /*tokens*/, int /*ntokens*/,
                               const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    res.ok = true;
    res.args.count = 0;
    return res;
}

// parseX — optional "soft" positional token; stored as STR arg if present.
static ParseResult parseX(const char* const* tokens, int ntokens,
                           const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    res.ok = true;
    if (ntokens >= 1 && strcmp(tokens[0], "soft") == 0) {
        // Pack "soft" as STR arg[0].
        res.args.count = 1;
        res.args.args[0].type = ArgType::STR;
        res.args.args[0].ival = 0;
        res.args.args[0].fval = 0.0f;
        res.args.args[0].sval[0] = 's';
        res.args.args[0].sval[1] = 'o';
        res.args.args[0].sval[2] = 'f';
        res.args.args[0].sval[3] = 't';
        res.args.args[0].sval[4] = '\0';
    } else {
        res.args.count = 0;
    }
    return res;
}

static void handleX(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    uint32_t now = ctx->robot->systemTime();

    // Check for "soft" positional arg — soft stop ramps BVC to zero.
    bool isSoft = (args.count >= 1 &&
                   args.args[0].type == ArgType::STR &&
                   strcmp(args.args[0].sval, "soft") == 0);

    if (isSoft) {
        ctx->mc->softStop(now);
    } else {
        ctx->mc->cancel(now, replyFn, replyCtx);
    }
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "x", nullptr, corrId, replyFn, replyCtx);
}

static void handleSTOP(const ArgList& /*args*/, const char* corrId,
                       ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    MotionCtx* ctx = static_cast<MotionCtx*>(handlerCtx);
    uint32_t now = ctx->robot->systemTime();
    ctx->mc->cancel(now, replyFn, replyCtx);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stop", nullptr, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// getMotionCommands — returns the full set of motion CommandDescriptors.
//
// Replaces MotionController::getCommands().  ctx->vwDesc is initialised here
// so that converter handlers can build ParsedCommands targeting handleVW.
// ctx must remain live for the lifetime of the returned descriptors.
// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> getMotionCommands(MotionCtx* ctx)
{
    // Initialise the stable VW descriptor that converter handlers reference
    // when building ParsedCommands to push_front onto the queue.
    ctx->vwDesc.prefix     = "VW";
    ctx->vwDesc.parseFn    = parseVW;
    ctx->vwDesc.handlerFn  = handleVW;
    ctx->vwDesc.handlerCtx = ctx;
    ctx->vwDesc.errFmt     = "badarg";
    ctx->vwDesc.forceReply = ForceReply::NONE;
    ctx->vwDesc.flags      = CMD_ACCESS_HARDWARE;

    return {
        makeCmd("S",    parseS,      handleS,    ctx, "badarg"), // set wheel speeds (mm/s)
        makeCmd("T",    parseT,      handleT,    ctx, "badarg"), // timed drive (ms)
        makeCmd("D",    parseD,      handleD,    ctx, "badarg"), // distance drive (mm)
        makeCmd("G",    parseG,      handleG,    ctx, "badarg"), // goto encoder position
        makeCmd("R",    parseR,      handleR,    ctx, "badarg"), // arc drive: R <speed> <radius_mm>
        makeCmd("TURN", parseTURN,   handleTURN, ctx, "badarg"), // spin in place to absolute heading
        makeCmd("RT",   parseRT,     handleRT,   ctx, "badarg"), // relative spin by <cdeg>
        makeCmd("VW",   parseVW,     handleVW,   ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE),
        makeCmd("_VW",  parse_VW,    handle_VW,  ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE),
        makeCmd("X",    parseX,      handleX,    ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE),
        makeCmd("STOP", parseNoArgs, handleSTOP, ctx, "badarg", ForceReply::NONE, CMD_ACCESS_HARDWARE),
    };
}
