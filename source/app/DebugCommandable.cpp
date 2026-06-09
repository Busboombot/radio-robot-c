// DebugCommandable.cpp — Commandable for all diagnostic commands.
//
// Owns: DBG LOOP RESET, DBG LOOP, DBG I2CLOG, DBG I2C, DBG IRQGUARD,
//       DBG WEDGE, I2CW, I2CR.
//
// All descriptors use ForceReply::SERIAL.
// Handler logic mirrors the existing switch cases in CommandProcessor.cpp
// exactly.  The old switch cases remain live until T011 cutover.

#include "DebugCommandable.h"
#include "CommandProcessor.h"
#include "LoopScheduler.h"
#include "I2CBus.h"
#include "WedgeTest.h"
#include "Robot.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
// Internal helper — cast handlerCtx to DebugCommandable* and get DbgCtx.
// handlerCtx is always const_cast<DebugCommandable*>(this).
// ---------------------------------------------------------------------------

// Forward declaration of accessor used by handlers (defined at bottom of file).
static DbgCtx dbgCtxFrom(void* p);

// ---------------------------------------------------------------------------
// DBG LOOP RESET
//   prefix "DBG LOOP RESET" — argTokens = [] (0 tokens after prefix strip)
//   parseFn: always succeeds, 0 args.
//   handler: no-op acknowledgement (loop timing stats removed with run_tasks)
// ---------------------------------------------------------------------------

static ParseResult parseDbgLoopReset(const char* const* /*tokens*/, int /*ntokens*/,
                                     const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    res.ok = true;
    res.args.count = 0;
    return res;
}

static void handleDbgLoopReset(const ArgList& /*args*/, const char* corrId,
                                ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/)
{
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "loop reset",
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DBG LOOP
//   prefix "DBG LOOP" — argTokens = []
//   parseFn: always succeeds, 0 args.
//   handler: confirm loop is running.
// ---------------------------------------------------------------------------

static ParseResult parseDbgLoop(const char* const* tokens, int ntokens,
                                 const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    res.ok = true;
    // Pass up to 2 tokens as STR args (x and state).
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    res.args.count = n;
    for (int i = 0; i < n; ++i) {
        res.args.args[i].type = ArgType::STR;
        // Copy into sval (bounded).
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(res.args.args[i].sval) - 1; ++j)
            res.args.args[i].sval[j] = tokens[i][j];
        res.args.args[i].sval[j] = '\0';
        res.args.args[i].ival = 0;
        res.args.args[i].fval = 0.0f;
    }
    return res;
}

static void handleDbgLoop(const ArgList& /*args*/, const char* corrId,
                           ReplyFn replyFn, void* replyCtx, void* /*handlerCtx*/)
{
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "loop running",
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DBG I2CLOG
//   prefix "DBG I2CLOG" — argTokens = [] or ["ARM"]
//   parseFn: always succeeds; 0 or 1 STR args.
//   handler: ARM → resetStats + setLogging(true); else → dumpRecent.
// ---------------------------------------------------------------------------

static ParseResult parseDbgI2clog(const char* const* tokens, int ntokens,
                                   const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    res.ok = true;
    if (ntokens >= 1) {
        res.args.count = 1;
        res.args.args[0].type = ArgType::STR;
        int j = 0;
        for (; tokens[0][j] != '\0' && j < (int)sizeof(res.args.args[0].sval) - 1; ++j)
            res.args.args[0].sval[j] = tokens[0][j];
        res.args.args[0].sval[j] = '\0';
        res.args.args[0].ival = 0;
        res.args.args[0].fval = 0.0f;
    } else {
        res.args.count = 0;
    }
    return res;
}

static void handleDbgI2clog(const ArgList& args, const char* corrId,
                              ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.bus == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                                   corrId, replyFn, replyCtx);
        return;
    }
    if (args.count >= 1 && strcmp(args.args[0].sval, "ARM") == 0) {
        ctx.bus->resetStats();
        ctx.bus->setLogging(true);
    } else {
        ctx.bus->dumpRecent(replyFn, replyCtx);
    }
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "i2clog",
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DBG I2C
//   prefix "DBG I2C" — argTokens = [] or ["RESET"]
//   parseFn: always succeeds; 0 or 1 STR args.
//   handler: RESET → resetStats + resetStuckCounters;
//            else → emit compact stats line + OK.
// ---------------------------------------------------------------------------

static ParseResult parseDbgI2c(const char* const* tokens, int ntokens,
                                const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    res.ok = true;
    if (ntokens >= 1) {
        res.args.count = 1;
        res.args.args[0].type = ArgType::STR;
        int j = 0;
        for (; tokens[0][j] != '\0' && j < (int)sizeof(res.args.args[0].sval) - 1; ++j)
            res.args.args[0].sval[j] = tokens[0][j];
        res.args.args[0].sval[j] = '\0';
        res.args.args[0].ival = 0;
        res.args.args[0].fval = 0.0f;
    } else {
        res.args.count = 0;
    }
    return res;
}

static void handleDbgI2c(const ArgList& args, const char* corrId,
                          ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.bus == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                                   corrId, replyFn, replyCtx);
        return;
    }
    if (args.count >= 1 && strcmp(args.args[0].sval, "RESET") == 0) {
        ctx.bus->resetStats();
        ctx.robot->motorController.resetStuckCounters();
        CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "i2c reset",
                                  corrId, replyFn, replyCtx);
        return;
    }
    // Emit compact stats dump.
    uint32_t rV = ctx.bus->reentryViolations();
    uint8_t  sL = ctx.robot->motorController.stuckCountL();
    uint8_t  sR = ctx.robot->motorController.stuckCountR();
    char buf[200];
    int n = snprintf(buf, sizeof(buf),
        "I2C 0x10:txn=%lu err=%lu last=%d "
        "0x17:txn=%lu err=%lu last=%d "
        "0x1A:txn=%lu err=%lu last=%d "
        "0x43:txn=%lu err=%lu last=%d "
        "reentry=%lu stuck=L:%u,R:%u",
        (unsigned long)ctx.bus->txnCount(0x10),
        (unsigned long)ctx.bus->errCount(0x10),
        ctx.bus->lastErr(0x10),
        (unsigned long)ctx.bus->txnCount(0x17),
        (unsigned long)ctx.bus->errCount(0x17),
        ctx.bus->lastErr(0x17),
        (unsigned long)ctx.bus->txnCount(0x1A),
        (unsigned long)ctx.bus->errCount(0x1A),
        ctx.bus->lastErr(0x1A),
        (unsigned long)ctx.bus->txnCount(0x43),
        (unsigned long)ctx.bus->errCount(0x43),
        ctx.bus->lastErr(0x43),
        (unsigned long)rV,
        (unsigned)sL,
        (unsigned)sR);
    (void)n;
    replyFn(buf, replyCtx);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "i2c",
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DBG IRQGUARD
//   prefix "DBG IRQGUARD" — argTokens = [] or ["0"|"1"]
//   parseFn: always succeeds; 0 or 1 INT args.
//   handler: if arg → setIrqGuard; always reply OK with state.
// ---------------------------------------------------------------------------

static ParseResult parseDbgIrqguard(const char* const* tokens, int ntokens,
                                     const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    res.ok = true;
    if (ntokens >= 1) {
        res.args.count = 1;
        res.args.args[0].type = ArgType::INT;
        res.args.args[0].ival = atoi(tokens[0]);
        res.args.args[0].fval = 0.0f;
        res.args.args[0].sval[0] = '\0';
    } else {
        res.args.count = 0;
    }
    return res;
}

static void handleDbgIrqguard(const ArgList& args, const char* corrId,
                               ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.bus == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                                   corrId, replyFn, replyCtx);
        return;
    }
    if (args.count >= 1) ctx.bus->setIrqGuard(args.args[0].ival != 0);
    char msg[24];
    snprintf(msg, sizeof(msg), "irqguard=%d", ctx.bus->irqGuard() ? 1 : 0);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", msg,
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// DBG WEDGE
//   prefix "DBG WEDGE" — argTokens = up to 7 optional ints
//   parseFn: always succeeds; 0..7 INT args.
//   handler: parse optional params with defaults, then runWedgeTest.
// ---------------------------------------------------------------------------

static ParseResult parseDbgWedge(const char* const* tokens, int ntokens,
                                  const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    res.ok = true;
    // Accept up to 7 optional int args.
    int n = (ntokens > 7) ? 7 : ntokens;
    if (n > MAX_ARGS) n = MAX_ARGS;
    res.args.count = n;
    for (int i = 0; i < n; ++i) {
        res.args.args[i].type = ArgType::INT;
        res.args.args[i].ival = atoi(tokens[i]);
        res.args.args[i].fval = 0.0f;
        res.args.args[i].sval[0] = '\0';
    }
    return res;
}

static void handleDbgWedge(const ArgList& args, const char* corrId,
                            ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.sched == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no scheduler",
                                   corrId, replyFn, replyCtx);
        return;
    }
    int wrate  = (args.count >= 1) ? args.args[0].ival : 50;
    int wwrite = (args.count >= 2) ? args.args[1].ival : 40;
    int wbus   = (args.count >= 3) ? args.args[2].ival : 400;
    int wdith  = (args.count >= 4) ? args.args[3].ival : 3;
    int wreg   = (args.count >= 5) ? args.args[4].ival : 0x46;
    int wsens  = (args.count >= 6) ? args.args[5].ival : 0;
    int wreal  = (args.count >= 7) ? args.args[6].ival : 0;
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "wedge start",
                              corrId, replyFn, replyCtx);
    runWedgeTest(ctx.sched->uBit(), wrate, wwrite, wbus, wdith, wreg, wsens,
                 wreal, ctx.robot);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "dbg", "wedge end",
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// I2CW
//   prefix "I2CW" — argTokens = [<addr7-hex>, <byte-hex>, ...]
//   parseFn: validate ≥2 tokens (addr + at least one byte).
//   handler: write bytes to addr via bus.
// ---------------------------------------------------------------------------

static ParseResult parseI2cw(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false;
        res.err = { "badarg", "usage: I2CW <addr> <byte>..." };
        return res;
    }
    res.ok = true;
    // Store all raw tokens as STR args.
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    res.args.count = n;
    for (int i = 0; i < n; ++i) {
        res.args.args[i].type = ArgType::STR;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(res.args.args[i].sval) - 1; ++j)
            res.args.args[i].sval[j] = tokens[i][j];
        res.args.args[i].sval[j] = '\0';
        res.args.args[i].ival = 0;
        res.args.args[i].fval = 0.0f;
    }
    return res;
}

static void handleI2cw(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.bus == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                                   corrId, replyFn, replyCtx);
        return;
    }
    uint8_t addr7 = (uint8_t)strtol(args.args[0].sval, nullptr, 16);
    uint8_t data[24];
    int len = 0;
    for (int i = 1; i < args.count && len < (int)sizeof(data); ++i) {
        data[len++] = (uint8_t)strtol(args.args[i].sval, nullptr, 16);
    }
    int status = ctx.bus->write((uint16_t)(addr7 << 1), data, len);
    char body[48];
    snprintf(body, sizeof(body), "addr=0x%02X n=%d status=%d", addr7, len, status);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "i2cw", body,
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// I2CR
//   prefix "I2CR" — argTokens = [<addr7-hex>, <count>, [<reg-hex>]]
//   parseFn: validate count 1..16, ≥2 tokens.
//   handler: optionally write reg byte with repeated start, then read.
// ---------------------------------------------------------------------------

static ParseResult parseI2cr(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 2) {
        res.ok = false;
        res.err = { "badarg", "usage: I2CR <addr> <count> [reg]" };
        return res;
    }
    int count = atoi(tokens[1]);
    if (count < 1 || count > 16) {
        res.ok = false;
        res.err = { "range", "count" };
        return res;
    }
    res.ok = true;
    int n = (ntokens > MAX_ARGS) ? MAX_ARGS : ntokens;
    res.args.count = n;
    for (int i = 0; i < n; ++i) {
        res.args.args[i].type = ArgType::STR;
        int j = 0;
        for (; tokens[i][j] != '\0' && j < (int)sizeof(res.args.args[i].sval) - 1; ++j)
            res.args.args[i].sval[j] = tokens[i][j];
        res.args.args[i].sval[j] = '\0';
        res.args.args[i].ival = 0;
        res.args.args[i].fval = 0.0f;
    }
    return res;
}

static void handleI2cr(const ArgList& args, const char* corrId,
                        ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    DbgCtx ctx = dbgCtxFrom(handlerCtx);
    char rbuf[64];
    if (ctx.bus == nullptr) {
        CommandProcessor::replyErr(rbuf, sizeof(rbuf), "noimpl", "no i2c bus",
                                   corrId, replyFn, replyCtx);
        return;
    }
    uint8_t addr7 = (uint8_t)strtol(args.args[0].sval, nullptr, 16);
    int count  = atoi(args.args[1].sval);
    int wstatus = 0;
    if (args.count >= 3) {
        uint8_t reg = (uint8_t)strtol(args.args[2].sval, nullptr, 16);
        wstatus = ctx.bus->write((uint16_t)(addr7 << 1), &reg, 1, true);
    }
    uint8_t buf[16];
    int status = ctx.bus->read((uint16_t)(addr7 << 1), buf, count);
    char body[120];
    int pos = snprintf(body, sizeof(body),
                       "addr=0x%02X n=%d wstatus=%d status=%d data=",
                       addr7, count, wstatus, status);
    for (int i = 0; i < count && pos < (int)sizeof(body) - 4; ++i) {
        pos += snprintf(body + pos, (size_t)((int)sizeof(body) - pos),
                        "%s%02X", i ? "," : "", buf[i]);
    }
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "i2cr", body,
                              corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// dbgCtxFrom — extract DbgCtx from handlerCtx.
// handlerCtx is always const_cast<DebugCommandable*>(this).
// ---------------------------------------------------------------------------
static DbgCtx dbgCtxFrom(void* p)
{
    return reinterpret_cast<DebugCommandable*>(p)->ctx();
}

// ---------------------------------------------------------------------------
// DebugCommandable implementation
// ---------------------------------------------------------------------------

DebugCommandable::DebugCommandable(DbgCtx ctx)
    : _ctx(ctx)
{
}

std::vector<CommandDescriptor> DebugCommandable::getCommands() const
{
    void* ctx = const_cast<DebugCommandable*>(this);
    // Longest-prefix entries first within each group so dispatchTable picks
    // the most-specific match (e.g. "DBG LOOP RESET" beats "DBG LOOP").
    return {
        makeCmd("DBG LOOP RESET", parseDbgLoopReset, handleDbgLoopReset, ctx, "badarg", ForceReply::SERIAL), // reset loop stats counters
        makeCmd("DBG LOOP",       parseDbgLoop,      handleDbgLoop,      ctx, "badarg", ForceReply::SERIAL), // report loop timing stats
        makeCmd("DBG I2CLOG",     parseDbgI2clog,    handleDbgI2clog,    ctx, "badarg", ForceReply::SERIAL), // dump I2C transaction log
        makeCmd("DBG I2C",        parseDbgI2c,       handleDbgI2c,       ctx, "badarg", ForceReply::SERIAL), // report I2C bus error counts
        makeCmd("DBG IRQGUARD",   parseDbgIrqguard,  handleDbgIrqguard,  ctx, "badarg", ForceReply::SERIAL), // enable/disable IRQ guard
        makeCmd("DBG WEDGE",      parseDbgWedge,     handleDbgWedge,     ctx, "badarg", ForceReply::SERIAL), // run encoder wedge self-check
        makeCmd("I2CW",           parseI2cw,         handleI2cw,         ctx, "badarg", ForceReply::SERIAL), // raw I2C write (addr reg data…)
        makeCmd("I2CR",           parseI2cr,         handleI2cr,         ctx, "badarg", ForceReply::SERIAL), // raw I2C read (addr reg count)
    };
}
