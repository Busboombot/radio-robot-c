// PortController.cpp — Commandable wrapper around PortIO.
//
// Owns P (digital port read/write) and PA (analog port read/write) command
// descriptors.  Handler logic mirrors the corresponding switch cases in
// CommandProcessor.cpp (ticket T010 will remove those).

#include "PortController.h"
#include "CommandProcessor.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ---------------------------------------------------------------------------
// Parse functions
// ---------------------------------------------------------------------------

// parseP — parse tokens for the "P" command (digital port read/write).
//   tokens[0] = port (1..4)
//   tokens[1] = value (optional, any int interpreted as bool)
// Packs args[0].ival = port, args[1].ival = val (or -1 for read-only).
static ParseResult parseP(const char* const* tokens, int ntokens,
                           const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 1) {
        res.ok = false;
        res.err = { "badarg", nullptr };
        return res;
    }
    int port = atoi(tokens[0]);
    if (port < 1 || port > 4) {
        res.ok = false;
        res.err = { "range", "port" };
        return res;
    }
    res.ok = true;
    res.args.count = (ntokens >= 2) ? 2 : 1;
    res.args.args[0].type  = ArgType::INT;
    res.args.args[0].ival  = port;
    if (ntokens >= 2) {
        res.args.args[1].type = ArgType::INT;
        res.args.args[1].ival = atoi(tokens[1]);
    }
    return res;
}

// parsePA — parse tokens for the "PA" command (analog port read/write).
//   tokens[0] = port (1..4)
//   tokens[1] = value (optional, 0..1023)
// Packs args[0].ival = port, args[1].ival = val (or absent for read-only).
static ParseResult parsePA(const char* const* tokens, int ntokens,
                            const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens < 1) {
        res.ok = false;
        res.err = { "badarg", nullptr };
        return res;
    }
    int port = atoi(tokens[0]);
    if (port < 1 || port > 4) {
        res.ok = false;
        res.err = { "range", "port" };
        return res;
    }
    if (ntokens >= 2) {
        int val = atoi(tokens[1]);
        if (val < 0 || val > 1023) {
            res.ok = false;
            res.err = { "range", "val" };
            return res;
        }
        res.ok = true;
        res.args.count = 2;
        res.args.args[0].type = ArgType::INT;
        res.args.args[0].ival = port;
        res.args.args[1].type = ArgType::INT;
        res.args.args[1].ival = val;
    } else {
        res.ok = true;
        res.args.count = 1;
        res.args.args[0].type = ArgType::INT;
        res.args.args[0].ival = port;
    }
    return res;
}

// ---------------------------------------------------------------------------
// Handler functions
// ---------------------------------------------------------------------------

// handleP — HandlerFn for the "P" command.
// args[0].ival = port (1..4), args[1].ival = val (present → write, absent → read)
static void handleP(const ArgList& args, const char* corrId,
                    ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    PortController* pc = reinterpret_cast<PortController*>(handlerCtx);
    int port = args.args[0].ival;
    int val;
    if (args.count >= 2) {
        val = args.args[1].ival;
        pc->pio().setDigital((uint8_t)port, val != 0);
    } else {
        val = pc->pio().readDigital((uint8_t)port);
    }
    char body[24];
    snprintf(body, sizeof(body), "p=%d v=%d", port, val);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "port", body, corrId,
                               replyFn, replyCtx);
}

// handlePA — HandlerFn for the "PA" command.
// args[0].ival = port (1..4), args[1].ival = val (present → write, absent → read)
static void handlePA(const ArgList& args, const char* corrId,
                     ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    PortController* pc = reinterpret_cast<PortController*>(handlerCtx);
    int port = args.args[0].ival;
    int val;
    if (args.count >= 2) {
        val = args.args[1].ival;
        pc->pio().setAnalog((uint8_t)port, (uint16_t)val);
    } else {
        val = pc->pio().readAnalog((uint8_t)port);
    }
    char body[24];
    snprintf(body, sizeof(body), "p=%d v=%d", port, val);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "aport", body, corrId,
                               replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// PortController implementation
// ---------------------------------------------------------------------------

PortController::PortController(PortIO& pio)
    : _pio(pio)
{
}

int PortController::getCommands(CommandDescriptor* buf, int max) const
{
    if (max < 2) return 0;
    void* ctx = const_cast<PortController*>(this);
    buf[0] = makeCmd("P",  parseP,  handleP,  ctx, "badarg");
    buf[1] = makeCmd("PA", parsePA, handlePA, ctx, "badarg");
    return 2;
}
