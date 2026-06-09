// ServoController.cpp — Commandable wrapper around Servo.
//
// Owns the GRIP command descriptor.  Handler logic mirrors the GRIP switch
// case in CommandProcessor.cpp (T010 will remove that case).

#include "ServoController.h"
#include "CommandProcessor.h"
#include <cstdio>
#include <cstdlib>

// ---------------------------------------------------------------------------
// Parse function
// ---------------------------------------------------------------------------

// parseGrip — parse tokens for the "GRIP" command.
//   tokens[0] = angle (0..180), optional.
// With no arg: args.count = 0 (read current angle).
// With arg:    args.count = 1, args[0].ival = clamped angle.
static ParseResult parseGrip(const char* const* tokens, int ntokens,
                              const KVPair* /*kvs*/, int /*nkv*/)
{
    ParseResult res;
    if (ntokens == 0) {
        // No argument — query mode.
        res.ok = true;
        res.args.count = 0;
        return res;
    }
    int deg = atoi(tokens[0]);
    if (deg < 0 || deg > 180) {
        res.ok = false;
        res.err = { "range", "deg" };
        return res;
    }
    res.ok = true;
    res.args.count = 1;
    res.args.args[0].type = ArgType::INT;
    res.args.args[0].ival = deg;
    return res;
}

// ---------------------------------------------------------------------------
// Handler function
// ---------------------------------------------------------------------------

// handleGrip — HandlerFn for the "GRIP" command.
// args.count == 0: read-only; args.count >= 1: args[0].ival = angle.
static void handleGrip(const ArgList& args, const char* corrId,
                       ReplyFn replyFn, void* replyCtx, void* handlerCtx)
{
    ServoController* sc = reinterpret_cast<ServoController*>(handlerCtx);
    int deg;
    if (args.count >= 1) {
        deg = args.args[0].ival;
        uint8_t clamped = (deg < 0) ? 0 : (deg > 180) ? 180 : (uint8_t)deg;
        sc->servo().setAngle(clamped);
    } else {
        deg = (int)sc->servo().currentAngle();
    }
    char body[24];
    snprintf(body, sizeof(body), "deg=%d", deg);
    char rbuf[64];
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "grip", body, corrId,
                               replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// ServoController implementation
// ---------------------------------------------------------------------------

ServoController::ServoController(Servo& srv)
    : _srv(srv)
{
}

int ServoController::getCommands(CommandDescriptor* buf, int max) const
{
    if (max < 1) return 0;
    void* ctx = const_cast<ServoController*>(this);
    buf[0] = makeCmd("GRIP", parseGrip, handleGrip, ctx, "badarg");
    return 1;
}
