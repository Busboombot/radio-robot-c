// ---------------------------------------------------------------------------
// ConfigCommands.cpp — config-registry command handlers (GET, GET VEL, SET).
//
// Split from source/app/SystemCommands.cpp (finding A3). Contains:
//   - file-local static parse*/handle* functions for GET VEL, GET, and SET
//   - appendConfigCommands() seam that registers these descriptors onto the
//     command table built by Robot::buildCommandTable (in SystemCommands.cpp)
//
// handleGet/handleSet are free functions declared in ConfigRegistry.h and
// defined in source/robot/ConfigRegistry.cpp — only the parse* halves and the
// GET VEL parse/handle pair live here. Behaviour is unchanged: descriptors are
// wired exactly as buildCommandTable previously wired them.
// ---------------------------------------------------------------------------

#include "ConfigCommands.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "ConfigRegistry.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace {

// ---------------------------------------------------------------------------
// Internal accessor -- cast handlerCtx to RobotSysCtx* (GET VEL only).
// ---------------------------------------------------------------------------
static RobotSysCtx& sysCtxFrom(void* p)
{
    return *reinterpret_cast<RobotSysCtx*>(p);
}

// ---------------------------------------------------------------------------
// GET VEL -- per-wheel velocity readout (separate descriptor from GET).
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
    Robot* robot = sysCtxFrom(handlerCtx).robot;
    float vL = robot->state.inputs.velLMms;
    float vR = robot->state.inputs.velRMms;
    char rbuf[64];
    char body[48];
    snprintf(body, sizeof(body), "vel=%d:E,%d:E", (int)vL, (int)vR);
    CommandProcessor::replyOK(rbuf, sizeof(rbuf), "get", body, corrId, replyFn, replyCtx);
}

// ---------------------------------------------------------------------------
// parseGet -- convert positional key-name tokens into STR args for handleGet.
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
// parseSet -- convert kv pairs into "key=value" STR args for handleSet.
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
// appendConfigCommands -- register GET VEL, GET, and SET descriptors.
//
// GET VEL is registered before GET so the longer prefix wins the linear scan
// in buildCommandTable's command table (it is appended to cmds in the same
// relative order it previously held).
// ---------------------------------------------------------------------------
void appendConfigCommands(std::vector<CommandDescriptor>& cmds,
                          CfgCtx* cfgCtx, void* sysCtx)
{
    cmds.push_back(makeCmd("GET VEL",  parseGetVel,    handleGetVel,    sysCtx,  "badarg")); // get velocity PID params
    cmds.push_back(makeCmd("GET",      parseGet,       handleGet,       cfgCtx,  "badkey")); // get config value by key
    cmds.push_back(makeCmd("SET",      parseSet,       handleSet,       cfgCtx,  "badkey")); // set config value by key
}
