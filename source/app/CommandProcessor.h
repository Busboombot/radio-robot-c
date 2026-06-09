#pragma once
#include <stdint.h>
#include "Protocol.h"
#include "CommandTypes.h"

// Forward declarations — CommandProcessor.cpp includes Robot.h directly.
// Keeping only forward decls here avoids including Robot.h's transitive
// header graph (MicroBit, CODAL, all subsystems) in every file that
// includes CommandProcessor.h.
struct Robot;
class LoopScheduler;
class I2CBus;

/**
 * CommandProcessor — protocol v2 wire-protocol parser and dispatcher.
 *
 * Token grammar:
 *   <verb> [<arg>…] [key=value…] [#<id>]
 *
 * - Only the verb token is upper-cased; all other tokens preserve case.
 * - Optional trailing '#<digits>' token is extracted as a correlation id
 *   and echoed in every response for that command.
 * - 'key=value' tokens are parsed by parseKV().
 *
 * Response taxonomy (see replyOK / replyErr / replyEvt):
 *   OK  <verb> <body> [#id]
 *   ERR <code> <detail> [#id]
 *   EVT <name> <body>
 *   TLM …
 *   CFG …
 *   ID  …
 *
 * Usage (main.cpp):
 *   CommandProcessor cmd(robot);
 *   // in loop:
 *   cmd.process(lineBuf, replyFn, ctx);
 */
class CommandProcessor {
public:
    // Legacy constructor — uses the old switch-based dispatch in process().
    explicit CommandProcessor(Robot& robot);

    // Table-dispatch constructor — when _cmds != nullptr, process() routes
    // through dispatchTable() instead of the legacy switch.
    // cmds must point to a static array of count CommandDescriptors that
    // outlives this object. No heap allocation.
    CommandProcessor(const CommandDescriptor* cmds, int count);

    // Parse and dispatch one command line. line must be NUL-terminated.
    // Calls replyFn(msg, ctx) for each response line.
    void process(const char* line, ReplyFn replyFn, void* ctx);

    // Wire the scheduler so the DBG LOOP command can toggle/inspect tasks.
    // Optional — if unset, DBG LOOP replies with an error.
    void setScheduler(LoopScheduler* sched) { _sched = sched; }

    // Wire the I2CBus instance so DBG I2C can read per-device stats (015-003).
    // Optional — if unset, DBG I2C replies with an error.
    void setI2CBus(I2CBus* bus) { _i2cBus = bus; }

    // Override the serial reply channel for ForceReply::SERIAL descriptors.
    // Optional — if unset, ForceReply::SERIAL uses the incoming replyFn/ctx.
    void setSerialReply(ReplyFn fn, void* ctx) { _serialFn = fn; _serialCtx = ctx; }

    // -------------------------------------------------------------------------
    // Static parse helpers — public so dependent tickets can call them
    // from within the dispatch table they add to this translation unit.
    // -------------------------------------------------------------------------

    // Maximum tokens in one command (verb + 23 args/kv + optional #id).
    static constexpr int MAX_TOKENS = 26;
    // Maximum key=value pairs in one command.
    static constexpr int MAX_KV     = 24;

    /**
     * parseTokens — tokenize line into at most maxTokens whitespace-delimited tokens.
     *
     * Copies line into workBuf (which must be at least strlen(line)+1 bytes).
     * Tokens point into workBuf. Upper-cases tokens[0] (the verb) in place.
     * If the last token starts with '#' followed by at least one digit, it is
     * extracted into corr_id (NUL-terminated, max corrIdSize-1 digits) and
     * excluded from the returned token list.
     *
     * Returns the count of tokens (NOT counting the corr_id token).
     */
    static int parseTokens(const char* line, char* workBuf, int workBufSize,
                           char** tokens, int maxTokens,
                           char* corr_id, int corrIdSize);

    /**
     * parseKV — scan tokens[1..ntokens-1] for key=value pairs.
     *
     * Tokens that contain '=' are split and appended to kvs[].
     * '=' without a key yields a "badarg" indicator: kv.key == nullptr.
     * '=' without a value yields: kv.value == "" (empty string).
     * Tokens without '=' are left as positional args (caller handles separately).
     *
     * Returns the count of kv pairs found.
     */
    static int parseKV(char** tokens, int ntokens, KVPair* kvs, int maxKV);

    /**
     * replyOK — write "OK <verb> <body> [#id]\n" into buf.
     *
     * body may be nullptr or "" (no body appended beyond verb).
     * id may be nullptr or "" (no #id suffix).
     */
    static void replyOK(char* buf, int size,
                        const char* verb, const char* body, const char* id,
                        ReplyFn fn, void* ctx);

    /**
     * replyErr — write "ERR <code> <detail> [#id]\n" into buf.
     *
     * detail may be nullptr or "".
     */
    static void replyErr(char* buf, int size,
                         const char* code, const char* detail, const char* id,
                         ReplyFn fn, void* ctx);

    /**
     * replyEvt — write "EVT <name> <body>\n" into buf.
     *
     * Async events carry no correlation id.
     * body may be nullptr or "".
     */
    static void replyEvt(char* buf, int size,
                         const char* name, const char* body,
                         ReplyFn fn, void* ctx);

private:
    Robot* _robot   = nullptr;
    LoopScheduler* _sched   = nullptr;
    I2CBus*        _i2cBus  = nullptr;

    // Table-dispatch state (set by the new constructor; nullptr in legacy mode).
    const CommandDescriptor* _cmds     = nullptr;
    int                      _cmdCount = 0;
    ReplyFn                  _serialFn  = nullptr;
    void*                    _serialCtx = nullptr;

    // Table-dispatch implementation — called by process() when _cmds != nullptr.
    void dispatchTable(char** tokens, int ntok, KVPair* kvs, int nkv,
                       const char* corrId, ReplyFn replyFn, void* ctx);

    static int clampInt(int v, int lo, int hi);
};
