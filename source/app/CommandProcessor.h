#pragma once
#include <stdint.h>
#include "Protocol.h"
#include "CommandTypes.h"
#include "CommandQueue.h"

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
 *   CommandProcessor cmd(robot.buildCommandTable(...));
 *   cmd.setSerialReply(serialFn, serialCtx);
 *   // in loop:
 *   cmd.process(lineBuf, replyFn, ctx);
 */
class CommandProcessor {
public:
    CommandProcessor() = default;
    explicit CommandProcessor(std::vector<CommandDescriptor> cmds);

    // Parse and dispatch one command line. line must be NUL-terminated.
    // Calls replyFn(msg, ctx) for each response line.
    void process(const char* line, ReplyFn replyFn, void* ctx);

    // Override the serial reply channel for ForceReply::SERIAL descriptors.
    // Optional — if unset, ForceReply::SERIAL uses the incoming replyFn/ctx.
    void setSerialReply(ReplyFn fn, void* ctx) { _serialFn = fn; _serialCtx = ctx; }

    // Attach a CommandQueue. When non-null, process() enqueues parsed commands
    // into the queue instead of dispatching them immediately.
    void setQueue(CommandQueue* q) { _queue = q; }

    // Dispatch one item from q. Returns false if q is empty.
    // Calls the descriptor's handlerFn directly (not process()) to avoid
    // re-enqueuing when _queue is still set.
    bool dequeueOne(CommandQueue& q);

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
    std::vector<CommandDescriptor> _cmds;
    ReplyFn                        _serialFn  = nullptr;
    void*                          _serialCtx = nullptr;
    CommandQueue*                  _queue     = nullptr;

    void dispatchTable(char** tokens, int ntok, KVPair* kvs, int nkv,
                       const char* corrId, ReplyFn replyFn, void* ctx);

    static int clampInt(int v, int lo, int hi);
};
