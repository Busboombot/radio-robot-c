// CommandProcessor.cpp — protocol v2 wire-protocol parser and dispatcher.
//
// Sprint 009, Ticket 002: v2 tokenizer, verb-only uppercasing, #id
// correlation, OK/ERR/EVT/TLM/CFG/ID response taxonomy.
// Sprint 019, Ticket 011: old Robot& constructor and switch statement removed;
// all commands now go through the table-dispatch path.

#include "CommandProcessor.h"
#include "CommandQueue.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommandProcessor::CommandProcessor(std::vector<CommandDescriptor> cmds)
    : _cmds(std::move(cmds))
{
}

// ---------------------------------------------------------------------------
// setSerialReply — override implemented inline in CommandProcessor.h
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// prefixMatchLen — static helper for dispatchTable()
//
// Tokenizes 'prefix' on spaces (into a local stack buffer), then compares
// token-by-token against tokens[0..ntok-1] using strcmp (verb is already
// uppercased by parseTokens; prefix strings should be written in uppercase).
//
// Returns the number of prefix tokens that matched (0 = no match).
// ---------------------------------------------------------------------------

static int prefixMatchLen(const char* prefix, char** tokens, int ntok)
{
    // Copy prefix into a stack buffer so we can tokenize it in place.
    char buf[64];
    int plen = 0;
    for (const char* p = prefix; *p && plen < (int)(sizeof(buf) - 1); ++p, ++plen) {
        buf[plen] = *p;
    }
    buf[plen] = '\0';

    // Tokenize buf on spaces.
    const char* ptoks[8];
    int nptoks = 0;
    char* cur = buf;
    char* end = buf + plen;
    while (cur < end && nptoks < (int)(sizeof(ptoks) / sizeof(ptoks[0]))) {
        while (cur < end && *cur == ' ') ++cur;
        if (cur >= end) break;
        ptoks[nptoks++] = cur;
        while (cur < end && *cur != ' ') ++cur;
        if (cur < end) { *cur = '\0'; ++cur; }
    }

    if (nptoks == 0 || nptoks > ntok) return 0;

    for (int i = 0; i < nptoks; ++i) {
        if (strcmp(ptoks[i], tokens[i]) != 0) return 0;
    }
    return nptoks;
}

// ---------------------------------------------------------------------------
// dispatchTable — table-driven dispatch.
//
// Scans _cmds for the descriptor whose prefix has the longest token match
// against tokens[0..ntok-1]. If no descriptor matches, replies ERR unknown.
// Otherwise:
//   1. Determines the effective reply channel (ForceReply::SERIAL override).
//   2. Calls parseFn (if non-null); on failure, replies ERR errFmt and returns.
//   3. Calls handlerFn with the parsed ArgList (or an empty ArgList).
// ---------------------------------------------------------------------------

void CommandProcessor::dispatchTable(char** tokens, int ntok, KVPair* kvs, int nkv,
                                     const char* corrId, ReplyFn replyFn, void* ctx)
{
    char rbuf[520];

    // Find the descriptor with the longest matching prefix.
    int bestMatch = 0;
    int bestIdx   = -1;
    for (int i = 0; i < (int)_cmds.size(); ++i) {
        int m = prefixMatchLen(_cmds[i].prefix, tokens, ntok);
        if (m > bestMatch) {
            bestMatch = m;
            bestIdx   = i;
        }
    }

    if (bestIdx < 0) {
        // No descriptor matched.
        replyErr(rbuf, sizeof(rbuf), "unknown", nullptr, corrId, replyFn, ctx);
        return;
    }

    const CommandDescriptor& desc = _cmds[bestIdx];

    // Determine effective reply channel.
    ReplyFn  effectiveFn  = replyFn;
    void*    effectiveCtx = ctx;
    if (desc.forceReply == ForceReply::SERIAL && _serialFn != nullptr) {
        effectiveFn  = _serialFn;
        effectiveCtx = _serialCtx;
    }

    // Strip the matched prefix tokens from the token list passed to parseFn.
    char** argTokens = tokens + bestMatch;
    int    argNtok   = ntok - bestMatch;

    ArgList args;
    args.count = 0;

    if (desc.parseFn != nullptr) {
        ParseResult result = desc.parseFn(
            const_cast<const char* const*>(argTokens), argNtok,
            kvs, nkv);
        if (!result.ok) {
            const char* detail = (result.err.detail != nullptr) ? result.err.detail : nullptr;
            const char* code   = (desc.errFmt != nullptr) ? desc.errFmt : "badarg";
            replyErr(rbuf, sizeof(rbuf), code, detail, corrId, effectiveFn, effectiveCtx);
            return;
        }
        args = result.args;
    }

    if (_queue != nullptr) {
        // Queue mode: build a ParsedCommand and enqueue it instead of
        // dispatching immediately. The caller is responsible for draining
        // the queue via dequeueOne().
        ParsedCommand pc;
        pc.desc    = &desc;
        pc.args    = args;
        pc.replyFn = effectiveFn;
        pc.replyCtx = effectiveCtx;
        // corrId is a local char array; copy into pc.corrId (8-byte field).
        int cidLen = 0;
        while (corrId[cidLen] != '\0' && cidLen < (int)(sizeof(pc.corrId) - 1)) {
            pc.corrId[cidLen] = corrId[cidLen];
            ++cidLen;
        }
        pc.corrId[cidLen] = '\0';
        _queue->push_back(pc);
    } else {
        desc.handlerFn(args, corrId, effectiveFn, effectiveCtx, desc.handlerCtx);
    }
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

// Note: appendKeyValue, handleGet, and handleSet have been moved to
// source/robot/ConfigRegistry.cpp (Sprint 019, Ticket 002).

// ---------------------------------------------------------------------------
// process — v2 command dispatch (table-driven)
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

    // Reply buffer used by error responses before dispatch.
    char rbuf[520];

    // Check for kv tokens with missing key — always badarg.
    KVPair kvs[MAX_KV];
    int    nkv = parseKV(tokens, ntok, kvs, MAX_KV);
    for (int i = 0; i < nkv; ++i) {
        if (kvs[i].key == nullptr) {
            replyErr(rbuf, sizeof(rbuf), "badarg", "missing key", corr_id, replyFn, ctx);
            return;
        }
    }

    dispatchTable(tokens, ntok, kvs, nkv, corr_id, replyFn, ctx);
}

// ---------------------------------------------------------------------------
// dequeueOne — dispatch one item from q directly (bypasses queue branch).
//
// Calls the descriptor's handlerFn directly rather than going through process()
// to avoid re-enqueuing when _queue is still set on this CommandProcessor.
// ---------------------------------------------------------------------------

bool CommandProcessor::dequeueOne(CommandQueue& q)
{
    ParsedCommand pc;
    if (!q.pop_front(pc)) return false;
    if (pc.desc != nullptr) {
        pc.desc->handlerFn(pc.args, pc.corrId, pc.replyFn, pc.replyCtx,
                           pc.desc->handlerCtx);
    }
    return true;
}
