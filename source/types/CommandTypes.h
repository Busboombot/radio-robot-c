#pragma once
#include <stdint.h>
#include "Protocol.h"

// ---------------------------------------------------------------------------
// CommandTypes.h — foundational types for the registration-based command
// dispatch system.  No STL, no exceptions, no heap allocation.
// Constraint: -std=c++11 -fno-exceptions -fno-rtti
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// ArgType — discriminator for a tagged argument
// ---------------------------------------------------------------------------
enum class ArgType : uint8_t { INT, FLOAT, STR };

// ---------------------------------------------------------------------------
// Argument — tagged union covering int, float, and short string values.
// sval[32] covers all practical protocol tokens.
// ---------------------------------------------------------------------------
struct Argument {
    ArgType type;
    union { int32_t ival; float fval; };
    char sval[32];
};

// ---------------------------------------------------------------------------
// ArgList — fixed-capacity argument list (stack-allocated in process()).
// ---------------------------------------------------------------------------
static constexpr int MAX_ARGS = 10;
struct ArgList {
    Argument args[MAX_ARGS];
    int      count;
};

// ---------------------------------------------------------------------------
// ParseError — error payload returned when a ParseFn rejects its input.
// ---------------------------------------------------------------------------
struct ParseError {
    const char* code;
    const char* detail;
};

// ---------------------------------------------------------------------------
// ParseResult — discriminated union returned by every ParseFn.
// Both union members are trivially constructible (no user constructors,
// plain data), which satisfies the C++11 unrestricted-union constraint.
// ---------------------------------------------------------------------------
struct ParseResult {
    bool ok;
    union {
        ArgList   args;
        ParseError err;
    };
};

// ---------------------------------------------------------------------------
// ParseFn — signature for a command's argument parser.
//   tokens  — array of ntokens raw token strings (verb already stripped)
//   kvs     — parsed key=value pairs from the same token stream
// ---------------------------------------------------------------------------
typedef ParseResult (*ParseFn)(const char* const* tokens, int ntokens,
                               const KVPair* kvs, int nkv);

// ---------------------------------------------------------------------------
// HandlerFn — signature for a command's handler.
//   args       — parsed argument list (from ParseFn, or empty if parseFn==nullptr)
//   corrId     — correlation id string (may be "" but never nullptr)
//   replyFn    — reply callback; call once per response line
//   replyCtx   — opaque context forwarded to replyFn
//   handlerCtx — subsystem context pointer (cast to subsystem type inside handler)
// ---------------------------------------------------------------------------
typedef void (*HandlerFn)(const ArgList& args, const char* corrId,
                          ReplyFn replyFn, void* replyCtx,
                          void* handlerCtx);

// ---------------------------------------------------------------------------
// ForceReply — controls reply channel override.
//   NONE   — reply on the channel the command arrived on
//   SERIAL — always reply via serial (used for DBG/I2C commands)
// ---------------------------------------------------------------------------
enum class ForceReply : uint8_t { NONE, SERIAL };

// ---------------------------------------------------------------------------
// CommandDescriptor — one entry in the command dispatch table.
// 24 bytes per entry; ~42 commands ≈ 1008 bytes of static BSS.
//
//   prefix      — command prefix string: "S", "DBG LOOP", "DBG LOOP RESET", …
//   parseFn     — nullptr means pass raw tokens directly to handlerFn
//   handlerFn   — command implementation
//   handlerCtx  — subsystem instance pointer; cast inside handlerFn
//   errFmt      — ERR code emitted when parseFn returns ok=false
//   forceReply  — channel override (see ForceReply)
// ---------------------------------------------------------------------------
struct CommandDescriptor {
    const char* prefix;
    ParseFn     parseFn;
    HandlerFn   handlerFn;
    void*       handlerCtx;
    const char* errFmt;
    ForceReply  forceReply;
};

// ---------------------------------------------------------------------------
// Commandable — interface for subsystems that register their own commands.
// Avoids std::vector: caller passes a fixed-size buffer and receives a count.
// ---------------------------------------------------------------------------
class Commandable {
public:
    // Fill buf[0..max-1] with this subsystem's descriptors.
    // Returns the number of descriptors written (never exceeds max).
    virtual int getCommands(CommandDescriptor* buf, int max) const = 0;
    virtual ~Commandable() {}
};

// ---------------------------------------------------------------------------
// makeCmd — inline helper to construct a CommandDescriptor without relying
// on brace-initialised aggregate literals (which are verbose in C++11).
// ---------------------------------------------------------------------------
inline CommandDescriptor makeCmd(const char* prefix, ParseFn parseFn,
                                 HandlerFn handlerFn, void* ctx,
                                 const char* errFmt  = "badarg",
                                 ForceReply  forceReply = ForceReply::NONE) {
    CommandDescriptor d;
    d.prefix      = prefix;
    d.parseFn     = parseFn;
    d.handlerFn   = handlerFn;
    d.handlerCtx  = ctx;
    d.errFmt      = errFmt;
    d.forceReply  = forceReply;
    return d;
}
