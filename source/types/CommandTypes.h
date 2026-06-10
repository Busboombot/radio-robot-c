#pragma once
#include <stdint.h>
#include <vector>
#include "Protocol.h"

// ---------------------------------------------------------------------------
// CommandTypes.h — foundational types for the registration-based command
// dispatch system.
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
// Command flags — bitmask values for CommandDescriptor::flags.
//   CMD_NONE            — command operates only on cached/config state
//   CMD_ACCESS_HARDWARE — command reads or writes physical hardware
//                         (motors, sensors, GPIO, I2C, servo)
// ---------------------------------------------------------------------------
static constexpr uint8_t CMD_NONE            = 0;
static constexpr uint8_t CMD_ACCESS_HARDWARE = 1;

// ---------------------------------------------------------------------------
// CommandDescriptor — one entry in the command dispatch table.
// 28 bytes per entry (flags field + 3 bytes pad to 4-byte alignment);
// ~42 commands ≈ 1176 bytes of static BSS.
//
//   prefix      — command prefix string: "S", "DBG LOOP", "DBG LOOP RESET", …
//   parseFn     — nullptr means pass raw tokens directly to handlerFn
//   handlerFn   — command implementation
//   handlerCtx  — subsystem instance pointer; cast inside handlerFn
//   errFmt      — ERR code emitted when parseFn returns ok=false
//   forceReply  — channel override (see ForceReply)
//   flags       — CMD_NONE or CMD_ACCESS_HARDWARE
// ---------------------------------------------------------------------------
struct CommandDescriptor {
    const char* prefix;
    ParseFn     parseFn;
    HandlerFn   handlerFn;
    void*       handlerCtx;
    const char* errFmt;
    ForceReply  forceReply;
    uint8_t     flags;       // CMD_NONE or CMD_ACCESS_HARDWARE
};

// ---------------------------------------------------------------------------
// Commandable — interface for subsystems that register their own commands.
// ---------------------------------------------------------------------------
class Commandable {
public:
    virtual std::vector<CommandDescriptor> getCommands() const = 0;
    virtual ~Commandable() {}
};

// ---------------------------------------------------------------------------
// makeCmd — inline helper to construct a CommandDescriptor without relying
// on brace-initialised aggregate literals (which are verbose in C++11).
// ---------------------------------------------------------------------------
inline CommandDescriptor makeCmd(const char* prefix, ParseFn parseFn,
                                 HandlerFn handlerFn, void* ctx,
                                 const char* errFmt     = "badarg",
                                 ForceReply  forceReply = ForceReply::NONE,
                                 uint8_t     flags      = CMD_NONE) {
    CommandDescriptor d;
    d.prefix      = prefix;
    d.parseFn     = parseFn;
    d.handlerFn   = handlerFn;
    d.handlerCtx  = ctx;
    d.errFmt      = errFmt;
    d.forceReply  = forceReply;
    d.flags       = flags;
    return d;
}

// ---------------------------------------------------------------------------
// ParsedCommand — a fully parsed command ready for dispatch or queuing.
//   desc      — points to the registered CommandDescriptor
//   args      — parsed argument list (from parseFn, or empty)
//   replyFn   — reply callback to call for each response line
//   replyCtx  — opaque context forwarded to replyFn
//   corrId    — correlation id string (up to 7 chars + NUL)
// ---------------------------------------------------------------------------
struct ParsedCommand {
    const CommandDescriptor* desc;
    ArgList  args;
    ReplyFn  replyFn;
    void*    replyCtx;
    char     corrId[8];
};
