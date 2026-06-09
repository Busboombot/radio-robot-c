---
id: '001'
title: "Add CommandTypes.h \u2014 Argument, ArgList, ParseResult, CommandDescriptor,\
  \ Commandable, makeCmd"
status: done
use-cases:
- SUC-001
- SUC-002
depends-on: []
github-issue: ''
issue: commandprocessor-composable-refactor.md
completes_issue: false
---

# Add CommandTypes.h — Argument, ArgList, ParseResult, CommandDescriptor, Commandable, makeCmd

## Description

Create `source/types/CommandTypes.h` with all the new dispatch types needed for the
registration-based command system. This is pure infrastructure — no behavior change,
no modification to existing files. All subsequent tickets depend on this type header.

The file must compile cleanly with `-std=c++11 -fno-exceptions -fno-rtti`. No
`std::variant`, `std::function`, or heap allocation.

## Acceptance Criteria

- [x] `source/types/CommandTypes.h` exists and defines all of the following:
  - `ArgType` enum class (`INT`, `FLOAT`, `STR`)
  - `Argument` struct with `ArgType type`, `union { int32_t ival; float fval; }`, and `char sval[32]`
  - `MAX_ARGS = 10` constant; `ArgList` struct with `Argument args[MAX_ARGS]` and `int count`
  - `ParseError` struct with `const char* code` and `const char* detail`
  - `ParseResult` struct with `bool ok` and `union { ArgList args; ParseError err; }`
  - `ParseFn` typedef: `ParseResult (*)(const char* const* tokens, int ntokens, const KVPair* kvs, int nkv)`
  - `HandlerFn` typedef: `void (*)(const ArgList& args, const char* corrId, ReplyFn replyFn, void* replyCtx, void* handlerCtx)`
  - `ForceReply` enum class with `NONE` and `SERIAL`
  - `CommandDescriptor` struct: `prefix`, `parseFn`, `handlerFn`, `handlerCtx`, `errFmt`, `forceReply` (24 bytes per entry)
  - `Commandable` abstract class with `virtual int getCommands(CommandDescriptor* buf, int max) const = 0` and `virtual ~Commandable() {}`
  - `makeCmd` helper function
- [x] Header includes `Protocol.h` (for `ReplyFn`) and `CommandProcessor.h` (for `KVPair`) and `<stdint.h>`
- [x] `python3 build.py` passes with no errors after adding the file
- [x] No changes to any existing file

## Implementation Plan

### Approach

Create the file from scratch in `source/types/`. The type definitions follow the spec
in `.clasi/issues/commandprocessor-composable-refactor.md` exactly. `makeCmd` is an
inline helper that fills a `CommandDescriptor` struct (avoids brace-initialized
aggregate literals at every call site, which are less readable in C++11).

### Files to Create

- `source/types/CommandTypes.h` — new file, all types

### Files to Modify

None.

### Testing Plan

- Build: `python3 build.py` must pass with no errors. The header is included by nothing yet, so this is a compilation smoke test.
- Optionally: write a minimal `.cpp` in a scratch location that includes `CommandTypes.h` and declares one `CommandDescriptor` to verify the struct layout compiles; delete after verification.

### Documentation Updates

None required for this ticket.
