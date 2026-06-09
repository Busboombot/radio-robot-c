---
id: '003'
title: Add CommandProcessor new constructor and dispatchTable() path (dual-mode with
  old switch behind _cmds==nullptr guard)
status: done
use-cases:
- SUC-002
- SUC-004
depends-on:
- '001'
- '002'
github-issue: ''
issue: ''
completes_issue: false
---

# Add CommandProcessor new constructor and dispatchTable() path (dual-mode with old switch behind _cmds==nullptr guard)

## Description

Add the table-dispatch infrastructure to `CommandProcessor` without enabling it yet.
This ticket installs the dual-mode routing — when `_cmds == nullptr` (old constructor),
the existing switch runs; when `_cmds != nullptr` (new constructor), the
`dispatchTable()` path runs. After this ticket, `main.cpp` still uses the old
`CommandProcessor(robot)` constructor, so no behavior changes in the running firmware.
The new path is wired but dormant.

## Acceptance Criteria

- [x] `CommandProcessor.h` declares:
  - New constructor: `CommandProcessor(const CommandDescriptor* cmds, int count)`
  - `void setSerialReply(ReplyFn fn, void* ctx)`
  - Private members: `const CommandDescriptor* _cmds = nullptr`, `int _cmdCount = 0`,
    `ReplyFn _serialFn = nullptr`, `void* _serialCtx = nullptr`
  - Old `CommandProcessor(Robot& robot)` constructor retained
- [x] `CommandProcessor.cpp` implements `dispatchTable()` as a private method:
  - Longest-prefix linear scan over `_cmds[0.._cmdCount-1]`
  - `prefixMatchLen(const char* prefix, char** tokens, int ntok)` helper
  - Calls `parseFn` if non-null; replies `ERR errFmt` on parse failure without calling handler
  - Substitutes `_serialFn`/`_serialCtx` for `ForceReply::SERIAL` descriptors
  - Falls through to `ERR unknown` if no prefix matches
- [x] `process()` routes to `dispatchTable()` when `_cmds != nullptr`, old switch when `_cmds == nullptr`
- [x] `python3 build.py` passes with no errors
- [x] All existing commands continue to work (old path still active; no behavior change)

## Implementation Plan

### Approach

Add the new constructor and members to `CommandProcessor.h`. Implement `dispatchTable()`
as a private method in `CommandProcessor.cpp`. Gate the call in `process()` with
`if (_cmds != nullptr) { dispatchTable(tokens, ntok, kvs, nkv, corrId, replyFn, ctx); return; }`.
The `prefixMatchLen` helper tokenizes the descriptor's `prefix` string on spaces and
compares token-by-token against the incoming `tokens[]` array.

### Files to Modify

- `source/app/CommandProcessor.h` — new constructor, `setSerialReply`, new members
- `source/app/CommandProcessor.cpp` — `dispatchTable()` implementation; `process()` routing gate

### Testing Plan

- Build: `python3 build.py` must pass.
- Behavioral smoke: Connect robot (old constructor still in use); verify S, PING, GET, DBG LOOP
  all work identically — the new code path is unreachable with `_cmds == nullptr`.
- Unit test for `prefixMatchLen` if the function is made accessible; otherwise verify
  via integration test after T010.
