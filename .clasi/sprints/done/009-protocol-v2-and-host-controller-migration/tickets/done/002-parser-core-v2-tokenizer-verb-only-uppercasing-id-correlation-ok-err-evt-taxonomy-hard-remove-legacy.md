---
id: '002'
title: 'Parser core v2: tokenizer, verb-only uppercasing, #id correlation, OK/ERR/EVT
  taxonomy, hard-remove legacy'
status: done
use-cases:
- SUC-002
- SUC-003
- SUC-004
- SUC-006
depends-on:
- '001'
issue: protocol-v2-raw250-hard-break.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 009-002: Parser core v2 — tokenizer, verb-only uppercasing, #id correlation, response taxonomy, hard-remove legacy

## Description

Rewrite `CommandProcessor::process()` to implement the v2 parsing foundation.
This ticket establishes the structural plumbing that all subsequent command tickets
(003–006) build on. It does **not** implement any specific commands beyond what is
needed to prove the parser works end-to-end with a single test command.

**What changes:**

1. **Tokenizer** — split the input line on whitespace into tokens. Upper-case only
   the first token (the verb). Leave all other tokens in their original case.
   Remove the old `toupper()` loop that upper-cased the entire line.

2. **`#id` correlation** — scan tokens for a trailing `#<digits>` token. If found,
   strip it from the token list and store the correlation ID. Every response for
   this command must append ` #<id>` before the newline.

3. **`key=value` token detection** — a helper `parseKV(tokens, out_map)` that
   splits tokens containing `=` into key/value pairs. No heap — use a fixed-size
   stack array of `{key, value}` pairs (max 24 entries, one per config param).

4. **Response taxonomy helpers** — static inline helpers for constructing
   prefixed replies:
   - `replyOK(buf, size, verb, body, id)` → `"OK <verb> <body> [#id]\n"`
   - `replyErr(buf, size, code, detail, id)` → `"ERR <code> <detail> [#id]\n"`
   - `replyEvt(buf, size, name, body)` → `"EVT <name> <body>\n"` (async, no #id)

5. **Hard-remove legacy** — delete `parseSignedArgs()` and all legacy command
   handlers (S-packed, T-packed, D-packed, G-packed, K*, ENC, EZ, SO, SZ, SI,
   OI, OK, OZ, OR, OP, OV, OL, OA, O, LS, CS, PA, P, the HELLO/DEVICE: intercept
   in Announcer). Delete `Announcer` class entirely.

6. **Stub fallback** — after removing legacy handlers, the parser dispatches on
   the verb. Unrecognized verbs reply `ERR unknown <verb>`. This is the only
   handler needed in this ticket to prove the parser works.

7. **`Protocol.h` update** — replace v1 string constants with v2 tag constants
   (`"OK"`, `"ERR"`, `"EVT"`, `"TLM"`, `"CFG"`, `"ID"`).

## Acceptance Criteria

- [x] `process()` tokenizes on whitespace; only the verb token is upper-cased.
- [x] `#7` in `UNKNOWNCMD arg1 #7` → response echoes `#7`.
- [x] `key=value` tokens are parsed correctly; `=` without a key or value yields `ERR badarg`.
- [x] `replyOK`, `replyErr`, `replyEvt` helpers produce correctly prefixed lines.
- [x] All legacy command handlers are deleted; `parseSignedArgs()` is deleted.
- [x] `Announcer` class is deleted; `main.cpp` no longer constructs it; `Robot.h` no longer holds it.
- [x] `Protocol.h` has v2 tag constants and no v1 constants.
- [x] Unrecognized verb → `ERR unknown <verb>`.
- [x] Firmware builds without error or warning.
- [ ] Serial test: send `HELLO` → `ERR unknown HELLO` (legacy gone); send `FOO #3` → `ERR unknown FOO #3`. [DEFERRED — bench test at sprint end per stakeholder process]

## Implementation Plan

**Approach**: In-place rewrite of `CommandProcessor.cpp`. Delete `Announcer.h` and
`Announcer.cpp`. Update `main.cpp` and `Robot.h` to remove Announcer references.

**Files to modify**:
- `source/app/CommandProcessor.cpp` — full rewrite of `process()`; add `parseTokens()`, `parseKV()`, `replyOK()`, `replyErr()`, `replyEvt()` statics
- `source/app/CommandProcessor.h` — update declarations; remove `parseSignedArgs()`
- `source/types/Protocol.h` — replace v1 constants with v2 tags
- `source/app/Announcer.h` / `source/app/Announcer.cpp` — delete both files
- `source/robot/Robot.h` — remove `Announcer _announcer` member and `announcer()` accessor
- `source/app/Robot.cpp` — remove Announcer construction and startup call
- `main.cpp` — remove Announcer references

**Token/KV parser spec**:
```
// Splits line into at most MAX_TOKENS tokens.
// Returns count. Tokens point into a local copy; caller must not free.
// Upper-cases tokens[0] (the verb) in place.
// If the last token starts with '#', extracts it as corr_id (digits only).
int parseTokens(const char* line, char** tokens, int maxTokens, char* corr_id, int corrIdSize);

// Scans tokens[1..n] for key=value pairs. Fills kvs[]. Returns kv count.
// Tokens without '=' are left as positional args (caller handles separately).
int parseKV(char** tokens, int ntokens, KVPair* kvs, int maxKV);
```

**Testing**:
- Build and send `FOO` → `ERR unknown FOO`
- Send `FOO #3` → `ERR unknown FOO #3`
- Send `FOO key=val` → `ERR unknown FOO` (key=val parsed but discarded by stub)
- Send `HELLO` → `ERR unknown HELLO` (legacy gone)
- Send `STOP` (will not be recognized yet) → `ERR unknown STOP` (OK — commands come in later tickets)

**Documentation**: None needed at this stage; spec doc is ticket 009.
