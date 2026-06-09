---
id: '010'
title: CommandQueue ring buffer + CommandProcessor queue integration
status: open
use-cases:
  - SUC-013
depends-on:
  - 020-009
github-issue: ''
issue: plan-command-flags-vw-unification-command-queue-and-test-loop.md
completes_issue: false
---

# CommandQueue ring buffer + CommandProcessor queue integration

## Description

Create the `CommandQueue` fixed-capacity ring buffer and integrate it into
`CommandProcessor`. When `CommandProcessor` has a queue set (via `setQueue()`), calls
to `process()` parse and enqueue the command instead of dispatching immediately. A new
`dequeueOne()` method dispatches one item from the queue.

`LoopScheduler` owns the queue and sets it on the CommandProcessor at boot. The queue
is drained in the tick body via `dequeueOne()`.

This ticket does NOT yet convert S/T/D/G/R/TURN to VW converters (that is ticket
020-011). In normal `run_blocks()` mode, the queue is fully transparent: commands are
enqueued and dequeued in the same tick body, producing identical behavior to the
current dispatch-immediate path.

## Acceptance Criteria

- [ ] `source/app/CommandQueue.h` created: `ParsedCommand _buf[16]`; `int _head`, `_count`; `push_back`, `push_front`, `pop_front`, `empty()`, `size()`; all no-heap, no-STL.
- [ ] `push_front` inserts at head (decrements `_head` modulo capacity); items dequeue in LIFO order for head, FIFO for back.
- [ ] `push_back` returns false when full; `push_front` returns false when full.
- [ ] `CommandProcessor` has `CommandQueue* _queue = nullptr` member; `setQueue(CommandQueue*)` setter.
- [ ] When `_queue != nullptr`, `process()` parses the command and calls `_queue->push_back()` instead of dispatching.
- [ ] `bool dequeueOne(CommandQueue& q)` added to CommandProcessor: dispatches one item from q; returns false if empty.
- [ ] `LoopScheduler` owns `CommandQueue _queue`; calls `cmd.setQueue(&_queue)` at boot.
- [ ] `LoopScheduler::run_blocks()` calls `cmd.dequeueOne(_queue)` in the tick body after processing inbound commands.
- [ ] Behavior in `run_blocks()` mode is unchanged: commands arrive, are enqueued, and dequeued in the same tick — net effect is identical to immediate dispatch.
- [ ] `push_front` / `pop_front` ordering verified: push_back A, B, C then push_front Z → pop order is Z, A, B, C.
- [ ] `python3 build.py --clean` passes.
- [ ] `uv run --with pytest python -m pytest` passes.

## Implementation Plan

### Approach

1. Create `source/app/CommandQueue.h` (header-only; no .cpp needed for a pure-data
   ring buffer).
2. Modify `CommandProcessor.h/.cpp`: add `_queue` pointer; add `setQueue()`; branch in
   `process()` on `_queue != nullptr`; add `dequeueOne()`.
3. Modify `LoopScheduler.h`: add `CommandQueue _queue` member.
4. Modify `LoopScheduler.cpp`: call `cmd.setQueue(&_queue)` in constructor or at start
   of `run_blocks()`; call `cmd.dequeueOne(_queue)` in tick body.
5. Build and verify `run_blocks()` behavior unchanged via bench drive command.

### Files to Create

- `source/app/CommandQueue.h`

### Files to Modify

- `source/app/CommandProcessor.h` — add `_queue`, `setQueue()`, `dequeueOne()`
- `source/app/CommandProcessor.cpp` — implement queue branch in `process()`, `dequeueOne()`
- `source/control/LoopScheduler.h` — add `CommandQueue _queue` member
- `source/control/LoopScheduler.cpp` — set queue at boot; drain queue in tick body

### CommandQueue ring buffer implementation

```cpp
static constexpr int COMMAND_QUEUE_CAPACITY = 16;

class CommandQueue {
    ParsedCommand _buf[COMMAND_QUEUE_CAPACITY];
    int _head  = 0;
    int _count = 0;
public:
    bool push_back(const ParsedCommand& cmd) {
        if (_count == COMMAND_QUEUE_CAPACITY) return false;
        int tail = (_head + _count) % COMMAND_QUEUE_CAPACITY;
        _buf[tail] = cmd;
        ++_count;
        return true;
    }
    bool push_front(const ParsedCommand& cmd) {
        if (_count == COMMAND_QUEUE_CAPACITY) return false;
        _head = (_head - 1 + COMMAND_QUEUE_CAPACITY) % COMMAND_QUEUE_CAPACITY;
        _buf[_head] = cmd;
        ++_count;
        return true;
    }
    bool pop_front(ParsedCommand& out) {
        if (_count == 0) return false;
        out = _buf[_head];
        _head = (_head + 1) % COMMAND_QUEUE_CAPACITY;
        --_count;
        return true;
    }
    bool empty() const { return _count == 0; }
    int  size()  const { return _count; }
};
```

### RAM budget check

`ParsedCommand` contains `ArgList` (10 × 40 bytes = 400 bytes) + pointer + corrId[8]
≈ 416 bytes. 16 × 416 = 6656 bytes for the queue buffer. This is significant — check
BSS output after this ticket. If it exceeds budget, reduce COMMAND_QUEUE_CAPACITY to 8
(3328 bytes) or MAX_ARGS to 6 (6 × 40 + overhead ≈ 258 bytes × 16 = 4128 bytes).

### Testing Plan

1. `python3 build.py --clean` — zero warnings; check BSS delta.
2. Flash via `mbdeploy deploy robot --clean`.
3. Bench: `D dist=500` → drives 500 mm; `EVT done D` received. Queue-transparent path.
4. Bench: `PING` → `OK ping ms=...` received. Simple round-trip.
5. `uv run --with pytest python -m pytest` — no regressions.

### Notes

- `ParsedCommand` was defined in ticket 020-009 (`CommandTypes.h`). Verify it is
  available here before creating `CommandQueue.h`.
- The `dequeueOne()` implementation must call the descriptor's `handlerFn` directly
  (not `process()` recursively) to avoid re-enqueuing when `_queue` is still set.
  Internal dispatch pattern:
  ```cpp
  bool CommandProcessor::dequeueOne(CommandQueue& q) {
      ParsedCommand pc;
      if (!q.pop_front(pc)) return false;
      pc.desc->handlerFn(pc.args, pc.corrId, pc.replyFn, pc.replyCtx, pc.desc->handlerCtx);
      return true;
  }
  ```
- The queue is intentionally not drained fully in one tick — only `dequeueOne()` is
  called once per tick in `run_blocks()`. This limits per-tick processing time. For
  `run_test()` (ticket 020-011), the inner loop drains all pending items.
