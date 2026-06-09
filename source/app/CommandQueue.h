#pragma once
#include "CommandTypes.h"

// ---------------------------------------------------------------------------
// CommandQueue — fixed-capacity FIFO ring buffer for ParsedCommand items.
//
// Capacity is kept small (4) to stay within the nRF52 RAM budget:
//   sizeof(ParsedCommand) ≈ 424 bytes on ARM Cortex-M4 (32-bit pointers).
//   4 × 424 + 8 (head/count) = 1704 bytes added to BSS.
//
// push_back  — enqueue at tail (normal FIFO insert)
// push_front — enqueue at head (priority insert, e.g. injected commands)
// pop_front  — dequeue from head (oldest item first)
//
// All methods are no-heap, no-STL, interrupt-unsafe (single-threaded firmware).
// ---------------------------------------------------------------------------

static constexpr int COMMAND_QUEUE_CAPACITY = 4;

class CommandQueue {
    ParsedCommand _buf[COMMAND_QUEUE_CAPACITY];
    int _head  = 0;
    int _count = 0;
public:
    // Enqueue cmd at tail. Returns false if queue is full.
    bool push_back(const ParsedCommand& cmd) {
        if (_count == COMMAND_QUEUE_CAPACITY) return false;
        int tail = (_head + _count) % COMMAND_QUEUE_CAPACITY;
        _buf[tail] = cmd;
        ++_count;
        return true;
    }

    // Enqueue cmd at head (priority insert). Returns false if queue is full.
    // Items inserted here dequeue before those already in the queue.
    bool push_front(const ParsedCommand& cmd) {
        if (_count == COMMAND_QUEUE_CAPACITY) return false;
        _head = (_head - 1 + COMMAND_QUEUE_CAPACITY) % COMMAND_QUEUE_CAPACITY;
        _buf[_head] = cmd;
        ++_count;
        return true;
    }

    // Dequeue from head into out. Returns false if queue is empty.
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
