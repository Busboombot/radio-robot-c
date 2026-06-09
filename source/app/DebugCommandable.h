#pragma once
#include "CommandTypes.h"

// Forward declarations — keeps the header-graph shallow.
class LoopScheduler;
class I2CBus;
struct Robot;

// ---------------------------------------------------------------------------
// DbgCtx — context bundle passed to every DBG / I2CW / I2CR handler.
// ---------------------------------------------------------------------------
struct DbgCtx {
    LoopScheduler* sched;
    I2CBus*        bus;
    Robot*         robot;
};

// ---------------------------------------------------------------------------
// DebugCommandable — Commandable that owns all diagnostic commands:
//   DBG LOOP RESET, DBG LOOP, DBG I2CLOG, DBG I2C, DBG IRQGUARD,
//   DBG WEDGE, I2CW, I2CR.
//
// All descriptors use ForceReply::SERIAL so debug output always goes to
// the serial port regardless of which channel the command arrived on.
//
// Handler logic mirrors the existing switch cases in CommandProcessor.cpp
// exactly.  Those switch cases remain live until T011 cutover.
// ---------------------------------------------------------------------------
class DebugCommandable : public Commandable {
public:
    explicit DebugCommandable(DbgCtx ctx);

    virtual std::vector<CommandDescriptor> getCommands() const override;

    // Accessor used by handler functions in DebugCommandable.cpp.
    DbgCtx ctx() const { return _ctx; }

private:
    DbgCtx _ctx;
};
