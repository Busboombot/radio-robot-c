#pragma once
#include <stdint.h>
#include "Protocol.h"

// Forward declaration — CommandProcessor.cpp includes Robot.h directly.
// Keeping only a forward decl here avoids including Robot.h's transitive
// header graph (MicroBit, CODAL, all subsystems) in every file that
// includes CommandProcessor.h.
class Robot;

/**
 * CommandProcessor — wire-protocol parser and dispatcher.
 *
 * Holds only a Robot reference and static parse helpers.
 * All command handlers call Robot public methods or component accessors.
 * No hardware pointers. No config pointers. No drive state.
 *
 * Usage (main.cpp):
 *   CommandProcessor cmd(robot);
 *   // in loop:
 *   cmd.process(lineBuf, replyFn, ctx);
 */
class CommandProcessor {
public:
    explicit CommandProcessor(Robot& robot);

    // Parse and dispatch one command line. line must be NUL-terminated.
    // Calls replyFn(msg, ctx) for each response line.
    void process(const char* line, ReplyFn replyFn, void* ctx);

private:
    Robot& _robot;

    // Static parse helpers
    static int  parseSignedArgs(const char* s, int32_t* out, int maxArgs);
    static int  clampInt(int v, int lo, int hi);
    static int  clampMinSpeed(int mms, int minSpeedMms);
};
