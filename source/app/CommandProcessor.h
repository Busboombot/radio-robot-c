#pragma once
#include <stdint.h>
#include "Protocol.h"

// Forward declarations to avoid circular includes.
// CommandProcessor is included by Robot.h, so cannot include Robot.h here.
class Robot;

/**
 * CommandProcessor — wire-protocol parser and dispatcher.
 *
 * Tokenizes command lines and calls Robot public methods or component
 * setters. Drive state is owned by DriveController (inside Robot);
 * this class holds no drive state of its own.
 *
 * Usage (Robot.cpp):
 *   _cmd.setRobot(this);
 *   // in loop:
 *   _cmd.process(lineBuf, replyFn, ctx);
 *   _cmd.tick(now_ms, replyFn, ctx);  // delegates to DriveController
 */
class CommandProcessor {
public:
    CommandProcessor();

    // Provide a back-pointer to Robot (called from Robot constructor).
    void setRobot(Robot* robot);

    // Parse and dispatch one command line. line must be NUL-terminated.
    // Calls replyFn(msg, ctx) for each response line.
    void process(const char* line, ReplyFn replyFn, void* ctx);

private:
    Robot* _robot;

    // Gripper state (not drive state — stays here until ticket 005)
    int32_t _currentGripperAngle;

    // Internal helpers
    static int  parseSignedArgs(const char* s, int32_t* out, int maxArgs);
    static int  clampInt(int v, int lo, int hi);
    static int  clampMinSpeed(int mms, int minSpeedMms);
};
