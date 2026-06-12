#pragma once
#include <vector>
#include "CommandTypes.h"

// ---------------------------------------------------------------------------
// MotionCommandHandlers — app-layer motion command parsing, conversion, and
// reply formatting.  Extracted from MotionController (sprint 026, ticket 002).
//
// Dependency direction: app/ → control/.
// All CommandProcessor::replyOK/Err/Evt calls live here, not in control/.
// ---------------------------------------------------------------------------

class MotionController;
class CommandQueue;
struct Robot;

// ---------------------------------------------------------------------------
// MotionCtx — context bundle used by Commandable-registered motion handlers.
//
// Moved from source/control/MotionController.h to app layer (sprint 026-002).
// ---------------------------------------------------------------------------
struct MotionCtx {
    MotionController*  mc;
    struct Robot*      robot;
    CommandQueue*      queue;    // command queue for VW converter push_front; may be null
    CommandDescriptor  vwDesc;   // stable VW descriptor used by converters to build ParsedCommand
};

// ---------------------------------------------------------------------------
// getMotionCommands — returns the full set of motion CommandDescriptors
// (S, T, D, G, R, TURN, RT, VW, _VW, X, STOP) bound to ctx.
//
// Replaces MotionController::getCommands().  Called by Robot::buildCommandTable().
// ctx must remain live for the lifetime of the returned descriptors.
// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> getMotionCommands(MotionCtx* ctx);
