#pragma once
#include <vector>
#include "../types/CommandTypes.h"

// ---------------------------------------------------------------------------
// ConfigCommands — config-registry command descriptors (GET, GET VEL, SET).
//
// Extracted from source/app/SystemCommands.cpp (finding A3 split). The
// parse*/handle* functions for these commands remain file-local statics inside
// ConfigCommands.cpp; this seam lets Robot::buildCommandTable (still in
// SystemCommands.cpp) register them across the translation-unit boundary.
//
// Behaviour-preserving: descriptors are wired exactly as buildCommandTable
// previously wired them:
//   - GET     -> parseGet/handleGet bound to CfgCtx*       (errFmt "badkey")
//   - SET     -> parseSet/handleSet bound to CfgCtx*       (errFmt "badkey")
//   - GET VEL -> parseGetVel/handleGetVel bound to RobotSysCtx* (errFmt "badarg")
//
// handleGet/handleSet are the free functions declared in ConfigRegistry.h
// (defined in source/robot/ConfigRegistry.cpp) — unchanged.
// ---------------------------------------------------------------------------

struct CfgCtx;  // defined in ConfigRegistry.h

// appendConfigCommands — push the GET VEL, GET, and SET descriptors onto cmds.
//
//   cfgCtx — CfgCtx* used by the GET and SET descriptors (handleGet/handleSet).
//   sysCtx — RobotSysCtx* (as a void*) used by the GET VEL descriptor
//            (handleGetVel reads robot->state.inputs.vel*).  Passed as void*
//            so this header need not include Robot.h.
//
// Both pointers must remain live for the lifetime of the returned descriptors
// (they are Robot members, so they are).
void appendConfigCommands(std::vector<CommandDescriptor>& cmds,
                          CfgCtx* cfgCtx, void* sysCtx);
