#pragma once
#include "CommandTypes.h"
#include "PortIO.h"

/**
 * PortController — Commandable wrapper around PortIO that owns the P and PA
 * command descriptors.
 *
 * P  <port> [val]  — digital read/write; replies "OK port p=<port> v=<val>"
 * PA <port> [val]  — analog read/write; replies "OK aport p=<port> v=<val>"
 *
 * The old switch cases in CommandProcessor.cpp remain live until T010.
 */
class PortController : public Commandable {
public:
    explicit PortController(PortIO& pio);
    virtual std::vector<CommandDescriptor> getCommands() const override;

    PortIO& pio() { return _pio; }

private:
    PortIO& _pio;
};
