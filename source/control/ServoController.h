#pragma once
#include "CommandTypes.h"
#include "IServo.h"

/**
 * ServoController — Commandable wrapper around IServo that owns the GRIP
 * command descriptor.
 *
 * GRIP <deg>  — set servo angle (0–180); replies "OK grip deg=<deg>"
 * GRIP        — query current angle; replies "OK grip deg=<current>"
 *
 * The old switch case in CommandProcessor.cpp remains live until T010.
 */
class ServoController : public Commandable {
public:
    explicit ServoController(IServo& srv);
    virtual std::vector<CommandDescriptor> getCommands() const override;

    IServo& servo() { return _srv; }

private:
    IServo& _srv;
};
