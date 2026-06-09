#pragma once
#include "CommandTypes.h"
#include "Servo.h"

/**
 * ServoController — Commandable wrapper around Servo that owns the GRIP
 * command descriptor.
 *
 * GRIP <deg>  — set servo angle (0–180); replies "OK grip deg=<deg>"
 * GRIP        — query current angle; replies "OK grip deg=<current>"
 *
 * The old switch case in CommandProcessor.cpp remains live until T010.
 */
class ServoController : public Commandable {
public:
    explicit ServoController(Servo& srv);
    virtual int getCommands(CommandDescriptor* buf, int max) const override;

    Servo& servo() { return _srv; }

private:
    Servo& _srv;
};
