#pragma once
#include "MicroBit.h"
#include "Config.h"
#include "NezhaV2.h"
#include "OtosSensor.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "GripperServo.h"
#include "PortIO.h"
#include "SerialPort.h"
#include "Radio.h"
#include "Announcer.h"
#include "MotorController.h"
#include "Odometry.h"
#include "CommandProcessor.h"

/**
 * Robot — top-level object that owns all firmware subsystems.
 *
 * MicroBit uBit MUST be the first member declared. C++ initializes members
 * in declaration order; placing uBit first ensures the CODAL singleton is
 * fully constructed before any driver references uBit.i2c, uBit.serial, etc.
 *
 * Usage:
 *   static Robot robot;   // constructed at program start
 *   robot.run();          // enters the tick loop; never returns
 */
class Robot {
public:
    Robot();     // Constructs and initializes all subsystems; calls uBit.init()
    void run();  // Never returns; enters tick loop

private:
    // MUST be first — CODAL singleton; all other members reference its fields.
    MicroBit uBit;

    // Required subsystems (constructed from uBit references)
    NezhaV2    _motor;
    SerialPort _serial;
    Radio      _radio;
    Announcer  _announcer;
    RobotConfig _config;

    // Optional subsystems (_*Present tracks hardware availability)
    OtosSensor   _otos;
    bool         _otosPresent;
    LineSensor   _line;
    bool         _linePresent;
    ColorSensor  _color;
    bool         _colorPresent;
    GripperServo _gripper;
    bool         _gripperPresent;
    PortIO       _portio;

    // Control layer — declared after _motor and _cal to ensure correct init order.
    MotorController  _mc;
    Odometry         _odo;
    CommandProcessor _cmd;

    char _buf[256];  // shared tick-loop scratch buffer (holds a 250-byte RAW250 message)
};
