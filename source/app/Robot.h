#pragma once
#include "MicroBit.h"
#include "types/Config.h"
#include "hal/NezhaV2.h"
#include "hal/OtosSensor.h"
#include "hal/LineSensor.h"
#include "hal/ColorSensor.h"
#include "hal/GripperServo.h"
#include "hal/PortIO.h"
#include "hal/SerialPort.h"
#include "hal/Radio.h"
#include "app/Announcer.h"

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
    CalibParams _cal;

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

    char _buf[128];  // shared tick-loop scratch buffer
};
