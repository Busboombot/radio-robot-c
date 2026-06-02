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
 * MicroBit uBit now lives in main.cpp as a file-scope static. Robot
 * receives references to the CODAL peripherals it needs so that hardware
 * ownership is explicit and Robot is a pure abstraction layer.
 *
 * Construction order is preserved: main.cpp calls uBit.init() before
 * constructing Robot, so all CODAL peripherals are fully initialised
 * when the subsystem constructors run.
 *
 * Usage (main.cpp):
 *   static MicroBit uBit;
 *   uBit.init();
 *   static Robot robot(uBit.i2c, uBit.serial, uBit.radio, uBit.io,
 *                      uBit.messageBus, uBit);
 *   robot.run();
 */
class Robot {
public:
    Robot(MicroBitI2C&    i2c,
          NRF52Serial&    serial,
          MicroBitRadio&  radio,
          MicroBitIO&     io,
          MessageBus&     messageBus,
          MicroBit&       uBit);

    void run();  // Never returns; enters tick loop

private:
    // Reference to the CODAL singleton — used for systemTime() in run().
    MicroBit& _uBit;

    // Required subsystems (constructed from received references)
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
