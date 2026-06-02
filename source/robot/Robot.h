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
#include "DriveController.h"

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
 *   // then run the visible main loop — see main.cpp.
 */
class Robot {
public:
    // ---------------------------------------------------------------------------
    // Query structs — returned by query methods; formatted to wire strings by
    // CommandProcessor.
    // ---------------------------------------------------------------------------
    struct EncoderReading { int32_t leftMm; int32_t rightMm; };
    struct Pose           { int32_t x_mm; int32_t y_mm; int32_t h_cdeg; };

    Robot(MicroBitI2C&    i2c,
          NRF52Serial&    serial,
          MicroBitRadio&  radio,
          MicroBitIO&     io,
          MessageBus&     messageBus,
          MicroBit&       uBit);

    // Advance all subsystems by one tick. Call from main loop each iteration.
    // now_ms: current system time (uBit.systemTime()).
    // fn/ctx: reply sink for the active channel (used for streaming telemetry).
    // Per-drive async completions (T+DONE, D+DONE, G+DONE, SAFETY_STOP) use
    // the sink that was captured when the drive command began.
    void tick(uint32_t now_ms, ReplyFn fn, void* ctx);

    // ---------------------------------------------------------------------------
    // Drive action methods — delegate to DriveController.
    // fn/ctx: originating reply sink captured for async completions.
    // ---------------------------------------------------------------------------
    void stop();
    void streamDrive(int32_t leftMms, int32_t rightMms, ReplyFn fn, void* ctx);
    void timedDrive(int32_t leftMms, int32_t rightMms, uint32_t durationMs,
                    ReplyFn fn, void* ctx);
    void distanceDrive(int32_t leftMms, int32_t rightMms, int32_t targetMm,
                       ReplyFn fn, void* ctx);
    void goTo(float tx, float ty, float speedMms, ReplyFn fn, void* ctx);

    // ---------------------------------------------------------------------------
    // Non-drive action methods
    // ---------------------------------------------------------------------------
    void setGripperAngle(int32_t deg);
    void zeroEncoders();
    void setPose(int32_t x_mm, int32_t y_mm, int32_t h_cdeg);
    void zeroOdometry();

    // ---------------------------------------------------------------------------
    // Query methods — return plain structs; callers format wire strings.
    // ---------------------------------------------------------------------------
    EncoderReading getEncoders() const;
    Pose           getPose()     const;

    // Current gripper angle (set by setGripperAngle / G command)
    int32_t gripperAngle() const { return _currentGripperAngle; }

    // ---------------------------------------------------------------------------
    // Component accessors — used by CommandProcessor for K*/O* setters
    // and by main.cpp to obtain the HAL objects needed by reply sinks.
    // ---------------------------------------------------------------------------
    RobotConfig&     config()          { return _config; }
    SerialPort&      serialPort()      { return _serial; }
    Radio&           radioPort()       { return _radio; }
    Announcer&       announcer()       { return _announcer; }
    MotorController& motor()           { return _mc; }
    DriveController& driveController() { return _dc; }
    Odometry&        odometry()        { return _odo; }
    OtosSensor*      otos()            { return _otosPresent  ? &_otos  : nullptr; }
    LineSensor*      lineSensor()      { return _linePresent  ? &_line  : nullptr; }
    ColorSensor*     colorSensor()     { return _colorPresent ? &_color : nullptr; }
    GripperServo*    gripper()         { return _gripperPresent ? &_gripper : nullptr; }
    PortIO&          portIO()          { return _portio; }

private:
    // Sensor streaming callback registered with DriveController
    static void sensorReport(ReplyFn fn, void* ctx, void* sensorCtx);


    // Reference to the CODAL singleton — used by drive action helpers for systemTime().
    MicroBit& _uBit;

    // Gripper angle tracking (owned here so CommandProcessor is stateless)
    int32_t _currentGripperAngle;

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

    // Control layer — declared after _motor and _config to ensure correct init order.
    MotorController  _mc;
    Odometry         _odo;
    DriveController  _dc;
};
