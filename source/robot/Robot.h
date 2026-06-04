#pragma once
#include "MicroBit.h"
#include "Config.h"
#include "Motor.h"
#include "OtosSensor.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "Servo.h"
#include "PortIO.h"
#include "SerialPort.h"
#include "Radio.h"
#include "MotorController.h"
#include "Odometry.h"
#include "DriveController.h"
#include "RobotState.h"

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

    // ---------------------------------------------------------------------------
    // Two-fiber split (013-010):
    //   controlTick  — run from the high-priority control fiber only.
    //                  Executes encoder reads → PID → setSpeed + drive-mode
    //                  state machines.  No serial/radio I/O.
    //   telemetryTick — run from the comms+telemetry fiber.  Drains pending
    //                  EVT completions from DriveController, then assembles
    //                  and emits the unified TLM frame.  Reads only cached
    //                  encoder/velocity values; line/color I2C is safe here
    //                  because Motor I2C is now atomic (busy-wait).
    // ---------------------------------------------------------------------------
    void controlTick(uint32_t now_ms);
    void telemetryTick(uint32_t now_ms, ReplyFn fn, void* ctx);

    // ---------------------------------------------------------------------------
    // Cooperative-loop task entry points (014-004 / 014-005).
    //
    //   odometryPredict() — apply midpoint dead-reckoning from _state.inputs.encLMm/R
    //                       into _state.inputs.poseX/Y/Hrad.  Called once per
    //                       odometry-predict task slot (ticket 006 wires the scheduler).
    //   otosCorrect(now_ms) — read OTOS hardware, write _state.inputs.otosX/Y/H,
    //                         apply complementary correction to pose.  Called at the
    //                         slow cadence (100 ms).  Sole OTOS correction path —
    //                         DriveController no longer has an OTOS block (014-005).
    //   driveAdvance(now_ms) — advance S/T/D/G state machines; emit EVT completions
    //                          inline via the captured per-drive reply sink.
    // ---------------------------------------------------------------------------
    void odometryPredict();
    void otosCorrect(uint32_t now_ms);
    void driveAdvance(uint32_t now_ms);

    // ---------------------------------------------------------------------------
    // Drive action methods — delegate to DriveController.
    // fn/ctx: originating reply sink captured for async completions.
    // ---------------------------------------------------------------------------
    void stop();
    void streamDrive(int32_t leftMms, int32_t rightMms, ReplyFn fn, void* ctx);
    // VW command: body-twist keepalive (v mm/s, omega rad/s); reuses STREAMING watchdog.
    void velocityDrive(float v_mms, float omega_rads, ReplyFn fn, void* ctx,
                       const char* corr_id = nullptr);
    void timedDrive(int32_t leftMms, int32_t rightMms, uint32_t durationMs,
                    ReplyFn fn, void* ctx, const char* corr_id = nullptr);
    void distanceDrive(int32_t leftMms, int32_t rightMms, int32_t targetMm,
                       ReplyFn fn, void* ctx, const char* corr_id = nullptr);
    void goTo(float tx, float ty, float speedMms, ReplyFn fn, void* ctx,
              const char* corr_id = nullptr);

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

    // Robot system time in milliseconds since boot (uBit.systemTime()).
    uint32_t systemTime() const { return _uBit.systemTime(); }

    // ---------------------------------------------------------------------------
    // ---------------------------------------------------------------------------
    // State accessor — returns the authoritative robot state container (014-003).
    // HardwareState::enc*/vel* are written each tick by controlCollect().
    // MotorCommands::tgt*/pwm* are written each tick by MotorController::controlTick().
    // ---------------------------------------------------------------------------
    const RobotStateContainer& state() const { return _state; }

    // Component accessors — used by CommandProcessor and main.cpp.
    // ---------------------------------------------------------------------------
    RobotConfig&     config()          { return _config; }
    SerialPort&      serialPort()      { return _serial; }
    Radio&           radioPort()       { return _radio; }
    MotorController& motor()           { return _mc; }
    DriveController& driveController() { return _dc; }
    Odometry&        odometry()        { return _odo; }
    OtosSensor*      otos()            { return _otosPresent  ? &_otos  : nullptr; }
    LineSensor*      lineSensor()      { return _linePresent  ? &_line  : nullptr; }
    ColorSensor*     colorSensor()     { return _colorPresent ? &_color : nullptr; }
    Servo*           servo()            { return _gripperPresent ? &_servo   : nullptr; }
    PortIO&          portIO()          { return _portio; }

private:
    // Reference to the CODAL singleton — used by drive action helpers for systemTime().
    MicroBit& _uBit;

    // Gripper angle tracking (owned here so CommandProcessor is stateless)
    int32_t _currentGripperAngle;

    // RobotConfig must be declared before Motor so fwdSign values are
    // available when the Motor constructors run (C++ initializes members
    // in declaration order).
    RobotConfig _config;

    // TLM streaming state — managed by tick(); period/fields/snap set via config().
    uint32_t _lastTlmMs;    // timestamp of last emitted TLM frame

    // Required subsystems (constructed from received references)
    Motor      _motorL;   // M2, left wheel
    Motor      _motorR;   // M1, right wheel
    SerialPort _serial;
    Radio      _radio;

    // Optional subsystems (_*Present tracks hardware availability)
    OtosSensor   _otos;
    bool         _otosPresent;
    LineSensor   _line;
    bool         _linePresent;
    ColorSensor  _color;
    bool         _colorPresent;
    Servo        _servo;
    bool         _gripperPresent;
    PortIO       _portio;

    // Control layer — declared after _motorL/_motorR and _config to ensure correct init order.
    MotorController  _mc;
    Odometry         _odo;
    DriveController  _dc;

    // Authoritative robot state container (014-003).
    // Owned here; written each control tick by controlCollect().
    RobotStateContainer _state;

    // Timestamp of the most recent controlCollect() call, used to compute dt_s.
    uint32_t _lastControlMs;

    // Slow-cadence OTOS polling: run otosCorrect() every kOtosSlowMs milliseconds.
    // Matches the cadence previously tracked by DriveController::_lastOtosMs (014-005).
    static constexpr uint32_t kOtosSlowMs = 100;  // 10 Hz OTOS correction cadence
    uint32_t _lastOtosMs;

    // ---------------------------------------------------------------------------
    // controlCollect — collect encoder readings, convert to mm, write
    // _state.inputs.enc*, then call _mc.controlTick(_state.inputs, _state.commands, dt_s).
    // Called from controlTick() in the cooperative-loop path (014-003).
    // ---------------------------------------------------------------------------
    void controlCollect(uint32_t now_ms);
};
