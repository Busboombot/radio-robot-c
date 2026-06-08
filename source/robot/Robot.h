#pragma once
#include "Config.h"
#include "Motor.h"
#include "OtosSensor.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "Servo.h"
#include "PortIO.h"
#include "Communicator.h"
#include "MotorController.h"
#include "Odometry.h"
#include "DriveController.h"
#include "RobotState.h"

/**
 * Robot — top-level object that owns all firmware subsystems.
 *
 * Devices (Motor, OtosSensor, LineSensor, ColorSensor, Servo, PortIO) and the
 * Communicator are constructed in main() as statics and passed in by reference.
 * Robot holds REFERENCES to them — it does NOT own or construct them.
 *
 * The control layer (MotorController, Odometry, DriveController,
 * RobotStateContainer) is still owned here.  RobotConfig is held as an owned
 * COPY initialised from the cfg argument (runtime SET commands mutate it).
 *
 * begin() for each device is called explicitly in main() before constructing
 * Robot, so a device can be disabled by commenting out its begin() call.
 *
 * Usage (main.cpp):
 *   static Motor motorL(uBit.i2c, 2, cfg.fwdSignL);
 *   static Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);
 *   comm.begin();
 *   otos.begin();
 *   static Robot robot(motorL, motorR, otos, line, color, gripper, portio, comm, cfg);
 */
class Robot {
public:
    // ---------------------------------------------------------------------------
    // Query structs — returned by query methods; formatted to wire strings by
    // CommandProcessor.
    // ---------------------------------------------------------------------------
    struct EncoderReading { int32_t leftMm; int32_t rightMm; };
    struct Pose           { int32_t x_mm; int32_t y_mm; int32_t h_cdeg; };

    Robot(Motor&        motorL,
          Motor&        motorR,
          OtosSensor&   otos,
          LineSensor&   line,
          ColorSensor&  color,
          Servo&        gripper,
          PortIO&       portio,
          Communicator& comm,
          const RobotConfig& cfg);


    // ---------------------------------------------------------------------------
    // Split-phase encoder control API (014-006 — used by LoopScheduler).
    //
    //   controlCollect(now_ms)
    //     Collects encoder readings (using the synchronous stub path; the
    //     LoopScheduler adds the inter-iteration delay via idle sleep), computes
    //     dt_s, then calls _mc.controlTick() to run PID and write PWM.
    //     Exposed public so LoopScheduler can drive the control task.
    //
    //   controlFireRequest(pendingWheel)
    //     Fires the encoder request for the wheel identified by pendingWheel
    //     (1 = left, 2 = right). Called LAST before the idle sleep to keep the
    //     motor's pending-read window free of other I2C.
    //     Returns the wheel that was just requested (same as pendingWheel).
    //
    //   controlCollectSplitPhase(now_ms, pendingWheel)
    //     Proper split-phase collect: reads back the encoder from the wheel
    //     identified by pendingWheel (1=left, 2=right), writes
    //     _state.inputs.enc{L,R}Mm, then calls _mc.controlTick().
    //     Skipped if pendingWheel == 0 (first-iteration guard — no request
    //     has been fired yet so there is nothing to collect).
    // ---------------------------------------------------------------------------
    void controlCollect(uint32_t now_ms);
    void controlFireRequest(int pendingWheel);
    void controlCollectSplitPhase(uint32_t now_ms, int pendingWheel);

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
    // Sensor read task entry points (014-007).
    //
    //   lineRead()   — read 4-channel line sensor into _state.inputs.line[];
    //                  updates lineVS.lastUpdMs and sets lineVS.valid.
    //   colorRead()  — non-blocking RGBC poll into _state.inputs.colorR/G/B/C;
    //                  updates colorVS.lastUpdMs and sets colorVS.valid.
    //   portsRead()  — read digital/analog GPIO into _state.inputs.digitalIn/analogIn;
    //                  updates portsVS.lastUpdMs and sets portsVS.valid.
    //
    // telemetryEmit — assemble the unified TLM frame from _state.inputs (no
    //                 direct sensor I2C calls) and emit via fn/ctx.
    //                 Respects tlmPeriodMs gating and SNAP flag.
    // ---------------------------------------------------------------------------
    void lineRead();
    void colorRead();
    void portsRead();
    void telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx);

    // buildTlmFrame — assemble the TLM line into buf; returns length. Shared by
    // the periodic stream and the synchronous telemetry request (SNAP reply).
    int buildTlmFrame(char* buf, int len);

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

    // Robot system time in milliseconds since boot.
    // Uses the CODAL free function system_timer_current_time() (declared in
    // codal-core Timer.h, pulled in via MicroBit.h headers).
    uint32_t systemTime() const;

    // noteActivity — mark the host as active (a command was received). Keeps the
    // periodic telemetry stream alive while the host is talking (so idle reads —
    // e.g. calibrate's pre-drive baseline — work), even when not driving. The
    // stream still auto-stops once the host goes quiet AND the robot is idle.
    void noteActivity(uint32_t now_ms) { _lastActiveMs = now_ms; }

    // ---------------------------------------------------------------------------
    // State accessor — returns the authoritative robot state container (014-003).
    // HardwareState::enc*/vel* are written each tick by controlCollect().
    // MotorCommands::tgt*/pwm* are written each tick by MotorController::controlTick().
    // ---------------------------------------------------------------------------
    const RobotStateContainer& state() const { return _state; }
    RobotStateContainer&       stateMut()    { return _state; }  // mutable accessor for LoopScheduler tasks

    // Component accessors — used by CommandProcessor and main.cpp.
    // ---------------------------------------------------------------------------
    RobotConfig&     config()          { return _config; }
    Communicator&    comm()            { return _comm; }
    MotorController& motor()           { return _mc; }
    DriveController& driveController() { return _dc; }
    Odometry&        odometry()        { return _odo; }
    OtosSensor*      otos()            { return _otos.is_initialized()  ? &_otos  : nullptr; }
    LineSensor*      lineSensor()      { return _line.is_initialized()  ? &_line  : nullptr; }
    ColorSensor*     colorSensor()     { return _color.is_initialized() ? &_color : nullptr; }
    Servo*           servo()           { return _gripperPresent ? &_servo   : nullptr; }
    PortIO&          portIO()          { return _portio; }

private:
    // Gripper angle tracking (owned here so CommandProcessor is stateless)
    int32_t _currentGripperAngle;

    // RobotConfig must be declared before Motor so fwdSign values are
    // available when the Motor constructors run (C++ initializes members
    // in declaration order).
    RobotConfig _config;

    // TLM streaming state — managed by tick(); period/fields/snap set via config().
    uint32_t _lastTlmMs;    // timestamp of last emitted TLM frame
    uint32_t _lastActiveMs; // last time a drive was active (mode != IDLE); the
                            // periodic stream auto-stops after an idle grace so it
                            // doesn't flood the link forever and block reconnect.

    // Device references (constructed externally in main(), passed in as refs).
    Motor&       _motorL;   // M2, left wheel
    Motor&       _motorR;   // M1, right wheel
    OtosSensor&  _otos;
    LineSensor&  _line;
    ColorSensor& _color;
    Servo&       _servo;
    bool         _gripperPresent;
    PortIO&      _portio;

    // Communications (owns SerialPort + Radio; begin() called in main() before Robot).
    Communicator& _comm;

    // Control layer — declared after device refs to ensure correct init order.
    MotorController  _mc;
    Odometry         _odo;
    DriveController  _dc;

    // Authoritative robot state container (014-003).
    // Owned here; written each control tick by controlCollect().
    RobotStateContainer _state;

    // Timestamp of the most recent controlCollect() call, used to compute dt_s.
    uint32_t _lastControlMs;

    // True if the previous control tick was actively driving. Used to suppress
    // the encoder read on the FIRST driving tick after a stop: on that tick the
    // 0x46 read would fire before this tick's 0x60 motor command, i.e. a read
    // while the Nezha chip is still physically stopped — the condition that
    // wedges its encoder readback (caught via stand_soak: wedge onset is the
    // first read on a stop→drive transition). One skipped read costs nothing.
    bool _prevDriving = false;

    // Slow-cadence OTOS polling: run otosCorrect() every kOtosSlowMs milliseconds.
    // Matches the cadence previously tracked by DriveController::_lastOtosMs (014-005).
    static constexpr uint32_t kOtosSlowMs = 100;  // 10 Hz OTOS correction cadence
    uint32_t _lastOtosMs;

    // (controlCollect is now public — see the split-phase encoder control API above.)
};
