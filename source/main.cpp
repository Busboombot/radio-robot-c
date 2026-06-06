#include "MicroBit.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "LoopScheduler.h"
#include "Communicator.h"
#include "I2CBus.h"
#include "Motor.h"
#include "OtosSensor.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "Servo.h"
#include "PortIO.h"
#include "Config.h"
#include "Icons.h"

// ---------------------------------------------------------------------------
// MicroBit uBit singleton — file-scope so CODAL peripherals are fully
// initialised before any device is constructed in main().
// ---------------------------------------------------------------------------
static MicroBit uBit;

// ---------------------------------------------------------------------------
// serialReply — thin adapter used for the boot HELLO banner.
// ---------------------------------------------------------------------------
static void serialReply(const char* msg, void* ctx)
{
    static_cast<SerialPort*>(ctx)->send(msg);
}

// ---------------------------------------------------------------------------
// main — construct all devices, call begin() explicitly, then run the single
// cooperative loop.
//
// Device ordering:
//   1. uBit.init() — all CODAL peripherals ready.
//   2. Static RobotConfig cfg — Motor constructors need fwdSign values.
//   3. Device singletons — each holds a reference to the CODAL bus/pin.
//   4. Communicator — begin() enables serial and radio.
//   5. Sensor begin() calls — straight-line, before the loop.
//      Comment a line out to disable that sensor; its task skips via
//      is_initialized() on every subsequent read.
//   6. Robot — built from its devices + communicator.
//   7. CommandProcessor / LoopScheduler — wired and started.
//
// Sensor detection rationale:
//   * NOT in the Robot constructor — detecting that early reads the line/color
//     chips before they have powered up and wedges the I2C bus.
//   * NOT inside the loop — the per-sensor retries would freeze the loop.
//   A short settle delay gives the sensors time to power up; each begin()
//   internally retries until the chip answers (mirrors the old firmware).
// ---------------------------------------------------------------------------
int main() {
    uBit.init();

    // Heart on the LED matrix — "powered and ready" (non-blocking).
    uBit.display.printAsync(icons::boot());

    // -----------------------------------------------------------------------
    // 2. Config (needed by Motor constructor for fwdSign values).
    // -----------------------------------------------------------------------
    static RobotConfig cfg = defaultRobotConfig();

    // -----------------------------------------------------------------------
    // 3. Devices (singletons) on the buses.
    //
    // I2CBus sits between uBit.i2c and every device so per-device transaction
    // counts, error rates, and re-entrancy violations are observable via the
    // DBG I2C command (ticket 015-003) without altering any transaction semantics.
    // -----------------------------------------------------------------------
    static I2CBus       bus(uBit.i2c);
    static Motor        motorL(bus, 2, cfg.fwdSignL);   // M2 left
    static Motor        motorR(bus, 1, cfg.fwdSignR);   // M1 right
    static OtosSensor   otos(bus, cfg);
    static LineSensor   line(bus);
    static ColorSensor  color(bus);
    static Servo        gripper(uBit.io.P1);
    static PortIO       portio(uBit.io);

    // -----------------------------------------------------------------------
    // 4. Communications — begin() enables serial + radio.
    // -----------------------------------------------------------------------
    static Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);
    comm.begin();

    // -----------------------------------------------------------------------
    // 5. Device initialisation — comment a line out to disable that device.
    //    begin() sets is_initialized(); read paths check it each tick.
    // -----------------------------------------------------------------------
    // Settle so the sensors have time to power up before begin() probes them.
    uBit.sleep(2500);
    otos.begin();
    line.begin();
    color.begin();

    // -----------------------------------------------------------------------
    // 6. Robot — built from its devices + communicator (no i2c/serial/radio/
    //    MicroBit refs; those are fully encapsulated by the device objects).
    // -----------------------------------------------------------------------
    static Robot            robot(motorL, motorR, otos, line, color, gripper, portio, comm, cfg);
    static CommandProcessor cmd(robot);

    // DEVICE: identification banner once at boot over serial.
    cmd.process("HELLO", serialReply, &comm.serial());

    // -----------------------------------------------------------------------
    // 7. Run the cooperative main loop — never returns.
    // -----------------------------------------------------------------------
    static LoopScheduler sched(robot, cmd, comm, uBit);
    cmd.setScheduler(&sched);             // enable DBG LOOP <x> <state> task toggling
    cmd.setI2CBus(&bus);                  // enable DBG I2C stats dump (015-003)

    // Wire the I2CBus and EVT sink into MotorController so enc_wedged events
    // are emitted with bus stats and go to the active serial/radio channel.
    robot.motor().setI2CBus(&bus);
    robot.motor().setEvtSink(&sched.activeFn, &sched.activeCtx);

    sched.run_all();

    return 0;
}
