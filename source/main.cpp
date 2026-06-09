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
#include "RadioChannel.h"
#include "DebugCommandable.h"
#include <cstdio>

// ---------------------------------------------------------------------------
// MicroBit uBit singleton — file-scope so CODAL peripherals are fully
// initialised before any device is constructed in main().
// ---------------------------------------------------------------------------
static MicroBit uBit;

// ---------------------------------------------------------------------------
// bootSelectRadioChannel — resolve the radio channel at boot, with display.
//
// Loads the persisted channel (default 0), shown as a single base-36 character
// (0-9 then A-Z, so channel 10 = 'A').
//
//   * Normal boot (A+B not both held): flash the channel character briefly,
//     then the heart. Returns the stored channel unchanged.
//   * Edit boot (hold A+B at power-on): the channel character stays on the LED
//     while you press A (−1) / B (+1); each press updates the character in
//     place (clamped 0..35). After ~5 s with no input it saves if changed,
//     flashes a checkmark, then shows the heart.
//
// The `RF` command (USB serial) is the precise/scripted path. Either way the
// display ends on the boot heart.
// ---------------------------------------------------------------------------
static int bootSelectRadioChannel(MicroBit& uBit)
{
    int channel = radiochan::load(uBit.storage);

    // Enter edit mode only when BOTH buttons are held at boot. Any other case
    // (nothing held, or a single button) is a normal boot: flash the channel
    // character, then the heart.
    if (!(uBit.buttonA.isPressed() && uBit.buttonB.isPressed())) {
        uBit.display.printCharAsync(radiochan::toChar(channel));
        uBit.sleep(900);
        uBit.display.printAsync(icons::boot());
        return channel;
    }

    // Edit mode — show the channel statically and keep it on as buttons change it.
    const int original = channel;
    uBit.display.printCharAsync(radiochan::toChar(channel));

    // Wait for the boot-held button(s) to be released before honoring input.
    // Otherwise holding A or B (or A+B) to ENTER edit mode is immediately read
    // as a step or as the A+B confirm, and we exit before you can adjust.
    while (uBit.buttonA.isPressed() || uBit.buttonB.isPressed()) {
        uBit.sleep(20);
    }

    bool prevA = false, prevB = false;        // buttons are now released
    uint32_t lastInputMs = uBit.systemTime(); // start the idle timer after release

    while (uBit.systemTime() - lastInputMs < 5000) {
        bool a = uBit.buttonA.isPressed();
        bool b = uBit.buttonB.isPressed();

        // Confirm + exit on a simultaneous A+B press.
        if (a && b) {
            break;
        }
        // Rising-edge detect so one press = one step; redraw the character in place.
        if (a && !prevA) {
            channel = radiochan::clamp(channel - 1);
            uBit.display.printCharAsync(radiochan::toChar(channel));
            lastInputMs = uBit.systemTime();
        } else if (b && !prevB) {
            channel = radiochan::clamp(channel + 1);
            uBit.display.printCharAsync(radiochan::toChar(channel));
            lastInputMs = uBit.systemTime();
        }
        prevA = a;
        prevB = b;
        uBit.sleep(20);
    }

    if (channel != original) {
        radiochan::save(uBit.storage, channel);
    }
    // Confirm: flash a checkmark, then settle on the heart.
    uBit.display.printAsync(icons::tick());
    uBit.sleep(900);
    uBit.display.printAsync(icons::boot());
    return channel;
}

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
//   6. Robot — built from its devices; owns config, state, and controllers.
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

    // Radio channel: load persisted channel (default 0); hold A/B at boot to
    // edit it. Resolved BEFORE the radio starts so it comes up on the right band.
    // Flashes the channel character then the heart (handles its own display).
    int rfChannel = bootSelectRadioChannel(uBit);

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

    // I2C bus speed — 100 kHz (uBit.init() defaults to 400 kHz).
    //
    // The Nezha V2 motor controller's encoder readback (0x46) wedges — freezes
    // at a constant while the wheels keep spinning — under fast, frequently
    // interleaved 0x60-write / 0x46-read traffic at 400 kHz. The WedgeTest bench
    // harness (DBG WEDGE) proved that dropping the bus to 100 kHz, reading BOTH
    // encoders every tick (M1 first), and writing motors only on change runs
    // 10 min / zero wedges. 400 kHz wedged within ~165 ticks. The other sensors
    // (OTOS/line/color) are speed-agnostic for correctness, so 100 kHz is safe
    // bus-wide. See docs/knowledge encoder-wedge note + WedgeTest.cpp.
    uBit.i2c.setFrequency(100000);

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
    comm.begin(rfChannel);

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
    // 6. Robot — built from devices; owns config, state, and controllers.
    //    No direct i2c/serial/radio/MicroBit refs — fully encapsulated by
    //    the device objects.
    // -----------------------------------------------------------------------
    static Robot robot(motorL, motorR, otos, line, color, gripper, portio, cfg);
    static CommandProcessor cmd(robot);

    // DEVICE: identification banner once at boot over serial.
    cmd.process("HELLO", serialReply, &comm.serial());

    // -----------------------------------------------------------------------
    // 7. Run the cooperative main loop — never returns.
    // -----------------------------------------------------------------------
    static LoopScheduler sched(robot, cmd, comm, uBit);
    cmd.setScheduler(&sched);             // enable DBG LOOP <x> <state> task toggling
    cmd.setI2CBus(&bus);                  // enable DBG I2C stats dump (015-003)

    // DebugCommandable — owns all DBG / I2CW / I2CR descriptors with
    // ForceReply::SERIAL.  Constructed here after sched and bus are live.
    // The legacy CommandProcessor switch cases remain active (cmd uses the
    // old constructor, _cmds == nullptr); this object is registered for
    // T011 cutover and the setSerialReply wiring is exercised now.
    static DbgCtx dbgCtx = { &sched, &bus, &robot };
    static DebugCommandable dbgCmd(dbgCtx);
    cmd.setSerialReply(serialReply, &comm.serial());

    // T010 dry run — build the full command table and log the descriptor count.
    // Confirms the table is well-formed before T011 cuts over to the new
    // constructor.  The static buffer must be ≥ the descriptor count (~42).
    {
        static CommandDescriptor cmdTable[60];
        int n = robot.buildCommandTable(cmdTable, 60, &dbgCmd, &sched);
        char countMsg[48];
        snprintf(countMsg, sizeof(countMsg), "DBG cmdtable n=%d", n);
        serialReply(countMsg, &comm.serial());
    }

    // Wire the I2CBus and EVT sink into MotorController so enc_wedged events
    // are emitted with bus stats and go to the active serial/radio channel.
    robot.motorController.setI2CBus(&bus);
    robot.motorController.setEvtSink(&sched.activeFn, &sched.activeCtx);

    sched.run_blocks();

    return 0;
}
